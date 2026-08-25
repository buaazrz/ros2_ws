#include "robot_hardware_interface/robot_interface.hpp"
#include "mujoco_ros2_control/mujoco_simulation.hpp"
#include "robot_math/robot_math.hpp" // 用于计算正运动学
#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <vector>
#include <algorithm>
#include <memory>
#include <string>
#include <thread>
#include <cmath>
#include <limits>

using namespace std::chrono_literals;

namespace hardwares
{
    class MujocoRobot : public hardware_interface::RobotInterface
    {
    public:
        MujocoRobot() = default;

        ~MujocoRobot() { cleanup_simulation(); }

        CallbackReturn on_configure(const rclcpp_lifecycle::State& prev) override
        {
            if (RobotInterface::on_configure(prev) != CallbackReturn::SUCCESS) 
                return CallbackReturn::FAILURE;


            rclcpp::NodeOptions sim_node_options;
            sim_node_options.context(node_->get_node_base_interface()->get_context());
            sim_node_ = std::make_shared<rclcpp::Node>(
                std::string(node_->get_name()) + "_mujoco",
                node_->get_namespace(),
                sim_node_options);

            // 1. 读取 YAML 参数
            std::string model_path, initial_keyframe;
            bool headless;
            double speed_factor;
            node_->get_parameter_or<std::string>("mujoco_model", model_path, "");
            node_->get_parameter_or<bool>("mujoco_headless", headless, false);
            node_->get_parameter_or<double>("mujoco_speed_factor", speed_factor, 1.0);
            node_->get_parameter_or<std::string>("mujoco_initial_keyframe", initial_keyframe, "home");

            if (model_path.empty()) {
                RCLCPP_ERROR(node_->get_logger(), "MujocoRobot: 'mujoco_model' is empty!");
                return CallbackReturn::FAILURE;
            }

            // 2. 初始化 MuJoCo
            sim_ = std::make_unique<mujoco_ros2_control::MujocoSimulation>();
            if (!sim_->initialize(sim_node_, model_path, "/mujoco_robot_description", speed_factor, headless)) {
                return CallbackReturn::FAILURE;
            }

            mjModel* m = sim_->model();

            if (!synchronize_payload_from_mujoco(m)) {
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "MujocoRobot: failed to synchronize payload dynamics");
                return CallbackReturn::FAILURE;
            }

            m->opt.disableflags |= mjDSBL_ACTUATION;
            control_data_ = mj_makeData(m);

            std::string force_sensor_name;
            std::string torque_sensor_name;

            node_->get_parameter_or<std::string>(
                "mujoco_force_sensor",
                force_sensor_name,
                "tool_force");

            node_->get_parameter_or<std::string>(
                "mujoco_torque_sensor",
                torque_sensor_name,
                "tool_torque");

            node_->get_parameter_or<std::string>(
                "mujoco_ft_component",
                ft_component_name_,
                "ft_sensor");

            if (components_.find(ft_component_name_) == components_.end())
            {
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "MujocoRobot: component '%s' is not configured",
                    ft_component_name_.c_str());

                return CallbackReturn::FAILURE;
            }

            auto resolve_sensor =
                [this, m](
                    const std::string & sensor_name,
                    int expected_type,
                    int & sensor_adr) -> bool
            {
                const int sensor_id = mj_name2id(
                    m,
                    mjOBJ_SENSOR,
                    sensor_name.c_str());

                if (sensor_id < 0)
                {
                    RCLCPP_ERROR(
                        node_->get_logger(),
                        "MujocoRobot: MuJoCo sensor '%s' does not exist",
                        sensor_name.c_str());

                    return false;
                }

                if (m->sensor_dim[sensor_id] != 3)
                {
                    RCLCPP_ERROR(
                        node_->get_logger(),
                        "MujocoRobot: sensor '%s' must have dimension 3, got %d",
                        sensor_name.c_str(),
                        m->sensor_dim[sensor_id]);

                    return false;
                }

                if (m->sensor_type[sensor_id] != expected_type)
                {
                    RCLCPP_ERROR(
                        node_->get_logger(),
                        "MujocoRobot: sensor '%s' has unexpected MuJoCo sensor type",
                        sensor_name.c_str());

                    return false;
                }

                sensor_adr = m->sensor_adr[sensor_id];

                if (sensor_adr < 0 || sensor_adr + 3 > m->nsensordata)
                {
                    RCLCPP_ERROR(
                        node_->get_logger(),
                        "MujocoRobot: invalid sensordata address for '%s': adr=%d",
                        sensor_name.c_str(),
                        sensor_adr);

                    return false;
                }

                RCLCPP_INFO(
                    node_->get_logger(),
                    "MujocoRobot: sensor '%s': id=%d, adr=%d, dim=3",
                    sensor_name.c_str(),
                    sensor_id,
                    sensor_adr);

                return true;
            };

            if (!resolve_sensor(
                    force_sensor_name,
                    mjSENS_FORCE,
                    force_sensor_adr_))
            {
                return CallbackReturn::FAILURE;
            }

            if (!resolve_sensor(
                    torque_sensor_name,
                    mjSENS_TORQUE,
                    torque_sensor_adr_))
            {
                return CallbackReturn::FAILURE;
            }

            required_sensordata_size_ =
                static_cast<std::size_t>(
                    std::max(
                        force_sensor_adr_ + 3,
                        torque_sensor_adr_ + 3));

            // 3. 解析关节地址
            qpos_addrs_.clear();
            qvel_addrs_.clear();
            for (const auto& joint_name : joint_names_)
            {
                const int joint_id = mj_name2id(m, mjOBJ_JOINT, joint_name.c_str());
                if (joint_id < 0) {
                    RCLCPP_ERROR(node_->get_logger(), "MuJoCo joint '%s' not found", joint_name.c_str());
                    return CallbackReturn::FAILURE;
                }
                qpos_addrs_.push_back(m->jnt_qposadr[joint_id]);
                qvel_addrs_.push_back(m->jnt_dofadr[joint_id]);
            }

            // FR3 力矩限制以及 MuJoCo 无控制器期间的安全保持增益。
            torque_limits_ = {87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0};
            hold_kp_ = {80.0, 80.0, 80.0, 80.0, 20.0, 20.0, 10.0};
            hold_kd_ = {12.0, 12.0, 12.0, 12.0, 4.0, 4.0, 2.0};

            node_->get_parameter_or<std::vector<double>>(
                "mujoco_hold_kp",
                hold_kp_,
                hold_kp_);

            node_->get_parameter_or<std::vector<double>>(
                "mujoco_hold_kd",
                hold_kd_,
                hold_kd_);

            if (torque_limits_.size() != static_cast<std::size_t>(dof_) ||
                hold_kp_.size() != static_cast<std::size_t>(dof_) ||
                hold_kd_.size() != static_cast<std::size_t>(dof_))
            {
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "MujocoRobot: torque limits and hold gains must have %d entries",
                    dof_);
                return CallbackReturn::FAILURE;
            }

            pre_dq_.assign(dof_, 0.0);
            last_tau_cmd_.assign(dof_, 0.0);
            hold_q_.assign(dof_, 0.0);

            // 4. 配置并同步初始关键帧
            if (!initial_keyframe.empty())
            {
                const int key_id = mj_name2id(
                    m,
                    mjOBJ_KEY,
                    initial_keyframe.c_str());

                if (key_id < 0)
                {
                    RCLCPP_ERROR(
                        node_->get_logger(),
                        "MujocoRobot: startup keyframe '%s' not found",
                        initial_keyframe.c_str());

                    return CallbackReturn::FAILURE;
                }

                // 告诉物理线程：Load 后再次应用该关键帧
                sim_->set_startup_keyframe(initial_keyframe);

                // 初始化硬件侧 control_data_
                mj_resetDataKeyframe(
                    m,
                    control_data_,
                    key_id);

                mj_forward(
                    m,
                    control_data_);

                auto& q_state = state_.get<double>("position");
                auto& dq_state = state_.get<double>("velocity");
                auto& ddq_state = state_.get<double>("acceleration");
                auto& tau_state = state_.get<double>("torque");

                std::fill(
                    control_data_->qfrc_applied,
                    control_data_->qfrc_applied + m->nv,
                    0.0);

                for (int i = 0; i < dof_; ++i)
                {
                    const int dof_adr = qvel_addrs_[i];

                    q_state[i] =
                        control_data_->qpos[qpos_addrs_[i]];

                    dq_state[i] =
                        control_data_->qvel[dof_adr];

                    ddq_state[i] = 0.0;
                    pre_dq_[i] = dq_state[i];
                    hold_q_[i] = q_state[i];

                    // 静止平衡：
                    // qfrc_applied + qfrc_passive = qfrc_bias
                    const double tau_hold =
                        control_data_->qfrc_bias[dof_adr] -
                        control_data_->qfrc_passive[dof_adr];

                    control_data_->qfrc_applied[dof_adr] = tau_hold;
                    last_tau_cmd_[i] = tau_hold;
                    tau_state[i] = tau_hold;
                }

                // 物理线程尚未启动也可以预先提交，staging 会保留
                stage_control_data();

                Eigen::Matrix4d T_init;
                robot_math::forward_kinematics(
                    &robot_,
                    q_state,
                    T_init);

                auto& T_state = state_.get<double>("T");
                T_state.assign(
                    T_init.data(),
                    T_init.data() + 16);

                auto& pose_state = state_.get<double>("pose");
                pose_state =
                    robot_math::tform_to_pose(T_init);

                RCLCPP_INFO(
                    node_->get_logger(),
                    "MujocoRobot: configured startup keyframe '%s'",
                    initial_keyframe.c_str());
            }

            last_mode_ = 0;

            // 启动 executor 线程
            sim_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
            sim_executor_->add_node(sim_node_);
            sim_spin_thread_ = std::thread([this]() { sim_executor_->spin(); });



            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_activate(const rclcpp_lifecycle::State& prev) override
        {
            if (RobotInterface::on_activate(prev) == CallbackReturn::SUCCESS)
            {
                /*
                 * 在启动物理线程以前就提交安全保持力矩。
                 * 这样第一次 mj_step() 不会经历零力矩自由下坠。
                 */
                auto& modes = command_.get<int>("mode");
                if (!modes.empty())
                {
                    modes[0] = 0;
                }

                capture_hold_reference();
                stage_hold_command();

                if (!physics_started_)
                {
                    sim_->start_physics_thread();
                    physics_started_ = true;
                }

                const auto& dq_state = state_.get<double>("velocity");
                if (dq_state.size() == static_cast<std::size_t>(dof_))
                {
                    pre_dq_.assign(dq_state.begin(), dq_state.end());
                }
                else
                {
                    std::fill(pre_dq_.begin(), pre_dq_.end(), 0.0);
                }
                previous_sim_time_ = -1.0;
                RCLCPP_INFO(node_->get_logger(), "MujocoRobot: Activated!");
                return CallbackReturn::SUCCESS;
            }
            return CallbackReturn::FAILURE;
        }

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State& prev) override
        {
            /*
             * MujocoRobot 停用时也不能向仍在运行的物理线程提交零力矩。
             * 最终 shutdown 会直接停止仿真线程；普通 deactivate 只进入保持。
             */
            if (sim_ && control_data_)
            {
                auto& modes = command_.get<int>("mode");
                if (!modes.empty())
                {
                    modes[0] = 0;
                }

                capture_hold_reference();
                stage_hold_command();
            }

            return RobotInterface::on_deactivate(prev);
        }

        CallbackReturn on_shutdown(const rclcpp_lifecycle::State& prev) override
        {
            RobotInterface::on_shutdown(prev);
            cleanup_simulation();
            return CallbackReturn::SUCCESS;
        }

        void read(const rclcpp::Time& t, const rclcpp::Duration& period) override
        {
            RobotInterface::read(t, period);
            if (!sim_ || !control_data_) return;
            sim_->copy_control_state(control_state_);

            auto& q = state_.get<double>("position");
            auto& dq = state_.get<double>("velocity");
            auto& ddq = state_.get<double>("acceleration");
            auto& tau_state = state_.get<double>("torque");

            auto ft_component_it = components_.find(ft_component_name_);

            Eigen::VectorXd tau_actuator(dof_);

            for (int i = 0; i < dof_; ++i)
            {
                tau_actuator[i] =
                    control_state_.qfrc_actuator[qvel_addrs_[i]];
            }

            // RCLCPP_INFO_STREAM_THROTTLE(
            //     node_->get_logger(),
            //     *node_->get_clock(),
            //     100,
            //     "qfrc_actuator = ["
            //         << tau_actuator.transpose()
            //         << "]");

            if (ft_component_it == components_.end())
            {
                RCLCPP_ERROR_THROTTLE(
                    node_->get_logger(),
                    *node_->get_clock(),
                    2000,
                    "MujocoRobot: FT component '%s' disappeared",
                    ft_component_name_.c_str());

                return;
            }

            if (control_state_.sensordata.size() < required_sensordata_size_)
            {
                std::fill(ft_wrench_.begin(), ft_wrench_.end(), 0.0);

                ft_component_it->second->write_state(
                    "force",
                    ft_wrench_);

                RCLCPP_WARN_THROTTLE(
                    node_->get_logger(),
                    *node_->get_clock(),
                    2000,
                    "MujocoRobot: sensordata size is %zu, expected at least %zu",
                    control_state_.sensordata.size(),
                    required_sensordata_size_);

                return;
            }

            // tool_force
            ft_wrench_[0] =
                control_state_.sensordata[force_sensor_adr_ + 0];
            ft_wrench_[1] =
                control_state_.sensordata[force_sensor_adr_ + 1];
            ft_wrench_[2] =
                control_state_.sensordata[force_sensor_adr_ + 2];

            // tool_torque
            ft_wrench_[3] =
                control_state_.sensordata[torque_sensor_adr_ + 0];
            ft_wrench_[4] =
                control_state_.sensordata[torque_sensor_adr_ + 1];
            ft_wrench_[5] =
                control_state_.sensordata[torque_sensor_adr_ + 2];

            ft_component_it->second->write_state(
                "force",
                ft_wrench_);

            if (ddq.size() != static_cast<size_t>(dof_)) ddq.resize(dof_, 0.0);

            // 1. 修复的物理时间差分逻辑
            const double sim_time = static_cast<double>(control_state_.time);
            double sim_dt = 0.0;
            if (previous_sim_time_ >= 0.0 && sim_time > previous_sim_time_) {
                sim_dt = sim_time - previous_sim_time_;
            }

            for (int i = 0; i < dof_; ++i) 
            {
                const double new_dq = control_state_.qvel[qvel_addrs_[i]];
                ddq[i] = (sim_dt > 1e-9) ? (new_dq - pre_dq_[i]) / sim_dt : 0.0;
                
                q[i] = control_state_.qpos[qpos_addrs_[i]];
                dq[i] = new_dq;
                pre_dq_[i] = new_dq;
                tau_state[i] = last_tau_cmd_[i]; // 填充历史扭矩
            }
            previous_sim_time_ = sim_time;
            // 2. 核心：计算控制器急需的正运动学矩阵 T 和 pose
            // (因为在 YAML 中已经声明了 T 和 pose，因此直接获取即可)
            Eigen::Matrix4d T;
            robot_math::forward_kinematics(&robot_, q, T);  // <--- 注意这里改成了 robot_
            
            auto& T_state = state_.get<double>("T");
            T_state.assign(T.data(), T.data() + 16);
            
            std::vector<double> pose_vec = robot_math::tform_to_pose(T);
            auto& pose_state = state_.get<double>("pose");
            pose_state.assign(pose_vec.begin(), pose_vec.end());
        }

        void write(const rclcpp::Time& t, const rclcpp::Duration& period) override
        {
            RobotInterface::write(t, period);
            if (!sim_ || !control_data_) return;

            const auto& int_commands = command_.get<int>();
            auto mode_it = int_commands.find("mode");
            int mode = 0;

            if (mode_it != int_commands.end() && !mode_it->second.empty())
            {
                mode = mode_it->second[0];
            }
            else
            {
                RCLCPP_WARN_THROTTLE(
                    node_->get_logger(),
                    *node_->get_clock(),
                    2000,
                    "MujocoRobot: missing mode command; using safe hold");
            }

            if (mode == 3) 
            {
                const auto& double_commands = command_.get<double>();
                auto tau_it = double_commands.find("torque");

                bool valid_torque_command =
                    tau_it != double_commands.end() &&
                    tau_it->second.size() == static_cast<std::size_t>(dof_);

                if (valid_torque_command)
                {
                    const auto& tau_cmd = tau_it->second;

                    valid_torque_command = std::all_of(
                        tau_cmd.begin(),
                        tau_cmd.end(),
                        [](double value)
                        {
                            return std::isfinite(value);
                        });
                }

                if (valid_torque_command)
                {
                    mjModel* model = sim_->model();
                    std::fill(
                        control_data_->qfrc_applied,
                        control_data_->qfrc_applied + model->nv,
                        0.0);

                    const auto& tau_cmd = tau_it->second;

                    for (int i = 0; i < dof_; ++i)
                    {
                        const double tau = std::clamp(
                            tau_cmd[i],
                            -torque_limits_[i],
                            torque_limits_[i]);

                        control_data_->qfrc_applied[qvel_addrs_[i]] = tau;
                        last_tau_cmd_[i] = tau;
                    }

                    last_mode_ = 3;
                    stage_control_data();
                    return;
                }

                RCLCPP_ERROR_THROTTLE(
                    node_->get_logger(),
                    *node_->get_clock(),
                    2000,
                    "MujocoRobot: invalid torque command; falling back to safe hold");
            }
            else if (mode != 0)
            {
                RCLCPP_WARN_THROTTLE(
                    node_->get_logger(),
                    *node_->get_clock(),
                    2000,
                    "MujocoRobot: unsupported mode %d; using safe hold",
                    mode);
            }

            // 从控制器模式进入保持模式时，锁定切换瞬间的关节位置。
            if (last_mode_ != 0)
            {
                capture_hold_reference();
            }

            stage_hold_command();
        }

        bool is_stop() override { return false; }

        void stage_control_data()
        {
            mjModel* model = sim_->model();

            // 当前硬件只进行力矩控制，不请求直接覆盖物理速度。
            std::fill(
                control_data_->qvel,
                control_data_->qvel + model->nv,
                std::numeric_limits<mjtNum>::quiet_NaN());

            sim_->apply_control_data(control_data_);
        }

    private:
        void capture_hold_reference()
        {
            const auto& q = state_.get<double>("position");

            if (q.size() != static_cast<std::size_t>(dof_))
            {
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "MujocoRobot: cannot capture hold reference; "
                    "position state has %zu entries, expected %d",
                    q.size(),
                    dof_);
                return;
            }

            hold_q_.assign(q.begin(), q.end());
        }

        void stage_hold_command()
        {
            if (!sim_ || !control_data_)
            {
                return;
            }

            const auto& q = state_.get<double>("position");
            const auto& dq = state_.get<double>("velocity");

            if (q.size() != static_cast<std::size_t>(dof_) ||
                dq.size() != static_cast<std::size_t>(dof_) ||
                hold_q_.size() != static_cast<std::size_t>(dof_))
            {
                RCLCPP_ERROR_THROTTLE(
                    node_->get_logger(),
                    *node_->get_clock(),
                    2000,
                    "MujocoRobot: invalid state size for safe hold");
                return;
            }

            mjModel* model = sim_->model();

            /*
             * 用最新物理状态更新专用 control_data_，仅用于计算
             * qfrc_bias 和 qfrc_passive。apply_control_data() 不会提交 qpos；
             * stage_control_data() 还会把 qvel 改成 NaN，避免速度硬覆盖。
             */
            for (int i = 0; i < dof_; ++i)
            {
                control_data_->qpos[qpos_addrs_[i]] = q[i];
                control_data_->qvel[qvel_addrs_[i]] =
                    std::isfinite(dq[i]) ? dq[i] : 0.0;
            }

            mj_forward(model, control_data_);

            std::fill(
                control_data_->qfrc_applied,
                control_data_->qfrc_applied + model->nv,
                0.0);

            for (int i = 0; i < dof_; ++i)
            {
                const int dof_adr = qvel_addrs_[i];
                const double velocity =
                    std::isfinite(dq[i]) ? dq[i] : 0.0;

                // 静态平衡 + 小幅关节位置/速度反馈。
                const double tau_gravity =
                    control_data_->qfrc_bias[dof_adr] -
                    control_data_->qfrc_passive[dof_adr];

                const double tau_feedback =
                    hold_kp_[i] * (hold_q_[i] - q[i]) -
                    hold_kd_[i] * velocity;

                const double tau_hold = std::clamp(
                    tau_gravity + tau_feedback,
                    -torque_limits_[i],
                    torque_limits_[i]);

                control_data_->qfrc_applied[dof_adr] = tau_hold;
                last_tau_cmd_[i] = tau_hold;
            }

            last_mode_ = 0;
            stage_control_data();
        }

        void cleanup_simulation()
        {
            if (sim_executor_) sim_executor_->cancel();
            if (sim_spin_thread_.joinable()) sim_spin_thread_.join();
            if (sim_) sim_->shutdown();
            if (control_data_) { mj_deleteData(control_data_); control_data_ = nullptr; }
            sim_executor_.reset();
            sim_node_.reset();
            sim_.reset();
            physics_started_ = false;
        }

        static Eigen::Matrix3d mujoco_quat_to_rotation(const mjtNum* quat)
        {
            mjtNum raw_rotation[9];
            mju_quat2Mat(raw_rotation, quat);

            Eigen::Matrix3d rotation;
            for (int row = 0; row < 3; ++row) {
                for (int col = 0; col < 3; ++col) {
                    rotation(row, col) =
                        static_cast<double>(raw_rotation[3 * row + col]);
                }
            }

            return rotation;
        }

        bool synchronize_payload_from_mujoco(const mjModel* model)
        {
            bool enabled = false;
            node_->get_parameter_or<bool>(
                "mujoco_sync_payload_to_controller",
                enabled,
                false);

            if (!enabled) {
                RCLCPP_INFO(
                    node_->get_logger(),
                    "MujocoRobot: controller payload synchronization disabled");
                return true;
            }

            std::string payload_body_name;
            std::string payload_parent_body_name;
            std::string target_joint_name;

            node_->get_parameter_or<std::string>(
                "mujoco_payload_body",
                payload_body_name,
                "tool_link");

            node_->get_parameter_or<std::string>(
                "mujoco_payload_parent_body",
                payload_parent_body_name,
                "fr3_link7");

            node_->get_parameter_or<std::string>(
                "controller_payload_target_joint",
                target_joint_name,
                "fr3_joint7");

            const int payload_body_id = mj_name2id(
                model,
                mjOBJ_BODY,
                payload_body_name.c_str());

            if (payload_body_id < 0) {
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "MujocoRobot: payload body '%s' not found",
                    payload_body_name.c_str());
                return false;
            }

            const int payload_parent_body_id = mj_name2id(
                model,
                mjOBJ_BODY,
                payload_parent_body_name.c_str());

            if (payload_parent_body_id < 0) {
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "MujocoRobot: payload parent body '%s' not found",
                    payload_parent_body_name.c_str());
                return false;
            }

            if (model->body_parentid[payload_body_id] != payload_parent_body_id) {
                const int actual_parent_id =
                    model->body_parentid[payload_body_id];

                const char* actual_parent_name =
                    mj_id2name(model, mjOBJ_BODY, actual_parent_id);

                RCLCPP_ERROR(
                    node_->get_logger(),
                    "MujocoRobot: payload '%s' is not a direct child of '%s'; "
                    "actual parent is '%s'",
                    payload_body_name.c_str(),
                    payload_parent_body_name.c_str(),
                    actual_parent_name ? actual_parent_name : "<unnamed>");

                return false;
            }

            // tool_link 不能包含独立关节，否则不能直接合并到 link7。
            if (model->body_jntnum[payload_body_id] != 0) {
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "MujocoRobot: payload body '%s' contains %d joints; "
                    "only fixed payload bodies are supported",
                    payload_body_name.c_str(),
                    model->body_jntnum[payload_body_id]);
                return false;
            }

            const auto joint_it = std::find(
                joint_names_.begin(),
                joint_names_.end(),
                target_joint_name);

            if (joint_it == joint_names_.end()) {
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "MujocoRobot: controller target joint '%s' not found",
                    target_joint_name.c_str());
                return false;
            }

            const std::size_t target_index =
                static_cast<std::size_t>(
                    std::distance(joint_names_.begin(), joint_it));

            if (target_index >= robot_.mass.size() ||
                target_index >= robot_.com.size() ||
                target_index >= robot_.inertia.size())
            {
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "MujocoRobot: invalid robot_math target index %zu",
                    target_index);
                return false;
            }

            const mjtNum* body_pos =
                model->body_pos + 3 * payload_body_id;

            const mjtNum* body_quat =
                model->body_quat + 4 * payload_body_id;

            const mjtNum* inertial_pos =
                model->body_ipos + 3 * payload_body_id;

            const mjtNum* inertial_quat =
                model->body_iquat + 4 * payload_body_id;

            const mjtNum* diagonal_inertia =
                model->body_inertia + 3 * payload_body_id;

            const double payload_mass =
                static_cast<double>(
                    model->body_mass[payload_body_id]);

            if (!std::isfinite(payload_mass) || payload_mass <= 0.0) {
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "MujocoRobot: payload '%s' has invalid mass %.9f",
                    payload_body_name.c_str(),
                    payload_mass);
                return false;
            }

            const Eigen::Vector3d parent_to_body(
                static_cast<double>(body_pos[0]),
                static_cast<double>(body_pos[1]),
                static_cast<double>(body_pos[2]));

            const Eigen::Vector3d body_to_com(
                static_cast<double>(inertial_pos[0]),
                static_cast<double>(inertial_pos[1]),
                static_cast<double>(inertial_pos[2]));

            const Eigen::Matrix3d R_parent_body =
                mujoco_quat_to_rotation(body_quat);

            const Eigen::Matrix3d R_body_inertial =
                mujoco_quat_to_rotation(inertial_quat);

            // 负载质心在父 body（fr3_link7）中的位置。
            const Eigen::Vector3d payload_com_parent =
                parent_to_body + R_parent_body * body_to_com;

            const Eigen::Matrix3d payload_inertia_com =
                Eigen::Vector3d(
                    static_cast<double>(diagonal_inertia[0]),
                    static_cast<double>(diagonal_inertia[1]),
                    static_cast<double>(diagonal_inertia[2]))
                    .asDiagonal();

            const Eigen::Matrix3d R_parent_inertial =
                R_parent_body * R_body_inertial;

            // 质心惯量先旋转到 fr3_link7 坐标系。
            const Eigen::Matrix3d payload_inertia_com_parent =
                R_parent_inertial *
                payload_inertia_com *
                R_parent_inertial.transpose();

            // 平行轴定理：从负载质心移动到 fr3_link7 原点。
            const Eigen::Matrix3d payload_inertia_parent_origin =
                payload_inertia_com_parent +
                payload_mass *
                (
                    payload_com_parent.squaredNorm() *
                        Eigen::Matrix3d::Identity()
                    -
                    payload_com_parent *
                        payload_com_parent.transpose()
                );

            const double original_mass =
                robot_.mass[target_index];

            Eigen::Map<Eigen::Vector3d> original_com(
                robot_.com[target_index].data());

            Eigen::Map<Eigen::Matrix3d> original_inertia(
                robot_.inertia[target_index].data());

            const Eigen::Vector3d original_com_copy =
                original_com;

            const double combined_mass =
                original_mass + payload_mass;

            if (combined_mass <= 0.0) {
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "MujocoRobot: invalid combined payload mass");
                return false;
            }

            // robot_math 中的 inertia 是相对于当前 link 原点的惯量，
            // 所以可直接把同一原点下的负载惯量加进去。
            original_inertia += payload_inertia_parent_origin;

            original_com =
                (
                    original_mass * original_com_copy +
                    payload_mass * payload_com_parent
                ) / combined_mass;

            robot_.mass[target_index] =
                combined_mass;

            RCLCPP_INFO(
                node_->get_logger(),
                "MujocoRobot: synchronized payload '%s' to joint '%s': "
                "payload_mass=%.6f kg, payload_com=[%.6f, %.6f, %.6f] m, "
                "combined_mass=%.6f kg",
                payload_body_name.c_str(),
                target_joint_name.c_str(),
                payload_mass,
                payload_com_parent.x(),
                payload_com_parent.y(),
                payload_com_parent.z(),
                combined_mass);

            RCLCPP_INFO(
                node_->get_logger(),
                "MujocoRobot: payload inertia about target origin diag="
                "[%.9f, %.9f, %.9f] kg*m^2",
                payload_inertia_parent_origin(0, 0),
                payload_inertia_parent_origin(1, 1),
                payload_inertia_parent_origin(2, 2));

            return true;
        }

        std::unique_ptr<mujoco_ros2_control::MujocoSimulation> sim_;
        mujoco_ros2_control::MujocoSimulation::ControlState control_state_;
        mjData* control_data_{nullptr};
        rclcpp::Node::SharedPtr sim_node_;
        std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> sim_executor_;
        std::thread sim_spin_thread_;
        bool physics_started_{false};

        std::vector<int> qpos_addrs_;
        std::vector<int> qvel_addrs_;
        std::vector<double> pre_dq_;
        double previous_sim_time_{-1.0};
        std::vector<double> last_tau_cmd_;
        std::vector<double> torque_limits_;
        std::vector<double> hold_q_;
        std::vector<double> hold_kp_;
        std::vector<double> hold_kd_;
        int last_mode_{0};

        std::string ft_component_name_{"ft_sensor"};

        int force_sensor_adr_{-1};
        int torque_sensor_adr_{-1};
        std::size_t required_sensordata_size_{0};

        std::vector<double> ft_wrench_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    };
} 

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(hardwares::MujocoRobot, hardware_interface::RobotInterface)
