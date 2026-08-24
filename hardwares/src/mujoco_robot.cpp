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

            // 4. 应用初始关键帧
            if (!initial_keyframe.empty()) {
                int key_id = mj_name2id(m, mjOBJ_KEY, initial_keyframe.c_str());
                if (key_id >= 0) {
                    mj_resetDataKeyframe(m, control_data_, key_id);
                    mj_resetDataKeyframe(m, sim_->data(), key_id);
                } else {
                    RCLCPP_WARN(node_->get_logger(), "Keyframe '%s' not found.", initial_keyframe.c_str());
                }
            }
            
            // 初始化状态与限幅 (针对 FR3)
            pre_dq_.assign(dof_, 0.0);
            last_tau_cmd_.assign(dof_, 0.0);
            torque_limits_ = {87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0};

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
                if (!physics_started_) {
                    sim_->start_physics_thread();
                    physics_started_ = true;
                }
                std::fill(pre_dq_.begin(), pre_dq_.end(), 0.0);
                std::fill(last_tau_cmd_.begin(), last_tau_cmd_.end(), 0.0);
                previous_sim_time_ = -1.0;
                RCLCPP_INFO(node_->get_logger(), "MujocoRobot: Activated!");
                return CallbackReturn::SUCCESS;
            }
            return CallbackReturn::FAILURE;
        }

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State& prev) override
        {
            RobotInterface::on_deactivate(prev);
            if(control_data_) {
                std::fill(control_data_->qfrc_applied, control_data_->qfrc_applied + sim_->model()->nv, 0.0);
                sim_->apply_control_data(control_data_);
            }
            return CallbackReturn::SUCCESS;
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
            if (mode_it == int_commands.end() || mode_it->second.empty()) return;

            const int mode = mode_it->second[0];
            mjModel* model = sim_->model();
            std::fill(control_data_->qfrc_applied, control_data_->qfrc_applied + model->nv, 0.0);

            if (mode == 3) 
            {
                const auto& double_commands = command_.get<double>();
                auto tau_it = double_commands.find("torque");
                if (tau_it != double_commands.end() && tau_it->second.size() == static_cast<size_t>(dof_)) 
                {
                    const auto& tau_cmd = tau_it->second;
                    for (int i = 0; i < dof_; ++i) 
                    {
                        double tau = std::isfinite(tau_cmd[i]) ? tau_cmd[i] : 0.0;
                        // 安全限幅
                        tau = std::clamp(tau, -torque_limits_[i], torque_limits_[i]);
                        control_data_->qfrc_applied[qvel_addrs_[i]] = tau;
                        last_tau_cmd_[i] = tau;
                    }
                }
            } 
            sim_->apply_control_data(control_data_);
        }

        bool is_stop() override { return false; }

    private:
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

        std::string ft_component_name_{"ft_sensor"};

        int force_sensor_adr_{-1};
        int torque_sensor_adr_{-1};
        std::size_t required_sensordata_size_{0};

        std::vector<double> ft_wrench_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    };
} 

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(hardwares::MujocoRobot, hardware_interface::RobotInterface)