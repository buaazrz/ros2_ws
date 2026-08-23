#include "robot_hardware_interface/robot_interface.hpp"
#include "mujoco_ros2_control/mujoco_simulation.hpp"
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
        // 注意：不在这里初始化 dof_，也不定义新的 dof_，直接使用基类的 dof_
        MujocoRobot() = default;

        ~MujocoRobot()
        {
            cleanup_simulation();
        }

        CallbackReturn on_configure(const rclcpp_lifecycle::State& prev) override
        {
            if (RobotInterface::on_configure(prev) != CallbackReturn::SUCCESS) 
                return CallbackReturn::FAILURE;

            // 1. 解决节点类型不匹配问题：单独创建一个标准的 Node 给 MuJoCo
            rclcpp::NodeOptions sim_node_options;
            sim_node_options.context(node_->get_node_base_interface()->get_context());
            sim_node_ = std::make_shared<rclcpp::Node>(
                std::string(node_->get_name()) + "_mujoco",
                node_->get_namespace(),
                sim_node_options);

            // 读取参数
            std::string model_path = "";
            node_->get_parameter_or<std::string>("mujoco_model", model_path, "");
            if (model_path.empty()) {
                RCLCPP_ERROR(node_->get_logger(), "MujocoRobot: 'mujoco_model' is empty! Check your yaml.");
                return CallbackReturn::FAILURE;
            }

            // 2. 初始化 MuJoCo 仿真核心
            sim_ = std::make_unique<mujoco_ros2_control::MujocoSimulation>();
            if (!sim_->initialize(sim_node_, model_path, "/mujoco_robot_description", 1.0, false)) {
                RCLCPP_ERROR(node_->get_logger(), "MujocoRobot: Failed to initialize MuJoCo simulation");
                return CallbackReturn::FAILURE;
            }

            mjModel* m = sim_->model();

            // 3. 禁用内部 actuator，我们将直接写入 qfrc_applied
            m->opt.disableflags |= mjDSBL_ACTUATION;
            control_data_ = mj_makeData(m);

            // 4. 解析关节，直接使用基类的 joint_names_，而不是硬编码拼接名字
            qpos_addrs_.clear();
            qvel_addrs_.clear();
            for (const auto& joint_name : joint_names_)
            {
                const int joint_id = mj_name2id(m, mjOBJ_JOINT, joint_name.c_str());
                if (joint_id < 0) {
                    RCLCPP_ERROR(node_->get_logger(), "MuJoCo joint '%s' not found", joint_name.c_str());
                    return CallbackReturn::FAILURE;
                }

                const int joint_type = m->jnt_type[joint_id];
                if (joint_type != mjJNT_HINGE && joint_type != mjJNT_SLIDE) {
                    RCLCPP_ERROR(node_->get_logger(), "Joint '%s' is not a single-DOF joint", joint_name.c_str());
                    return CallbackReturn::FAILURE;
                }

                qpos_addrs_.push_back(m->jnt_qposadr[joint_id]);
                qvel_addrs_.push_back(m->jnt_dofadr[joint_id]);
            }

            // 5. 应用初始关键帧，防止机器人塌陷或违反限位
            std::string initial_keyframe;
            node_->get_parameter_or<std::string>("initial_keyframe", initial_keyframe, "home");
            if (!initial_keyframe.empty()) {
                // if (!sim_->apply_keyframe(initial_keyframe)) { // 注意：如果新版API没有此函数，请注释掉或手动查找keyid
                //     RCLCPP_WARN(node_->get_logger(), "Failed to apply keyframe '%s'", initial_keyframe.c_str());
                // }
                // 稳妥的手动设置关键帧方法：
                int key_id = mj_name2id(m, mjOBJ_KEY, initial_keyframe.c_str());
                if (key_id >= 0) {
                    mj_resetDataKeyframe(m, control_data_, key_id);
                    // 同步到 simulation 内部的 d_
                    mj_resetDataKeyframe(m, sim_->data(), key_id);
                } else {
                    RCLCPP_WARN(node_->get_logger(), "Keyframe '%s' not found in MJCF.", initial_keyframe.c_str());
                }
            }
            
            // sim_->capture_initial_state(); // 如果存在该API则保留，没有可删

            // 6. 初始化状态数组与安全限幅（适配 FR3）
            pre_dq_.assign(dof_, 0.0);
            last_tau_cmd_.assign(dof_, 0.0);
            // FR3 力矩安全限幅
            torque_limits_ = {87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0};
            // 单周期力矩最大突变限制
            delta_tau_max_.assign(dof_, 10.0); 

            // 7. 配置并启动 executor 线程处理 MuJoCo 的 ROS 回调
            sim_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
            sim_executor_->add_node(sim_node_);
            sim_spin_thread_ = std::thread([this]() {
                sim_executor_->spin();
            });

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
                previous_sim_time_ = -1.0;
                RCLCPP_INFO(node_->get_logger(), "MujocoRobot: Activated!");
                return CallbackReturn::SUCCESS;
            }
            return CallbackReturn::FAILURE;
        }

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State& prev) override
        {
            RobotInterface::on_deactivate(prev);
            // 只清零控制力，不停止物理线程，保证仿真平滑
            if(control_data_) {
                std::fill(control_data_->qfrc_applied, control_data_->qfrc_applied + sim_->model()->nv, 0.0);
                sim_->apply_control_data(control_data_);
            }
            RCLCPP_INFO(node_->get_logger(), "MujocoRobot: Deactivated (Zero torque applied).");
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

            // 安全获取引用，防止越界或崩溃
            auto& q = state_.get<double>("position");
            auto& dq = state_.get<double>("velocity");
            
            // 安全扩展 acceleration
            auto& ddq = state_.get<double>("acceleration");
            if (ddq.size() != static_cast<size_t>(dof_)) ddq.resize(dof_, 0.0);

            // 真正的 MuJoCo 仿真时间差分（解决异步时钟问题）
            const double sim_time = static_cast<double>(control_state_.time);
            const double sim_dt = (previous_sim_time_ >= 0.0) ? (sim_time - previous_sim_time_) : 0.0;

            for (int i = 0; i < dof_; ++i) 
            {
                const int qadr = qpos_addrs_[i];
                const int vadr = qvel_addrs_[i];

                q[i] = control_state_.qpos[qadr];
                const double new_dq = control_state_.qvel[vadr];

                if (sim_dt > 1e-9) {
                    ddq[i] = (new_dq - pre_dq_[i]) / sim_dt;
                } else {
                    ddq[i] = 0.0;
                }

                dq[i] = new_dq;
                pre_dq_[i] = new_dq;
            }

            if (sim_dt > 1e-9) {
                previous_sim_time_ = sim_time;
            }
        }

        void write(const rclcpp::Time& t, const rclcpp::Duration& period) override
        {
            RobotInterface::write(t, period);

            if (!sim_ || !control_data_) return;

            const auto& int_commands = command_.get<int>();
            auto mode_it = int_commands.find("mode");

            if (mode_it == int_commands.end() || mode_it->second.empty()) {
                RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "MujocoRobot: mode command is unavailable");
                return;
            }

            const int mode = mode_it->second[0];
            mjModel* model = sim_->model();

            // 每次写入前清空施加力
            std::fill(control_data_->qfrc_applied, control_data_->qfrc_applied + model->nv, 0.0);

            // Mode 3: 纯力矩控制 (Raw)
            if (mode == 3) 
            {
                const auto& double_commands = command_.get<double>();
                auto tau_it = double_commands.find("torque");

                if (tau_it == double_commands.end() || tau_it->second.size() != static_cast<size_t>(dof_)) {
                    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "MujocoRobot: invalid torque interface");
                    return;
                }

                const auto& tau_cmd = tau_it->second;

                for (int i = 0; i < dof_; ++i) 
                {
                    double requested_tau = tau_cmd[i];
                    if (!std::isfinite(requested_tau)) {
                        requested_tau = 0.0;
                    }

                    // 1. 绝对限幅
                    double safe_tau = std::clamp(requested_tau, -torque_limits_[i], torque_limits_[i]);
                    // 2. 变化率限幅 (防止瞬间尖峰)
                    safe_tau = std::clamp(safe_tau, last_tau_cmd_[i] - delta_tau_max_[i], last_tau_cmd_[i] + delta_tau_max_[i]);

                    control_data_->qfrc_applied[qvel_addrs_[i]] = safe_tau;
                    last_tau_cmd_[i] = safe_tau;
                }
            }

            // 发送给物理引擎
            sim_->apply_control_data(control_data_);
        }

        bool is_stop() override { return false; }

    private:
        void cleanup_simulation()
        {
            if (sim_) {
                sim_->shutdown();
            }
            if (control_data_) {
                mj_deleteData(control_data_);
                control_data_ = nullptr;
            }
            if (sim_executor_) {
                sim_executor_->cancel();
            }
            if (sim_spin_thread_.joinable()) {
                sim_spin_thread_.join();
            }
            sim_executor_.reset();
            sim_node_.reset();
            sim_.reset();
            physics_started_ = false;
        }

        // MuJoCo 组件
        std::unique_ptr<mujoco_ros2_control::MujocoSimulation> sim_;
        mujoco_ros2_control::MujocoSimulation::ControlState control_state_;
        mjData* control_data_{nullptr};

        // 独立 Node 与 线程管理
        rclcpp::Node::SharedPtr sim_node_;
        std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> sim_executor_;
        std::thread sim_spin_thread_;
        bool physics_started_{false};

        // 关节映射与数据
        std::vector<int> qpos_addrs_;
        std::vector<int> qvel_addrs_;
        std::vector<double> pre_dq_;
        double previous_sim_time_{-1.0};

        // 限幅保护
        std::vector<double> last_tau_cmd_;
        std::vector<double> torque_limits_;
        std::vector<double> delta_tau_max_;
    };
} // namespace hardwares

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(hardwares::MujocoRobot, hardware_interface::RobotInterface)