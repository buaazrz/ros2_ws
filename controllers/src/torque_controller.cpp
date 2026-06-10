// #include "robot_controller_interface/controller_interface.hpp"
// #include "robot_math/robot_math.hpp"
// #include <iostream>
// namespace controllers
// {
//     class TorqueController : public controller_interface::ControllerInterface
//     {
//     public:
//         TorqueController()
//         {
//         }
//         void update(const rclcpp::Time & /*t*/, const rclcpp::Duration & /*period*/) override
//         {
//             auto &cmd_torque = command_->get<double>("torque");
//             std::fill(cmd_torque.begin(), cmd_torque.end(), 0);
//             // auto &q = state_->get<double>("position");
//             auto &dq = state_->get<double>("velocity");
//             int n = robot_->dof;
//             for(int i = 0; i < n; i++)
//                 cmd_torque[i] = -dq[i];
//             // Eigen::MatrixXd M, C, Jb, dJb, dM;
//             // Eigen::VectorXd g;
//             // Eigen::Matrix4d Tb, dTb;
//             // std::vector<double> cmd(n);
//             // m_c_g_matrix(robot_, q, dq, M, C, g, Jb, dJb, dM, dTb, Tb);
//             //std::copy(g.data(), g.data() + n, cmd_torque.begin());
//         }
//     };
// } // namespace controllers

// #include <pluginlib/class_list_macros.hpp>

// PLUGINLIB_EXPORT_CLASS(controllers::TorqueController, controller_interface::ControllerInterface)

#include "robot_controller_interface/controller_interface.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "robot_math/robot_math.hpp"
#include "ros2_utility/data_logger.hpp"
#include <iostream>
#include <iomanip>
#include <vector>
#include <numeric>

namespace controllers
{
    class TorqueController : public controller_interface::ControllerInterface
    {
    public:
        TorqueController() {}
        ~TorqueController()
        {
            if (data_logger_)
                data_logger_->save("/home/wjc/experiment_logs/friction_test_controller/", "friction_test_data");
        }

        CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            dof_ = robot_->dof;

            // ===== 核心测试参数配置 =====
            node_->get_parameter_or<int>("test_joint_idx", test_joint_idx_, 6);    // 默认测试1关节(索引0)
            node_->get_parameter_or<double>("ramp_rate", ramp_rate_, 0.5);         // 斜坡上升速率 0.5 Nm/s
            node_->get_parameter_or<double>("cooldown_rate", cooldown_rate_, 3.0); // 刹车时的下降速率 3.0 Nm/s
            node_->get_parameter_or<double>("max_torque", max_torque_, 15.0);      // 最大安全力矩 Nm
            // node_->get_parameter_or<double>("vel_thresh", vel_thresh_, 0.001);         // 判定的速度阈值 rad/s
            node_->get_parameter_or<double>("disp_thresh", disp_thresh_, 5e-4); // 判定的位移阈值 rad
            node_->get_parameter_or<int>("iterations", total_iterations_, 5);   // 循环测试次数，默认 5 次

            RCLCPP_INFO(node_->get_logger(), "配置摩擦力测试控制器: 测试关节 [%d], 循环次数 [%d]", test_joint_idx_, total_iterations_);
            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            time_ = 0;
            test_state_ = 0;
            state_timer_ = 0.0;
            current_test_torque_ = 0.0;
            breakaway_torque_pos_ = 0.0;
            breakaway_torque_neg_ = 0.0;
            test_finished_ = false;

            // 重置循环变量
            current_iteration_ = 0;
            pos_breakaways_.clear();
            neg_breakaways_.clear();

            const std::vector<double> &q_vec = state_->get<double>("position");
            q_ = Eigen::Map<const Eigen::VectorXd>(q_vec.data(), dof_);
            dq_ = Eigen::VectorXd::Zero(dof_);
            tau_cmd_ = Eigen::VectorXd::Zero(dof_);

            data_logger_ = std::make_unique<DataLogger>(
                std::initializer_list<DataInfo>{
                    DATA_WRAPPER(time_),
                    DATA_WRAPPER(q_),
                    // DATA_WRAPPER(dq_),
                    DATA_WRAPPER(tau_cmd_),
                    // DATA_WRAPPER(current_test_torque_) // 单独记录测试附加力矩
                },
                std::initializer_list<ExperimentContext>{},
                1000);

            RCLCPP_INFO(node_->get_logger(), "启动测试! 底层已有重力补偿，请保持机械臂周围无障碍物。");
            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            return CallbackReturn::SUCCESS;
        }

        void update(const rclcpp::Time &t, const rclcpp::Duration &period) override
        {
            double dt = period.seconds();
            time_ += dt;

            std::vector<double> &tau_cmd_vec = command_->get<double>("torque");
            const std::vector<double> &q_vec = state_->get<double>("position");
            const std::vector<double> &dq_vec = state_->get<double>("velocity");

            command_->get<int>("mode")[0] = 3; // 硬件底层的力矩模式

            q_ = Eigen::Map<const Eigen::VectorXd>(q_vec.data(), dof_);
            dq_ = Eigen::Map<const Eigen::VectorXd>(dq_vec.data(), dof_);

            tau_cmd_.setZero();

            // ========== 测试状态机 ==========
            switch (test_state_)
            {
            case 0: // 阶段0：初始化等待 (等机械臂在重力补偿下稳定2秒)
                state_timer_ += dt;
                if (state_timer_ > 2.0)
                {
                    initial_q_ = q_(test_joint_idx_);
                    test_state_ = 1;
                    RCLCPP_INFO(node_->get_logger(), "[第 %d/%d 次] 开始正向摩擦力测试...", current_iteration_ + 1, total_iterations_);
                }
                break;

            case 1: // 阶段1：正向斜坡加力
                current_test_torque_ += ramp_rate_ * dt;

                // 破局判定
                if (std::abs(q_(test_joint_idx_) - initial_q_) > disp_thresh_)
                {
                    breakaway_torque_pos_ = current_test_torque_;
                    test_state_ = 2; // 切换到冷却刹车
                }
                // 安全上限判定
                else if (current_test_torque_ > max_torque_)
                {
                    breakaway_torque_pos_ = max_torque_;
                    RCLCPP_WARN(node_->get_logger(), "[安全中止] 达到最大力矩 %.2f Nm 仍未滑动!", max_torque_);
                    test_state_ = 2;
                }
                break;

            case 2: // 阶段2：正向冷却卸力 (快速平滑降至0)
                current_test_torque_ -= cooldown_rate_ * dt;
                if (current_test_torque_ <= 0.0)
                {
                    current_test_torque_ = 0.0;
                    state_timer_ = 0.0;
                    test_state_ = 3; // 等待稳定
                }
                break;

            case 3: // 阶段3：反向测试前等待
                state_timer_ += dt;
                if (state_timer_ > 2.0)
                {
                    initial_q_ = q_(test_joint_idx_);
                    test_state_ = 4;
                    RCLCPP_INFO(node_->get_logger(), "[第 %d/%d 次] 开始反向摩擦力测试...", current_iteration_ + 1, total_iterations_);
                }
                break;

            case 4: // 阶段4：反向斜坡加力
                current_test_torque_ -= ramp_rate_ * dt;

                // 破局判定
                if (std::abs(q_(test_joint_idx_) - initial_q_) > disp_thresh_)
                {
                    breakaway_torque_neg_ = std::abs(current_test_torque_);
                    test_state_ = 5;
                }
                else if (current_test_torque_ < -max_torque_)
                {
                    breakaway_torque_neg_ = max_torque_;
                    RCLCPP_WARN(node_->get_logger(), "[安全中止] 反向达到最大力矩 %.2f Nm 仍未滑动!", max_torque_);
                    test_state_ = 5;
                }
                break;

            case 5: // 阶段5：反向冷却卸力
                current_test_torque_ += cooldown_rate_ * dt;
                if (current_test_torque_ >= 0.0)
                {
                    current_test_torque_ = 0.0;
                    test_state_ = 6;
                }
                break;

            case 6: // 阶段6：一次循环完成，记录数据并判定是否继续
                pos_breakaways_.push_back(breakaway_torque_pos_);
                neg_breakaways_.push_back(breakaway_torque_neg_);

                RCLCPP_INFO(node_->get_logger(), " --- 第 %d 次循环结果: 正向 %.4f Nm, 反向 %.4f Nm ---",
                            current_iteration_ + 1, breakaway_torque_pos_, breakaway_torque_neg_);

                current_iteration_++;

                if (current_iteration_ < total_iterations_)
                {
                    // 还没有做够设定次数，回到阶段 0 继续
                    test_state_ = 0;
                    state_timer_ = 0.0;
                }
                else
                {
                    // 已经完成所有循环，进入结算阶段
                    test_state_ = 7;
                }
                break;

            case 7: // 阶段7：最终统计与结算输出
                if (!test_finished_)
                {
                    double sum_pos = 0.0, sum_neg = 0.0;
                    for (int i = 0; i < total_iterations_; i++)
                    {
                        sum_pos += pos_breakaways_[i];
                        sum_neg += neg_breakaways_[i];
                    }
                    double avg_pos = sum_pos / total_iterations_;
                    double avg_neg = sum_neg / total_iterations_;
                    double avg_total = (avg_pos + avg_neg) / 2.0;

                    RCLCPP_INFO(node_->get_logger(), "====================================");
                    RCLCPP_INFO(node_->get_logger(), " 测试彻底完成! 关节 [%d] 共循环 %d 次", test_joint_idx_, total_iterations_);
                    RCLCPP_INFO(node_->get_logger(), " 正向静摩擦平均值: %.4f Nm", avg_pos);
                    RCLCPP_INFO(node_->get_logger(), " 反向静摩擦平均值: %.4f Nm", avg_neg);
                    RCLCPP_INFO(node_->get_logger(), " 最终推荐前馈配置 (F_s_actual_): %.4f Nm", avg_total);
                    RCLCPP_INFO(node_->get_logger(), "====================================");

                    test_finished_ = true;
                }
                break;
            }

            // 只在被测试的关节上施加斜坡力矩
            tau_cmd_(test_joint_idx_) = current_test_torque_;

            // 写入硬件
            std::copy(tau_cmd_.data(), tau_cmd_.data() + dof_, tau_cmd_vec.begin());

            data_logger_->record();
        }

    protected:
        int dof_;
        double time_;
        int test_state_;
        double state_timer_;
        bool test_finished_;

        int test_joint_idx_;
        int total_iterations_;  // 设定的总循环次数
        int current_iteration_; // 当前进行的循环索引

        double ramp_rate_, cooldown_rate_, max_torque_;
        double vel_thresh_, disp_thresh_;

        double initial_q_;
        double current_test_torque_;
        double breakaway_torque_pos_;
        double breakaway_torque_neg_;

        std::vector<double> pos_breakaways_; // 保存每次正向测试的结果
        std::vector<double> neg_breakaways_; // 保存每次反向测试的结果

        Eigen::VectorXd q_, dq_, tau_cmd_;
        std::unique_ptr<DataLogger> data_logger_;
    };
} // namespace controllers

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(controllers::TorqueController, controller_interface::ControllerInterface)