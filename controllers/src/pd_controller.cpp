#include "robot_controller_interface/controller_interface.hpp"
#include <iostream>
#include "robot_math/robot_math.hpp"
// 移除了 JointTrajectoryPlanner，因为 PD 不需要轨迹规划
#include "realtime_tools/realtime_buffer.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "robot_control_msgs/action/robot_motion.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace controllers
{

    class PDMotionController : public controller_interface::ControllerInterface
    {
    public:
        using ACTION = robot_control_msgs::action::RobotMotion;
        using GoalHandle = rclcpp_action::ServerGoalHandle<ACTION>;

        PDMotionController() 
        {
            // 初始化默认 PD 参数 (需要根据实际机器人动力学调整)
            // 这里假设是7轴机器人，简单赋相同值
            kp_ = std::vector<double>(7, 1000.0); // 刚度系数
            kd_ = std::vector<double>(7, 50.0);   // 阻尼系数
                // q0_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        }

        void update(const rclcpp::Time & t, const rclcpp::Duration & period) override
        {
            auto goal_handle = *real_time_buffer_.readFromRT();
            // 【重要修改】获取 "effort" (力矩) 接口用于写入，而不是 "position"
            // 如果你的机器人不支持力矩接口，这里仍然可以用 "position" 做差分，
            // 但那就不是纯正的动力学PD控制了。这里假设是力矩控制。
            auto &cmd = command_->get<double>("torque"); 
            
            auto &q = state_->get<double>("position");
            auto &dq = state_->get<double>("velocity");

            // 初始化读取
            if(period.seconds() == 0)
            {
                q0_ = q;
            }

            // 【修改】切换模式为力矩控制 (假设 0 或 10 是力矩模式，需根据驱动确认)
            command_->get<int>("mode")[0] = 3; 

            std::vector<double> target_q;

            // 1. 确定目标位置 (Target Position)
            bool has_active_goal = false;
            if (goal_handle && goal_handle->is_active())
            {
                if (goal_handle->is_canceling())
                {
                    auto result = std::make_shared<ACTION::Result>();
                    result->success = false;
                    goal_handle->canceled(result);
                    q0_ = q; // 取消时，目标变为当前位置（刹车效果）
                    target_q = q0_;
                }
                else
                {
                    target_q = goal_handle->get_goal()->target_position.data;
                    has_active_goal = true;

                    // 检查是否到达目标 (收敛判断)
                    double err = robot_math::distance(q, target_q);
                    // PD控制中，需要误差足够小且速度足够小才算稳定到达
                    double velocity_norm = 0.0; 
                    for(double v : dq) velocity_norm += v*v;
                    
                    if (err < 0.1 && velocity_norm < 1e-3)
                    {
                        auto result = std::make_shared<ACTION::Result>();
                        result->success = true;
                        goal_handle->succeed(result);
                        q0_ = target_q; // 成功后，保持在目标位置
                        // 注意：PD控制由于重力影响，可能存在稳态误差，
                        // 除非有重力补偿 (Gravity Compensation) 叠加在输出上。
                    }
                }
            }
            else
            {
                // 无任务时，维持在这个位置
                target_q = q0_;
            }

            // 2. 计算 PD 控制律 (Control Law)
            // tau = Kp * (q_des - q) + Kd * (dq_des - dq)
            // 假设目标速度 dq_des 为 0
            size_t dof = q.size();
            
            // 确保 cmd 大小正确
            if (cmd.size() != dof) cmd.resize(dof);

            for(size_t i = 0; i < dof; ++i)
            {
                double error_p = target_q[i] - q[i];
                double error_d = 0.0 - dq[i]; // 目标速度为0

                // 计算力矩
                double tau = kp_[i] * error_p + kd_[i] * error_d;

                // 安全限幅 (根据机器人规格添加)
                // tau = std::max(std::min(tau, 30.0), -30.0);

                cmd[i] = tau;
            }
            
            // 注意：实际机器人通常需要叠加重力补偿模型
            // cmd[i] += gravity_torques[i]; 
        }

        CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            // 可以从 yaml 读取 PD 参数
            // node_->get_parameter("kp", kp_);
            
            real_time_buffer_.reset();
            
            // Action Server 配置 (与原代码相同)
            auto handle_goal = [this](const rclcpp_action::GoalUUID &uuid,
                                   std::shared_ptr<const ACTION::Goal> goal)
            {
                (void)uuid;
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            };

            auto handle_cancel = [this](const std::shared_ptr<GoalHandle> goal_handle)
            {
                (void)goal_handle;
                return rclcpp_action::CancelResponse::ACCEPT;
            };

            auto handle_accepted = [this](const std::shared_ptr<GoalHandle> goal_handle)
            {
                real_time_buffer_.writeFromNonRT(goal_handle);
            };

            call_back_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
            this->action_server_ = rclcpp_action::create_server<ACTION>(
                node_,
                "~/goal",
                handle_goal,
                handle_cancel,
                handle_accepted,
                rcl_action_server_get_default_options(),
                call_back_group_);
            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            action_server_ = nullptr;
            return CallbackReturn::SUCCESS;
        }

    protected:
        realtime_tools::RealtimeBuffer<std::shared_ptr<GoalHandle>> real_time_buffer_;
        std::vector<double> q0_;
        
        // PD 增益
        std::vector<double> kp_;
        std::vector<double> kd_;

        rclcpp_action::Server<ACTION>::SharedPtr action_server_;
        rclcpp::CallbackGroup::SharedPtr call_back_group_;
    };

} // namespace controllers

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(controllers::PDMotionController, controller_interface::ControllerInterface)