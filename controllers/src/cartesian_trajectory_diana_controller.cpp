#include "robot_controller_interface/controller_interface.hpp"
#include <iostream>
#include "robot_math/robot_math.hpp"
#include "ros2_utility/ros2_visual_tools.hpp"
#include "robot_math/CartesianTrajectory.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "robot_control_msgs/action/robot_motion.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_control_msgs/msg/robot_state.hpp"
#include "ros2_utility/data_logger.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "ros2_utility/file_utils.hpp"
#include <fstream>
#include <iomanip>

// === 新增：包含滤波器的头文件 ===
#include "robot_math/MovingFilter.h"

namespace controllers
{

    class CartesianTrajectoryDianaController : public controller_interface::ControllerInterface
    {
    public:
        using ACTION = robot_control_msgs::action::RobotMotion;
        using GoalHandle = rclcpp_action::ServerGoalHandle<ACTION>;
        using BufferType = std::pair<std::shared_ptr<GoalHandle>, std::shared_ptr<robot_math::CartesianTrajectory>>;
        
        // === 新增：在构造函数中初始化滤波器 (6个通道,窗口大小为15) ===
        CartesianTrajectoryDianaController() : f_filter_(6, 15)
        {
        }

        ~CartesianTrajectoryDianaController() 
        {
            if (data_logger_)
               data_logger_->save("/home/wjc/experiment_logs/cartesian_trajectory_controller_diana/", "cartesian_trajectory_controller_diana");
        }

        void update(const rclcpp::Time & t, const rclcpp::Duration & period) override
        {
            time_ += period.seconds();
            auto handle_pair = *real_time_buffer_.readFromRT();
            auto goal_handle = handle_pair.first;
            auto trajectory = handle_pair.second;
            auto &cmd = command_->get<double>("pose");
            auto &q = state_->get<double>("position");
            auto &dq = state_->get<double>("velocity");
            auto &force_vec = com_state_->at("ft_sensor")->get<double>("force");
            const std::vector<double> &pose_vec = state_->get<double>("pose");
            
            // 1. 获取正运动学矩阵 T (相当于参考代码里的 Tb_)
            Eigen::Matrix4d T;
            robot_math::forward_kinematics(robot_, q, T);
            auto pose = robot_math::tform_to_pose(T);

            // =======================================================
            // === 核心修改：力传感器重力补偿与滤波 ===
            // =======================================================
            force_ = Eigen::Map<const Eigen::Vector6d>(force_vec.data());

            // 调用外部补偿函数 (注意这里最后一个参数传 T)
            Eigen::Vector6d raw_compensated = robot_math::get_ext_force(
                force_vec, 
                sensor_weight_, 
                sensor_offset_vec_, 
                sensor_cog_vec_, 
                T_sensor_, 
                T 
            );
            
            force_.head(3) = raw_compensated.tail(3); 
            force_.tail(3) = raw_compensated.head(3); 
            Eigen::Matrix3d R_tcp_to_sensor = T_sensor_.block<3,3>(0,0);
            force_.head(3) = R_tcp_to_sensor * force_.head(3);
            force_.tail(3) = R_tcp_to_sensor * force_.tail(3);
 
            // 滤波
            f_filter_.filtering(force_.data(), force_.data());

            // force_.setZero(); // === 临时措施：先把力清零,验证轨迹走位逻辑正常后再放开 ===
            // =======================================================

            pose_ = Eigen::Map<const Eigen::Vector6d>(pose_vec.data());
        
            robot_control_msgs::msg::RobotState msg;
            msg.header.stamp = t;
            std::fill_n(std::back_inserter(msg.robot_state), 28, 0);
            std::copy(q.begin(), q.end(), msg.robot_state.begin());
            std::copy(dq.begin(), dq.end(), msg.robot_state.begin() + 7);
            if (real_time_publisher_->trylock())
            {
                real_time_publisher_->msg_ = msg;
                real_time_publisher_->unlockAndPublish();
            }

            if(period.seconds() == 0)
            {
                q0_ = q;
                pose0_ = pose;
            }
            command_->get<int>("mode")[0] = 0;

            if (goal_handle && goal_handle->is_active())
            {
                if (goal_handle->is_canceling())
                {
                    auto result = std::make_shared<ACTION::Result>();
                    result->success = false;
                    goal_handle->canceled(result);
                    pose0_ = pose;
                }
                else
                {
                    // ========================================================
                    // === 结合上一轮的逻辑：滤波补偿后的 力控接触检测 ===
                    // ========================================================
                    double force_z = force_[2]; // 此时的 force_[2] 已经是去除了工具重力、且经过滤波的干净力！
                    const double FORCE_THRESHOLD = 2.0; // 接触阈值

                    if (std::abs(force_z) >= FORCE_THRESHOLD)
                    {
                        cmd = pose; // 原地停止
                        pose0_ = pose;
                        
                        auto result = std::make_shared<ACTION::Result>();
                        result->success = true;
                        goal_handle->succeed(result);

                        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, 
                            "Contact Detected! Fz = %.2f N. Stopping.", force_z);
                        return;
                    }
                    // ========================================================

                    // 正常的轨迹走位逻辑
                    auto goal = std::vector<double>(goal_handle->get_goal()->target_position.data.end() - 6, goal_handle->get_goal()->target_position.data.end());
                    auto goal_T = robot_math::pose_to_tform(goal);
                    auto errs = robot_math::distance(goal_T, T);
                    auto dt = node_->now() - last_time_;
                    
                    if (errs.first < 1e-2 && errs.second < 1e-5 && dt.seconds() >= trajectory->total_time()) 
                    {
                        auto result = std::make_shared<ACTION::Result>();
                        result->success = true;
                        goal_handle->succeed(result);
                        pose0_ = pose;
                    }
                    else
                    {
                        Eigen::Matrix4d Td;
                        Eigen::Vector6d V, dV;
                        trajectory->evaluate(dt.seconds(), Td, V, dV);
                        cmd = robot_math::tform_to_pose(Td);
                        double v_norm = V.norm();
                        
                        if (v_norm < 1e-5) 
                        {
                            if (!has_logged_current_pose_) 
                            {
                                data_logger_->record();
                                has_logged_current_pose_ = true; 
                                current_waypoint_index_++;
                                RCLCPP_INFO(node_->get_logger(), 
                                    "Auto-logged at waypoint [%d]. Current Pose: {%.4f, %.4f, %.4f, %.4f, %.4f, %.4f}", 
                                    current_waypoint_index_,
                                    pose_[0], pose_[1], pose_[2], pose_[3], pose_[4], pose_[5]);
                            }
                        }
                        else 
                        {
                            if (v_norm > 1e-3) {
                                has_logged_current_pose_ = false;
                            }
                        }
                    }
                }
            }
            else
            {
                cmd = pose0_;
            }
        }

        CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            time_ = 0;

            // =======================================================
            // === 新增：初始化传感器补偿参数与重置滤波器 ===
            // =======================================================
            f_filter_.reset();
            sensor_weight_ = 0.61569; 
            sensor_cog_vec_ = {-0.0004 ,  -0.0000 ,   0.0026};
            sensor_offset_vec_ = { -5.8497   , 5.6095,   -9.4235  ,  0.4396 ,   0.2853   , 0.2743};
            T_sensor_ = Eigen::Matrix4d::Identity();
            T_sensor_ << 1,  0,  0, 0,
                         0, -1,  0, 0,
                         0,  0, -1, 0,
                         0,  0,  0, 1;
            // =======================================================

            robot_state_publisher_ = node_->create_publisher<robot_control_msgs::msg::RobotState>("robot_states", rclcpp::SensorDataQoS());
            real_time_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<robot_control_msgs::msg::RobotState>>(robot_state_publisher_);
            real_time_buffer_.reset();
            
            auto handle_goal = [this](const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const ACTION::Goal> goal) {
                (void)uuid; return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            };
            auto handle_cancel = [this](const std::shared_ptr<GoalHandle> goal_handle) {
                (void)goal_handle; return rclcpp_action::CancelResponse::ACCEPT;
            };

            auto handle_accepted = [this](const std::shared_ptr<GoalHandle> goal_handle) {
                auto trajectory = std::make_shared<robot_math::CartesianTrajectory>();
                // const std::vector<double> &pose_current = state_->get<double>("pose");
                const std::vector<double> &q_current = state_->get<double>("position");
                Eigen::Matrix4d T_current;
                robot_math::forward_kinematics(robot_, q_current, T_current);
                std::vector<double> pose_current = robot_math::tform_to_pose(T_current);
                std::vector<double> full_traj_data;
                full_traj_data.push_back(0.0); 
                for(int i = 0; i < 6; ++i) {
                    full_traj_data.push_back(pose_current[i]); 
                }
                const auto& goal_data = goal_handle->get_goal()->target_position.data;
                full_traj_data.insert(full_traj_data.end(), goal_data.begin(), goal_data.end());
                trajectory->set_traj(full_traj_data);
                
                last_time_ = node_->now();
                current_waypoint_index_ = 0; 
                real_time_buffer_.writeFromNonRT({goal_handle, trajectory});
            };

            call_back_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
            this->action_server_ = rclcpp_action::create_server<ACTION>(
                node_, "~/goal", handle_goal, handle_cancel, handle_accepted,
                rcl_action_server_get_default_options(), call_back_group_);
                
            pose_ = Eigen::Vector6d::Zero();
            force_ = Eigen::Vector6d::Zero();
            data_logger_ = std::make_unique<DataLogger>(
                std::initializer_list<DataInfo>{
                    DATA_WRAPPER(time_), DATA_WRAPPER(pose_), DATA_WRAPPER(force_),
                },
                std::initializer_list<ExperimentContext>{},
                1000);
            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            action_server_ = nullptr;
            real_time_publisher_ = nullptr;
            robot_state_publisher_ = nullptr;
            return CallbackReturn::SUCCESS;
        }

    protected:
        realtime_tools::RealtimeBuffer<BufferType> real_time_buffer_;
        rclcpp_action::Server<ACTION>::SharedPtr action_server_;
        std::vector<double> q0_;
        std::vector<double> pose0_;
        rclcpp::Time last_time_;
        rclcpp::CallbackGroup::SharedPtr call_back_group_;
        rclcpp::Publisher<robot_control_msgs::msg::RobotState>::SharedPtr robot_state_publisher_;
        std::shared_ptr<realtime_tools::RealtimePublisher<robot_control_msgs::msg::RobotState>> real_time_publisher_;
        std::unique_ptr<DataLogger> data_logger_;
        Eigen::Vector6d pose_, force_;
        int current_waypoint_index_ = 0; 
        bool has_logged_current_pose_ = true;
        double time_;

        // =======================================================
        // === 新增：保护成员变量声明 ===
        // =======================================================
        robot_math::MovingFilter<double> f_filter_;
        std::vector<double> sensor_cog_vec_;
        std::vector<double> sensor_offset_vec_;
        double sensor_weight_;
        Eigen::Matrix4d T_sensor_;
        // =======================================================
    };

} // namespace controllers

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(controllers::CartesianTrajectoryDianaController, controller_interface::ControllerInterface)