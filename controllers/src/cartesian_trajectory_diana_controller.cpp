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

namespace controllers
{

    class CartesianTrajectoryDianaController : public controller_interface::ControllerInterface
    {
    public:
        using ACTION = robot_control_msgs::action::RobotMotion;
        using GoalHandle = rclcpp_action::ServerGoalHandle<ACTION>;
        using BufferType = std::pair<std::shared_ptr<GoalHandle>, std::shared_ptr<robot_math::CartesianTrajectory>>;
        CartesianTrajectoryDianaController() 
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
            force_ = Eigen::Map<const Eigen::Vector6d>(force_vec.data());

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

            Eigen::Matrix4d T;
            robot_math::forward_kinematics(robot_, q, T);
            auto pose = robot_math::tform_to_pose(T);
            // inital reading should be put here!!
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
                    auto goal = std::vector<double>(goal_handle->get_goal()->target_position.data.end() - 6, goal_handle->get_goal()->target_position.data.end());
                    auto goal_T = robot_math::pose_to_tform(goal);
                    auto errs = robot_math::distance(goal_T, T);
                    auto dt = node_->now() - last_time_;
                    if (errs.first < 1e-2 && errs.second < 1e-5 && dt.seconds() >= trajectory->total_time()) // rv and pos
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
                        //visual_tools_->publishMarker(T.block(0, 3, 3, 1), "base", 0.5);
                        double v_norm = V.norm();
                        
                        if (v_norm < 1e-5) // 速度几乎为0，说明到达了停留点
                        {
                            // 2. 并且只在这个停留周期内记录一次
                            if (!has_logged_current_pose_) 
                            {
                                data_logger_->record();
                                has_logged_current_pose_ = true; // 锁定，防止这 2 秒内记录 2000 次
                                
                                // 可选：用节流打印提示一下记录成功
                                RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, 
                                    "Auto-logged data at target pose. Force Z: %.2f", force_[2]);
                            }
                        }
                        else 
                        {
                            // 3. 一旦速度起来了（开始前往下一个点），重置标志位
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

            // data_logger_->record();
        }
        CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            //visual_tools_ = std::make_shared<ROS2VisualTools>(node_);
            return CallbackReturn::SUCCESS;
        }
        CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            time_ = 0;
            robot_state_publisher_ = node_->create_publisher<robot_control_msgs::msg::RobotState>("robot_states", rclcpp::SensorDataQoS());
            real_time_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<robot_control_msgs::msg::RobotState>>(robot_state_publisher_);
            real_time_buffer_.reset();
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

            auto handle_accepted = [this](const std::shared_ptr<GoalHandle> goal_handle) {
                auto trajectory = std::make_shared<robot_math::CartesianTrajectory>();
                const std::vector<double> &pose_current = state_->get<double>("pose");
                std::vector<double> full_traj_data;
                full_traj_data.push_back(0.0); 
                for(int i = 0; i < 6; ++i) {
                    full_traj_data.push_back(pose_current[i]); 
                }
                const auto& goal_data = goal_handle->get_goal()->target_position.data;
                full_traj_data.insert(full_traj_data.end(), goal_data.begin(), goal_data.end());
                trajectory->set_traj(full_traj_data);
                
                last_time_ = node_->now();
                real_time_buffer_.writeFromNonRT({goal_handle, trajectory});

                double dt = 0.001;

                std::ofstream outfile("/tmp/reference_trajectory.csv"); 
                if (outfile.is_open())
                {
                    outfile << "time,x,y,z,qw,qx,qy,qz\n";
                    
                    for (double t = 0; t <= trajectory->total_time(); t += dt)
                    {
                        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
                        Eigen::Vector6d V, dV;
                        
                        trajectory->evaluate(t, T, V, dV);

                        double x = T(0, 3);
                        double y = T(1, 3);
                        double z = T(2, 3);
                        Eigen::Quaterniond q(T.block<3, 3>(0, 0));

                        outfile << std::fixed << std::setprecision(6)
                                << t << ","
                                << x << "," << y << "," << z << ","
                                << q.w() << "," << q.x() << "," << q.y() << "," << q.z() << "\n";
                    }
                    outfile.close();
                    RCLCPP_INFO(node_->get_logger(), "Reference trajectory saved to /tmp/reference_trajectory.csv");
                }
                else
                {
                    RCLCPP_ERROR(node_->get_logger(), "Failed to open file for trajectory saving!");
                }
            };
            // must be member variable
            // otherwise, the callback group will be destroyed before the action server
            call_back_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
            this->action_server_ = rclcpp_action::create_server<ACTION>(
                node_,
                "~/goal",
                handle_goal,
                handle_cancel,
                handle_accepted,
                rcl_action_server_get_default_options(),
                call_back_group_);
            pose_ = Eigen::Vector6d::Zero();
            force_ = Eigen::Vector6d::Zero();
            data_logger_ = std::make_unique<DataLogger>(
                std::initializer_list<DataInfo>{
                    DATA_WRAPPER(time_),
                    DATA_WRAPPER(pose_),
                    DATA_WRAPPER(force_),
                },
                std::initializer_list<ExperimentContext>{
                },
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
        //std::shared_ptr<ROS2VisualTools> visual_tools_;
        rclcpp::CallbackGroup::SharedPtr call_back_group_;
        rclcpp::Publisher<robot_control_msgs::msg::RobotState>::SharedPtr robot_state_publisher_;
        std::shared_ptr<realtime_tools::RealtimePublisher<robot_control_msgs::msg::RobotState>> real_time_publisher_;
        std::unique_ptr<DataLogger> data_logger_;
        Eigen::Vector6d pose_, force_;
        bool has_logged_current_pose_ = true;

        double time_;
    };

} // namespace controllers

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(controllers::CartesianTrajectoryDianaController, controller_interface::ControllerInterface)