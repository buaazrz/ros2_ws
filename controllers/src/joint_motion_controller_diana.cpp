#include "robot_controller_interface/controller_interface.hpp"
#include <iostream>
#include "robot_math/robot_math.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "robot_control_msgs/action/robot_motion.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros2_utility/data_logger.hpp"

namespace controllers
{
    // 定义一个结构体用于存储带时间戳的关节路点
    struct JointWaypoint {
        double time;
        std::vector<double> positions;
    };

    class JointMotionControllerDiana : public controller_interface::ControllerInterface
    {
    public:
        using ACTION = robot_control_msgs::action::RobotMotion;
        using GoalHandle = rclcpp_action::ServerGoalHandle<ACTION>;
        using BufferType = std::pair<std::shared_ptr<GoalHandle>, std::vector<JointWaypoint>>;

        JointMotionControllerDiana()
        {
        }

        ~JointMotionControllerDiana() 
        {
            if (data_logger_)
               data_logger_->save("/home/wjc/experiment_logs/joint_trajectory_controller/", "joint_trajectory_data");
        }

        void update(const rclcpp::Time & t, const rclcpp::Duration & period) override
        {
            time_ += period.seconds();
            auto handle_pair = *real_time_buffer_.readFromRT();
            auto goal_handle = handle_pair.first;
            auto waypoints = handle_pair.second;

            auto &cmd = command_->get<double>("position");
            auto &q = state_->get<double>("position");
            auto &pose = state_->get<double>("pose");
            
            // 为了 DataLogger 记录，读取力传感器数据并通过正运动学计算当前笛卡尔位姿
            auto &force_vec = com_state_->at("ft_sensor")->get<double>("force");
            force_ = Eigen::Map<const Eigen::Vector6d>(force_vec.data());

            
            pose_ = Eigen::Map<const Eigen::Vector6d>(pose.data());

            if(period.seconds() == 0)
            {
                q0_ = q;
            }
            command_->get<int>("mode")[0] = 1; // 1 表示关节位置模式

            if (goal_handle && goal_handle->is_active())
            {
                if (goal_handle->is_canceling())
                {
                    auto result = std::make_shared<ACTION::Result>();
                    result->success = false;
                    goal_handle->canceled(result);
                    q0_ = q;
                }
                else
                {
                    if (waypoints.empty()) return;

                    auto dt = (node_->now() - last_time_).seconds();
                    double total_time = waypoints.back().time;

                    // 1. 如果已经超过总时间，说明到达终点
                    if (dt >= total_time)
                    {
                        auto result = std::make_shared<ACTION::Result>();
                        result->success = true;
                        goal_handle->succeed(result);
                        q0_ = q;
                        cmd = waypoints.back().positions;
                    }
                    else
                    {
                        // 2. 遍历寻找当前处于哪一个时间段
                        for (size_t i = 0; i < waypoints.size() - 1; ++i)
                        {
                            if (dt >= waypoints[i].time && dt < waypoints[i+1].time)
                            {
                                double t0 = waypoints[i].time;
                                double t1 = waypoints[i+1].time;
                                double ratio = (dt - t0) / (t1 - t0);

                                bool is_staying = true; // 标志位：判断当前段是否是"停留"段

                                // 对所有关节进行线性插值
                                for (size_t j = 0; j < q.size(); ++j) {
                                    double start_q = waypoints[i].positions[j];
                                    double end_q = waypoints[i+1].positions[j];
                                    cmd[j] = start_q + ratio * (end_q - start_q);

                                    // 如果起点和终点不一样，说明在移动
                                    if (std::abs(end_q - start_q) > 1e-5) {
                                        is_staying = false;
                                    }
                                }

                                // 3. 核心：触发 Logger 记录
                                if (is_staying) 
                                {
                                    // 如果还没记录过这一位的位姿
                                    if (!has_logged_current_pose_) 
                                    {
                                        if (!is_waiting_to_log_) 
                                        {
                                            // 刚刚进入静止状态，开始计时
                                            stay_start_time_ = node_->now();
                                            is_waiting_to_log_ = true;
                                        } 
                                        else 
                                        {
                                            // 已经在等待中，检查是否超过 1.0 秒
                                            double wait_duration = (node_->now() - stay_start_time_).seconds();
                                            if (wait_duration >= 1.0) 
                                            {
                                                // 到达 1 秒，执行记录
                                                data_logger_->record();
                                                has_logged_current_pose_ = true;
                                                is_waiting_to_log_ = false; // 结束等待状态
                                                current_waypoint_index_++;
                                                
                                                RCLCPP_INFO(node_->get_logger(), 
                                                    "Stable for 1s. Auto-logged at Waypoint [%d]. Fz: %.2f", 
                                                    current_waypoint_index_, force_[2]);
                                            }
                                        }
                                    }
                                } 
                                else 
                                {
                                    // 只要机械臂在移动，就重置所有状态
                                    has_logged_current_pose_ = false; 
                                    is_waiting_to_log_ = false; 
                                }
                                break;
                            }
                        }
                    }
                }
            }
            else
            {
                cmd = q0_;
            }
        }

        CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            time_ = 0;
            real_time_buffer_.reset();
            has_logged_current_pose_ = false;
            is_waiting_to_log_ = false; // 新增
            current_waypoint_index_ = 0;

            auto handle_goal = [this](const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const ACTION::Goal> goal)
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
                auto raw_data = goal_handle->get_goal()->target_position.data;
                
                // 【核心修复 1】：获取当前机械臂真实的关节角度
                const auto& q_current = state_->get<double>("position");
                const size_t num_joints = q_current.size();
                const size_t stride = num_joints + 1; // 1个时间戳 + N个关节角

                std::vector<JointWaypoint> waypoints;
                
                // 【核心修复 2】：强制把当前的真实位置作为 t=0 的绝对起点
                JointWaypoint start_wp;
                start_wp.time = 0.0;
                start_wp.positions = q_current;
                waypoints.push_back(start_wp);

                // 解析发过来的数据
                for (size_t i = 0; i < raw_data.size(); i += stride) {
                    JointWaypoint wp;
                    wp.time = raw_data[i];
                    wp.positions = std::vector<double>(raw_data.begin() + i + 1, raw_data.begin() + i + stride);
                    
                    // 防呆机制：如果客户端发来了时间小于等于0的点，直接忽略
                    // 因为 t=0 已经被真实的机械臂位置占据了
                    if (wp.time <= 0.0) {
                        continue;
                    }
                    waypoints.push_back(wp);
                }

                last_time_ = node_->now();
                current_waypoint_index_ = 0;
                has_logged_current_pose_ = false;

                real_time_buffer_.writeFromNonRT({goal_handle, waypoints});
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

            pose_ = Eigen::Vector6d::Zero();
            force_ = Eigen::Vector6d::Zero();
            
            // 初始化 Logger
            data_logger_ = std::make_unique<DataLogger>(
                std::initializer_list<DataInfo>{
                    // DATA_WRAPPER(time_),
                    DATA_WRAPPER(pose_),  // 记录解算出的笛卡尔位姿
                    DATA_WRAPPER(force_), // 记录力传感器
                },
                std::initializer_list<ExperimentContext>{},
                1000);

            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            action_server_ = nullptr;
            return CallbackReturn::SUCCESS;
        }

    protected:
        realtime_tools::RealtimeBuffer<BufferType> real_time_buffer_;
        std::vector<double> q0_;
        rclcpp::Time last_time_;
        bool is_waiting_to_log_ = false; // 是否处于等待记录的1秒期间
        rclcpp::Time stay_start_time_;
        rclcpp_action::Server<ACTION>::SharedPtr action_server_;
        rclcpp::CallbackGroup::SharedPtr call_back_group_;
        
        std::unique_ptr<DataLogger> data_logger_;
        Eigen::Vector6d pose_, force_;
        double time_;
        
        bool has_logged_current_pose_ = false;
        int current_waypoint_index_ = 0;
    };

} // namespace controllers

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(controllers::JointMotionControllerDiana, controller_interface::ControllerInterface)