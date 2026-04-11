#include "robot_hardware_interface/robot_interface.hpp"
#include "robot_math/robot_math.hpp"
#include "DianaAPI.h"
#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <vector>
#include <cstring>
#include <algorithm>
#include <thread>
#include <cstdlib>
#include <ctime>
#include "robot_math/KalmanFilter.h"
#include <mutex>
#include <condition_variable>
#include <pthread.h>

using namespace robot_math;
using namespace std::chrono_literals;

namespace hardwares
{
    class DIANARobot; // 前置声明类

    // 前置声明回调函数
    void robotStateCallback(StrRobotStateInfo *pinfo, const char *strIpAddress);

    class DIANARobot : public hardware_interface::RobotInterface
    {
        friend void robotStateCallback(StrRobotStateInfo *pinfo, const char *strIpAddress);

    public:
        static DIANARobot* instance_;

        DIANARobot() : pre_dq_(7), prev_torque_(7, 0.0), kf_dq_(7, 0.01, 0.05)
        {
            srand((unsigned int)time(nullptr));
        }
        ~DIANARobot() {}

        void write(const rclcpp::Time& t, const rclcpp::Duration& period) override
        {
            hardware_interface::RobotInterface::write(t, period);
            double dt = period.seconds();
            int mode = command_.get<int>("mode")[0]; 

            // ==============================================================
            // ✅ 优化 1：状态机平滑切换，彻底剔除 sleep_for，防止破坏 1ms 周期！
            // ==============================================================
            if (mode != prev_mode_)
            {
                if (prev_mode_ == 3) {
                    enableTorqueReceiver(false, robot_ip_.c_str());
                }
                
                if (mode == 3) {
                    enableTorqueReceiver(true, robot_ip_.c_str());
                    // 用 Tick 计数器代替 5ms 的 sleep，每个 tick 是 1ms
                    mode_transition_ticks_ = 5; 
                } else if (mode == 4) {
                    changeControlMode(T_MODE_CART_IMPEDANCE, robot_ip_.c_str());
                    mode_transition_ticks_ = 0;
                }
                prev_mode_ = mode;
                RCLCPP_INFO(node_->get_logger(), "Diana switched to Mode: %d", mode);
            }

            // 如果正在等待底层硬件模式切换生效，直接 return 放弃本周期的写入
            if (mode_transition_ticks_ > 0) {
                mode_transition_ticks_--;
                return; 
            }

            // ==============================================================
            // ✅ 优化 2：全部使用 _ex 异步无阻塞 API (realiable = false)
            // 确保指令下发后立刻返回，避免 TCP/UDP 握手造成的网络阻塞
            // ==============================================================
            switch (mode)
            {
                case 0: 
                {
                    auto &cmd_pose = command_.get<double>("pose");
                    double pose_cmd[6];
                    std::copy(cmd_pose.begin(), cmd_pose.begin() + 6, pose_cmd);
                    // realiable = false 保证非阻塞返回
                    servoL_ex(pose_cmd, dt, 0.05, 500.0, 1.0, false, nullptr, robot_ip_.c_str());
                    break;
                }
                case 1: 
                {
                    auto &cmd_pos = command_.get<double>("position");
                    double pos_cmd[7];
                    std::copy(cmd_pos.begin(), cmd_pos.begin() + 7, pos_cmd);
                    // realiable = false 保证非阻塞返回
                    servoJ_ex(pos_cmd, dt, 0.05, 500.0, false, robot_ip_.c_str());
                    break;
                }
                case 2: 
                {
                    auto &cmd_vel = command_.get<double>("velocity");
                    double vel_cmd[7];
                    std::copy(cmd_vel.begin(), cmd_vel.begin() + 7, vel_cmd);
                    // realiable = false 保证非阻塞返回
                    speedJ_ex(vel_cmd, 1.5, 0.0, false, robot_ip_.c_str());
                    break;
                }
                case 3: 
                {
                    auto &cmd_tau = command_.get<double>("torque");
                    double torque_cmd[7]; 
                    std::copy(cmd_tau.begin(), cmd_tau.begin() + 7, torque_cmd);
                    // 实时力矩本身就是火控即丢(Fire-and-forget)的逻辑
                    sendTorque_rt(torque_cmd, 0.002, robot_ip_.c_str());
                    break;   
                }    
                case 4:
                {
                    auto &cmd_cart_pose = command_.get<double>("pose");
                    double target_pose[6]; 
                    std::copy(cmd_cart_pose.begin(), cmd_cart_pose.begin() + 6, target_pose);
                    servoL_ex(target_pose, dt, 0.05, 500, 1.0, false, nullptr, robot_ip_.c_str());
                    break;
                } 
                default:
                    stop(robot_ip_.c_str());
                    break;
            }     
        }

        bool is_stop() override { return false; }

        void read(const rclcpp::Time& t, const rclcpp::Duration& period) override
        {
            auto& q = state_.get<double>("position");
            auto& dq = state_.get<double>("velocity");
            auto& ddq = state_.get<double>("acceleration");
            auto& pose = state_.get<double>("pose");
            auto& torque = state_.get<double>("torque");

            // ==============================================================
            // ✅ 优化 3：硬件时钟驱动（Hardware-Driven）
            // 类似于 Franka，在此处严格阻塞，直到机器人发来 1ms 的最新数据
            // ==============================================================
            {
                std::unique_lock<std::mutex> lock(state_mutex_);
                // 设置 1.5ms 超时。如果在 1.5ms 内底层没有推数据，说明网络丢包，打印警告
                bool data_arrived = state_cv_.wait_for(lock, std::chrono::microseconds(1500), 
                                                       [this] { return new_data_received_; });
                
                if (!data_arrived) {
                    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, 
                                         "Diana 1ms real-time loop missed a tick! (UDP Drop or High CPU Load)");
                } else {
                    new_data_received_ = false; // 消费状态标志
                }

                // 安全拷贝
                std::copy(latest_state_.jointPos, latest_state_.jointPos + 7, q.data());
                std::copy(latest_state_.jointAngularVel, latest_state_.jointAngularVel + 7, dq.data());
                std::copy(latest_state_.tcpPos, latest_state_.tcpPos + 6, pose.data());
                std::copy(latest_state_.jointTorque, latest_state_.jointTorque + 7, torque.data());
            }

            // 计算加速度
            static std::vector<double> prev_dq(7, 0.0);
            double dt = period.seconds();
            if (dt > 0.0) {
                for (int i = 0; i < 7; ++i) {
                    ddq[i] = (dq[i] - prev_dq[i]) / dt;
                }
            }
            prev_dq = dq;
        }

        CallbackReturn on_configure(const rclcpp_lifecycle::State& prev) override
        {
            if (RobotInterface::on_configure(prev) != CallbackReturn::SUCCESS) 
                return CallbackReturn::FAILURE;

            instance_ = this; // 绑定单例

            node_->get_parameter_or<std::string>("robot_ip", robot_ip_, std::string(""));
            if (robot_ip_.empty()) 
            {
                RCLCPP_ERROR(node_->get_logger(), "robot_ip is not set");
                return CallbackReturn::FAILURE;
            }

            srv_net_ = std::make_unique<srv_net_st>();
            initSrvNetInfo(srv_net_.get());
            strcpy(srv_net_->SrvIp, robot_ip_.c_str());
            return initSrv(nullptr, robotStateCallback, srv_net_.get()) == 0 ? 
                CallbackReturn::SUCCESS : CallbackReturn::FAILURE;
        }

        CallbackReturn on_shutdown(const rclcpp_lifecycle::State& prev) override
        {
            RobotInterface::on_shutdown(prev);
            if (!robot_ip_.empty()) 
            {               
                if(prev_mode_ == 3) {
                    double zero_torque[7] = {0.0};
                    sendTorque_rt(zero_torque, 0, robot_ip_.c_str());
                    enableTorqueReceiver(false, robot_ip_.c_str());
                }              
                changeControlMode(T_MODE_POSITION, robot_ip_.c_str());
                stop(robot_ip_.c_str());
                destroySrv(robot_ip_.c_str());
            }
            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_activate(const rclcpp_lifecycle::State& prev) override
        {
            // 确保底层开启 1ms 数据推送频率
            int ret = setPushPeriod(1, robot_ip_.c_str());
            if (ret < 0) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to set push period to 1ms!");
            }

            cleanErrorInfo(robot_ip_.c_str()); 
            if (RobotInterface::on_activate(prev) == CallbackReturn::SUCCESS)
            {
                double joint_collision[7] = {200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0};
                double cart_collision[6] = {200.0, 200.0, 200.0, 200.0, 200.0, 200.0};
                setJointCollision(joint_collision, robot_ip_.c_str());
                setCartCollision(cart_collision, robot_ip_.c_str());
                
                double payload[10] = {0.0};
                setActiveTcpPayload(payload, robot_ip_.c_str());
                kf_dq_.reset();

                std::fill(prev_torque_.begin(), prev_torque_.end(), 0.0);   
                std::fill(pre_dq_.begin(), pre_dq_.end(), 0.0);

                releaseBrake(robot_ip_.c_str());
                rclcpp::sleep_for(std::chrono::seconds(2));

                return CallbackReturn::SUCCESS;
            }
            return CallbackReturn::FAILURE;
        }

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State& prev) override
        {
            RobotInterface::on_deactivate(prev);
            if (prev_mode_ == 3) {
                double zero_torque[7] = {0.0};
                sendTorque_rt(zero_torque, 0, robot_ip_.c_str());  
                enableTorqueReceiver(false, robot_ip_.c_str());
            }
            
            std::fill(prev_torque_.begin(), prev_torque_.end(), 0.0);       
            stop(robot_ip_.c_str());
            return CallbackReturn::SUCCESS;
        }

    protected:
        std::string robot_ip_;
        std::unique_ptr<srv_net_st> srv_net_;
        std::vector<double> pre_dq_;

        std::vector<double> prev_torque_; 
        robot_math::KalmanFilter<double> kf_dq_;
        
        StrRobotStateInfo latest_state_;
        std::mutex state_mutex_;
        std::condition_variable state_cv_;
        bool new_data_received_ = false; 
        
        int prev_mode_ = -1;
        int mode_transition_ticks_ = 0; // ✅ 用于在模式切换时非阻塞跳过指定数目的 1ms 周期
    };

    DIANARobot* DIANARobot::instance_ = nullptr;

    // ==============================================================
    // ✅ 回调函数内处理：收到数据立刻唤醒 PC，这是实时循环跳动的心脏
    // ==============================================================
    void robotStateCallback(StrRobotStateInfo *pinfo, const char *strIpAddress)
    {
        static bool priority_boosted = false;
        if (!priority_boosted) {
            struct sched_param param;
            param.sched_priority = 60; // 设为 60！比 ROS 2 的 50 还要高，确保第一时间收包
            if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) == 0) {
                printf("[RT Setup] Diana API thread promoted to FIFO Priority 60\n");
                priority_boosted = true;
            } else {
                perror("pthread_setschedparam failed");
            }
        }
        static auto last_cb_time = std::chrono::high_resolution_clock::now();
        auto now = std::chrono::high_resolution_clock::now();
        auto interval = std::chrono::duration_cast<std::chrono::microseconds>(now - last_cb_time).count();
        last_cb_time = now;
        if (interval > 1100) { 
            // 使用 printf 以减少对实时性的干扰
            printf("[API Debug] Robot heartbeat delay: %ld us\n", interval);
        }
        if (DIANARobot::instance_ != nullptr && pinfo != nullptr)
        {
            {
                std::lock_guard<std::mutex> lock(DIANARobot::instance_->state_mutex_);
                DIANARobot::instance_->latest_state_ = *pinfo;
                DIANARobot::instance_->new_data_received_ = true;
            }
            // 立刻发出唤醒信号！让堵在 read() 里的 ROS 控制循环向下运行！
            DIANARobot::instance_->state_cv_.notify_one();
        }
    }

} // namespace hardwares

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(hardwares::DIANARobot, hardware_interface::RobotInterface)