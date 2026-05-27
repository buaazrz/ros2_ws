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
#include <chrono>

using namespace robot_math;
using namespace std::chrono_literals;

namespace hardwares
{
    class DIANARobot : public hardware_interface::RobotInterface
    {
    public:
        DIANARobot() : pre_dq_(7), prev_torque_(7, 0.0)
        {
            srand((unsigned int)time(nullptr));
        }
        ~DIANARobot()
        {
        }

        void write(const rclcpp::Time& t, const rclcpp::Duration& period) override
        {
            auto start_time = std::chrono::high_resolution_clock::now();
            hardware_interface::RobotInterface::write(t, period);
            double dt = 1.0 / update_rate_;
            int mode = command_.get<int>("mode")[0]; 
            if (mode != prev_mode_)
            {
                if (prev_mode_ == 3) {
                    enableTorqueReceiver(false, robot_ip_.c_str());
                    RCLCPP_INFO(node_->get_logger(), "Diana: Torque mode disabled");
                }
                if (mode == 3) {
                    enableTorqueReceiver(true, robot_ip_.c_str());
                    RCLCPP_INFO(node_->get_logger(), "Diana: Torque mode enabled");
                } else if (mode == 4) {
                    changeControlMode(T_MODE_CART_IMPEDANCE, robot_ip_.c_str());
                    RCLCPP_INFO(node_->get_logger(), "Diana: Switched to Cartesian Impedance mode");
                }           
                prev_mode_ = mode;
                RCLCPP_INFO(node_->get_logger(), "Diana: Control mode changed to %d", mode);
            }

            switch (mode)
            {
                case 0: 
                {
                    auto &cmd_pose = command_.get<double>("pose");
                    double pose_cmd[6];
                    std::copy(cmd_pose.begin(), cmd_pose.begin() + 6, pose_cmd);
                    servoL(pose_cmd, dt, 0.05, 500.0, 1.0, nullptr, robot_ip_.c_str());
                    break;
                }
                case 1: 
                {
                    auto &cmd_pos = command_.get<double>("position");
                    double pos_cmd[7];
                    std::copy(cmd_pos.begin(), cmd_pos.begin() + 7, pos_cmd);
                    servoJ(pos_cmd, dt, 0.1, 300.0, robot_ip_.c_str());
                    // servoJ_ex(pos_cmd, dt, 0.05, 500.0, false, robot_ip_.c_str());
                    break;
                }
                case 2: 
                {
                    auto &cmd_vel = command_.get<double>("velocity");
                    double vel_cmd[7];
                    std::copy(cmd_vel.begin(), cmd_vel.begin() + 7, vel_cmd);
                    speedJ(vel_cmd, 1.5, 0.0, robot_ip_.c_str());
                    break;
                }
                case 3: 
                {
                    auto &cmd_tau = command_.get<double>("torque");
                    double torque_cmd[7]; 
                    for(int i = 0; i < 7; i++)
                    {
                        torque_cmd[i] = cmd_tau[i];
                    }
                    sendTorque_rt(torque_cmd, 0.002, robot_ip_.c_str());
                    break;   
                }    
                case 4:
                {
                    auto &cmd_cart_pose = command_.get<double>("pose");
                    double target_pose[6]; 
                    std::copy(cmd_cart_pose.begin(), cmd_cart_pose.begin() + 6, target_pose);
                    servoL_ex(target_pose, 0.001, 0.05, 500, 1.0, false, nullptr, robot_ip_.c_str());
                    break;
                } 
                default:
                    stop(robot_ip_.c_str());
                    break;
            } 
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
            // RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, 
            //                       "Diana write() execution time: %ld us", duration_us);    
        }

        bool is_stop() override { return false; }

        void read(const rclcpp::Time& t, const rclcpp::Duration& period) override
        {
            auto start_time = std::chrono::high_resolution_clock::now();
            hardware_interface::RobotInterface::read(t, period);
            auto& q = state_.get<double>("position");
            auto& dq = state_.get<double>("velocity");
            auto& ddq = state_.get<double>("acceleration");
            auto& pose = state_.get<double>("pose");
            auto& pose_q_ = state_.get<double>("pose_q_");
            auto& torque = state_.get<double>("torque");
            auto& origin_torque = state_.get<double>("o_torque");
            // auto& force = state_.get<double>("force");


            auto dt = period.seconds();
            getJointPos(q.data(), robot_ip_.c_str());
            getJointAngularVel(dq.data(), robot_ip_.c_str());
            getTcpPos(pose.data(), robot_ip_.c_str());
            getJointTorque(torque.data(),robot_ip_.c_str());
            getOriginalJointTorque(origin_torque.data(),robot_ip_.c_str());
            // getTcpForce(force.data(),robot_ip_.c_str());
            forward(q.data(), pose_q_.data(), nullptr, robot_ip_.c_str());

            if (dt > 0) 
            {
                for (int i = 0; i < 7; i++) 
                {
                    ddq[i] = (dq[i] - pre_dq_[i]) / dt;
                    pre_dq_[i] = dq[i];
                }
            } 
            else 
            {
                pre_dq_ = dq;
                std::fill(ddq.begin(), ddq.end(), 0);
            }
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
            // RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, 
            //                       "torque: %f", torque.data());
            // RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, 
            //                       "o_torque: %f", origin_torque.data());
        }

        CallbackReturn on_configure(const rclcpp_lifecycle::State& prev) override
        {
            if (RobotInterface::on_configure(prev) != CallbackReturn::SUCCESS) 
                return CallbackReturn::FAILURE;
            node_->get_parameter_or<std::string>("robot_ip", robot_ip_, std::string(""));
            if (robot_ip_.empty()) 
            {
                RCLCPP_ERROR(node_->get_logger(), "robot_ip is not set");
                return CallbackReturn::FAILURE;
            }

            srv_net_ = std::make_unique<srv_net_st>();
            initSrvNetInfo(srv_net_.get());
            strcpy(srv_net_->SrvIp, robot_ip_.c_str());
            return initSrv(nullptr, nullptr, srv_net_.get()) == 0 ? 
                CallbackReturn::SUCCESS : CallbackReturn::FAILURE;
        }

        CallbackReturn on_shutdown(const rclcpp_lifecycle::State& prev) override
        {
            RobotInterface::on_shutdown(prev);
            if (!robot_ip_.empty()) 
            {               
                double zero_torque[7] = {0.0};
                sendTorque_rt(zero_torque, 0, robot_ip_.c_str());
                enableTorqueReceiver(false, robot_ip_.c_str());
                              
                changeControlMode(T_MODE_POSITION, robot_ip_.c_str());
                stop(robot_ip_.c_str());
                destroySrv(robot_ip_.c_str());
            }
            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_activate(const rclcpp_lifecycle::State& prev) override
        {
            if (RobotInterface::on_activate(prev) == CallbackReturn::SUCCESS)
            {
                cleanErrorInfo(robot_ip_.c_str()); 

                double joint_collision[7] = {200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0};
                double cart_collision[6] = {200.0, 200.0, 200.0, 200.0, 200.0, 200.0};
                setJointCollision(joint_collision, robot_ip_.c_str());
                setCartCollision(cart_collision, robot_ip_.c_str());
                
                // double payload[10] = {0.0};
                // setActiveTcpPayload(payload, robot_ip_.c_str());
                // int tcp_ret = setDefaultToolTcpCoordinate("zrz", robot_ip_.c_str());
                // if (tcp_ret == 0) {
                //     RCLCPP_INFO(node_->get_logger(), "Diana: Successfully set Tool TCP Coordinate to 'zrz'");
                // } else {
                //     // 如果名字拼写错误或控制器里没这个TCP，会返回 -1
                //     RCLCPP_ERROR(node_->get_logger(), "Diana: Failed to set Tool TCP Coordinate to 'zrz'");
                // }
                prev_mode_ = 1; 
                
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
            double zero_torque[7] = {0.0};
            sendTorque_rt(zero_torque, 0, robot_ip_.c_str());  
            enableTorqueReceiver(false, robot_ip_.c_str());
            
            std::fill(prev_torque_.begin(), prev_torque_.end(), 0.0);       
            stop(robot_ip_.c_str());
            return CallbackReturn::SUCCESS;
        }

    protected:
        std::string robot_ip_;
        std::unique_ptr<srv_net_st> srv_net_;
        std::vector<double> pre_dq_;

        std::vector<double> prev_torque_; 
        const std::vector<double> max_torque_{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0};
        const std::vector<double> friction_pos_ = {4.5, 2.8, 3.3, 4.0, 1.14, 1.37, 1.60};
        const std::vector<double> friction_neg_ = {3.5, 5.2, 3.3, 3.9, 1.41, 1.67, 1.55}; 

        int prev_mode_;
    };
} // namespace hardwares

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(hardwares::DIANARobot, hardware_interface::RobotInterface)