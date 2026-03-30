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
            hardware_interface::RobotInterface::write(t, period);
            
            // RCLCPP_INFO(node_->get_logger(), "当前周期(秒): %.6f，周期(毫秒): %.3f", period.seconds(), period.seconds() * 1000.0);
            
            double dt = 1.0 / update_rate_;
            int mode = command_.get<int>("mode")[0]; 

            switch (mode)
            {
                case 0: // 0: 笛卡尔空间 姿态控制 (Pose)
                {
                    auto &cmd_pose = command_.get<double>("pose");
                    double pose_cmd[6];
                    std::copy(cmd_pose.begin(), cmd_pose.begin() + 6, pose_cmd);
                    
                    // API: pose, time(dt), look_ahead_time(0.05), gain(500), scale(1.0), active_tcp
                    servoL(pose_cmd, dt, 0.05, 500.0, 1.0, nullptr, robot_ip_.c_str());
                    break;
                }
                case 1: // 1: 关节空间 位置控制 (Position)
                {
                    auto &cmd_pos = command_.get<double>("position");
                    double pos_cmd[7];
                    std::copy(cmd_pos.begin(), cmd_pos.begin() + 7, pos_cmd);
                    
                    // API: joints, time(dt), look_ahead_time(0.05), gain(500)
                    servoJ(pos_cmd, dt, 0.05, 500.0, robot_ip_.c_str());
                    break;
                }
                case 2: // 2: 关节空间 速度控制 (Velocity)
                {
                    auto &cmd_vel = command_.get<double>("velocity");
                    double vel_cmd[7];
                    std::copy(cmd_vel.begin(), cmd_vel.begin() + 7, vel_cmd);
                    
                    // API: speed, acceleration(1.5), t(0表示不提前减速)
                    speedJ(vel_cmd, 1.5, 0.0, robot_ip_.c_str());
                    break;
                }
                case 3: // 3: 关节空间 力矩控制 (Torque)
                {
                    enableTorqueReceiver(true, robot_ip_.c_str());
                    auto &cmd_tau = command_.get<double>("torque");
                    double torque_cmd[7]; 
                    // for(int i = 0; i < 7; i++)
                    // {
                    //     torque_cmd[i] = std::clamp(cmd_tau[i], -max_torque_[i], max_torque_[i]);
                    // }
                    for(int i = 0; i < 7; i++)
                    {
                        torque_cmd[i] = cmd_tau[i];
                    }
                    // for (int i = 0; i < 7; i++) 
                    // {   
                    //     double temp_tau = std::clamp(cmd_tau[i], -max_torque_[i], max_torque_[i]);

                    //     double delta = temp_tau - prev_torque_[i];
                    //     delta = std::clamp(delta, -max_single_delta, max_single_delta);
                        
                    //     torque_cmd[i] = prev_torque_[i] + delta;
                    //     prev_torque_[i] = torque_cmd[i];
                    // }
                    sendTorque_rt(torque_cmd, dt, robot_ip_.c_str());
            //         RCLCPP_INFO(get_node()->get_logger(), "torque_cmd: %.6f %.6f %.6f %.6f %.6f %.6f %.6f Nm", 
            // torque_cmd[0], torque_cmd[1], torque_cmd[2], torque_cmd[3],torque_cmd[4], torque_cmd[5],torque_cmd[6]);

                    break;   
                }    
                case 4:
                {
                    changeControlMode(T_MODE_CART_IMPEDANCE, robot_ip_.c_str());
                    auto &cmd_cart_pose = command_.get<double>("pose");
                    double target_pose[6]; 
                    // 拷贝控制器生成的轨迹：[X,Y,Z,Rx,Ry,Rz]
                    std::copy(cmd_cart_pose.begin(), cmd_cart_pose.begin() + 6, target_pose);
                    
                    // 3. 调用官方API：阻抗模式下跟踪轨迹
                    servoL_ex(target_pose, 0.001, 0.05, 500, 1.0, false, nullptr, robot_ip_.c_str());
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
            hardware_interface::RobotInterface::read(t, period);
            auto& q = state_.get<double>("position");
            auto& dq = state_.get<double>("velocity");
            auto& ddq = state_.get<double>("acceleration");
            auto& pose = state_.get<double>("pose");
            auto& torque = state_.get<double>("torque");


            auto dt = period.seconds();
            // 

            getJointPos(q.data(), robot_ip_.c_str());
            getJointAngularVel(dq.data(), robot_ip_.c_str());
            getTcpPos(pose.data(), robot_ip_.c_str());
            // getJointTorque(torque.data(), robot_ip_.c_str());
            
            // RCLCPP_INFO(get_node()->get_logger(), "torque: %.6f %.6f %.6f %.6f %.6f %.6f %.6f Nm", 
            // torque[0], torque[1], torque[2], torque[3],torque[4], torque[5],torque[6]);
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
                // 关机时下发零扭矩并关闭力矩接收器，确保切断动力安全
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
                
                double payload[10] = {0.0};
                setActiveTcpPayload(payload, robot_ip_.c_str());
                
                std::fill(prev_torque_.begin(), prev_torque_.end(), 0.0);   
                std::fill(pre_dq_.begin(), pre_dq_.end(), 0.0);

                releaseBrake(robot_ip_.c_str());
                // enableTorqueReceiver(true, robot_ip_.c_str());

                rclcpp::sleep_for(std::chrono::seconds(2));

                return CallbackReturn::SUCCESS;
            }
            return CallbackReturn::FAILURE;
        }

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State& prev) override
        {
            RobotInterface::on_deactivate(prev);
            // 停用控制器时下发零扭矩并关闭力矩接收模式
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

        std::vector<double> prev_torque_;  // 上一次发送的扭矩
        // const std::vector<double> min_torque_{4, 3, 4, 4.5, 1.5, 1.5, 1.5};        // 
        const std::vector<double> max_torque_{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0};  // 最大安全扭矩边界
        const std::vector<double> friction_pos_ = {4.5, 2.8, 3.3, 4.0, 1.14, 1.37, 1.60}; // 正向死区
        const std::vector<double> friction_neg_ = {3.5, 5.2, 3.3, 3.9, 1.41, 1.67, 1.55}; // 负向死区 (取绝对值)
    };
} // namespace hardwares

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(hardwares::DIANARobot, hardware_interface::RobotInterface)