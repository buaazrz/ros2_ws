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
        DIANARobot():pre_dq_(7),prev_torque_(7,0.0)
        {
            srand((unsigned int)time(nullptr));
        }
        ~DIANARobot()
        {
        }

        void write(const rclcpp::Time& t, const rclcpp::Duration& period) override
        {
            hardware_interface::RobotInterface::write(t, period);
            double max_single_delta = 1000.0; 
            auto &cmd_tau = command_.get<double>("torque");
            int dof = 7; 
            double torque_cmd[7]; 

            std::vector<double> temp_torque(cmd_tau.begin(), cmd_tau.end());                
            for (int i = 0; i < dof; i++) 
            {   
                // 增加安全防护：限制绝对最大扭矩 (解决你代码里定义了没用上的问题)
                temp_torque[i] = std::clamp(temp_torque[i], -max_torque_[i], max_torque_[i]);

                double delta = temp_torque[i] - prev_torque_[i];
                delta = std::clamp(delta, -max_single_delta, max_single_delta);
                temp_torque[i] = prev_torque_[i] + delta;
                prev_torque_[i] = temp_torque[i];
                torque_cmd[i] = temp_torque[i];
            }
            double dt = 1.0 / update_rate_;
            
            auto &mode = command_.get<int>("mode")[0]; 
            switch (mode)
            {
            case 3:
                {
                    // enableTorqueReceiver(true, robot_ip_.c_str());                            
                    sendTorque_rt(torque_cmd, dt, robot_ip_.c_str());
                    // sleep(dt);
                    // 更新历史扭矩和首次发送标志
                    // std::copy(torque_cmd, torque_cmd + dof, prev_torque_.begin());
                    break;   
                }     
            default:
                break;
            }
                // 仅发送扭矩，绝不 sleep，绝不 enableTorqueReceiver
                // sendTorque_rt(torque_cmd, period.seconds(), robot_ip_.c_str());
                // sleep(dt);
  
        }

        bool is_stop() override { return false; }

        void read(const rclcpp::Time& t, const rclcpp::Duration& period) override
        {
            // 原读取逻辑不变，确保关节位置、速度、外力等数据正确反馈给PD+控制器
            hardware_interface::RobotInterface::read(t, period);
            auto& q = state_.get<double>("position");
            auto& dq = state_.get<double>("velocity");
            auto& ddq = state_.get<double>("acceleration");
            auto& pose = state_.get<double>("pose");
            auto& torque = state_.get<double>("torque");

            auto dt = period.seconds();

            getJointPos(q.data());
            getJointAngularVel(dq.data());
            getTcpPos(pose.data());
            getJointTorque(torque.data());
            
            
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
                // 关机时下发零扭矩，确保安全
                double zero_torque[7] = {0.0};
                sendTorque_rt(zero_torque, 0, robot_ip_.c_str());
                              
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

                // 碰撞阈值、阻抗参数、负载配置保持原逻辑
                double joint_collision[7] = {200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0};
                double cart_collision[6] = {200.0, 200.0, 200.0, 200.0, 200.0, 200.0};
                setJointCollision(joint_collision, robot_ip_.c_str());
                setCartCollision(cart_collision, robot_ip_.c_str());
                
                double payload[10] = {0.0};
                setActiveTcpPayload(payload, robot_ip_.c_str());
                
                // double joint_stiff[7] = {1000, 1000, 1000, 1000, 500, 500, 500};
                // double cart_stiff[6] = {1000, 1000, 1000, 500, 500, 500};
                // setJointImpeda(joint_stiff, 0.3, robot_ip_.c_str());
                // setCartImpeda(cart_stiff, 0.3, robot_ip_.c_str());
                
                std::fill(prev_torque_.begin(), prev_torque_.end(), 0.0);   

                //enableTorqueReceiver(true, robot_ip_.c_str());
                releaseBrake(robot_ip_.c_str());
                rclcpp::sleep_for(std::chrono::seconds(2));

                enableTorqueReceiver(true, robot_ip_.c_str()); 

                return CallbackReturn::SUCCESS;
            }
            return CallbackReturn::FAILURE;
        }

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State& prev) override
        {
            RobotInterface::on_deactivate(prev);
            // 停用控制器时下发零扭矩
            double zero_torque[7] = {0.0};
            sendTorque_rt(zero_torque, 0, robot_ip_.c_str());  
            std::fill(prev_torque_.begin(), prev_torque_.end(), 0.0);       
            enableTorqueReceiver(false, robot_ip_.c_str());
            stop(robot_ip_.c_str());
            return CallbackReturn::SUCCESS;
        }

    protected:
        std::string robot_ip_;
        std::unique_ptr<srv_net_st> srv_net_;
        std::vector<double> pre_dq_;

        std::vector<double> prev_torque_;  // 上一次发送的扭矩
        const std::vector<double> min_torque_{0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};  // 最小有效扭矩
        const std::vector<double> max_torque_{30.0, 30.0, 30.0, 20.0, 20.0, 20.0, 20.0};  // 最大有效扭矩
    };
} // namespace hardwares

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(hardwares::DIANARobot, hardware_interface::RobotInterface)