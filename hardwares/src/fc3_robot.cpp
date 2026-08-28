#include "robot_hardware_interface/robot_interface.hpp"
#include "robot_math/robot_math.hpp"
#include <franka/active_torque_control.h>
#include <franka/active_control_base.h>
#include <franka/active_motion_generator.h>
#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <iostream>
#include <vector>
#include <Eigen/Dense> // 添加 Eigen 依赖以处理矩阵映射

using namespace robot_math;
namespace hardwares
{
    class FC3Robot : public hardware_interface::RobotInterface
    {
    public:
        FC3Robot() : prev_mode_(3)
        {
        }
        ~FC3Robot()
        {
        }
        void write(const rclcpp::Time &t, const rclcpp::Duration &period) override
        {
            auto start_time = std::chrono::high_resolution_clock::now();
            hardware_interface::RobotInterface::write(t, period);

            // 安全获取 mode，默认为上一周期的模式
            int mode = command_.get<int>("mode")[0];

            if (mode != prev_mode_)
            {
                try
                {
                    // 必须先停止当前的 1kHz 控制流，释放旧的通讯通道
                    if (control_)
                    {
                        franka_robot_->stop();
                        control_ = nullptr;
                    }

                    // 启动新的控制流通道
                    if (mode == 0) // 0: 笛卡尔空间位置/位姿控制
                    {
                        RCLCPP_INFO(node_->get_logger(), "Franka: Switched to Cartesian Pose Mode");
                        control_ = franka_robot_->startCartesianPoseControl(
                            research_interface::robot::Move::ControllerMode::kCartesianImpedance);
                    }
                    else if (mode == 1) // 1: 关节空间位置控制
                    {
                        RCLCPP_INFO(node_->get_logger(), "Franka: Switched to Joint Position Mode");
                        control_ = franka_robot_->startJointPositionControl(
                            research_interface::robot::Move::ControllerMode::kJointImpedance);
                    }
                    else if (mode == 3) // 3: 力矩控制
                    {
                        RCLCPP_INFO(node_->get_logger(), "Franka: Switched to Torque Mode");
                        control_ = franka_robot_->startTorqueControl();
                    }
                    prev_mode_ = mode;
                }
                catch (const franka::Exception &e)
                {
                    RCLCPP_ERROR(node_->get_logger(), "Franka Mode Switch Error: %s", e.what());
                    return; // 切换失败直接返回
                }
            }

            if (!control_)
                return;

            try
            {
                switch (mode)
                {
                case 0: // 笛卡尔空间位置控制
                {
                    auto &cmd_pose = command_.get<double>("pose"); // 上层下发的 6D 位姿 [x, y, z, rx, ry, rz]

                    // 将 6D 位姿转为 4x4 齐次变换矩阵
                    Eigen::Matrix4d T_d = robot_math::pose_to_tform(cmd_pose);

                    // Franka 需要的是长度为 16 的一维列主序数组
                    std::array<double, 16> pose_array;
                    // Eigen 默认是列主序(Column-major)，完美映射到 std::array
                    Eigen::Map<Eigen::Matrix4d>(pose_array.data()) = T_d;

                    franka::CartesianPose cartesian_pose(pose_array);
                    control_->writeOnce(cartesian_pose);
                    break;
                }
                case 1: // 关节空间位置控制
                {
                    auto &cmd = command_.get<double>("position");
                    franka::JointPositions positions{cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], cmd[6]};
                    control_->writeOnce(positions);
                    break;
                }
                case 3: // 力矩控制
                {
                    auto &cmd = command_.get<double>("torque");
                    franka::Torques torques{cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], cmd[6]};
                    control_->writeOnce(torques);
                    break;
                }
                }
            }
            catch (const franka::Exception &e)
            {
                RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                      "Franka Write Error: %s", e.what());
            }
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
            RCLCPP_INFO(node_->get_logger(), "Write time: %ld us", duration_us);
        }

        bool is_stop() override
        {
            return false;
        }

        void read(const rclcpp::Time &t, const rclcpp::Duration &period) override
        {
            auto start_time = std::chrono::high_resolution_clock::now();
            hardware_interface::RobotInterface::read(t, period);
            if (!control_)
                return;

            try
            {
                auto &&state = control_->readOnce().first;
                // auto success_rate = state.control_command_success_rate;
                // auto &&mass = franka_model_->mass(state);
                // auto &&coriolis = franka_model_->coriolis(state);
                // auto &&gravity = franka_model_->gravity(state);
                auto &&T = state.O_T_EE;
                // auto &&tau_d = state.tau_J_d;
                // auto &&ext_tau = state.tau_ext_hat_filtered;
                // auto &force = com_state_["ft_sensor"]->get<double>("force");

                // state_.get<double>("success")[0] = success_rate;
                // std::copy(mass.begin(), mass.end(), state_.get<double>("m").begin());
                // std::copy(ext_tau.begin(), ext_tau.end(), state_.get<double>("external_torque").begin());
                // std::copy(tau_d.begin(), tau_d.end(), state_.get<double>("torque").begin());
                std::copy(T.begin(), T.end(), state_.get<double>("T").begin());
                std::copy(state.q.begin(), state.q.end(), state_.get<double>("position").begin());
                std::copy(state.dq.begin(), state.dq.end(), state_.get<double>("velocity").begin());
                // std::copy(coriolis.begin(), coriolis.end(), state_.get<double>("c").begin());
                // std::copy(gravity.begin(), gravity.end(), state_.get<double>("g").begin());
            }
            catch (const franka::Exception &e)
            {
                RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                      "Franka Read Error: %s", e.what());
            }
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
            RCLCPP_INFO(node_->get_logger(), "Read Time: %ld us", duration_us);
        }

        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override
        {
            if (RobotInterface::on_configure(previous_state) == CallbackReturn::SUCCESS)
            {
                node_->get_parameter_or<std::string>("robot_ip", robot_ip_, "");
                if (robot_ip_.empty())
                {
                    RCLCPP_ERROR(node_->get_logger(), "robot_ip is not set");
                    return CallbackReturn::FAILURE;
                }
                try
                {
                    franka_robot_ = std::make_shared<franka::Robot>(robot_ip_);
                }
                catch (std::exception &e)
                {
                    RCLCPP_ERROR(node_->get_logger(), "can not establish connection with FC3 robot with %s", robot_ip_.c_str());
                    return CallbackReturn::FAILURE;
                }
                return CallbackReturn::SUCCESS;
            }

            return CallbackReturn::FAILURE;
        }

        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override
        {
            RobotInterface::on_shutdown(previous_state);
            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override
        {
            if (RobotInterface::on_activate(previous_state) == CallbackReturn::SUCCESS)
            {
                try
                {
                    franka_robot_->automaticErrorRecovery();
                    franka_robot_->setCollisionBehavior(
                        {{200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0}}, {{200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0}},
                        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                        {{200.0, 200.0, 200.0, 200.0, 200.0, 200.0}}, {{200.0, 200.0, 200.0, 200.0, 200.0, 200.0}},
                        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

                    // franka_robot_->setLoad(0.2067, {0, 0, 0.029}, {1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6});
                    franka_robot_->setJointImpedance({{3000, 3000, 3000, 2500, 2500, 200, 200}});
                    franka_robot_->setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});

                    prev_mode_ = 3;
                    control_ = franka_robot_->startTorqueControl();
                    franka_model_ = std::make_shared<franka::Model>(franka_robot_->loadModel());

                    return CallbackReturn::SUCCESS;
                }
                catch (const franka::Exception &e)
                {
                    RCLCPP_ERROR(node_->get_logger(), "Franka Activate Error: %s", e.what());
                    return CallbackReturn::FAILURE;
                }
            }
            return CallbackReturn::FAILURE;
        }

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override
        {
            if (control_)
            {
                franka_robot_->stop();
                control_ = nullptr;
            }
            RobotInterface::on_deactivate(previous_state);
            return CallbackReturn::SUCCESS;
        }

    protected:
        std::string robot_ip_;
        std::shared_ptr<franka::Robot> franka_robot_;
        std::unique_ptr<franka::ActiveControlBase> control_;
        std::shared_ptr<franka::Model> franka_model_;

        int prev_mode_;
    };

} // namespace hardwares

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(hardwares::FC3Robot, hardware_interface::RobotInterface)