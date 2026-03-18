#include "robot_hardware_interface/robot_interface.hpp"
#include <iostream>
#include <vector>

namespace hardwares
{
    class SimulationRobot : public hardware_interface::RobotInterface
    {
    public:
        SimulationRobot()
        {
        }
        void read(const rclcpp::Time &t, const rclcpp::Duration &period) override
        {
            hardware_interface::RobotInterface::read(t, period);
            // auto &force = com_state_["ft_sensor"]->get<double>("force");
            // std::cerr << "raw force: " << force[0] << " " << force[1] << " " << force[2] << " " << force[3] << " " << force[4] << " " << force[5] << std::endl;
                
        }
        void write(const rclcpp::Time &t, const rclcpp::Duration &period) override
        {
            // RCLCPP_INFO(rclcpp::get_logger("SimulationRobot"), "===== SimulationRobot::write 函数被调用 =====");
            // std::cerr << "[SimulationRobot] Logger initialized successfully!" << std::endl;
            
            hardware_interface::RobotInterface::write(t, period);
            static int debug_count = 0;
            if (++debug_count >= 500) 
            {
                auto mode_val = command_.get<int>("mode")[0];
                // 只用 printf 或 std::cerr 简单打印，且每 1 秒才打印一次
                std::cerr << "[SimulationRobot] write() is running! Current mode: " << mode_val << std::endl;
                debug_count = 0;
            }

            auto &cmd = command_.get<double>();
            auto &state = state_.get<double>();
            auto mode = command_.get<int>("mode")[0];
            // RCLCPP_INFO(rclcpp::get_logger("SimulationRobot"), "当前mode原始值：%d", mode);
            if (mode == 0) // cartisan space
            {
                std::vector<double> jt;
                if (inverse_kinematics(state["position"], robot_math::pose_to_tform(cmd["pose"]), jt))
                {
                    if (period.seconds() > 0)
                        for (int i = 0; i < dof_; i++)
                            state["velocity"][i] = (jt[i] - state["position"][i]) / period.seconds();
                    state["position"] = jt;
                }
                else
                    throw std::runtime_error("ik error");
            }
            else if (mode == 1) // joint space
            {
                if (period.seconds() > 0)
                    for (int i = 0; i < dof_; i++)
                        // state["velocity"][i] = (cmd["position"][i] - state["position"][i]) / period.seconds();
                state["position"] = cmd["position"];
                // RCLCPP_INFO(node_->get_logger(), "mode==1，进入位置控制模式");

                // state["velocity"] = cmd["velocity"];
            }
            else if (mode == 2) // velocity
            {
                state["velocity"] = cmd["velocity"];
                // RCLCPP_INFO(node_->get_logger(), "mode==2，进入速度控制模式");

                for (int i = 0; i < dof_; i++)
                    state["position"][i] += cmd["velocity"][i] * period.seconds();
            }
            else if (mode == 3) // torque 控制
            {
                // 1. 获取引用，避免多次查找
                auto &cmd_torque = cmd["torque"];
                auto &pos = state["position"];
                auto &vel = state["velocity"];

                // 2. 构造状态向量 x: [q, dq]
                std::vector<double> x;
                x.reserve(dof_ * 2);
                x.insert(x.end(), pos.begin(), pos.end());
                x.insert(x.end(), vel.begin(), vel.end());

                // 3. 初始化 dx (size 将在 robot_dynamics 内部被 resize 为 2*n)
                std::vector<double> dx; 

                // 获取连杆数量，通常是 dof_ + 1 (base + links)
                // 如果不确定，可以填一个足够大的数（如 10），但最标准的是从 robot_ 对象获取
                int num_segments = dof_ + 1; 

                // 4. 调用动力学解算
                robot_dynamics(
                    x, 
                    dx, 
                    t.seconds(), 
                    // 修正点 1：返回 6 行，num_segments 列的零矩阵
                    [num_segments](double) { 
                        return Eigen::MatrixXd::Zero(6, num_segments); 
                    }, 
                    // 修正点 2：直接返回命令中的扭矩
                    [&](double, const std::vector<double>&, const Eigen::MatrixXd&) { 
                        return cmd_torque; 
                    }
                );

                // 5. 积分更新状态
                double dt = period.seconds();
                if (dt > 0 && dx.size() >= (size_t)(2 * dof_)) 
                {
                    for (int i = 0; i < dof_; i++)
                    {
                        // 根据 robot_dynamics 的实现：
                        // dx[i] 是速度 dq
                        // dx[dof_ + i] 是计算出的加速度 ddq
                        double acc = dx[dof_ + i];
                        vel[i] += acc * dt;        // v = v + a*dt
                        pos[i] += vel[i] * dt;     // p = p + v*dt
                    }
                }

                // 6. 赋值当前扭矩状态（用于监控）
                state["torque"] = cmd_torque;

                // 修正点 3：严禁在此处直接打印日志。
                // 如果非要打印，请使用之前定义的 debug_count 逻辑
                if (debug_count == 0) {
                    // std::cerr << "[SimulationRobot] Torque mode active, acc[0]: " << dx[dof_] << std::endl;
                }
            }
        }
        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override
        {
            std::cerr << "[SimulationRobot] Logger initialized successfully!" << std::endl;

            if (RobotInterface::on_activate(previous_state) == CallbackReturn::SUCCESS)
            {
                // to do
                // std::fill(pre_dq_.begin(), pre_dq_.end(), 0);

                return CallbackReturn::SUCCESS;
            }
            return CallbackReturn::FAILURE;
        }

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override
        {
            RobotInterface::on_deactivate(previous_state);
            std::cerr << "[SimulationRobot] Logger initialized successfully!" << std::endl;

            auto dq = state_.get<double>("velocity");
            std::fill(dq.begin(), dq.end(), 0);
            return CallbackReturn::SUCCESS;
        }
    };

} // namespace hardwares

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(hardwares::SimulationRobot, hardware_interface::RobotInterface)