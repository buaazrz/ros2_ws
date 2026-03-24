#include "robot_controller_interface/controller_interface.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "realtime_tools/realtime_box.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "robot_math/MovingFilter.h"
#include "robot_math/robot_math.hpp"
#include "robot_math/CartesianTrajectory.hpp"
#include "ros2_utility/data_comm.hpp"
#include "ros2_utility/data_logger.hpp"
#include "ros2_utility/file_utils.hpp"
#include "robot_control_msgs/action/robot_motion.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <iostream>
#include "robot_controller_interface/controller_interface.hpp"
#include "ros2_utility/ros2_visual_tools.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

// using namespace robot_math;

namespace controllers
{
    class CartesianTrajImpPDController : public controller_interface::ControllerInterface
    {
    public:
        using ACTION = robot_control_msgs::action::RobotMotion;
        using GoalHandle = rclcpp_action::ServerGoalHandle<ACTION>;
        using BufferType = std::pair<std::shared_ptr<GoalHandle>, std::shared_ptr<robot_math::CartesianTrajectory>>;

        CartesianTrajImpPDController() {}
        ~CartesianTrajImpPDController() {}

        CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            dof_ = robot_->dof;
            
            node_->get_parameter_or<std::vector<double>>("Kx", Kx_vec_, {100.0, 100.0, 100.0, 50.0, 50.0, 50.0});
            node_->get_parameter_or<std::vector<double>>("Bx", Bx_vec_, {10.0, 10.0, 10.0, 5.0, 5.0, 5.0});
            node_->get_parameter_or<std::vector<double>>("Kn", Kn_vec_, {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0});
            node_->get_parameter_or<std::vector<double>>("Bn", Bn_vec_, {6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0});
            
            Kx_in_box_.set(Kx_vec_);
            Bx_in_box_.set(Bx_vec_);
            Kn_in_box_.set(Kn_vec_);
            Bn_in_box_.set(Bn_vec_);

            parameters_callback_handle_ = node_->add_on_set_parameters_callback(
                [&](std::vector<rclcpp::Parameter> parameters) -> rcl_interfaces::msg::SetParametersResult
                {
                    for (const auto &parameter : parameters)
                    {
                        if (parameter.get_name() == "Kx") Kx_in_box_.set([=](auto &value) { value = parameter.as_double_array(); });
                        else if (parameter.get_name() == "Bx") Bx_in_box_.set([=](auto &value) { value = parameter.as_double_array(); });
                        else if (parameter.get_name() == "Kn") Kn_in_box_.set([=](auto &value) { value = parameter.as_double_array(); });
                        else if (parameter.get_name() == "Bn") Bn_in_box_.set([=](auto &value) { value = parameter.as_double_array(); });
                    }
                    auto result = rcl_interfaces::msg::SetParametersResult();
                    result.successful = true;
                    return result;
                });
                
            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            time_ = 0;
            traj_time_ = 0;
            real_time_buffer_.reset();

            const std::vector<double> &q_vec = state_->get<double>("position");
            qd_ = Eigen::Map<const Eigen::VectorXd>(q_vec.data(), dof_).eval(); 
            dqd_ = Eigen::VectorXd::Zero(dof_);
            ddqd_ = Eigen::VectorXd::Zero(dof_);

            Eigen::Matrix4d Tb_tmp; 
            forward_kinematics(robot_, q_vec, Tb_tmp);
            Rd_ = Tb_tmp.block(0, 0, 3, 3);
            pd_ = Tb_tmp.block(0, 3, 3, 1);
            
            wd_ = Eigen::Vector3d::Zero();
            vd_ = Eigen::Vector3d::Zero();
            ddxd_ = Eigen::Vector6d::Zero();

            Thb_ = Eigen::Matrix6d::Identity();
            dThb_ = Eigen::Matrix6d::Zero();

            // 2. 初始化 Action Server (用于接收轨迹)
            auto handle_goal =[this](const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const ACTION::Goal> goal) {
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            };

            auto handle_cancel = [this](const std::shared_ptr<GoalHandle> goal_handle) {
                return rclcpp_action::CancelResponse::ACCEPT;
            };

            auto handle_accepted = [this](const std::shared_ptr<GoalHandle> goal_handle) {
                auto trajectory = std::make_shared<robot_math::CartesianTrajectory>();
                trajectory->set_traj(goal_handle->get_goal()->target_position.data);
                // 重置轨迹执行时间
                traj_time_ = 0.0; 
                real_time_buffer_.writeFromNonRT({goal_handle, trajectory});
            };

            call_back_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
            this->action_server_ = rclcpp_action::create_server<ACTION>(
                node_, "~/goal", handle_goal, handle_cancel, handle_accepted, 
                rcl_action_server_get_default_options(), call_back_group_);

            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            action_server_ = nullptr;
            return CallbackReturn::SUCCESS;
        }

        void update(const rclcpp::Time &t, const rclcpp::Duration &period) override
        {
            time_ += period.seconds();

            // 1. 获取当前状态与底层控制指令接口
            std::vector<double> &tau_cmd_vec = command_->get<double>("torque");
            command_->get<int>("mode")[0] = 3; // 设置底层硬件为力矩控制模式 (Torque Mode)

            const std::vector<double> &q_vec = state_->get<double>("position");
            const std::vector<double> &dq_vec = state_->get<double>("velocity");
            
            Eigen::Map<const Eigen::VectorXd> q(q_vec.data(), dof_);
            Eigen::Map<const Eigen::VectorXd> dq(dq_vec.data(), dof_);
            Eigen::Map<Eigen::VectorXd> tau_cmd(tau_cmd_vec.data(), dof_);
            
            if (q.hasNaN() || dq.hasNaN()) {
                RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, 
                    "🚨 灾难性错误：检测到关节状态为 NaN！已切断力矩输出以保护 CPU。");
                Eigen::Map<Eigen::VectorXd> tau_cmd(tau_cmd_vec.data(), dof_);
                tau_cmd.setZero(); // 输出零力矩
                return; // 直接退出，绝对不能让 NaN 矩阵进入后面的计算！
            }

            // 2. 更新阻抗参数
            Kx_in_box_.try_get([=](auto const &value) { Kx_vec_ = value; });
            Bx_in_box_.try_get([=](auto const &value) { Bx_vec_ = value; });
            Kn_in_box_.try_get([=](auto const &value) { Kn_vec_ = value; });
            Bn_in_box_.try_get([=](auto const &value) { Bn_vec_ = value; });
            Kx_ = Eigen::Map<Eigen::VectorXd>(Kx_vec_.data(), 6);
            Bx_ = Eigen::Map<Eigen::VectorXd>(Bx_vec_.data(), 6);
            Kn_ = Eigen::Map<Eigen::VectorXd>(Kn_vec_.data(), dof_);
            Bn_ = Eigen::Map<Eigen::VectorXd>(Bn_vec_.data(), dof_);

            // 3. 计算动力学与运动学矩阵 (M, C, g, J, 正运动学 Tb)
            m_c_g_matrix(robot_, q_vec, dq_vec, M_, C_, g_, Jb_, dJb_, dM_, dTb_, Tb_);
            R_ = Tb_.block(0, 0, 3, 3); // 实际姿态
            p_ = Tb_.block(0, 3, 3, 1); // 实际位置

            // ==================== 4. 轨迹插补层 (Trajectory Level) ====================
            auto handle_pair = *real_time_buffer_.readFromRT();
            auto goal_handle = handle_pair.first;
            auto trajectory = handle_pair.second;

            if (goal_handle && goal_handle->is_active())
            {
                if (goal_handle->is_canceling())
                {
                    auto result = std::make_shared<ACTION::Result>();
                    result->success = false;
                    goal_handle->canceled(result);
                    // 取消时，以当前期望位姿为目标，速度/加速度清零，相当于急停
                    vd_.setZero(); wd_.setZero(); ddxd_.setZero();
                }
                else
                {
                    traj_time_ += period.seconds();
                    Eigen::Matrix4d Td_curr;
                    Eigen::Vector6d Vd_curr, dVd_curr;

                    // 计算当前时刻的期望位姿、速度、加速度
                    trajectory->evaluate(traj_time_, Td_curr, Vd_curr, dVd_curr);

                    Rd_ = Td_curr.block(0, 0, 3, 3);
                    pd_ = Td_curr.block(0, 3, 3, 1);
                    wd_ = Vd_curr.head(3);  // 角速度
                    vd_ = Vd_curr.tail(3);  // 线速度
                    ddxd_ = dVd_curr;       // 六维加速度

                    Eigen::Vector3d pos_err = pd_ - p_;
                    Eigen::Vector3d rot_err = robot_math::logR(R_.transpose() * Rd_);
                    if (pos_err.norm() < 1e-2 && rot_err.norm() < 1e-3 && traj_time_ >= trajectory->total_time())
                    {
                        auto result = std::make_shared<ACTION::Result>();
                        result->success = true;
                        goal_handle->succeed(result);
                        vd_.setZero(); wd_.setZero(); ddxd_.setZero();
                    }
                }
            }
            else
            {
                // 没有激活的轨迹任务时，保持在最后设定的位置
                vd_.setZero(); wd_.setZero(); ddxd_.setZero();
            }

            // ==================== 5. 阻抗控制层 (Impedance Level) ====================
            
            // 雅可比坐标变换
            Thb_.block(3, 3, 3, 3) = R_;
            dThb_.block(3, 3, 3, 3) = dTb_.block(0, 0, 3, 3);
            Jh_ = Thb_ * Jb_;
            dJh_ = dThb_ * Jb_ + Thb_ * dJb_;

            // 计算任务空间误差 (位姿与速度误差)
            xe_.head(3) = robot_math::logR(R_.transpose() * Rd_); // 姿态对数映射误差
            xe_.tail(3) = pd_ - p_;                   // 位置误差
            dxe_.head(3) = R_.transpose() * wd_ - (Jh_ * dq).head(3);
            dxe_.tail(3) = vd_ - (Jh_ * dq).tail(3);

            double max_trans_err = 0.03; 
            if (xe_.tail(3).norm() > max_trans_err) {
                xe_.tail(3) = xe_.tail(3).normalized() * max_trans_err;
            }
            // 限制最大姿态误差为 ~5.7度 (0.1 rad)
            double max_rot_err = 0.1;
            if (xe_.head(3).norm() > max_rot_err) {
                xe_.head(3) = xe_.head(3).normalized() * max_rot_err;
            }

            // 任务空间阻抗控制律: 期望加速度 + 阻尼项 + 刚度项 - 雅可比导数补偿
            ddxc_ = ddxd_ + Bx_.asDiagonal() * dxe_ + Kx_.asDiagonal() * xe_ - dJh_ * dq;
            
            // 映射到关节力矩
            tau_task_ = M_ * robot_math::J_sharp(Jh_, M_) * ddxc_;

            qe_ = qd_ - q;
            dqe_ = dqd_ - dq;
            Eigen::LDLT<Eigen::MatrixXd> ldlt(M_);
            tau_null_ = M_ * robot_math::null_proj(Jh_, M_, ddqd_ + ldlt.solve(Bn_.asDiagonal() * dqe_ + Kn_.asDiagonal() * qe_));

            tau_cmd = tau_task_ + tau_null_ + C_*dq + g_;

        }

    protected:
        // ROS 2 通讯
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;
        rclcpp::CallbackGroup::SharedPtr call_back_group_;
        rclcpp_action::Server<ACTION>::SharedPtr action_server_;
        realtime_tools::RealtimeBuffer<BufferType> real_time_buffer_;

        // 控制器状态与数学变量
        int dof_;
        double time_, traj_time_;
        Eigen::MatrixXd M_, C_, Jb_, dJb_, dM_, Jh_, dJh_;
        Eigen::VectorXd g_;
        Eigen::Matrix4d Tb_, dTb_;
        Eigen::VectorXd Kx_, Bx_, Kn_, Bn_;
        Eigen::VectorXd tau_task_, tau_null_;
        Eigen::VectorXd qd_, dqd_, ddqd_, qe_, dqe_;
        Eigen::Vector6d xe_, dxe_, ddxd_, ddxc_;
        Eigen::Matrix3d Rd_, R_;
        Eigen::Matrix6d Thb_, dThb_;
        Eigen::Vector3d pd_, p_, wd_, vd_;

        // 线程安全的参数盒子
        realtime_tools::RealtimeBox<std::vector<double>> Kx_in_box_, Bx_in_box_, Kn_in_box_, Bn_in_box_;
        std::vector<double> Kx_vec_, Bx_vec_, Kn_vec_, Bn_vec_;
    };
} // namespace controllers

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(controllers::CartesianTrajImpPDController, controller_interface::ControllerInterface)