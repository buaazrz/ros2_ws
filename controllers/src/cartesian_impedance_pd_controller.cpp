#include "robot_controller_interface/controller_interface.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "realtime_tools/realtime_box.hpp"
#include "robot_math/robot_math.hpp"
#include <Eigen/Dense>

using namespace robot_math;

namespace controllers
{
    class CartesianAdmittanceController : public controller_interface::ControllerInterface
    {
    public:
        CartesianAdmittanceController() {}
        
        CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            dof_ = robot_->dof;
            
            // 导纳模型参数 (Mass, Damping, Stiffness)
            node_->get_parameter_or<std::vector<double>>("Ma", Ma_vec_, {2.0, 2.0, 2.0, 0.5, 0.5, 0.5});
            node_->get_parameter_or<std::vector<double>>("Ba", Ba_vec_, {40.0, 40.0, 40.0, 5.0, 5.0, 5.0});
            node_->get_parameter_or<std::vector<double>>("Ka", Ka_vec_, {100.0, 100.0, 100.0, 10.0, 10.0, 10.0});
            
            // 内环位置跟踪增益 (用于生成力矩)
            node_->get_parameter_or<std::vector<double>>("Kp", Kp_vec_, {200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0});
            node_->get_parameter_or<std::vector<double>>("Kd", Kd_vec_, {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0});

            Ma_in_box_.set(Ma_vec_);
            Ba_in_box_.set(Ba_vec_);
            Ka_in_box_.set(Ka_vec_);

            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            time_ = 0;
            // 初始化状态
            const std::vector<double> &q_vec = state_->get<double>("position");
            q_initial_ = Eigen::Map<const Eigen::VectorXd>(q_vec.data(), dof_);
            
            // 获取初始末端位姿作为期望位姿 Td
            Eigen::MatrixXd M_tmp, C_tmp, Jb_tmp, dJb_tmp, dM_tmp;
            Eigen::Matrix4d Tb_init, dTb_tmp;
            Eigen::VectorXd g_tmp;
            std::vector<double> dq_zero(dof_, 0.0);
            m_c_g_matrix(robot_, q_vec, dq_zero, M_tmp, C_tmp, g_tmp, Jb_tmp, dJb_tmp, dM_tmp, dTb_tmp, Tb_init);
            
            Td_ = Tb_init; // 设定当前点为固定目标点
            Rd_ = Td_.block(0,0,3,3);
            pd_ = Td_.block(0,3,3,1);

            // 导纳积分项初始化清零
            xe_.setZero();  // 位姿偏差 [rot_err, pos_err]
            dxe_.setZero(); // 偏差速度
            
            return CallbackReturn::SUCCESS;
        }

        void update(const rclcpp::Time & /*t*/, const rclcpp::Duration &period) override
        {
            double dt = period.seconds();
            time_ += dt;

            // 1. 获取传感器数据
            const std::vector<double> &q_vec = state_->get<double>("position");
            const std::vector<double> &dq_vec = state_->get<double>("velocity");
            const std::vector<double> &f_vec = state_->get<double>("wrench"); // 外部力/力矩传感器 [f, tau]
            std::vector<double> &tau_cmd_vec = command_->get<double>("torque");

            Eigen::Map<const Eigen::VectorXd> q(q_vec.data(), dof_);
            Eigen::Map<const Eigen::VectorXd> dq(dq_vec.data(), dof_);
            Eigen::Map<const Eigen::VectorXd> F_ext(f_vec.data(), 6);
            Eigen::Map<Eigen::VectorXd> tau_cmd(tau_cmd_vec.data(), dof_);

            // 2. 更新导纳模型参数
            Ma_in_box_.try_get([&](auto const &v){ Ma_ = Eigen::Map<const Eigen::VectorXd>(v.data(), 6); });
            Ba_in_box_.try_get([&](auto const &v){ Ba_ = Eigen::Map<const Eigen::VectorXd>(v.data(), 6); });
            Ka_in_box_.try_get([&](auto const &v){ Ka_ = Eigen::Map<const Eigen::VectorXd>(v.data(), 6); });

            // 3. 导纳核心逻辑：积分计算期望位移偏差 xe
            // 导纳方程: Ma*ddxe + Ba*dxe + Ka*xe = F_ext
            // 计算加速度 ddxe
            Eigen::Vector6d ddxe;
            for(int i=0; i<6; ++i) {
                ddxe(i) = (F_ext(i) - Ba_(i)*dxe_(i) - Ka_(i)*xe_(i)) / Ma_(i);
            }

            // 数值积分更新 dxe 和 xe
            dxe_ += ddxe * dt;
            xe_ += dxe_ * dt;

            // 4. 计算导纳修正后的目标位姿 (参考书中 8-51, 8-52)
            // 修正后的旋转矩阵 R_r 和位置 p_r
            Eigen::Vector3d re = xe_.head(3);
            Eigen::Vector3d pe = xe_.tail(3);

            Eigen::Matrix3d R_r = Rd_ * exp_w(-re); 
            Eigen::Vector3d p_r = pd_ - R_r * pe;

            // 5. 内环跟踪：将修正后的 (R_r, p_r) 转换为关节空间指令
            // 这里为了演示，采用简单的任务空间 PD + 动力学补偿，或者你可以调用逆运动学
            m_c_g_matrix(robot_, q_vec, dq_vec, M_, C_, g_, Jb_, dJb_, dM_, dTb_, Tb_);
            
            Eigen::Matrix3d R_curr = Tb_.block(0,0,3,3);
            Eigen::Vector3d p_curr = Tb_.block(0,3,3,1);

            // 计算跟踪误差
            Eigen::Vector6d error;
            error.head(3) = logR(R_curr.transpose() * R_r); // 姿态误差
            error.tail(3) = p_r - p_curr;                  // 位置误差

            // 映射到关节力矩 (简单示意的阻抗形式跟踪内环)
            Eigen::VectorXd Kp_inner = Eigen::Map<Eigen::VectorXd>(Kp_vec_.data(), dof_);
            Eigen::VectorXd Kd_inner = Eigen::Map<Eigen::VectorXd>(Kd_vec_.data(), dof_);

            // 任务空间反馈 + 动力学补偿 (C*dq + g)
            tau_cmd = Jb_.transpose() * (error * 100.0) - dq * 5.0 + C_ * dq + g_; 

            // 设置控制模式
            command_->get<int>("mode")[0] = 3; 
        }

    protected:
        int dof_;
        double time_;
        
        // 导纳模型参数
        Eigen::Vector6d Ma_, Ba_, Ka_;
        std::vector<double> Ma_vec_, Ba_vec_, Ka_vec_;
        realtime_tools::RealtimeBox<std::vector<double>> Ma_in_box_, Ba_in_box_, Ka_in_box_;

        // 导纳积分状态量
        Eigen::Vector6d xe_, dxe_; 

        // 初始参考位姿
        Eigen::Matrix4d Td_;
        Eigen::Matrix3d Rd_;
        Eigen::Vector3d pd_;
        Eigen::VectorXd q_initial_;

        // 机器人动力学矩阵
        Eigen::MatrixXd M_, C_, Jb_, dJb_, dM_;
        Eigen::VectorXd g_;
        Eigen::Matrix4d Tb_, dTb_;

        // 内环增益
        std::vector<double> Kp_vec_, Kd_vec_;
    };
} 

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(controllers::CartesianAdmittanceController, controller_interface::ControllerInterface)