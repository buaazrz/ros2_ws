#include "robot_controller_interface/controller_interface.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "realtime_tools/realtime_box.hpp"
#include "robot_math/MovingFilter.h"
#include "robot_math/robot_math.hpp"
// #include "ros2_utility/data_comm.hpp"
#include "ros2_utility/data_logger.hpp"
#include "ros2_utility/file_utils.hpp"
#include <iostream>

using namespace robot_math;
namespace controllers
{
    class CartesianDualLoopAdmittanceController : public controller_interface::ControllerInterface
    {
    public:
        CartesianDualLoopAdmittanceController() {}
        ~CartesianDualLoopAdmittanceController()
        {
            // if (data_logger_)
                // data_logger_->save(FileUtils::getHomeDirectory() + "/experiment_logs/cartesian_impedance_pdplus_controller/", "cartesian_impedance_pdplus_controller");
        }

        CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            dof_ = robot_->dof;
            node_->get_parameter_or<std::vector<double>>("Kx", Kx_vec_, {10.0, 10.0, 10.0, 100.0, 100.0, 100.0});
            node_->get_parameter_or<std::vector<double>>("Bx", Bx_vec_, {10.0, 10.0, 10.0, 10.0, 10.0, 10.0});
            node_->get_parameter_or<std::vector<double>>("Kn", Kn_vec_, {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0});
            node_->get_parameter_or<std::vector<double>>("Bn", Bn_vec_, {6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0});
            node_->get_parameter_or<std::vector<double>>("M", M_vec_, {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
            node_->get_parameter_or<std::vector<double>>("K", K_vec_, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
            node_->get_parameter_or<std::vector<double>>("B", B_vec_, {20.0, 20.0, 20.0, 20.0, 20.0, 20.0});
            if (B_vec_.empty() || B_vec_.size() != 6)
            {
                B_vec_.resize(6);
                for (size_t i = 0; i < 6; ++i)
                    B_vec_[i] = 2.0 * std::sqrt(K_vec_[i]);
            } 
            node_->get_parameter_or<double>("Y", Y_gain_, 1.0);
            Y_ = Y_gain_ * Eigen::MatrixXd::Identity(dof_, dof_);         
            Kx_in_box_.set(Kx_vec_);
            Bx_in_box_.set(Bx_vec_);
            Kn_in_box_.set(Kn_vec_);
            Bn_in_box_.set(Bn_vec_);
            M_in_box_.set(M_vec_);
            K_in_box_.set(K_vec_);
            B_in_box_.set(B_vec_);


            parameters_callback_handle_ = node_->add_on_set_parameters_callback(
                [&](std::vector<rclcpp::Parameter> parameters) -> rcl_interfaces::msg::SetParametersResult
                {
                    RCLCPP_INFO(node_->get_logger(), "Parameter %s update requested.", parameters[0].get_name().c_str());
                    for (const auto &parameter : parameters)
                    {
                        if (parameter.get_name() == "Kx")
                            Kx_in_box_.set([=](auto &value)
                                           { value = parameter.as_double_array(); });
                        else if (parameter.get_name() == "Bx")
                            Bx_in_box_.set([=](auto &value)
                                           { value = parameter.as_double_array(); });
                        else if (parameter.get_name() == "Kn")
                            Kn_in_box_.set([=](auto &value)
                                           { value = parameter.as_double_array(); });
                        else if (parameter.get_name() == "Bn")
                            Bn_in_box_.set([=](auto &value)
                                           { value = parameter.as_double_array(); });
                        else if (parameter.get_name() == "M")
                            M_in_box_.set([=](auto &value)
                                           { value = parameter.as_double_array(); });
                        else if (parameter.get_name() == "K")
                            K_in_box_.set([=](auto &value)
                                           { value = parameter.as_double_array(); });  
                        else if (parameter.get_name() == "B")
                            B_in_box_.set([=](auto &value)
                                           { value = parameter.as_double_array(); });                 
                    }
                    auto result = rcl_interfaces::msg::SetParametersResult();
                    result.successful = true;
                    return result;
                });
            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
        {
            // DataComm::getInstance()->setDestAddress("127.0.0.1", 7755);
            time_ = 0;
            const std::vector<double> &q_vec = state_->get<double>("position");
            qd_ = Eigen::Map<const Eigen::VectorXd>(q_vec.data(), dof_).eval();
            initial_q_ = qd_;
            dqd_ = Eigen::VectorXd::Zero(dof_);
            ddqd_ = Eigen::VectorXd::Zero(dof_);

            // const std::vector<double> &T_vec = state_->get<double>("T");
            // Eigen::Matrix4d T = Eigen::Map<const Eigen::Matrix4d>(T_vec.data(), 4, 4).eval();
            Eigen::Matrix4d Tb_tmp; 
            // 直接调用库中的正运动学函数
            forward_kinematics(robot_, q_vec, Tb_tmp);

            Rd_fixed_ = Tb_tmp.block(0, 0, 3, 3);
            pd_fixed_ = Tb_tmp.block(0, 3, 3, 1);

            // Rd_ = T.block(0, 0, 3, 3);
            // pd_ = T.block(0, 3, 3, 1);
            Rd_ = Rd_fixed_;
            pd_ = pd_fixed_;
            wd_.setZero();
            vd_.setZero();
            ddxd_.setZero();

            // 初始化导纳误差状态（初始为零，表示无偏差）
            re_.setZero();
            red_.setZero();
            pe_.setZero();
            ped_.setZero();

            // 初始化干扰估计
            td_ = Eigen::VectorXd::Zero(dof_);

            Thb_ = Eigen::Matrix6d::Identity();
            dThb_ = Eigen::Matrix6d::Zero();

            q_ = Eigen::VectorXd::Zero(dof_);
            dq_ = Eigen::VectorXd::Zero(dof_);
            tau_task_ = Eigen::VectorXd::Zero(dof_);
            tau_null_ = Eigen::VectorXd::Zero(dof_);
            tau_cmd_ = Eigen::VectorXd::Zero(dof_);
            data_logger_ = std::make_unique<DataLogger>(
                std::initializer_list<DataInfo>{
                    DATA_WRAPPER(time_),
                    DATA_WRAPPER(success_rate_),
                    // DATA_WRAPPER(cal_time_),
                    DATA_WRAPPER(q_),
                    DATA_WRAPPER(dq_),
                    DATA_WRAPPER(xe_),
                    DATA_WRAPPER(dxe_),
                    DATA_WRAPPER(tau_task_),
                    DATA_WRAPPER(tau_null_),
                },
                std::initializer_list<ExperimentContext>{
                    CONFIG_WRAPPER(Kx_vec_),
                    CONFIG_WRAPPER(Bx_vec_),
                    CONFIG_WRAPPER(Kn_vec_),
                    CONFIG_WRAPPER(Bn_vec_),
                },
                1000);
            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
        {
            return CallbackReturn::SUCCESS;
        }

        void update(const rclcpp::Time &t, const rclcpp::Duration &period) override
        {
            double dt = period.seconds();
            time_ += dt;

            auto start_time = std::chrono::high_resolution_clock::now();
            // std::vector<double> &tau_cmd_vec = command_->get<double>("torque");
            const std::vector<double> &q_vec = state_->get<double>("position");
            const std::vector<double> &dq_vec = state_->get<double>("velocity");
            success_rate_ = state_->get<double>("success")[0];
            const std::vector<double> &f_ext_vec = state_->get<double>("f_ext");
            Eigen::Vector6d f_ext = Eigen::Vector6d::Zero();
            if (f_ext_vec.size() == 6) {
                f_ext = Eigen::Map<const Eigen::Vector6d>(f_ext_vec.data());
            }

            // Eigen::Map<Eigen::VectorXd> tau_cmd(tau_cmd_vec.data(), dof_);
            Eigen::Map<const Eigen::VectorXd> q(q_vec.data(), dof_);
            Eigen::Map<const Eigen::VectorXd> dq(dq_vec.data(), dof_);

            q_ = q;
            dq_ = dq;

            Kx_in_box_.try_get([=](auto const &value)
                               { Kx_vec_ = value; });
            Bx_in_box_.try_get([=](auto const &value)
                               { Bx_vec_ = value; });
            Kn_in_box_.try_get([=](auto const &value)
                               { Kn_vec_ = value; });
            Bn_in_box_.try_get([=](auto const &value)
                               { Bn_vec_ = value; });
            M_in_box_.try_get([=](auto const &value)
                               { M_vec_ = value; });
            B_in_box_.try_get([=](auto const &value)
                               { B_vec_ = value; });
            K_in_box_.try_get([=](auto const &value)
                               { K_vec_ = value; });

            Kx_ = Eigen::Map<Eigen::VectorXd>(Kx_vec_.data(), 6);
            Bx_ = Eigen::Map<Eigen::VectorXd>(Bx_vec_.data(), 6);
            Kn_ = Eigen::Map<Eigen::VectorXd>(Kn_vec_.data(), dof_);
            Bn_ = Eigen::Map<Eigen::VectorXd>(Bn_vec_.data(), dof_);
            M_ = Eigen::Map<Eigen::VectorXd>(M_vec_.data(), 6);
            K_ = Eigen::Map<Eigen::VectorXd>(K_vec_.data(), 6);
            B_ = Eigen::Map<Eigen::VectorXd>(B_vec_.data(), 6);

            m_c_g_matrix(robot_, q_vec, dq_vec, M_mat_, C_mat_, g_, Jb_, dJb_, dM_, dTb_, Tb_);

            // 当前末端位姿
            R_ = Tb_.block(0, 0, 3, 3);
            p_ = Tb_.block(0, 3, 3, 1);

            // 构建任务雅可比 Jh_
            Thb_.setIdentity();
            Thb_.block(3, 3, 3, 3) = R_;
            dThb_.setZero();
            dThb_.block(3, 3, 3, 3) = dTb_.block(0, 0, 3, 3);
            Jh_ = Thb_ * Jb_;
            dJh_ = dThb_ * Jb_ + Thb_ * dJb_;

            // ---------- 导纳外环（零刚度，仅质量-阻尼）----------
            // 外力取负（根据定义，f_ext 可能是环境施加给机器人的力）
            Eigen::Vector6d F_ext = -f_ext;   // 机器人施加给环境的力

            // 误差加速度计算（刚度K=0）
            Eigen::Vector3d redd = (F_ext.head(3) - B_.head(3).cwiseProduct(red_)).cwiseQuotient(M_.head(3));
            Eigen::Vector3d pedd = (F_ext.tail(3) - B_.tail(3).cwiseProduct(ped_)).cwiseQuotient(M_.tail(3));

            // 积分更新误差状态（半隐式欧拉）
            red_ += redd * dt;
            re_ += red_ * dt;
            ped_ += pedd * dt;
            pe_ += ped_ * dt;

            // 修正期望位姿：从固定期望 Rd_fixed_, pd_fixed_ 出发，叠加误差
            Eigen::Matrix3d R = Rd_fixed_ * exp_r(-re_);            // R = Rd_fixed * exp(-re)
            Eigen::Vector3d p = pd_fixed_ - R * pe_;                // p = pd_fixed - R * pe

            // 修正期望速度（由于定点控制，原始期望速度为零，但误差变化产生速度）
            Eigen::Matrix3d A = A_r(re_);
            Eigen::Matrix3d dA = dA_r(re_, red_);
            Eigen::Vector3d w = - Rd_fixed_ * A * red_;            // wd_fixed=0
            Eigen::Vector3d alpha = - Rd_fixed_ * A * (redd + (A.inverse() * dA) * (A.inverse() * Rd_fixed_.transpose()) * (-w)) + w.cross(w); // 简化：wd_fixed=0
            // 更简洁的角加速度（忽略高阶项，但为准确可使用上述公式）
            // 实际中，如果刚度为零且运动缓慢，可近似 alpha = -Rd_fixed * A * redd
            // 此处保留完整公式以匹配 MATLAB

            Eigen::Vector3d v = - R * ped_ - w.cross(pd_fixed_ - p);
            Eigen::Vector3d a = - R * pedd - alpha.cross(pd_fixed_ - p) - 2.0 * w.cross(v) + w.cross(w.cross(pd_fixed_ - p));

            // 更新内环期望值
            Rd_ = R;
            pd_ = p;
            wd_ = w;
            vd_ = v;
            ddxd_.head(3) = alpha;
            ddxd_.tail(3) = a;

            // ---------- 内环位置控制（计算力矩 + 零空间 + 干扰观测器）----------
            // 任务空间误差（相对于修正后的期望）
            xe_.head(3) = logR(R_.transpose() * Rd_);
            xe_.tail(3) = pd_ - p_;
            dxe_.head(3) = R_.transpose() * wd_ - (Jh_ * dq_).head(3);
            dxe_.tail(3) = vd_ - (Jh_ * dq_).tail(3);

            // 期望加速度补偿（与之前相同）
            Eigen::Vector6d ddxc = ddxd_ + A_x_inv(Jh_, M_mat_) *
                                             (Mu_x_X(Jh_, M_mat_, dJh_, C_mat_, dxe_) +
                                              Bx_.asDiagonal() * dxe_ +
                                              Kx_.asDiagonal() * xe_) -
                                             dJh_ * dq_;

            // 任务空间力矩
            tau_task_ = M_mat_ * J_sharp_X(Jh_, M_mat_, ddxc);

            // 零空间力矩（简化：期望关节位置 qd_ 取固定值，可通过逆运动学得到）
            // 这里为了简化，假设 qd_ 为初始关节位置，且保持恒定（即零空间刚度使关节趋向初始构型）
            // 实际中可能需要实时逆解，但零重力拖动通常不要求精确零空间位置，此处用 qd_ = initial_q_
            Eigen::VectorXd qe_des = initial_q_ - q_;
            Eigen::VectorXd dqe_des = -dq_;
            Eigen::LDLT<Eigen::MatrixXd> ldlt(M_mat_);
            tau_null_ = M_mat_ * null_proj(Jh_, M_mat_,
                                            ddqd_ + ldlt.solve(Bn_.asDiagonal() * dqe_des + Kn_.asDiagonal() * qe_des));

            // 干扰观测器
            Eigen::VectorXd P = Y_ * dq_;
            Eigen::VectorXd td_est = td_ + P;
            Eigen::MatrixXd Jb = Jh_;
            Eigen::MatrixXd Lambda_inv = Jb * M_mat_.inverse() * Jb.transpose();
            // td_est = Jb.transpose() * Lambda_inv.inverse() * (Jb * M_mat_.inverse() * td_est);
            td_est.setZero();
            Eigen::VectorXd tau_cmd = tau_task_ + tau_null_ + C_mat_ * dq_ - td_est + g_;

            // 更新干扰估计
            Eigen::VectorXd td_dot = Y_ * (M_mat_.inverse() * (C_mat_ * dq_ + g_ - tau_cmd - P - td_));
            td_.setZero();
            td_ += td_dot * dt;

            // 输出指令
            std::vector<double> &tau_cmd_vec = command_->get<double>("torque");
            Eigen::Map<Eigen::VectorXd>(tau_cmd_vec.data(), dof_) = tau_cmd;
            command_->get<int>("mode")[0] = 3; // 力矩模式

            // 记录数据
            data_logger_->record();
        }

            // log2Channel(robot_data_, 0, xe_.data(), 6);
            // log2Channel(robot_data_, 1, dxe_.data(), 6);
            // log2Channel(robot_data_, 2, tau_task_.data(), 6);
            // log2Channel(robot_data_, 3, tau_null_.data(), dof_);
            // robot_data_.t = time_;
            // DataComm::getInstance()->sendRobotStatus(robot_data_);
            // cal_time_ = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
            // data_logger_->record();

    protected:
        // 参数回调句柄
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;
        // 实时参数
        realtime_tools::RealtimeBox<std::vector<double>> Kx_in_box_, Bx_in_box_, Kn_in_box_, Bn_in_box_, M_in_box_, B_in_box_, K_in_box_;
        std::vector<double> Kx_vec_, Bx_vec_, Kn_vec_, Bn_vec_, M_vec_, B_vec_, K_vec_;
        // 机器人维度和动力学
        int dof_;
        Eigen::MatrixXd M_mat_, C_mat_, Jb_, dJb_, dM_, Jh_, dJh_;
        Eigen::VectorXd g_, q_, dq_;
        Eigen::Matrix4d Tb_, dTb_;
        // 内环增益
        Eigen::VectorXd Kx_, Bx_, Kn_, Bn_;
        // 导纳参数
        Eigen::Vector6d M_, K_, B_;
        // 导纳误差状态
        Eigen::Vector3d re_, red_, pe_, ped_;
        // 定点期望位姿（固定）
        Eigen::Matrix3d Rd_fixed_;
        Eigen::Vector3d pd_fixed_;
        Eigen::VectorXd initial_q_; 
        // 期望轨迹（内环参考）
        Eigen::Matrix3d Rd_, R_;
        Eigen::Vector3d pd_, p_;
        Eigen::Vector3d wd_, vd_;
        Eigen::Vector6d ddxd_;
        // 干扰观测器
        Eigen::VectorXd td_;
        Eigen::MatrixXd Y_;
        double Y_gain_;

        Eigen::VectorXd tau_cmd_, tau_task_, tau_null_;
        Eigen::VectorXd qd_, dqd_, ddqd_, qe_, dqe_;
        Eigen::Vector6d xe_, dxe_, ddxc_;
        Eigen::Matrix6d Thb_, dThb_;
        double success_rate_, cal_time_;
        
        std::unique_ptr<DataLogger> data_logger_;
        double time_;
        // RobotData robot_data_;
    };
} // namespace controllers

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(controllers::CartesianDualLoopAdmittanceController, controller_interface::ControllerInterface)