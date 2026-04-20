#include "robot_controller_interface/controller_interface.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "realtime_tools/realtime_box.hpp"
#include "robot_math/MovingFilter.h"
#include "robot_math/robot_math.hpp"
#include "ros2_utility/data_comm.hpp"
#include "ros2_utility/data_logger.hpp"
#include "ros2_utility/file_utils.hpp"
#include <iostream>

using namespace robot_math;
namespace controllers
{
    class CartesianImpedancePDController : public controller_interface::ControllerInterface
    {
    public:
        CartesianImpedancePDController() {}
        ~CartesianImpedancePDController()
        {
            if (data_logger_)
                data_logger_->save(FileUtils::getHomeDirectory() + "/experiment_logs/cartesian_impedance_pd_controller/", "cartesian_impedance_pd_controller");
        }
        Eigen::VectorXd saturate_torque(const Eigen::VectorXd &tau_d_calculated, const Eigen::VectorXd &tau_J_d, double tol = 1.0)
        {
            Eigen::VectorXd tau_d_saturated(dof_);
            for (int i = 0; i < dof_; i++)
            {
                double difference = tau_d_calculated[i] - tau_J_d[i];
                tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, tol), -tol);
            }
            return tau_d_saturated;
        }

        CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            dof_ = robot_->dof;
            node_->get_parameter_or<std::vector<double>>("Kx", Kx_vec_, {10.0, 10.0, 10.0, 100.0, 100.0, 100.0});
            node_->get_parameter_or<std::vector<double>>("Bx", Bx_vec_, {10.0, 10.0, 10.0, 10.0, 10.0, 10.0});
            node_->get_parameter_or<std::vector<double>>("Kn", Kn_vec_, {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0});
            node_->get_parameter_or<std::vector<double>>("Bn", Bn_vec_, {6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0});
            Kx_in_box_.set(Kx_vec_);
            Bx_in_box_.set(Bx_vec_);
            Kn_in_box_.set(Kn_vec_);
            Bn_in_box_.set(Bn_vec_);
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
                    }
                    auto result = rcl_interfaces::msg::SetParametersResult();
                    result.successful = true;
                    return result;
                });
            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
        {
            tau_d.setZero();
            time_ = 0;
            const std::vector<double> &q_vec = state_->get<double>("position");
            qd_ = Eigen::Map<const Eigen::VectorXd>(q_vec.data(), dof_).eval();
            dqd_ = Eigen::VectorXd::Zero(dof_);
            ddqd_ = Eigen::VectorXd::Zero(dof_);

            const std::vector<double> &pose = state_->get<double>("pose");
            Eigen::Matrix4d T = robot_math::pose_to_tform(pose);
            Eigen::Matrix4d Tb_tmp; 
            robot_math::forward_kinematics(robot_, q_vec, Tb_tmp);
            Rd_ = Tb_tmp.block(0, 0, 3, 3);
            pd_ = Tb_tmp.block(0, 3, 3, 1);

            wd_ = Eigen::Vector3d::Zero();
            vd_ = Eigen::Vector3d::Zero();
            ddxd_ = Eigen::Vector6d::Zero();

            q_ = Eigen::VectorXd::Zero(dof_);
            dq_ = Eigen::VectorXd::Zero(dof_);
            tau_task_ = Eigen::VectorXd::Zero(dof_);
            tau_null_ = Eigen::VectorXd::Zero(dof_);
            tau_cmd_ = Eigen::VectorXd::Zero(dof_);
            data_logger_ = std::make_unique<DataLogger>(
                std::initializer_list<DataInfo>{
                    DATA_WRAPPER(time_),
                    DATA_WRAPPER(cal_time_),
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
            time_ += period.seconds();
            auto start_time = std::chrono::high_resolution_clock::now();
            command_->get<int>("mode")[0] = 3; 

            std::vector<double> &tau_cmd_vec = command_->get<double>("torque");
            const std::vector<double> &q_vec = state_->get<double>("position");
            const std::vector<double> &dq_vec = state_->get<double>("velocity");
            const std::vector<double> &pose_vec = state_->get<double>("pose");

            Eigen::Map<Eigen::VectorXd> tau_cmd(tau_cmd_vec.data(), dof_);
            Eigen::Map<const Eigen::VectorXd> q(q_vec.data(), dof_);
            Eigen::Map<const Eigen::VectorXd> dq(dq_vec.data(), dof_);

            Kx_in_box_.try_get([=](auto const &value)
                               { Kx_vec_ = value; });
            Bx_in_box_.try_get([=](auto const &value)
                               { Bx_vec_ = value; });
            Kn_in_box_.try_get([=](auto const &value)
                               { Kn_vec_ = value; });
            Bn_in_box_.try_get([=](auto const &value)
                               { Bn_vec_ = value; });
            Kx_ = Eigen::Map<Eigen::VectorXd>(Kx_vec_.data(), 6);
            Bx_ = Eigen::Map<Eigen::VectorXd>(Bx_vec_.data(), 6);
            Kn_ = Eigen::Map<Eigen::VectorXd>(Kn_vec_.data(), dof_);
            Bn_ = Eigen::Map<Eigen::VectorXd>(Bn_vec_.data(), dof_);

            m_c_g_matrix(robot_, q_vec, dq_vec, M_, C_, g_, Jb_, dJb_, dM_, dTb_, Tb_);
            R_ = Tb_.block(0, 0, 3, 3);
            p_ = Tb_.block(0, 3, 3, 1);
            Eigen::Vector3d w = (Jb_ * dq).head(3);
            Eigen::Vector3d v = (Jb_ * dq).tail(3);

            xe_.head(3) = robot_math::logR(R_.transpose() * Rd_);
            xe_.tail(3) = R_.transpose() * (pd_ - p_);
            dxe_.head(3) = R_.transpose() * wd_ - w;
            dxe_.tail(3) = R_.transpose() * vd_ - v;

            ddxc_ = ddxd_ + robot_math::A_x_inv(Jb_, M_) * (robot_math::Mu_x_X(Jb_, M_, dJb_, C_, dxe_) + Bx_.asDiagonal() * dxe_ + Kx_.asDiagonal() * xe_);
            tau_task_ = M_ * robot_math::J_sharp(Jb_, M_)*(ddxc_- dJb_* dq);
            Eigen::LDLT<Eigen::MatrixXd> ldlt(M_);
            tau_null_ = M_ * robot_math::null_proj(Jb_, M_, ldlt.solve(Bn_.asDiagonal() * (-dq)));

            tau_cmd = tau_task_ + tau_null_ + C_ * dq;
            tau_cmd = saturate_torque(tau_cmd, tau_d);
            tau_d = tau_cmd;

            cal_time_ = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
            data_logger_->record();
        }

    protected:
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;
        int dof_;
        Eigen::MatrixXd M_, C_, Jb_, dJb_, dM_, Jh_, dJh_;
        Eigen::VectorXd g_, q_, dq_;
        Eigen::Matrix4d Tb_, dTb_;
        Eigen::VectorXd Kx_, Bx_, Kn_, Bn_;
        Eigen::VectorXd tau_cmd_, tau_task_, tau_null_;
        Eigen::VectorXd qd_, dqd_, ddqd_, qe_, dqe_;
        Eigen::Vector6d xe_, dxe_, ddxd_, ddxc_;
        Eigen::Matrix3d Rd_, R_;
        Eigen::Vector3d pd_, p_, wd_, vd_;
        double cal_time_;
        realtime_tools::RealtimeBox<std::vector<double>> Kx_in_box_, Bx_in_box_, Kn_in_box_, Bn_in_box_;
        std::vector<double> Kx_vec_, Bx_vec_, Kn_vec_, Bn_vec_;
        std::unique_ptr<DataLogger> data_logger_;
        double time_;
        Eigen::Vector7d tau_d;
    };
} // namespace controllers

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(controllers::CartesianImpedancePDController, controller_interface::ControllerInterface)