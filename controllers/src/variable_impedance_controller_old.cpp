#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "robot_controller_interface/controller_interface.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "realtime_tools/realtime_box.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "robot_math/MovingFilter.h"
#include "robot_math/robot_math.hpp"
#include "robot_math/CartesianTrajectory.hpp"
#include "ros2_utility/data_logger.hpp"
#include "ros2_utility/file_utils.hpp"
#include "math.h"
#include "robot_control_msgs/action/robot_motion.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <iostream>
#include "ros2_utility/ros2_visual_tools.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "robot_control_msgs/msg/robot_state.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include <fstream>
#include <iomanip>
#include "optimization.h"
#include <deque>
#include <numeric>

// 定义传给 ALGLIB 目标函数回调的动态参数结构体
struct OptParams
{
    double Q_weight;
    double R_weight;
    double xe;
    double dx_B;     // 相当于阻尼力 D_c * dx_r
    double F_target; // 期望力 F_des + F_com
    double K_min;
};

// ALGLIB 目标函数与梯度计算回调
static void alglib_grad_callback(const alglib::real_1d_array &x, double &func, alglib::real_1d_array &grad, void *ptr)
{
    // 解析传入的参数
    OptParams *params = static_cast<OptParams *>(ptr);
    double K = x[0];
    double Q = params->Q_weight;
    double R = params->R_weight;
    double xe = params->xe;
    double dx_B = params->dx_B;
    double F_target = params->F_target;
    double K_min = params->K_min;

    double F_ext_est = K * xe - dx_B;
    
    double force_err = F_ext_est - F_target;

    // 代价函数 J = 0.5 * Q * (F_ext - F_target)^2 + 0.5 * R * (K - K_min)^2
    func = 0.5 * Q * force_err * force_err + 0.5 * R * (K - K_min) * (K - K_min);

    // 对 K 求偏导计算梯度
    grad[0] = Q * xe * force_err + R * (K - K_min);
}

// using namespace robot_math;

namespace controllers
{
    class VariableImpedanceControllerold : public controller_interface::ControllerInterface
    {
    public:
        using ACTION = robot_control_msgs::action::RobotMotion;
        using GoalHandle = rclcpp_action::ServerGoalHandle<ACTION>;
        using BufferType = std::pair<std::shared_ptr<GoalHandle>, std::shared_ptr<robot_math::CartesianTrajectory>>;

        VariableImpedanceControllerold() : f_filter_(6, 15) , t_filter_(7, 15) {}
        ~VariableImpedanceControllerold()
        {
            if (data_logger_)
                data_logger_->save("/home/wjc/experiment_logs/variable_imp_controller/", "variable_imp_controller");
        }

        CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            dof_ = robot_->dof;

            node_->get_parameter_or<std::vector<double>>("Kx", Kx_vec_, {500.0, 500.0, 500.0, 2500.0, 2500.0, 2500.0});
            node_->get_parameter_or<std::vector<double>>("Bx", Bx_vec_, {30.0, 30.0, 30.0, 150.0, 150.0, 150.0});
            node_->get_parameter_or<std::vector<double>>("Kn", Kn_vec_, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
            node_->get_parameter_or<std::vector<double>>("Bn", Bn_vec_, {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
            node_->get_parameter_or<double>("Q_weight", Q_weight_, 100000.0);
            node_->get_parameter_or<double>("R_weight", R_weight_, 0.1);
            node_->get_parameter_or<double>("F_des_z", F_des_z_, -2.0);
            node_->get_parameter_or<double>("F_max_z", F_max_z_, 10.0);
            node_->get_parameter_or<double>("Kp_f", Kp_f_, 0.0);
            node_->get_parameter_or<double>("Ki_f", Ki_f_, 0.0);
            node_->get_parameter_or<double>("Kd_f", Kd_f_, 0.0);
            node_->get_parameter_or<double>("Kz_min", Kz_min_, 100.0);
            node_->get_parameter_or<double>("Kz_max", Kz_max_, 2500.0);
            node_->get_parameter_or<double>("dob_gain", dob_gain_val_, 0.0);

            Kx_in_box_.set(Kx_vec_);
            Bx_in_box_.set(Bx_vec_);
            Kn_in_box_.set(Kn_vec_);
            Bn_in_box_.set(Bn_vec_);

            parameters_callback_handle_ = node_->add_on_set_parameters_callback(
                [&](std::vector<rclcpp::Parameter> parameters) -> rcl_interfaces::msg::SetParametersResult
                {
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

        CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            time_ = 0;
            traj_time_ = 0;
            force_int_z_ = 0.0;
            force_err_z_prev_ = 0.0;
            force_err_window_.clear();
            is_contact_established_ = false; // 重置接触状态

            real_time_buffer_.reset();
            tau_d.setZero();
            f_filter_.reset();
            t_filter_.reset();
            F_imp_.setZero();

            z_ = Eigen::VectorXd::Zero(dof_);
            tau_d_est_ = Eigen::VectorXd::Zero(dof_);
            tau_x_est_ = Eigen::VectorXd::Zero(dof_);
            Y_ = Eigen::VectorXd::Constant(dof_, dob_gain_val_);

            tau_fric_ff_ = Eigen::VectorXd::Zero(dof_); 
            tau_fric_ff_prev_ = Eigen::VectorXd::Zero(dof_);

            robot_state_publisher_ = node_->create_publisher<robot_control_msgs::msg::RobotState>("robot_states", rclcpp::SensorDataQoS());
            real_time_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<robot_control_msgs::msg::RobotState>>(robot_state_publisher_);

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
            tau_task_ = Eigen::VectorXd::Zero(dof_);
            tau_null_ = Eigen::VectorXd::Zero(dof_);
            tau_cmd_ = Eigen::VectorXd::Zero(dof_);

            sensor_weight_ =  0.61583;
            sensor_cog_vec_ = {    0.0001  , 0.000  ,  0.0029};
            sensor_offset_vec_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            T_sensor_ = Eigen::Matrix4d::Identity();
            T_sensor_ << 1, 0, 0, 0,
                0, -1, 0, 0,
                0, 0, -1, 0,
                0, 0, 0, 1;

            x_opt_.setlength(1);
            bndl_.setlength(1);
            bndu_.setlength(1);
            c_.setlength(2, 2);
            ct_.setlength(2);
            scale_.setlength(1);

            bndl_[0] = Kz_min_;
            bndu_[0] = Kz_max_;
            ct_[0] = 1;
            ct_[1] = -1;
            scale_[0] = 1.0;

            // 执行一次初始建档 (黑盒生成) 
            x_opt_[0] = Kx_vec_[5];
            alglib::minbleiccreate(x_opt_, alglib_state_);
            alglib::minbleicsetbc(alglib_state_, bndl_, bndu_);
            alglib::minbleicsetscale(alglib_state_, scale_);
            alglib::minbleicsetcond(alglib_state_, 0.0, 0.0, 1e-4, 10); // maxits=10

            auto handle_goal = [this](const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const ACTION::Goal> goal)
            {
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            };

            auto handle_cancel = [this](const std::shared_ptr<GoalHandle> goal_handle)
            {
                return rclcpp_action::CancelResponse::ACCEPT;
            };

            auto handle_accepted = [this](const std::shared_ptr<GoalHandle> goal_handle)
            {
                auto trajectory = std::make_shared<robot_math::CartesianTrajectory>();
                const auto &goal_data = goal_handle->get_goal()->target_position.data;
                trajectory->set_traj(goal_data);

                traj_time_ = 0.0;
                is_contact_established_ = false; // 接收新目标时重置接触状态
                force_int_z_ = 0.0; // 重置积分项
                force_err_z_prev_ = 0.0; // 重置微分项历史
                force_err_window_.clear();
                real_time_buffer_.writeFromNonRT({goal_handle, trajectory});
            };

            call_back_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
            this->action_server_ = rclcpp_action::create_server<ACTION>(
                node_, "~/goal", handle_goal, handle_cancel, handle_accepted,
                rcl_action_server_get_default_options(), call_back_group_);

            pose_ = Eigen::Vector6d::Zero();
            force_ = Eigen::Vector6d::Zero();
            dq_ = Eigen::Vector7d::Zero();
            q_ = Eigen::Vector7d::Zero();
            data_logger_ = std::make_unique<DataLogger>(
                std::initializer_list<DataInfo>{
                    DATA_WRAPPER(time_),
                    DATA_WRAPPER(cal_time_),
                    DATA_WRAPPER(params_.F_target),
                    DATA_WRAPPER(F_imp_(5)), 
                    DATA_WRAPPER(force_(2)),
                    // DATA_WRAPPER(pose_),
                    DATA_WRAPPER(tau_d),
                    DATA_WRAPPER(Kx_(5)),
                    DATA_WRAPPER(Bx_(5)),
                    DATA_WRAPPER(tau_fric_ff_),
                    DATA_WRAPPER(dq_), 
                    // DATA_WRAPPER(tau_base_),
                    // DATA_WRAPPER(xe_),
                    // DATA_WRAPPER(tau_d_est_),
                    // DATA_WRAPPER(tau_x_est_),
                    // DATA_WRAPPER(tau_null_),
                    // DATA_WRAPPER(xe_),
                    // DATA_WRAPPER(dxe_),
                    // DATA_WRAPPER(ddxd_),
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

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            action_server_ = nullptr;
            real_time_publisher_ = nullptr;
            robot_state_publisher_ = nullptr;
            return CallbackReturn::SUCCESS;
        }

        void update(const rclcpp::Time &t, const rclcpp::Duration &period) override
        {
            time_ += period.seconds();
            double dt = period.seconds();

            auto start_time = std::chrono::high_resolution_clock::now();

            std::vector<double> &tau_cmd_vec = command_->get<double>("torque");
            const std::vector<double> &q_vec = state_->get<double>("position");
            const std::vector<double> &dq_vec = state_->get<double>("velocity");
            const std::vector<double> &pose_vec = state_->get<double>("pose");
            auto &force_vec = com_state_->at("ft_sensor")->get<double>("force");
            command_->get<int>("mode")[0] = 3;
            pose_ = Eigen::Map<const Eigen::Vector6d>(pose_vec.data());
            force_ = Eigen::Map<const Eigen::Vector6d>(force_vec.data());
            dq_ = Eigen::Map<const Eigen::Vector7d>(dq_vec.data());
            q_ = Eigen::Map<const Eigen::Vector7d>(q_vec.data());
            Eigen::Matrix4d T = robot_math::pose_to_tform(pose_vec);
            // R_ = T.block(0, 0, 3, 3);
            // p_ = T.block(0, 3, 3, 1);

            Eigen::Map<const Eigen::VectorXd> q(q_vec.data(), dof_);
            Eigen::Map<const Eigen::VectorXd> dq(dq_vec.data(), dof_);
            Eigen::Map<const Eigen::VectorXd> force(force_vec.data(), 6);
            Eigen::Map<Eigen::VectorXd> tau_cmd(tau_cmd_vec.data(), dof_);
            std::fill(tau_cmd_vec.begin(), tau_cmd_vec.end(), 0);

            

            // Kx_in_box_.try_get([=](auto const &value)
            //                    { Kx_vec_ = value; });
            // Bx_in_box_.try_get([=](auto const &value)
            //                    { Bx_vec_ = value; });
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
            Eigen::Matrix4d T1;
            robot_math::forward_kinematics(robot_, q_vec, T1);

            Eigen::Vector6d raw_compensated = robot_math::get_ext_force(
                force_vec,
                sensor_weight_,
                sensor_offset_vec_,
                sensor_cog_vec_,
                T_sensor_,
                T1);
            force_.head(3) = raw_compensated.tail(3);
            force_.tail(3) = raw_compensated.head(3);
            Eigen::Matrix3d R_tcp_to_sensor = T_sensor_.block<3, 3>(0, 0);
            force_.head(3) = R_tcp_to_sensor * force_.head(3);
            force_.tail(3) = R_tcp_to_sensor * force_.tail(3);

            f_filter_.filtering(force_.data(), force_.data());
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                 "Filtered Force/Torque: [Fx: %.3f, Fy: %.3f, Fz: %.3f, Mx: %.3f, My: %.3f, Mz: %.3f]",
                                 force_[0], force_[1], force_[2], force_[3], force_[4], force_[5]);

            auto handle_pair = *real_time_buffer_.readFromRT();
            auto goal_handle = handle_pair.first;
            auto trajectory = handle_pair.second;

            dVd.setZero();

            if (goal_handle && goal_handle->is_active())
            {
                if (goal_handle->is_canceling())
                {
                    auto result = std::make_shared<ACTION::Result>();
                    result->success = false;
                    goal_handle->canceled(result);
                    vd_.setZero();
                    wd_.setZero();
                    ddxd_.setZero();
                }
                else
                {
                    traj_time_ += period.seconds();
                    Eigen::Matrix4d Td_curr;
                    Eigen::Vector6d Vd_curr, dVd_curr;

                    trajectory->evaluate(traj_time_, Td_curr, Vd_curr, dVd_curr);

                    Rd_ = Td_curr.block(0, 0, 3, 3);
                    pd_ = Td_curr.block(0, 3, 3, 1);
                    wd_ = Vd_curr.head(3);
                    vd_ = Vd_curr.tail(3);
                    dVd = dVd_curr;

                    Eigen::Vector3d pos_err = pd_ - p_;
                    Eigen::Vector3d rot_err = robot_math::logR(R_.transpose() * Rd_);
                    if (pos_err.norm() < 1e-3 && rot_err.norm() < 1e-2 && traj_time_ >= trajectory->total_time())
                    {
                        auto result = std::make_shared<ACTION::Result>();
                        result->success = true;
                        goal_handle->succeed(result);
                        vd_.setZero();
                        wd_.setZero();
                        ddxd_.setZero();
                    }
                }
            }
            else
            {
                vd_.setZero();
                wd_.setZero();
                ddxd_.setZero();
            }

            Eigen::Vector3d w = (Jb_ * dq).head(3);
            Eigen::Vector3d v = (Jb_ * dq).tail(3);

            xe_.head(3) = robot_math::logR(R_.transpose() * Rd_);
            xe_.tail(3) = R_.transpose() * (pd_ - p_);
            dxe_.head(3) = R_.transpose() * wd_ - w;
            dxe_.tail(3) = R_.transpose() * vd_ - v;
            ddxd_.head(3) = R_.transpose() * (dVd.head(3) - (R_ * w).cross(wd_));
            ddxd_.tail(3) = R_.transpose() * (dVd.tail(3) - (R_ * w).cross(vd_));

            double xe_z = xe_(5); 
            double dxe_z = dxe_(5);
            double B_z = Bx_(5);

            if (!is_contact_established_) {
                if (std::abs(force_(2)) >= std::abs(F_des_z_)) {
                    is_contact_established_ = true;
                    force_int_z_ = 0.0; 
                    force_err_z_prev_ = 0.0;
                    RCLCPP_WARN(node_->get_logger(), "Z轴接触力到达阈值: %.2f N, 开始变刚度模式!", force_(2));
                }
            }

            if (is_contact_established_ && std::abs(xe_z) > 1e-3 && xe_z * F_des_z_ > 0) // 只有当位置误差不小且力误差与位置误差同号时才优化刚度
            {
                double F_ext_z = force_(2);
                double F_err_z = F_des_z_ -(- F_ext_z);
                double current_integral_step = F_err_z * dt;
                force_err_window_.push_back(current_integral_step);
                force_int_z_ += current_integral_step; 
                if (force_err_window_.size() > static_cast<size_t>(integral_window_size_)) {
                    force_int_z_ -= force_err_window_.front();
                    force_err_window_.pop_front();
                }
                // force_int_z_ += F_err_z * dt;
                // std::cerr << "F_err_z: " << F_err_z << std::endl;
                double dF_err_z = 0.0;
                if (dt > 1e-6) {
                    if (force_err_z_prev_ == 0.0) {
                        dF_err_z = 0.0;
                    } else {
                        dF_err_z = (F_err_z - force_err_z_prev_) / dt;
                    }
                }
                double F_com_z = Kp_f_ * F_err_z + Ki_f_ * force_int_z_ + Kd_f_ * dF_err_z;
                force_err_z_prev_ = F_err_z;

                params_.Q_weight = Q_weight_;
                params_.R_weight = R_weight_;
                params_.xe = xe_z;
                params_.dx_B = -B_z * dxe_z;
                params_.F_target = F_des_z_ + F_com_z;
                params_.K_min = Kz_min_; 

                try
                {
                    x_opt_[0] = Kx_(5);

                    c_[0][0] = xe_z;
                    c_[0][1] = -F_max_z_ + params_.dx_B;
                    c_[1][0] = xe_z;
                    c_[1][1] = F_max_z_ + params_.dx_B;

                    alglib::minbleicsetlc(alglib_state_, c_, ct_);
                    alglib::minbleicrestartfrom(alglib_state_, x_opt_);
                    alglib::minbleicoptimize(alglib_state_, alglib_grad_callback, NULL, &params_);
                    alglib::minbleicresultsbuf(alglib_state_, x_opt_, rep_);

                    if (int(rep_.terminationtype) > 0)
                    {
                        double max_step = 20.0; 
                        x_opt_[0] = std::clamp(x_opt_[0], Kx_(5) - max_step, Kx_(5) + max_step);
                        Kx_vec_[5] = x_opt_[0];
                        Kx_(5) = x_opt_[0];
                        Bx_(5) = 1.42 * sqrt(Kx_(5));
                    }
                    else
                    {
                        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
                                             "ALGLIB failed (Code: %d). Fallback to K_min.", int(rep_.terminationtype));
                        std::cerr << "ALGLIB optimization failed with termination type: " << int(rep_.terminationtype) << std::endl;
                        Kx_vec_[5] = Kz_min_;
                        Kx_(5) = Kz_min_;
                    }
                }
                catch (alglib::ap_error alglib_exception)
                {
                    RCLCPP_WARN(node_->get_logger(), "ALGLIB exception: %s", alglib_exception.msg.c_str());
                }
            }
            else
            {
                Kx_vec_[5] = Kz_max_;
                Kx_(5) = Kz_max_;
                Bx_(5) = 1.42 * sqrt(Kx_(5));
                force_int_z_ = 0.0;
                force_err_z_prev_ = 0.0;
            }

            double current_M_dot_norm = dM_.norm();
            // RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500, 
            //                     "当前质量矩阵导数范数 ||dM||: %f", current_M_dot_norm);

            // Eigen::VectorXd H = C_ * dq + g_;
            Eigen::VectorXd p_dob = Y_.cwiseProduct(dq);
            Eigen::VectorXd dob_rhs =  - tau_d - z_ - p_dob;
            Eigen::VectorXd Minv_rhs = M_.ldlt().solve(dob_rhs);
            Eigen::VectorXd dz = Y_.cwiseProduct(Minv_rhs);

            z_ += dz * dt;
            tau_d_est_ = z_ + p_dob;
            Eigen::MatrixXd Lambda = robot_math::A_x(Jb_, M_);
            Eigen::MatrixXd Lambda_inv = robot_math::A_x_inv(Jb_, M_);
            Eigen::VectorXd Minv_taud = M_.ldlt().solve(tau_d_est_);
            tau_x_est_ = Jb_.transpose() * Lambda * (Jb_ * Minv_taud);
         
            ddxc_ = ddxd_ + robot_math::A_x_inv(Jb_, M_) * (robot_math::Mu_x_X(Jb_, M_, dJb_, C_, dxe_) + Bx_.asDiagonal() * dxe_ + Kx_.asDiagonal() * xe_);
            tau_task_ = M_ * robot_math::J_sharp(Jb_, M_) * (ddxc_ - dJb_ * dq);
            // tau_task_ = M_ * robot_math::J_sharp_X(Jb_, M_, ddxc_- dJb_* dq);
            Eigen::LDLT<Eigen::MatrixXd> ldlt(M_);
            tau_null_ = M_ * robot_math::null_proj(Jb_, M_, ldlt.solve(Bn_.asDiagonal() * (-dq)));
            

            F_imp_ = Lambda_inv.ldlt().solve(ddxc_ - dJb_ * dq);

            double fric_comp_ratio = 1.0;
            // Eigen::VectorXd tau_base = tau_task_ + tau_null_ - tau_x_est_;   
            // t_filter_.filtering(tau_base.data(), tau_base.data());

            // std::cerr<<"tau_base: "<<tau_base.transpose()<<std::endl;
            // double max_torque_rate = 40.0; 
            // double max_delta_tau = max_torque_rate * dt; 
            double max_delta_tau = 0.5; 

            // // ★ Karnopp 模型参数
            // double DV = 0.01; // 速度死区 (Deviation Velocity)，可根据底噪调整 0.002~0.005
            // double K_assist = 3.0; // 意图放大系数 (通常 1.0 ~ 3.0，决定起步时的"助力"有多快跟上)

            
            double delay_time = 1.0; // 启动后完全关闭补偿的时间 (秒)
            double fade_time  = 1.0; // 过渡时间，用 1 秒的时间平滑增加到 100%
            
            if (time_ > delay_time) {
                // time_ 从 1.0 到 2.0 时，ratio 从 0 线性变到 1.0
                fric_comp_ratio = std::min(1.0, (time_ - delay_time) / fade_time);
            }
            
            for (int i = 0; i < dof_; i++)
            {
                double v = dq(i);
                double tau_f_i = 0.0;

                double K_v;
                if (i == 0 || i == 1) {
                    K_v = 400.0; 
                } else {
                    K_v = 1500.0; 
                }
                
                double sign_smooth = (2.0 / M_PI) * std::atan(K_v * v);
 
                double term_Fc = F_c_[i];
                double term_Fs_fit = F_s_[i];  
                double term_vs = v_s_[i];
                double term_alpha = alpha_[i];
                double term_sigma2 = sigma2_[i];

                double stribeck_decay = std::exp(-std::pow(std::abs(v) / term_vs, term_alpha));
                double base_shape = term_Fc + (term_Fs_fit - term_Fc) * stribeck_decay;
                
                double F_limit = F_s_actual_[i]; 

                double delta_F = std::max(0.0, F_limit - term_Fs_fit); 
                double v_boost = 0.005; 
                double boost_shape = delta_F * std::exp(-std::abs(v) / v_boost);

                double raw_tau_f = (base_shape + boost_shape) * sign_smooth + term_sigma2 * v;

                if (raw_tau_f > F_limit) {
                    tau_f_i = F_limit;
                } else if (raw_tau_f < -F_limit) {
                    tau_f_i = -F_limit;
                } else {
                    tau_f_i = raw_tau_f;
                }
                
                double target_tau_f = tau_f_i * fric_comp_ratio;

                double delta_tau = target_tau_f - tau_fric_ff_prev_(i);
                if (delta_tau > max_delta_tau) {
                    target_tau_f = tau_fric_ff_prev_(i) + max_delta_tau;
                } else if (delta_tau < -max_delta_tau) {
                    target_tau_f = tau_fric_ff_prev_(i) - max_delta_tau;
                }
                
                // 6. 更新状态并输出
                tau_fric_ff_prev_(i) = target_tau_f;
                tau_fric_ff_(i) = target_tau_f;
            }
            

            // 叠加指令并下发
            // tau_cmd = tau_task_ + tau_null_ - tau_x_est_ + tau_fric_ff_;
            tau_cmd = tau_task_ + tau_null_; 
            tau_cmd = saturate_torque(tau_cmd, tau_d);
            tau_d = tau_cmd;

            publish_robot_state(t, q_vec, dq_vec, force_);

            cal_time_ = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
            data_logger_->record();
        }

    private:
        void publish_robot_state(const rclcpp::Time &t,
                                 const std::vector<double> &q,
                                 const std::vector<double> &dq,
                                 const Eigen::Vector6d &force // <--- 修改 1：参数类型改为 Eigen::Vector6d
        )
        {
            if (!real_time_publisher_)
                return;
            robot_control_msgs::msg::RobotState msg;
            msg.header.stamp = t;
            std::fill_n(std::back_inserter(msg.robot_state), 28, 0);
            std::copy(q.begin(), q.end(), msg.robot_state.begin());
            std::copy(dq.begin(), dq.end(), msg.robot_state.begin() + 7);
            std::copy(force.data(), force.data() + 6, msg.robot_state.begin() + 14);
            if (real_time_publisher_->trylock())
            {
                real_time_publisher_->msg_ = msg;
                real_time_publisher_->unlockAndPublish();
            }
        }

    protected:
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;
        rclcpp::Publisher<robot_control_msgs::msg::RobotState>::SharedPtr robot_state_publisher_;
        std::shared_ptr<realtime_tools::RealtimePublisher<robot_control_msgs::msg::RobotState>> real_time_publisher_;
        rclcpp::CallbackGroup::SharedPtr call_back_group_;
        rclcpp_action::Server<ACTION>::SharedPtr action_server_;
        realtime_tools::RealtimeBuffer<BufferType> real_time_buffer_;

        int dof_;
        double time_, traj_time_;
        Eigen::MatrixXd M_, C_, Jb_, dJb_, dM_;
        Eigen::Vector6d pose_, force_;
        Eigen::VectorXd g_;
        Eigen::Matrix4d Tb_, dTb_;
        Eigen::VectorXd Kx_, Bx_, Kn_, Bn_;
        Eigen::VectorXd tau_cmd_, tau_task_, tau_null_;
        Eigen::VectorXd qd_, dqd_, ddqd_, qe_, dqe_, dq_, q_;
        Eigen::Vector6d xe_, dxe_, ddxd_, ddxc_, dVd;
        Eigen::Matrix3d Rd_, R_;
        Eigen::Vector3d pd_, p_, wd_, vd_;
        Eigen::Vector7d tau_d;
        Eigen::Vector6d F_imp_;
        std::vector<double> sensor_cog_vec_;
        std::vector<double> sensor_offset_vec_;
        double sensor_weight_;
        alglib::real_1d_array x_opt_;
        alglib::real_1d_array bndl_, bndu_;
        alglib::real_2d_array c_;
        alglib::integer_1d_array ct_;
        alglib::real_1d_array scale_;
        alglib::minbleicstate alglib_state_;
        alglib::minbleicreport rep_;
        OptParams params_;
        Eigen::Matrix4d T_sensor_;
        double F_des_z_, F_max_z_;
        double Kz_min_, Kz_max_;
        double Q_weight_, R_weight_;
        double Kp_f_, Ki_f_, Kd_f_, force_int_z_;
        double force_err_z_prev_;
        double cal_time_;
        std::unique_ptr<DataLogger> data_logger_;
        realtime_tools::RealtimeBox<std::vector<double>> Kx_in_box_, Bx_in_box_, Kn_in_box_, Bn_in_box_;
        std::vector<double> Kx_vec_, Bx_vec_, Kn_vec_, Bn_vec_;
        robot_math::MovingFilter<double> f_filter_,t_filter_;
        bool is_contact_established_{false};
        std::deque<double> force_err_window_;
        int integral_window_size_ ={50};
        Eigen::VectorXd z_;
        Eigen::VectorXd Y_;
        Eigen::VectorXd tau_d_est_;
        Eigen::VectorXd tau_x_est_;
        double dob_gain_val_;
        const std::vector<double> F_c_ = {0.000, 0.976, 0.000, 0.453, 0.254, 0.468, 0.000};
        // const std::vector<double> F_s_ = {11.872, 11.722, 2.6, 3.0, 0.9, 1.1, 1.3};
        const std::vector<double> F_s_ = {11.872, 11.722, 1.155, 1.256, 0.539, 0.573, 1.038};
        const std::vector<double> v_s_ = {0.010, 0.001, 0.103, 0.181, 1.000, 0.397, 0.469};
        const std::vector<double> sigma2_ = {0.086, 0.134, 0.000, 0.000, 0.000, 0.000, 0.000};
        const std::vector<double> alpha_ = {0.167, 0.199, 0.493, 0.654, 0.362, 1.276, 0.164};
        const std::vector<double> F_s_actual_ = {3.0, 1.5, 2.6, 3.0, 0.9, 1.1, 1.3};
        Eigen::VectorXd tau_fric_ff_;
        Eigen::VectorXd tau_fric_ff_prev_;
    };
} // namespace controllers

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(controllers::VariableImpedanceControllerold, controller_interface::ControllerInterface)


// for (int i = 0; i < dof_; i++)
            // {
            //     double v = dq(i);
            //     double tau_f_i = 0.0;
                
            //     // =========================================
            //     // 1. 发力意图死区 (Deadband)
            //     // 只有当基础控制力矩绝对值 > 0.1 Nm 时，才激活补偿，滤除静止时的噪声
            //     // =========================================
            //     if (std::abs(tau_base(i)) > 0.1) {
                    
            //         // =========================================
            //         // 2. 统一使用 LuGre (Stribeck) 模型计算理论摩擦力
            //         // =========================================
            //         // 确定摩擦力的方向：
            //         // 当速度极小(如低于 0.005)时，速度信号充满噪声，方向由控制意图(tau_base)决定，打破静摩擦
            //         // 当有明显速度时，方向严格由真实速度 v 决定，阻碍运动
            //         double sign_dir = 0.0;
            //         if (std::abs(v) > 0.005) {
            //             sign_dir = (v > 0) ? 1.0 : -1.0;
            //         } else {
            //             sign_dir = (tau_base(i) > 0) ? 1.0 : -1.0;
            //         }

            //         // 计算标准的 LuGre 稳态大小 (不含方向)
            //         double term1 = F_c_[i];
            //         double term2 = (F_s_[i] - F_c_[i]) * std::exp(-std::pow(std::abs(v) / v_s_[i], alpha_[i]));
            //         double term3 = sigma2_[i] * v;
                    
            //         // 得到带有方向的原始理论摩擦力补偿值
            //         double raw_tau_f = (term1 + term2) * sign_dir + term3;

            //         // =========================================
            //         // 3. 真实死区截断 (Capping / Truncation)
            //         // 使用真实测得的最大静摩擦力 F_s_actual_ 进行上下限截断
            //         // =========================================
            //         // 乘以 0.9 是工程安全系数，防止 100% 抵消后 PD 失去阻尼而在原地高频跳动
            //         double limit = F_s_actual_[i] ; 
                    
            //         if (raw_tau_f > limit) {
            //             tau_f_i = limit;
            //         } else if (raw_tau_f < -limit) {
            //             tau_f_i = -limit;
            //         } else {
            //             tau_f_i = raw_tau_f;
            //         }
                    
            //     } else {
            //         // 控制力矩在 0.1Nm 以内，视为静止或到位，不输出摩擦力补偿
            //         tau_f_i = 0.0;
            //     }

            //     // =========================================
            //     // 终极硬件保护：力矩变化率限制 (Rate Limiter)
            //     // 解决 "力矩插值失败" 报错的核心！
            //     // =========================================
            //     double delta_tau = tau_f_i - tau_fric_ff_prev_(i);
            //     if (delta_tau > max_delta_tau) {
            //         tau_f_i = tau_fric_ff_prev_(i) + max_delta_tau;
            //     } else if (delta_tau < -max_delta_tau) {
            //         tau_f_i = tau_fric_ff_prev_(i) - max_delta_tau;
            //     }
                
            //     // 更新记忆状态
            //     tau_fric_ff_prev_(i) = tau_f_i;
                
            //     // 输出最终前馈
            //     tau_fric_ff_(i) = tau_f_i * fric_comp_ratio;
            // }