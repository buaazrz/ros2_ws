#include <cstring>

#include "robot_controller_interface/controller_interface.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "realtime_tools/realtime_box.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "robot_math/MovingFilter.h"
#include "robot_math/robot_math.hpp"
#include "robot_math/PiecewiseCartesianTrajectory.hpp"
#include "ros2_utility/data_logger.hpp"
#include "robot_control_msgs/action/robot_motion.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_control_msgs/msg/robot_state.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace controllers
{
    class VariableImpedanceController : public controller_interface::ControllerInterface
    {
    public:
        using ACTION = robot_control_msgs::action::RobotMotion;
        using GoalHandle = rclcpp_action::ServerGoalHandle<ACTION>;
        using BufferType = std::pair<std::shared_ptr<GoalHandle>, std::shared_ptr<robot_math::PiecewiseCartesianTrajectory>>;

        // [修改1] 增加 50 点关节速度滤波器；15 点力/意图力矩滤波保持原控制器设置。
        VariableImpedanceController()
            : f_filter_(6, 15), t_filter_(7, 15), dq_filter_(7, 50)
        {
        }
        ~VariableImpedanceController()
        {
            if (data_logger_)
                data_logger_->save("/home/wjc/experiment_logs/variable_imp_controller/", "variable_imp_controller");
        }

        CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            dof_ = robot_->dof;

            // [修改2] 本控制器及真机接口均按 Diana7 的 7 轴固定维度实现，先显式拒绝错误模型。
            if (dof_ != 7)
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "VariableImpedanceController requires 7 DOF, got %d", dof_);
                return CallbackReturn::FAILURE;
            }

            node_->get_parameter_or<std::vector<double>>("Kx", Kx_vec_, {500.0, 500.0, 500.0, 2500.0, 2500.0, 2500.0});
            node_->get_parameter_or<std::vector<double>>("Bx", Bx_vec_, {30.0, 30.0, 30.0, 150.0, 150.0, 150.0});
            node_->get_parameter_or<std::vector<double>>("Kn", Kn_vec_, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
            node_->get_parameter_or<std::vector<double>>("Bn", Bn_vec_, {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
            node_->get_parameter_or<double>("Q_weight", Q_weight_, 100000.0);
            node_->get_parameter_or<double>("R_weight", R_weight_, 0.1);
            node_->get_parameter_or<double>("F_des_z", F_des_z_, -2.0);
            node_->get_parameter_or<double>("F_max_z", F_max_z_, 10.0);
            node_->get_parameter_or<double>("Kp_f", Kp_f_, 1.5);
            node_->get_parameter_or<double>("Ki_f", Ki_f_, 2.0);
            // [修改3] 兼容 main 配置里已有的 kd_f 拼写；新配置请统一使用 Kd_f。
            if (node_->has_parameter("Kd_f"))
            {
                node_->get_parameter("Kd_f", Kd_f_);
            }
            else
            {
                node_->get_parameter_or<double>("kd_f", Kd_f_, 0.0);
            }
            node_->get_parameter_or<double>("Kz_min", Kz_min_, 100.0);
            node_->get_parameter_or<double>("Kz_max", Kz_max_, 2500.0);

            // [修改4] 真机摩擦补偿参数。默认关闭且比例为 0，必须在 YAML 中显式开启。
            node_->get_parameter_or<bool>("friction_comp_enabled", friction_comp_enabled_, false);
            node_->get_parameter_or<double>("friction_comp_ratio", friction_comp_ratio_, 0.0);
            node_->get_parameter_or<double>("hold_friction_ratio", hold_friction_ratio_, 0.0);
            node_->get_parameter_or<double>("friction_start_delay", friction_start_delay_, 1.0);
            node_->get_parameter_or<double>("friction_fade_time", friction_fade_time_, 1.0);
            node_->get_parameter_or<double>("friction_torque_rate", friction_torque_rate_, 40.0);
            node_->get_parameter_or<double>("vel_des_deadzone", vel_des_deadzone_, 1e-5);
            node_->get_parameter_or<double>("vel_actual_deadzone", vel_actual_deadzone_, 1e-4);
            node_->get_parameter_or<double>("tau_intent_deadzone", tau_intent_deadzone_, 0.05);
            node_->get_parameter_or<double>("max_stiffness_rate", max_stiffness_rate_, 20000.0);
            node_->get_parameter_or<int>("integral_window_size", integral_window_size_, 50);
            node_->get_parameter_or<std::vector<double>>(
                "friction_pos", friction_pos_, {4.5, 2.8, 3.3, 4.0, 1.14, 1.37, 1.60});
            node_->get_parameter_or<std::vector<double>>(
                "friction_neg", friction_neg_, {3.5, 5.2, 3.3, 3.9, 1.41, 1.67, 1.55});

            // [修改5] 配置期就阻止尺寸错误、NaN、负阻尼等参数进入 1 kHz 实时循环。
            if (!is_finite_nonnegative_vector(Kx_vec_, 6) ||
                !is_finite_nonnegative_vector(Bx_vec_, 6) ||
                !is_finite_nonnegative_vector(Kn_vec_, static_cast<std::size_t>(dof_)) ||
                !is_finite_nonnegative_vector(Bn_vec_, static_cast<std::size_t>(dof_)) ||
                !is_finite_nonnegative_vector(friction_pos_, static_cast<std::size_t>(dof_)) ||
                !is_finite_nonnegative_vector(friction_neg_, static_cast<std::size_t>(dof_)))
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "Kx/Bx must contain 6 finite non-negative values; "
                             "Kn/Bn/friction_pos/friction_neg must contain %d", dof_);
                return CallbackReturn::FAILURE;
            }

            const bool scalar_parameters_valid =
                std::isfinite(Q_weight_) && Q_weight_ > 0.0 &&
                std::isfinite(R_weight_) && R_weight_ > 0.0 &&
                std::isfinite(F_des_z_) && std::abs(F_des_z_) > 1e-9 &&
                std::isfinite(F_max_z_) && F_max_z_ > 0.0 &&
                std::abs(F_des_z_) <= F_max_z_ &&
                std::isfinite(Kp_f_) && std::isfinite(Ki_f_) && std::isfinite(Kd_f_) &&
                std::isfinite(Kz_min_) && Kz_min_ > 0.0 &&
                std::isfinite(Kz_max_) && Kz_max_ >= Kz_min_ &&
                Kx_vec_[5] >= Kz_min_ && Kx_vec_[5] <= Kz_max_ &&
                std::isfinite(friction_comp_ratio_) &&
                std::isfinite(hold_friction_ratio_) &&
                std::isfinite(friction_start_delay_) && friction_start_delay_ >= 0.0 &&
                std::isfinite(friction_fade_time_) && friction_fade_time_ > 0.0 &&
                std::isfinite(friction_torque_rate_) && friction_torque_rate_ > 0.0 &&
                std::isfinite(vel_des_deadzone_) && vel_des_deadzone_ >= 0.0 &&
                std::isfinite(vel_actual_deadzone_) && vel_actual_deadzone_ >= 0.0 &&
                std::isfinite(tau_intent_deadzone_) && tau_intent_deadzone_ >= 0.0 &&
                std::isfinite(max_stiffness_rate_) && max_stiffness_rate_ > 0.0 &&
                integral_window_size_ > 0;

            if (!scalar_parameters_valid)
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "Invalid VIC scalar parameter: check force/stiffness bounds, "
                             "weights, deadzones, rates and integral_window_size");
                return CallbackReturn::FAILURE;
            }

            if (friction_comp_ratio_ < 0.0 || friction_comp_ratio_ > 1.0 ||
                hold_friction_ratio_ < 0.0 || hold_friction_ratio_ > 1.0)
            {
                RCLCPP_WARN(node_->get_logger(),
                            "Friction ratios are clamped to [0, 1]");
            }
            friction_comp_ratio_ = std::clamp(friction_comp_ratio_, 0.0, 1.0);
            hold_friction_ratio_ = std::clamp(hold_friction_ratio_, 0.0, 1.0);

            Kx_in_box_.set(Kx_vec_);
            Bx_in_box_.set(Bx_vec_);
            Kn_in_box_.set(Kn_vec_);
            Bn_in_box_.set(Bn_vec_);

            parameters_callback_handle_ = node_->add_on_set_parameters_callback(
                [this](std::vector<rclcpp::Parameter> parameters) -> rcl_interfaces::msg::SetParametersResult
                {
                    auto result = rcl_interfaces::msg::SetParametersResult();
                    result.successful = true;

                    for (const auto &parameter : parameters)
                    {
                        if (parameter.get_name() == "Kx")
                        {
                            if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
                            {
                                result.successful = false;
                                result.reason = "Kx must be a double array";
                                return result;
                            }
                            const auto value = parameter.as_double_array();
                            if (!is_finite_nonnegative_vector(value, 6))
                            {
                                result.successful = false;
                                result.reason = "Kx must contain 6 finite non-negative values";
                                return result;
                            }
                            Kx_in_box_.set(value);
                        }
                        else if (parameter.get_name() == "Bx")
                        {
                            if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
                            {
                                result.successful = false;
                                result.reason = "Bx must be a double array";
                                return result;
                            }
                            const auto value = parameter.as_double_array();
                            if (!is_finite_nonnegative_vector(value, 6))
                            {
                                result.successful = false;
                                result.reason = "Bx must contain 6 finite non-negative values";
                                return result;
                            }
                            Bx_in_box_.set(value);
                        }
                        else if (parameter.get_name() == "Kn")
                        {
                            if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
                            {
                                result.successful = false;
                                result.reason = "Kn must be a double array";
                                return result;
                            }
                            const auto value = parameter.as_double_array();
                            if (!is_finite_nonnegative_vector(value, static_cast<std::size_t>(dof_)))
                            {
                                result.successful = false;
                                result.reason = "Kn has invalid size or value";
                                return result;
                            }
                            Kn_in_box_.set(value);
                        }
                        else if (parameter.get_name() == "Bn")
                        {
                            if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
                            {
                                result.successful = false;
                                result.reason = "Bn must be a double array";
                                return result;
                            }
                            const auto value = parameter.as_double_array();
                            if (!is_finite_nonnegative_vector(value, static_cast<std::size_t>(dof_)))
                            {
                                result.successful = false;
                                result.reason = "Bn has invalid size or value";
                                return result;
                            }
                            Bn_in_box_.set(value);
                        }
                    }
                    return result;
                });

            RCLCPP_INFO(node_->get_logger(),
                        "VIC configured: friction=%s, ratio=%.3f, hold_ratio=%.3f",
                        friction_comp_enabled_ ? "enabled" : "disabled",
                        friction_comp_ratio_, hold_friction_ratio_);

            return CallbackReturn::SUCCESS;
        }

        double solve_closed_form_stiffness(double xe_z, double dxe_z, double Bz_curr, double F_target)
        {
            double xe_eps = 1e-4;
            
            if (std::abs(xe_z) <= xe_eps)
            {
                return Kz_max_; 
            }

            double dx_B = -Bz_curr * dxe_z;

            double denom = Q_weight_ * (xe_z * xe_z) + R_weight_;
            double numer = Q_weight_ * xe_z * (dx_B + F_target) + R_weight_ * Kz_min_;
            double K_star = numer / denom;

            double k1 = (dx_B - F_max_z_) / xe_z;
            double k2 = (dx_B + F_max_z_) / xe_z;
            double force_lb = std::min(k1, k2);
            double force_ub = std::max(k1, k2);

            double lb = std::max(Kz_min_, force_lb);
            double ub = std::min(Kz_max_, force_ub);

            if (lb > ub)
            {
                return Kz_min_;
            }
            
            return std::max(lb, std::min(K_star, ub));
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
            time_ = 0.0;
            traj_time_ = 0.0;
            cal_time_ = 0.0;
            // 窗口仅在非实时激活阶段分配；update() 内使用固定环形缓存，不再 push/pop 分配。
            force_err_window_.assign(static_cast<std::size_t>(integral_window_size_), 0.0);
            reset_force_control_state();
            is_contact_established_ = false;
            friction_active_ratio_ = 0.0;
            // 每次重新激活都从自由空间安全刚度开始，不继承上次接触结束时的自适应值。
            Kx_vec_[5] = Kz_max_;
            Bx_vec_[5] = 1.42 * std::sqrt(Kz_max_);
            F_ext_z_log_ = 0.0;
            F_err_z_log_ = 0.0;
            F_target_z_log_ = F_des_z_;
            K_opt_log_ = Kx_vec_[5];
            B_opt_log_ = Bx_vec_[5];

            real_time_buffer_.reset();
            active_goal_handle_.reset();
            tau_d.setZero();
            f_filter_.reset();
            t_filter_.reset();
            dq_filter_.reset();
            F_imp_.setZero();

            // [修改6] 初始化摩擦方向判定和日志所需的全部向量，避免首次实时周期尺寸不匹配。
            tau_fric_ff_ = Eigen::VectorXd::Zero(dof_);
            tau_fric_ff_prev_ = Eigen::VectorXd::Zero(dof_);
            tau_intent_ = Eigen::VectorXd::Zero(dof_);
            tau_intent_filtered_ = Eigen::VectorXd::Zero(dof_);
            qd_des_ = Eigen::VectorXd::Zero(dof_);
            dq_filtered_ = Eigen::VectorXd::Zero(dof_);

            Kx_ = Eigen::Map<const Eigen::VectorXd>(Kx_vec_.data(), 6);
            Bx_ = Eigen::Map<const Eigen::VectorXd>(Bx_vec_.data(), 6);
            Kn_ = Eigen::Map<const Eigen::VectorXd>(Kn_vec_.data(), dof_);
            Bn_ = Eigen::Map<const Eigen::VectorXd>(Bn_vec_.data(), dof_);

            robot_state_publisher_ = node_->create_publisher<robot_control_msgs::msg::RobotState>("robot_states", rclcpp::SensorDataQoS());
            real_time_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<robot_control_msgs::msg::RobotState>>(robot_state_publisher_);

            const std::vector<double> &q_vec = state_->get<double>("position");
            const std::vector<double> &dq_vec = state_->get<double>("velocity");
            const std::vector<double> &pose = state_->get<double>("pose");
            std::vector<double> &tau_command = command_->get<double>("torque");
            std::vector<int> &mode_command = command_->get<int>("mode");

            if (q_vec.size() != static_cast<std::size_t>(dof_) ||
                dq_vec.size() != static_cast<std::size_t>(dof_) ||
                pose.size() != 6 ||
                tau_command.size() != static_cast<std::size_t>(dof_) ||
                mode_command.empty())
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "Invalid VIC interface sizes: q=%zu dq=%zu pose=%zu tau=%zu mode=%zu",
                             q_vec.size(), dq_vec.size(), pose.size(),
                             tau_command.size(), mode_command.size());
                return CallbackReturn::FAILURE;
            }

            if (!com_state_)
            {
                RCLCPP_ERROR(node_->get_logger(), "Missing component state map");
                return CallbackReturn::FAILURE;
            }
            const auto force_sensor = com_state_->find("ft_sensor");
            if (force_sensor == com_state_->end() || !force_sensor->second)
            {
                RCLCPP_ERROR(node_->get_logger(), "Missing or invalid ft_sensor force interface");
                return CallbackReturn::FAILURE;
            }
            const auto &force_states = force_sensor->second->get<double>();
            const auto force_entry = force_states.find("force");
            if (force_entry == force_states.end() || force_entry->second.size() != 6)
            {
                RCLCPP_ERROR(node_->get_logger(), "Missing or invalid ft_sensor force interface");
                return CallbackReturn::FAILURE;
            }

            qd_ = Eigen::Map<const Eigen::VectorXd>(q_vec.data(), dof_).eval();
            dqd_ = Eigen::VectorXd::Zero(dof_);
            ddqd_ = Eigen::VectorXd::Zero(dof_);

            pose_ = Eigen::Map<const Eigen::Vector6d>(pose.data());

            // [修改7] 期望初始位姿用主分支同一 URDF/机器人模型的 FK，避免混用 MuJoCo 的 state[\"T\"]。
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

            sensor_weight_ =  0.0;
            sensor_cog_vec_ = {    0.000  , 0.000  ,  0.0029};
            sensor_offset_vec_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            T_sensor_ = Eigen::Matrix4d::Identity();
            T_sensor_ << 1, 0, 0, 0,
                0, -1, 0, 0,
                0, 0, -1, 0,
                0, 0, 0, 1;

            // [修改8] 在接收阶段校验完整轨迹，避免无效时间戳进入实时线程。
            auto handle_goal = [this](const rclcpp_action::GoalUUID &uuid,
                                      std::shared_ptr<const ACTION::Goal> goal)
            {
                (void)uuid;
                std::string reason;
                if (!goal)
                {
                    RCLCPP_ERROR(node_->get_logger(), "Reject VIC trajectory: null goal");
                    return rclcpp_action::GoalResponse::REJECT;
                }
                if (!validate_trajectory_data(goal->target_position.data, reason))
                {
                    RCLCPP_ERROR(node_->get_logger(), "Reject VIC trajectory: %s", reason.c_str());
                    return rclcpp_action::GoalResponse::REJECT;
                }

                const auto *current_pair = real_time_buffer_.readFromNonRT();
                if (current_pair && current_pair->first && current_pair->first->is_active())
                {
                    RCLCPP_WARN(node_->get_logger(),
                                "Reject VIC trajectory: another goal is still active");
                    return rclcpp_action::GoalResponse::REJECT;
                }
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            };

            auto handle_cancel = [](const std::shared_ptr<GoalHandle> goal_handle)
            {
                (void)goal_handle;
                return rclcpp_action::CancelResponse::ACCEPT;
            };

            auto handle_accepted = [this](const std::shared_ptr<GoalHandle> goal_handle)
            {
                auto trajectory = std::make_shared<robot_math::PiecewiseCartesianTrajectory>();

                const std::vector<double> &q_current = state_->get<double>("position");
                if (q_current.size() != static_cast<std::size_t>(dof_))
                {
                    RCLCPP_ERROR(node_->get_logger(),
                                 "Abort VIC trajectory: invalid joint state size %zu", q_current.size());
                    auto result = std::make_shared<ACTION::Result>();
                    result->success = false;
                    goal_handle->abort(result);
                    return;
                }

                Eigen::Matrix4d T_current;
                robot_math::forward_kinematics(robot_, q_current, T_current);
                const std::vector<double> pose_current = robot_math::tform_to_pose(T_current);
                std::vector<double> full_traj_data = goal_handle->get_goal()->target_position.data;

                constexpr double kTimeZeroTolerance = 1e-9;
                if (full_traj_data.front() <= kTimeZeroTolerance)
                {
                    // 用户已经给了 t=0：覆盖其位姿，不再重复插入相同时间戳。
                    full_traj_data[0] = 0.0;
                    std::copy(pose_current.begin(), pose_current.end(), full_traj_data.begin() + 1);
                }
                else
                {
                    // 用户首点晚于 t=0：在前面插入接收目标瞬间的实际 TCP。
                    std::vector<double> with_current_pose;
                    with_current_pose.reserve(full_traj_data.size() + 7);
                    with_current_pose.push_back(0.0);
                    with_current_pose.insert(with_current_pose.end(),
                                             pose_current.begin(), pose_current.end());
                    with_current_pose.insert(with_current_pose.end(),
                                             full_traj_data.begin(), full_traj_data.end());
                    full_traj_data.swap(with_current_pose);
                }

                if (!trajectory->set_traj(full_traj_data))
                {
                    RCLCPP_ERROR(node_->get_logger(),
                                 "Abort VIC trajectory: PiecewiseCartesianTrajectory rejected the data");
                    auto result = std::make_shared<ACTION::Result>();
                    result->success = false;
                    goal_handle->abort(result);
                    return;
                }

                // 实时状态的复位在 update() 检测到新 goal 后执行，避免非实时回调与 deque 竞态。
                real_time_buffer_.writeFromNonRT({goal_handle, trajectory});

                RCLCPP_INFO(node_->get_logger(),
                            "VIC trajectory accepted: actual model-FK TCP is t=0, total points=%zu",
                            full_traj_data.size() / 7);
            };

            call_back_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
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
                    // [修改9] 补齐变阻抗和摩擦补偿的关键诊断量，便于真机离线核验。
                    DATA_WRAPPER(F_ext_z_log_),
                    DATA_WRAPPER(F_err_z_log_),
                    DATA_WRAPPER(F_target_z_log_),
                    DATA_WRAPPER(K_opt_log_),
                    DATA_WRAPPER(B_opt_log_),
                    DATA_WRAPPER(friction_active_ratio_),
                    DATA_WRAPPER(tau_d),
                    DATA_WRAPPER(tau_fric_ff_),
                    DATA_WRAPPER(qd_des_),
                    DATA_WRAPPER(dq_),
                    DATA_WRAPPER(dq_filtered_),
                    DATA_WRAPPER(tau_intent_filtered_),
                    DATA_WRAPPER(tau_task_),
                    DATA_WRAPPER(tau_null_),
                },
                std::initializer_list<ExperimentContext>{
                    CONFIG_WRAPPER(Kx_vec_),
                    CONFIG_WRAPPER(Bx_vec_),
                    CONFIG_WRAPPER(Kn_vec_),
                    CONFIG_WRAPPER(Bn_vec_),
                    CONFIG_WRAPPER(friction_pos_),
                    CONFIG_WRAPPER(friction_neg_),
                    CONFIG_WRAPPER(friction_comp_ratio_),
                    CONFIG_WRAPPER(hold_friction_ratio_),
                },
                // main 的测试流程包含激活后等待和约 60 s 轨迹，预留 120 s 避免实时循环中扩容。
                120000);

            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            // [修改10] 清空前馈记忆，防止下次激活继承上次摩擦力矩。
            real_time_buffer_.reset();
            active_goal_handle_.reset();
            if (tau_fric_ff_.size() == dof_)
            {
                tau_fric_ff_.setZero();
                tau_fric_ff_prev_.setZero();
            }
            friction_active_ratio_ = 0.0;
            action_server_ = nullptr;
            real_time_publisher_ = nullptr;
            robot_state_publisher_ = nullptr;
            return CallbackReturn::SUCCESS;
        }

        void update(const rclcpp::Time &t, const rclcpp::Duration &period) override
        {
            (void)t;
            const double period_seconds = period.seconds();
            const double dt = std::isfinite(period_seconds) && period_seconds > 0.0
                                  ? period_seconds
                                  : 0.0;
            // 只给积分和变化率限制使用；调度暂停后不允许单周期大跳变。
            const double safety_dt = std::min(dt, 0.01);
            time_ += dt;

            if (dt > 0.01)
            {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                     "VIC control period is too large: %.6f s", dt);
            }

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

            Eigen::Map<const Eigen::VectorXd> q(q_vec.data(), dof_);
            Eigen::Map<const Eigen::VectorXd> dq(dq_vec.data(), dof_);
            Eigen::Map<Eigen::VectorXd> tau_cmd(tau_cmd_vec.data(), dof_);
            std::fill(tau_cmd_vec.begin(), tau_cmd_vec.end(), 0);

            if (!q.allFinite() || !dq.allFinite())
            {
                RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                      "Invalid joint state; ramping torque command toward zero");
                dq_filter_.reset();
                tau_task_.setZero();
                tau_null_.setZero();
                tau_fric_ff_.setZero();
                tau_fric_ff_prev_.setZero();
                tau_intent_.setZero();
                tau_intent_filtered_.setZero();
                qd_des_.setZero();
                dq_filtered_.setZero();
                friction_active_ratio_ = 0.0;
                tau_cmd = saturate_torque(Eigen::VectorXd::Zero(dof_), tau_d);
                tau_d = tau_cmd;
                cal_time_ = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(
                                         std::chrono::high_resolution_clock::now() - start_time)
                                         .count();
                data_logger_->record();
                return;
            }

            // [修改11] Kx/Bx 的前 5 维允许在线更新；Z 维由变阻抗算法独占，避免被参数回调覆盖。
            Kx_in_box_.try_get([this](auto const &value)
                               {
                                   std::copy_n(value.begin(), 5, Kx_vec_.begin());
                               });
            Bx_in_box_.try_get([this](auto const &value)
                               {
                                   std::copy_n(value.begin(), 5, Bx_vec_.begin());
                               });
            Kn_in_box_.try_get([this](auto const &value)
                               { Kn_vec_ = value; });
            Bn_in_box_.try_get([this](auto const &value)
                               { Bn_vec_ = value; });
            Kx_ = Eigen::Map<const Eigen::VectorXd>(Kx_vec_.data(), 6);
            Bx_ = Eigen::Map<const Eigen::VectorXd>(Bx_vec_.data(), 6);
            Kn_ = Eigen::Map<const Eigen::VectorXd>(Kn_vec_.data(), dof_);
            Bn_ = Eigen::Map<const Eigen::VectorXd>(Bn_vec_.data(), dof_);

            // [修改12] 真机实际关节速度先做 50 周期均值滤波，再参与“反向运动”保护判断。
            dq_filter_.filtering(dq_vec.data(), dq_filtered_.data());

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
            const bool force_measurement_valid = force_.allFinite();
            if (!force_measurement_valid)
            {
                RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                      "Invalid force sensor sample; reset force control and ignore sample");
                f_filter_.reset();
                force_.setZero();
                is_contact_established_ = false;
                reset_force_control_state();
            }
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                 "Filtered Force/Torque: [Fx: %.3f, Fy: %.3f, Fz: %.3f, Mx: %.3f, My: %.3f, Mz: %.3f]",
                                 force_[0], force_[1], force_[2], force_[3], force_[4], force_[5]);

            auto handle_pair = *real_time_buffer_.readFromRT();
            auto goal_handle = handle_pair.first;
            auto trajectory = handle_pair.second;

            // [修改13] 仅在实时线程内复位轨迹/力控状态，消除 action 回调和控制循环的数据竞争。
            if (goal_handle && goal_handle != active_goal_handle_)
            {
                active_goal_handle_ = goal_handle;
                traj_time_ = 0.0;
                is_contact_established_ = false;
                reset_force_control_state();
            }

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
                    traj_time_ += dt;
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

            const double xe_z = xe_(5);
            const double dxe_z = dxe_(5);
            const double B_z = Bx_(5);

            F_ext_z_log_ = force_(2);
            F_err_z_log_ = 0.0;
            F_target_z_log_ = F_des_z_;

            if (force_measurement_valid && !is_contact_established_)
            {
                if (std::abs(force_(2)) >= std::abs(F_des_z_))
                {
                    is_contact_established_ = true;
                    reset_force_control_state();
                    RCLCPP_WARN(node_->get_logger(), "Z轴接触力到达阈值: %.2f N, 开始变刚度模式!", force_(2));
                }
            }

            if (force_measurement_valid && is_contact_established_ &&
                std::abs(xe_z) > 1e-3 && xe_z * F_des_z_ > 0)
            {
                const double F_ext_z = force_(2);
                // [修改14] 保留 main/Diana 的力传感器轴向和符号约定：受控力为 -F_ext_z。
                const double F_err_z = F_des_z_ - (-F_ext_z);

                const double current_integral_step = F_err_z * safety_dt;
                if (force_err_window_count_ == force_err_window_.size())
                {
                    force_int_z_ -= force_err_window_[force_err_window_index_];
                }
                else
                {
                    ++force_err_window_count_;
                }
                force_err_window_[force_err_window_index_] = current_integral_step;
                force_int_z_ += current_integral_step;
                force_err_window_index_ =
                    (force_err_window_index_ + 1) % force_err_window_.size();

                double dF_err_z = 0.0;
                if (force_error_initialized_ && safety_dt > 1e-6)
                {
                    dF_err_z = (F_err_z - force_err_z_prev_) / safety_dt;
                }
                force_err_z_prev_ = F_err_z;
                force_error_initialized_ = true;

                const double F_com_z =
                    Kp_f_ * F_err_z + Ki_f_ * force_int_z_ + Kd_f_ * dF_err_z;
                const double F_target = F_des_z_ + F_com_z;

                double K_opt = solve_closed_form_stiffness(xe_z, dxe_z, B_z, F_target);
                if (!std::isfinite(K_opt))
                {
                    K_opt = Kz_max_;
                }

                // [修改15] 变化率按实际周期计算；K/B 的 vector 与 Eigen 副本始终同步。
                set_z_impedance(K_opt, safety_dt);
                F_err_z_log_ = F_err_z;
                F_target_z_log_ = F_target;
            }
            else
            {
                // 离开优化条件时也限速回到 Kz_max，避免从低刚度瞬间跳回硬阻抗。
                set_z_impedance(Kz_max_, safety_dt);
                reset_force_control_state();
            }

            K_opt_log_ = Kx_(5);
            B_opt_log_ = Bx_(5);

            ddxc_ = ddxd_ + robot_math::A_x_inv(Jb_, M_) *
                                 (Bx_.asDiagonal() * dxe_ + Kx_.asDiagonal() * xe_);
            tau_task_ = M_ * robot_math::J_sharp(Jb_, M_) * (ddxc_ - dJb_ * dq);
            Eigen::LDLT<Eigen::MatrixXd> ldlt(M_);
            tau_null_ = M_ * robot_math::null_proj(Jb_, M_, ldlt.solve(Bn_.asDiagonal() * (-dq)));

            Eigen::MatrixXd Lambda_inv = robot_math::A_x_inv(Jb_, M_);
            F_imp_ = Lambda_inv.ldlt().solve(ddxc_ - dJb_ * dq);

            // =====================================================================
            // [修改16] 移植 MuJoCo 中有效的“期望速度方向”策略，但替换为 Diana 实测的
            //          正/负向不对称候选值，并增加默认关闭、滤波、反向切断、淡入和斜率保护。
            // =====================================================================
            Eigen::Vector6d desired_body_twist;
            desired_body_twist.head(3) = R_.transpose() * wd_;
            desired_body_twist.tail(3) = R_.transpose() * vd_;
            qd_des_ = robot_math::J_sharp(Jb_, M_) * desired_body_twist;

            tau_intent_ = tau_task_ + tau_null_;
            t_filter_.filtering(tau_intent_.data(), tau_intent_filtered_.data());

            friction_active_ratio_ = 0.0;
            const bool trajectory_goal_active =
                goal_handle && goal_handle->is_active() && !goal_handle->is_canceling();
            if (friction_comp_enabled_ && trajectory_goal_active &&
                traj_time_ > friction_start_delay_)
            {
                const double fade = std::clamp(
                    (traj_time_ - friction_start_delay_) / friction_fade_time_, 0.0, 1.0);
                friction_active_ratio_ = friction_comp_ratio_ * fade;
            }

            const double max_delta_tau = friction_torque_rate_ * safety_dt;
            for (int i = 0; i < dof_; i++)
            {
                const double desired_velocity = qd_des_(i);
                const double actual_velocity = dq_filtered_(i);
                double target_tau_f = 0.0;

                if (std::abs(desired_velocity) > vel_des_deadzone_)
                {
                    const bool moving_opposite =
                        desired_velocity * actual_velocity < 0.0 &&
                        std::abs(actual_velocity) > vel_actual_deadzone_;

                    if (!moving_opposite)
                    {
                        const double identified_friction = desired_velocity > 0.0
                                                               ? friction_pos_[i]
                                                               : -friction_neg_[i];
                        target_tau_f = friction_active_ratio_ * identified_friction;
                    }
                }
                else if (hold_friction_ratio_ > 0.0 &&
                         std::abs(tau_intent_filtered_(i)) > tau_intent_deadzone_)
                {
                    // 停止段默认不补偿；只有显式设置 hold_friction_ratio 才按意图力矩辅助保持。
                    const double identified_friction = tau_intent_filtered_(i) > 0.0
                                                           ? friction_pos_[i]
                                                           : -friction_neg_[i];
                    target_tau_f = friction_active_ratio_ * hold_friction_ratio_ * identified_friction;
                }

                const double delta_tau = std::clamp(
                    target_tau_f - tau_fric_ff_prev_(i), -max_delta_tau, max_delta_tau);
                target_tau_f = tau_fric_ff_prev_(i) + delta_tau;
                tau_fric_ff_prev_(i) = target_tau_f;
                tau_fric_ff_(i) = target_tau_f;
            }

            // [修改17] 保留 main 真机已验证的力矩组成，不复制 MuJoCo 分支的 C*dq+g。
            tau_cmd = tau_task_ + tau_null_ + tau_fric_ff_;

            // 非有限数绝不下发；回退到上周期已限幅的安全命令，同时撤掉摩擦前馈。
            if (!tau_cmd.allFinite())
            {
                RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                      "Non-finite VIC torque detected; holding previous command");
                tau_fric_ff_.setZero();
                tau_fric_ff_prev_.setZero();
                tau_cmd = tau_d;
            }

            tau_cmd = saturate_torque(tau_cmd, tau_d);
            tau_d = tau_cmd;

            cal_time_ = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
            data_logger_->record();
        }

    private:
        static bool is_finite_nonnegative_vector(const std::vector<double> &values,
                                                 std::size_t expected_size)
        {
            return values.size() == expected_size &&
                   std::all_of(values.begin(), values.end(), [](double value)
                               { return std::isfinite(value) && value >= 0.0; });
        }

        static bool validate_trajectory_data(const std::vector<double> &data,
                                             std::string &reason)
        {
            constexpr std::size_t kWaypointWidth = 7;
            constexpr double kTimeZeroTolerance = 1e-9;

            if (data.empty() || data.size() % kWaypointWidth != 0)
            {
                reason = "data must be a non-empty multiple of 7: [t,x,y,z,rx,ry,rz]";
                return false;
            }

            const std::size_t point_count = data.size() / kWaypointWidth;
            double previous_time = -1.0;
            for (std::size_t point = 0; point < point_count; ++point)
            {
                const std::size_t base = point * kWaypointWidth;
                for (std::size_t offset = 0; offset < kWaypointWidth; ++offset)
                {
                    if (!std::isfinite(data[base + offset]))
                    {
                        reason = "trajectory contains NaN or infinity";
                        return false;
                    }
                }

                const double waypoint_time = data[base];
                if (waypoint_time < 0.0 || (point > 0 && waypoint_time <= previous_time))
                {
                    reason = "timestamps must be non-negative and strictly increasing";
                    return false;
                }
                previous_time = waypoint_time;
            }

            // 首点为 t=0 时会被实际 TCP 覆盖，因此还必须有一个目标点。
            if (data.front() <= kTimeZeroTolerance && point_count < 2)
            {
                reason = "a trajectory beginning at t=0 needs at least two waypoints";
                return false;
            }

            reason.clear();
            return true;
        }

        void set_z_impedance(double target_stiffness, double dt)
        {
            double current_stiffness = Kx_(5);
            if (!std::isfinite(current_stiffness))
            {
                current_stiffness = Kz_max_;
            }

            target_stiffness = std::clamp(target_stiffness, Kz_min_, Kz_max_);
            const double max_step = max_stiffness_rate_ * std::max(dt, 0.0);
            target_stiffness = std::clamp(
                target_stiffness,
                current_stiffness - max_step,
                current_stiffness + max_step);
            target_stiffness = std::clamp(target_stiffness, Kz_min_, Kz_max_);

            const double target_damping = 1.42 * std::sqrt(target_stiffness);
            Kx_vec_[5] = target_stiffness;
            Kx_(5) = target_stiffness;
            Bx_vec_[5] = target_damping;
            Bx_(5) = target_damping;
        }

        void reset_force_control_state()
        {
            force_int_z_ = 0.0;
            force_err_z_prev_ = 0.0;
            force_error_initialized_ = false;
            if (force_err_window_count_ > 0)
            {
                std::fill(force_err_window_.begin(), force_err_window_.end(), 0.0);
            }
            force_err_window_index_ = 0;
            force_err_window_count_ = 0;
        }

    protected:
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;
        rclcpp::Publisher<robot_control_msgs::msg::RobotState>::SharedPtr robot_state_publisher_;
        std::shared_ptr<realtime_tools::RealtimePublisher<robot_control_msgs::msg::RobotState>> real_time_publisher_;
        rclcpp::CallbackGroup::SharedPtr call_back_group_;
        rclcpp_action::Server<ACTION>::SharedPtr action_server_;
        realtime_tools::RealtimeBuffer<BufferType> real_time_buffer_;
        std::shared_ptr<GoalHandle> active_goal_handle_;

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
        robot_math::MovingFilter<double> f_filter_, t_filter_, dq_filter_;
        bool is_contact_established_{false};
        bool force_error_initialized_{false};
        std::vector<double> force_err_window_;
        std::size_t force_err_window_index_{0};
        std::size_t force_err_window_count_{0};
        int integral_window_size_{50};

        // [修改18] 用 Diana 正/负方向候选补偿值替代 MuJoCo 对称 frictionloss。
        bool friction_comp_enabled_{false};
        double friction_comp_ratio_{0.0};
        double hold_friction_ratio_{0.0};
        double friction_start_delay_{1.0};
        double friction_fade_time_{1.0};
        double friction_torque_rate_{40.0};
        double vel_des_deadzone_{1e-5};
        double vel_actual_deadzone_{1e-4};
        double tau_intent_deadzone_{0.05};
        double max_stiffness_rate_{20000.0};
        std::vector<double> friction_pos_;
        std::vector<double> friction_neg_;
        Eigen::VectorXd tau_fric_ff_, tau_fric_ff_prev_;
        Eigen::VectorXd tau_intent_, tau_intent_filtered_;
        Eigen::VectorXd qd_des_, dq_filtered_;

        double friction_active_ratio_{0.0};
        double F_ext_z_log_{0.0};
        double F_err_z_log_{0.0};
        double F_target_z_log_{0.0};
        double K_opt_log_{0.0};
        double B_opt_log_{0.0};
    };
} // namespace controllers

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(controllers::VariableImpedanceController, controller_interface::ControllerInterface)
