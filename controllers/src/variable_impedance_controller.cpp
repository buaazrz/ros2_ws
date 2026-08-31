#include "robot_controller_interface/controller_interface.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_server_goal_handle.hpp"
#include "robot_math/MovingFilter.h"
#include "robot_math/robot_math.hpp"
#include "robot_math/PiecewiseCartesianTrajectory.hpp"
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
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>

// using namespace robot_math;

namespace controllers
{
    class VariableImpedanceController : public controller_interface::ControllerInterface
    {
    public:
        using ACTION = robot_control_msgs::action::RobotMotion;
        using GoalHandle = rclcpp_action::ServerGoalHandle<ACTION>;
        using RealtimeGoalHandle = realtime_tools::RealtimeServerGoalHandle<ACTION>;
        using RealtimeGoalHandlePtr = std::shared_ptr<RealtimeGoalHandle>;
        struct GoalCommand
        {
            RealtimeGoalHandlePtr goal_handle;
            std::shared_ptr<robot_math::PiecewiseCartesianTrajectory> trajectory;
            std::uint64_t generation{0};
        };
        using BufferType = GoalCommand;
        struct GainSnapshot
        {
            std::array<double, 6> Kx{};
            std::array<double, 6> Bx{};
            std::array<double, 7> Kn{};
            std::array<double, 7> Bn{};
            std::uint64_t version{0};
        };

        template <std::size_t N>
        static bool copy_gain_parameter(const std::vector<double> &source, std::array<double, N> &destination)
        {
            if (source.size() != N ||
                !std::all_of(source.begin(), source.end(), [](double value)
                             { return std::isfinite(value) && value >= 0.0; }))
                return false;
            std::copy(source.begin(), source.end(), destination.begin());
            return true;
        }

        VariableImpedanceController() : f_filter_(6, 15) {}
        ~VariableImpedanceController()
        {
            if (data_logger_)
                data_logger_->save("/home/luo/experiment_logs/variable_imp_controller/", "variable_imp_controller");
        }

        CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            dof_ = robot_->dof;
            if (dof_ != 7)
            {
                RCLCPP_ERROR(node_->get_logger(), "VariableImpedanceController requires 7 DOF, got %d", dof_);
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
            node_->get_parameter_or<double>("Kp_f", Kp_f_, 0.0);
            node_->get_parameter_or<double>("Ki_f", Ki_f_, 0.0);
            node_->get_parameter_or<double>("kd_f", Kd_f_, 0.0);
            node_->get_parameter_or<double>("Kz_min", Kz_min_, 100.0);
            node_->get_parameter_or<double>("Kz_max", Kz_max_, 2500.0);
            node_->get_parameter_or<double>("max_stiffness_rate", max_stiffness_rate_, 20000.0);
            node_->get_parameter_or<double>("max_torque_rate", max_torque_rate_, 1000.0);
            node_->get_parameter_or<int>("integral_window_size", integral_window_size_, 50);
            node_->get_parameter_or<double>("F_stop_z", F_stop_z_, std::max(12.0, F_max_z_));
            node_->get_parameter_or<double>("F_stop_norm", F_stop_norm_, std::max(20.0, F_stop_z_));
            node_->get_parameter_or<int>("force_stop_samples", force_stop_samples_, 5);
            node_->get_parameter_or<double>("sensor_mass_kg", sensor_mass_kg_, 0.0);

            const auto valid_vector = [](const std::vector<double> &values, std::size_t expected)
            {
                return values.size() == expected &&
                       std::all_of(values.begin(), values.end(), [](double value)
                                   { return std::isfinite(value) && value >= 0.0; });
            };
            if (!valid_vector(Kx_vec_, 6) || !valid_vector(Bx_vec_, 6) ||
                !valid_vector(Kn_vec_, 7) || !valid_vector(Bn_vec_, 7) ||
                !std::isfinite(Q_weight_) || Q_weight_ < 0.0 ||
                !std::isfinite(R_weight_) || R_weight_ < 0.0 ||
                !std::isfinite(F_des_z_) || std::abs(F_des_z_) > F_max_z_ ||
                !std::isfinite(Kz_min_) || !std::isfinite(Kz_max_) || Kz_min_ > Kz_max_ ||
                Kx_vec_[5] < Kz_min_ || Kx_vec_[5] > Kz_max_ ||
                !std::isfinite(F_max_z_) || F_max_z_ <= 0.0 ||
                !std::isfinite(max_stiffness_rate_) || max_stiffness_rate_ <= 0.0 ||
                !std::isfinite(max_torque_rate_) || max_torque_rate_ <= 0.0 ||
                integral_window_size_ <= 0 || force_stop_samples_ <= 0 ||
                !std::isfinite(F_stop_z_) || !std::isfinite(F_stop_norm_) ||
                F_stop_z_ <= 0.0 || F_stop_norm_ <= 0.0 ||
                !std::isfinite(sensor_mass_kg_) || sensor_mass_kg_ < 0.0 ||
                !std::isfinite(Kp_f_) || !std::isfinite(Ki_f_) || !std::isfinite(Kd_f_))
            {
                RCLCPP_ERROR(node_->get_logger(), "invalid VariableImpedanceController parameters");
                return CallbackReturn::FAILURE;
            }

            GainSnapshot initial_gains;
            std::copy(Kx_vec_.begin(), Kx_vec_.end(), initial_gains.Kx.begin());
            std::copy(Bx_vec_.begin(), Bx_vec_.end(), initial_gains.Bx.begin());
            std::copy(Kn_vec_.begin(), Kn_vec_.end(), initial_gains.Kn.begin());
            std::copy(Bn_vec_.begin(), Bn_vec_.end(), initial_gains.Bn.begin());
            initial_gains.version = 1;
            gain_buffer_.writeFromNonRT(initial_gains);

            parameters_callback_handle_ = node_->add_on_set_parameters_callback(
                [this](std::vector<rclcpp::Parameter> parameters) -> rcl_interfaces::msg::SetParametersResult
                {
                    auto snapshot_ptr = gain_buffer_.readFromNonRT();
                    GainSnapshot gains = snapshot_ptr ? *snapshot_ptr : GainSnapshot{};
                    bool valid = true;
                    bool changed = false;
                    for (const auto &parameter : parameters)
                    {
                        if (parameter.get_name() == "Kx")
                        {
                            changed = true;
                            valid = copy_gain_parameter(parameter.as_double_array(), gains.Kx) && valid;
                        }
                        else if (parameter.get_name() == "Bx")
                        {
                            changed = true;
                            valid = copy_gain_parameter(parameter.as_double_array(), gains.Bx) && valid;
                        }
                        else if (parameter.get_name() == "Kn")
                        {
                            changed = true;
                            valid = copy_gain_parameter(parameter.as_double_array(), gains.Kn) && valid;
                        }
                        else if (parameter.get_name() == "Bn")
                        {
                            changed = true;
                            valid = copy_gain_parameter(parameter.as_double_array(), gains.Bn) && valid;
                        }
                    }
                    valid = valid && gains.Kx[5] >= Kz_min_ && gains.Kx[5] <= Kz_max_;
                    auto result = rcl_interfaces::msg::SetParametersResult();
                    result.successful = valid;
                    if (changed && valid)
                    {
                        ++gains.version;
                        gain_buffer_.writeFromNonRT(gains);
                    }
                    if (!result.successful)
                        result.reason = "gain arrays must be finite/nonnegative with Kx[5] inside Kz bounds";
                    return result;
                });

            return CallbackReturn::SUCCESS;
        }

        void saturate_torque(
            Eigen::Ref<const Eigen::VectorXd> tau_d_calculated,
            Eigen::Ref<const Eigen::VectorXd> tau_J_d,
            double max_delta, Eigen::Ref<Eigen::VectorXd> output)
        {
            for (int i = 0; i < dof_; i++)
            {
                double difference = tau_d_calculated[i] - tau_J_d[i];
                output[i] = tau_J_d[i] + std::clamp(difference, -max_delta, max_delta);
            }
        }

        CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            time_ = 0;
            traj_time_ = 0;
            force_int_z_ = 0.0;
            force_err_z_prev_ = 0.0;
            force_err_window_.assign(static_cast<std::size_t>(integral_window_size_), 0.0);
            force_window_index_ = 0;
            force_window_count_ = 0;
            is_contact_established_ = false; // 重置接触状态
            safety_stop_requested_.store(false, std::memory_order_release);
            over_force_count_ = 0;
            active_goal_generation_ = 0;
            applied_gain_version_ = 0;
            have_previous_update_time_ = false;
            robot_state_publish_count_ = 0;

            real_time_buffer_.reset();
            const std::vector<double> &tau_vec = state_->get<double>("torque");
            tau_d = Eigen::Map<const Eigen::VectorXd>(tau_vec.data(), dof_);
            f_filter_.reset();
            F_imp_.setZero();

            robot_state_publisher_ = node_->create_publisher<robot_control_msgs::msg::RobotState>("robot_states", rclcpp::SensorDataQoS());
            real_time_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<robot_control_msgs::msg::RobotState>>(robot_state_publisher_);
            real_time_publisher_->msg_.robot_state.resize(28, 0.0);

            const std::vector<double> &q_vec = state_->get<double>("position");
            qd_ = Eigen::Map<const Eigen::VectorXd>(q_vec.data(), dof_).eval();
            for (std::size_t i = 0; i < q_snapshot_.size(); ++i)
                q_snapshot_[i].store(q_vec[i], std::memory_order_relaxed);
            dqd_ = Eigen::VectorXd::Zero(dof_);
            ddqd_ = Eigen::VectorXd::Zero(dof_);

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
            tau_requested_ = Eigen::VectorXd::Zero(dof_);

            M_.resize(dof_, dof_);
            C_.resize(dof_, dof_);
            Jb_.resize(6, dof_);
            dJb_.resize(6, dof_);
            dM_.resize(dof_, dof_);
            g_.resize(dof_);
            // dynamics_workspace_.resize(dof_);

            sensor_cog_vec_ = {0.000, 0.000, 0.0029};
            sensor_offset_vec_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            T_sensor_ = Eigen::Matrix4d::Identity();
            T_sensor_ << 1, 0, 0, 0,
                0, -1, 0, 0,
                0, 0, -1, 0,
                0, 0, 0, 1;

            auto handle_goal = [this](const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const ACTION::Goal> goal)
            {
                const auto &data = goal->target_position.data;
                if (data.size() < 7 || data.size() % 7 != 0)
                    return rclcpp_action::GoalResponse::REJECT;
                double previous_time = 0.0;
                for (std::size_t i = 0; i < data.size(); i += 7)
                {
                    if (!std::all_of(data.begin() + i, data.begin() + i + 7,
                                     [](double value)
                                     { return std::isfinite(value); }) ||
                        data[i] <= previous_time)
                        return rclcpp_action::GoalResponse::REJECT;
                    previous_time = data[i];
                }
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            };

            auto handle_cancel = [](const std::shared_ptr<GoalHandle> /*goal_handle*/)
            {
                return rclcpp_action::CancelResponse::ACCEPT;
            };

            auto handle_accepted = [this](std::shared_ptr<GoalHandle> goal_handle)
            {
                auto trajectory = std::make_shared<robot_math::PiecewiseCartesianTrajectory>();
                std::vector<double> q_current(q_snapshot_.size(), 0.0);
                for (std::size_t i = 0; i < q_snapshot_.size(); ++i)
                    q_current[i] = q_snapshot_[i].load(std::memory_order_relaxed);
                Eigen::Matrix4d current_transform;
                robot_math::forward_kinematics(robot_, q_current, current_transform);
                const std::vector<double> pose_current = robot_math::tform_to_pose(current_transform);

                std::vector<double> full_traj_data;
                full_traj_data.reserve(7 + goal_handle->get_goal()->target_position.data.size());
                full_traj_data.push_back(0.0); // t = 0
                for (int i = 0; i < 6; ++i)
                {
                    full_traj_data.push_back(pose_current[i]);
                }

                // 拼接客户端发来的轨迹目标
                const auto &goal_data = goal_handle->get_goal()->target_position.data;
                full_traj_data.insert(full_traj_data.end(), goal_data.begin(), goal_data.end());

                // HUMBLE-FIX 20: Validate before publishing and let the RT thread
                // reset all trajectory/contact state when it sees a new generation.
                if (!trajectory->set_traj(full_traj_data))
                {
                    auto result = std::make_shared<ACTION::Result>();
                    result->success = false;
                    goal_handle->abort(result);
                    return;
                }

                // HUMBLE-FIX 31: Allocate the action result and wrapper only in the
                // executor callback. The RT thread merely requests a state change;
                // the monitor timer performs the ROS Action API call.
                const auto previous_command = real_time_buffer_.readFromNonRT();
                if (previous_command && previous_command->goal_handle &&
                    previous_command->goal_handle->valid() &&
                    previous_command->goal_handle->gh_->is_active())
                {
                    previous_command->goal_handle->preallocated_result_->success = false;
                    previous_command->goal_handle->setAborted(
                        previous_command->goal_handle->preallocated_result_);
                    previous_command->goal_handle->runNonRealtime();
                }

                auto preallocated_result = std::make_shared<ACTION::Result>();
                auto realtime_goal = std::make_shared<RealtimeGoalHandle>(
                    goal_handle, preallocated_result, nullptr, node_->get_logger());
                realtime_goal->execute();
                realtime_goal->runNonRealtime();
                const auto generation = goal_generation_.fetch_add(1, std::memory_order_relaxed) + 1;
                real_time_buffer_.writeFromNonRT({realtime_goal, trajectory, generation});
            };

            call_back_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
            this->action_server_ = rclcpp_action::create_server<ACTION>(
                node_, "~/goal", handle_goal, handle_cancel, handle_accepted,
                rcl_action_server_get_default_options(), call_back_group_);
            action_monitor_timer_ = node_->create_wall_timer(
                std::chrono::milliseconds(20),
                [this]()
                {
                    const auto command = real_time_buffer_.readFromNonRT();
                    if (command && command->goal_handle)
                        command->goal_handle->runNonRealtime();
                },
                call_back_group_);

            force_ = Eigen::Vector6d::Zero();
            dq_ = Eigen::Vector7d::Zero();
            q_ = Eigen::Vector7d::Zero();
            data_logger_ = std::make_unique<DataLogger>(
                std::initializer_list<DataInfo>{
                    DATA_WRAPPER(time_),
                    DATA_WRAPPER(period_log_),
                    DATA_WRAPPER(host_period_log_),
                    DATA_WRAPPER(cal_time_),
                    DATA_WRAPPER(command_success_log_),
                    DATA_WRAPPER(force_target_log_),
                    DATA_WRAPPER(F_imp_(5)),
                    DATA_WRAPPER(force_(2)),
                    // DATA_WRAPPER(pose_),
                    // DATA_WRAPPER(tau_d),
                    DATA_WRAPPER(Kx_(5)),
                    DATA_WRAPPER(Bx_(5)),
                    // DATA_WRAPPER(tau_fric_ff_),
                    // DATA_WRAPPER(dq_),
                    // DATA_WRAPPER(tau_base_),
                    DATA_WRAPPER(xe_),
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
                static_cast<std::size_t>(std::max(1, update_rate_)) * 120U);

            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            action_monitor_timer_.reset();
            const auto command = real_time_buffer_.readFromNonRT();
            if (command && command->goal_handle && command->goal_handle->valid() &&
                command->goal_handle->gh_->is_active())
            {
                command->goal_handle->preallocated_result_->success = false;
                command->goal_handle->setAborted(command->goal_handle->preallocated_result_);
                command->goal_handle->runNonRealtime();
            }
            real_time_buffer_.reset();
            action_server_ = nullptr;
            real_time_publisher_ = nullptr;
            robot_state_publisher_ = nullptr;
            return CallbackReturn::SUCCESS;
        }

        bool requests_stop() const noexcept override
        {
            return safety_stop_requested_.load(std::memory_order_acquire);
        }

        void update(const rclcpp::Time &t, const rclcpp::Duration &period) override
        {
            const double raw_dt = period.seconds();
            const double dt = std::clamp(raw_dt, 2.0e-4, 5.0e-3);
            time_ += std::max(0.0, raw_dt);
            period_log_ = raw_dt;
            const auto start_time = std::chrono::steady_clock::now();
            if (have_previous_update_time_)
                host_period_log_ = std::chrono::duration<double>(start_time - previous_update_time_).count();
            else
                host_period_log_ = raw_dt;
            previous_update_time_ = start_time;
            have_previous_update_time_ = true;

            std::vector<double> &tau_cmd_vec = command_->get<double>("torque");
            const std::vector<double> &q_vec = state_->get<double>("position");
            const std::vector<double> &dq_vec = state_->get<double>("velocity");
            auto &force_vec = com_state_->at("ft_sensor")->get<double>("force");
            command_->get<int>("mode")[0] = 3;
            force_ = Eigen::Map<const Eigen::Vector6d>(force_vec.data());
            for (std::size_t i = 0; i < q_snapshot_.size(); ++i)
                q_snapshot_[i].store(q_vec[i], std::memory_order_relaxed);

            command_success_log_ = 0.0;
            const auto &double_states = state_->get<double>();
            const auto success_it = double_states.find("success");
            if (success_it != double_states.end() && !success_it->second.empty())
                command_success_log_ = success_it->second.front();

            dq_ = Eigen::Map<const Eigen::Vector7d>(dq_vec.data());
            q_ = Eigen::Map<const Eigen::Vector7d>(q_vec.data());
            // Eigen::Matrix4d T = robot_math::pose_to_tform(pose_vec);
            // R_ = T.block(0, 0, 3, 3);
            // p_ = T.block(0, 3, 3, 1);

            Eigen::Map<const Eigen::VectorXd> q(q_vec.data(), dof_);
            Eigen::Map<const Eigen::VectorXd> dq(dq_vec.data(), dof_);
            Eigen::Map<Eigen::VectorXd> tau_cmd(tau_cmd_vec.data(), dof_);
            std::fill(tau_cmd_vec.begin(), tau_cmd_vec.end(), 0);

            const auto gains = gain_buffer_.readFromRT();
            if (gains && gains->version != applied_gain_version_)
            {
                std::copy(gains->Kx.begin(), gains->Kx.end(), Kx_vec_.begin());
                std::copy(gains->Bx.begin(), gains->Bx.end(), Bx_vec_.begin());
                std::copy(gains->Kn.begin(), gains->Kn.end(), Kn_vec_.begin());
                std::copy(gains->Bn.begin(), gains->Bn.end(), Bn_vec_.begin());
                applied_gain_version_ = gains->version;
            }
            Kx_ = Eigen::Map<Eigen::VectorXd>(Kx_vec_.data(), 6);
            Bx_ = Eigen::Map<Eigen::VectorXd>(Bx_vec_.data(), 6);
            Kn_ = Eigen::Map<Eigen::VectorXd>(Kn_vec_.data(), dof_);
            Bn_ = Eigen::Map<Eigen::VectorXd>(Bn_vec_.data(), dof_);

            m_c_g_matrix(
                robot_, q_vec, dq_vec, M_, C_, g_, Jb_, dJb_, dM_, dTb_, Tb_);
            R_ = Tb_.block(0, 0, 3, 3);
            p_ = Tb_.block(0, 3, 3, 1);
            Eigen::Matrix4d T1;
            robot_math::forward_kinematics(robot_, q_vec, T1);

            Eigen::Vector6d raw_compensated = robot_math::get_ext_force(
                force_vec,
                sensor_mass_kg_,
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

            const bool severe_over_force =
                std::abs(force_(2)) > 1.5 * F_stop_z_ || force_.head<3>().norm() > 1.5 * F_stop_norm_;
            const bool over_force =
                std::abs(force_(2)) > F_stop_z_ || force_.head<3>().norm() > F_stop_norm_;
            over_force_count_ = over_force ? over_force_count_ + 1 : 0;
            if (severe_over_force || over_force_count_ >= force_stop_samples_)
                safety_stop_requested_.store(true, std::memory_order_release);

            const auto command_ptr = real_time_buffer_.readFromRT();
            const GoalCommand command = command_ptr ? *command_ptr : GoalCommand{};
            auto goal_handle = command.goal_handle;
            auto trajectory = command.trajectory;

            if (command.generation != 0 && command.generation != active_goal_generation_)
            {
                active_goal_generation_ = command.generation;
                traj_time_ = 0.0;
                is_contact_established_ = false;
                force_int_z_ = 0.0;
                force_err_z_prev_ = 0.0;
                std::fill(force_err_window_.begin(), force_err_window_.end(), 0.0);
                force_window_index_ = 0;
                force_window_count_ = 0;
            }

            dVd.setZero();

            if (goal_handle && trajectory && goal_handle->valid() && goal_handle->gh_->is_active())
            {
                if (goal_handle->gh_->is_canceling())
                {
                    goal_handle->preallocated_result_->success = false;
                    goal_handle->setCanceled(goal_handle->preallocated_result_);
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
                        goal_handle->preallocated_result_->success = true;
                        goal_handle->setSucceeded(goal_handle->preallocated_result_);
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

            if (!is_contact_established_)
            {
                if (std::abs(force_(2)) >= std::abs(F_des_z_))
                {
                    is_contact_established_ = true;
                    contact_loss_count_ = 0;
                    force_int_z_ = 0.0;
                    force_err_z_prev_ = 0.0;
                }
            }
            else if (std::abs(force_(2)) < 0.5 * std::abs(F_des_z_))
            {
                if (++contact_loss_count_ >= 20)
                {
                    is_contact_established_ = false;
                    contact_loss_count_ = 0;
                }
            }
            else
            {
                contact_loss_count_ = 0;
            }

            if (is_contact_established_ && std::abs(xe_z) > 1e-3 && xe_z * F_des_z_ > 0) // 只有当位置误差不小且力误差与位置误差同号时才优化刚度
            {
                double F_ext_z = force_(2);
                double F_err_z = F_des_z_ - F_ext_z;
                double current_integral_step = F_err_z * dt;
                if (!force_err_window_.empty())
                {
                    if (force_window_count_ == force_err_window_.size())
                        force_int_z_ -= force_err_window_[force_window_index_];
                    else
                        ++force_window_count_;
                    force_err_window_[force_window_index_] = current_integral_step;
                    force_int_z_ += current_integral_step;
                    force_window_index_ = (force_window_index_ + 1) % force_err_window_.size();
                }
                double dF_err_z = 0.0;
                if (dt > 1e-6 && force_window_count_ > 1)
                    dF_err_z = (F_err_z - force_err_z_prev_) / dt;
                double F_com_z = Kp_f_ * F_err_z + Ki_f_ * force_int_z_ + Kd_f_ * dF_err_z;
                force_err_z_prev_ = F_err_z;
                force_target_log_ = F_des_z_ + F_com_z;
                const double dx_B = -B_z * dxe_z;
                const double desired_stiffness = solve_closed_form_stiffness(
                    xe_z, dx_B, force_target_log_, Q_weight_, R_weight_,
                    Kz_min_, Kz_max_, F_max_z_);
                Kx_(5) = move_towards(
                    Kx_(5), desired_stiffness, max_stiffness_rate_ * dt);
                Kx_vec_[5] = Kx_(5);
                Bx_(5) = 1.42 * std::sqrt(std::max(0.0, Kx_(5)));
            }
            else
            {
                force_target_log_ = F_des_z_;
                Kx_(5) = move_towards(Kx_(5), Kz_max_, max_stiffness_rate_ * dt);
                Kx_vec_[5] = Kx_(5);
                Bx_(5) = 1.42 * std::sqrt(std::max(0.0, Kx_(5)));
                if (!is_contact_established_)
                {
                    force_int_z_ = 0.0;
                    force_err_z_prev_ = 0.0;
                    std::fill(force_err_window_.begin(), force_err_window_.end(), 0.0);
                    force_window_index_ = 0;
                    force_window_count_ = 0;
                }
            }

            Eigen::MatrixXd Lambda_inv = robot_math::A_x_inv(Jb_, M_);
            ddxc_ = ddxd_ + Lambda_inv * (robot_math::Mu_x_X(Jb_, M_, dJb_, C_, dxe_) + Bx_.asDiagonal() * dxe_ + Kx_.asDiagonal() * xe_);
            tau_task_ = M_ * robot_math::J_sharp(Jb_, M_) * (ddxc_ - dJb_ * dq);
            Eigen::LDLT<Eigen::MatrixXd> ldlt(M_);
            tau_null_ = M_ * robot_math::null_proj(Jb_, M_, ldlt.solve(Bn_.asDiagonal() * (-dq)));
            F_imp_ = Lambda_inv.ldlt().solve(ddxc_ - dJb_ * dq);
            tau_requested_ = tau_task_ + tau_null_ + C_ * dq;
            const auto &tau_reference_vec = state_->get<double>("torque");
            const Eigen::Map<const Eigen::VectorXd> tau_reference(
                tau_reference_vec.data(), dof_);
            if (safety_stop_requested_.load(std::memory_order_acquire))
            {
                tau_requested_.setZero();
                if (goal_handle && goal_handle->valid() && goal_handle->gh_->is_active())
                {
                    goal_handle->preallocated_result_->success = false;
                    goal_handle->setAborted(goal_handle->preallocated_result_);
                }
            }
            saturate_torque(
                tau_requested_, tau_reference, max_torque_rate_ * dt, tau_cmd);
            tau_d = tau_cmd;

            publish_robot_state(t, q_vec, dq_vec, force_);

            cal_time_ = std::chrono::duration<double>(
                            std::chrono::steady_clock::now() - start_time)
                            .count();
            data_logger_->record();
        }

    private:
        static double move_towards(double current, double target, double max_step)
        {
            return current + std::clamp(target - current, -max_step, max_step);
        }

        static double solve_closed_form_stiffness(
            double xe, double dx_B, double force_target, double Q, double R,
            double K_min, double K_max, double F_max)
        {
            const double denominator = Q * xe * xe + R;
            double unconstrained = K_min;
            if (denominator > std::numeric_limits<double>::epsilon())
                unconstrained = (Q * xe * (force_target + dx_B) + R * K_min) / denominator;

            double feasible_lower = K_min;
            double feasible_upper = K_max;
            if (std::abs(xe) > 1.0e-12)
            {
                const double force_bound_a = (dx_B - F_max) / xe;
                const double force_bound_b = (dx_B + F_max) / xe;
                feasible_lower = std::max(K_min, std::min(force_bound_a, force_bound_b));
                feasible_upper = std::min(K_max, std::max(force_bound_a, force_bound_b));
            }
            else if (std::abs(dx_B) > F_max)
            {
                feasible_lower = K_max + 1.0;
                feasible_upper = K_max;
            }

            if (feasible_lower <= feasible_upper)
                return std::clamp(unconstrained, feasible_lower, feasible_upper);

            // Infeasible force/stiffness bounds: choose the stiffness boundary
            // producing the smallest estimated absolute force.
            const double force_at_min = std::abs(K_min * xe - dx_B);
            const double force_at_max = std::abs(K_max * xe - dx_B);
            return force_at_min <= force_at_max ? K_min : K_max;
        }

        void publish_robot_state(const rclcpp::Time &t,
                                 const std::vector<double> &q,
                                 const std::vector<double> &dq,
                                 const Eigen::Vector6d &force // <--- 修改 1：参数类型改为 Eigen::Vector6d
        )
        {
            if (!real_time_publisher_)
                return;
            if (++robot_state_publish_count_ < static_cast<std::size_t>(std::max(1, update_rate_ / 100)))
                return;
            robot_state_publish_count_ = 0;
            if (real_time_publisher_->trylock())
            {
                auto &msg = real_time_publisher_->msg_;
                msg.header.stamp = t;
                std::copy(q.begin(), q.end(), msg.robot_state.begin());
                std::copy(dq.begin(), dq.end(), msg.robot_state.begin() + 7);
                std::copy(force.data(), force.data() + 6, msg.robot_state.begin() + 14);
                real_time_publisher_->unlockAndPublish();
            }
        }

    protected:
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;
        rclcpp::Publisher<robot_control_msgs::msg::RobotState>::SharedPtr robot_state_publisher_;
        std::shared_ptr<realtime_tools::RealtimePublisher<robot_control_msgs::msg::RobotState>> real_time_publisher_;
        rclcpp::CallbackGroup::SharedPtr call_back_group_;
        rclcpp_action::Server<ACTION>::SharedPtr action_server_;
        rclcpp::TimerBase::SharedPtr action_monitor_timer_;
        realtime_tools::RealtimeBuffer<BufferType> real_time_buffer_;
        realtime_tools::RealtimeBuffer<GainSnapshot> gain_buffer_;

        int dof_;
        double time_, traj_time_;
        Eigen::MatrixXd M_, C_, Jb_, dJb_, dM_;
        // robot_math::MCGWorkspace dynamics_workspace_;
        Eigen::Vector6d force_;
        Eigen::VectorXd g_;
        Eigen::Matrix4d Tb_, dTb_;
        Eigen::VectorXd Kx_, Bx_, Kn_, Bn_;
        Eigen::VectorXd tau_cmd_, tau_requested_, tau_task_, tau_null_;
        Eigen::VectorXd qd_, dqd_, ddqd_, qe_, dqe_, dq_, q_;
        Eigen::Vector6d xe_, dxe_, ddxd_, ddxc_, dVd;
        Eigen::Matrix3d Rd_, R_;
        Eigen::Vector3d pd_, p_, wd_, vd_;
        Eigen::Vector7d tau_d;
        Eigen::Vector6d F_imp_;
        std::vector<double> sensor_cog_vec_;
        std::vector<double> sensor_offset_vec_;
        double sensor_mass_kg_;
        Eigen::Matrix4d T_sensor_;
        double F_des_z_, F_max_z_;
        double F_stop_z_, F_stop_norm_;
        double Kz_min_, Kz_max_;
        double max_stiffness_rate_, max_torque_rate_;
        double Q_weight_, R_weight_;
        double Kp_f_, Ki_f_, Kd_f_, force_int_z_;
        double force_err_z_prev_;
        double cal_time_{0.0};
        double period_log_{0.0};
        double host_period_log_{0.0};
        double command_success_log_{0.0};
        double force_target_log_{0.0};
        std::unique_ptr<DataLogger> data_logger_;
        std::vector<double> Kx_vec_, Bx_vec_, Kn_vec_, Bn_vec_;
        robot_math::MovingFilter<double> f_filter_;
        bool is_contact_established_{false};
        std::vector<double> force_err_window_;
        int integral_window_size_{50};
        std::size_t force_window_index_{0};
        std::size_t force_window_count_{0};
        int contact_loss_count_{0};
        int force_stop_samples_{5};
        int over_force_count_{0};
        std::atomic_bool safety_stop_requested_{false};
        std::atomic<std::uint64_t> goal_generation_{0};
        std::uint64_t active_goal_generation_{0};
        std::uint64_t applied_gain_version_{0};
        std::array<std::atomic<double>, 7> q_snapshot_{};
        std::chrono::steady_clock::time_point previous_update_time_{};
        bool have_previous_update_time_{false};
        std::size_t robot_state_publish_count_{0};
    };
} // namespace controllers

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(controllers::VariableImpedanceController, controller_interface::ControllerInterface)
