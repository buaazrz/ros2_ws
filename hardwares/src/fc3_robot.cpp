#include "robot_hardware_interface/robot_interface.hpp"
#include "robot_math/robot_math.hpp"

#include <franka/active_control_base.h>
#include <franka/active_motion_generator.h>
#include <franka/active_torque_control.h>
#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace hardwares
{
    class FC3Robot : public hardware_interface::RobotInterface
    {
    public:
        FC3Robot()
            : stop_requested_(false), last_period_ns_(1'000'000), active_mode_(-1),
              fault_code_(0),
              compute_model_state_(false), set_payload_(false), payload_mass_kg_(0.0),
              payload_cog_{{0.0, 0.0, 0.0}},
              payload_inertia_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
              collision_joint_torque_thresholds_{{200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0}},
              collision_cartesian_force_thresholds_{{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}
        {
        }

        ~FC3Robot() override
        {
            end_control();
        }

        // HUMBLE-FIX 08: FCI readOnce() is the loop clock. ControlManager must not
        // add a second sleep_until() to this plugin.
        bool is_hardware_paced() const noexcept override { return true; }

        rclcpp::Duration get_last_read_period() const override
        {
            return rclcpp::Duration::from_nanoseconds(
                last_period_ns_.load(std::memory_order_relaxed));
        }

        bool is_stop() override
        {
            return stop_requested_.load(std::memory_order_acquire);
        }

        void request_stop() override
        {
            stop_requested_.store(true, std::memory_order_release);
        }

        bool begin_control() override
        {
            if (!franka_robot_ || control_)
                return control_ != nullptr;

            const auto &mode_interfaces = command_.get<int>();
            const auto mode_it = mode_interfaces.find("mode");
            if (mode_it == mode_interfaces.end() || mode_it->second.empty())
            {
                RCLCPP_ERROR(node_->get_logger(), "Franka command interface 'mode' is missing");
                return false;
            }

            const int requested_mode = mode_it->second.front();
            try
            {
                switch (requested_mode)
                {
                case 0:
                    control_ = franka_robot_->startCartesianPoseControl(
                        research_interface::robot::Move::ControllerMode::kCartesianImpedance);
                    break;
                case 1:
                    control_ = franka_robot_->startJointPositionControl(
                        research_interface::robot::Move::ControllerMode::kJointImpedance);
                    break;
                case 3:
                    control_ = franka_robot_->startTorqueControl();
                    break;
                default:
                    RCLCPP_ERROR(
                        node_->get_logger(), "unsupported Franka control mode: %d", requested_mode);
                    return false;
                }
                active_mode_ = requested_mode;
                fault_code_.store(0, std::memory_order_relaxed);
                stop_requested_.store(false, std::memory_order_release);
                return true;
            }
            catch (const franka::Exception &e)
            {
                RCLCPP_ERROR(node_->get_logger(), "failed to start Franka control: %s", e.what());
                control_.reset();
                active_mode_ = -1;
                stop_requested_.store(true, std::memory_order_release);
                return false;
            }
        }

        void end_control() override
        {
            if (control_)
            {
                try
                {
                    franka_robot_->stop();
                }
                catch (const franka::Exception &e)
                {
                    if (node_)
                        RCLCPP_ERROR(node_->get_logger(), "failed to stop Franka control: %s", e.what());
                }
                control_.reset();
                active_mode_ = -1;
            }
            const int fault = fault_code_.exchange(0, std::memory_order_relaxed);
            if (fault != 0 && node_)
                RCLCPP_ERROR(node_->get_logger(), "Franka control stream stopped after RT fault code %d", fault);
        }

        void write(const rclcpp::Time &t, const rclcpp::Duration &period) override
        {
            hardware_interface::RobotInterface::write(t, period);
            if (!control_ || stop_requested_.load(std::memory_order_acquire))
                return;

            const auto &mode_interfaces = command_.get<int>();
            const auto mode_it = mode_interfaces.find("mode");
            if (mode_it == mode_interfaces.end() || mode_it->second.empty() ||
                mode_it->second.front() != active_mode_)
            {
                // HUMBLE-FIX 09: Never stop/restart FCI from the 1 kHz write path.
                // A controller-mode change must go through deactivate/activate.
                fault_code_.store(3, std::memory_order_relaxed);
                stop_requested_.store(true, std::memory_order_release);
                return;
            }

            try
            {
                switch (active_mode_)
                {
                case 0:
                {
                    const auto &cmd_pose = command_.get<double>("pose");
                    if (cmd_pose.size() != 6)
                        throw std::runtime_error("Franka pose command must contain 6 values");
                    const Eigen::Matrix4d desired_pose = robot_math::pose_to_tform(cmd_pose);
                    std::array<double, 16> pose_array{};
                    Eigen::Map<Eigen::Matrix4d>(pose_array.data()) = desired_pose;
                    control_->writeOnce(franka::CartesianPose(pose_array));
                    break;
                }
                case 1:
                {
                    const auto &cmd = command_.get<double>("position");
                    if (cmd.size() != 7)
                        throw std::runtime_error("Franka position command must contain 7 values");
                    const std::array<double, 7> position_command{{
                        cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], cmd[6]}};
                    control_->writeOnce(franka::JointPositions(position_command));
                    break;
                }
                case 3:
                {
                    const auto &cmd = command_.get<double>("torque");
                    if (cmd.size() != 7)
                        throw std::runtime_error("Franka torque command must contain 7 values");
                    const std::array<double, 7> torque_command{{
                        cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], cmd[6]}};
                    control_->writeOnce(franka::Torques(torque_command));
                    break;
                }
                default:
                    fault_code_.store(3, std::memory_order_relaxed);
                    stop_requested_.store(true, std::memory_order_release);
                    break;
                }
            }
            catch (const std::exception &e)
            {
                (void)e;
                // HUMBLE-FIX 34: Do not format/log exceptions from the 1 kHz path.
                // Report a compact fault code after the stream has stopped.
                fault_code_.store(2, std::memory_order_relaxed);
                stop_requested_.store(true, std::memory_order_release);
            }
        }

        void read(const rclcpp::Time &t, const rclcpp::Duration & /*period*/) override
        {
            if (!control_ || stop_requested_.load(std::memory_order_acquire))
                return;

            try
            {
                auto read_result = control_->readOnce();
                const auto period_ns = static_cast<std::int64_t>(
                    std::llround(read_result.second.toSec() * 1.0e9));
                if (period_ns > 0)
                    last_period_ns_.store(period_ns, std::memory_order_relaxed);
                update_state(read_result.first);

                // Read ATI/other attached sensors after the FCI sample so their
                // snapshot is as close as possible to this robot state.
                hardware_interface::RobotInterface::read(t, get_last_read_period());
            }
            catch (const franka::Exception &e)
            {
                (void)e;
                fault_code_.store(1, std::memory_order_relaxed);
                stop_requested_.store(true, std::memory_order_release);
            }
        }

        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override
        {
            if (RobotInterface::on_configure(previous_state) != CallbackReturn::SUCCESS)
                return CallbackReturn::FAILURE;

            if (get_dof() != 7)
            {
                RCLCPP_ERROR(node_->get_logger(), "FC3Robot requires a 7-DOF URDF chain, got %d", get_dof());
                return CallbackReturn::FAILURE;
            }

            node_->get_parameter_or<std::string>("robot_ip", robot_ip_, "");
            node_->get_parameter_or<bool>("compute_model_state", compute_model_state_, false);
            node_->get_parameter_or<bool>("set_payload", set_payload_, false);
            node_->get_parameter_or<double>("payload_mass_kg", payload_mass_kg_, 0.0);
            std::vector<double> payload_cog(payload_cog_.begin(), payload_cog_.end());
            std::vector<double> payload_inertia(payload_inertia_.begin(), payload_inertia_.end());
            std::vector<double> collision_joint_torque_thresholds(
                collision_joint_torque_thresholds_.begin(), collision_joint_torque_thresholds_.end());
            std::vector<double> collision_cartesian_force_thresholds(
                collision_cartesian_force_thresholds_.begin(), collision_cartesian_force_thresholds_.end());
            node_->get_parameter_or<std::vector<double>>("payload_cog", payload_cog, payload_cog);
            node_->get_parameter_or<std::vector<double>>(
                "payload_inertia", payload_inertia, payload_inertia);
            node_->get_parameter_or<std::vector<double>>(
                "collision_joint_torque_thresholds", collision_joint_torque_thresholds,
                collision_joint_torque_thresholds);
            node_->get_parameter_or<std::vector<double>>(
                "collision_cartesian_force_thresholds", collision_cartesian_force_thresholds,
                collision_cartesian_force_thresholds);

            const auto valid_positive = [](const std::vector<double> &values, std::size_t expected)
            {
                return values.size() == expected &&
                       std::all_of(values.begin(), values.end(), [](double value)
                                   { return std::isfinite(value) && value > 0.0; });
            };
            const auto valid_finite = [](const std::vector<double> &values, std::size_t expected)
            {
                return values.size() == expected &&
                       std::all_of(values.begin(), values.end(), [](double value)
                                   { return std::isfinite(value); });
            };
            if (robot_ip_.empty() || !valid_finite(payload_cog, 3) ||
                !valid_finite(payload_inertia, 9) ||
                !valid_positive(collision_joint_torque_thresholds, 7) ||
                !valid_positive(collision_cartesian_force_thresholds, 6) ||
                !std::isfinite(payload_mass_kg_) || payload_mass_kg_ < 0.0)
            {
                RCLCPP_ERROR(node_->get_logger(), "invalid Franka IP, payload, or collision parameters");
                return CallbackReturn::FAILURE;
            }
            std::copy(payload_cog.begin(), payload_cog.end(), payload_cog_.begin());
            std::copy(payload_inertia.begin(), payload_inertia.end(), payload_inertia_.begin());
            std::copy(
                collision_joint_torque_thresholds.begin(),
                collision_joint_torque_thresholds.end(),
                collision_joint_torque_thresholds_.begin());
            std::copy(
                collision_cartesian_force_thresholds.begin(),
                collision_cartesian_force_thresholds.end(),
                collision_cartesian_force_thresholds_.begin());

            try
            {
                franka_robot_ = std::make_shared<franka::Robot>(robot_ip_);
                franka_model_ = std::make_shared<franka::Model>(franka_robot_->loadModel());
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(
                    node_->get_logger(), "cannot connect to Franka at %s: %s",
                    robot_ip_.c_str(), e.what());
                return CallbackReturn::FAILURE;
            }
            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override
        {
            if (RobotInterface::on_activate(previous_state) != CallbackReturn::SUCCESS)
                return CallbackReturn::FAILURE;
            try
            {
                stop_requested_.store(false, std::memory_order_release);
                franka_robot_->automaticErrorRecovery();
                franka_robot_->setCollisionBehavior(
                    collision_joint_torque_thresholds_,
                    collision_joint_torque_thresholds_,
                    collision_joint_torque_thresholds_,
                    collision_joint_torque_thresholds_,
                    collision_cartesian_force_thresholds_,
                    collision_cartesian_force_thresholds_,
                    collision_cartesian_force_thresholds_,
                    collision_cartesian_force_thresholds_);
                if (set_payload_)
                    franka_robot_->setLoad(payload_mass_kg_, payload_cog_, payload_inertia_);
                franka_robot_->setJointImpedance({{3000, 3000, 3000, 2500, 2500, 200, 200}});
                franka_robot_->setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});

                // Populate a fresh state before the controller activation callback.
                update_state(franka_robot_->readOnce());
                hardware_interface::RobotInterface::read(
                    node_->now(), RobotInterface::get_last_read_period());
                return CallbackReturn::SUCCESS;
            }
            catch (const franka::Exception &e)
            {
                stop_requested_.store(true, std::memory_order_release);
                RobotInterface::on_deactivate(previous_state);
                RCLCPP_ERROR(node_->get_logger(), "Franka activation failed: %s", e.what());
                return CallbackReturn::FAILURE;
            }
        }

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override
        {
            end_control();
            return RobotInterface::on_deactivate(previous_state);
        }

        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override
        {
            end_control();
            franka_model_.reset();
            franka_robot_.reset();
            return RobotInterface::on_shutdown(previous_state);
        }

    private:
        void update_state(const franka::RobotState &state)
        {
            auto &interfaces = state_.get<double>();
            const auto copy_to_interface = [&interfaces](const std::string &name, const auto &source)
            {
                const auto it = interfaces.find(name);
                if (it == interfaces.end())
                    return;
                const auto count = std::min(it->second.size(), source.size());
                std::copy_n(source.begin(), count, it->second.begin());
            };

            copy_to_interface("T", state.O_T_EE);
            copy_to_interface("position", state.q);
            copy_to_interface("velocity", state.dq);
            copy_to_interface("torque", state.tau_J_d);
            copy_to_interface("external_torque", state.tau_ext_hat_filtered);
            const auto success_it = interfaces.find("success");
            if (success_it != interfaces.end() && !success_it->second.empty())
                success_it->second.front() = state.control_command_success_rate;

            if (compute_model_state_ && franka_model_)
            {
                copy_to_interface("m", franka_model_->mass(state));
                copy_to_interface("c", franka_model_->coriolis(state));
                copy_to_interface("g", franka_model_->gravity(state));
            }
        }

        std::string robot_ip_;
        std::shared_ptr<franka::Robot> franka_robot_;
        std::unique_ptr<franka::ActiveControlBase> control_;
        std::shared_ptr<franka::Model> franka_model_;
        std::atomic_bool stop_requested_;
        std::atomic<std::int64_t> last_period_ns_;
        int active_mode_;
        std::atomic_int fault_code_;
        bool compute_model_state_;
        bool set_payload_;
        double payload_mass_kg_;
        std::array<double, 3> payload_cog_;
        std::array<double, 9> payload_inertia_;
        std::array<double, 7> collision_joint_torque_thresholds_;
        std::array<double, 6> collision_cartesian_force_thresholds_;
    };

} // namespace hardwares

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(hardwares::FC3Robot, hardware_interface::RobotInterface)
