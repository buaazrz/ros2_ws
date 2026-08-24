#include "robot_hardware_interface/sensor_interface.hpp"

#include <algorithm>
#include <pluginlib/class_list_macros.hpp>

namespace hardwares
{

class MujocoFTSensor : public hardware_interface::SensorInterface
{
public:
    MujocoFTSensor() = default;
    ~MujocoFTSensor() override = default;

    CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override
    {
        (void)previous_state;

        auto & wrench = state_.get<double>("force");

        if (wrench.size() != 6)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "MujocoFTSensor: state interface 'force' must have length 6, "
                "but got %zu",
                wrench.size());

            return CallbackReturn::FAILURE;
        }

        std::fill(wrench.begin(), wrench.end(), 0.0);

        RCLCPP_INFO(
            node_->get_logger(),
            "MujocoFTSensor activated: passive MuJoCo wrench container");

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override
    {
        auto & wrench = state_.get<double>("force");
        std::fill(wrench.begin(), wrench.end(), 0.0);

        // 只执行基类清理，不存在网络线程
        return hardware_interface::SensorInterface::on_deactivate(
            previous_state);
    }
};

}  // namespace hardwares

PLUGINLIB_EXPORT_CLASS(
    hardwares::MujocoFTSensor,
    hardware_interface::SensorInterface)