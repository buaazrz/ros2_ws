#pragma once
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/empty.hpp"
#include "urdf/model.h"
#include "robot_math/robot_math.hpp"
#include "robot_hardware_interface/robot_interface.hpp"
#include "robot_controller_interface/controller_interface.hpp"
#include <pluginlib/class_loader.hpp>
#include "robot_control_msgs/srv/control_command.hpp"
#include <functional>
#include <chrono>
#include <atomic>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include "yaml-cpp/yaml.h"
namespace control_node
{

    class ControlManager : public rclcpp::Node
    {
    public:
        using ControllerPtr = controller_interface::ControllerInterface::SharedPtr;
        using ControllerList = std::vector<ControllerPtr>;
        using ControllerListSnapshot = std::shared_ptr<const ControllerList>;

        ControlManager(std::shared_ptr<rclcpp::Executor> executor,
                       const std::string &node_name,
                       const std::string &name_space,
                       const rclcpp::NodeOptions &option);
        ~ControlManager();
        int get_update_rate();
        void control_loop();
        void prepare_loop();
        void end_loop();
        void interrupt();
        void command_callback(const std::shared_ptr<robot_control_msgs::srv::ControlCommand::Request> request,
                              std::shared_ptr<robot_control_msgs::srv::ControlCommand::Response> response);
        bool activate_controller(const std::string &controller_name);
        bool deactivate_controller();
        bool add_secondary_controller(const std::string &controller_name);
        bool remove_secondary_controller(const std::string &controller_name);
        bool clear_secondary_controllers();
        bool load_controller(const std::string &controller_name);
        void shutdown_robot();
        void read(const rclcpp::Time &t, const rclcpp::Duration &period);
        void update(const rclcpp::Time &t, const rclcpp::Duration &period);
        void write(const rclcpp::Time &t, const rclcpp::Duration &period);
        void start_simulation(double time = 10.0); // seconds
        std::vector<double> simulation_controller(double t, const std::vector<double> &x, const Eigen::MatrixXd &fext);
        Eigen::MatrixXd simulation_external_force(double t);
        void simulation_observer(const std::vector<double> &x, double t);
        bool is_simulation();
        bool is_keep_running() const;

    protected:
        pluginlib::UniquePtr<pluginlib::ClassLoader<hardware_interface::RobotInterface>> robot_loader_;
        pluginlib::UniquePtr<pluginlib::ClassLoader<controller_interface::ControllerInterface>> controller_loader_;
        std::shared_ptr<hardware_interface::RobotInterface> robot_;
        ControllerList controllers_;
        std::string default_controller_;
        ControllerPtr active_controller_;
        // HUMBLE-FIX 03: Humble RealtimeBox::get() takes a blocking mutex. Publish
        // immutable controller snapshots from executor callbacks and only use
        // RealtimeBuffer::readFromRT() in the 1 kHz path.
        realtime_tools::RealtimeBuffer<ControllerPtr> active_controller_buffer_;
        realtime_tools::RealtimeBuffer<ControllerListSnapshot> secondary_controllers_buffer_;
        ControllerList secondary_controllers_non_rt_;
        std::mutex controller_management_mutex_;
        std::shared_ptr<rclcpp::Executor> executor_;
        int update_rate_;
        std::string robot_description_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
        std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>> real_time_publisher_;
        rclcpp::Service<robot_control_msgs::srv::ControlCommand>::SharedPtr service_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_service_;
        bool is_simulation_;
        bool is_sim_real_time_;
        bool is_publish_joint_state_;
        std::atomic_bool running_;
        std::atomic_bool control_loop_active_;
        std::size_t joint_state_publish_divider_;
        std::size_t joint_state_publish_count_;
        rclcpp::Time sim_start_time_;
        std::atomic_bool keep_running_;
        std::shared_ptr<YAML::Node> config_;
    };

}
