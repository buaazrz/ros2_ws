#include "control_node/control_manager.h"
#include "lifecycle_msgs/msg/state.hpp"
#include <boost/numeric/odeint.hpp>
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <algorithm>
#include <cmath>
#include <thread>
using namespace std::chrono_literals;
using namespace boost::numeric::odeint;
namespace control_node
{
    ControlManager::ControlManager(std::shared_ptr<rclcpp::Executor> executor,
                                   const std::string &node_name, const std::string &name_space, const rclcpp::NodeOptions &option)
        : rclcpp::Node(node_name, name_space, option),
          executor_(executor),
          running_(false),
          control_loop_active_(false),
          joint_state_publish_divider_(1),
          joint_state_publish_count_(0),
          keep_running_(true)
    {
        auto parameter_file = this->get_parameter_or<std::string>("parameters", "");
        if (!parameter_file.empty())
        {
            auto path = ament_index_cpp::get_package_share_directory("applications");
            if (!path.empty())
            {
                config_ = std::make_shared<YAML::Node>();
                *config_ = YAML::LoadFile(path + "/config/" + parameter_file);
                // auto matrix = (*config_)["matrix"];
                // for(auto&& row : matrix)
                // {
                //     for(auto&& col : row)
                //     {
                //         RCLCPP_INFO(this->get_logger(), "%f", col.as<double>());
                //     }
                // }
            }
        }
        update_rate_ = this->get_parameter_or<int>("update_rate", 1000);
        if (update_rate_ <= 0)
            throw std::invalid_argument("update_rate must be greater than zero");
        is_simulation_ = this->get_parameter_or<bool>("simulation", true);
        is_sim_real_time_ = this->get_parameter_or<bool>("sim_real_time", true);
        is_publish_joint_state_ = this->get_parameter_or<bool>("publish_joint_state", true);
        const int joint_state_publish_rate =
            this->get_parameter_or<int>("joint_state_publish_rate", std::min(update_rate_, 100));
        joint_state_publish_divider_ = static_cast<std::size_t>(
            std::max(1, update_rate_ / std::max(1, joint_state_publish_rate)));
        if (is_publish_joint_state_ || is_simulation_)
        {
            joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SensorDataQoS());
            real_time_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(joint_state_publisher_);
        }
        robot_description_ = this->get_parameter_or<std::string>("robot_description", "");
        if (robot_description_.empty())
            throw std::runtime_error("robot description file is empty!");

        std::string robot_class = this->get_parameter_or<std::string>("robot", "");
        std::vector<std::string> controller_class = this->get_parameter_or<std::vector<std::string>>("controllers", std::vector<std::string>());
        default_controller_ = this->get_parameter_or<std::string>("default_controller", "");
        robot_loader_ = std::make_unique<pluginlib::ClassLoader<hardware_interface::RobotInterface>>("robot_hardware_interface", "hardware_interface::RobotInterface");
        controller_loader_ = std::make_unique<pluginlib::ClassLoader<controller_interface::ControllerInterface>>("robot_controller_interface", "controller_interface::ControllerInterface");
        rclcpp::NodeOptions node_options;
        node_options.allow_undeclared_parameters(true);
        node_options.automatically_declare_parameters_from_overrides(true);
        try
        {
            robot_ = robot_loader_->createSharedInstance(robot_class);
            int pos = robot_class.rfind(":");
            auto robot_name = robot_class.substr(pos + 1);
            robot_->set_update_rate(update_rate_);
            if (!robot_->initialize(robot_name))
                throw std::runtime_error("failed to initialize robot plugin " + robot_class);
            auto nodes = robot_->get_all_nodes();
            for (auto &no : nodes)
                executor_->add_node(no);

            for (auto name : controller_class)
            {
                auto controller = controller_loader_->createSharedInstance(name);
                pos = name.rfind(":");
                name = name.substr(pos + 1);
                controller->loan_interface(update_rate_,
                                           &robot_->get_robot_math(),
                                           &robot_->get_command_interface(),
                                           &robot_->get_state_interface(),
                                           &robot_->get_com_command_interface(),
                                           &robot_->get_com_state_interface());
                controller->initialize(name);
                controllers_.push_back(controller);
                executor_->add_node(controller->get_node()->get_node_base_interface());
            }

            // HUMBLE-FIX 04: Allocate JointState storage once, before the RT loop.
            if (real_time_publisher_)
            {
                auto &msg = real_time_publisher_->msg_;
                msg.name = robot_->get_joint_names();
                const auto dof = static_cast<std::size_t>(robot_->get_dof());
                msg.position.resize(dof);
                msg.velocity.resize(dof);
                msg.effort.resize(dof);
            }
        }
        catch (pluginlib::PluginlibException &ex)
        {
            RCLCPP_INFO(this->get_logger(), "%s", ex.what());
            throw ex;
        }
        service_ = create_service<robot_control_msgs::srv::ControlCommand>("~/control_command",
                                                                           std::bind(&ControlManager::command_callback, this, std::placeholders::_1, std::placeholders::_2));

        active_controller_buffer_.writeFromNonRT(ControllerPtr{});
        secondary_controllers_buffer_.writeFromNonRT(
            std::make_shared<const ControllerList>(ControllerList{}));

        executor_->add_node(this->get_node_base_interface());
    }

    ControlManager::~ControlManager()
    {
    }
    bool ControlManager::remove_secondary_controller(const std::string &controller_name)
    {
        std::lock_guard<std::mutex> lock(controller_management_mutex_);
        auto name = controller_name;
        int pos = name.rfind(":");
        name = name.substr(pos + 1);
        auto it = std::find_if(
            secondary_controllers_non_rt_.begin(), secondary_controllers_non_rt_.end(), [=](auto &&v)
            { return v->get_node()->get_name() == name; });
        if (it == secondary_controllers_non_rt_.end())
            return false;
        if ((*it)->get_node_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
        {
            const auto state = (*it)->get_node()->deactivate();
            if (state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
                return false;
        }
        secondary_controllers_non_rt_.erase(it);
        secondary_controllers_buffer_.writeFromNonRT(
            std::make_shared<const ControllerList>(secondary_controllers_non_rt_));
        return true;
    }
    bool ControlManager::clear_secondary_controllers()
    {
        std::lock_guard<std::mutex> lock(controller_management_mutex_);
        for (auto &controller : secondary_controllers_non_rt_)
        {
            if (controller->get_node_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
                controller->get_node()->deactivate();
        }
        secondary_controllers_non_rt_.clear();
        secondary_controllers_buffer_.writeFromNonRT(
            std::make_shared<const ControllerList>(ControllerList{}));
        return true;
    }
    bool ControlManager::add_secondary_controller(const std::string &controller_name)
    {
        std::lock_guard<std::mutex> lock(controller_management_mutex_);
        auto name = controller_name;
        int pos = name.rfind(":");
        name = name.substr(pos + 1);
        const auto active_ptr = active_controller_buffer_.readFromNonRT();
        if (active_ptr && *active_ptr && (*active_ptr)->get_node()->get_name() == name)
            return false;

        for (auto &controller : controllers_)
        {

            if (controller->get_node()->get_name() == name)
            {
                if (std::find(
                        secondary_controllers_non_rt_.begin(),
                        secondary_controllers_non_rt_.end(), controller) !=
                    secondary_controllers_non_rt_.end())
                    return true;
                const auto state = controller->get_node()->activate();
                if (state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
                    return false;
                secondary_controllers_non_rt_.push_back(controller);
                secondary_controllers_buffer_.writeFromNonRT(
                    std::make_shared<const ControllerList>(secondary_controllers_non_rt_));
                return true;
            }
        }

        return false;
    }
    bool ControlManager::load_controller(const std::string &controller_name)
    {
        std::lock_guard<std::mutex> lock(controller_management_mutex_);
        auto name = controller_name;
        int pos = name.rfind(":");
        name = name.substr(pos + 1);
        for (auto &controller : controllers_)
        {
            if (controller->get_node()->get_name() == name)
            {
                RCLCPP_INFO(get_logger(), "controller %s is already loaded!", name.c_str());
                return true;
            }
        }
        try
        {

            auto controller = controller_loader_->createSharedInstance(controller_name);
            controller->loan_interface(update_rate_,
                                       &robot_->get_robot_math(),
                                       &robot_->get_command_interface(),
                                       &robot_->get_state_interface(),
                                       &robot_->get_com_command_interface(),
                                       &robot_->get_com_state_interface());
            controller->initialize(name);
            controllers_.push_back(controller);
            executor_->add_node(controller->get_node()->get_node_base_interface());
        }
        catch (pluginlib::PluginlibException &ex)
        {
            RCLCPP_INFO(this->get_logger(), "%s", ex.what());
            return false;
        }
        return true;
    }
    void ControlManager::interrupt()
    {
        // HUMBLE-FIX 05: Atomics avoid the blocking RealtimeBox mutex in the loop.
        keep_running_.store(false, std::memory_order_release);
        running_.store(false, std::memory_order_release);
        if (robot_)
            robot_->request_stop();
    }
    bool ControlManager::is_keep_running() const
    {
        return keep_running_.load(std::memory_order_acquire);
    }

    bool ControlManager::deactivate_controller()
    {
        running_.store(false, std::memory_order_release);

        // The control thread performs the normal stop/deactivate sequence. Bound the
        // wait so a failed SDK call cannot hang the service callback forever.
        const auto deadline = std::chrono::steady_clock::now() + 2s;
        while (control_loop_active_.load(std::memory_order_acquire) &&
               std::chrono::steady_clock::now() < deadline)
            std::this_thread::sleep_for(2ms);

        if (control_loop_active_.load(std::memory_order_acquire))
        {
            RCLCPP_ERROR(get_logger(), "timed out while stopping the active control loop");
            return false;
        }

        std::lock_guard<std::mutex> lock(controller_management_mutex_);
        auto active_ptr = active_controller_buffer_.readFromNonRT();
        ControllerPtr controller = active_ptr ? *active_ptr : ControllerPtr{};
        if (controller &&
            controller->get_node_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
        {
            const auto state = controller->get_node()->deactivate();
            if (state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
                return false;
        }
        active_controller_buffer_.writeFromNonRT(ControllerPtr{});
        active_controller_.reset();
        return true;
    }

    bool ControlManager::activate_controller(const std::string &controller_name)
    {
        if (!deactivate_controller())
            return false;

        std::lock_guard<std::mutex> lock(controller_management_mutex_);
        auto name = controller_name;
        const auto pos = name.rfind(":");
        if (pos != std::string::npos)
            name = name.substr(pos + 1);
        for (auto &controller : controllers_)
        {
            if (controller->get_node()->get_name() != name)
                continue;
            if (controller->get_node_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
            {
                RCLCPP_ERROR(get_logger(), "controller %s is not inactive", name.c_str());
                return false;
            }
            const auto state = controller->get_node()->activate();
            if (state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
                return false;
            active_controller_buffer_.writeFromNonRT(controller);
            RCLCPP_INFO(get_logger(), "controller %s is activated", name.c_str());
            return true;
        }
        RCLCPP_ERROR(get_logger(), "controller %s is not loaded", name.c_str());
        return false;
    }

    void ControlManager::command_callback(const std::shared_ptr<robot_control_msgs::srv::ControlCommand::Request> request,
                                          std::shared_ptr<robot_control_msgs::srv::ControlCommand::Response> response)
    {
        std::string cmd = request->cmd_name;
        response->result = false;
        if (cmd == "activate")
            response->result = activate_controller(request->cmd_params);
        else if (cmd == "load")
            response->result = load_controller(request->cmd_params);
        else if (cmd == "add")
            response->result = add_secondary_controller(request->cmd_params);
        else if (cmd == "remove")
            response->result = remove_secondary_controller(request->cmd_params);
        else if (cmd == "clear")
            response->result = clear_secondary_controllers();
        else if (cmd == "deactivate" || cmd == "stop")
        {
            response->result = deactivate_controller();
        }
        else
            RCLCPP_WARN(get_logger(), "unknown control command: %s", cmd.c_str());
    }

    int ControlManager::get_update_rate()
    {
        return update_rate_;
    }

    void ControlManager::shutdown_robot()
    {
        RCLCPP_INFO(this->get_logger(), "shutting down controllers and robot and all attached hardwares");
        ControllerList controller_snapshot;
        {
            std::lock_guard<std::mutex> lock(controller_management_mutex_);
            controller_snapshot = controllers_;
        }
        for (auto &controller : controller_snapshot)
        {
            controller->finalize();
        }
        robot_->finalize();
    }

    void control_node::ControlManager::read(const rclcpp::Time &t, const rclcpp::Duration &period)
    {
        robot_->read(t, period);
        if (!is_publish_joint_state_ || !real_time_publisher_)
            return;

        ++joint_state_publish_count_;
        if (joint_state_publish_count_ < joint_state_publish_divider_)
            return;
        joint_state_publish_count_ = 0;

        // HUMBLE-FIX 06: No make_shared/vector assignment in the RT path. The
        // message vectors were sized in the constructor and are filled in place.
        if (real_time_publisher_->trylock())
        {
            auto &msg = real_time_publisher_->msg_;
            const auto &state = robot_->get_state_interface().get<double>();
            const auto copy_state = [&state](const char *name, auto &destination)
            {
                const auto it = state.find(name);
                if (it == state.end())
                    return;
                const auto count = std::min(destination.size(), it->second.size());
                std::copy_n(it->second.begin(), count, destination.begin());
            };
            copy_state("position", msg.position);
            copy_state("velocity", msg.velocity);
            copy_state("torque", msg.effort);
            msg.header.stamp = this->now();
            real_time_publisher_->unlockAndPublish();
        }
    }

    void ControlManager::update(const rclcpp::Time &t, const rclcpp::Duration &period)
    {

        if (!active_controller_)
            return;
        active_controller_->update(t, period);

        const auto snapshot_ptr = secondary_controllers_buffer_.readFromRT();
        if (!snapshot_ptr || !*snapshot_ptr)
            return;
        for (const auto &controller : **snapshot_ptr)
        {
            if (controller->get_node_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
                controller->update(t, period);
        }
    }

    void ControlManager::write(const rclcpp::Time &t, const rclcpp::Duration &period)
    {
        robot_->write(t, period);
    }

    Eigen::MatrixXd ControlManager::simulation_external_force(double /*t*/)
    {
        return Eigen::MatrixXd::Zero(6, robot_->get_dof());
    }
    void ControlManager::simulation_observer(const std::vector<double> &x, double t)
    {
        // std::cerr << t << " : ";
        // for (int i = 0; i < dof_; i++)
        //     std::cerr << x[i] << " ";
        // std::cerr << "\n";
        // std::copy(x.begin(), x.begin() + dof_, joint_position_.begin());
        // std::copy(x.begin() + dof_, x.begin() + 2 * dof_, joint_velocity_.begin());
        int n = robot_->get_dof();
        if (t == 0)
        {
            Eigen::MatrixXd f_ext = simulation_external_force(t);
            simulation_controller(t, x, f_ext);
        }
        auto states = std::make_shared<sensor_msgs::msg::JointState>();
        states->name = robot_->get_joint_names();
        std::copy(x.begin(), x.begin() + n, std::back_inserter(states->position));
        std::copy(x.begin() + n, x.begin() + 2 * n, std::back_inserter(states->velocity));
        states->effort = robot_->get_command_interface().get<double>("torque");
        auto time = sim_start_time_ + rclcpp::Duration::from_seconds(t); //(std::chrono::duration<double>(t));
        // auto time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(t));
        states->header.stamp = time; // rclcpp::Time(nano_time.count());//this->now(); // ;
        // joint_state_publisher_->publish(*states);
        if (real_time_publisher_->trylock())
        {
            real_time_publisher_->msg_ = *states;
            real_time_publisher_->unlockAndPublish();
        }
        // wait for real time elapse
        if (is_sim_real_time_)
        {
            auto const nano_time = std::chrono::nanoseconds(time.nanoseconds());
            std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> until_time{nano_time};
            std::this_thread::sleep_until(until_time);
        }
    }

    bool ControlManager::is_simulation()
    {
        return is_simulation_;
    }

    void ControlManager::start_simulation(double time)
    {
        if (!running_)
            return;

        typedef std::vector<double> state_type;

        auto f_external = std::bind(&ControlManager::simulation_external_force, this,
                                    std::placeholders::_1);

        auto controller = std::bind(&ControlManager::simulation_controller, this,
                                    std::placeholders::_1,
                                    std::placeholders::_2,
                                    std::placeholders::_3);

        auto dynamics = std::bind(&hardware_interface::RobotInterface::robot_dynamics, robot_.get(),
                                  std::placeholders::_1,
                                  std::placeholders::_2,
                                  std::placeholders::_3,
                                  std::cref(f_external), std::cref(controller));

        auto observer = std::bind(&ControlManager::simulation_observer, this,
                                  std::placeholders::_1,
                                  std::placeholders::_2);

        // Error stepper, used to create the controlled stepper
        typedef runge_kutta_cash_karp54<state_type> error_stepper_type;
        // typedef controlled_runge_kutta<error_stepper_type> controlled_stepper_type;
        state_type x0(2 * robot_->get_dof(), 0);
        sim_start_time_ = this->now();
        integrate_adaptive(make_controlled(1.0e-10, 1.0e-6, error_stepper_type()), dynamics, x0, 0.0, time, 0.001, observer);
        // size_t steps = integrate_adaptive(runge_kutta4<std::vector<double>>(), dynamics, x0, 0.0, time, 0.01, observer);
    }

    std::vector<double> ControlManager::simulation_controller(double t, const std::vector<double> &x, const Eigen::MatrixXd &fext)
    {
        int n = robot_->get_dof();
        std::vector<double> f{fext(0, n - 1), fext(1, n - 1), fext(2, n - 1),
                              fext(3, n - 1), fext(4, n - 1), fext(5, n - 1)};
        robot_->write_state(x, f);
        auto std_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(t));
        auto time = rclcpp::Time(std_time.count());
        auto period = rclcpp::Duration(std::chrono::duration<double>(0.0));
        active_controller_->write_state(x.begin() + 2 * n, x.end());
        active_controller_->update(time, period);
        auto cmd = robot_->get_command_interface().get<double>("torque");
        cmd.insert(cmd.end(), active_controller_->get_internal_state().begin(), active_controller_->get_internal_state().end());
        return cmd;
    }
    void ControlManager::control_loop()
    {
        // HUMBLE-FIX 07: Split hardware-paced and software-paced loops. Franka's
        // readOnce() supplies the 1 kHz clock; UR/Diana/simulation use steady_clock.
        using SteadyClock = std::chrono::steady_clock;
        const auto nominal_period = std::chrono::nanoseconds(1'000'000'000LL / update_rate_);
        const auto nominal_ros_period = rclcpp::Duration::from_nanoseconds(nominal_period.count());
        const bool hardware_paced = robot_->is_hardware_paced();
        auto next_iteration_time = SteadyClock::now();
        auto previous_read_time = next_iteration_time;
        bool have_previous_read = false;
        control_loop_active_.store(true, std::memory_order_release);

        while (running_.load(std::memory_order_acquire) && !robot_->is_stop())
        {
            if (!hardware_paced)
                std::this_thread::sleep_until(next_iteration_time);

            const auto read_stamp = this->now();
            read(read_stamp, nominal_ros_period);
            const auto after_read = SteadyClock::now();

            rclcpp::Duration measured_period = nominal_ros_period;
            if (hardware_paced)
            {
                const auto hardware_period = robot_->get_last_read_period();
                if (hardware_period.nanoseconds() > 0)
                    measured_period = hardware_period;
            }
            else if (have_previous_read)
            {
                const auto host_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    after_read - previous_read_time);
                if (host_period.count() > 0)
                    measured_period = rclcpp::Duration::from_nanoseconds(host_period.count());
            }
            previous_read_time = after_read;
            have_previous_read = true;

            // ROS time is a message/controller timestamp only; the period above is
            // derived from FCI or steady_clock and cannot jump with /clock.
            const auto current_time = this->now();
            update(current_time, measured_period);
            write(current_time, measured_period);
            if (active_controller_ && active_controller_->requests_stop())
                running_.store(false, std::memory_order_release);

            if (!hardware_paced)
            {
                next_iteration_time += nominal_period;
                const auto now = SteadyClock::now();
                if (now > next_iteration_time + nominal_period)
                    next_iteration_time = now;
            }
        }
        running_.store(false, std::memory_order_release);
    }

    void ControlManager::prepare_loop()
    {
        auto state = robot_->get_node_state();
        while (keep_running_.load(std::memory_order_acquire) &&
               state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
        {
            RCLCPP_WARN(this->get_logger(), "robot is not configured!");
            std::this_thread::sleep_for(1s);
            state = robot_->get_node_state();
        }
        if (!keep_running_.load(std::memory_order_acquire))
        {
            running_.store(false, std::memory_order_release);
            return;
        }
        const auto robot_state = robot_->get_node()->activate();
        if (robot_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
            throw std::runtime_error("failed to activate robot hardware");

        RCLCPP_INFO(get_logger(), "waiting for controller to be activated...");
        std::stringstream ss;
        ControllerList controller_snapshot;
        {
            std::lock_guard<std::mutex> lock(controller_management_mutex_);
            controller_snapshot = controllers_;
        }
        for (const auto &controller : controller_snapshot)
        {
            ss << controller->get_node()->get_name() << " ";
        }
        RCLCPP_INFO(get_logger(), "available controllers are: %s", ss.str().c_str());
        std::stringstream ss2;
        const auto secondary_ptr = secondary_controllers_buffer_.readFromNonRT();
        if (secondary_ptr && *secondary_ptr)
        {
            for (const auto &controller : **secondary_ptr)
                ss2 << controller->get_node()->get_name() << " ";
        }
        RCLCPP_INFO(get_logger(), "secondary controllers are: %s", ss2.str().c_str());

        if (!default_controller_.empty())
        {
            const auto requested_default = default_controller_;
            default_controller_.clear();
            if (!activate_controller(requested_default))
                throw std::runtime_error("failed to activate default controller " + requested_default);
        }

        do
        {
            const auto active_ptr = active_controller_buffer_.readFromRT();
            active_controller_ = active_ptr ? *active_ptr : ControllerPtr{};
            if (!active_controller_)
                std::this_thread::sleep_for(20ms);
        } while (keep_running_.load(std::memory_order_acquire) && !active_controller_);
        if (!keep_running_.load(std::memory_order_acquire))
        {
            running_.store(false, std::memory_order_release);
            return;
        }

        // HUMBLE-FIX 33: Existing controllers select their hardware mode in the
        // first update(period=0). FC3 activation has already obtained a fresh
        // Robot::readOnce() sample, so initialize the command and mode now. No
        // command is sent to FCI until control_loop() performs read-update-write.
        active_controller_->update(this->now(), rclcpp::Duration(0, 0));

        // Start the SDK control stream only now, immediately before the continuous
        // read-update-write sequence.
        if (!robot_->begin_control())
            throw std::runtime_error("failed to start the robot control stream");
        running_.store(true, std::memory_order_release);
    }

    void ControlManager::end_loop()
    {
        running_.store(false, std::memory_order_release);
        robot_->end_control();

        {
            std::lock_guard<std::mutex> lock(controller_management_mutex_);
            auto active_ptr = active_controller_buffer_.readFromNonRT();
            ControllerPtr value = active_ptr ? *active_ptr : ControllerPtr{};
            if (value &&
                value->get_node_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
                value->get_node()->deactivate();
            active_controller_buffer_.writeFromNonRT(ControllerPtr{});
            active_controller_.reset();
        }

        const auto state = robot_->get_node_state();
        if (state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
        {
            const auto inactive_state = robot_->get_node()->deactivate();
            if (inactive_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
                RCLCPP_ERROR(get_logger(), "failed to deactivate robot hardware");
        }
        control_loop_active_.store(false, std::memory_order_release);
    }

}
