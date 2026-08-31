#include "control_node/control_manager.h"
#include "lifecycle_msgs/msg/state.hpp"
#include <boost/numeric/odeint.hpp>
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <algorithm>
#include <sstream>
#include <stdexcept>
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
          running_request_(false),
          keep_running_(true)
    {
        secondary_controllers_buffer_.writeFromNonRT(
            std::make_shared<const ControllerList>(secondary_controllers_));
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
            this->get_parameter_or<int>("joint_state_publish_rate", 100);
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
            robot_->initialize(robot_name);
            if (real_time_publisher_)
            {
                auto &msg = real_time_publisher_->msg_;
                msg.name = robot_->get_joint_names();
                const auto dof = static_cast<std::size_t>(robot_->get_dof());
                msg.position.resize(dof);
                msg.velocity.resize(dof);
                msg.effort.resize(dof);
            }
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
        }
        catch (pluginlib::PluginlibException &ex)
        {
            RCLCPP_INFO(this->get_logger(), "%s", ex.what());
            throw ex;
        }
        service_ = create_service<robot_control_msgs::srv::ControlCommand>("~/control_command",
                                                                           std::bind(&ControlManager::command_callback, this, std::placeholders::_1, std::placeholders::_2));

        executor_->add_node(this->get_node_base_interface());
    }

    ControlManager::~ControlManager()
    {
    }
    bool ControlManager::remove_secondary_controller(const std::string &controller_name)
    {
        auto name = controller_name;
        int pos = name.rfind(":");
        name = name.substr(pos + 1);
        // secondary_controllers_box_.set([=, &name](auto &value)
        //                                {
        //                                    auto it = std::find_if(value.begin(), value.end(), [=](auto &&v)
        //                                                           { return v->get_node()->get_name() == name; });
        //                                    if (it != value.end())
        //                                    {
        //                                        (*it)->get_node()->deactivate();
        //                                        value.erase(it);
        //                                    } });
        auto it = std::find_if(secondary_controllers_.begin(), secondary_controllers_.end(), [=](auto &&v)
                               { return v->get_node()->get_name() == name; });
        if (it != secondary_controllers_.end())
        {
            (*it)->get_node()->deactivate();
            secondary_controllers_.erase(it);
        }
        secondary_controllers_buffer_.writeFromNonRT(
            std::make_shared<const ControllerList>(secondary_controllers_));

        return true;
    }
    bool ControlManager::clear_secondary_controllers()
    {
        // secondary_controllers_box_.set([this](auto &value)
        //                                {
        //                                    std::for_each(value.begin(), value.end(), [=](auto &&v)
        //                                                  { v->get_node()->deactivate(); });
        //                                    value.clear(); });
        std::for_each(secondary_controllers_.begin(), secondary_controllers_.end(), [=](auto &&v)
                      { v->get_node()->deactivate(); });
        secondary_controllers_.clear();
        secondary_controllers_buffer_.writeFromNonRT(
            std::make_shared<const ControllerList>(secondary_controllers_));
        return true;
    }
    bool ControlManager::add_secondary_controller(const std::string &controller_name)
    {
        auto name = controller_name;
        int pos = name.rfind(":");
        name = name.substr(pos + 1);
        bool ret;
        // active_controller_box_.get([=, &ret, &name](const auto &value)
        //                            {
        //                                if (value != nullptr && value->get_node()->get_name() == name)
        //                                    ret = false;
        //                                else
        //                                    ret = true; });

        std::shared_ptr<controller_interface::ControllerInterface> value;
        active_controller_box_.get(value);
        if (value != nullptr && value->get_node()->get_name() == name)
            ret = false;
        else
            ret = true;

        if (!ret)
            return false;

        for (auto &controller : controllers_)
        {

            if (controller->get_node()->get_name() == name)
            {
                // secondary_controllers_box_.set([=](auto &value)
                //                                {
                //                             if(std::find(value.begin(), value.end(), controller) == value.end())
                //                                 value.push_back(controller);
                //                             controller->get_node()->activate(); });
                if (std::find(secondary_controllers_.begin(), secondary_controllers_.end(), controller) ==
                    secondary_controllers_.end())
                {
                    secondary_controllers_.push_back(controller);
                }
                controller->get_node()->activate();
                secondary_controllers_buffer_.writeFromNonRT(
                    std::make_shared<const ControllerList>(secondary_controllers_));
                return true;
            }
        }

        return false;
    }
    bool ControlManager::load_controller(const std::string &controller_name)
    {
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
        keep_running_ = false;
        running_request_.store(false, std::memory_order_release);
    }
    bool ControlManager::is_keep_running()
    {
        return keep_running_;
    }
    bool ControlManager::activate_controller(const std::string &controller_name)
    {
        bool running = false;
        // active_controller_box_.get([=, &running](const auto &value)
        //                            {
        //     if (value)
        //         running = true; });
        std::shared_ptr<controller_interface::ControllerInterface> value;
        active_controller_box_.get(value);
        if (value)
            running = true;

        if (running)
        {

            do
            {
                running_request_.store(false, std::memory_order_release);
                std::this_thread::sleep_for(5ms);
                //     active_controller_box_.get([=, &running](const auto &value)
                //                                {
                // if (!value)
                //     running = false; });
                std::shared_ptr<controller_interface::ControllerInterface> value;
                active_controller_box_.get(value);
                if (!value)
                    running = false;

            } while (running);
        }

        for (auto &controller : controllers_)
        {
            if (controller->get_node_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE && controller->get_node()->get_name() == controller_name)
            {
                bool ret = true;
                // active_controller_box_.set([=, &ret, &controller](auto &value)
                //                            {
                //     if (value)
                //     {
                //         auto state = value->get_node()->deactivate();
                //         if (state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
                //         {
                //             ret = false;
                //             return;
                //         }
                //     }
                //     value = controller;
                //     auto state = value->get_node()->activate();
                //     if (state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
                //     {
                //         value = nullptr;
                //         ret = false;
                //     }
                //     else
                //         RCLCPP_INFO(get_logger(), "controller %s is activated!", controller->get_node()->get_name()); });
                // bool ret = true;
                std::shared_ptr<controller_interface::ControllerInterface> value;
                active_controller_box_.get(value);
                if (value)
                {
                    auto state = value->get_node()->deactivate();
                    if (state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
                    {
                        return false;
                    }
                }
                value = controller;
                auto state = value->get_node()->activate();
                if (state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
                {
                    value = nullptr;
                    ret = false;
                }
                else
                {
                    RCLCPP_INFO(get_logger(), "controller %s is activated!", controller->get_node()->get_name());
                }
                active_controller_box_.set(value);
                return ret;
            }
        }
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
        else if (cmd == "stop" || cmd == "deactivate")
        {
            running_request_.store(false, std::memory_order_release);
            response->result = true;
        }
    }

    int ControlManager::get_update_rate()
    {
        return update_rate_;
    }

    void ControlManager::shutdown_robot()
    {
        RCLCPP_INFO(this->get_logger(), "shutting down controllers and robot and all attached hardwares");
        for (auto &controller : controllers_)
        {
            controller->finalize();
        }
        robot_->finalize();
    }

    void control_node::ControlManager::read(const rclcpp::Time &t, const rclcpp::Duration &period)
    {
        robot_->read(t, period);
        if (is_publish_joint_state_ &&
            ++joint_state_publish_counter_ >= joint_state_publish_divider_)
        {
            joint_state_publish_counter_ = 0;
            // [HUMBLE-FIX 06] No make_shared/vector assignment at 1 kHz.  Data
            // are copied only after trylock succeeds and publication is decimated.
            if (real_time_publisher_->trylock())
            {
                auto &msg = real_time_publisher_->msg_;
                const auto &state = robot_->get_state_interface().get<double>();
                const auto position = state.find("position");
                const auto velocity = state.find("velocity");
                const auto torque = state.find("torque");
                if (position != state.end())
                    std::copy_n(position->second.begin(),
                                std::min(position->second.size(), msg.position.size()),
                                msg.position.begin());
                if (velocity != state.end())
                    std::copy_n(velocity->second.begin(),
                                std::min(velocity->second.size(), msg.velocity.size()),
                                msg.velocity.begin());
                if (torque != state.end())
                    std::copy_n(torque->second.begin(),
                                std::min(torque->second.size(), msg.effort.size()),
                                msg.effort.begin());
                msg.header.stamp = t;
                real_time_publisher_->unlockAndPublish();
            }
        }
    }

    void ControlManager::update(const rclcpp::Time &t, const rclcpp::Duration &period)
    {

        active_controller_->update(t, period);
        // secondary_controllers_box_.try_get([&t, &period](const auto &value)
        //                                    {
        //     for (auto &&controller : value)
        //     {
        //         if (controller->get_node_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
        //         {
        //             controller->update(t, period);
        //         }
        //     } });
        const auto *secondary_snapshot = secondary_controllers_buffer_.readFromRT();
        if (secondary_snapshot && *secondary_snapshot)
        {
            for (const auto &controller : **secondary_snapshot)
            {
                if (controller->get_node_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
                    controller->update(t, period);
            }
        }
        // for (auto &controller : controllers_)
        // {
        //     if (controller->get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
        //     {
        //         controller->update(t, period);
        //     }
        // }
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
        const auto target_period = std::chrono::nanoseconds(1'000'000'000 / update_rate_);
        const bool hardware_paced = robot_->is_hardware_paced();
        auto next_iteration_time = std::chrono::steady_clock::now();
        auto previous_steady_time = next_iteration_time;
        rclcpp::Duration measured_period(0, 0);
        bool first_controller_update = true;

        while (running_ && !robot_->is_stop()) // give robot a change to stop running
        {
            // [HUMBLE-FIX 07] Franka readOnce() is the only 1 kHz pacer.  Its
            // returned period is authoritative and there is no second sleep.
            const auto read_stamp = this->now();
            read(read_stamp, measured_period);
            if (robot_->is_stop())
            {
                // [HUMBLE-FIX 52] A hardware/FCI exception is not a normal
                // controller stop request.  Do not silently re-arm the robot in
                // the outer lifecycle loop; shut the process down for diagnosis.
                keep_running_.store(false, std::memory_order_release);
                break;
            }

            if (hardware_paced)
            {
                measured_period = robot_->get_last_control_period();
            }
            else
            {
                const auto steady_now = std::chrono::steady_clock::now();
                measured_period = rclcpp::Duration(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        steady_now - previous_steady_time));
                previous_steady_time = steady_now;
            }

            const auto current_time = this->now();
            // [HUMBLE-FIX 45] Existing controllers use a zero first period to snapshot their
            // hold pose/joint position.  Preserve that lifecycle convention,
            // then provide the hardware-reported period from cycle two onward.
            const rclcpp::Duration controller_period = first_controller_update
                                                           ? rclcpp::Duration(0, 0)
                                                           : measured_period;
            first_controller_update = false;
            update(current_time, controller_period);
            write(current_time, measured_period);
            running_ = running_request_.load(std::memory_order_acquire);

            // Non-hardware-paced backends still require an absolute steady-clock
            // deadline.  Absolute scheduling avoids accumulating loop drift.
            if (!hardware_paced)
            {
                next_iteration_time += target_period;
                std::this_thread::sleep_until(next_iteration_time);
            }
        }
    }

    void ControlManager::prepare_loop()
    {
        while (keep_running_ &&
               robot_->get_node_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
        {
            RCLCPP_WARN(this->get_logger(), "robot is not configured!");
            std::this_thread::sleep_for(1s);
        }
        if (!keep_running_)
        {
            running_ = false;
            return;
        }
        const auto activated_state = robot_->get_node()->activate();
        if (activated_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
            throw std::runtime_error("robot activation failed");
        RCLCPP_INFO(get_logger(), "waiting for controller to be activated...");
        std::stringstream ss;
        for (auto &&controller : controllers_)
        {
            ss << controller->get_node()->get_name() << " ";
        }
        RCLCPP_INFO(get_logger(), "available controllers are: %s", ss.str().c_str());
        // std::stringstream ss2;
        // secondary_controllers_box_.get([this, &ss2](const auto &value)
        //                                {
        //     for (auto &&controller : value)
        //     {
        //         ss2 << controller->get_node()->get_name() << " ";
        //     } });
        std::stringstream ss2;
        const auto *secondary_snapshot = secondary_controllers_buffer_.readFromRT();
        if (secondary_snapshot && *secondary_snapshot)
        {
            for (const auto &controller : **secondary_snapshot)
                ss2 << controller->get_node()->get_name() << " ";
        }
        RCLCPP_INFO(get_logger(), "secondary controllers are: %s", ss2.str().c_str());
        do
        {
            if (robot_->is_hardware_paced())
            {
                // [HUMBLE-FIX 08] Keep the FCI read/write handshake alive while
                // waiting for controller activation.  Zero torque is initialized
                // by the hardware interface before this loop starts.
                const auto stamp = this->now();
                read(stamp, robot_->get_last_control_period());
                if (robot_->is_stop())
                {
                    running_ = false;
                    keep_running_.store(false, std::memory_order_release);
                    return;
                }
                write(this->now(), robot_->get_last_control_period());
            }
            else
            {
                std::this_thread::sleep_for(1s);
                read(this->now(), rclcpp::Duration::from_seconds(1.0));
            }
            // active_controller_box_.get([=](const auto &value)
            //                            { active_controller_ = value; });
            std::shared_ptr<controller_interface::ControllerInterface> value;
            active_controller_box_.get(value);
            active_controller_ = value;
            if (!default_controller_.empty())
            {
                activate_controller(default_controller_);
                default_controller_.clear();
            }
        } while (keep_running_ && !active_controller_);
        if (!keep_running_)
        {
            running_ = false;
            return;
        }
        running_request_.store(true, std::memory_order_release);
        running_ = true;
    }

    void ControlManager::end_loop()
    {
        // active_controller_box_.set([=](auto &value)
        //                            {
        //         if (value)
        //         {
        //             auto state = value->get_node_state();
        //             if(state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
        //                 value->get_node()->deactivate();

        //             value = nullptr;
        //         } });
        std::shared_ptr<controller_interface::ControllerInterface> value;
        active_controller_box_.get(value);
        if (value)
        {
            auto state = value->get_node_state();
            if (state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
                value->get_node()->deactivate();

            value = nullptr;
        }
        active_controller_box_.set(value);
        auto state = robot_->get_node_state();
        if (state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
            robot_->get_node()->deactivate();
    }

}