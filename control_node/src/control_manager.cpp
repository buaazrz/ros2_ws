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
          running_box_(false),
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
        update_rate_ = this->get_parameter_or<int>("update_rate", 500);
        is_simulation_ = this->get_parameter_or<bool>("simulation", true);
        simulation_backend_ =
            this->get_parameter_or<std::string>(
                "simulation_backend", "");
        use_mujoco_hold_ =
            is_simulation_ &&
            simulation_backend_ == "mujoco";
        RCLCPP_INFO(
            get_logger(),
            "Simulation backend: '%s', MuJoCo hold: %s",
            simulation_backend_.c_str(),
            use_mujoco_hold_ ? "enabled" : "disabled");
        is_sim_real_time_ = this->get_parameter_or<bool>("sim_real_time", true);
        is_publish_joint_state_ = this->get_parameter_or<bool>("publish_joint_state", true);
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
            // robot_->initialize(robot_name);
            if (!robot_->initialize(robot_name))
            {
                throw std::runtime_error(
                    "Failed to initialize robot plugin: " + robot_class);
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
        std::vector<std::shared_ptr<controller_interface::ControllerInterface>> value;
        secondary_controllers_box_.get(value);
        auto it = std::find_if(value.begin(), value.end(), [=](auto &&v)
                               { return v->get_node()->get_name() == name; });
        if (it != value.end())
        {
            (*it)->get_node()->deactivate();
            value.erase(it);
        }
        secondary_controllers_box_.set(value);

        return true;
    }
    bool ControlManager::clear_secondary_controllers()
    {
        // secondary_controllers_box_.set([this](auto &value)
        //                                {
        //                                    std::for_each(value.begin(), value.end(), [=](auto &&v)
        //                                                  { v->get_node()->deactivate(); });
        //                                    value.clear(); });
        std::vector<std::shared_ptr<controller_interface::ControllerInterface>> value;
        secondary_controllers_box_.get(value);
        std::for_each(value.begin(), value.end(), [=](auto &&v)
                      { v->get_node()->deactivate(); });
        value.clear();
        secondary_controllers_box_.set(value);
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
                std::vector<std::shared_ptr<controller_interface::ControllerInterface>> sec_value;
                secondary_controllers_box_.get(sec_value);
                if (std::find(sec_value.begin(), sec_value.end(), controller) == sec_value.end())
                {
                    sec_value.push_back(controller);
                }
                controller->get_node()->activate();
                secondary_controllers_box_.set(sec_value);
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
        // running_box_ = false;
        running_box_.set(false);
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
                running_box_.set(false);
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
        else if (cmd == "stop")
        {
            // running_box_ = false;
            running_box_.set(false);
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
        if (is_publish_joint_state_)
        {
            auto joint_state = std::make_shared<sensor_msgs::msg::JointState>();
            joint_state->name = robot_->get_joint_names();
            auto &state = robot_->get_state_interface().get<double>();
            auto it = state.find("position");
            if (it != state.end())
            {
                joint_state->position = it->second;
            }
            it = state.find("velocity");
            if (it != state.end())
            {
                joint_state->velocity = it->second;
            }
            it = state.find("torque");
            if (it != state.end())
            {
                joint_state->effort = it->second;
            }
            joint_state->header.stamp = t;
            // this is faster but may block the rt thread
            // joint_state_publisher_->publish(*joint_state);
            if (real_time_publisher_->trylock())
            {
                real_time_publisher_->msg_ = *joint_state;
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
        std::vector<std::shared_ptr<controller_interface::ControllerInterface>> sec_controllers;
        secondary_controllers_box_.get(sec_controllers);
        for (auto &&controller : sec_controllers)
        {
            if (controller->get_node_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
            {
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
    // void ControlManager::control_loop()
    // {
    //     // for calculating sleep time
    //     // double dt = 1.0 / update_rate_;
    //     auto const period = std::chrono::nanoseconds(1'000'000'000 / update_rate_);
    //     auto const cm_now = std::chrono::nanoseconds(this->now().nanoseconds());
    //     std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>
    //         next_iteration_time{cm_now};

    //     rclcpp::Time previous_time = this->now();
    //     rclcpp::Duration measured_period(0, 0);
    //     bool flag = false;
    //     while (running_ && !robot_->is_stop()) // give robot a change to stop running
    //     {
    //         // calculate measured period
    //         auto current_time = this->now();
    //         if (flag)
    //             measured_period = current_time - previous_time;
    //         else
    //             flag = true;
    //         previous_time = current_time;

    //         // execute update loop
    //         read(current_time, measured_period);
    //         update(current_time, measured_period);
    //         write(current_time, measured_period);
    //         // get running state from box
    //         // running_box_.try_get([this](const auto &value)
    //         //                      { running_ = value; });
    //         running_box_.get(running_);

    //         // wait until we hit the end of the period
    //         next_iteration_time += period;
    //         std::this_thread::sleep_until(next_iteration_time);
    //     }
    // }
    void ControlManager::control_loop()
    {
        using SteadyClock = std::chrono::steady_clock;

        const auto nominal_period =
            std::chrono::nanoseconds(1'000'000'000LL / update_rate_);

        auto next_iteration_time = SteadyClock::now();
        auto previous_steady_time =
            next_iteration_time - nominal_period;

        while (running_ && keep_running_ && !robot_->is_stop())
        {
            const auto current_steady_time = SteadyClock::now();
            const auto elapsed =
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    current_steady_time - previous_steady_time);

            previous_steady_time = current_steady_time;

            // 传给控制器的时间戳仍然使用 ROS time。
            const rclcpp::Time ros_time = this->now();

            // period 使用稳态时钟测量，不受 /clock 跳变影响。
            const rclcpp::Duration measured_period(elapsed);

            read(ros_time, measured_period);
            update(ros_time, measured_period);
            write(ros_time, measured_period);

            running_box_.try_get(
                [this](const auto & value)
                {
                    running_ = value;
                });

            next_iteration_time += nominal_period;

            const auto work_finished_time = SteadyClock::now();

            // 超过一个完整周期时重新同步，避免持续追赶造成空转。
            if (work_finished_time >
                next_iteration_time + nominal_period)
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "Control loop overrun");

                next_iteration_time = work_finished_time;
            }

            std::this_thread::sleep_until(next_iteration_time);
        }
    }

    // void ControlManager::prepare_loop()
    // {
    //     auto state = robot_->get_node_state();
    //     while (keep_running_ && state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    //     {
    //         RCLCPP_WARN(this->get_logger(), "robot is not configured!");
    //         std::this_thread::sleep_for(1s);
    //     }
    //     if (!keep_running_)
    //     {
    //         running_ = false;
    //         return;
    //     }
    //     robot_->get_node()->activate();
    //     RCLCPP_INFO(get_logger(), "waiting for controller to be activated...");
    //     std::stringstream ss;
    //     for (auto &&controller : controllers_)
    //     {
    //         ss << controller->get_node()->get_name() << " ";
    //     }
    //     RCLCPP_INFO(get_logger(), "available controllers are: %s", ss.str().c_str());
    //     // std::stringstream ss2;
    //     // secondary_controllers_box_.get([this, &ss2](const auto &value)
    //     //                                {
    //     //     for (auto &&controller : value)
    //     //     {
    //     //         ss2 << controller->get_node()->get_name() << " ";
    //     //     } });
    //     std::stringstream ss2;
    //     std::vector<std::shared_ptr<controller_interface::ControllerInterface>> sec_controllers;
    //     secondary_controllers_box_.get(sec_controllers);
    //     for (auto &&controller : sec_controllers)
    //     {
    //         ss2 << controller->get_node()->get_name() << " ";
    //     }
    //     RCLCPP_INFO(get_logger(), "secondary controllers are: %s", ss2.str().c_str());
    //     do
    //     {
    //         std::this_thread::sleep_for(1s);
    //         read(this->now(), rclcpp::Duration::from_seconds(1.0));
    //         // active_controller_box_.get([=](const auto &value)
    //         //                            { active_controller_ = value; });
    //         std::shared_ptr<controller_interface::ControllerInterface> value;
    //         active_controller_box_.get(value);
    //         active_controller_ = value;
    //         if (!default_controller_.empty())
    //         {
    //             activate_controller(default_controller_);
    //             default_controller_.clear();
    //         }
    //     } while (keep_running_ && !active_controller_);
    //     if (!keep_running_)
    //     {
    //         running_ = false;
    //         return;
    //     }
    //     // running_box_ = true;
    //     running_box_.set(true);
    //     running_ = true;
    // }
    void ControlManager::prepare_loop()
    {
        using SteadyClock = std::chrono::steady_clock;

        const auto nominal_period =
            std::chrono::nanoseconds(
                1'000'000'000LL / update_rate_);

        running_ = false;
        running_box_.set(false);
        active_controller_.reset();

        /*
         * 真机沿用原有生命周期：只有 INACTIVE 才能进入后续激活流程。
         * MuJoCo 在控制器切换期间保持 Robot ACTIVE，避免物理线程失去力矩。
         */
        while (keep_running_)
        {
            const auto robot_state = robot_->get_node_state();

            const bool robot_is_inactive =
                robot_state.id() ==
                lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;

            const bool active_mujoco_robot =
                use_mujoco_hold_ &&
                robot_state.id() ==
                lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;

            if (robot_is_inactive || active_mujoco_robot)
            {
                break;
            }

            if (robot_state.id() ==
                lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED)
            {
                throw std::runtime_error(
                    "Robot has been finalized and cannot be activated");
            }

            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Waiting for robot lifecycle state. Current state: %s",
                robot_state.label().c_str());

            std::this_thread::sleep_for(100ms);
        }

        if (!keep_running_)
        {
            return;
        }

        const auto robot_state = robot_->get_node_state();

        if (robot_state.id() ==
            lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
        {
            const auto activated_state =
                robot_->get_node()->activate();

            if (activated_state.id() !=
                lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
            {
                throw std::runtime_error(
                    "Failed to activate robot. Lifecycle state is: " +
                    activated_state.label());
            }

            RCLCPP_INFO(
                get_logger(),
                "Robot '%s' activated",
                robot_->get_node()->get_name());
        }
        else if (use_mujoco_hold_ &&
                 robot_state.id() ==
                     lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
        {
            RCLCPP_INFO(
                get_logger(),
                "MuJoCo robot '%s' remains active during controller switch",
                robot_->get_node()->get_name());
        }
        else
        {
            throw std::runtime_error(
                "Real robot must be INACTIVE before activation");
        }

        /*
         * 仅供 MuJoCo 使用的硬件保持周期。
         * mode=0 必须在 MujocoRobot::write() 中实现为重力补偿 + 位置保持，
         * 不能解释为零力矩。
         */
        auto run_mujoco_hold_cycle =
            [this](const rclcpp::Duration &period)
            {
                if (!use_mujoco_hold_)
                {
                    return;
                }

                if (robot_->get_node_state().id() !=
                    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
                {
                    return;
                }

                std::shared_ptr<
                    controller_interface::ControllerInterface>
                    controller;

                active_controller_box_.get(controller);

                // 已经存在活动控制器时，不覆盖控制器命令。
                if (controller)
                {
                    return;
                }

                const rclcpp::Time stamp = this->now();

                // 先读取最新状态，MujocoRobot 再以该状态更新保持力矩。
                read(stamp, period);

                auto &mode =
                    robot_->get_command_interface().get<int>("mode");

                if (mode.empty())
                {
                    RCLCPP_ERROR(
                        get_logger(),
                        "Cannot enter MuJoCo hold mode: "
                        "command interface 'mode' is empty");
                    return;
                }

                mode[0] = 0;
                write(stamp, period);
            };

        /*
        * 3. 打印可用控制器。
        */
        std::stringstream available_controller_names;

        for (const auto & controller : controllers_)
        {
            if (controller)
            {
                available_controller_names
                    << controller->get_node()->get_name() << " ";
            }
        }

        RCLCPP_INFO(
            get_logger(),
            "Available controllers: %s",
            available_controller_names.str().c_str());

        /*
        * 4. 打印辅助控制器。
        */
        std::vector<
            std::shared_ptr<controller_interface::ControllerInterface>>
            secondary_controllers;

        secondary_controllers_box_.get(secondary_controllers);

        std::stringstream secondary_controller_names;

        for (const auto & controller : secondary_controllers)
        {
            if (controller)
            {
                secondary_controller_names
                    << controller->get_node()->get_name() << " ";
            }
        }

        RCLCPP_INFO(
            get_logger(),
            "Secondary controllers: %s",
            secondary_controller_names.str().c_str());

        if (use_mujoco_hold_)
        {
            // 物理线程启动后、默认控制器激活前立即提交一次保持力矩。
            run_mujoco_hold_cycle(
                rclcpp::Duration(nominal_period));
        }
        else
        {
            // 真机完整保留原来的启动行为。
            read(
                this->now(),
                rclcpp::Duration::from_seconds(0.0));
        }

        /*
        * 5. 如果配置了默认控制器，立即激活。
        *
        * 原代码先 sleep(1s)，再激活默认控制器，
        * 会导致启动过程无意义地等待至少一秒。
        */
        if (!default_controller_.empty())
        {
            const std::string controller_name = default_controller_;

            RCLCPP_INFO(
                get_logger(),
                "Activating default controller '%s'",
                controller_name.c_str());

            if (!activate_controller(controller_name))
            {
                if (use_mujoco_hold_)
                {
                    run_mujoco_hold_cycle(
                        rclcpp::Duration(nominal_period));
                }
                else
                {
                    // 真机保留原有失败清理行为。
                    robot_->get_node()->deactivate();
                }

                throw std::runtime_error(
                    "Failed to activate default controller: " +
                    controller_name);
            }

            // 只有激活成功后才清除。
            default_controller_.clear();
        }

        RCLCPP_INFO(
            get_logger(),
            "Waiting for an active controller...");

        auto next_wait_time = SteadyClock::now();
        auto previous_wait_time =
            next_wait_time - nominal_period;

        while (keep_running_)
        {
            std::shared_ptr<
                controller_interface::ControllerInterface>
                controller;

            active_controller_box_.get(controller);

            if (controller)
            {
                active_controller_ = controller;
                break;
            }

            if (use_mujoco_hold_)
            {
                const auto current_wait_time = SteadyClock::now();
                const auto elapsed =
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        current_wait_time - previous_wait_time);

                previous_wait_time = current_wait_time;

                // 没有活动控制器时，以控制频率持续刷新安全保持力矩。
                run_mujoco_hold_cycle(
                    rclcpp::Duration(elapsed));

                next_wait_time += nominal_period;

                const auto work_finished_time = SteadyClock::now();
                if (work_finished_time >
                    next_wait_time + nominal_period)
                {
                    next_wait_time = work_finished_time;
                }

                std::this_thread::sleep_until(next_wait_time);
            }
            else
            {
                // 真机沿用原来的先等待、再读取状态的行为。
                std::this_thread::sleep_for(100ms);

                const auto current_read_time = SteadyClock::now();
                const auto elapsed =
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        current_read_time - previous_wait_time);

                previous_wait_time = current_read_time;

                read(
                    this->now(),
                    rclcpp::Duration(elapsed));
            }
        }

        /*
        * 7. 退出期间进行清理。
        */
        if (!keep_running_)
        {
            active_controller_.reset();
            running_box_.set(false);
            running_ = false;

            const auto exit_robot_state = robot_->get_node_state();

            if (exit_robot_state.id() ==
                lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
            {
                robot_->get_node()->deactivate();
            }

            return;
        }

        if (!active_controller_)
        {
            if (!use_mujoco_hold_)
            {
                robot_->get_node()->deactivate();
            }

            throw std::runtime_error(
                "Control loop cannot start without an active controller");
        }

        RCLCPP_INFO(
            get_logger(),
            "Controller '%s' is active",
            active_controller_->get_node()->get_name());

        /*
        * 8. 允许 control_loop() 开始运行。
        */
        running_box_.set(true);
        running_ = true;
    }

    void ControlManager::end_loop()
    {
        running_ = false;
        running_box_.set(false);

        std::shared_ptr<controller_interface::ControllerInterface> controller;
        active_controller_box_.get(controller);

        // 旧控制器停止后，MuJoCo 仍保留最后一拍力矩，直到下面提交保持力矩。
        if (controller)
        {
            const auto controller_state = controller->get_node_state();

            if (controller_state.id() ==
                lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
            {
                controller->get_node()->deactivate();
            }
        }

        if (use_mujoco_hold_)
        {
            /*
             * 先提交保持力矩，再清空 active_controller_box_。
             * activate_controller() 只有看到 box 为空后才会激活新控制器，
             * 因而切换顺序是：旧控制器 -> 保持模式 -> 新控制器。
             */
            if (robot_->get_node_state().id() ==
                lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
            {
                const auto nominal_period =
                    std::chrono::nanoseconds(
                        1'000'000'000LL / update_rate_);

                const rclcpp::Time stamp = this->now();
                const rclcpp::Duration period(nominal_period);

                read(stamp, period);

                auto &mode =
                    robot_->get_command_interface().get<int>("mode");

                if (!mode.empty())
                {
                    mode[0] = 0;
                    write(stamp, period);
                }
                else
                {
                    RCLCPP_ERROR(
                        get_logger(),
                        "Cannot enter MuJoCo hold mode: "
                        "command interface 'mode' is empty");
                }
            }

            controller.reset();
            active_controller_box_.set(controller);
            active_controller_.reset();

            // MuJoCo Robot 保持 ACTIVE，物理线程和保持力矩持续运行。
            return;
        }

        // 真机完整保留原来的控制器和 Robot 生命周期行为。
        controller.reset();
        active_controller_box_.set(controller);
        active_controller_.reset();

        const auto robot_state = robot_->get_node_state();
        if (robot_state.id() ==
            lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
        {
            robot_->get_node()->deactivate();
        }
    }

}
