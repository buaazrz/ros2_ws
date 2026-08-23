#include <cerrno>
#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

#include "control_node/control_manager.h"
#include "rclcpp/executors.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_helpers.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
    // 使用 rclcpp 默认 SIGINT/SIGTERM handler。
    // 不要自己在 signal_handler 中调用 RCLCPP、join、shared_ptr 等非异步信号安全操作。
    rclcpp::init(argc, argv);

    constexpr int kDefaultSchedPriority = 50;

    auto executor =
        std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    rclcpp::NodeOptions node_options;
    node_options.allow_undeclared_parameters(true);
    node_options.automatically_declare_parameters_from_overrides(true);

    std::vector<std::string> node_arguments;
    for (int i = 1; i < argc; ++i)
    {
        if (node_arguments.empty() && std::string(argv[i]) != "--ros-args")
        {
            continue;
        }
        node_arguments.emplace_back(argv[i]);
    }
    node_options.arguments(node_arguments);

    auto cm = std::make_shared<control_node::ControlManager>(
        executor, "control_node", "", node_options);

    const bool lock_memory =
        cm->get_parameter_or<bool>("lock_memory", true);

    if (lock_memory)
    {
        const auto result = realtime_tools::lock_memory();
        if (!result.first)
        {
            RCLCPP_WARN(
                cm->get_logger(),
                "Unable to lock memory: '%s'",
                result.second.c_str());
        }
    }

    const int cpu_affinity =
        cm->get_parameter_or<int>("cpu_affinity", -1);

    if (cpu_affinity >= 0)
    {
        const auto result =
            realtime_tools::set_current_thread_affinity(cpu_affinity);

        if (!result.first)
        {
            RCLCPP_WARN(
                cm->get_logger(),
                "Unable to set CPU affinity: '%s'",
                result.second.c_str());
        }
    }

    const int thread_priority =
        cm->get_parameter_or<int>(
            "thread_priority", kDefaultSchedPriority);

    /*
     * simulation_backend:
     *
     *   boost_ode : ControlManager::start_simulation()
     *   mujoco    : ControlManager::control_loop()
     *   hardware  : ControlManager::control_loop()
     */
    const std::string default_backend =
        cm->is_simulation() ? "boost_ode" : "hardware";

    const std::string simulation_backend =
        cm->get_parameter_or<std::string>(
            "simulation_backend", default_backend);

    if (simulation_backend != "boost_ode" &&
        simulation_backend != "mujoco" &&
        simulation_backend != "hardware")
    {
        RCLCPP_FATAL(
            cm->get_logger(),
            "Unsupported simulation_backend '%s'. "
            "Expected: boost_ode, mujoco or hardware.",
            simulation_backend.c_str());

        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(
        cm->get_logger(),
        "Update rate: %d Hz, backend: %s",
        cm->get_update_rate(),
        simulation_backend.c_str());

    std::thread control_thread(
        [cm, thread_priority, simulation_backend]()
        {
            if (!realtime_tools::configure_sched_fifo(thread_priority))
            {
                RCLCPP_WARN(
                    cm->get_logger(),
                    "Could not enable FIFO scheduling: errno=%d (%s)",
                    errno,
                    std::strerror(errno));
            }
            else
            {
                RCLCPP_INFO(
                    cm->get_logger(),
                    "FIFO scheduling enabled with priority %d",
                    thread_priority);
            }

            try
            {
                while (cm->is_keep_running())
                {
                    cm->prepare_loop();

                    if (!cm->is_keep_running())
                    {
                        break;
                    }

                    if (simulation_backend == "boost_ode")
                    {
                        // 保留原来的内部动力学积分方式。
                        cm->start_simulation(10.0);
                    }
                    else
                    {
                        /*
                         * mujoco 和 hardware 都走：
                         *
                         * RobotInterface::read()
                         * ControllerInterface::update()
                         * RobotInterface::write()
                         *
                         * MuJoCo 的 mj_step() 由 MujocoSimulation
                         * 自己的物理线程执行。
                         */
                        cm->control_loop();
                    }

                    cm->end_loop();
                }
            }
            catch (const std::exception & e)
            {
                RCLCPP_ERROR(
                    cm->get_logger(),
                    "Control loop exception: %s",
                    e.what());

                cm->interrupt();

                // 让 executor->spin() 返回。
                if (rclcpp::ok())
                {
                    rclcpp::shutdown();
                }
            }

            try
            {
                cm->shutdown_robot();
            }
            catch (const std::exception & e)
            {
                RCLCPP_ERROR(
                    cm->get_logger(),
                    "Robot shutdown exception: %s",
                    e.what());
            }
        });

    // ROS 回调线程。
    executor->spin();

    // 默认信号处理器调用 rclcpp::shutdown() 后，spin() 会返回。
    // 此时安全地通知控制线程退出并 join。
    cm->interrupt();

    if (control_thread.joinable())
    {
        control_thread.join();
    }

    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }

    return 0;
}