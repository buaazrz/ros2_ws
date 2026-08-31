#include "robot_hardware_interface/sensor_interface.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <thread>

using namespace robot_math;

namespace hardware_interface
{

    SensorInterface::SensorInterface()
        : update_rate_(0), is_running_(false), sample_sequence_(0), last_sample_time_ns_(0)
    {
    }
    SensorInterface::~SensorInterface()
    {
        stop_thread();
    }

    bool SensorInterface::wait_data_comming(double t)
    {
        const auto start = std::chrono::steady_clock::now();
        const auto initial_sequence = sample_sequence_.load(std::memory_order_acquire);
        while (is_running_.load(std::memory_order_acquire))
        {
            if (sample_sequence_.load(std::memory_order_acquire) != initial_sequence)
                return true;
            const auto elapsed = std::chrono::duration<double>(
                                     std::chrono::steady_clock::now() - start)
                                     .count();
            if (elapsed > t)
            {
                RCLCPP_ERROR(node_->get_logger(), "no data comming!");
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        return false;
    }
    void SensorInterface::stop_thread()
    {
        if (thread_ && thread_->joinable())
        {
            is_running_ = false;
            thread_->join();
        }
        thread_ = nullptr;
    }
    void SensorInterface::read(const rclcpp::Time & /*t*/, const rclcpp::Duration & /*period*/)
    {
        auto &s_d = state_.get<double>();
        for (auto &s : get<double>())
        {
            const auto data = s.second.readFromRT();
            if (data)
            {
                auto &destination = s_d.at(s.first);
                std::copy_n(data->begin(), std::min(data->size(), destination.size()), destination.begin());
            }
        }
        auto &s_int = state_.get<int>();
        for (auto &s : get<int>())
        {
            const auto data = s.second.readFromRT();
            if (data)
            {
                auto &destination = s_int.at(s.first);
                std::copy_n(data->begin(), std::min(data->size(), destination.size()), destination.begin());
            }
        }
        auto &s_b = state_.get<bool>();
        for (auto &s : get<bool>())
        {
            const auto data = s.second.readFromRT();
            if (data)
            {
                auto &destination = s_b.at(s.first);
                std::copy_n(data->begin(), std::min(data->size(), destination.size()), destination.begin());
            }
        }
        auto &age = s_d.at("sample_age_seconds");
        const auto sample_ns = last_sample_time_ns_.load(std::memory_order_relaxed);
        if (sample_ns <= 0)
        {
            age.front() = std::numeric_limits<double>::infinity();
        }
        else
        {
            const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                    std::chrono::steady_clock::now().time_since_epoch())
                                    .count();
            age.front() = std::max(0.0, static_cast<double>(now_ns - sample_ns) * 1.0e-9);
        }
    }

    CallbackReturn SensorInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        stop_thread();
        sample_sequence_.store(0, std::memory_order_release);
        last_sample_time_ns_.store(0, std::memory_order_relaxed);
        return CallbackReturn::SUCCESS;
    }
    CallbackReturn SensorInterface::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        if (SuperClass::on_configure(previous_state) != CallbackReturn::SUCCESS)
            return CallbackReturn::FAILURE;

        node_->get_parameter_or<int>("update_rate", update_rate_, 0);
        auto &double_interface = state_.get<double>();
        auto &double_buffer = get<double>(); // std::get<0>(real_time_buffer_);
        double_buffer.clear();
        for (auto &s : double_interface)
        {
            double_buffer.try_emplace(s.first, s.second);
        }
        auto &int_interface = state_.get<int>();
        auto &int_buffer = get<int>(); //(real_time_buffer_);
        int_buffer.clear();
        for (auto &s : int_interface)
        {
            int_buffer.try_emplace(s.first, s.second);
        }
        auto &bool_interface = state_.get<bool>();
        auto &bool_buffer = get<bool>(); //(real_time_buffer_);
        bool_buffer.clear();
        for (auto &s : bool_interface)
        {
            bool_buffer.try_emplace(s.first, s.second);
        }

        sample_sequence_.store(0, std::memory_order_release);
        last_sample_time_ns_.store(0, std::memory_order_relaxed);
        // Exposed to controllers so stale force samples can trigger a safe stop.
        double_interface.try_emplace(
            "sample_age_seconds", std::vector<double>{std::numeric_limits<double>::infinity()});

        return CallbackReturn::SUCCESS;
    }

}
