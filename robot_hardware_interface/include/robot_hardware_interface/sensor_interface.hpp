#ifndef SENSOR_INTERFACE_HPP
#define SENSOR_INTERFACE_HPP

#include "robot_hardware_interface/hardware_interface.hpp"
#include "robot_math/robot_math.hpp"
#include "realtime_tools/realtime_buffer.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <vector>
namespace hardware_interface
{

    class SensorInterface : public HardwareInterface
    {
    public:
        // HUMBLE-FIX 13: Store samples by value in pre-sized double buffers. The old
        // shared_ptr<vector> design allocated once for every UDP packet.
        typedef std::unordered_map<std::string, realtime_tools::RealtimeBuffer<std::vector<double>>> RealtimeBufferDouble;
        typedef std::unordered_map<std::string, realtime_tools::RealtimeBuffer<std::vector<int>>> RealtimeBufferInt;
        typedef std::unordered_map<std::string, realtime_tools::RealtimeBuffer<std::vector<bool>>> RealtimeBufferBool;
        typedef std::tuple<RealtimeBufferDouble, RealtimeBufferInt, RealtimeBufferBool> RealtimeBufferType;
        using SuperClass = HardwareInterface;
        ~SensorInterface();
        SensorInterface();
        void read(const rclcpp::Time & /*t*/, const rclcpp::Duration & /*period*/) override;
        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
        void stop_thread();
        bool wait_data_comming(double t = 10);
    protected:
        template <typename T>
        realtime_tools::RealtimeBuffer<std::vector<T>> &get(const std::string &name)
        {
            return std::get<std::unordered_map<std::string, realtime_tools::RealtimeBuffer<std::vector<T>>>>(real_time_buffer_).at(name);
        }

        template <typename T>
        std::unordered_map<std::string, realtime_tools::RealtimeBuffer<std::vector<T>>> &get()
        {
            return std::get<std::unordered_map<std::string, realtime_tools::RealtimeBuffer<std::vector<T>>>>(real_time_buffer_);
        }

        template <typename T>
        void write_sample(const std::string &name, const std::vector<T> &sample)
        {
            get<T>(name).writeFromNonRT(sample);
            const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            last_sample_time_ns_.store(now_ns, std::memory_order_relaxed);
            sample_sequence_.fetch_add(1, std::memory_order_release);
        }
    protected:
        int update_rate_;
        std::unique_ptr<std::thread> thread_;
        std::atomic_bool is_running_;
        std::atomic<std::uint64_t> sample_sequence_;
        std::atomic<std::int64_t> last_sample_time_ns_;
        RealtimeBufferType real_time_buffer_;
    };

} // namespace hardware

#endif // FT_SENSOR_INTERFACE_HPP
