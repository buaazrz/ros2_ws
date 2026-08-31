#include "robot_hardware_interface/sensor_interface.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <thread>
#include <vector>

namespace hardwares
{
    namespace
    {
        constexpr std::uint16_t kAtiHeader = 0x1234;
        constexpr std::uint16_t kStartCommand = 2;
        constexpr std::uint16_t kStopCommand = 0;
        constexpr std::uint32_t kContinuousSamples = 0;
        constexpr std::uint16_t kAtiPort = 49152;
        constexpr std::size_t kResponseBytes = 36;

        template <typename T>
        T read_network_value(const std::uint8_t *source)
        {
            T value{};
            std::memcpy(&value, source, sizeof(T));
            return value;
        }
    }

    class FTATISensor : public hardware_interface::SensorInterface
    {
    public:
        FTATISensor()
            : handle_(-1), force_counts_per_unit_(1.0e6), torque_counts_per_unit_(1.0e6),
              last_sequence_(0), dropped_packets_(0), bad_status_packets_(0) {}

        ~FTATISensor() override
        {
            shutdown_socket();
        }

        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override
        {
            if (SensorInterface::on_configure(previous_state) != CallbackReturn::SUCCESS)
                return CallbackReturn::FAILURE;

            std::string sensor_ip;
            node_->get_parameter_or<std::string>("sensor_ip", sensor_ip, "");
            node_->get_parameter_or<double>(
                "force_counts_per_unit", force_counts_per_unit_, 1.0e6);
            node_->get_parameter_or<double>(
                "torque_counts_per_unit", torque_counts_per_unit_, 1.0e6);
            if (sensor_ip.empty() || !std::isfinite(force_counts_per_unit_) ||
                !std::isfinite(torque_counts_per_unit_) || force_counts_per_unit_ <= 0.0 ||
                torque_counts_per_unit_ <= 0.0)
            {
                RCLCPP_ERROR(node_->get_logger(), "invalid ATI IP or counts-per-unit parameters");
                return CallbackReturn::FAILURE;
            }

            handle_ = ::socket(AF_INET, SOCK_DGRAM, 0);
            if (handle_ < 0)
            {
                RCLCPP_ERROR(node_->get_logger(), "failed to create ATI UDP socket");
                return CallbackReturn::FAILURE;
            }

            // HUMBLE-FIX 14: 100 ms bounds shutdown latency without the old 10 us
            // near-busy-poll loop.
            timeval read_timeout{};
            read_timeout.tv_usec = 100'000;
            if (::setsockopt(handle_, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof(read_timeout)) != 0)
            {
                RCLCPP_ERROR(node_->get_logger(), "failed to configure ATI socket timeout");
                shutdown_socket();
                return CallbackReturn::FAILURE;
            }

            std::memset(&addr_, 0, sizeof(addr_));
            addr_.sin_family = AF_INET;
            addr_.sin_port = htons(kAtiPort);
            if (::inet_pton(AF_INET, sensor_ip.c_str(), &addr_.sin_addr) != 1)
            {
                RCLCPP_ERROR(node_->get_logger(), "invalid ATI sensor_ip: %s", sensor_ip.c_str());
                shutdown_socket();
                return CallbackReturn::FAILURE;
            }
            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override
        {
            if (SensorInterface::on_activate(previous_state) != CallbackReturn::SUCCESS ||
                send_command(kStartCommand) != 8)
                return CallbackReturn::FAILURE;

            last_sequence_ = 0;
            dropped_packets_.store(0, std::memory_order_relaxed);
            bad_status_packets_.store(0, std::memory_order_relaxed);
            is_running_.store(true, std::memory_order_release);
            thread_ = std::make_unique<std::thread>(&FTATISensor::receive_loop, this);
            if (!wait_data_comming())
            {
                is_running_.store(false, std::memory_order_release);
                send_command(kStopCommand);
                stop_thread();
                return CallbackReturn::FAILURE;
            }
            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override
        {
            is_running_.store(false, std::memory_order_release);
            send_command(kStopCommand);
            SensorInterface::on_deactivate(previous_state);
            RCLCPP_INFO(
                node_->get_logger(), "ATI stopped: dropped=%lu bad_status=%lu",
                static_cast<unsigned long>(dropped_packets_.load(std::memory_order_relaxed)),
                static_cast<unsigned long>(bad_status_packets_.load(std::memory_order_relaxed)));
            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override
        {
            is_running_.store(false, std::memory_order_release);
            send_command(kStopCommand);
            stop_thread();
            shutdown_socket();
            return SensorInterface::on_shutdown(previous_state);
        }

    private:
        int send_command(std::uint16_t command)
        {
            if (handle_ < 0)
                return -1;
            std::array<std::uint8_t, 8> request{};
            const std::uint16_t header = htons(kAtiHeader);
            const std::uint16_t network_command = htons(command);
            const std::uint32_t samples = htonl(kContinuousSamples);
            std::memcpy(request.data(), &header, sizeof(header));
            std::memcpy(request.data() + 2, &network_command, sizeof(network_command));
            std::memcpy(request.data() + 4, &samples, sizeof(samples));
            return static_cast<int>(::sendto(
                handle_, request.data(), request.size(), 0,
                reinterpret_cast<const sockaddr *>(&addr_), sizeof(addr_)));
        }

        void receive_loop()
        {
            std::array<std::uint8_t, kResponseBytes> packet{};
            std::vector<double> force(6, 0.0);
            while (is_running_.load(std::memory_order_acquire))
            {
                sockaddr_in source_addr{};
                socklen_t source_len = sizeof(source_addr);
                const auto received = ::recvfrom(
                    handle_, packet.data(), packet.size(), MSG_TRUNC,
                    reinterpret_cast<sockaddr *>(&source_addr), &source_len);
                if (received != static_cast<ssize_t>(packet.size()) ||
                    source_addr.sin_addr.s_addr != addr_.sin_addr.s_addr)
                    continue;

                const auto rdt_sequence = ntohl(read_network_value<std::uint32_t>(packet.data()));
                const auto status = ntohl(read_network_value<std::uint32_t>(packet.data() + 8));
                if (status != 0)
                {
                    bad_status_packets_.fetch_add(1, std::memory_order_relaxed);
                    continue;
                }
                if (last_sequence_ != 0)
                {
                    const std::uint32_t expected = last_sequence_ + 1;
                    if (rdt_sequence != expected)
                    {
                        const std::uint32_t missed = rdt_sequence > expected
                                                         ? rdt_sequence - expected
                                                         : 1U;
                        dropped_packets_.fetch_add(missed, std::memory_order_relaxed);
                    }
                }
                last_sequence_ = rdt_sequence;

                for (std::size_t i = 0; i < force.size(); ++i)
                {
                    const auto raw_network = read_network_value<std::uint32_t>(
                        packet.data() + 12 + i * sizeof(std::uint32_t));
                    const auto raw = static_cast<std::int32_t>(ntohl(raw_network));
                    const double counts_per_unit =
                        i < 3 ? force_counts_per_unit_ : torque_counts_per_unit_;
                    force[i] = static_cast<double>(raw) / counts_per_unit;
                }
                write_sample<double>("force", force);
            }
        }

        void shutdown_socket()
        {
            is_running_.store(false, std::memory_order_release);
            if (handle_ >= 0)
            {
                send_command(kStopCommand);
                ::shutdown(handle_, SHUT_RDWR);
            }
            stop_thread();
            if (handle_ >= 0)
            {
                ::close(handle_);
                handle_ = -1;
            }
        }

        int handle_;
        sockaddr_in addr_{};
        double force_counts_per_unit_;
        double torque_counts_per_unit_;
        std::uint32_t last_sequence_;
        std::atomic<std::uint64_t> dropped_packets_;
        std::atomic<std::uint64_t> bad_status_packets_;
    };

} // namespace hardwares

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(hardwares::FTATISensor, hardware_interface::SensorInterface)
