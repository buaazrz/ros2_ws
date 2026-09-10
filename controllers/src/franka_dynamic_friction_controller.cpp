#include "robot_controller_interface/controller_interface.hpp"
#include "friction_scan_core.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <pthread.h>
#include <sched.h>
#include <vector>

namespace controllers
{
    // Franka torque-mode residual friction identification. NO gravity/model/friction
    // feedforward is added here. Same-position +/- steady motion cancels common bias.
    // The six other joints are held by PD+bounded I; this is NOT Cartesian scanning.
    class FrankaDynamicFrictionController : public controller_interface::ControllerInterface
    {
        using Joint = friction_scan::Joint;
        using Clock = std::chrono::steady_clock;
        enum Phase
        {
            SETTLE = 0,
            POSITION = 1,
            SCAN = 2,
            DONE = 3,
            FAULT = 4
        };
        enum Reason
        {
            NONE = 0,
            COMPLETE = 1,
            USER_STOP = 2,
            BAD_STATE = 3,
            PERIOD = 4,
            ENVELOPE = 5,
            VELOCITY = 6,
            TRACKING = 7,
            SETTLE_TIMEOUT = 8,
            LOG_OVERFLOW = 9,
            LOG_IO = 10,
            EXPERIMENT_TIMEOUT = 11
        };
        struct Step
        {
            Phase phase{SETTLE};
            double target{};
            friction_scan::Profile profile;
            int speed_id{-1}, repeat{-1}, direction{};
        };
        struct Sample
        {
            double time{}, dt{}, host_dt{}, step_time{}, target_speed{}, qr_test{}, vr_test{}, ar_test{};
            double success{std::numeric_limits<double>::quiet_NaN()};
            int phase{}, reason{}, speed_id{}, repeat{}, direction{}, valid{}, joint{};
            unsigned limited_mask{};
            Joint q{}, dq{}, raw{}, command{}, feedback{}, measured{}, integral{};
        };
        // Fixed SPSC queue: producer=update(), consumer=logger thread. A full queue
        // faults the experiment instead of silently dropping identification data.
        static constexpr std::size_t queue_size = 4096;

    public:
        ~FrankaDynamicFrictionController() override { close_session(); }
        CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
        {
            try
            {
                if (!robot_ || robot_->dof != 7)
                    throw std::runtime_error("requires 7 DOF");
                get("joint_number", joint_number_, 2); // human numbering: 1..7
                get("repeats", repeats_, 3);
                get("speeds", speeds_, std::vector<double>{0.005, 0.01, 0.02});
                get("half_range_rad", half_range_, 0.05);
                get("test_joint_min_rad", q_min_, -1.5);
                get("test_joint_max_rad", q_max_, 1.5);
                get("ramp_time_s", ramp_time_, 1.0);
                get("max_acceleration", max_acceleration_, 0.04);
                get("reposition_speed", reposition_speed_, 0.02);
                get("cruise_guard_s", guard_, 0.5);
                get("settle_time_s", settle_time_, 2.0);
                get("settle_timeout_s", settle_timeout_, 15.0);
                get("settle_position_tol", settle_position_tol_, 0.003);
                get("settle_velocity_tol", settle_velocity_tol_, 0.003);
                get("sample_speed_rel_tol", speed_rel_tol_, 0.15);
                get("sample_speed_abs_tol", speed_abs_tol_, 0.0005);
                get("sample_hold_position_tol", hold_position_tol_, 0.003);
                get("sample_hold_velocity_tol", hold_velocity_tol_, 0.003);
                get("max_position_error", max_position_error_, 0.035);
                get("max_other_displacement", max_other_displacement_, 0.04);
                get("max_velocity", max_velocity_, 0.08);
                get("max_period_s", max_period_, 0.005);
                get("max_torque_rate", max_torque_rate_, 100.0);
                get("max_duration_s", max_duration_, 1200.0);
                get("log_directory", log_directory_, std::string("/home/luo/experiment_logs/dynamic_friction"));
                get("require_measured_torque", require_measured_, false);
                get_joints("kp", kp_, {100, 100, 100, 80, 50, 40, 30});
                get_joints("kd", kd_, {10, 10, 10, 8, 5, 4, 3});
                get_joints("ki", ki_, {20, 20, 20, 15, 10, 8, 5});
                get_joints("integral_limits", integral_limits_, {2, 2, 2, 1.5, 1, 1, 0.7});
                get_joints("torque_limits", torque_limits_, {8, 8, 8, 6, 3, 3, 2});
                const std::vector<double> positive{half_range_, ramp_time_, max_acceleration_,
                                                   reposition_speed_, guard_, settle_time_, settle_timeout_, settle_position_tol_,
                                                   settle_velocity_tol_, speed_rel_tol_, speed_abs_tol_, hold_position_tol_,
                                                   hold_velocity_tol_, max_position_error_, max_other_displacement_, max_velocity_,
                                                   max_period_, max_torque_rate_, max_duration_};
                for (double v : positive)
                    if (!std::isfinite(v) || v <= 0)
                        throw std::runtime_error("nonpositive/nonfinite parameter");
                if (joint_number_ < 1 || joint_number_ > 7 || repeats_ < 1 || repeats_ > 50 ||
                    speeds_.empty() || speeds_.size() > 30 || !std::isfinite(q_min_) || !std::isfinite(q_max_) ||
                    q_min_ >= q_max_ || settle_timeout_ <= settle_time_ || log_directory_.empty() ||
                    reposition_speed_ >= max_velocity_ || max_period_ > 0.02)
                    throw std::runtime_error("invalid scan/settle parameters");
                joint_ = joint_number_ - 1;
                for (std::size_t i = 0; i < 7; ++i)
                {
                    if (kp_[i] <= 0 || kd_[i] <= 0 || torque_limits_[i] <= 0 || integral_limits_[i] > torque_limits_[i])
                        throw std::runtime_error("invalid servo gains/limits");
                }
                for (double v : speeds_)
                {
                    if (!std::isfinite(v) || v <= 0 || v >= max_velocity_)
                        throw std::runtime_error("invalid scan speed");
                    friction_scan::Profile p;
                    p.reset(-half_range_, half_range_, v, ramp_time_, max_acceleration_);
                    if (std::abs(p.speed() - v) > 1e-10 || p.flat() < 2 * guard_ + 0.5)
                        throw std::runtime_error("range too short for requested speed and usable cruise");
                }
                std::filesystem::create_directories(log_directory_);
                queue_ = std::make_unique<std::array<Sample, queue_size>>();
                return CallbackReturn::SUCCESS;
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(node_->get_logger(), "Dynamic friction configure: %s", e.what());
                return CallbackReturn::FAILURE;
            }
        }
        CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
        {
            std::lock_guard<std::mutex> lock(gate_);
            close_session();
            try
            {
                if (!read_joint("position", center_) || !read_joint("velocity", velocity_) ||
                    !read_joint("torque", previous_))
                    throw std::runtime_error("missing/invalid position, velocity or torque state");
                for (std::size_t i = 0; i < 7; ++i)
                {
                    if (std::abs(velocity_[i]) > settle_velocity_tol_)
                        throw std::runtime_error("activate from rest");
                    if (std::abs(previous_[i]) > torque_limits_[i])
                        throw std::runtime_error("initial torque reference exceeds configured limit");
                }
                if (center_[joint_] - half_range_ - max_position_error_ < q_min_ ||
                    center_[joint_] + half_range_ + max_position_error_ > q_max_)
                    throw std::runtime_error("scan envelope outside configured test-joint bounds");
                Joint measured;
                measured_available_ = read_joint("measured_torque", measured);
                if (require_measured_ && !measured_available_)
                    throw std::runtime_error("measured_torque interface required");
                if (!measured_available_)
                    RCLCPP_WARN(node_->get_logger(), "No measured_torque: tau_J columns will be NaN; command residual can still be identified.");
                // Validate command interfaces before starting the writer/RT session.
                auto &cmd = command_->get<double>("torque");
                auto &mode = command_->get<int>("mode");
                if (cmd.size() != 7 || mode.empty())
                    throw std::runtime_error("invalid command torque/mode interface");
                make_plan();
                const auto stamp = std::chrono::duration_cast<std::chrono::microseconds>(
                                       std::chrono::system_clock::now().time_since_epoch())
                                       .count();
                path_ = (std::filesystem::path(log_directory_) / ("joint_" + std::to_string(joint_number_) +
                                                                  "_" + std::to_string(stamp) + ".csv"))
                            .string();
                log_.open(path_, std::ios::out | std::ios::trunc);
                if (!log_)
                    throw std::runtime_error("cannot open CSV");
                log_ << std::setprecision(17);
                write_header();
                log_.flush();
                if (!log_)
                    throw std::runtime_error("cannot write CSV header");
                write_index_ = 0;
                read_index_ = 0;
                logger_stop_ = false;
                io_failed_ = false;
                stop_ = false;
                reason_ = NONE;
                step_index_ = 0;
                step_time_ = time_ = good_settle_ = 0;
                first_update_ = true;
                integral_.fill(0);
                last_host_ = Clock::now();
                // Warm start bounded I at existing reference, avoiding a command jump.
                for (std::size_t i = 0; i < 7; ++i)
                    integral_[i] = std::clamp(previous_[i], -integral_limits_[i], integral_limits_[i]);
                writer_ = std::thread([this]
                                      { writer_loop(); });
                active_ = true;
                RCLCPP_INFO(node_->get_logger(), "Joint %d dynamic friction: %.6f +/- %.6f rad; CSV %s",
                            joint_number_, center_[joint_], half_range_, path_.c_str());
                return CallbackReturn::SUCCESS;
            }
            catch (const std::exception &e)
            {
                close_session();
                RCLCPP_ERROR(node_->get_logger(), "Dynamic friction activate: %s", e.what());
                return CallbackReturn::FAILURE;
            }
        }
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
        {
            // Manager must stop read-update-write before lifecycle teardown.
            std::lock_guard<std::mutex> lock(gate_);
            close_session();
            return CallbackReturn::SUCCESS;
        }
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &s) override { return on_deactivate(s); }
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &s) override { return on_deactivate(s); }
        CallbackReturn on_error(const rclcpp_lifecycle::State &s) override { return on_deactivate(s); }
        bool requests_stop() const noexcept { return stop_.load(std::memory_order_acquire); }

        void update(const rclcpp::Time &, const rclcpp::Duration &period) override
        {
            std::unique_lock<std::mutex> lock(gate_, std::try_to_lock);
            if (!lock.owns_lock() || !active_)
                return;
            auto &cmd = command_->get<double>("torque");
            auto &mode = command_->get<int>("mode");
            if (cmd.size() != 7 || mode.empty())
            {
                stop_reason(BAD_STATE);
                return;
            }
            mode[0] = 3; // framework torque mode
            if (stop_)
            {
                std::copy(previous_.begin(), previous_.end(), cmd.begin());
                return;
            }
            const double dt = period.seconds();
            // prepare_loop() calls update(period=0) once before starting FCI.
            if (first_update_ && dt == 0)
            {
                std::copy(previous_.begin(), previous_.end(), cmd.begin());
                return;
            }
            const auto now = Clock::now();
            const double host = first_update_ ? dt : std::chrono::duration<double>(now - last_host_).count();
            last_host_ = now;
            first_update_ = false;
            Sample s;
            s.time = time_;
            s.dt = dt;
            s.host_dt = host;
            s.joint = joint_number_;
            if (!read_joint("position", s.q) || !read_joint("velocity", s.dq) || !read_joint("torque", s.feedback))
            {
                std::copy(previous_.begin(), previous_.end(), cmd.begin());
                stop_reason(BAD_STATE);
                return;
            }
            s.measured.fill(std::numeric_limits<double>::quiet_NaN());
            if (measured_available_ && !read_joint("measured_torque", s.measured))
            {
                std::copy(previous_.begin(), previous_.end(), cmd.begin());
                stop_reason(BAD_STATE);
                return;
            }
            const auto &states = state_->get<double>();
            const auto success = states.find("success");
            if (success != states.end() && !success->second.empty())
                s.success = success->second[0];
            if (!std::isfinite(dt) || dt <= 0 || dt > max_period_)
                stop_reason(PERIOD);
            if (io_failed_)
                stop_reason(LOG_IO);
            if (time_ > max_duration_)
                stop_reason(EXPERIMENT_TIMEOUT);
            for (std::size_t i = 0; i < 7; ++i)
            {
                if (std::abs(s.dq[i]) > max_velocity_)
                    stop_reason(VELOCITY);
                const double extent = (i == static_cast<std::size_t>(joint_)) ? half_range_ + max_position_error_ : max_other_displacement_;
                if (std::abs(s.q[i] - center_[i]) > extent)
                    stop_reason(ENVELOPE);
            }
            if (s.q[joint_] < q_min_ || s.q[joint_] > q_max_)
                stop_reason(ENVELOPE);
            if (stop_)
            {
                s.phase = FAULT;
                s.reason = reason_;
                s.command = previous_;
                s.raw = previous_;
                enqueue(s);
                std::copy(previous_.begin(), previous_.end(), cmd.begin());
                return;
            }
            if (step_index_ >= plan_.size())
            {
                s.phase = DONE;
                s.reason = COMPLETE;
                s.command = previous_;
                s.raw = previous_;
                enqueue(s);
                std::copy(previous_.begin(), previous_.end(), cmd.begin());
                stop_reason(COMPLETE);
                return;
            }
            const Step &step = plan_[step_index_];
            friction_scan::Reference ref{step.target, 0, 0, false};
            if (step.phase != SETTLE)
                ref = step.profile.at(step_time_);
            Joint qr = center_, vr{};
            qr[joint_] = ref.q;
            vr[joint_] = ref.v;
            if (std::abs(s.q[joint_] - ref.q) > max_position_error_)
                stop_reason(TRACKING);
            s.phase = step.phase;
            s.step_time = step_time_;
            s.speed_id = step.speed_id;
            s.repeat = step.repeat;
            s.direction = step.direction;
            s.target_speed = step.speed_id >= 0 ? speeds_[step.speed_id] : 0;
            s.qr_test = ref.q;
            s.vr_test = ref.v;
            s.ar_test = ref.a;
            if (stop_)
            {
                s.phase = FAULT;
                s.reason = reason_;
                s.command = previous_;
                s.raw = previous_;
                enqueue(s);
                std::copy(previous_.begin(), previous_.end(), cmd.begin());
                return;
            }
            const auto out = friction_scan::servo(s.q, s.dq, qr, vr, previous_, kp_, kd_, ki_,
                                                  torque_limits_, integral_limits_, dt, max_torque_rate_, integral_);
            s.raw = out.raw;
            s.command = out.command;
            s.integral = integral_;
            s.limited_mask = out.limited_mask;
            s.valid = step.phase == SCAN && ref.cruise &&
                      step_time_ >= step.profile.ramp() + guard_ &&
                      step_time_ <= step.profile.ramp() + step.profile.flat() - guard_ &&
                      std::abs(s.dq[joint_] - ref.v) <= std::max(speed_abs_tol_, speed_rel_tol_ * s.target_speed) &&
                      step.direction * s.dq[joint_] > 0 && !out.limited_mask;
            for (std::size_t i = 0; i < 7; ++i)
                if (i != static_cast<std::size_t>(joint_))
                {
                    if (std::abs(s.q[i] - center_[i]) > hold_position_tol_ || std::abs(s.dq[i]) > hold_velocity_tol_)
                        s.valid = 0;
                }
            previous_ = out.command;
            std::copy(previous_.begin(), previous_.end(), cmd.begin());
            if (!enqueue(s))
            {
                stop_reason(LOG_OVERFLOW);
                return;
            }
            time_ += dt;
            step_time_ += dt;
            if (step.phase == SETTLE)
            {
                bool settled = true;
                for (std::size_t i = 0; i < 7; ++i)
                    settled = settled && std::abs(qr[i] - s.q[i]) <= settle_position_tol_ && std::abs(s.dq[i]) <= settle_velocity_tol_;
                good_settle_ = settled ? good_settle_ + dt : 0;
                if (good_settle_ >= settle_time_)
                    advance();
                else if (step_time_ > settle_timeout_)
                    stop_reason(SETTLE_TIMEOUT);
            }
            else if (step_time_ >= step.profile.duration())
                advance();
        }

    private:
        template <typename T>
        void get(const char *key, T &v, const T &fallback)
        {
            node_->get_parameter_or<T>(key, v, fallback);
        }
        void get_joints(const char *key, Joint &dest, const Joint &fallback)
        {
            std::vector<double> v;
            get(key, v, std::vector<double>(fallback.begin(), fallback.end()));
            if (v.size() != 7)
                throw std::runtime_error(std::string(key) + " must have 7 values");
            for (std::size_t i = 0; i < 7; ++i)
            {
                if (!std::isfinite(v[i]) || v[i] < 0)
                    throw std::runtime_error(std::string("invalid ") + key);
                dest[i] = v[i];
            }
        }
        bool read_joint(const char *key, Joint &v) const
        {
            const auto &map = state_->get<double>();
            const auto it = map.find(key);
            if (it == map.end() || it->second.size() != 7)
                return false;
            for (std::size_t i = 0; i < 7; ++i)
            {
                v[i] = it->second[i];
                if (!std::isfinite(v[i]))
                    return false;
            }
            return true;
        }
        void make_plan()
        {
            plan_.clear();
            double current = center_[joint_];
            auto settle = [&]
            { Step s; s.phase=SETTLE; s.target=current; plan_.push_back(s); };
            auto move = [&](double target, double speed, Phase phase, int sid, int rep, int direction)
            {
                Step s;
                s.phase = phase;
                s.target = target;
                s.speed_id = sid;
                s.repeat = rep;
                s.direction = direction;
                s.profile.reset(current, target, speed, ramp_time_, max_acceleration_);
                plan_.push_back(s);
                current = target;
            };
            settle();
            for (std::size_t k = 0; k < speeds_.size(); ++k)
                for (int r = 0; r < repeats_; ++r)
                {
                    // Alternate the first direction to reduce order/thermal bias.
                    const int first = (r % 2 == 0) ? 1 : -1;
                    const double start = center_[joint_] - first * half_range_, end = center_[joint_] + first * half_range_;
                    move(start, reposition_speed_, POSITION, -1, r, 0);
                    settle();
                    move(end, speeds_[k], SCAN, static_cast<int>(k), r, first);
                    settle();
                    move(start, speeds_[k], SCAN, static_cast<int>(k), r, -first);
                    settle();
                }
            move(center_[joint_], reposition_speed_, POSITION, -1, -1, 0);
            settle();
            double minimum = 0;
            for (const auto &s : plan_)
                minimum += s.phase == SETTLE ? settle_time_ : s.profile.duration();
            if (minimum + 5 > max_duration_)
                throw std::runtime_error("max_duration_s shorter than planned experiment");
        }
        void advance()
        {
            ++step_index_;
            step_time_ = good_settle_ = 0;
        }
        void stop_reason(Reason reason)
        {
            if (!stop_.load(std::memory_order_relaxed))
            {
                reason_ = reason;
                stop_.store(true, std::memory_order_release);
            }
        }
        bool enqueue(const Sample &s)
        {
            const auto w = write_index_.load(std::memory_order_relaxed);
            if (w - read_index_.load(std::memory_order_acquire) >= queue_size)
                return false;
            (*queue_)[w % queue_size] = s;
            write_index_.store(w + 1, std::memory_order_release);
            return true;
        }
        void write_header()
        {
            log_ << "# torque_command: external command without added gravity; NOT raw motor torque\n"
                 << "# tau_feedback: state interface torque must map to Franka tau_J_d\n"
                 << "# tau_measured: measured_torque must map to Franka tau_J (NaN if unavailable)\n"
                 << "# phase: 0=settle 1=position 2=scan 3=done 4=fault; repeat/speed_id are zero-based\n";
            const auto config = [&](const char *name, const Joint &values)
            {
                log_ << "# " << name << '=';
                for (double x : values)
                    log_ << x << ' ';
                log_ << '\n';
            };
            config("center", center_);
            config("kp", kp_);
            config("kd", kd_);
            config("ki", ki_);
            config("torque_limits", torque_limits_);
            config("integral_limits", integral_limits_);
            log_ << "# half_range=" << half_range_ << " ramp_time=" << ramp_time_ << " max_acceleration=" << max_acceleration_
                 << " max_torque_rate=" << max_torque_rate_ << " measured_available=" << measured_available_ << '\n';
            log_ << "time,dt,host_dt,step_time,joint,phase,reason,speed_id,repeat,direction,target_speed,q_ref,v_ref,a_ref,valid,limited_mask,success";
            for (const char *prefix : {"q", "dq", "tau_raw", "tau_command", "tau_feedback", "tau_measured", "tau_integral"})
                for (int i = 1; i <= 7; ++i)
                    log_ << ',' << prefix << i;
            log_ << '\n';
        }
        void write_sample(const Sample &s)
        {
            log_ << s.time << ',' << s.dt << ',' << s.host_dt << ',' << s.step_time << ',' << s.joint << ',' << s.phase << ',' << s.reason
                 << ',' << s.speed_id << ',' << s.repeat << ',' << s.direction << ',' << s.target_speed << ',' << s.qr_test
                 << ',' << s.vr_test << ',' << s.ar_test << ',' << s.valid << ',' << s.limited_mask << ',' << s.success;
            for (const Joint *a : {&s.q, &s.dq, &s.raw, &s.command, &s.feedback, &s.measured, &s.integral})
                for (double v : *a)
                    log_ << ',' << v;
            log_ << '\n';
        }
        void writer_loop() noexcept
        {
            // Do not inherit FIFO priority if activation happened on an RT thread.
            sched_param scheduling{};
            if (pthread_setschedparam(pthread_self(), SCHED_OTHER, &scheduling) != 0)
            {
                io_failed_ = true;
                return;
            }
            try
            {
                auto last_flush = Clock::now();
                while (true)
                {
                    auto r = read_index_.load(std::memory_order_relaxed);
                    const auto w = write_index_.load(std::memory_order_acquire);
                    // Bound each batch so periodic flush remains possible under continuous input.
                    for (std::size_t n = 0; r < w && n < 512; ++r, ++n)
                        write_sample((*queue_)[r % queue_size]);
                    read_index_.store(r, std::memory_order_release);
                    if (Clock::now() - last_flush > std::chrono::milliseconds(500))
                    {
                        log_.flush();
                        last_flush = Clock::now();
                    }
                    if (!log_)
                    {
                        io_failed_ = true;
                        break;
                    }
                    if (logger_stop_.load(std::memory_order_acquire) && r == write_index_.load(std::memory_order_acquire))
                        break;
                    if (r == w)
                        std::this_thread::sleep_for(std::chrono::milliseconds(2));
                }
                log_.flush();
                if (!log_)
                    io_failed_ = true;
            }
            catch (...)
            {
                io_failed_ = true;
            }
        }
        static const char *reason_text(int r)
        {
            switch (r)
            {
            case COMPLETE:
                return "complete";
            case USER_STOP:
                return "user_stop";
            case BAD_STATE:
                return "invalid_state";
            case PERIOD:
                return "invalid_period";
            case ENVELOPE:
                return "position_envelope";
            case VELOCITY:
                return "velocity_limit";
            case TRACKING:
                return "tracking_error";
            case SETTLE_TIMEOUT:
                return "settle_timeout";
            case LOG_OVERFLOW:
                return "log_queue_overflow";
            case LOG_IO:
                return "log_io_error";
            case EXPERIMENT_TIMEOUT:
                return "experiment_timeout";
            default:
                return "none";
            }
        }
        void close_session() noexcept
        {
            active_ = false;
            const bool had_file = log_.is_open();
            if (had_file && reason_ == NONE)
                reason_ = USER_STOP;
            logger_stop_.store(true, std::memory_order_release);
            if (writer_.joinable())
                writer_.join();
            if (log_.is_open())
                log_.close();
            if (had_file)
            {
                try
                {
                    std::ofstream status(path_ + ".status.txt");
                    status << "reason=" << reason_text(reason_) << "\nreason_code=" << reason_ << "\n"
                           << "io_failed=" << io_failed_.load() << "\nrows=" << read_index_.load() << "\n";
                    if (node_)
                        RCLCPP_INFO(node_->get_logger(), "Dynamic friction finished: %s; io_failed=%d; CSV %s",
                                    reason_text(reason_), static_cast<int>(io_failed_.load()), path_.c_str());
                }
                catch (...)
                { /* destructor cannot throw */
                }
            }
        }
        int joint_number_{2}, joint_{1}, repeats_{3};
        std::vector<double> speeds_;
        std::vector<Step> plan_;
        double half_range_{}, q_min_{}, q_max_{}, ramp_time_{}, max_acceleration_{}, reposition_speed_{}, guard_{};
        double settle_time_{}, settle_timeout_{}, settle_position_tol_{}, settle_velocity_tol_{};
        double speed_rel_tol_{}, speed_abs_tol_{}, hold_position_tol_{}, hold_velocity_tol_{};
        double max_position_error_{}, max_other_displacement_{}, max_velocity_{}, max_period_{}, max_torque_rate_{}, max_duration_{};
        Joint kp_{}, kd_{}, ki_{}, integral_limits_{}, torque_limits_{}, center_{}, velocity_{}, previous_{}, integral_{};
        bool require_measured_{false}, measured_available_{false}, active_{false}, first_update_{true};
        double time_{}, step_time_{}, good_settle_{};
        std::size_t step_index_{};
        int reason_{NONE};
        std::string log_directory_, path_;
        Clock::time_point last_host_;
        std::mutex gate_;
        std::ofstream log_;
        std::thread writer_;
        std::atomic<bool> stop_{false}, logger_stop_{true}, io_failed_{false};
        std::unique_ptr<std::array<Sample, queue_size>> queue_;
        std::atomic<std::size_t> write_index_{0}, read_index_{0};
    };
} // namespace controllers
PLUGINLIB_EXPORT_CLASS(controllers::FrankaDynamicFrictionController, controller_interface::ControllerInterface)
