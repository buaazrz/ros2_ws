#pragma once
// No ROS dependency: trajectory/servo primitives can be tested on a desktop.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace friction_scan
{
    constexpr double pi = 3.14159265358979323846;
    using Joint = std::array<double, 7>;
    struct Reference
    {
        double q{}, v{}, a{};
        bool cruise{};
    };

    // Sinusoidal acceleration/deceleration of velocity: continuous q, v, a.
    // Ramp distance = v*T/2 per end. Both scan directions use the same geometry.
    class Profile
    {
    public:
        void reset(double from, double to, double speed, double ramp_min, double amax)
        {
            if (!(speed > 0 && ramp_min > 0 && amax > 0))
                throw std::invalid_argument("invalid motion profile");
            from_ = from;
            to_ = to;
            sign_ = to >= from ? 1.0 : -1.0;
            const double distance = std::abs(to - from);
            if (distance < 1e-12)
            {
                speed_ = ramp_ = flat_ = duration_ = 0;
                return;
            }
            // Reduce peak speed if the requested move is too short for both ramps.
            speed_ = std::min({speed, distance / ramp_min,
                               std::sqrt(2.0 * amax * distance / pi)});
            ramp_ = std::max(ramp_min, pi * speed_ / (2.0 * amax));
            flat_ = std::max(0.0, distance / speed_ - ramp_);
            duration_ = 2.0 * ramp_ + flat_;
        }
        Reference at(double t) const
        {
            if (duration_ == 0 || t >= duration_)
                return {to_, 0, 0, false};
            if (t <= 0)
                return {from_, 0, 0, false};
            double x, v, a;
            bool flat = false;
            if (t < ramp_)
            {
                const double u = pi * t / ramp_;
                x = 0.5 * speed_ * (t - ramp_ / pi * std::sin(u));
                v = 0.5 * speed_ * (1 - std::cos(u));
                a = 0.5 * speed_ * pi / ramp_ * std::sin(u);
            }
            else if (t < ramp_ + flat_)
            {
                x = 0.5 * speed_ * ramp_ + speed_ * (t - ramp_);
                v = speed_;
                a = 0;
                flat = true;
            }
            else
            {
                const double s = t - ramp_ - flat_, u = pi * s / ramp_;
                x = 0.5 * speed_ * ramp_ + speed_ * flat_ +
                    0.5 * speed_ * (s + ramp_ / pi * std::sin(u));
                v = 0.5 * speed_ * (1 + std::cos(u));
                a = -0.5 * speed_ * pi / ramp_ * std::sin(u);
            }
            return {from_ + sign_ * x, sign_ * v, sign_ * a, flat};
        }
        double duration() const { return duration_; }
        double ramp() const { return ramp_; }
        double flat() const { return flat_; }
        double speed() const { return speed_; }

    private:
        double from_{}, to_{}, sign_{1}, speed_{}, ramp_{}, flat_{}, duration_{};
    };

    struct ServoOutput
    {
        Joint raw{}, command{};
        unsigned limited_mask{};
    };
    // Integral state is in Nm. Conditional integration suppresses windup on either
    // absolute or slew saturation. Total output (P+D+I), not I alone, is identified.
    inline ServoOutput servo(const Joint &q, const Joint &dq, const Joint &qr,
                             const Joint &vr, const Joint &previous,
                             const Joint &kp, const Joint &kd, const Joint &ki,
                             const Joint &torque_limits, const Joint &integral_limits,
                             double dt, double torque_rate, Joint &integral)
    {
        ServoOutput out;
        for (std::size_t i = 0; i < 7; ++i)
        {
            const double error = qr[i] - q[i];
            const double old_i = integral[i];
            double next_i = std::clamp(old_i + ki[i] * error * dt,
                                       -integral_limits[i], integral_limits[i]);
            const double pd = kp[i] * error + kd[i] * (vr[i] - dq[i]);
            // previous must already be within the absolute limit (activation checks it).
            const double low = std::max(-torque_limits[i], previous[i] - torque_rate * dt);
            const double high = std::min(torque_limits[i], previous[i] + torque_rate * dt);
            double raw = pd + next_i;
            if ((raw > high && next_i > old_i) || (raw < low && next_i < old_i))
                next_i = old_i;
            integral[i] = next_i;
            raw = pd + next_i;
            out.raw[i] = raw;
            out.command[i] = std::clamp(raw, low, high);
            if (std::abs(out.command[i] - raw) > 1e-9)
                out.limited_mask |= 1u << i;
        }
        return out;
    }
} // namespace friction_scan
