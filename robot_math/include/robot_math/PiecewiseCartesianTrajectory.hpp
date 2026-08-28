#pragma once

#include "robot_math/robot_math.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <vector>

namespace robot_math
{

/**
 * @brief Piecewise Cartesian trajectory with a quintic time-scaling law.
 *
 * Input layout for every waypoint:
 *
 *   [time, x, y, z, rx, ry, rz]
 *
 * where [rx, ry, rz] is a rotation vector, consistent with
 * robot_math::rv_to_quaternion().
 *
 * Every segment is planned independently with
 *
 *   h(s) = 10 s^3 - 15 s^4 + 6 s^5,  s in [0, 1].
 *
 * Consequently, translational and angular velocity/acceleration are zero at
 * every waypoint. If two adjacent positions or orientations are identical,
 * that component remains exactly constant over the complete segment.
 */
class PiecewiseCartesianTrajectory
{
public:
    PiecewiseCartesianTrajectory() = default;

    explicit PiecewiseCartesianTrajectory(const std::vector<double> &traj)
    {
        set_traj(traj);
    }

    [[nodiscard]] bool is_empty() const noexcept
    {
        return num_of_point_ == 0;
    }

    void clear() noexcept
    {
        time_.clear();
        pos_waypoint_.clear();
        orientation_waypoint_.clear();
        num_of_point_ = 0;
        T_ = 0.0;
    }

    /**
     * @brief Return the final trajectory timestamp.
     *
     * VariableImpedanceController inserts the current TCP as the t=0
     * waypoint, so this is also the trajectory duration used by the
     * controller's traj_time_.
     */
    [[nodiscard]] double total_time() const noexcept
    {
        return T_;
    }

    /**
     * @brief Parse and store a Cartesian waypoint sequence.
     * @return true on success; false if the input is invalid.
     *
     * Requirements:
     * - at least two waypoints;
     * - exactly seven values per waypoint;
     * - finite values;
     * - non-negative, strictly increasing timestamps.
     */
    bool set_traj(const std::vector<double> &traj)
    {
        clear();

        constexpr std::size_t kWaypointWidth = 7;

        if (traj.size() < 2 * kWaypointWidth ||
            traj.size() % kWaypointWidth != 0)
        {
            return false;
        }

        const std::size_t point_count = traj.size() / kWaypointWidth;

        time_.reserve(point_count);
        pos_waypoint_.reserve(point_count);
        orientation_waypoint_.reserve(point_count);

        for (std::size_t i = 0; i < point_count; ++i)
        {
            const std::size_t base = i * kWaypointWidth;

            for (std::size_t k = 0; k < kWaypointWidth; ++k)
            {
                if (!std::isfinite(traj[base + k]))
                {
                    clear();
                    return false;
                }
            }

            const double waypoint_time = traj[base];

            if (waypoint_time < 0.0 ||
                (!time_.empty() && waypoint_time <= time_.back()))
            {
                clear();
                return false;
            }

            time_.push_back(waypoint_time);

            pos_waypoint_.emplace_back(
                traj[base + 1],
                traj[base + 2],
                traj[base + 3]);

            Eigen::Quaterniond orientation = rv_to_quaternion(
                Eigen::Vector3d(
                    traj[base + 4],
                    traj[base + 5],
                    traj[base + 6]));

            orientation.normalize();

            // Store quaternions on one continuous hemisphere. q and -q
            // represent the same rotation, but the sign matters to SLERP.
            if (!orientation_waypoint_.empty() &&
                orientation_waypoint_.back().dot(orientation) < 0.0)
            {
                orientation.coeffs() *= -1.0;
            }

            orientation_waypoint_.push_back(orientation);
        }

        num_of_point_ = point_count;
        T_ = time_.back();
        return true;
    }

    /**
     * @brief Evaluate desired pose, spatial velocity and spatial acceleration.
     *
     * V layout:
     *   [angular velocity in base frame, linear velocity in base frame]
     *
     * dV uses the corresponding angular and linear acceleration layout.
     * Values before the first timestamp and after the last timestamp are held
     * at the endpoint with zero velocity and acceleration; extrapolation is
     * deliberately disabled.
     */
    void evaluate(
        double t,
        Eigen::Matrix4d &Td,
        Eigen::Vector6d &V,
        Eigen::Vector6d &dV) const
    {
        Td.setIdentity();
        V.setZero();
        dV.setZero();

        if (is_empty())
        {
            return;
        }

        if (num_of_point_ == 1 || t <= time_.front())
        {
            set_endpoint(0, Td);
            return;
        }

        if (t >= time_.back())
        {
            set_endpoint(num_of_point_ - 1, Td);
            return;
        }

        const auto upper = std::upper_bound(time_.begin(), time_.end(), t);
        std::size_t idx = static_cast<std::size_t>(
            std::distance(time_.begin(), upper) - 1);

        idx = std::min(idx, num_of_point_ - 2);

        const double t0 = time_[idx];
        const double t1 = time_[idx + 1];
        const double duration = t1 - t0;

        // Strictly positive because set_traj() checks timestamp ordering.
        const double s = std::clamp((t - t0) / duration, 0.0, 1.0);

        const double s2 = s * s;
        const double s3 = s2 * s;
        const double s4 = s3 * s;
        const double s5 = s4 * s;

        const double h =
            10.0 * s3 -
            15.0 * s4 +
             6.0 * s5;

        const double h_dot =
            (30.0 * s2 -
             60.0 * s3 +
             30.0 * s4) /
            duration;

        const double h_ddot =
            (60.0 * s -
             180.0 * s2 +
             120.0 * s3) /
            (duration * duration);

        // Translation: independent quintic interpolation for this segment.
        const Eigen::Vector3d delta_p =
            pos_waypoint_[idx + 1] - pos_waypoint_[idx];

        const Eigen::Vector3d p =
            pos_waypoint_[idx] + h * delta_p;

        Td.block<3, 1>(0, 3) = p;
        V.tail<3>() = h_dot * delta_p;
        dV.tail<3>() = h_ddot * delta_p;

        // Orientation: shortest-path SLERP driven by the same quintic scalar.
        Eigen::Quaterniond q0 = orientation_waypoint_[idx];
        Eigen::Quaterniond q1 = orientation_waypoint_[idx + 1];

        if (q0.dot(q1) < 0.0)
        {
            q1.coeffs() *= -1.0;
        }

        const Eigen::Quaterniond q = q0.slerp(h, q1).normalized();
        const Eigen::Matrix3d R = q.toRotationMatrix();
        Td.block<3, 3>(0, 0) = R;

        // Relative rotation vector expressed in the starting body frame.
        const Eigen::Quaterniond q_relative =
            (q0.conjugate() * q1).normalized();
        const Eigen::AngleAxisd relative_angle_axis(q_relative);
        const Eigen::Vector3d relative_rotation_vector =
            relative_angle_axis.axis() * relative_angle_axis.angle();

        // Controller convention expects desired angular velocity and
        // acceleration in the base frame. For R = R0 Exp(h * phi^), R*phi is
        // constant because phi is the interpolation rotation axis.
        const Eigen::Vector3d rotation_axis_base =
            R * relative_rotation_vector;

        V.head<3>() = h_dot * rotation_axis_base;
        dV.head<3>() = h_ddot * rotation_axis_base;
    }

private:
    void set_endpoint(std::size_t idx, Eigen::Matrix4d &Td) const
    {
        Td.block<3, 3>(0, 0) =
            orientation_waypoint_[idx].toRotationMatrix();
        Td.block<3, 1>(0, 3) = pos_waypoint_[idx];
    }

    double T_{0.0};
    std::vector<double> time_;
    std::size_t num_of_point_{0};
    std::vector<Eigen::Vector3d> pos_waypoint_;
    std::vector<Eigen::Quaterniond> orientation_waypoint_;
};

}  // namespace robot_math