#include <path_searching/timed_trajectory_builder.h>

#include <algorithm>
#include <cmath>

namespace cane_planner
{
namespace
{
constexpr double kEps = 1e-6;

double yawOf(const Eigen::Vector2d &delta, double fallback)
{
    if (delta.norm() < kEps)
        return fallback;
    return std::atan2(delta.y(), delta.x());
}

} // namespace

TimedTrajectory TimedTrajectoryBuilder::buildNominal(
    const std::vector<Eigen::Vector3d> &previous_mppi_path,
    const std::vector<Eigen::Vector2d> &global_reference,
    double mppi_dt,
    double reference_speed,
    double max_length)
{
    TimedTrajectory timed = fromPreviousMppi(previous_mppi_path, mppi_dt, max_length);
    if (timed.valid())
        return timed;
    return fromGlobalReference(global_reference, reference_speed, max_length);
}

TimedTrajectory TimedTrajectoryBuilder::fromPreviousMppi(
    const std::vector<Eigen::Vector3d> &path,
    double dt,
    double max_length)
{
    TimedTrajectory timed;
    if (path.size() < 2 || dt <= kEps)
        return timed;

    timed.source = TimedTrajectorySource::PREVIOUS_MPPI;
    double accum = 0.0;
    for (size_t i = 0; i < path.size(); ++i)
    {
        if (i > 0)
        {
            accum += (path[i].head(2) - path[i - 1].head(2)).norm();
            if (max_length > kEps && accum > max_length)
                break;
        }

        TimedTrajectoryPoint point;
        point.position = path[i].head(2);
        point.yaw = path[i](2);
        point.t_from_now = static_cast<double>(i) * dt;
        timed.points.push_back(point);
    }

    if (!timed.valid())
        timed.clear();
    return timed;
}

TimedTrajectory TimedTrajectoryBuilder::fromGlobalReference(
    const std::vector<Eigen::Vector2d> &path,
    double speed,
    double max_length)
{
    TimedTrajectory timed;
    if (path.size() < 2 || speed <= kEps)
        return timed;

    timed.source = TimedTrajectorySource::ASTAR_BOOTSTRAP;
    double accum = 0.0;
    double fallback_yaw = yawOf(path[1] - path[0], 0.0);

    TimedTrajectoryPoint first;
    first.position = path[0];
    first.yaw = fallback_yaw;
    first.t_from_now = 0.0;
    timed.points.push_back(first);

    Eigen::Vector2d prev = path[0];
    for (size_t i = 1; i < path.size(); ++i)
    {
        const Eigen::Vector2d seg = path[i] - prev;
        const double seg_len = seg.norm();
        if (seg_len < kEps)
            continue;

        Eigen::Vector2d next = path[i];
        double next_accum = accum + seg_len;
        if (max_length > kEps && next_accum > max_length)
        {
            const double remain = std::max(0.0, max_length - accum);
            next = prev + (remain / seg_len) * seg;
            next_accum = max_length;
        }

        TimedTrajectoryPoint point;
        point.position = next;
        point.yaw = yawOf(next - prev, fallback_yaw);
        point.t_from_now = next_accum / speed;
        timed.points.push_back(point);

        fallback_yaw = point.yaw;
        accum = next_accum;
        prev = next;
        if (max_length > kEps && accum >= max_length - kEps)
            break;
    }

    if (!timed.valid())
        timed.clear();
    return timed;
}

} // namespace cane_planner
