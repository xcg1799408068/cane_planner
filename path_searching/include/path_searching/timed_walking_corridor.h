#ifndef _TIMED_WALKING_CORRIDOR_H_
#define _TIMED_WALKING_CORRIDOR_H_

#include <Eigen/Eigen>

#include <path_searching/timed_trajectory.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace cane_planner
{

struct TimedWalkingCorridorSegment
{
    struct HalfPlane2D
    {
        Eigen::Vector2d normal = Eigen::Vector2d::UnitX();
        double offset = 0.0;

        double value(const Eigen::Vector2d &point) const
        {
            return normal.dot(point) + offset;
        }
    };

    double t_start = 0.0;
    double t_end = 0.0;
    Eigen::Vector2d start = Eigen::Vector2d::Zero();
    Eigen::Vector2d end = Eigen::Vector2d::Zero();
    Eigen::Vector2d forward = Eigen::Vector2d::UnitX();
    Eigen::Vector2d left = Eigen::Vector2d::UnitY();
    std::vector<Eigen::Vector2d> centerline;
    double half_width = 0.0;
    std::vector<HalfPlane2D> half_planes;
    std::vector<Eigen::Vector2d> polygon;
    bool feasible = true;
    bool blocked_static = false;
    bool blocked_dynamic = false;

    struct DynamicEllipseObstacle
    {
        Eigen::Vector2d center = Eigen::Vector2d::Zero();
        Eigen::Vector2d forward = Eigen::Vector2d::UnitX();
        Eigen::Vector2d left = Eigen::Vector2d::UnitY();
        double longitudinal_radius = 0.0;
        double lateral_radius = 0.0;
        std::vector<Eigen::Vector2d> vertices;

        double penetrationDepth(const Eigen::Vector2d &point) const
        {
            if (vertices.size() >= 3)
            {
                double signed_area = 0.0;
                for (size_t i = 0; i < vertices.size(); ++i)
                {
                    const Eigen::Vector2d &a = vertices[i];
                    const Eigen::Vector2d &b = vertices[(i + 1) % vertices.size()];
                    signed_area += a.x() * b.y() - a.y() * b.x();
                }
                const double orientation = signed_area >= 0.0 ? 1.0 : -1.0;
                double min_inside_dist = std::numeric_limits<double>::infinity();
                for (size_t i = 0; i < vertices.size(); ++i)
                {
                    const Eigen::Vector2d &a = vertices[i];
                    const Eigen::Vector2d &b = vertices[(i + 1) % vertices.size()];
                    const Eigen::Vector2d edge = b - a;
                    const double edge_len = std::max(1e-6, edge.norm());
                    const double signed_dist =
                        orientation *
                        (edge.x() * (point.y() - a.y()) -
                         edge.y() * (point.x() - a.x())) /
                        edge_len;
                    if (signed_dist < 0.0)
                        return 0.0;
                    min_inside_dist = std::min(min_inside_dist, signed_dist);
                }
                return std::isfinite(min_inside_dist) ? min_inside_dist : 0.0;
            }

            const double a = std::max(1e-6, longitudinal_radius);
            const double b = std::max(1e-6, lateral_radius);
            const Eigen::Vector2d f =
                forward.norm() > 1e-6 ? forward.normalized() : Eigen::Vector2d::UnitX();
            const Eigen::Vector2d l =
                left.norm() > 1e-6 ? left.normalized() : Eigen::Vector2d(-f.y(), f.x());
            const Eigen::Vector2d rel = point - center;
            const double u = rel.dot(f) / a;
            const double v = rel.dot(l) / b;
            const double normalized_dist = std::sqrt(u * u + v * v);
            if (normalized_dist >= 1.0)
                return 0.0;
            return std::min(a, b) * (1.0 - normalized_dist);
        }
    };

    std::vector<DynamicEllipseObstacle> dynamic_obstacles;
};

struct TimedWalkingCorridor
{
    TimedTrajectorySource source = TimedTrajectorySource::EMPTY;
    std::vector<TimedWalkingCorridorSegment> segments;

    bool valid() const { return !segments.empty(); }
    double tStart() const { return valid() ? segments.front().t_start : 0.0; }
    double tEnd() const { return valid() ? segments.back().t_end : 0.0; }
    int segmentIndexAtTime(const double t) const
    {
        for (int i = 0; i < static_cast<int>(segments.size()); ++i)
        {
            const auto &segment = segments[static_cast<size_t>(i)];
            if (t >= segment.t_start && t < segment.t_end)
            {
                return i;
            }
        }
        return -1;
    }
    const TimedWalkingCorridorSegment *segmentAtTime(const double t) const
    {
        const int index = segmentIndexAtTime(t);
        if (index < 0)
        {
            return nullptr;
        }
        return &segments[static_cast<size_t>(index)];
    }
    static double outsideDistanceForSegment(const TimedWalkingCorridorSegment &segment,
                                            const Eigen::Vector2d &point)
    {
        if (!segment.half_planes.empty())
        {
            double max_violation = 0.0;
            for (const auto &hp : segment.half_planes)
            {
                const double norm = std::max(1e-6, hp.normal.norm());
                max_violation = std::max(max_violation, hp.value(point) / norm);
            }
            return max_violation;
        }

        const Eigen::Vector2d delta = segment.end - segment.start;
        const double length = delta.norm();
        if (length < 1e-6)
            return 0.0;

        const Eigen::Vector2d forward =
            segment.forward.norm() > 1e-6 ? segment.forward.normalized() : delta / length;
        const Eigen::Vector2d left =
            segment.left.norm() > 1e-6 ? segment.left.normalized()
                                       : Eigen::Vector2d(-forward.y(), forward.x());
        const Eigen::Vector2d rel = point - segment.start;
        const double s = rel.dot(forward);
        const double l = rel.dot(left);
        const double ds = std::max({0.0, -s, s - length});
        const double dl = std::max(0.0, std::abs(l) - segment.half_width);
        if (ds <= 0.0)
            return dl;
        if (dl <= 0.0)
            return ds;
        return std::sqrt(ds * ds + dl * dl);
    }
    double outsideDistanceAtTime(const double t, const Eigen::Vector2d &point) const
    {
        const int index = segmentIndexAtTime(t);
        if (index < 0)
            return 0.0;

        const auto &segment = segments[static_cast<size_t>(index)];
        double best = outsideDistanceForSegment(segment, point);
        if (segment.half_planes.empty())
            return best;

        if (index > 0)
        {
            const auto &prev = segments[static_cast<size_t>(index - 1)];
            if (!prev.half_planes.empty())
            {
                best = std::min(best, outsideDistanceForSegment(prev, point));
            }
        }
        if (index + 1 < static_cast<int>(segments.size()))
        {
            const auto &next = segments[static_cast<size_t>(index + 1)];
            if (!next.half_planes.empty())
            {
                best = std::min(best, outsideDistanceForSegment(next, point));
            }
        }
        return best;
    }

    double dynamicObstacleViolationAtTime(const double t,
                                          const Eigen::Vector2d &point) const
    {
        const auto *segment = segmentAtTime(t);
        if (!segment)
            return 0.0;

        double max_violation = 0.0;
        for (const auto &obstacle : segment->dynamic_obstacles)
        {
            max_violation = std::max(max_violation, obstacle.penetrationDepth(point));
        }
        return max_violation;
    }

    void clear()
    {
        source = TimedTrajectorySource::EMPTY;
        segments.clear();
    }
};

} // namespace cane_planner

#endif // _TIMED_WALKING_CORRIDOR_H_
