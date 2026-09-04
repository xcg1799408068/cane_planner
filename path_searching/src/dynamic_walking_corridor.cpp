#include <path_searching/dynamic_walking_corridor.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace cane_planner
{

namespace
{
Eigen::Vector2d normalizedOrDefault(const Eigen::Vector2d &v)
{
    if (v.norm() < 1e-6)
        return Eigen::Vector2d::UnitX();
    return v.normalized();
}

double cross2d(const Eigen::Vector2d &a, const Eigen::Vector2d &b)
{
    return a.x() * b.y() - a.y() * b.x();
}

double obstacleRadius(size_t idx,
                      const std::vector<Eigen::Vector3d> &obs_size,
                      double min_radius)
{
    double r = min_radius;
    if (idx < obs_size.size())
        r = std::max(r, 0.5 * std::max(obs_size[idx](0), obs_size[idx](1)));
    return r;
}

double polylineLength(const std::vector<Eigen::Vector2d> &path)
{
    double length = 0.0;
    for (size_t i = 0; i + 1 < path.size(); ++i)
        length += (path[i + 1] - path[i]).norm();
    return length;
}

std::vector<Eigen::Vector2d> truncatePolylineAtLength(
    const std::vector<Eigen::Vector2d> &path,
    double target_length)
{
    std::vector<Eigen::Vector2d> out;
    if (path.empty())
        return out;

    out.push_back(path.front());
    if (path.size() == 1 || target_length <= 0.0)
        return out;

    double accum = 0.0;
    for (size_t i = 0; i + 1 < path.size(); ++i)
    {
        const Eigen::Vector2d a = path[i];
        const Eigen::Vector2d b = path[i + 1];
        const Eigen::Vector2d seg = b - a;
        const double seg_len = seg.norm();
        if (seg_len < 1e-6)
            continue;

        if (accum + seg_len >= target_length)
        {
            const double u = std::max(0.0, std::min(1.0, (target_length - accum) / seg_len));
            const Eigen::Vector2d p = a + u * seg;
            if ((p - out.back()).norm() > 1e-6)
                out.push_back(p);
            return out;
        }

        if ((b - out.back()).norm() > 1e-6)
            out.push_back(b);
        accum += seg_len;
    }
    return out;
}

Eigen::Vector2d polylineDirectionAtStart(const std::vector<Eigen::Vector2d> &path)
{
    for (size_t i = 0; i + 1 < path.size(); ++i)
    {
        Eigen::Vector2d seg = path[i + 1] - path[i];
        if (seg.norm() > 1e-6)
            return seg.normalized();
    }
    return Eigen::Vector2d::UnitX();
}

Eigen::Vector2d interpolatePolyline(const std::vector<Eigen::Vector2d> &path,
                                    double distance)
{
    if (path.empty())
        return Eigen::Vector2d::Zero();
    if (path.size() == 1 || distance <= 0.0)
        return path.front();

    double accum = 0.0;
    for (size_t i = 0; i + 1 < path.size(); ++i)
    {
        const Eigen::Vector2d seg = path[i + 1] - path[i];
        const double seg_len = seg.norm();
        if (seg_len < 1e-6)
            continue;
        if (accum + seg_len >= distance)
            return path[i] + ((distance - accum) / seg_len) * seg;
        accum += seg_len;
    }
    return path.back();
}

double interpolateTimeAtPolylineDistance(const std::vector<Eigen::Vector2d> &path,
                                         const std::vector<double> &times,
                                         double distance)
{
    if (path.empty() || times.size() != path.size())
        return 0.0;
    if (path.size() == 1 || distance <= 0.0)
        return times.front();

    double accum = 0.0;
    for (size_t i = 0; i + 1 < path.size(); ++i)
    {
        const Eigen::Vector2d seg = path[i + 1] - path[i];
        const double seg_len = seg.norm();
        if (seg_len < 1e-6)
            continue;
        if (accum + seg_len >= distance)
        {
            const double u = std::max(0.0, std::min(1.0, (distance - accum) / seg_len));
            return times[i] + u * (times[i + 1] - times[i]);
        }
        accum += seg_len;
    }
    return times.back();
}

bool hasTimedPolyline(const DynamicWalkingCorridor::Candidate &candidate)
{
    return candidate.centerline.size() >= 2 &&
           candidate.centerline_times.size() == candidate.centerline.size();
}

bool insideHalfPlanes(
    const std::vector<TimedWalkingCorridorSegment::HalfPlane2D> &half_planes,
    const Eigen::Vector2d &point,
    const double eps = 1e-6)
{
    for (const auto &hp : half_planes)
    {
        if (hp.value(point) > eps)
            return false;
    }
    return true;
}

std::vector<Eigen::Vector2d> polygonFromHalfPlanes(
    const std::vector<TimedWalkingCorridorSegment::HalfPlane2D> &half_planes)
{
    std::vector<Eigen::Vector2d> vertices;
    for (size_t i = 0; i < half_planes.size(); ++i)
    {
        for (size_t j = i + 1; j < half_planes.size(); ++j)
        {
            const Eigen::Vector2d n0 = half_planes[i].normal;
            const Eigen::Vector2d n1 = half_planes[j].normal;
            const double det = cross2d(n0, n1);
            if (std::abs(det) < 1e-8)
                continue;
            const double c0 = -half_planes[i].offset;
            const double c1 = -half_planes[j].offset;
            const Eigen::Vector2d p((c0 * n1.y() - n0.y() * c1) / det,
                                    (n0.x() * c1 - c0 * n1.x()) / det);
            if (insideHalfPlanes(half_planes, p, 1e-5))
            {
                bool duplicate = false;
                for (const auto &v : vertices)
                {
                    if ((v - p).norm() < 1e-4)
                    {
                        duplicate = true;
                        break;
                    }
                }
                if (!duplicate)
                    vertices.push_back(p);
            }
        }
    }

    if (vertices.size() < 3)
        return {};

    Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
    for (const auto &v : vertices)
        centroid += v;
    centroid /= static_cast<double>(vertices.size());
    std::sort(vertices.begin(), vertices.end(),
              [&](const Eigen::Vector2d &a, const Eigen::Vector2d &b)
              {
                  return std::atan2(a.y() - centroid.y(), a.x() - centroid.x()) <
                         std::atan2(b.y() - centroid.y(), b.x() - centroid.x());
              });
    return vertices;
}

std::vector<Eigen::Vector2d> sampleDynamicEllipse(
    const TimedWalkingCorridorSegment::DynamicEllipseObstacle &obstacle,
    const int samples = 32)
{
    std::vector<Eigen::Vector2d> points;
    points.reserve(samples);
    const Eigen::Vector2d f = normalizedOrDefault(obstacle.forward);
    const Eigen::Vector2d l = obstacle.left.norm() > 1e-6
                                  ? obstacle.left.normalized()
                                  : Eigen::Vector2d(-f.y(), f.x());
    const double a = std::max(1e-3, obstacle.longitudinal_radius);
    const double b = std::max(1e-3, obstacle.lateral_radius);
    for (int i = 0; i < samples; ++i)
    {
        const double theta = 2.0 * M_PI * static_cast<double>(i) /
                             static_cast<double>(samples);
        points.push_back(obstacle.center +
                         a * std::cos(theta) * f +
                         b * std::sin(theta) * l);
    }
    return points;
}

std::vector<Eigen::Vector2d> convexHull2D(std::vector<Eigen::Vector2d> points)
{
    if (points.size() < 3)
        return points;

    std::sort(points.begin(), points.end(),
              [](const Eigen::Vector2d &a, const Eigen::Vector2d &b)
              {
                  if (std::abs(a.x() - b.x()) > 1e-9)
                      return a.x() < b.x();
                  return a.y() < b.y();
              });
    points.erase(std::unique(points.begin(), points.end(),
                             [](const Eigen::Vector2d &a, const Eigen::Vector2d &b)
                             {
                                 return (a - b).norm() < 1e-6;
                             }),
                 points.end());
    if (points.size() < 3)
        return points;

    std::vector<Eigen::Vector2d> hull;
    hull.reserve(points.size() * 2);
    for (const auto &p : points)
    {
        while (hull.size() >= 2 &&
               cross2d(hull.back() - hull[hull.size() - 2],
                       p - hull.back()) <= 1e-9)
        {
            hull.pop_back();
        }
        hull.push_back(p);
    }

    const size_t lower_size = hull.size();
    for (int i = static_cast<int>(points.size()) - 2; i >= 0; --i)
    {
        const auto &p = points[static_cast<size_t>(i)];
        while (hull.size() > lower_size &&
               cross2d(hull.back() - hull[hull.size() - 2],
                       p - hull.back()) <= 1e-9)
        {
            hull.pop_back();
        }
        hull.push_back(p);
    }
    if (!hull.empty())
        hull.pop_back();
    return hull;
}

std::vector<Eigen::Vector2d> buildDynamicObstacleFootprint(
    const Eigen::Vector2d &center,
    const Eigen::Vector2d &velocity,
    const Eigen::Vector2d &fallback_forward,
    const double base_radius,
    const double prediction_extension)
{
    const double radius = std::max(1e-3, base_radius);
    const double speed = velocity.norm();
    const Eigen::Vector2d forward =
        speed > 1e-3 ? velocity / speed : normalizedOrDefault(fallback_forward);
    const Eigen::Vector2d left(-forward.y(), forward.x());

    std::vector<Eigen::Vector2d> points;
    points.reserve(5);

    const double rear_offset = 1.25 * radius;
    const double rear_half_width = 0.45 * radius;
    const double shoulder_offset = -0.20 * radius;
    const double shoulder_half_width = 1.10 * radius;
    const double apex_offset =
        std::max(2.75 * radius, 1.10 * radius + 1.35 * prediction_extension);

    const Eigen::Vector2d rear_center = center - rear_offset * forward;
    const Eigen::Vector2d shoulder_center = center + shoulder_offset * forward;
    points.push_back(rear_center - rear_half_width * left);
    points.push_back(rear_center + rear_half_width * left);
    points.push_back(shoulder_center + shoulder_half_width * left);
    points.push_back(shoulder_center - shoulder_half_width * left);
    points.push_back(center + apex_offset * forward);

    return convexHull2D(points);
}

std::vector<Eigen::Vector2d> sampleDynamicObstacleBoundary(
    const TimedWalkingCorridorSegment::DynamicEllipseObstacle &obstacle,
    const double ds = 0.15)
{
    if (obstacle.vertices.size() < 3)
        return sampleDynamicEllipse(obstacle);

    std::vector<Eigen::Vector2d> points;
    for (size_t i = 0; i < obstacle.vertices.size(); ++i)
    {
        const Eigen::Vector2d a = obstacle.vertices[i];
        const Eigen::Vector2d b = obstacle.vertices[(i + 1) % obstacle.vertices.size()];
        points.push_back(a);
        const double len = (b - a).norm();
        const int steps = std::max(1, static_cast<int>(std::floor(len / std::max(0.05, ds))));
        for (int k = 1; k < steps; ++k)
        {
            const double u = static_cast<double>(k) / static_cast<double>(steps);
            points.push_back(a + u * (b - a));
        }
    }
    return points;
}

Eigen::Vector2d closestPointOnSegment(const Eigen::Vector2d &a,
                                      const Eigen::Vector2d &b,
                                      const Eigen::Vector2d &point)
{
    const Eigen::Vector2d ab = b - a;
    const double len_sq = ab.squaredNorm();
    if (len_sq < 1e-9)
        return a;
    double u = (point - a).dot(ab) / len_sq;
    u = std::max(0.0, std::min(1.0, u));
    return a + u * ab;
}

void addFiriLikeSupportHalfPlane(
    const Eigen::Vector2d &a,
    const Eigen::Vector2d &b,
    const Eigen::Vector2d &obstacle,
    const double clearance,
    std::vector<TimedWalkingCorridorSegment::HalfPlane2D> &half_planes)
{
    const Eigen::Vector2d ab = b - a;
    const double len_sq = ab.squaredNorm();
    if (len_sq < 1e-9)
        return;

    const Eigen::Vector2d closest = closestPointOnSegment(a, b, obstacle);
    Eigen::Vector2d to_obstacle = obstacle - closest;
    const double dist = to_obstacle.norm();
    if (dist < 1e-4)
        return;
    to_obstacle /= dist;

    const double seed_limit = std::max(to_obstacle.dot(a), to_obstacle.dot(b));
    const double boundary = to_obstacle.dot(obstacle) - std::max(0.0, clearance);
    if (boundary <= seed_limit + 1e-5)
        return;

    TimedWalkingCorridorSegment::HalfPlane2D hp;
    hp.normal = to_obstacle;
    hp.offset = -boundary;

    if (hp.value(a) <= 1e-5 && hp.value(b) <= 1e-5 && hp.value(obstacle) > 1e-6)
        half_planes.push_back(hp);
}

double obstacleSupportMinimum(
    const TimedWalkingCorridorSegment::DynamicEllipseObstacle &obstacle,
    const Eigen::Vector2d &normal)
{
    if (obstacle.vertices.size() >= 3)
    {
        double min_proj = std::numeric_limits<double>::infinity();
        for (const auto &v : obstacle.vertices)
            min_proj = std::min(min_proj, normal.dot(v));
        return min_proj;
    }

    const Eigen::Vector2d f = normalizedOrDefault(obstacle.forward);
    const Eigen::Vector2d l = obstacle.left.norm() > 1e-6
                                  ? obstacle.left.normalized()
                                  : Eigen::Vector2d(-f.y(), f.x());
    const double a = std::max(1e-3, obstacle.longitudinal_radius);
    const double b = std::max(1e-3, obstacle.lateral_radius);
    const double nf = normal.dot(f);
    const double nl = normal.dot(l);
    return normal.dot(obstacle.center) -
           std::sqrt(a * a * nf * nf + b * b * nl * nl);
}

double polygonArea(const std::vector<Eigen::Vector2d> &polygon);

bool addDynamicObstacleSupportHalfPlane(
    const TimedWalkingCorridorSegment::DynamicEllipseObstacle &obstacle,
    const Eigen::Vector2d &normal,
    const double clearance,
    std::vector<TimedWalkingCorridorSegment::HalfPlane2D> &half_planes)
{
    Eigen::Vector2d unit_normal = normal;
    const double normal_len = unit_normal.norm();
    if (normal_len < 1e-4)
        return false;
    unit_normal /= normal_len;

    const double boundary =
        obstacleSupportMinimum(obstacle, unit_normal) - std::max(0.0, clearance);

    TimedWalkingCorridorSegment::HalfPlane2D hp;
    hp.normal = unit_normal;
    hp.offset = -boundary;
    half_planes.push_back(hp);
    return true;
}

int addDynamicObstacleSupportHalfPlane(
    const Eigen::Vector2d &a,
    const Eigen::Vector2d &b,
    const TimedWalkingCorridorSegment::DynamicEllipseObstacle &obstacle,
    const double clearance,
    std::vector<TimedWalkingCorridorSegment::HalfPlane2D> &half_planes)
{
    std::vector<Eigen::Vector2d> normals;
    normals.reserve(33);

    const Eigen::Vector2d center_normal =
        obstacle.center - closestPointOnSegment(a, b, obstacle.center);
    if (center_normal.norm() > 1e-4)
        normals.push_back(center_normal);

    const auto boundary_points = sampleDynamicObstacleBoundary(obstacle);
    for (const auto &point : boundary_points)
    {
        const Eigen::Vector2d n = point - closestPointOnSegment(a, b, point);
        if (n.norm() > 1e-4)
            normals.push_back(n);
    }

    bool found = false;
    double best_area = -1.0;
    std::vector<TimedWalkingCorridorSegment::HalfPlane2D> best_half_planes;

    for (const auto &normal : normals)
    {
        std::vector<TimedWalkingCorridorSegment::HalfPlane2D> trial = half_planes;
        if (!addDynamicObstacleSupportHalfPlane(obstacle, normal, clearance, trial))
            continue;
        if (trial.back().value(a) > 1e-5 || trial.back().value(b) > 1e-5)
            continue;
        const auto polygon = polygonFromHalfPlanes(trial);
        if (polygon.size() >= 3)
        {
            const double area = polygonArea(polygon);
            if (!found || area > best_area)
            {
                found = true;
                best_area = area;
                best_half_planes.swap(trial);
            }
        }
    }

    if (!found)
        return 0;

    half_planes.swap(best_half_planes);
    return 1;
}

struct FiriFrame2D
{
    Eigen::Vector2d center = Eigen::Vector2d::Zero();
    Eigen::Matrix2d axes = Eigen::Matrix2d::Identity();
    Eigen::Vector2d radii = Eigen::Vector2d::Ones();
};

Eigen::Vector2d transformToFiriFrame(const FiriFrame2D &frame,
                                     const Eigen::Vector2d &point)
{
    Eigen::Vector2d y = frame.axes.transpose() * (point - frame.center);
    y.x() /= std::max(1e-3, frame.radii.x());
    y.y() /= std::max(1e-3, frame.radii.y());
    return y;
}

TimedWalkingCorridorSegment::HalfPlane2D transformHalfPlaneToFiriFrame(
    const FiriFrame2D &frame,
    const TimedWalkingCorridorSegment::HalfPlane2D &hp)
{
    TimedWalkingCorridorSegment::HalfPlane2D out;
    out.normal = frame.radii.asDiagonal() * frame.axes.transpose() * hp.normal;
    out.offset = hp.normal.dot(frame.center) + hp.offset;
    return out;
}

TimedWalkingCorridorSegment::HalfPlane2D transformHalfPlaneFromFiriFrame(
    const FiriFrame2D &frame,
    const TimedWalkingCorridorSegment::HalfPlane2D &hp)
{
    const Eigen::Matrix2d forward =
        frame.radii.cwiseInverse().asDiagonal() * frame.axes.transpose();
    TimedWalkingCorridorSegment::HalfPlane2D out;
    out.normal = forward.transpose() * hp.normal;
    out.offset = hp.offset - out.normal.dot(frame.center);
    return out;
}

double halfPlaneDistanceToOrigin(
    const TimedWalkingCorridorSegment::HalfPlane2D &hp);

bool buildFiriObstacleTangent(
    const Eigen::Vector2d &seed_a,
    const Eigen::Vector2d &seed_b,
    const Eigen::Vector2d &obstacle,
    TimedWalkingCorridorSegment::HalfPlane2D &hp)
{
    const double eps = 1e-6;
    const double obstacle_norm = obstacle.norm();
    if (obstacle_norm < eps)
        return false;

    hp.normal = obstacle / obstacle_norm;
    hp.offset = -obstacle_norm;

    auto repairAgainstSeedPoint = [&](const Eigen::Vector2d &seed)
    {
        if (hp.value(seed) <= 1e-5)
            return true;
        const Eigen::Vector2d delta = obstacle - seed;
        const double delta_sq = delta.squaredNorm();
        if (delta_sq < eps)
            return false;
        Eigen::Vector2d tangent_normal =
            seed - (delta.dot(seed) / delta_sq) * delta;
        const double tangent_norm = tangent_normal.norm();
        if (tangent_norm < eps)
            return false;
        hp.normal = tangent_normal / tangent_norm;
        hp.offset = -tangent_norm;
        return true;
    };

    if (!repairAgainstSeedPoint(seed_a))
        return false;
    if (!repairAgainstSeedPoint(seed_b))
        return false;

    if (hp.value(seed_a) <= 1e-5 && hp.value(seed_b) <= 1e-5)
        return true;

    const Eigen::Vector2d seed_delta = seed_b - seed_a;
    if (seed_delta.norm() < eps)
        return false;
    hp.normal = Eigen::Vector2d(-seed_delta.y(), seed_delta.x()).normalized();
    hp.offset = -hp.normal.dot(seed_a);
    if (hp.value(obstacle) < 0.0)
    {
        hp.normal = -hp.normal;
        hp.offset = -hp.offset;
    }
    return hp.value(seed_a) <= 1e-5 && hp.value(seed_b) <= 1e-5;
}

bool buildFiriPolygonObstacleTangent(
    const Eigen::Vector2d &seed_a,
    const Eigen::Vector2d &seed_b,
    const std::vector<Eigen::Vector2d> &obstacle,
    TimedWalkingCorridorSegment::HalfPlane2D &hp)
{
    if (obstacle.size() < 3)
        return false;

    Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
    for (const auto &v : obstacle)
        centroid += v;
    centroid /= static_cast<double>(obstacle.size());

    std::vector<Eigen::Vector2d> normals;
    normals.reserve(obstacle.size() + 1);
    const Eigen::Vector2d center_normal =
        centroid - closestPointOnSegment(seed_a, seed_b, centroid);
    if (center_normal.norm() > 1e-5)
        normals.push_back(center_normal);
    for (const auto &v : obstacle)
    {
        const Eigen::Vector2d n = v - closestPointOnSegment(seed_a, seed_b, v);
        if (n.norm() > 1e-5)
            normals.push_back(n);
    }

    bool found = false;
    double best_distance = std::numeric_limits<double>::infinity();
    TimedWalkingCorridorSegment::HalfPlane2D best;
    for (auto normal : normals)
    {
        const double normal_norm = normal.norm();
        if (normal_norm < 1e-9)
            continue;
        normal /= normal_norm;

        double obstacle_min = std::numeric_limits<double>::infinity();
        for (const auto &v : obstacle)
            obstacle_min = std::min(obstacle_min, normal.dot(v));

        TimedWalkingCorridorSegment::HalfPlane2D candidate;
        candidate.normal = normal;
        candidate.offset = -obstacle_min;
        if (candidate.value(seed_a) > 1e-5 || candidate.value(seed_b) > 1e-5)
            continue;

        bool excludes_obstacle = true;
        for (const auto &v : obstacle)
        {
            if (candidate.value(v) < -1e-5)
            {
                excludes_obstacle = false;
                break;
            }
        }
        if (!excludes_obstacle)
            continue;

        const double distance = halfPlaneDistanceToOrigin(candidate);
        if (!found || distance < best_distance)
        {
            found = true;
            best_distance = distance;
            best = candidate;
        }
    }

    if (!found)
        return false;

    hp = best;
    return true;
}

double halfPlaneDistanceToOrigin(
    const TimedWalkingCorridorSegment::HalfPlane2D &hp)
{
    return std::abs(hp.offset) / std::max(1e-6, hp.normal.norm());
}

Eigen::Vector2d polygonCentroid(const std::vector<Eigen::Vector2d> &polygon)
{
    if (polygon.empty())
        return Eigen::Vector2d::Zero();

    double signed_area = 0.0;
    Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
    for (size_t i = 0; i < polygon.size(); ++i)
    {
        const Eigen::Vector2d &p0 = polygon[i];
        const Eigen::Vector2d &p1 = polygon[(i + 1) % polygon.size()];
        const double cross = cross2d(p0, p1);
        signed_area += cross;
        centroid += (p0 + p1) * cross;
    }

    if (std::abs(signed_area) < 1e-9)
    {
        centroid.setZero();
        for (const auto &p : polygon)
            centroid += p;
        return centroid / static_cast<double>(polygon.size());
    }

    return centroid / (3.0 * signed_area);
}

double polygonArea(const std::vector<Eigen::Vector2d> &polygon)
{
    if (polygon.size() < 3)
        return 0.0;

    double area = 0.0;
    for (size_t i = 0; i < polygon.size(); ++i)
        area += cross2d(polygon[i], polygon[(i + 1) % polygon.size()]);
    return 0.5 * std::abs(area);
}

bool projectionSeparated(
    const std::vector<Eigen::Vector2d> &a,
    const std::vector<Eigen::Vector2d> &b,
    const Eigen::Vector2d &axis,
    const double clearance)
{
    const double axis_norm = axis.norm();
    if (axis_norm < 1e-9)
        return false;
    const Eigen::Vector2d unit_axis = axis / axis_norm;
    double a_min = std::numeric_limits<double>::infinity();
    double a_max = -std::numeric_limits<double>::infinity();
    double b_min = std::numeric_limits<double>::infinity();
    double b_max = -std::numeric_limits<double>::infinity();
    for (const auto &p : a)
    {
        const double v = unit_axis.dot(p);
        a_min = std::min(a_min, v);
        a_max = std::max(a_max, v);
    }
    for (const auto &p : b)
    {
        const double v = unit_axis.dot(p);
        b_min = std::min(b_min, v);
        b_max = std::max(b_max, v);
    }
    return a_max + clearance < b_min || b_max + clearance < a_min;
}

bool convexPolygonsOverlap(
    const std::vector<Eigen::Vector2d> &a,
    const std::vector<Eigen::Vector2d> &b,
    const double clearance = 0.0)
{
    if (a.size() < 3 || b.size() < 3)
        return false;

    auto hasSeparatingAxis = [&](const std::vector<Eigen::Vector2d> &poly)
    {
        for (size_t i = 0; i < poly.size(); ++i)
        {
            const Eigen::Vector2d edge = poly[(i + 1) % poly.size()] - poly[i];
            const Eigen::Vector2d axis(-edge.y(), edge.x());
            if (projectionSeparated(a, b, axis, clearance))
                return true;
        }
        return false;
    };

    return !hasSeparatingAxis(a) && !hasSeparatingAxis(b);
}

std::vector<Eigen::Vector2d> buildHumanCaneFootprintRelative(
    const Eigen::Vector2d &forward,
    const DynamicWalkingCorridor::Config &cfg)
{
    std::vector<Eigen::Vector2d> samples_world;
    if (!cfg.human_cane_footprint_enable)
        return samples_world;

    const Eigen::Vector2d f = normalizedOrDefault(forward);
    const Eigen::Vector2d l(-f.y(), f.x());
    const int samples = std::max(4, cfg.human_cane_footprint_samples);
    const double body_radius = std::max(
        0.0, cfg.human_cane_body_radius + cfg.human_cane_safety_margin);
    const double front_radius = std::max(
        0.0, cfg.human_cane_front_radius + cfg.human_cane_safety_margin);
    const double cane_length = std::max(0.0, cfg.human_cane_cane_length);

    auto appendDisk = [&](const Eigen::Vector2d &center, const double radius)
    {
        if (radius < 1e-6)
        {
            samples_world.push_back(center);
            return;
        }
        for (int i = 0; i < samples; ++i)
        {
            const double theta =
                2.0 * std::acos(-1.0) *
                static_cast<double>(i) / static_cast<double>(samples);
            samples_world.push_back(
                center + radius * (std::cos(theta) * f + std::sin(theta) * l));
        }
    };

    appendDisk(Eigen::Vector2d::Zero(), body_radius);
    appendDisk(cane_length * f, front_radius);
    if (samples_world.size() <= 3)
        return samples_world;

    std::sort(samples_world.begin(), samples_world.end(),
              [](const Eigen::Vector2d &a, const Eigen::Vector2d &b)
              {
                  if (std::abs(a.x() - b.x()) > 1e-9)
                      return a.x() < b.x();
                  return a.y() < b.y();
              });

    std::vector<Eigen::Vector2d> hull;
    hull.reserve(samples_world.size());
    for (const auto &p : samples_world)
    {
        while (hull.size() >= 2 &&
               cross2d(hull.back() - hull[hull.size() - 2],
                       p - hull.back()) <= 1e-9)
        {
            hull.pop_back();
        }
        hull.push_back(p);
    }
    const size_t lower_size = hull.size();
    for (int i = static_cast<int>(samples_world.size()) - 2; i >= 0; --i)
    {
        const auto &p = samples_world[static_cast<size_t>(i)];
        while (hull.size() > lower_size &&
               cross2d(hull.back() - hull[hull.size() - 2],
                       p - hull.back()) <= 1e-9)
        {
            hull.pop_back();
        }
        hull.push_back(p);
    }
    if (!hull.empty())
        hull.pop_back();
    return hull;
}

void erodeHalfPlanesForHumanCaneFootprint(
    std::vector<TimedWalkingCorridorSegment::HalfPlane2D> &half_planes,
    const Eigen::Vector2d &forward,
    const DynamicWalkingCorridor::Config &cfg)
{
    const auto footprint = buildHumanCaneFootprintRelative(forward, cfg);
    if (footprint.empty())
        return;

    for (auto &hp : half_planes)
    {
        double support = -std::numeric_limits<double>::infinity();
        for (const auto &v : footprint)
            support = std::max(support, hp.normal.dot(v));
        if (std::isfinite(support))
            hp.offset += support;
    }
}

double rayLimitInHalfPlanes(
    const std::vector<TimedWalkingCorridorSegment::HalfPlane2D> &half_planes,
    const Eigen::Vector2d &origin,
    const Eigen::Vector2d &direction)
{
    double limit = std::numeric_limits<double>::infinity();
    for (const auto &hp : half_planes)
    {
        const double value = hp.value(origin);
        const double denom = hp.normal.dot(direction);
        if (denom > 1e-9)
            limit = std::min(limit, -value / denom);
    }
    return std::isfinite(limit) ? std::max(1e-3, limit) : 1.0;
}

bool updateFiriFrameFromHalfPlanes(
    const std::vector<TimedWalkingCorridorSegment::HalfPlane2D> &half_planes,
    const double shrink,
    FiriFrame2D &frame)
{
    const auto polygon = polygonFromHalfPlanes(half_planes);
    if (polygon.size() < 3)
        return false;

    frame.center = polygonCentroid(polygon);

    Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
    for (const auto &p : polygon)
    {
        const Eigen::Vector2d d = p - frame.center;
        cov += d * d.transpose();
    }
    cov /= static_cast<double>(polygon.size());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(cov);
    if (solver.info() == Eigen::Success)
    {
        frame.axes.col(0) = solver.eigenvectors().col(1);
        frame.axes.col(1) = solver.eigenvectors().col(0);
        if (frame.axes.determinant() < 0.0)
            frame.axes.col(1) = -frame.axes.col(1);
    }

    const double safe_shrink = std::max(0.2, std::min(1.0, shrink));
    for (int i = 0; i < 2; ++i)
    {
        const Eigen::Vector2d axis = frame.axes.col(i);
        const double positive = rayLimitInHalfPlanes(half_planes, frame.center, axis);
        const double negative = rayLimitInHalfPlanes(half_planes, frame.center, -axis);
        frame.radii(i) = safe_shrink * std::min(positive, negative);
    }
    frame.radii.x() = std::max(1e-3, frame.radii.x());
    frame.radii.y() = std::max(1e-3, frame.radii.y());
    return true;
}

std::vector<TimedWalkingCorridorSegment::HalfPlane2D> buildFiriHalfPlanes2D(
    const Eigen::Vector2d &seed_a,
    const Eigen::Vector2d &seed_b,
    const std::vector<TimedWalkingCorridorSegment::HalfPlane2D> &boundary_half_planes,
    const std::vector<Eigen::Vector2d> &obstacle_points,
    const std::vector<std::vector<Eigen::Vector2d>> &obstacle_polygons,
    const DynamicWalkingCorridor::Config &cfg)
{
    if (boundary_half_planes.empty())
        return {};

    const Eigen::Vector2d seed_delta = seed_b - seed_a;
    FiriFrame2D frame;
    frame.center = 0.5 * (seed_a + seed_b);
    frame.axes.col(0) = normalizedOrDefault(seed_delta);
    frame.axes.col(1) = Eigen::Vector2d(-frame.axes.col(0).y(), frame.axes.col(0).x());
    frame.radii.x() = std::max(0.25, 0.5 * seed_delta.norm());
    frame.radii.y() = std::max(0.10, cfg.half_width);

    std::vector<TimedWalkingCorridorSegment::HalfPlane2D> h_poly = boundary_half_planes;
    const int iterations = std::max(1, cfg.firi_iterations);
    const double clearance = std::max(0.0, cfg.firi_clearance);

    for (int loop = 0; loop < iterations; ++loop)
    {
        std::vector<TimedWalkingCorridorSegment::HalfPlane2D> forward_boundary;
        forward_boundary.reserve(boundary_half_planes.size());
        for (const auto &hp : boundary_half_planes)
            forward_boundary.push_back(transformHalfPlaneToFiriFrame(frame, hp));

        std::vector<Eigen::Vector2d> forward_obstacles;
        forward_obstacles.reserve(obstacle_points.size() + obstacle_polygons.size());
        std::vector<TimedWalkingCorridorSegment::HalfPlane2D> tangents;
        std::vector<double> tangent_distances;
        const Eigen::Vector2d forward_seed_a = transformToFiriFrame(frame, seed_a);
        const Eigen::Vector2d forward_seed_b = transformToFiriFrame(frame, seed_b);
        for (const auto &obstacle : obstacle_points)
        {
            const Eigen::Vector2d forward_obstacle = transformToFiriFrame(frame, obstacle);
            TimedWalkingCorridorSegment::HalfPlane2D tangent;
            if (!buildFiriObstacleTangent(forward_seed_a, forward_seed_b,
                                          forward_obstacle, tangent))
                continue;
            forward_obstacles.push_back(forward_obstacle);
            tangents.push_back(tangent);
            tangent_distances.push_back(halfPlaneDistanceToOrigin(tangent));
        }
        for (const auto &obstacle_polygon : obstacle_polygons)
        {
            if (obstacle_polygon.size() < 3)
                continue;

            std::vector<Eigen::Vector2d> forward_polygon;
            forward_polygon.reserve(obstacle_polygon.size());
            Eigen::Vector2d forward_centroid = Eigen::Vector2d::Zero();
            for (const auto &v : obstacle_polygon)
            {
                const Eigen::Vector2d fv = transformToFiriFrame(frame, v);
                forward_polygon.push_back(fv);
                forward_centroid += fv;
            }
            forward_centroid /= static_cast<double>(forward_polygon.size());

            TimedWalkingCorridorSegment::HalfPlane2D tangent;
            if (!buildFiriPolygonObstacleTangent(forward_seed_a, forward_seed_b,
                                                 forward_polygon, tangent))
                continue;
            forward_obstacles.push_back(forward_centroid);
            tangents.push_back(tangent);
            tangent_distances.push_back(halfPlaneDistanceToOrigin(tangent));
        }

        std::vector<char> boundary_active(forward_boundary.size(), 1);
        std::vector<char> obstacle_active(tangents.size(), 1);
        std::vector<TimedWalkingCorridorSegment::HalfPlane2D> forward_selected;
        std::vector<char> forward_selected_from_obstacle;
        forward_selected.reserve(forward_boundary.size() + tangents.size());
        forward_selected_from_obstacle.reserve(forward_boundary.size() + tangents.size());

        for (size_t k = 0; k < forward_boundary.size() + tangents.size(); ++k)
        {
            int best_boundary = -1;
            int best_obstacle = -1;
            double best_boundary_dist = std::numeric_limits<double>::infinity();
            double best_obstacle_dist = std::numeric_limits<double>::infinity();

            for (size_t i = 0; i < forward_boundary.size(); ++i)
            {
                if (!boundary_active[i])
                    continue;
                const double dist = halfPlaneDistanceToOrigin(forward_boundary[i]);
                if (dist < best_boundary_dist)
                {
                    best_boundary_dist = dist;
                    best_boundary = static_cast<int>(i);
                }
            }

            for (size_t i = 0; i < tangents.size(); ++i)
            {
                if (!obstacle_active[i])
                    continue;
                if (tangent_distances[i] < best_obstacle_dist)
                {
                    best_obstacle_dist = tangent_distances[i];
                    best_obstacle = static_cast<int>(i);
                }
            }

            if (best_boundary < 0 && best_obstacle < 0)
                break;

            TimedWalkingCorridorSegment::HalfPlane2D selected;
            if (best_boundary >= 0 && best_boundary_dist < best_obstacle_dist)
            {
                selected = forward_boundary[best_boundary];
                boundary_active[best_boundary] = 0;
                forward_selected_from_obstacle.push_back(0);
            }
            else
            {
                selected = tangents[best_obstacle];
                obstacle_active[best_obstacle] = 0;
                forward_selected_from_obstacle.push_back(1);
            }
            forward_selected.push_back(selected);

            for (size_t i = 0; i < tangents.size(); ++i)
            {
                if (obstacle_active[i] &&
                    selected.value(forward_obstacles[i]) > -1e-6)
                {
                    obstacle_active[i] = 0;
                }
            }
        }

        h_poly.clear();
        h_poly.reserve(forward_selected.size());
        for (size_t i = 0; i < forward_selected.size(); ++i)
        {
            auto world_hp = transformHalfPlaneFromFiriFrame(frame, forward_selected[i]);
            const double norm = world_hp.normal.norm();
            if (forward_selected_from_obstacle[i] && norm > 1e-9)
                world_hp.offset += clearance * norm;
            if (world_hp.value(seed_a) <= 1e-5 && world_hp.value(seed_b) <= 1e-5)
                h_poly.push_back(world_hp);
        }

        if (h_poly.size() < 3 || polygonFromHalfPlanes(h_poly).empty())
        {
            h_poly = boundary_half_planes;
            for (const auto &obstacle : obstacle_points)
                addFiriLikeSupportHalfPlane(seed_a, seed_b, obstacle,
                                            clearance, h_poly);
            break;
        }

        if (loop + 1 < iterations &&
            !updateFiriFrameFromHalfPlanes(h_poly, cfg.firi_ellipse_shrink, frame))
        {
            break;
        }
    }

    return h_poly;
}

void buildConvexCellForSegment(
    TimedWalkingCorridorSegment &segment,
    const CollisionDetection::Ptr &collision,
    const DynamicWalkingCorridor::Config &cfg)
{
    const Eigen::Vector2d a = segment.start;
    const Eigen::Vector2d b = segment.end;
    if ((b - a).norm() < 1e-6)
        return;

    const double range = std::max(cfg.half_width,
                                  std::max(0.3, cfg.static_opt_lateral_range));
    const double min_x = std::min(a.x(), b.x()) - range;
    const double max_x = std::max(a.x(), b.x()) + range;
    const double min_y = std::min(a.y(), b.y()) - range;
    const double max_y = std::max(a.y(), b.y()) + range;

    std::vector<TimedWalkingCorridorSegment::HalfPlane2D> boundary_half_planes;
    auto addHp = [&](double nx, double ny, double offset)
    {
        TimedWalkingCorridorSegment::HalfPlane2D hp;
        hp.normal = Eigen::Vector2d(nx, ny);
        hp.offset = offset;
        boundary_half_planes.push_back(hp);
    };
    addHp(1.0, 0.0, -max_x);
    addHp(-1.0, 0.0, min_x);
    addHp(0.0, 1.0, -max_y);
    addHp(0.0, -1.0, min_y);

    std::vector<Eigen::Vector2d> obstacle_points;
    if (collision)
    {
        const double ds = std::max(0.08, std::min(0.25, cfg.static_sample_ds));
        for (double x = min_x; x <= max_x + 1e-6; x += ds)
        {
            for (double y = min_y; y <= max_y + 1e-6; y += ds)
            {
                if (!collision->isTraversable(x, y))
                    obstacle_points.emplace_back(x, y);
            }
        }
    }

    std::vector<std::vector<Eigen::Vector2d>> obstacle_polygons;
    obstacle_polygons.reserve(segment.dynamic_obstacles.size());
    for (const auto &obstacle : segment.dynamic_obstacles)
    {
        if (obstacle.vertices.size() >= 3)
            obstacle_polygons.push_back(obstacle.vertices);
    }

    if (cfg.firi_enable)
    {
        segment.half_planes = buildFiriHalfPlanes2D(
            a, b, boundary_half_planes, obstacle_points, obstacle_polygons, cfg);
    }
    else
    {
        segment.half_planes = boundary_half_planes;
        for (const auto &obstacle : obstacle_points)
            addFiriLikeSupportHalfPlane(a, b, obstacle,
                                        cfg.firi_clearance, segment.half_planes);
    }

    for (const auto &obstacle : segment.dynamic_obstacles)
    {
        if (addDynamicObstacleSupportHalfPlane(a, b, obstacle,
                                               cfg.firi_clearance, segment.half_planes) == 0)
        {
            segment.feasible = false;
            segment.blocked_dynamic = true;
        }
    }

    erodeHalfPlanesForHumanCaneFootprint(
        segment.half_planes, segment.forward, cfg);

    segment.polygon = polygonFromHalfPlanes(segment.half_planes);
    if (segment.polygon.empty())
    {
        segment.feasible = false;
        segment.blocked_dynamic = true;
    }

    if (!segment.polygon.empty() &&
        (!insideHalfPlanes(segment.half_planes, segment.start, 1e-5) ||
         !insideHalfPlanes(segment.half_planes, segment.end, 1e-5)))
    {
        segment.feasible = false;
        if (!segment.dynamic_obstacles.empty())
            segment.blocked_dynamic = true;
        else
            segment.blocked_static = true;
    }

    if (!segment.polygon.empty())
    {
        for (const auto &obstacle : segment.dynamic_obstacles)
        {
            if (obstacle.vertices.size() >= 3 &&
                convexPolygonsOverlap(segment.polygon, obstacle.vertices,
                                      std::max(0.0, cfg.firi_clearance)))
            {
                segment.feasible = false;
                segment.blocked_dynamic = true;
                break;
            }
        }
    }
}

TimedWalkingCorridorSegment buildTimedCorridorSegment(
    const DynamicWalkingCorridor::Candidate &candidate,
    const double start_s,
    const double end_s,
    const std::vector<Eigen::Vector3d> &obs_pos,
    const std::vector<Eigen::Vector3d> &obs_vel,
    const std::vector<Eigen::Vector3d> &obs_size,
    const DynamicWalkingCorridor::Config &cfg,
    const CollisionDetection::Ptr &collision)
{
    TimedWalkingCorridorSegment segment;
    segment.t_start = interpolateTimeAtPolylineDistance(
        candidate.centerline, candidate.centerline_times, start_s);
    segment.t_end = interpolateTimeAtPolylineDistance(
        candidate.centerline, candidate.centerline_times, end_s);
    segment.start = interpolatePolyline(candidate.centerline, start_s);
    segment.end = interpolatePolyline(candidate.centerline, end_s);
    const Eigen::Vector2d delta = segment.end - segment.start;
    segment.forward = normalizedOrDefault(delta);
    segment.left = Eigen::Vector2d(-segment.forward.y(), segment.forward.x());
    segment.centerline = {segment.start, segment.end};
    segment.half_width = candidate.half_width;
    segment.feasible = candidate.feasible;
    segment.blocked_static = candidate.blocked_static;
    segment.blocked_dynamic = candidate.blocked_dynamic;
    const double tm = 0.5 * (segment.t_start + segment.t_end);
    const double segment_dt = std::max(0.0, segment.t_end - segment.t_start);
    segment.dynamic_obstacles.reserve(obs_pos.size());
    for (size_t j = 0; j < obs_pos.size(); ++j)
    {
        Eigen::Vector2d center(obs_pos[j](0), obs_pos[j](1));
        Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
        if (j < obs_vel.size())
        {
            velocity = Eigen::Vector2d(obs_vel[j](0), obs_vel[j](1));
            center += velocity * tm;
        }

        TimedWalkingCorridorSegment::DynamicEllipseObstacle obstacle;
        obstacle.center = center;
        const double speed = velocity.norm();
        obstacle.forward = speed > 1e-3 ? velocity / speed : segment.forward;
        obstacle.left = Eigen::Vector2d(-obstacle.forward.y(), obstacle.forward.x());
        const double base_radius =
            obstacleRadius(j, obs_size, cfg.dynamic_min_radius) +
            cfg.robot_radius + cfg.dynamic_safety_margin;
        const double prediction_extension =
            speed * (0.5 * segment_dt + std::max(0.0, cfg.prediction_dt));
        obstacle.lateral_radius = base_radius;
        obstacle.longitudinal_radius = base_radius + prediction_extension;
        obstacle.vertices = buildDynamicObstacleFootprint(
            center, velocity, segment.forward, base_radius, prediction_extension);
        segment.dynamic_obstacles.push_back(obstacle);
    }
    buildConvexCellForSegment(segment, collision, cfg);
    return segment;
}

void appendSegmentWithDynamicSplitting(
    TimedWalkingCorridor &corridor,
    const DynamicWalkingCorridor::Candidate &candidate,
    const double start_s,
    const double end_s,
    const std::vector<Eigen::Vector3d> &obs_pos,
    const std::vector<Eigen::Vector3d> &obs_vel,
    const std::vector<Eigen::Vector3d> &obs_size,
    const DynamicWalkingCorridor::Config &cfg,
    const CollisionDetection::Ptr &collision,
    const int depth)
{
    auto segment = buildTimedCorridorSegment(
        candidate, start_s, end_s, obs_pos, obs_vel, obs_size, cfg, collision);

    const double length = std::max(0.0, end_s - start_s);
    const int max_depth = std::max(0, cfg.dynamic_split_max_depth);
    const double min_length = std::max(0.05, cfg.dynamic_split_min_length);
    if (segment.blocked_dynamic && !segment.blocked_static &&
        depth < max_depth && length > 2.0 * min_length)
    {
        const double mid_s = 0.5 * (start_s + end_s);
        appendSegmentWithDynamicSplitting(
            corridor, candidate, start_s, mid_s,
            obs_pos, obs_vel, obs_size, cfg, collision, depth + 1);
        appendSegmentWithDynamicSplitting(
            corridor, candidate, mid_s, end_s,
            obs_pos, obs_vel, obs_size, cfg, collision, depth + 1);
        return;
    }

    corridor.segments.push_back(segment);
}

TimedWalkingCorridor toTimedWalkingCorridor(
    const DynamicWalkingCorridor::Candidate &candidate,
    const TimedTrajectorySource source,
    const std::vector<Eigen::Vector3d> &obs_pos,
    const std::vector<Eigen::Vector3d> &obs_vel,
    const std::vector<Eigen::Vector3d> &obs_size,
    const DynamicWalkingCorridor::Config &cfg,
    const CollisionDetection::Ptr &collision)
{
    TimedWalkingCorridor corridor;
    corridor.source = source;
    if (!hasTimedPolyline(candidate))
        return corridor;

    corridor.segments.reserve(candidate.centerline.size() - 1);
    const double total_length = polylineLength(candidate.centerline);
    if (total_length < 1e-6)
        return corridor;

    const double cover_length = std::max(cfg.static_min_feasible_length,
                                         0.5 * std::max(cfg.length, cfg.static_min_feasible_length));
    const double cover_step = 0.5 * cover_length;

    for (double start_s = 0.0; start_s < total_length - 1e-6; start_s += cover_step)
    {
        double end_s = std::min(total_length, start_s + cover_length);
        if (total_length - end_s > 1e-6 &&
            total_length - end_s < cfg.static_min_feasible_length)
        {
            end_s = total_length;
        }

        appendSegmentWithDynamicSplitting(
            corridor, candidate, start_s, end_s,
            obs_pos, obs_vel, obs_size, cfg, collision, 0);
    }
    return corridor;
}

double polylineDistanceAtTime(const std::vector<Eigen::Vector2d> &path,
                              const std::vector<double> &times,
                              double t)
{
    if (path.size() < 2 || times.size() != path.size())
        return 0.0;
    if (t <= times.front())
        return 0.0;

    double accum = 0.0;
    for (size_t i = 0; i + 1 < path.size(); ++i)
    {
        const Eigen::Vector2d seg = path[i + 1] - path[i];
        const double seg_len = seg.norm();
        const double t0 = times[i];
        const double t1 = times[i + 1];
        if (t <= t1)
        {
            const double denom = std::max(1e-6, t1 - t0);
            const double u = std::max(0.0, std::min(1.0, (t - t0) / denom));
            return accum + u * seg_len;
        }
        accum += seg_len;
    }
    return accum;
}

bool projectPointToPolyline(const std::vector<Eigen::Vector2d> &path,
                            const Eigen::Vector2d &point,
                            double &along,
                            double &distance)
{
    if (path.size() < 2)
        return false;

    double best_dist = std::numeric_limits<double>::infinity();
    double best_along = 0.0;
    double accum = 0.0;
    for (size_t i = 0; i + 1 < path.size(); ++i)
    {
        const Eigen::Vector2d a = path[i];
        const Eigen::Vector2d b = path[i + 1];
        const Eigen::Vector2d ab = b - a;
        const double len_sq = ab.squaredNorm();
        const double seg_len = std::sqrt(len_sq);
        if (seg_len < 1e-6)
            continue;
        double u = (point - a).dot(ab) / len_sq;
        u = std::max(0.0, std::min(1.0, u));
        const Eigen::Vector2d proj = a + u * ab;
        const double d = (point - proj).norm();
        if (d < best_dist)
        {
            best_dist = d;
            best_along = accum + u * seg_len;
        }
        accum += seg_len;
    }

    if (!std::isfinite(best_dist))
        return false;
    along = best_along;
    distance = best_dist;
    return true;
}

std::vector<Eigen::Vector2d> shiftPolyline(const std::vector<Eigen::Vector2d> &path,
                                           double lateral_offset)
{
    std::vector<Eigen::Vector2d> shifted;
    shifted.reserve(path.size());
    if (path.empty())
        return shifted;

    for (size_t i = 0; i < path.size(); ++i)
    {
        Eigen::Vector2d tangent = Eigen::Vector2d::Zero();
        if (i > 0)
            tangent += normalizedOrDefault(path[i] - path[i - 1]);
        if (i + 1 < path.size())
            tangent += normalizedOrDefault(path[i + 1] - path[i]);
        tangent = normalizedOrDefault(tangent);
        Eigen::Vector2d left(-tangent.y(), tangent.x());
        shifted.push_back(path[i] + lateral_offset * left);
    }
    return shifted;
}

Eigen::Vector2d pathLeftAt(const std::vector<Eigen::Vector2d> &path, size_t idx)
{
    Eigen::Vector2d tangent = Eigen::Vector2d::Zero();
    if (idx > 0)
        tangent += normalizedOrDefault(path[idx] - path[idx - 1]);
    if (idx + 1 < path.size())
        tangent += normalizedOrDefault(path[idx + 1] - path[idx]);
    tangent = normalizedOrDefault(tangent);
    return Eigen::Vector2d(-tangent.y(), tangent.x());
}

double availableHalfWidthAt(
    const Eigen::Vector2d &center,
    const Eigen::Vector2d &left,
    const DynamicWalkingCorridor::Config &cfg,
    const std::function<bool(double, double)> &is_traversable)
{
    if (!is_traversable(center.x(), center.y()))
        return 0.0;

    const double dl = std::max(
        0.02,
        std::min(cfg.static_sample_dl, 0.5 * std::max(0.02, cfg.min_half_width)));
    const double max_half_width = std::max(0.0, cfg.half_width);

    double left_free = 0.0;
    for (double l = dl; l <= max_half_width + 1e-6; l += dl)
    {
        const Eigen::Vector2d p = center + left * l;
        if (!is_traversable(p.x(), p.y()))
            break;
        left_free = l;
    }

    double right_free = 0.0;
    for (double l = dl; l <= max_half_width + 1e-6; l += dl)
    {
        const Eigen::Vector2d p = center - left * l;
        if (!is_traversable(p.x(), p.y()))
            break;
        right_free = l;
    }

    return std::min(left_free, right_free);
}
} // namespace

void DynamicWalkingCorridor::setParam(ros::NodeHandle &nh)
{
    nh.param("dwc/enable", cfg_.enable, true);
    nh.param("dwc/lateral_candidates_enable", cfg_.lateral_candidates_enable, false);
    nh.param("dwc/length", cfg_.length, 4.0);
    nh.param("dwc/half_width", cfg_.half_width, 0.45);
    nh.param("dwc/min_half_width", cfg_.min_half_width, 0.25);
    nh.param("dwc/lateral_shift", cfg_.lateral_shift, 0.8);
    nh.param("dwc/prediction_horizon", cfg_.prediction_horizon, 3.0);
    nh.param("dwc/prediction_dt", cfg_.prediction_dt, 0.25);
    nh.param("dwc/robot_speed", cfg_.robot_speed, 1.0);
    nh.param("dwc/robot_radius", cfg_.robot_radius, 0.25);
    nh.param("dwc/dynamic_min_radius", cfg_.dynamic_min_radius, 0.20);
    nh.param("dwc/dynamic_safety_margin", cfg_.dynamic_safety_margin, 0.15);
    nh.param("dwc/dynamic_progress_margin", cfg_.dynamic_progress_margin, 0.50);
    nh.param("dwc/dynamic_blocking_enable", cfg_.dynamic_blocking_enable, false);
    nh.param("dwc/static_centerline_opt_enable", cfg_.static_centerline_opt_enable, true);
    nh.param("dwc/static_truncate_enable", cfg_.static_truncate_enable, true);
    nh.param("dwc/static_opt_lateral_range", cfg_.static_opt_lateral_range, 1.0);
    nh.param("dwc/static_opt_lateral_step", cfg_.static_opt_lateral_step, 0.1);
    nh.param("dwc/static_opt_smooth_weight", cfg_.static_opt_smooth_weight, 2.0);
    nh.param("dwc/static_min_feasible_length", cfg_.static_min_feasible_length, 0.8);
    nh.param("dwc/static_truncate_backoff", cfg_.static_truncate_backoff, 0.2);
    nh.param("dwc/static_start_grace_length", cfg_.static_start_grace_length, 0.4);
    nh.param("dwc/lateral_offset_weight", cfg_.lateral_offset_weight, 0.2);
    nh.param("dwc/front_pass_weight", cfg_.front_pass_weight, 4.0);
    nh.param("dwc/front_pass_sigma_s", cfg_.front_pass_sigma_s, 1.2);
    nh.param("dwc/front_pass_sigma_l", cfg_.front_pass_sigma_l, 1.0);
    nh.param("dwc/front_pass_length", cfg_.front_pass_length, 3.0);
    nh.param("dwc/front_pass_lateral", cfg_.front_pass_lateral, 1.8);
    nh.param("dwc/static_sample_ds", cfg_.static_sample_ds, 0.4);
    nh.param("dwc/static_sample_dl", cfg_.static_sample_dl, 0.3);
    nh.param("dwc/firi_enable", cfg_.firi_enable, true);
    nh.param("dwc/firi_iterations", cfg_.firi_iterations, 4);
    nh.param("dwc/firi_clearance", cfg_.firi_clearance, 0.02);
    nh.param("dwc/firi_ellipse_shrink", cfg_.firi_ellipse_shrink, 0.85);
    nh.param("dwc/dynamic_split_max_depth", cfg_.dynamic_split_max_depth, 2);
    nh.param("dwc/dynamic_split_min_length", cfg_.dynamic_split_min_length, 0.5);
    nh.param("dwc/human_cane_footprint_enable", cfg_.human_cane_footprint_enable, true);
    nh.param("dwc/human_cane_body_radius", cfg_.human_cane_body_radius, 0.25);
    nh.param("dwc/human_cane_cane_length", cfg_.human_cane_cane_length, 0.65);
    nh.param("dwc/human_cane_front_radius", cfg_.human_cane_front_radius, 0.12);
    nh.param("dwc/human_cane_safety_margin", cfg_.human_cane_safety_margin, 0.03);
    nh.param("dwc/human_cane_footprint_samples", cfg_.human_cane_footprint_samples, 8);
}

DynamicWalkingCorridor::Result DynamicWalkingCorridor::plan(
    const Eigen::Vector2d &robot_pos,
    const Eigen::Vector2d &path_forward,
    const std::vector<Eigen::Vector3d> &obs_pos,
    const std::vector<Eigen::Vector3d> &obs_vel,
    const std::vector<Eigen::Vector3d> &obs_size) const
{
    Result result;
    Eigen::Vector2d forward = normalizedOrDefault(path_forward);
    Eigen::Vector2d left(-forward.y(), forward.x());
    std::vector<double> offsets = {0.0};
    if (cfg_.lateral_candidates_enable)
    {
        offsets.push_back(cfg_.lateral_shift);
        offsets.push_back(-cfg_.lateral_shift);
    }

    result.candidates.reserve(offsets.size());
    for (size_t i = 0; i < offsets.size(); ++i)
    {
        result.candidates.push_back(evaluateCandidate(
            (int)i, offsets[i], robot_pos, forward, left,
            obs_pos, obs_vel, obs_size));
    }

    double best_cost = std::numeric_limits<double>::infinity();
    for (const auto &candidate : result.candidates)
    {
        if (!candidate.feasible)
            continue;
        if (candidate.total_cost < best_cost)
        {
            best_cost = candidate.total_cost;
            result.selected = candidate;
            result.has_feasible = true;
        }
    }
    return result;
}

DynamicWalkingCorridor::Result DynamicWalkingCorridor::plan(
    const std::vector<Eigen::Vector2d> &reference_path,
    const std::vector<Eigen::Vector3d> &obs_pos,
    const std::vector<Eigen::Vector3d> &obs_vel,
    const std::vector<Eigen::Vector3d> &obs_size) const
{
    if (reference_path.size() < 2)
    {
        const Eigen::Vector2d robot_pos =
            reference_path.empty() ? Eigen::Vector2d::Zero() : reference_path.front();
        return plan(robot_pos, Eigen::Vector2d::UnitX(), obs_pos, obs_vel, obs_size);
    }

    Result result;
    std::vector<double> offsets = {0.0};
    if (cfg_.lateral_candidates_enable)
    {
        offsets.push_back(cfg_.lateral_shift);
        offsets.push_back(-cfg_.lateral_shift);
    }

    result.candidates.reserve(offsets.size());
    for (size_t i = 0; i < offsets.size(); ++i)
    {
        result.candidates.push_back(evaluateCandidate(
            (int)i, offsets[i], reference_path, obs_pos, obs_vel, obs_size));
    }

    double best_cost = std::numeric_limits<double>::infinity();
    for (const auto &candidate : result.candidates)
    {
        if (!candidate.feasible)
            continue;
        if (candidate.total_cost < best_cost)
        {
            best_cost = candidate.total_cost;
            result.selected = candidate;
            result.has_feasible = true;
        }
    }
    return result;
}

DynamicWalkingCorridor::Result DynamicWalkingCorridor::plan(
    const TimedTrajectory &nominal_trajectory,
    const std::vector<Eigen::Vector3d> &obs_pos,
    const std::vector<Eigen::Vector3d> &obs_vel,
    const std::vector<Eigen::Vector3d> &obs_size) const
{
    if (!nominal_trajectory.valid())
        return plan(std::vector<Eigen::Vector2d>(), obs_pos, obs_vel, obs_size);

    Result result;
    std::vector<double> offsets = {0.0};
    if (cfg_.lateral_candidates_enable)
    {
        offsets.push_back(cfg_.lateral_shift);
        offsets.push_back(-cfg_.lateral_shift);
    }

    result.candidates.reserve(offsets.size());
    for (size_t i = 0; i < offsets.size(); ++i)
    {
        result.candidates.push_back(evaluateCandidate(
            (int)i, offsets[i], nominal_trajectory, obs_pos, obs_vel, obs_size));
    }

    double best_cost = std::numeric_limits<double>::infinity();
    for (const auto &candidate : result.candidates)
    {
        if (!candidate.feasible)
            continue;
        if (candidate.total_cost < best_cost)
        {
            best_cost = candidate.total_cost;
            result.selected = candidate;
            result.has_feasible = true;
            result.timed_corridor = toTimedWalkingCorridor(
                candidate, nominal_trajectory.source,
                obs_pos, obs_vel, obs_size, cfg_, collision_);
        }
    }
    return result;
}

DynamicWalkingCorridor::Candidate DynamicWalkingCorridor::evaluateCandidate(
    int id,
    double lateral_offset,
    const Eigen::Vector2d &robot_pos,
    const Eigen::Vector2d &forward,
    const Eigen::Vector2d &left,
    const std::vector<Eigen::Vector3d> &obs_pos,
    const std::vector<Eigen::Vector3d> &obs_vel,
    const std::vector<Eigen::Vector3d> &obs_size) const
{
    Candidate candidate;
    candidate.id = id;
    candidate.lateral_offset = lateral_offset;
    candidate.forward = forward;
    candidate.left = left;
    candidate.length = cfg_.length;
    candidate.half_width = cfg_.half_width;
    candidate.start = robot_pos + left * lateral_offset;
    candidate.end = candidate.start + forward * cfg_.length;
    if (cfg_.static_centerline_opt_enable && collision_)
    {
        std::vector<Eigen::Vector2d> reference = {candidate.start, candidate.end};
        double optimized_half_width = candidate.half_width;
        if (optimizeCenterlineForStaticMap(
                reference, cfg_,
                [this](double x, double y) { return collision_->isTraversable(x, y); },
                candidate.centerline, optimized_half_width))
        {
            candidate.half_width = std::min(candidate.half_width, optimized_half_width);
            candidate.start = candidate.centerline.front();
            candidate.end = candidate.centerline.back();
            candidate.forward = polylineDirectionAtStart(candidate.centerline);
            candidate.left = Eigen::Vector2d(-candidate.forward.y(), candidate.forward.x());
        }
    }

    candidate.blocked_static = shrinkStaticWidth(candidate);
    candidate.blocked_dynamic = isDynamicallyBlocked(
        candidate, obs_pos, obs_vel, obs_size, candidate.dynamic_block_count);
    candidate.front_pass_cost = computeFrontPassCost(candidate, obs_pos, obs_vel);
    candidate.total_cost =
        cfg_.lateral_offset_weight * std::abs(lateral_offset) +
        candidate.front_pass_cost;
    candidate.feasible =
        !candidate.blocked_static &&
        (cfg_.dynamic_blocking_enable ? !candidate.blocked_dynamic : true);
    if (!candidate.feasible)
        candidate.total_cost = std::numeric_limits<double>::infinity();
    return candidate;
}

DynamicWalkingCorridor::Candidate DynamicWalkingCorridor::evaluateCandidate(
    int id,
    double lateral_offset,
    const std::vector<Eigen::Vector2d> &reference_path,
    const std::vector<Eigen::Vector3d> &obs_pos,
    const std::vector<Eigen::Vector3d> &obs_vel,
    const std::vector<Eigen::Vector3d> &obs_size) const
{
    Candidate candidate;
    candidate.id = id;
    candidate.lateral_offset = lateral_offset;
    candidate.centerline = shiftPolyline(reference_path, lateral_offset);
    candidate.length = polylineLength(candidate.centerline);
    candidate.half_width = cfg_.half_width;
    if (cfg_.static_centerline_opt_enable && collision_)
    {
        double optimized_half_width = candidate.half_width;
        std::vector<Eigen::Vector2d> optimized;
        if (optimizeCenterlineForStaticMap(
                candidate.centerline, cfg_,
                [this](double x, double y) { return collision_->isTraversable(x, y); },
                optimized, optimized_half_width))
        {
            candidate.centerline = optimized;
            candidate.half_width = std::min(candidate.half_width, optimized_half_width);
        }
    }
    candidate.start = candidate.centerline.empty() ? Eigen::Vector2d::Zero() : candidate.centerline.front();
    candidate.end = candidate.centerline.empty() ? candidate.start : candidate.centerline.back();
    candidate.forward = polylineDirectionAtStart(candidate.centerline);
    candidate.left = Eigen::Vector2d(-candidate.forward.y(), candidate.forward.x());

    candidate.blocked_static = shrinkStaticWidth(candidate);
    candidate.blocked_dynamic = isDynamicallyBlocked(
        candidate, obs_pos, obs_vel, obs_size, candidate.dynamic_block_count);
    candidate.front_pass_cost = computeFrontPassCost(candidate, obs_pos, obs_vel);
    candidate.total_cost =
        cfg_.lateral_offset_weight * std::abs(lateral_offset) +
        candidate.front_pass_cost;
    candidate.feasible =
        !candidate.blocked_static &&
        (cfg_.dynamic_blocking_enable ? !candidate.blocked_dynamic : true);
    if (!candidate.feasible)
        candidate.total_cost = std::numeric_limits<double>::infinity();
    return candidate;
}

DynamicWalkingCorridor::Candidate DynamicWalkingCorridor::evaluateCandidate(
    int id,
    double lateral_offset,
    const TimedTrajectory &nominal_trajectory,
    const std::vector<Eigen::Vector3d> &obs_pos,
    const std::vector<Eigen::Vector3d> &obs_vel,
    const std::vector<Eigen::Vector3d> &obs_size) const
{
    std::vector<Eigen::Vector2d> reference_path;
    reference_path.reserve(nominal_trajectory.points.size());
    std::vector<double> reference_times;
    reference_times.reserve(nominal_trajectory.points.size());
    for (const auto &point : nominal_trajectory.points)
    {
        reference_path.push_back(point.position);
        reference_times.push_back(point.t_from_now);
    }

    Candidate candidate;
    candidate.id = id;
    candidate.lateral_offset = lateral_offset;
    candidate.centerline = shiftPolyline(reference_path, lateral_offset);
    candidate.centerline_times = reference_times;
    candidate.length = polylineLength(candidate.centerline);
    candidate.half_width = cfg_.half_width;
    if (cfg_.static_centerline_opt_enable && collision_)
    {
        double optimized_half_width = candidate.half_width;
        std::vector<Eigen::Vector2d> optimized;
        if (optimizeCenterlineForStaticMap(
                candidate.centerline, cfg_,
                [this](double x, double y) { return collision_->isTraversable(x, y); },
                optimized, optimized_half_width))
        {
            candidate.centerline = optimized;
            candidate.half_width = std::min(candidate.half_width, optimized_half_width);
            if (candidate.centerline_times.size() != candidate.centerline.size())
                candidate.centerline_times.clear();
        }
    }
    candidate.start = candidate.centerline.empty() ? Eigen::Vector2d::Zero() : candidate.centerline.front();
    candidate.end = candidate.centerline.empty() ? candidate.start : candidate.centerline.back();
    candidate.forward = polylineDirectionAtStart(candidate.centerline);
    candidate.left = Eigen::Vector2d(-candidate.forward.y(), candidate.forward.x());

    candidate.blocked_static = shrinkStaticWidth(candidate);
    candidate.blocked_dynamic = isDynamicallyBlocked(
        candidate, obs_pos, obs_vel, obs_size, candidate.dynamic_block_count);
    candidate.front_pass_cost = computeFrontPassCost(candidate, obs_pos, obs_vel);
    candidate.total_cost =
        cfg_.lateral_offset_weight * std::abs(lateral_offset) +
        candidate.front_pass_cost;
    candidate.feasible =
        !candidate.blocked_static &&
        (cfg_.dynamic_blocking_enable ? !candidate.blocked_dynamic : true);
    if (!candidate.feasible)
        candidate.total_cost = std::numeric_limits<double>::infinity();
    return candidate;
}

bool DynamicWalkingCorridor::shrinkStaticWidth(Candidate &candidate) const
{
    if (!collision_)
        return false;

    const double ds = std::max(0.05, cfg_.static_sample_ds);
    const double dl = std::max(
        0.02,
        std::min(cfg_.static_sample_dl, 0.5 * std::max(0.02, cfg_.min_half_width)));
    const double max_half_width = std::max(0.0, candidate.half_width);
    const double start_grace = std::max(0.0, cfg_.static_start_grace_length);
    double min_available_half_width = max_half_width;
    double prefix_min_available_half_width = max_half_width;
    candidate.static_min_width_valid = false;
    candidate.static_min_half_width = max_half_width;
    candidate.static_min_s = 0.0;
    candidate.static_min_point = candidate.start;
    candidate.truncated_static = false;

    auto updateAvailableAt = [&](const Eigen::Vector2d &center,
                                 const Eigen::Vector2d &left,
                                 double path_s) -> double {
        double available_width = 0.0;
        if (!collision_->isTraversable(center.x(), center.y()))
        {
            available_width = 0.0;
        }
        else
        {
            double left_free = 0.0;
            for (double l = dl; l <= max_half_width + 1e-6; l += dl)
            {
                Eigen::Vector2d p = center + left * l;
                if (!collision_->isTraversable(p.x(), p.y()))
                    break;
                left_free = l;
            }

            double right_free = 0.0;
            for (double l = dl; l <= max_half_width + 1e-6; l += dl)
            {
                Eigen::Vector2d p = center - left * l;
                if (!collision_->isTraversable(p.x(), p.y()))
                    break;
                right_free = l;
            }
            available_width = std::min(left_free, right_free);
        }

        if (!candidate.static_min_width_valid ||
            available_width < candidate.static_min_half_width)
        {
            candidate.static_min_width_valid = true;
            candidate.static_min_half_width = available_width;
            candidate.static_min_s = path_s;
            candidate.static_min_point = center;
        }
        return available_width;
    };

    auto maybeTruncateBeforeBlock = [&](double path_s) -> bool {
        if (!cfg_.static_truncate_enable)
            return false;
        if (path_s <= std::max(0.0, cfg_.static_min_feasible_length))
            return false;
        if (candidate.centerline.size() < 2)
            return false;

        const double cutoff = std::max(
            std::max(0.0, cfg_.static_min_feasible_length),
            path_s - std::max(0.0, cfg_.static_truncate_backoff));
        std::vector<Eigen::Vector2d> truncated =
            truncatePolylineAtLength(candidate.centerline, cutoff);
        if (truncated.size() < 2 || polylineLength(truncated) < cfg_.static_min_feasible_length)
            return false;

        candidate.centerline = truncated;
        candidate.start = candidate.centerline.front();
        candidate.end = candidate.centerline.back();
        candidate.length = polylineLength(candidate.centerline);
        candidate.forward = polylineDirectionAtStart(candidate.centerline);
        candidate.left = Eigen::Vector2d(-candidate.forward.y(), candidate.forward.x());
        candidate.half_width = std::min(candidate.half_width, prefix_min_available_half_width);
        candidate.truncated_static = true;
        return true;
    };

    if (candidate.centerline.size() >= 2)
    {
        double path_s_base = 0.0;
        for (size_t i = 0; i + 1 < candidate.centerline.size(); ++i)
        {
            const Eigen::Vector2d a = candidate.centerline[i];
            const Eigen::Vector2d b = candidate.centerline[i + 1];
            const Eigen::Vector2d seg = b - a;
            const double seg_len = seg.norm();
            if (seg_len < 1e-6)
                continue;
            const Eigen::Vector2d forward = seg / seg_len;
            const Eigen::Vector2d left(-forward.y(), forward.x());
            for (double s = 0.0; s <= seg_len + 1e-6; s += ds)
            {
                Eigen::Vector2d center = a + forward * s;
                const double path_s = path_s_base + s;
                const double available_width = updateAvailableAt(center, left, path_s);
                if (path_s <= start_grace)
                    continue;
                min_available_half_width = std::min(min_available_half_width, available_width);
                if (available_width < cfg_.min_half_width)
                {
                    if (maybeTruncateBeforeBlock(path_s))
                        return false;
                    candidate.half_width = std::min(candidate.half_width, min_available_half_width);
                    return true;
                }
                prefix_min_available_half_width =
                    std::min(prefix_min_available_half_width, available_width);
            }
            path_s_base += seg_len;
        }
        candidate.half_width = std::min(candidate.half_width, min_available_half_width);
        return candidate.half_width < cfg_.min_half_width;
    }

    for (double s = 0.0; s <= cfg_.length + 1e-6; s += ds)
    {
        Eigen::Vector2d center = candidate.start + candidate.forward * s;
        const double available_width = updateAvailableAt(center, candidate.left, s);
        if (s <= start_grace)
            continue;
        min_available_half_width = std::min(min_available_half_width, available_width);
        if (available_width < cfg_.min_half_width)
        {
            candidate.half_width = std::min(candidate.half_width, min_available_half_width);
            return true;
        }
        prefix_min_available_half_width =
            std::min(prefix_min_available_half_width, available_width);
    }
    candidate.half_width = std::min(candidate.half_width, min_available_half_width);
    return candidate.half_width < cfg_.min_half_width;
}

bool DynamicWalkingCorridor::optimizeCenterlineForStaticMap(
    const std::vector<Eigen::Vector2d> &reference_path,
    const Config &cfg,
    const std::function<bool(double, double)> &is_traversable,
    std::vector<Eigen::Vector2d> &optimized_path,
    double &optimized_half_width)
{
    optimized_path.clear();
    optimized_half_width = 0.0;
    if (reference_path.size() < 2 || !is_traversable)
        return false;

    const double range = std::max(0.0, cfg.static_opt_lateral_range);
    const double step = std::max(0.02, cfg.static_opt_lateral_step);
    std::vector<double> offsets;
    for (double off = -range; off <= range + 1e-6; off += step)
        offsets.push_back(off);
    if (offsets.empty())
        offsets.push_back(0.0);

    const size_t n = reference_path.size();
    const size_t m = offsets.size();
    const double inf = std::numeric_limits<double>::infinity();
    std::vector<std::vector<double>> dp(n, std::vector<double>(m, inf));
    std::vector<std::vector<int>> parent(n, std::vector<int>(m, -1));
    std::vector<std::vector<double>> widths(n, std::vector<double>(m, 0.0));

    for (size_t i = 0; i < n; ++i)
    {
        const Eigen::Vector2d left = pathLeftAt(reference_path, i);
        for (size_t j = 0; j < m; ++j)
        {
            const Eigen::Vector2d p = reference_path[i] + offsets[j] * left;
            const double width = availableHalfWidthAt(p, left, cfg, is_traversable);
            widths[i][j] = width;
            if (width + 1e-6 < cfg.min_half_width)
                continue;

            const double base_cost =
                cfg.lateral_offset_weight * std::abs(offsets[j]) -
                0.05 * std::min(width, cfg.half_width);
            if (i == 0)
            {
                dp[i][j] = base_cost;
                continue;
            }

            for (size_t k = 0; k < m; ++k)
            {
                if (!std::isfinite(dp[i - 1][k]))
                    continue;
                const double d = offsets[j] - offsets[k];
                const double cost =
                    dp[i - 1][k] + base_cost + cfg.static_opt_smooth_weight * d * d;
                if (cost < dp[i][j])
                {
                    dp[i][j] = cost;
                    parent[i][j] = static_cast<int>(k);
                }
            }
        }
    }

    int best = -1;
    double best_cost = inf;
    for (size_t j = 0; j < m; ++j)
    {
        if (dp[n - 1][j] < best_cost)
        {
            best_cost = dp[n - 1][j];
            best = static_cast<int>(j);
        }
    }
    if (best < 0 || !std::isfinite(best_cost))
        return false;

    std::vector<int> choice(n, 0);
    choice[n - 1] = best;
    for (size_t i = n - 1; i > 0; --i)
    {
        const int p = parent[i][choice[i]];
        if (p < 0)
            return false;
        choice[i - 1] = p;
    }

    optimized_path.reserve(n);
    optimized_half_width = cfg.half_width;
    for (size_t i = 0; i < n; ++i)
    {
        const Eigen::Vector2d left = pathLeftAt(reference_path, i);
        const int j = choice[i];
        optimized_path.push_back(reference_path[i] + offsets[j] * left);
        optimized_half_width = std::min(optimized_half_width, widths[i][j]);
    }
    return optimized_half_width + 1e-6 >= cfg.min_half_width;
}

bool DynamicWalkingCorridor::isDynamicallyBlocked(
    const Candidate &candidate,
    const std::vector<Eigen::Vector3d> &obs_pos,
    const std::vector<Eigen::Vector3d> &obs_vel,
    const std::vector<Eigen::Vector3d> &obs_size,
    int &block_count) const
{
    block_count = 0;
    if (obs_pos.empty())
        return false;

    const double dt = std::max(0.05, cfg_.prediction_dt);
    const double robot_speed = std::max(0.0, cfg_.robot_speed);
    for (double t = 0.0; t <= cfg_.prediction_horizon + 1e-6; t += dt)
    {
        const double robot_s = hasTimedPolyline(candidate)
                                   ? polylineDistanceAtTime(candidate.centerline,
                                                            candidate.centerline_times,
                                                            t)
                                   : std::min(candidate.length, robot_speed * t);
        for (size_t i = 0; i < obs_pos.size(); ++i)
        {
            Eigen::Vector2d ped(obs_pos[i](0), obs_pos[i](1));
            if (i < obs_vel.size())
                ped += Eigen::Vector2d(obs_vel[i](0), obs_vel[i](1)) * t;

            Eigen::Vector2d rel = ped - candidate.start;
            const double s = rel.dot(candidate.forward);
            const double l = rel.dot(candidate.left);
            const double r = obstacleRadius(i, obs_size, cfg_.dynamic_min_radius) +
                             cfg_.robot_radius + cfg_.dynamic_safety_margin;
            const double progress_window = r + std::max(0.0, cfg_.dynamic_progress_margin);

            if (candidate.centerline.size() >= 2)
            {
                double ped_s = 0.0;
                double lateral_dist = std::numeric_limits<double>::infinity();
                if (projectPointToPolyline(candidate.centerline, ped, ped_s, lateral_dist) &&
                    lateral_dist <= candidate.half_width + r &&
                    std::abs(ped_s - robot_s) <= progress_window)
                    block_count++;
            }
            else if (s >= -r && s <= cfg_.length + r &&
                     std::abs(l) <= candidate.half_width + r &&
                     std::abs(s - robot_s) <= progress_window)
            {
                block_count++;
            }
        }
    }
    return block_count > 0;
}

double DynamicWalkingCorridor::computeFrontPassCost(
    const Candidate &candidate,
    const std::vector<Eigen::Vector3d> &obs_pos,
    const std::vector<Eigen::Vector3d> &obs_vel) const
{
    if (cfg_.front_pass_weight <= 0.0 || obs_pos.empty())
        return 0.0;

    double cost = 0.0;
    const double dt = std::max(0.05, cfg_.prediction_dt);
    const double sigma_s_sq = std::max(1e-4, cfg_.front_pass_sigma_s * cfg_.front_pass_sigma_s);
    const double sigma_l_sq = std::max(1e-4, cfg_.front_pass_sigma_l * cfg_.front_pass_sigma_l);

    for (double t = 0.0; t <= cfg_.prediction_horizon + 1e-6; t += dt)
    {
        const double travel = hasTimedPolyline(candidate)
                                  ? polylineDistanceAtTime(candidate.centerline,
                                                           candidate.centerline_times,
                                                           t)
                                  : std::min(cfg_.length, std::max(0.0, cfg_.robot_speed) * t);
        const Eigen::Vector2d robot =
            candidate.centerline.size() >= 2
                ? interpolatePolyline(candidate.centerline, travel)
                : candidate.start + candidate.forward * travel;
        for (size_t i = 0; i < obs_pos.size(); ++i)
        {
            if (i >= obs_vel.size())
                continue;
            Eigen::Vector2d v(obs_vel[i](0), obs_vel[i](1));
            const double speed = v.norm();
            if (speed < 1e-3)
                continue;

            const Eigen::Vector2d e = v / speed;
            const Eigen::Vector2d e_perp(-e.y(), e.x());
            Eigen::Vector2d ped(obs_pos[i](0), obs_pos[i](1));
            ped += v * t;

            const Eigen::Vector2d rel = robot - ped;
            const double s = rel.dot(e);
            const double l = rel.dot(e_perp);
            if (s <= 0.0 || s > cfg_.front_pass_length ||
                std::abs(l) > cfg_.front_pass_lateral)
                continue;

            const double gs = std::exp(-0.5 * s * s / sigma_s_sq);
            const double gl = std::exp(-0.5 * l * l / sigma_l_sq);
            cost += cfg_.front_pass_weight * gs * gl * dt;
        }
    }
    return cost;
}

double DynamicWalkingCorridor::outsideDistance(const Candidate &candidate,
                                               const Eigen::Vector2d &point)
{
    if (candidate.centerline.size() >= 2)
    {
        double min_centerline_dist = std::numeric_limits<double>::infinity();
        double beyond_end_lateral_dist = std::numeric_limits<double>::infinity();
        bool beyond_truncated_end = false;
        for (size_t i = 0; i + 1 < candidate.centerline.size(); ++i)
        {
            const Eigen::Vector2d a = candidate.centerline[i];
            const Eigen::Vector2d b = candidate.centerline[i + 1];
            const Eigen::Vector2d ab = b - a;
            const double len_sq = ab.squaredNorm();
            if (len_sq < 1e-9)
                continue;
            double t = (point - a).dot(ab) / len_sq;
            t = std::max(0.0, std::min(1.0, t));
            const Eigen::Vector2d proj = a + t * ab;
            min_centerline_dist = std::min(min_centerline_dist, (point - proj).norm());

            if (candidate.truncated_static && i + 2 == candidate.centerline.size())
            {
                const double seg_len = ab.norm();
                if (seg_len > 1e-6)
                {
                    const Eigen::Vector2d forward = ab / seg_len;
                    const Eigen::Vector2d left(-forward.y(), forward.x());
                    const Eigen::Vector2d rel_end = point - b;
                    if (rel_end.dot(forward) > 0.0)
                    {
                        beyond_truncated_end = true;
                        beyond_end_lateral_dist = std::abs(rel_end.dot(left));
                    }
                }
            }
        }
        if (beyond_truncated_end && std::isfinite(beyond_end_lateral_dist))
            return std::max(0.0, beyond_end_lateral_dist - std::max(0.0, candidate.half_width));
        if (std::isfinite(min_centerline_dist))
            return std::max(0.0, min_centerline_dist - std::max(0.0, candidate.half_width));
    }

    const Eigen::Vector2d forward = normalizedOrDefault(candidate.forward);
    Eigen::Vector2d left = candidate.left;
    if (left.norm() < 1e-6)
        left = Eigen::Vector2d(-forward.y(), forward.x());
    else
        left.normalize();

    const double length = std::max(0.0, candidate.length);
    const double half_width = std::max(0.0, candidate.half_width);
    const Eigen::Vector2d rel = point - candidate.start;
    const double s = rel.dot(forward);
    const double l = rel.dot(left);

    const double outside_s = std::max(std::max(-s, s - length), 0.0);
    const double outside_l = std::max(std::abs(l) - half_width, 0.0);
    return std::sqrt(outside_s * outside_s + outside_l * outside_l);
}

} // namespace cane_planner
