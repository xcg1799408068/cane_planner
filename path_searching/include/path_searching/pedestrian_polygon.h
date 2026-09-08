#ifndef PATH_SEARCHING_PEDESTRIAN_POLYGON_H
#define PATH_SEARCHING_PEDESTRIAN_POLYGON_H
#include <path_searching/convex_corridor.h>
namespace cane_planner {
// Current-frame engineering approximation of IROS 2025 DWCG Algorithm 1.
// Acceleration-like force (m/s^2), not a trajectory prediction or body certificate
// for the human/cane system. Desired velocity = observed velocity: drive is zero.
class PedestrianPolygon {
public:
    using Polygon = std::vector<Eigen::Vector2d>;
    struct Observation {
        Eigen::Vector2d position = Eigen::Vector2d::Zero();
        Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
        Eigen::Vector2d full_size = Eigen::Vector2d::Zero(); // world-axis enclosing box, m
    };
    struct Config {
        double safety_radius = .4;       // m; raised to physical enclosing radius
        double wall_gain = 1., system_gain = 1.; // m/s^2
        double decay_length = .5;        // m
        double fov_amplification = 2.;   // dimensionless, +/-110 degrees
        double alpha = .3;               // s^2, force-to-distance, NOT prediction time
        double max_force = 4.;           // m/s^2
        double max_extension = 1.5;      // m beyond safety radius
        double influence_distance = 3.; // m; finite nearest-boundary query range
        double max_seconds = .02;        // cooperative per-observation bound
    };
    struct Result {
        bool valid = false;
        std::string reason;
        Polygon body, hull;
        Eigen::Vector2d force = Eigen::Vector2d::Zero();
    };
    static bool validConfig(const Config&);
    static bool validObservation(const Observation&);
    static Polygon convexHull(Polygon);
    static Result fromForce(const Observation&, const Eigen::Vector2d&, const Config&);
    static Result build(const Observation&, const Eigen::Vector3d& system_pose,
                        const ConvexCorridor::Grid&, const Config&);
};
}
#endif
