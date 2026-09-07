#ifndef PATH_SEARCHING_CORRIDOR_FAILURE_SNAPSHOT_H
#define PATH_SEARCHING_CORRIDOR_FAILURE_SNAPSHOT_H
#include <path_searching/convex_corridor.h>
namespace cane_planner {
// Diagnostic-only input capture. No ROS transport or alternative geometry implementation.
struct CorridorFailureSnapshot {
    ConvexCorridor::Grid grid;
    std::vector<Eigen::Vector2d> path;
    ConvexCorridor::Config config;
    ConvexCorridor::Result failure;
};
bool readCorridorFailureSnapshot(const std::string& file, CorridorFailureSnapshot& snapshot,
                                 std::string& error);
std::string defaultCorridorFailureDirectory();
class CorridorFailureCapture {
public:
    enum class Status { SKIPPED, SAVED, IO_ERROR };
    void resetForGoal() { attempted_ = false; }
    // The first eligible failure consumes the latch even on I/O error: no per-frame disk retries.
    Status saveOnce(const std::string& directory, const ConvexCorridor::Grid& grid,
                    const std::vector<Eigen::Vector2d>& path, const ConvexCorridor::Config& config,
                    const ConvexCorridor::Result& failure, std::string& file, std::string& error);
private:
    bool attempted_ = false;
};
}
#endif
