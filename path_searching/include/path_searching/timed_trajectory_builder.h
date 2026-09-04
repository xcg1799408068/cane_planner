#ifndef _TIMED_TRAJECTORY_BUILDER_H_
#define _TIMED_TRAJECTORY_BUILDER_H_

#include <path_searching/timed_trajectory.h>

namespace cane_planner
{

class TimedTrajectoryBuilder
{
public:
    static TimedTrajectory buildNominal(const std::vector<Eigen::Vector3d> &previous_mppi_path,
                                        const std::vector<Eigen::Vector2d> &global_reference,
                                        double mppi_dt,
                                        double reference_speed,
                                        double max_length);

private:
    static TimedTrajectory fromPreviousMppi(const std::vector<Eigen::Vector3d> &path,
                                            double dt,
                                            double max_length);
    static TimedTrajectory fromGlobalReference(const std::vector<Eigen::Vector2d> &path,
                                               double speed,
                                               double max_length);
};

} // namespace cane_planner

#endif // _TIMED_TRAJECTORY_BUILDER_H_
