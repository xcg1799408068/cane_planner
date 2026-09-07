#ifndef _TRAJECTORY_FEASIBILITY_H_
#define _TRAJECTORY_FEASIBILITY_H_

#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace cane_planner
{

enum class FinalSequenceChoice { STOP, BEST, WEIGHTED };

// The optimizer and its regression tests share this final safety arbitration.
inline FinalSequenceChoice chooseFinalSequence(bool corridor_valid, int best_index,
                                               bool use_best, double weighted_cost,
                                               bool weighted_corridor_feasible)
{
    if (!corridor_valid || best_index < 0)
        return FinalSequenceChoice::STOP;
    if (!use_best && std::isfinite(weighted_cost) && weighted_corridor_feasible)
        return FinalSequenceChoice::WEIGHTED;
    return FinalSequenceChoice::BEST;
}

struct TrajectoryFeasibility
{
    int valid_trajectory_count = 0;
    int corridor_feasible_trajectory_count = 0;
    double inside_corridor_ratio = 0.0;
    bool corridor_evaluated = false;

    bool hasFeasibleTrajectory() const
    {
        return corridor_evaluated
                   ? corridor_feasible_trajectory_count > 0
                   : valid_trajectory_count > 0;
    }
    bool shouldStop() const { return !hasFeasibleTrajectory(); }
    std::string stopReason() const
    {
        return shouldStop() ? "NO_FEASIBLE_TRAJECTORY" : "OK";
    }
};

inline bool corridorDeviationFeasible(const double outside_distance,
                                      const double hard_margin)
{
    return outside_distance <= std::max(0.0, hard_margin);
}

inline int selectBestTrajectoryIndex(const std::vector<double> &costs,
                                     const std::vector<bool> &corridor_feasible,
                                     const bool corridor_evaluated)
{
    auto choose_best = [&](const bool require_corridor_feasible) {
        int best = -1;
        double best_cost = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < costs.size(); ++i)
        {
            if (!std::isfinite(costs[i]))
                continue;
            if (require_corridor_feasible &&
                (i >= corridor_feasible.size() || !corridor_feasible[i]))
                continue;
            if (costs[i] < best_cost)
            {
                best_cost = costs[i];
                best = static_cast<int>(i);
            }
        }
        return best;
    };

    if (corridor_evaluated)
    {
        const int corridor_best = choose_best(true);
        if (corridor_best >= 0)
            return corridor_best;
    }
    return choose_best(false);
}

} // namespace cane_planner

#endif // _TRAJECTORY_FEASIBILITY_H_
