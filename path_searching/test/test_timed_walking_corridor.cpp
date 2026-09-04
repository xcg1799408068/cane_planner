#include <gtest/gtest.h>

#include <path_searching/timed_walking_corridor.h>
#include <path_searching/trajectory_feasibility.h>

using cane_planner::TimedTrajectorySource;
using cane_planner::TimedWalkingCorridor;
using cane_planner::TimedWalkingCorridorSegment;
using cane_planner::TrajectoryFeasibility;
using cane_planner::corridorDeviationFeasible;
using cane_planner::selectBestTrajectoryIndex;

TEST(TimedWalkingCorridor, TracksTimeRangeAndSegmentLookup)
{
    TimedWalkingCorridor corridor;
    corridor.source = TimedTrajectorySource::PREVIOUS_MPPI;

    TimedWalkingCorridorSegment first;
    first.t_start = 0.0;
    first.t_end = 0.5;
    first.half_width = 0.4;
    first.centerline = {Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(1.0, 0.0)};

    TimedWalkingCorridorSegment second;
    second.t_start = 0.5;
    second.t_end = 1.0;
    second.half_width = 0.4;
    second.centerline = {Eigen::Vector2d(1.0, 0.0), Eigen::Vector2d(1.0, 1.0)};

    corridor.segments = {first, second};

    ASSERT_TRUE(corridor.valid());
    EXPECT_NEAR(0.0, corridor.tStart(), 1e-9);
    EXPECT_NEAR(1.0, corridor.tEnd(), 1e-9);
    ASSERT_NE(nullptr, corridor.segmentAtTime(0.25));
    EXPECT_NEAR(1.0, corridor.segmentAtTime(0.75)->centerline.back().y(), 1e-9);
    EXPECT_EQ(nullptr, corridor.segmentAtTime(1.5));
}

TEST(TimedWalkingCorridor, ComputesOutsideDistanceAtSegmentTime)
{
    TimedWalkingCorridor corridor;
    TimedWalkingCorridorSegment first;
    first.t_start = 0.0;
    first.t_end = 0.5;
    first.start = Eigen::Vector2d(0.0, 0.0);
    first.end = Eigen::Vector2d(1.0, 0.0);
    first.forward = Eigen::Vector2d::UnitX();
    first.left = Eigen::Vector2d::UnitY();
    first.half_width = 0.4;

    TimedWalkingCorridorSegment second;
    second.t_start = 0.5;
    second.t_end = 1.0;
    second.start = Eigen::Vector2d(1.0, 0.0);
    second.end = Eigen::Vector2d(1.0, 1.0);
    second.forward = Eigen::Vector2d::UnitY();
    second.left = Eigen::Vector2d(-1.0, 0.0);
    second.half_width = 0.4;
    corridor.segments = {first, second};

    EXPECT_DOUBLE_EQ(0.0, corridor.outsideDistanceAtTime(
                              0.25, Eigen::Vector2d(0.5, 0.2)));
    EXPECT_NEAR(0.2, corridor.outsideDistanceAtTime(
                         0.25, Eigen::Vector2d(0.5, 0.6)), 1e-9);
    EXPECT_DOUBLE_EQ(0.0, corridor.outsideDistanceAtTime(
                              0.75, Eigen::Vector2d(0.8, 0.5)));
    EXPECT_NEAR(0.2, corridor.outsideDistanceAtTime(
                         0.75, Eigen::Vector2d(0.4, 0.5)), 1e-9);
    EXPECT_DOUBLE_EQ(0.0, corridor.outsideDistanceAtTime(
                              1.5, Eigen::Vector2d(5.0, 5.0)));
}

TEST(TimedWalkingCorridor, HalfPlanesOverrideCenterlineBandDistance)
{
    TimedWalkingCorridor corridor;
    TimedWalkingCorridorSegment segment;
    segment.t_start = 0.0;
    segment.t_end = 1.0;
    segment.start = Eigen::Vector2d(0.0, 0.0);
    segment.end = Eigen::Vector2d(1.0, 0.0);
    segment.forward = Eigen::Vector2d::UnitX();
    segment.left = Eigen::Vector2d::UnitY();
    segment.half_width = 0.1;
    using HalfPlane2D = TimedWalkingCorridorSegment::HalfPlane2D;
    segment.half_planes = {
        HalfPlane2D{Eigen::Vector2d(1.0, 0.0), -1.0},
        HalfPlane2D{Eigen::Vector2d(-1.0, 0.0), 0.0},
        HalfPlane2D{Eigen::Vector2d(0.0, 1.0), -1.0},
        HalfPlane2D{Eigen::Vector2d(0.0, -1.0), 0.0},
    };
    corridor.segments = {segment};

    EXPECT_DOUBLE_EQ(0.0, corridor.outsideDistanceAtTime(
                              0.5, Eigen::Vector2d(0.5, 0.8)));
    EXPECT_NEAR(0.2, corridor.outsideDistanceAtTime(
                         0.5, Eigen::Vector2d(1.2, 0.5)), 1e-9);
}

TEST(TimedWalkingCorridor, AllowsAdjacentConvexCellContinuityAtTurns)
{
    using HalfPlane2D = TimedWalkingCorridorSegment::HalfPlane2D;
    TimedWalkingCorridor corridor;

    TimedWalkingCorridorSegment first;
    first.t_start = 0.0;
    first.t_end = 0.5;
    first.half_planes = {
        HalfPlane2D{Eigen::Vector2d(1.0, 0.0), -1.0},
        HalfPlane2D{Eigen::Vector2d(-1.0, 0.0), 0.0},
        HalfPlane2D{Eigen::Vector2d(0.0, 1.0), -1.0},
        HalfPlane2D{Eigen::Vector2d(0.0, -1.0), 0.0},
    };

    TimedWalkingCorridorSegment second;
    second.t_start = 0.5;
    second.t_end = 1.0;
    second.half_planes = {
        HalfPlane2D{Eigen::Vector2d(1.0, 0.0), -2.0},
        HalfPlane2D{Eigen::Vector2d(-1.0, 0.0), 1.0},
        HalfPlane2D{Eigen::Vector2d(0.0, 1.0), -1.0},
        HalfPlane2D{Eigen::Vector2d(0.0, -1.0), 0.0},
    };
    corridor.segments = {first, second};

    EXPECT_DOUBLE_EQ(0.0, corridor.outsideDistanceAtTime(
                              0.25, Eigen::Vector2d(1.5, 0.5)));
}

TEST(TrajectoryFeasibility, EncodesGoStopFromMppiFeasibleTrajectory)
{
    TrajectoryFeasibility feasible;
    feasible.valid_trajectory_count = 3;
    feasible.inside_corridor_ratio = 0.82;

    EXPECT_TRUE(feasible.hasFeasibleTrajectory());
    EXPECT_FALSE(feasible.shouldStop());

    TrajectoryFeasibility infeasible;
    infeasible.valid_trajectory_count = 0;
    infeasible.inside_corridor_ratio = 0.0;

    EXPECT_FALSE(infeasible.hasFeasibleTrajectory());
    EXPECT_TRUE(infeasible.shouldStop());
}

TEST(TrajectoryFeasibility, CorridorEvaluationOverridesGenericValidity)
{
    TrajectoryFeasibility feasibility;
    feasibility.valid_trajectory_count = 12;
    feasibility.corridor_feasible_trajectory_count = 0;
    feasibility.corridor_evaluated = true;
    feasibility.inside_corridor_ratio = 0.25;

    EXPECT_FALSE(feasibility.hasFeasibleTrajectory());
    EXPECT_TRUE(feasibility.shouldStop());

    feasibility.corridor_feasible_trajectory_count = 1;
    EXPECT_TRUE(feasibility.hasFeasibleTrajectory());
    EXPECT_FALSE(feasibility.shouldStop());
}

TEST(TrajectoryFeasibility, ReportsNoFeasibleTrajectoryStopReason)
{
    TrajectoryFeasibility infeasible;
    infeasible.valid_trajectory_count = 8;
    infeasible.corridor_feasible_trajectory_count = 0;
    infeasible.corridor_evaluated = true;

    EXPECT_TRUE(infeasible.shouldStop());
    EXPECT_EQ("NO_FEASIBLE_TRAJECTORY", infeasible.stopReason());

    TrajectoryFeasibility feasible;
    feasible.valid_trajectory_count = 8;
    feasible.corridor_feasible_trajectory_count = 2;
    feasible.corridor_evaluated = true;

    EXPECT_FALSE(feasible.shouldStop());
    EXPECT_EQ("OK", feasible.stopReason());
}

TEST(TrajectoryFeasibility, SelectsBestInsideCorridorWhenAvailable)
{
    const std::vector<double> costs = {4.0, 1.0, 2.0};
    const std::vector<bool> corridor_feasible = {true, false, true};

    EXPECT_EQ(2, selectBestTrajectoryIndex(costs, corridor_feasible, true));
    EXPECT_EQ(1, selectBestTrajectoryIndex(costs, corridor_feasible, false));
}

TEST(TrajectoryFeasibility, AllowsBoundedCorridorDeviation)
{
    EXPECT_TRUE(corridorDeviationFeasible(0.0, 0.3));
    EXPECT_TRUE(corridorDeviationFeasible(0.2, 0.3));
    EXPECT_FALSE(corridorDeviationFeasible(0.31, 0.3));
}

TEST(TrajectoryFeasibility, FallsBackToBestFiniteCostWhenNoCorridorFeasibleSample)
{
    const std::vector<double> costs = {
        std::numeric_limits<double>::infinity(),
        3.0,
        2.0,
    };
    const std::vector<bool> corridor_feasible = {false, false, false};

    EXPECT_EQ(2, selectBestTrajectoryIndex(costs, corridor_feasible, true));
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
