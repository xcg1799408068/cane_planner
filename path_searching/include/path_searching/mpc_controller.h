#ifndef _MPC_CONTROLLER_H_
#define _MPC_CONTROLLER_H_

#include <Eigen/Eigen>
#include <limits>
#include <random>
#include <vector>

#include <ros/ros.h>

#include <path_searching/lfpc.h>
#include <path_searching/dynamic_risk_field.h>
#include <path_searching/dynamic_walking_corridor.h>
#include <path_searching/convex_corridor.h>
#include <plan_env/collision_detection.h>

namespace cane_planner
{

class MpcController
{
public:
    struct Config
    {
        // Horizon
        int horizon_steps = 6;       // N: lookahead steps

        // MPPI sampling
        int num_samples = 100;       // K
        double temperature = 0.1;    // lower = more greedy
        int mpc_iters = 1;           // MPPI iterations per plan

        // Control bounds
        double max_al = 0.4;
        double min_al = 0.05;
        double max_aw = 0.15;
        double max_api = 0.5236;     // ~30 deg

        // Exploration noise std
        double sigma_al = 0.06;
        double sigma_aw = 0.04;
        double sigma_api = 0.12;

        // Cost weights
        double w_move = 1.0;
        double w_steer = 0.5;
        double w_risk = 0.0;
        double w_goal = 10.0;
        double w_dapi = 0.0;    // steering rate penalty: |api[n] - api[n-1]|

        // Dynamic pedestrians are handled as geometric timed-corridor obstacles.
        // This legacy DRF switch is kept for parameter compatibility only.
        bool dynamic_hard_reject_enable = false;
        bool dynamic_collision_hard_reject_enable = true;
        double risk_hard_threshold = 8.5;

        // Static obstacle: high penalty per colliding point (effectively hard)
        double static_penalty = 500.0;
        double w_static = 1.0;

        // Dynamic obstacle geometry. This is the physical collision check;
        // wider pedestrian keep-out regions come from timed corridor ellipses.
        bool use_dynamic_size = true;
        double dynamic_safety_margin = 0.05;
        double dynamic_min_radius = 0.20;

        // SDF proximity cost: repulsive gradient around obstacles
        double w_prox = 5.0;
        double prox_margin = 0.6;  // wider than collision margin (0.3)

        // Dynamic walking corridor constraint. Points outside the selected
        // corridor receive a quadratic penalty; large violations are rejected.
        bool corridor_enable = true;
        bool corridor_hard_reject_enable = true;
        double w_corridor = 20.0;
        double w_corridor_dynamic = 20.0;
        double corridor_hard_margin = 0.30;

        // Optional convex corridor constraint. This is developed in parallel
        // with the existing DWC corridor and is soft by default.
        bool convex_corridor_enable = false;
        bool convex_corridor_hard_reject_enable = false;
        double w_convex_corridor = 30.0;
        double convex_corridor_hard_margin = 0.05;

        // Use best trajectory instead of weighted average (avoids mode collapse)
        bool use_best = true;

        // Fix step length/width to nominal (human provides, only optimize api)
        bool fix_step_params = false;

        // Early goal arrival: stop rollout and skip terminal cost if within this radius
        double goal_arrival_threshold = 0.3;

        // FOV limitation: points beyond this distance from robot are treated as traversable
        // Set to <= 0 to disable (full map access)
        double fov_range = 5.0;

        // Warm-start nominal
        double nominal_al = 0.25;
        double nominal_aw = 0.0;
        double nominal_api = 0.0;
    };

    struct DebugMetrics
    {
        double plan_time_ms = 0.0;
        double valid_sample_ratio = 0.0;
        double best_total_cost = std::numeric_limits<double>::infinity();
        double min_dynamic_clearance = std::numeric_limits<double>::infinity();
        double min_cpa_time = std::numeric_limits<double>::infinity();
        double best_min_dynamic_clearance = std::numeric_limits<double>::infinity();
        double best_min_cpa_time = std::numeric_limits<double>::infinity();
        int dynamic_reject_count = 0;
        int static_reject_count = 0;
        int corridor_reject_count = 0;
        int convex_corridor_reject_count = 0;
        int convex_corridor_segments = 0;
        double max_convex_corridor_violation = 0.0;
        double candidate_inside_corridor_ratio = 1.0;
        int valid_trajectory_count = 0;
        int corridor_feasible_trajectory_count = 0;
        bool corridor_evaluated = false;
        int num_samples = 0;
        bool plan_valid = false;
    };

    MpcController();
    ~MpcController();

    void setParam(ros::NodeHandle &nh);
    void init();
    void reset();
    void resetWarmStart();
    void setModel(const LFPC::Ptr &model);
    void setCollision(const CollisionDetection::Ptr &col);
    void setRiskField(const DynamicRiskField &rf);
    void setWalkingCorridor(const DynamicWalkingCorridor::Candidate &candidate);
    void setWalkingCorridor(const DynamicWalkingCorridor::Candidate &candidate,
                            const TimedWalkingCorridor &timed_corridor);
    void clearWalkingCorridor();
    void setConvexCorridor(const std::vector<ConvexCorridor::Segment> &segments);
    void clearConvexCorridor();

    void setDynamicObstacles(const std::vector<Eigen::Vector3d> &pos,
                             const std::vector<Eigen::Vector3d> &vel);

    // Main API: plan one step, returns [al, aw, api]
    Eigen::Vector3d plan(const LFPC::Ptr &lfpc_base,
                         const Eigen::Vector3d &goal_pos,
                         const std::vector<Eigen::Vector3d> &obs_pos,
                         const std::vector<Eigen::Vector3d> &obs_vel,
                         const std::vector<Eigen::Vector3d> &obs_size = {});

    // Return the best predicted CoM path from last plan (for visualization)
    std::vector<Eigen::Vector3d> getBestPath() const { return best_path_; }

    // Access risk field for external visualization
    const DynamicRiskField& getRiskField() const { return risk_field_; }

    double lastPlanTimeMs() const { return last_plan_time_ms_; }

    bool lastPlanValid() const { return last_plan_valid_; }

    DebugMetrics getDebugMetrics() const { return last_debug_metrics_; }

    typedef shared_ptr<MpcController> Ptr;

private:
    Config cfg_;
    LFPC::Ptr lfpc_model_;
    CollisionDetection::Ptr collision_;
    DynamicRiskField risk_field_;
    bool has_walking_corridor_ = false;
    DynamicWalkingCorridor::Candidate walking_corridor_;
    bool has_timed_walking_corridor_ = false;
    TimedWalkingCorridor timed_walking_corridor_;
    bool has_convex_corridor_ = false;
    std::vector<ConvexCorridor::Segment> convex_corridor_segments_;

    // Pre-allocated LFPC pool for rollout
    std::vector<LFPC::Ptr> lfpc_pool_;
    bool pool_initialized_;

    // Warm-start control sequence: N x 3
    Eigen::MatrixXd warm_start_;

    // RNG
    std::mt19937 rng_;
    std::normal_distribution<double> normal_dist_;

    // Best path from last plan
    std::vector<Eigen::Vector3d> best_path_;

    // Timing
    double last_plan_time_ms_;
    bool last_plan_valid_ = true;
    DebugMetrics last_debug_metrics_;

    // Internal methods
    Eigen::MatrixXd makeNominalSequence(int N);
    std::vector<Eigen::MatrixXd> sampleSequences(const Eigen::MatrixXd &mean,
                                                  int N, int K);
    void rolloutBatch(const LFPC::Ptr &lfpc_base,
                      const std::vector<Eigen::MatrixXd> &samples,
                      const Eigen::Vector3d &goal_pos,
                      const std::vector<Eigen::Vector3d> &obs_pos,
                      const std::vector<Eigen::Vector3d> &obs_vel,
                      const std::vector<Eigen::Vector3d> &obs_size,
                      Eigen::VectorXd &costs,
                      std::vector<std::vector<Eigen::Vector3d>> &paths,
                      std::vector<double> &sample_min_dynamic_clearances,
                      std::vector<double> &sample_min_cpa_times,
                      std::vector<bool> &sample_corridor_feasible);
    Eigen::VectorXd computeWeights(const Eigen::VectorXd &costs);
    Eigen::MatrixXd weightedUpdate(const std::vector<Eigen::MatrixXd> &samples,
                                    const Eigen::VectorXd &weights,
                                    int N);
    void shiftSequence(Eigen::MatrixXd &mean, int N);
    void ensurePool(int K);
    const ConvexCorridor::Segment* nearestConvexSegment(const Eigen::Vector2d &point) const;
};

} // namespace cane_planner

#endif // _MPC_CONTROLLER_H_
