#ifndef _MPC_CONTROLLER_H_
#define _MPC_CONTROLLER_H_

#include <Eigen/Eigen>
#include <limits>
#include <random>
#include <vector>

#include <ros/ros.h>

#include <path_searching/lfpc.h>
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
        double w_steer = 0.5;
        double w_goal = 10.0;
        double w_dapi = 0.0;    // steering rate penalty: |api[n] - api[n-1]|

        // Use best trajectory instead of weighted average (avoids mode collapse)
        bool use_best = true;

        // Fix step length/width to nominal (human provides, only optimize api)
        bool fix_step_params = false;

        // Early goal arrival: stop rollout and skip terminal cost if within this radius
        double goal_arrival_threshold = 0.3;

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
        bool weighted_checked = false;
        bool weighted_fallback = false;
        double executed_total_cost = std::numeric_limits<double>::infinity();
    };

    MpcController();
    ~MpcController();

    void setParam(ros::NodeHandle &nh);
    void init();
    void reset();
    void resetWarmStart();
    void setCollision(const CollisionDetection::Ptr &col);
    void setConvexCorridor(const std::vector<ConvexCorridor::Segment> &segments);
    void clearConvexCorridor();

    // Static planner API: one LFPC step, returns [al, aw, api].
    Eigen::Vector3d plan(const LFPC::Ptr &lfpc_base,
                         const Eigen::Vector3d &goal_pos);

    // Return the best predicted CoM path from last plan (for visualization)
    std::vector<Eigen::Vector3d> getBestPath() const { return best_path_; }

    double lastPlanTimeMs() const { return last_plan_time_ms_; }

    bool lastPlanValid() const { return last_plan_valid_; }

    DebugMetrics getDebugMetrics() const { return last_debug_metrics_; }

    typedef shared_ptr<MpcController> Ptr;

private:
    friend class StaticMpcTestAccess;
    Config cfg_;
    CollisionDetection::Ptr collision_;
    bool has_convex_corridor_ = false;
    std::vector<ConvexCorridor::Segment> convex_corridor_segments_;

    // Pre-allocated LFPC pool for rollout
    std::vector<LFPC::Ptr> lfpc_pool_;

    // Warm-start control sequence: N x 3
    Eigen::MatrixXd warm_start_;

    // RNG
    std::mt19937 rng_;
    std::normal_distribution<double> normal_dist_;

    // Best path from last plan
    std::vector<Eigen::Vector3d> best_path_;

    // Timing
    double last_plan_time_ms_;
    bool last_plan_valid_ = false;
    DebugMetrics last_debug_metrics_;

    // Bounded diagnostic output; never used by planning or sampling.
    ros::Publisher sample_vis_pub_;
    bool sample_vis_enable_ = true;
    int sample_vis_count_ = 20;
    void publishSampleTrajectories(
        const std::vector<std::vector<Eigen::Vector3d>> &paths,
        const Eigen::VectorXd &costs);

    // Internal methods
    Eigen::MatrixXd makeNominalSequence(int N);
    std::vector<Eigen::MatrixXd> sampleSequences(const Eigen::MatrixXd &mean,
                                                  int N, int K);
    void rolloutBatch(const LFPC::Ptr &lfpc_base,
                      const std::vector<Eigen::MatrixXd> &samples,
                      const Eigen::Vector3d &goal_pos,
                      Eigen::VectorXd &costs,
                      std::vector<std::vector<Eigen::Vector3d>> &paths,
                      std::vector<bool> &sample_corridor_feasible);
    Eigen::VectorXd computeWeights(const Eigen::VectorXd &costs);
    Eigen::MatrixXd weightedUpdate(const std::vector<Eigen::MatrixXd> &samples,
                                    const Eigen::VectorXd &weights,
                                    int N);
    void shiftSequence(Eigen::MatrixXd &mean, int N);
    void ensurePool(int K);
};

} // namespace cane_planner

#endif // _MPC_CONTROLLER_H_
