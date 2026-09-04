#include <path_searching/mpc_controller.h>
#include <path_searching/trajectory_feasibility.h>
#include <cmath>
#include <algorithm>
#include <chrono>

namespace cane_planner
{

MpcController::MpcController()
    : pool_initialized_(false)
    , normal_dist_(0.0, 1.0)
    , last_plan_time_ms_(0.0)
{
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    rng_.seed(seed);
}

MpcController::~MpcController()
{
    lfpc_pool_.clear();
}

void MpcController::setParam(ros::NodeHandle &nh)
{
    nh.param("mpc/horizon_steps", cfg_.horizon_steps, 6);
    nh.param("mpc/num_samples", cfg_.num_samples, 100);
    nh.param("mpc/temperature", cfg_.temperature, 0.1);
    nh.param("mpc/mpc_iters", cfg_.mpc_iters, 1);

    nh.param("mpc/max_al", cfg_.max_al, 0.4);
    nh.param("mpc/min_al", cfg_.min_al, 0.05);
    nh.param("mpc/max_aw", cfg_.max_aw, 0.15);
    nh.param("mpc/max_api", cfg_.max_api, 0.5236);

    nh.param("mpc/sigma_al", cfg_.sigma_al, 0.06);
    nh.param("mpc/sigma_aw", cfg_.sigma_aw, 0.04);
    nh.param("mpc/sigma_api", cfg_.sigma_api, 0.12);

    nh.param("mpc/w_move", cfg_.w_move, 1.0);
    nh.param("mpc/w_steer", cfg_.w_steer, 0.5);
    nh.param("mpc/w_risk", cfg_.w_risk, 0.0);
    nh.param("mpc/w_goal", cfg_.w_goal, 10.0);
    nh.param("mpc/w_dapi", cfg_.w_dapi, 0.0);
    nh.param("mpc/dynamic_hard_reject_enable", cfg_.dynamic_hard_reject_enable, false);
    nh.param("mpc/dynamic_collision_hard_reject_enable", cfg_.dynamic_collision_hard_reject_enable, true);
    nh.param("mpc/static_penalty", cfg_.static_penalty, 500.0);
    nh.param("mpc/w_static", cfg_.w_static, 1.0);
    nh.param("mpc/use_dynamic_size", cfg_.use_dynamic_size, true);
    nh.param("mpc/dynamic_safety_margin", cfg_.dynamic_safety_margin, 0.05);
    nh.param("mpc/dynamic_min_radius", cfg_.dynamic_min_radius, 0.20);
    nh.param("mpc/w_prox", cfg_.w_prox, 5.0);
    nh.param("mpc/prox_margin", cfg_.prox_margin, 0.6);
    nh.param("mpc/corridor_enable", cfg_.corridor_enable, true);
    nh.param("mpc/corridor_hard_reject_enable", cfg_.corridor_hard_reject_enable, true);
    nh.param("mpc/w_corridor", cfg_.w_corridor, 20.0);
    nh.param("mpc/w_corridor_dynamic", cfg_.w_corridor_dynamic, cfg_.w_corridor);
    nh.param("mpc/corridor_hard_margin", cfg_.corridor_hard_margin, 0.30);
    nh.param("mpc/convex_corridor_enable", cfg_.convex_corridor_enable, false);
    nh.param("mpc/convex_corridor_hard_reject_enable", cfg_.convex_corridor_hard_reject_enable, false);
    nh.param("mpc/w_convex_corridor", cfg_.w_convex_corridor, 30.0);
    nh.param("mpc/convex_corridor_hard_margin", cfg_.convex_corridor_hard_margin, 0.05);
    nh.param("mpc/use_best", cfg_.use_best, true);
    nh.param("mpc/fix_step_params", cfg_.fix_step_params, false);
    nh.param("mpc/goal_arrival_threshold", cfg_.goal_arrival_threshold, 0.3);
    nh.param("mpc/fov_range", cfg_.fov_range, 5.0);

    nh.param("mpc/nominal_al", cfg_.nominal_al, 0.25);
    nh.param("mpc/nominal_aw", cfg_.nominal_aw, 0.0);
    nh.param("mpc/nominal_api", cfg_.nominal_api, 0.0);

    // Risk field config
    DynamicRiskField::Config rf_cfg;
    nh.param("mpc/risk_tau", rf_cfg.tau, 1.0);
    nh.param("mpc/risk_A", rf_cfg.A_risk, 5.0);
    nh.param("mpc/risk_sigma_x", rf_cfg.sigma_x, 0.4);
    nh.param("mpc/risk_sigma_y", rf_cfg.sigma_y, 0.22);
    nh.param("mpc/risk_cutoff", rf_cfg.cutoff_dist, 3.0);
    nh.param("mpc/risk_halo_scale", rf_cfg.halo_scale, 0.0);
    nh.param("mpc/risk_halo_ratio", rf_cfg.halo_ratio, 0.25);
    nh.param("mpc/risk_cpa_enable", rf_cfg.cpa_enable, false);
    nh.param("mpc/risk_cpa_weight", rf_cfg.cpa_weight, 0.6);
    nh.param("mpc/risk_cpa_sigma_d", rf_cfg.cpa_sigma_d, 0.65);
    nh.param("mpc/risk_cpa_tau", rf_cfg.cpa_tau, 1.0);
    nh.param("mpc/risk_cpa_time_horizon", rf_cfg.cpa_time_horizon, 2.0);
    nh.param("mpc/risk_cpa_cutoff_dist", rf_cfg.cpa_cutoff_dist, 3.0);
    risk_field_.setConfig(rf_cfg);

    // Hard threshold = peak × ratio, auto-scales with risk_A
    double hard_ratio;
    nh.param("mpc/risk_hard_threshold_ratio", hard_ratio, 0.5);
    cfg_.risk_hard_threshold = rf_cfg.A_risk * hard_ratio;
}

void MpcController::init()
{
    warm_start_.resize(0, 0);
    best_path_.clear();
}

void MpcController::reset()
{
    warm_start_.resize(0, 0);
    best_path_.clear();
}

void MpcController::resetWarmStart()
{
    warm_start_.resize(0, 0);  // forces fresh nominal init on next plan()
}

void MpcController::setModel(const LFPC::Ptr &model)
{
    lfpc_model_ = model;
}

void MpcController::setCollision(const CollisionDetection::Ptr &col)
{
    collision_ = col;
}

void MpcController::setRiskField(const DynamicRiskField &rf)
{
    risk_field_ = rf;
}

void MpcController::setWalkingCorridor(const DynamicWalkingCorridor::Candidate &candidate)
{
    walking_corridor_ = candidate;
    has_walking_corridor_ = candidate.feasible && candidate.length > 1e-6 && candidate.half_width > 1e-6;
    has_timed_walking_corridor_ = false;
    timed_walking_corridor_.clear();
}

void MpcController::setWalkingCorridor(
    const DynamicWalkingCorridor::Candidate &candidate,
    const TimedWalkingCorridor &timed_corridor)
{
    setWalkingCorridor(candidate);
    timed_walking_corridor_ = timed_corridor;
    has_timed_walking_corridor_ = has_walking_corridor_ && timed_walking_corridor_.valid();
}

void MpcController::clearWalkingCorridor()
{
    has_walking_corridor_ = false;
    has_timed_walking_corridor_ = false;
    timed_walking_corridor_.clear();
}

void MpcController::setConvexCorridor(const std::vector<ConvexCorridor::Segment> &segments)
{
    convex_corridor_segments_ = segments;
    has_convex_corridor_ = !convex_corridor_segments_.empty();
}

void MpcController::clearConvexCorridor()
{
    has_convex_corridor_ = false;
    convex_corridor_segments_.clear();
}

void MpcController::setDynamicObstacles(const std::vector<Eigen::Vector3d> &pos,
                                        const std::vector<Eigen::Vector3d> &vel)
{
    // No-op: obstacles are passed in plan() directly.
    // This method exists for API compatibility.
}

// =========================================================================
// Main API
// =========================================================================

Eigen::Vector3d MpcController::plan(const LFPC::Ptr &lfpc_base,
                                     const Eigen::Vector3d &goal_pos,
                                     const std::vector<Eigen::Vector3d> &obs_pos,
                                     const std::vector<Eigen::Vector3d> &obs_vel,
                                     const std::vector<Eigen::Vector3d> &obs_size)
{
    auto t_start = std::chrono::high_resolution_clock::now();

    int N = cfg_.horizon_steps;
    int K = cfg_.num_samples;
    last_debug_metrics_ = DebugMetrics();
    last_debug_metrics_.num_samples = K;

    ensurePool(K);

    // Initialize or shift warm-start
    if (warm_start_.rows() != N)
    {
        warm_start_ = makeNominalSequence(N);
        // Tiny random bias on first control to break left-right symmetry
        warm_start_(0, 2) += normal_dist_(rng_) * 0.03;
    }

    Eigen::MatrixXd mean_seq = warm_start_;

    Eigen::Vector3d control_cmd;
    int best_idx_global = -1;

    for (int iter = 0; iter < cfg_.mpc_iters; ++iter)
    {
        // 1. Sample K sequences around current mean
        std::vector<Eigen::MatrixXd> samples = sampleSequences(mean_seq, N, K);

        // 2. Rollout and compute costs
        Eigen::VectorXd costs(K);
        std::vector<std::vector<Eigen::Vector3d>> paths(K);
        std::vector<double> sample_min_dynamic_clearances;
        std::vector<double> sample_min_cpa_times;
        std::vector<bool> sample_corridor_feasible;
        rolloutBatch(lfpc_base, samples, goal_pos, obs_pos, obs_vel, obs_size, costs, paths,
                     sample_min_dynamic_clearances, sample_min_cpa_times, sample_corridor_feasible);

        // 3. Find best trajectory
        int valid_count = 0;
        for (int kk = 0; kk < K; ++kk)
        {
            if (std::isfinite(costs(kk)))
                valid_count++;
        }
        std::vector<double> cost_values(K, std::numeric_limits<double>::infinity());
        for (int kk = 0; kk < K; ++kk)
            cost_values[kk] = costs(kk);
        int best_idx = selectBestTrajectoryIndex(
            cost_values, sample_corridor_feasible,
            last_debug_metrics_.corridor_evaluated);
        bool has_corridor_feasible_sample = false;
        if (last_debug_metrics_.corridor_evaluated)
        {
            for (int kk = 0; kk < K; ++kk)
            {
                if (std::isfinite(costs(kk)) &&
                    kk < static_cast<int>(sample_corridor_feasible.size()) &&
                    sample_corridor_feasible[kk])
                {
                    has_corridor_feasible_sample = true;
                    break;
                }
            }
        }
        last_debug_metrics_.valid_sample_ratio = K > 0 ? (double)valid_count / (double)K : 0.0;
        last_debug_metrics_.best_total_cost = best_idx >= 0 ? costs(best_idx) : std::numeric_limits<double>::infinity();
        if (best_idx >= 0 && best_idx < (int)sample_min_dynamic_clearances.size())
            last_debug_metrics_.best_min_dynamic_clearance = sample_min_dynamic_clearances[best_idx];
        if (best_idx >= 0 && best_idx < (int)sample_min_cpa_times.size())
            last_debug_metrics_.best_min_cpa_time = sample_min_cpa_times[best_idx];

        if (cfg_.use_best && best_idx >= 0)
        {
            // Best-of-K: avoids MPPI mode collapse with multimodal obstacles
            control_cmd = samples[best_idx].row(0);
            mean_seq = samples[best_idx];
            best_path_ = paths[best_idx];
            best_idx_global = best_idx;
        }
        else
        {
            // MPPI weighted average
            Eigen::VectorXd weight_costs = costs;
            if (last_debug_metrics_.corridor_evaluated && has_corridor_feasible_sample)
            {
                for (int kk = 0; kk < K; ++kk)
                {
                    if (kk >= static_cast<int>(sample_corridor_feasible.size()) ||
                        !sample_corridor_feasible[kk])
                    {
                        weight_costs(kk) = std::numeric_limits<double>::infinity();
                    }
                }
            }
            Eigen::VectorXd weights = computeWeights(weight_costs);
            mean_seq = weightedUpdate(samples, weights, N);
            control_cmd = mean_seq.row(0);
            if (best_idx >= 0)
            {
                best_path_ = paths[best_idx];
                best_idx_global = best_idx;
            }
        }
    }

    // Shift warm-start for next cycle
    shiftSequence(mean_seq, N);
    warm_start_ = mean_seq;

    // No valid trajectory found (all INF) → full stop, wait for obstacle to pass
    if (best_idx_global < 0)
    {
        control_cmd << 0.0, 0.0, 0.0;
        last_plan_valid_ = false;
    }
    else
    {
        last_plan_valid_ = true;
    }

    auto t_end = std::chrono::high_resolution_clock::now();
    last_plan_time_ms_ = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    last_debug_metrics_.plan_time_ms = last_plan_time_ms_;
    last_debug_metrics_.plan_valid = last_plan_valid_;
    if (!last_plan_valid_)
    {
        last_debug_metrics_.best_min_dynamic_clearance = std::numeric_limits<double>::infinity();
        last_debug_metrics_.best_min_cpa_time = std::numeric_limits<double>::infinity();
    }

    return control_cmd;
}

// =========================================================================
// Pool management
// =========================================================================

void MpcController::ensurePool(int K)
{
    if ((int)lfpc_pool_.size() >= K)
        return;

    lfpc_pool_.clear();
    lfpc_pool_.reserve(K);
    for (int i = 0; i < K; ++i)
    {
        LFPC::Ptr lfpc = std::make_shared<LFPC>();
        lfpc_pool_.push_back(lfpc);
    }
    pool_initialized_ = true;
}

const ConvexCorridor::Segment* MpcController::nearestConvexSegment(const Eigen::Vector2d &point) const
{
    if (!has_convex_corridor_ || convex_corridor_segments_.empty())
        return nullptr;

    const ConvexCorridor::Segment* best = nullptr;
    double best_dist = std::numeric_limits<double>::infinity();
    for (const auto &seg : convex_corridor_segments_)
    {
        const double d = (point - seg.center).squaredNorm();
        if (d < best_dist)
        {
            best_dist = d;
            best = &seg;
        }
    }
    return best;
}

// =========================================================================
// Sampling
// =========================================================================

Eigen::MatrixXd MpcController::makeNominalSequence(int N)
{
    Eigen::MatrixXd seq(N, 3);
    seq.col(0).setConstant(cfg_.nominal_al);
    seq.col(1).setConstant(cfg_.nominal_aw);
    seq.col(2).setConstant(cfg_.nominal_api);
    return seq;
}

std::vector<Eigen::MatrixXd> MpcController::sampleSequences(
    const Eigen::MatrixXd &mean, int N, int K)
{
    std::vector<Eigen::MatrixXd> samples(K);
    double sa = cfg_.sigma_al;
    double sw = cfg_.sigma_aw;
    double sp = cfg_.sigma_api;

    for (int k = 0; k < K; ++k)
    {
        if (k == 0)
        {
            samples[k] = mean;
            continue;
        }

        samples[k] = Eigen::MatrixXd(N, 3);
        for (int n = 0; n < N; ++n)
        {
            double al, aw, api;
            if (cfg_.fix_step_params)
            {
                al  = cfg_.nominal_al;
                aw  = cfg_.nominal_aw;
                api = mean(n, 2) + sp * normal_dist_(rng_);
            }
            else
            {
                al  = mean(n, 0) + sa * normal_dist_(rng_);
                aw  = mean(n, 1) + sw * normal_dist_(rng_);
                api = mean(n, 2) + sp * normal_dist_(rng_);
            }

            // Clamp
            al  = std::max(cfg_.min_al, std::min(cfg_.max_al, al));
            aw  = std::max(0.0, std::min(cfg_.max_aw, aw));
            api = std::max(-cfg_.max_api, std::min(cfg_.max_api, api));

            samples[k](n, 0) = al;
            samples[k](n, 1) = aw;
            samples[k](n, 2) = api;
        }
    }
    return samples;
}

// =========================================================================
// Rollout (hot path)
// =========================================================================

void MpcController::rolloutBatch(
    const LFPC::Ptr &lfpc_base,
    const std::vector<Eigen::MatrixXd> &samples,
    const Eigen::Vector3d &goal_pos,
    const std::vector<Eigen::Vector3d> &obs_pos,
    const std::vector<Eigen::Vector3d> &obs_vel,
    const std::vector<Eigen::Vector3d> &obs_size,
    Eigen::VectorXd &costs,
    std::vector<std::vector<Eigen::Vector3d>> &paths,
    std::vector<double> &sample_min_dynamic_clearances,
    std::vector<double> &sample_min_cpa_times,
    std::vector<bool> &sample_corridor_feasible)
{
    int K = (int)samples.size();
    int N = cfg_.horizon_steps;
    costs.resize(K);
    costs.setConstant(std::numeric_limits<double>::infinity());
    sample_min_dynamic_clearances.assign(K, std::numeric_limits<double>::infinity());
    sample_min_cpa_times.assign(K, std::numeric_limits<double>::infinity());
    sample_corridor_feasible.assign(K, true);

    int n_obs = (int)obs_pos.size();
    last_debug_metrics_.dynamic_reject_count = 0;
    last_debug_metrics_.static_reject_count = 0;
    last_debug_metrics_.corridor_reject_count = 0;
    last_debug_metrics_.convex_corridor_reject_count = 0;
    last_debug_metrics_.convex_corridor_segments =
        has_convex_corridor_ ? static_cast<int>(convex_corridor_segments_.size()) : 0;
    last_debug_metrics_.max_convex_corridor_violation = 0.0;
    last_debug_metrics_.candidate_inside_corridor_ratio = 1.0;
    last_debug_metrics_.valid_trajectory_count = 0;
    last_debug_metrics_.corridor_feasible_trajectory_count = 0;
    last_debug_metrics_.corridor_evaluated = false;
    last_debug_metrics_.min_dynamic_clearance = std::numeric_limits<double>::infinity();
    last_debug_metrics_.min_cpa_time = std::numeric_limits<double>::infinity();
    int corridor_check_count = 0;
    int corridor_inside_count = 0;
    double w_move = cfg_.w_move;
    double w_steer = cfg_.w_steer;
    double w_dapi = cfg_.w_dapi;
    double w_goal = cfg_.w_goal;
    double w_prox = cfg_.w_prox;
    double w_convex_corridor = cfg_.w_convex_corridor;
    double prox_margin = cfg_.prox_margin;
    double dyn_margin = std::max(0.0, cfg_.dynamic_safety_margin);
    double dyn_min_radius = std::max(0.0, cfg_.dynamic_min_radius);
    double step_dt = lfpc_base->getTimeUpdate();
    double slice_h = collision_ ? collision_->getSliceHeight() : 0.0;

    // FOV limitation: robot's actual current position (step 0, not rollout)
    double base_x = lfpc_base->getCOMPos()(0);
    double base_y = lfpc_base->getCOMPos()(1);
    bool limit_fov = (cfg_.fov_range > 0.0);
    double fov_sq = cfg_.fov_range * cfg_.fov_range;

    for (int k = 0; k < K; ++k)
    {
        LFPC::Ptr lfpc = lfpc_pool_[k];
        lfpc->copyState(*lfpc_base);

        double prev_com_x = lfpc->getCOMPos()(0);
        double prev_com_y = lfpc->getCOMPos()(1);
        double prev_api = samples[k](0, 2);  // for steering rate penalty
        double total_cost = 0.0;
        bool static_collided = false;
        bool dyn_collided = false;
        bool corridor_violated = false;
        bool corridor_evaluated_for_sample = false;
        bool sample_corridor_feasible_for_sample = true;
        bool convex_corridor_violated = false;
        bool arrived_early = false;
        double rollout_min_dynamic_clearance = std::numeric_limits<double>::infinity();
        double rollout_min_cpa_time = std::numeric_limits<double>::infinity();

        for (int n = 0; n < N; ++n)
        {
            double al  = samples[k](n, 0);
            double aw  = samples[k](n, 1);
            double api = samples[k](n, 2);

            lfpc->SetCtrlParams(Eigen::Vector3d(al, aw, api));
            lfpc->updateOneStep();

            std::vector<Eigen::Vector3d> com_path = lfpc->getStepCOMPath();
            Eigen::Vector3d final_com = lfpc->getCOMPos();
            double fx = final_com(0);
            double fy = final_com(1);
            double path_dt = com_path.empty() ? step_dt : step_dt / (double)com_path.size();

            for (const auto &pt : com_path)
                paths[k].push_back(pt);

            // Move + steer cost
            double dx = fx - prev_com_x;
            double dy = fy - prev_com_y;
            double move_cost = w_move * std::sqrt(dx * dx + dy * dy);
            double steer_cost = w_steer * std::abs(api);

            // Steering rate penalty: penalize large api changes between steps
            double dapi_cost = 0.0;
            if (n > 0)
                dapi_cost = w_dapi * std::abs(api - prev_api);

            // ---- Foot placement: hard check (FOV-gated) ----
            Eigen::Vector2d foot_pos = lfpc->getFootPosition();
            bool foot_in_fov = true;
            if (limit_fov)
            {
                double fdx = foot_pos(0) - base_x;
                double fdy = foot_pos(1) - base_y;
                foot_in_fov = (fdx*fdx + fdy*fdy <= fov_sq);
            }
            if (collision_ && foot_in_fov && !collision_->isTraversable(foot_pos(0), foot_pos(1)))
            {
                static_collided = true;
                break;
            }

            // ---- CoM path: FOV-gated proximity cost + dynamic check ----
            double prox_cost = 0.0;
            double corridor_cost = 0.0;
            double convex_corridor_cost = 0.0;
            double step_min_dynamic_clearance = std::numeric_limits<double>::infinity();
            double step_min_cpa_time = std::numeric_limits<double>::infinity();
            for (size_t pi = 0; pi < com_path.size(); ++pi)
            {
                double px = com_path[pi](0);
                double py = com_path[pi](1);
                const double point_t = n * step_dt + (double)(pi + 1) * path_dt;
                const bool key_state_point = (pi + 1 == com_path.size());

                if (cfg_.corridor_enable && has_walking_corridor_)
                {
                    const Eigen::Vector2d point(px, py);
                    double outside = 0.0;
                    bool corridor_point_evaluated = false;
                    if (has_timed_walking_corridor_)
                    {
                        corridor_point_evaluated =
                            timed_walking_corridor_.segmentAtTime(point_t) != nullptr;
                        if (corridor_point_evaluated)
                            outside = timed_walking_corridor_.outsideDistanceAtTime(point_t, point);
                    }
                    else
                    {
                        corridor_point_evaluated = true;
                        outside = DynamicWalkingCorridor::outsideDistance(
                            walking_corridor_, point);
                    }
                    if (!corridor_point_evaluated)
                        continue;

                    corridor_evaluated_for_sample = true;
                    corridor_check_count++;
                    if (outside <= 0.0)
                        corridor_inside_count++;
                    if (key_state_point &&
                        !corridorDeviationFeasible(outside, cfg_.corridor_hard_margin))
                        sample_corridor_feasible_for_sample = false;
                    if (outside > 0.0)
                    {
                        corridor_cost += cfg_.w_corridor * outside * outside;
                        if (cfg_.corridor_hard_reject_enable &&
                            outside > cfg_.corridor_hard_margin)
                        {
                            corridor_violated = true;
                            break;
                        }
                    }
                    if (has_timed_walking_corridor_)
                    {
                        const double dyn_violation =
                            timed_walking_corridor_.dynamicObstacleViolationAtTime(
                                point_t, point);
                        if (dyn_violation > 0.0 && cfg_.w_corridor_dynamic > 0.0)
                        {
                            corridor_cost += cfg_.w_corridor_dynamic *
                                             dyn_violation * dyn_violation;
                            if (!corridorDeviationFeasible(
                                    dyn_violation, cfg_.corridor_hard_margin))
                            {
                                sample_corridor_feasible_for_sample = false;
                            }
                            if (cfg_.corridor_hard_reject_enable &&
                                !corridorDeviationFeasible(
                                    dyn_violation, cfg_.corridor_hard_margin))
                            {
                                corridor_violated = true;
                                break;
                            }
                        }
                    }
                }

                if (cfg_.convex_corridor_enable && has_convex_corridor_)
                {
                    const ConvexCorridor::Segment* seg =
                        nearestConvexSegment(Eigen::Vector2d(px, py));
                    if (seg)
                    {
                        const double v =
                            ConvexCorridor::violation(*seg, Eigen::Vector2d(px, py));
                        if (v > 0.0)
                        {
                            convex_corridor_cost += w_convex_corridor * v * v;
                            if (v > last_debug_metrics_.max_convex_corridor_violation)
                                last_debug_metrics_.max_convex_corridor_violation = v;
                            if (cfg_.convex_corridor_hard_reject_enable &&
                                v > cfg_.convex_corridor_hard_margin)
                            {
                                convex_corridor_violated = true;
                                break;
                            }
                        }
                    }
                }

                // FOV gate: beyond perception range → treat as traversable
                bool in_fov = true;
                if (limit_fov)
                {
                    double rdx = px - base_x;
                    double rdy = py - base_y;
                    in_fov = (rdx*rdx + rdy*rdy <= fov_sq);
                }

                if (in_fov)
                {
                    // Static obstacle: high penalty per colliding point
                    if (collision_ && !collision_->isTraversable(px, py))
                    {
                        static_collided = true;
                        break;
                    }
                    else if (collision_)
                    {
                        // SDF proximity gradient (only for non-colliding points)
                        Eigen::Vector3d pt(px, py, slice_h);
                        double dist = collision_->sdf_map_->getDistance(pt);
                        if (dist < prox_margin)
                        {
                            double d = prox_margin - dist;
                            prox_cost += w_prox * d * d;
                        }
                    }

                    // Dynamic obstacle: physical geometry only. Wider pedestrian
                    // keep-out is represented in the timed walking corridor.
                    for (int oi = 0; oi < n_obs; ++oi)
                    {
                        double vx = obs_vel[oi](0);
                        double vy = obs_vel[oi](1);
                        double ox = obs_pos[oi](0) + point_t * vx;
                        double oy = obs_pos[oi](1) + point_t * vy;
                        double dyn_radius = dyn_min_radius;
                        if (cfg_.use_dynamic_size && oi < (int)obs_size.size())
                            dyn_radius = std::max(dyn_radius, 0.5 * std::max(obs_size[oi](0), obs_size[oi](1)));
                        dyn_radius += dyn_margin;

                        double ddx = px - ox;
                        double ddy = py - oy;
                        double dyn_dist = std::sqrt(ddx * ddx + ddy * ddy);
                        double dyn_clearance = dyn_dist - dyn_radius;
                        if (dyn_clearance < step_min_dynamic_clearance)
                            step_min_dynamic_clearance = dyn_clearance;
                        if (dyn_clearance < rollout_min_dynamic_clearance)
                            rollout_min_dynamic_clearance = dyn_clearance;
                        if (dyn_clearance < last_debug_metrics_.min_dynamic_clearance)
                            last_debug_metrics_.min_dynamic_clearance = dyn_clearance;

                        if (cfg_.dynamic_collision_hard_reject_enable &&
                            ddx * ddx + ddy * ddy < dyn_radius * dyn_radius)
                        {
                            dyn_collided = true;
                            break;
                        }
                        double prev_px = (pi == 0) ? prev_com_x : com_path[pi - 1](0);
                        double prev_py = (pi == 0) ? prev_com_y : com_path[pi - 1](1);
                        double rvx = (px - prev_px) / std::max(1e-3, path_dt);
                        double rvy = (py - prev_py) / std::max(1e-3, path_dt);
                        double rel_vx = rvx - vx;
                        double rel_vy = rvy - vy;
                        double rel_speed_sq = rel_vx * rel_vx + rel_vy * rel_vy;
                        if (rel_speed_sq > 1e-6)
                        {
                            double t_cpa = -(ddx * rel_vx + ddy * rel_vy) / rel_speed_sq;
                            if (t_cpa >= 0.0 && t_cpa < step_min_cpa_time)
                                step_min_cpa_time = t_cpa;
                            if (t_cpa >= 0.0 && t_cpa < rollout_min_cpa_time)
                                rollout_min_cpa_time = t_cpa;
                            if (t_cpa >= 0.0 && t_cpa < last_debug_metrics_.min_cpa_time)
                                last_debug_metrics_.min_cpa_time = t_cpa;
                        }

                    }
                }
                // else: beyond FOV, skip all checks (optimistic)
                if (dyn_collided || static_collided || corridor_violated || convex_corridor_violated)
                    break;
            }

            if (dyn_collided || static_collided || corridor_violated || convex_corridor_violated)
            {
                total_cost = std::numeric_limits<double>::infinity();
                break;
            }

            total_cost += move_cost + steer_cost + dapi_cost + prox_cost +
                          corridor_cost + convex_corridor_cost;

            prev_com_x = fx;
            prev_com_y = fy;
            prev_api = api;

            // Early goal arrival: stop rollout if within threshold
            {
                double dxg = fx - goal_pos(0);
                double dyg = fy - goal_pos(1);
                if (std::sqrt(dxg*dxg + dyg*dyg) < cfg_.goal_arrival_threshold)
                {
                    arrived_early = true;
                    break;
                }
            }

            lfpc->prepareNextStep();
        }

        // Hard-inf any trajectory that touched a static or dynamic obstacle,
        // regardless of where the break happened (foot placement or CoM path).
        if (static_collided || dyn_collided || corridor_violated || convex_corridor_violated)
        {
            total_cost = std::numeric_limits<double>::infinity();
            if (dyn_collided)
                last_debug_metrics_.dynamic_reject_count++;
            if (static_collided)
                last_debug_metrics_.static_reject_count++;
            if (corridor_violated)
                last_debug_metrics_.corridor_reject_count++;
            if (convex_corridor_violated)
                last_debug_metrics_.convex_corridor_reject_count++;
        }
        else if (!arrived_early)
        {
            double gx = goal_pos(0);
            double gy = goal_pos(1);
            Eigen::Vector3d final_com = lfpc->getCOMPos();
            double fx = final_com(0);
            double fy = final_com(1);
            double dx = fx - gx;
            double dy = fy - gy;
            total_cost += (w_goal / N) * std::sqrt(dx * dx + dy * dy);
        }

        costs(k) = total_cost;
        sample_min_dynamic_clearances[k] = rollout_min_dynamic_clearance;
        sample_min_cpa_times[k] = rollout_min_cpa_time;
        sample_corridor_feasible[k] =
            !corridor_evaluated_for_sample || sample_corridor_feasible_for_sample;
        if (std::isfinite(total_cost))
        {
            last_debug_metrics_.valid_trajectory_count++;
            if (sample_corridor_feasible[k])
                last_debug_metrics_.corridor_feasible_trajectory_count++;
        }
    }
    if (corridor_check_count > 0)
    {
        last_debug_metrics_.corridor_evaluated = true;
        last_debug_metrics_.candidate_inside_corridor_ratio =
            static_cast<double>(corridor_inside_count) /
            static_cast<double>(corridor_check_count);
    }
}

// =========================================================================
// MPPI weight computation
// =========================================================================

Eigen::VectorXd MpcController::computeWeights(const Eigen::VectorXd &costs)
{
    int K = (int)costs.size();
    Eigen::VectorXd weights = Eigen::VectorXd::Zero(K);

    // Find min finite cost
    double min_cost = std::numeric_limits<double>::max();
    bool any_finite = false;
    for (int i = 0; i < K; ++i)
    {
        if (std::isfinite(costs(i)))
        {
            any_finite = true;
            if (costs(i) < min_cost)
                min_cost = costs(i);
        }
    }

    if (!any_finite)
    {
        weights.setConstant(1.0 / K);
        return weights;
    }

    double w_sum = 0.0;
    for (int i = 0; i < K; ++i)
    {
        if (std::isfinite(costs(i)))
        {
            weights(i) = std::exp(-(costs(i) - min_cost) / cfg_.temperature);
            w_sum += weights(i);
        }
    }

    if (w_sum < 1e-12)
    {
        weights.setConstant(1.0 / K);
        return weights;
    }

    weights /= w_sum;
    return weights;
}

Eigen::MatrixXd MpcController::weightedUpdate(
    const std::vector<Eigen::MatrixXd> &samples,
    const Eigen::VectorXd &weights,
    int N)
{
    Eigen::MatrixXd new_mean = Eigen::MatrixXd::Zero(N, 3);
    int K = (int)samples.size();

    for (int k = 0; k < K; ++k)
    {
        new_mean += weights(k) * samples[k];
    }

    // Clamp
    for (int n = 0; n < N; ++n)
    {
        new_mean(n, 0) = std::max(cfg_.min_al, std::min(cfg_.max_al, new_mean(n, 0)));
        new_mean(n, 1) = std::max(0.0, std::min(cfg_.max_aw, new_mean(n, 1)));
        new_mean(n, 2) = std::max(-cfg_.max_api, std::min(cfg_.max_api, new_mean(n, 2)));
    }

    return new_mean;
}

void MpcController::shiftSequence(Eigen::MatrixXd &mean, int N)
{
    // Roll by -1 (shift up)
    for (int n = 0; n < N - 1; ++n)
    {
        mean.row(n) = mean.row(n + 1);
    }
    // Set last row to nominal
    mean(N - 1, 0) = cfg_.nominal_al;
    mean(N - 1, 1) = cfg_.nominal_aw;
    mean(N - 1, 2) = cfg_.nominal_api;
}

} // namespace cane_planner
