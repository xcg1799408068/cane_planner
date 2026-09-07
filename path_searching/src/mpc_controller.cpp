#include <path_searching/mpc_controller.h>
#include <path_searching/trajectory_feasibility.h>
#include <cmath>
#include <algorithm>
#include <visualization_msgs/MarkerArray.h>
#include <chrono>

namespace cane_planner
{

MpcController::MpcController()
    : normal_dist_(0.0, 1.0)
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
    nh.param("mpc/sample_vis_enable", sample_vis_enable_, true);
    nh.param("mpc/sample_vis_count", sample_vis_count_, 20);
    sample_vis_count_ = std::max(0, std::min(sample_vis_count_, 50));
    sample_vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
        "/mpc/sample_trajectories", 1);
    publishSampleTrajectories({}, Eigen::VectorXd());

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

    nh.param("mpc/w_steer", cfg_.w_steer, 0.5);
    nh.param("mpc/w_goal", cfg_.w_goal, 10.0);
    nh.param("mpc/w_dapi", cfg_.w_dapi, 0.0);
    nh.param("mpc/use_best", cfg_.use_best, true);
    nh.param("mpc/fix_step_params", cfg_.fix_step_params, false);
    nh.param("mpc/goal_arrival_threshold", cfg_.goal_arrival_threshold, 0.3);

    nh.param("mpc/nominal_al", cfg_.nominal_al, 0.25);
    nh.param("mpc/nominal_aw", cfg_.nominal_aw, 0.0);
    nh.param("mpc/nominal_api", cfg_.nominal_api, 0.0);

}

void MpcController::init()
{
    reset();
}

void MpcController::reset()
{
    warm_start_.resize(0, 0);
    best_path_.clear();
    last_plan_valid_ = false;
    last_plan_time_ms_ = 0.0;
    last_debug_metrics_ = DebugMetrics();
    publishSampleTrajectories({}, Eigen::VectorXd());
}

void MpcController::resetWarmStart()
{
    warm_start_.resize(0, 0);  // forces fresh nominal init on next plan()
}

void MpcController::publishSampleTrajectories(
    const std::vector<std::vector<Eigen::Vector3d>> &paths,
    const Eigen::VectorXd &costs)
{
    if (!sample_vis_pub_)
        return;

    visualization_msgs::MarkerArray msg;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    // This dedicated topic owns its markers. Clear old IDs even if disabled,
    // reset, or publishing fewer samples than on the previous frame.
    marker.action = visualization_msgs::Marker::DELETEALL;
    msg.markers.push_back(marker);
    const int available = std::min(static_cast<int>(paths.size()),
                                   static_cast<int>(costs.size()));
    const int count = sample_vis_enable_ ? std::min(sample_vis_count_, available) : 0;
    marker.ns = "mpc_sample_trajectories";
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.015;
    marker.color.a = 0.65;
    marker.lifetime = ros::Duration(0.5);
    for (int i = 0; i < count; ++i)
    {
        // Uniform original indices, not top-cost ranking and no RNG draws.
        const int k = count > 1 ? static_cast<int>(
            static_cast<size_t>(i) * (available - 1) / (count - 1)) : 0;
        marker.id = k;
        const bool finite_cost = std::isfinite(costs(k));
        marker.color.r = finite_cost ? 0.0 : 1.0;
        marker.color.g = finite_cost ? 0.7 : 0.4;
        marker.color.b = finite_cost ? 1.0 : 0.0;
        marker.points.clear();
        // Copy only real computed CoM points. Rejected/early-arrival paths
        // may be shorter than the configured horizon; never extend them.
        for (const auto &pt : paths[k])
        {
            geometry_msgs::Point p;
            p.x = pt.x();
            p.y = pt.y();
            p.z = pt.z();
            marker.points.push_back(p);
        }
        if (marker.points.size() >= 2)
            msg.markers.push_back(marker);
    }
    sample_vis_pub_.publish(msg);
}

void MpcController::setCollision(const CollisionDetection::Ptr &col)
{
    collision_ = col;
}

void MpcController::setConvexCorridor(const std::vector<ConvexCorridor::Segment> &segments)
{
    convex_corridor_segments_ = segments;
    has_convex_corridor_ = !segments.empty() && std::all_of(
        segments.begin(), segments.end(), [](const ConvexCorridor::Segment &segment) {
            if (!segment.static_feasible || segment.halfspaces.size() < 3 ||
                !segment.center.allFinite())
                return false;
            return std::all_of(segment.halfspaces.begin(), segment.halfspaces.end(),
                [](const ConvexCorridor::Halfspace &h) {
                    return h.normal.allFinite() && h.normal.norm() > 1e-9 &&
                           std::isfinite(h.offset);
                });
        });
}

void MpcController::clearConvexCorridor()
{
    has_convex_corridor_ = false;
    convex_corridor_segments_.clear();
}

// =========================================================================
// Main API
// =========================================================================

Eigen::Vector3d MpcController::plan(const LFPC::Ptr &lfpc_base,
                                     const Eigen::Vector3d &goal_pos)
{
    auto t_start = std::chrono::high_resolution_clock::now();

    int N = cfg_.horizon_steps;
    int K = cfg_.num_samples;
    last_debug_metrics_ = DebugMetrics();
    last_debug_metrics_.num_samples = K;
    const bool initial_inside = lfpc_base && lfpc_base->getCOMPos().allFinite() &&
        std::any_of(convex_corridor_segments_.begin(), convex_corridor_segments_.end(),
            [&lfpc_base](const ConvexCorridor::Segment &segment) {
                return ConvexCorridor::contains(segment, lfpc_base->getCOMPos().head(2), 1e-6);
            });
    const bool initial_free = initial_inside && collision_ &&
        (collision_->hasStaticGlobalMap()
            ? collision_->isStaticTraversable(lfpc_base->getCOMPos().x(), lfpc_base->getCOMPos().y())
            : collision_->isTraversable(lfpc_base->getCOMPos().x(), lfpc_base->getCOMPos().y()));
    if (!has_convex_corridor_ || !initial_free || !collision_ || !lfpc_base ||
        !goal_pos.allFinite() || N <= 0 || K <= 0 || cfg_.mpc_iters <= 0 ||
        !std::isfinite(cfg_.temperature) || cfg_.temperature <= 0.0)
    {
        best_path_.clear();
        warm_start_.resize(0, 0);
        last_plan_valid_ = false;
        last_debug_metrics_.plan_valid = false;
        last_plan_time_ms_ = std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now() - t_start).count();
        last_debug_metrics_.plan_time_ms = last_plan_time_ms_;
        publishSampleTrajectories({}, Eigen::VectorXd());
        return Eigen::Vector3d::Zero();
    }

    ensurePool(K);

    // Initialize or shift warm-start
    if (warm_start_.rows() != N)
    {
        warm_start_ = makeNominalSequence(N);
        // Tiny random bias on first control to break left-right symmetry
        warm_start_(0, 2) += normal_dist_(rng_) * 0.03;
    }

    Eigen::MatrixXd mean_seq = warm_start_;

    Eigen::Vector3d control_cmd = Eigen::Vector3d::Zero();
    int best_idx_global = -1;
    best_path_.clear();

    for (int iter = 0; iter < cfg_.mpc_iters; ++iter)
    {
        // Validity must describe this iteration, never an earlier proposal.
        best_idx_global = -1;
        best_path_.clear();
        last_debug_metrics_.weighted_checked = false;
        last_debug_metrics_.weighted_fallback = false;
        // 1. Sample K sequences around current mean
        std::vector<Eigen::MatrixXd> samples = sampleSequences(mean_seq, N, K);

        // 2. Rollout and compute costs
        Eigen::VectorXd costs(K);
        std::vector<std::vector<Eigen::Vector3d>> paths(K);
        std::vector<bool> sample_corridor_feasible;
        rolloutBatch(lfpc_base, samples, goal_pos, costs, paths,
                     sample_corridor_feasible);

        // Publish only the final iteration, including all-rejected/STOP frames.
        if (iter + 1 == cfg_.mpc_iters)
            publishSampleTrajectories(paths, costs);

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
        last_debug_metrics_.executed_total_cost = last_debug_metrics_.best_total_cost;

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
            if (best_idx >= 0)
            {
                // A weighted control sequence is not necessarily collision-free,
                // even when every contributing sample is. Reuse the exact rollout
                // evaluator without drawing samples or changing candidate metrics.
                const DebugMetrics candidate_metrics = last_debug_metrics_;
                Eigen::VectorXd checked_costs;
                std::vector<std::vector<Eigen::Vector3d>> checked_paths(1);
                std::vector<bool> checked_corridor_feasible;
                rolloutBatch(lfpc_base, {mean_seq}, goal_pos,
                             checked_costs, checked_paths, checked_corridor_feasible);
                last_debug_metrics_ = candidate_metrics;
                const bool weighted_valid = chooseFinalSequence(
                    has_convex_corridor_, best_idx, cfg_.use_best, checked_costs(0),
                    !candidate_metrics.corridor_evaluated || checked_corridor_feasible[0])
                    == FinalSequenceChoice::WEIGHTED;
                last_debug_metrics_.weighted_checked = true;
                last_debug_metrics_.weighted_fallback = !weighted_valid;
                if (weighted_valid)
                {
                    best_path_ = checked_paths[0];
                    last_debug_metrics_.executed_total_cost = checked_costs(0);
                }
                else
                {
                    mean_seq = samples[best_idx];
                    best_path_ = paths[best_idx];
                }
                control_cmd = mean_seq.row(0);
                best_idx_global = best_idx;
            }
        }
    }

    // Shift warm-start for next cycle
    shiftSequence(mean_seq, N);
    warm_start_ = mean_seq;

    // No valid trajectory found (all INF) → full stop, wait for obstacle to pass
    if (chooseFinalSequence(has_convex_corridor_, best_idx_global, true,
                            std::numeric_limits<double>::infinity(), false)
        == FinalSequenceChoice::STOP)
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
        best_path_.clear();
        warm_start_.resize(0, 0);
        last_debug_metrics_.executed_total_cost = std::numeric_limits<double>::infinity();
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
    Eigen::VectorXd &costs,
    std::vector<std::vector<Eigen::Vector3d>> &paths,
    std::vector<bool> &sample_corridor_feasible)
{
    int K = (int)samples.size();
    int N = cfg_.horizon_steps;
    costs.resize(K);
    costs.setConstant(std::numeric_limits<double>::infinity());
    sample_corridor_feasible.assign(K, true);

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
    double w_steer = cfg_.w_steer;
    double w_dapi = cfg_.w_dapi;
    double w_goal = cfg_.w_goal;

    for (int k = 0; k < K; ++k)
    {
        LFPC::Ptr lfpc = lfpc_pool_[k];
        lfpc->copyState(*lfpc_base);

        double prev_api = samples[k](0, 2);  // for steering rate penalty
        double total_cost = 0.0;
        bool static_collided = false;
        bool corridor_evaluated_for_sample = false;
        bool sample_corridor_feasible_for_sample = true;
        bool convex_corridor_violated = false;
        bool arrived_early = false;

        for (int n = 0; n < N; ++n)
        {
            double al  = samples[k](n, 0);
            double aw  = samples[k](n, 1);
            double api = samples[k](n, 2);

            lfpc->SetCtrlParams(Eigen::Vector3d(al, aw, api));
            lfpc->updateOneStep();

            std::vector<Eigen::Vector3d> com_path = lfpc->getStepCOMPath();
            Eigen::Vector3d final_com = lfpc->getCOMPos();
            if (com_path.empty() || !final_com.allFinite())
            {
                static_collided = true;
                break;
            }
            double fx = final_com(0);
            double fy = final_com(1);

            for (const auto &pt : com_path)
                paths[k].push_back(pt);

            // Steering magnitude and inter-step steering variation.
            double steer_cost = w_steer * std::abs(api);

            // Steering rate penalty: penalize large api changes between steps
            double dapi_cost = 0.0;
            if (n > 0)
                dapi_cost = w_dapi * std::abs(api - prev_api);

            // Static foot placement uses the map's existing safety boundary.
            Eigen::Vector2d foot_pos = lfpc->getFootPosition();
            const bool foot_free = foot_pos.allFinite() && (collision_->hasStaticGlobalMap()
                ? collision_->isStaticTraversable(foot_pos.x(), foot_pos.y())
                : collision_->isTraversable(foot_pos.x(), foot_pos.y()));
            if (!foot_pos.allFinite() || !foot_free)
            {
                static_collided = true;
                break;
            }

            // Static CoM samples are checked against the same spatial union.
            for (size_t pi = 0; pi < com_path.size(); ++pi)
            {
                double px = com_path[pi](0);
                double py = com_path[pi](1);

                // Membership in the connected spatial union, independent of
                // timing and nearest-centre heuristics. No soft corridor cost.
                const Eigen::Vector2d point(px, py);
                const bool inside = point.allFinite() && std::any_of(
                    convex_corridor_segments_.begin(), convex_corridor_segments_.end(),
                    [&point](const ConvexCorridor::Segment &segment) {
                        return ConvexCorridor::contains(segment, point, 1e-6);
                    });
                corridor_evaluated_for_sample = true;
                ++corridor_check_count;
                if (!inside)
                {
                    double union_violation = std::numeric_limits<double>::infinity();
                    for (const auto &segment : convex_corridor_segments_)
                        union_violation = std::min(union_violation,
                            ConvexCorridor::violation(segment, point));
                    last_debug_metrics_.max_convex_corridor_violation = std::max(
                        last_debug_metrics_.max_convex_corridor_violation, union_violation);
                    sample_corridor_feasible_for_sample = false;
                    convex_corridor_violated = true;
                    break;
                }
                ++corridor_inside_count;

                const bool com_free = collision_->hasStaticGlobalMap()
                    ? collision_->isStaticTraversable(px, py)
                    : collision_->isTraversable(px, py);
                if (!com_free)
                {
                    static_collided = true;
                    break;
                }
                if (static_collided || convex_corridor_violated)
                    break;
            }

            if (static_collided || convex_corridor_violated)
            {
                total_cost = std::numeric_limits<double>::infinity();
                break;
            }

            total_cost += steer_cost + dapi_cost;

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

        // Hard-inf any trajectory that touched a static obstacle or left the union,
        // regardless of where the break happened (foot placement or CoM path).
        if (static_collided || convex_corridor_violated)
        {
            total_cost = std::numeric_limits<double>::infinity();
            if (static_collided)
                last_debug_metrics_.static_reject_count++;
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
