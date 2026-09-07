#include <path_searching/astar.h>
#include <limits>
#include <sstream>

using namespace std;
using namespace Eigen;

namespace cane_planner
{
  Astar::~Astar()
  {
    for (int i = 0; i < allocate_num_; i++)
    {
      delete path_node_pool_[i];
    }
  }

  bool Astar::corridorEdgeFree(const Eigen::Vector2d& a, const Eigen::Vector2d& b)
  {
    if (!collision_ || !a.allFinite() || !b.allFinite() ||
        !std::isfinite(corridor_edge_clearance_) || corridor_edge_clearance_ < 0.) return false;
    const double clearance = corridor_edge_clearance_;
    // Expand by a numerical guard only to enumerate both bins at exact grid
    // boundaries. Collision distances themselves have no permissive tolerance.
    const Eigen::Vector2d padding = Eigen::Vector2d::Constant(clearance + 1e-9);
    const Eigen::Vector2d lower = a.cwiseMin(b) - padding;
    const Eigen::Vector2d upper = a.cwiseMax(b) + padding;
    Eigen::Vector2d origin; Eigen::Vector2i size; double resolution;
    std::vector<uint8_t> blocked; std::string reason;
    // Candidate edges are one A* step: enumerate only their local native bins,
    // not a complete map or point samples along the edge.
    if (!collision_->getStaticCorridorGrid(lower, upper, origin, resolution, size, blocked, reason)) return false;
    const Eigen::Vector2d extent = origin + resolution * size.cast<double>();
    if ((a.cwiseMin(b).array() - clearance <= origin.array()).any() ||
        (a.cwiseMax(b).array() + clearance >= extent.array()).any()) return false;
    const Eigen::Vector2d d = b-a;
    auto point_segment_distance = [](const Eigen::Vector2d& p, const Eigen::Vector2d& u, const Eigen::Vector2d& v) {
      const Eigen::Vector2d e = v-u;
      const double t = e.squaredNorm() > 0. ? std::max(0., std::min(1., (p-u).dot(e)/e.squaredNorm())) : 0.;
      return (p-u-t*e).norm();
    };
    for (int y=0; y<size.y(); ++y) for (int x=0; x<size.x(); ++x) {
      if (!blocked[y*size.x()+x]) continue;
      const Eigen::Vector2d lo = origin + resolution*Eigen::Vector2d(x,y);
      const Eigen::Vector2d hi = lo + Eigen::Vector2d::Constant(resolution);
      double enter=0., leave=1.; bool intersects=true;
      for (int axis=0; axis<2; ++axis) {
        if (d[axis]==0.) { if (a[axis]<lo[axis] || a[axis]>hi[axis]) intersects=false; }
        else {
          double t0=(lo[axis]-a[axis])/d[axis], t1=(hi[axis]-a[axis])/d[axis];
          if (t0>t1) std::swap(t0,t1);
          enter=std::max(enter,t0); leave=std::min(leave,t1);
        }
      }
      if (intersects && enter<=leave) return false;
      double distance=std::min((a-a.cwiseMax(lo).cwiseMin(hi)).norm(),
                               (b-b.cwiseMax(lo).cwiseMin(hi)).norm());
      for (int i=0;i<2;++i) for(int j=0;j<2;++j)
        distance=std::min(distance,point_segment_distance(Eigen::Vector2d(i?hi.x():lo.x(),j?hi.y():lo.y()),a,b));
      if (distance < clearance) return false;
    }
    return true;
  }

  double Astar::corridorEdgeCost(const Eigen::Vector2d& a, const Eigen::Vector2d& b)
  {
    const double length = (b-a).norm();
    if (w_clearance_ == 0. || length == 0.) return length;
    const double range = clearance_sigma_ + corridor_edge_clearance_;
    const Eigen::Vector2d padding = Eigen::Vector2d::Constant(range + 1e-9);
    Eigen::Vector2d origin; Eigen::Vector2i size; double resolution;
    std::vector<uint8_t> blocked; std::string reason;
    // Only the edge's finite preference neighborhood, using the SAME classifier
    // as the hard certificate. Raw ESDF distances have different semantics.
    if (!collision_->getStaticCorridorGrid(a.cwiseMin(b)-padding, a.cwiseMax(b)+padding,
        origin, resolution, size, blocked, reason)) return std::numeric_limits<double>::infinity();
    const Eigen::Vector2d extent = origin + resolution*size.cast<double>();
    std::vector<Eigen::Vector2d> cells;
    for (int y=0; y<size.y(); ++y) for (int x=0; x<size.x(); ++x)
      if (blocked[y*size.x()+x]) cells.push_back(origin+resolution*Eigen::Vector2d(x,y));

    // Composite midpoint quadrature over the COMPLETE edge, not just its end.
    // Step <= 2 cm, half a native bin and 1/16 of the soft range. This is a
    // comfort-cost approximation, never the continuous hard collision check.
    const double step = std::min(.02, std::min(.5*resolution, clearance_sigma_/16.));
    const int count = std::max(1, static_cast<int>(std::ceil(length/step)));
    double penalty = 0.;
    for (int i=0; i<count; ++i) {
      const Eigen::Vector2d p = a+((i+.5)/count)*(b-a);
      // A cropped side is >= range away unless it is the real map boundary;
      // hence taking its distance cannot change the truncated penalty.
      double distance = std::min(range, std::min((p-origin).minCoeff(), (extent-p).minCoeff()));
      for (const auto& lo : cells) {
        const Eigen::Vector2d hi = lo+Eigen::Vector2d::Constant(resolution);
        distance = std::min(distance, (p-p.cwiseMax(lo).cwiseMin(hi)).norm());
      }
      const double additional = std::max(0., distance-corridor_edge_clearance_);
      const double t = std::max(0., 1.-additional/clearance_sigma_);
      penalty += t*t;
    }
    return length*(1.+w_clearance_*penalty/count);
  }

  double Astar::corridorHeuristic(const Eigen::Vector2d& a, const Eigen::Vector2d& b) const
  {
    // Euclidean metres remain a lower bound with an off-lattice goal connector.
    // No multiplicative tie inflation. lambda > 1 is weighted, not optimal A*.
    return lambda_heu_*(b-a).norm();
  }

  bool Astar::search(Eigen::Vector2d start_pt, Eigen::Vector2d end_pt, bool dynamic, double time_start)
  {
    append_goal_ = false;
    const bool corridor_edges = corridor_edge_clearance_ != -1.0;
    if (!start_pt.allFinite() || !end_pt.allFinite() || !collision_ ||
        !std::isfinite(resolution_) || resolution_ <= 0. || path_node_pool_.empty()) return false;
    if (corridor_edges && (!std::isfinite(w_clearance_) || w_clearance_ < 0. ||
        !std::isfinite(clearance_sigma_) || clearance_sigma_ < .01 || clearance_sigma_ > 5. ||
        !std::isfinite(lambda_heu_) || lambda_heu_ < 0.)) {
      ROS_WARN("[Astar] Invalid clearance preference or heuristic configuration");
      return false;
    }
    if (corridor_edges && (!corridorEdgeFree(start_pt,start_pt) || !corridorEdgeFree(end_pt,end_pt))) return false;
    if (corridor_edges && w_clearance_ > 0.) {
      // Endpoint checks established an available, valid selected native grid.
      // A terminal connector can span 1.5 search steps per axis (nearest goal
      // index plus one neighbor). Bound its uncropped, aligned preference query
      // before expanding any nodes; do not turn GRID_BUDGET into no-path/INF.
      const double native = collision_->getStaticQueryResolution();
      const double span = 1.5*resolution_ + 2.*(clearance_sigma_+corridor_edge_clearance_+1e-9);
      const double bins = std::ceil(span/native) + 1.;
      if (!std::isfinite(bins) || bins > 1000. || bins*native > 100.) {
        ROS_WARN("[Astar] UNSUPPORTED_PREFERENCE_GRID: sigma=%.6g clearance=%.6g search_resolution=%.6g native_resolution=%.6g requires up to %.0f bins/axis; adapter limits 1000000 bins and 100 m/axis",
                 clearance_sigma_, corridor_edge_clearance_, resolution_, native, bins);
        return false;
      }
    }
    const bool use_static = (corridor_edges || use_static_global_map_) && collision_->hasStaticGlobalMap();
    auto in_astar_bounds = [this](const Eigen::Vector2d &pt) {
      return pt(0) > origin_(0) && pt(0) < map_max_2d_(0) &&
             pt(1) > origin_(1) && pt(1) < map_max_2d_(1);
    };
    if (use_static)
    {
      bool start_free = collision_->isStaticTraversable(start_pt(0), start_pt(1));
      bool end_free = collision_->isStaticTraversable(end_pt(0), end_pt(1));
      const int start_near_free = collision_->countStaticTraversableAround(start_pt, 0.5);
      const int end_near_free = collision_->countStaticTraversableAround(end_pt, 0.5);
      ROS_WARN_THROTTLE(1.0,
                        "[Astar static] bounds=[%.2f %.2f]x[%.2f %.2f], start=(%.2f, %.2f) in=%d free=%d near_free=%d dist=%.2f, goal=(%.2f, %.2f) in=%d free=%d near_free=%d dist=%.2f",
                        origin_(0), map_max_2d_(0), origin_(1), map_max_2d_(1),
                        start_pt(0), start_pt(1), in_astar_bounds(start_pt), start_free, start_near_free,
                        collision_->getStaticCollisionDistance(start_pt),
                        end_pt(0), end_pt(1), in_astar_bounds(end_pt), end_free, end_near_free,
                        collision_->getStaticCollisionDistance(end_pt));
    }
    else
    {
      auto count_near_traversable = [this](const Eigen::Vector2d &pt, double radius) {
        const int step = std::max(1, static_cast<int>(std::ceil(radius * inv_resolution_)));
        int count = 0;
        for (int dx = -step; dx <= step; ++dx)
        {
          for (int dy = -step; dy <= step; ++dy)
          {
            if (std::hypot(dx, dy) * resolution_ > radius)
              continue;
            const double x = pt(0) + dx * resolution_;
            const double y = pt(1) + dy * resolution_;
            if (x <= origin_(0) || x >= map_max_2d_(0) ||
                y <= origin_(1) || y >= map_max_2d_(1))
              continue;
            if (collision_->isTraversable(x, y))
              count++;
          }
        }
        return count;
      };
      const bool start_free = collision_->isTraversable(start_pt(0), start_pt(1));
      const bool end_free = collision_->isTraversable(end_pt(0), end_pt(1));
      ROS_WARN_THROTTLE(1.0,
                        "[Astar sdf] bounds=[%.2f %.2f]x[%.2f %.2f], start=(%.2f, %.2f) in=%d free=%d near_free=%d dist=%.2f, goal=(%.2f, %.2f) in=%d free=%d near_free=%d dist=%.2f",
                        origin_(0), map_max_2d_(0), origin_(1), map_max_2d_(1),
                        start_pt(0), start_pt(1), in_astar_bounds(start_pt), start_free,
                        count_near_traversable(start_pt, 0.5),
                        collision_->getCollisionDistance(start_pt),
                        end_pt(0), end_pt(1), in_astar_bounds(end_pt), end_free,
                        count_near_traversable(end_pt, 0.5),
                        collision_->getCollisionDistance(end_pt));
    }

    if (!in_astar_bounds(start_pt) || !in_astar_bounds(end_pt))
    {
      ROS_WARN("[Astar] Rejecting out-of-bounds endpoint: start=(%.2f, %.2f) in=%d, goal=(%.2f, %.2f) in=%d",
               start_pt(0), start_pt(1), in_astar_bounds(start_pt),
               end_pt(0), end_pt(1), in_astar_bounds(end_pt));
      return false;
    }

    const bool start_traversable = use_static ?
        collision_->isStaticTraversable(start_pt(0), start_pt(1)) :
        collision_->isTraversable(start_pt(0), start_pt(1));
    const bool end_traversable = use_static ?
        collision_->isStaticTraversable(end_pt(0), end_pt(1)) :
        collision_->isTraversable(end_pt(0), end_pt(1));
    if (!start_traversable || !end_traversable)
    {
      ROS_WARN("[Astar] Rejecting occupied endpoint: start_free=%d goal_free=%d",
               start_traversable, end_traversable);
      return false;
    }

    // Immutable queue entries are essential: mutating a Node priority in the
    // legacy pointer heap does not restore heap order. Keep legacy callers intact.
    struct Entry {
      NodePtr node; double f, g; size_t order;
      bool operator<(const Entry& other) const {
        if (f != other.f) return f > other.f;
        if (g != other.g) return g < other.g;
        return order > other.order;
      }
    };
    std::priority_queue<Entry> corridor_open;
    size_t queue_order = 0;
    auto push_open = [&](NodePtr node) {
      if (corridor_edges) corridor_open.push({node,node->f_score,node->g_score,queue_order++});
      else open_set_.push(node);
    };
    auto finish = [&](NodePtr node) {
      if (corridor_edges && (node->position-end_pt).squaredNorm() <= 1e-20 &&
          (!node->parent || corridorEdgeFree(node->parent->position,end_pt)))
        node->position = end_pt; // Collapse numerical duplicates only after recertifying the edge.
      retrievePath(node);
      append_goal_ = corridor_edges && (node->position-end_pt).squaredNorm() > 0.;
      exact_goal_ = end_pt;
      has_path_ = true;
      return true;
    };
    NodePtr goal_parent = nullptr;
    double goal_cost = std::numeric_limits<double>::infinity();

    /* ---------- initialize ---------- */
    NodePtr cur_node = path_node_pool_[0];
    cur_node->parent = NULL;
    cur_node->position = start_pt;
    cur_node->index = posToIndex(start_pt);
    cur_node->g_score = 0.0;

    // Eigen::Vector2d end_state(6);
    Eigen::Vector2i end_index;
    // double time_to_goal;

    const Eigen::Vector2i start_index = cur_node->index;
    end_index = corridor_edges ? (start_index +
        ((end_pt-start_pt)*inv_resolution_).array().round().cast<int>().matrix()).eval() : posToIndex(end_pt);
    cur_node->f_score = corridor_edges ? corridorHeuristic(start_pt,end_pt) :
        lambda_heu_ * getEuclHeu(cur_node->position, end_pt);
    cur_node->node_state = IN_OPEN_SET;

    push_open(cur_node);
    use_node_num_ += 1;

    if (dynamic)
    {
      time_origin_ = time_start;
      cur_node->time = time_start;
      cur_node->time_idx = timeToIndex(time_start);
      expanded_nodes_.insert(cur_node->index, cur_node->time_idx, cur_node);
      // cout << "time start: " << time_start << endl;
    }
    else
      expanded_nodes_.insert(cur_node->index, cur_node);

    // NodePtr neighbor = NULL;
    NodePtr terminate_node = NULL;
    Eigen::Vector2d expanded_min = start_pt;
    Eigen::Vector2d expanded_max = start_pt;
    Eigen::Vector2d closest_to_goal = start_pt;
    double closest_goal_dist = (start_pt - end_pt).norm();
    int reject_bounds = 0;
    int reject_closed = 0;
    int reject_collision = 0;

    /* ---------- search loop ---------- */
    while (corridor_edges ? !corridor_open.empty() : !open_set_.empty())
    {
      /* ---------- get lowest f_score node ---------- */
      if (corridor_edges) {
        const Entry entry = corridor_open.top();
        cur_node = entry.node;
        if (cur_node->node_state == IN_CLOSE_SET || entry.g != cur_node->g_score) {
          corridor_open.pop();
          continue;
        }
        // The exact goal is a virtual node whose connector has the SAME cost.
        if (goal_parent && goal_cost <= entry.f) return finish(goal_parent);
      } else cur_node = open_set_.top();
      // cout << "pos: " << cur_node->position.transpose() << endl;
      // cout << "time: " << cur_node->time << endl;
      // cout << "dist: " <<
      // edt_environment_->evaluateCoarseEDT(cur_node->state.head(3),
      // cur_node->time) <<
      // endl;

      /* ---------- determine termination ---------- */

      bool reach_end = abs(cur_node->index(0) - end_index(0)) <= 1 &&
                       abs(cur_node->index(1) - end_index(1)) <= 1;
      double reach_horizon = (cur_node->position - start_pt).norm();

      if (reach_end && (!corridor_edges || corridorEdgeFree(cur_node->position, end_pt)))
      {
        if (!corridor_edges) return finish(cur_node);
        const double candidate = cur_node->g_score + corridorEdgeCost(cur_node->position,end_pt);
        if (candidate < goal_cost) { goal_cost = candidate; goal_parent = cur_node; }
        if (goal_parent && goal_cost <= cur_node->f_score) return finish(goal_parent);
      }
      // horizon 为最大搜索距离, 如果到达了最大搜索距离, 但是还是没有找到路径, 则认为没有路径
      if (reach_horizon >= horizon_)
      {
        // Horizon limits further search, not an already certified exact-goal
        // connector. Prefer the best complete candidate discovered so far;
        // horizon termination does not assert objective optimality.
        if (corridor_edges && goal_parent) return finish(goal_parent);
        double cur_near_end = (cur_node->position - end_pt).norm();
        double start_near_end = (start_pt - end_pt).norm();
        if (cur_near_end <= start_near_end)
        {
          std::cout << "[Astar](horizon):---------------------- " << use_node_num_ << std::endl;
          std::cout << use_node_num_ << "," << iter_num_ << ",";
          terminate_node = cur_node;

          retrievePath(terminate_node);
          has_path_ = true;

          return true;
        }
        else
        {
          std::cout << "[Astar](horizon):---------------------- " << use_node_num_ << std::endl;
          std::cout << "!---in horizion no find path--" << std::endl;
        }
      }
      /* ---------- pop node and add to close set ---------- */
      if (corridor_edges) corridor_open.pop(); else open_set_.pop();
      cur_node->node_state = IN_CLOSE_SET;
      iter_num_ += 1;
      expanded_min = expanded_min.cwiseMin(cur_node->position);
      expanded_max = expanded_max.cwiseMax(cur_node->position);
      const double cur_goal_dist = (cur_node->position - end_pt).norm();
      if (cur_goal_dist < closest_goal_dist)
      {
        closest_goal_dist = cur_goal_dist;
        closest_to_goal = cur_node->position;
      }

      /* ---------- init neighbor expansion ---------- */

      Eigen::Vector2d cur_pos = cur_node->position;
      Eigen::Vector2d pro_pos;
      double pro_t;

      vector<Eigen::Vector2d> inputs;
      Eigen::Vector2d d_pos;

      /* ---------- expansion loop ---------- */
      for (double dx = -resolution_; dx <= resolution_ + 1e-3; dx += resolution_)
        for (double dy = -resolution_; dy <= resolution_ + 1e-3; dy += resolution_)
        {
          d_pos << dx, dy;

          if (d_pos.norm() < 1e-3)
            continue;

          // Search states form a lattice anchored at the actual start, not at
          // native-map bin boundaries. Integer offsets prevent floor roundoff
          // from merging adjacent states at finer search resolutions.
          Eigen::Vector2i pro_id = corridor_edges ? (cur_node->index +
              Eigen::Vector2i(static_cast<int>(std::lround(dx*inv_resolution_)),
                              static_cast<int>(std::lround(dy*inv_resolution_)))).eval() : posToIndex(cur_pos+d_pos);
          pro_pos = corridor_edges ? (start_pt + resolution_*(pro_id-start_index).cast<double>()).eval() : cur_pos+d_pos;

          /* ---------- check if in feasible space ---------- */
          /* inside map range */
          if (pro_pos(0) <= origin_(0) || pro_pos(0) >= map_max_2d_(0) || pro_pos(1) <= origin_(1) || pro_pos(1) >= map_max_2d_(1))
          {
            // cout << "outside map" << endl;
            reject_bounds++;
            continue;
          }

          /* not in close set */
          pro_t = dynamic ? cur_node->time + 1.0 : 0.0;
          int pro_t_id = dynamic ? timeToIndex(pro_t) : 0;
          NodePtr pro_node =
              dynamic ? expanded_nodes_.find(pro_id, pro_t_id) : expanded_nodes_.find(pro_id);
          if (!corridor_edges && pro_node != NULL && pro_node->node_state == IN_CLOSE_SET)
          {
            // cout << "in closeset" << endl;
            reject_closed++;
            continue;
          }
          Eigen::Vector3d pro_pos_3d;
          pro_pos_3d << pro_pos(0), pro_pos(1), collision_->getSliceHeight(); // 设置路径高度
          /* collision free: the collision layer owns the static safety boundary. */
          const bool traversable = use_static ?
              collision_->isStaticTraversable(pro_pos(0), pro_pos(1)) :
              collision_->isTraversable(pro_pos(0), pro_pos(1));
          if (!traversable || (corridor_edges && !corridorEdgeFree(cur_pos, pro_pos)))
          // if (!collision_->isTraversable(pro_pos_3d))
          {
            // cout << "Can't Traversable" << endl;
            reject_collision++;
            continue;
          }

          /* ---------- compute cost ---------- */
          double tmp_g_score, tmp_f_score;
          tmp_g_score = cur_node->g_score + (corridor_edges ?
              corridorEdgeCost(cur_pos, pro_pos) : d_pos.squaredNorm());
          if (!std::isfinite(tmp_g_score)) continue;

          // Preserve the opt-in legacy objective for non-corridor callers.
          if (!corridor_edges && w_clearance_ > 1e-6)
          {
            const double sdf_dist = use_static ?
                collision_->getStaticCollisionDistance(pro_pos) :
                collision_->getCollisionDistance(pro_pos);
            if (sdf_dist < clearance_sigma_)
            {
              double t = 1.0 - sdf_dist / clearance_sigma_;
              tmp_g_score += w_clearance_ * d_pos.norm() * t * t;
            }
          }
          tmp_f_score = tmp_g_score + (corridor_edges ? corridorHeuristic(pro_pos,end_pt) :
              lambda_heu_ * getDiagHeu(pro_pos, end_pt));

          if (pro_node == NULL)
          {
            pro_node = path_node_pool_[use_node_num_];
            pro_node->index = pro_id;
            pro_node->position = pro_pos;
            pro_node->f_score = tmp_f_score;
            pro_node->g_score = tmp_g_score;
            pro_node->parent = cur_node;
            pro_node->node_state = IN_OPEN_SET;
            if (dynamic)
            {
              pro_node->time = pro_t;
              pro_node->time_idx = timeToIndex(pro_node->time);
            }
            push_open(pro_node);

            if (dynamic)
              expanded_nodes_.insert(pro_id, pro_node->time_idx, pro_node);
            else
              expanded_nodes_.insert(pro_id, pro_node);

            use_node_num_ += 1;
            if (use_node_num_ == allocate_num_)
            {
              cout << "run out of memory." << endl;
              return false;
            }
          }
          else if (corridor_edges || pro_node->node_state == IN_OPEN_SET)
          {
            if (tmp_g_score < pro_node->g_score)
            {
              // pro_node->index = pro_id;
              pro_node->position = pro_pos;
              pro_node->f_score = tmp_f_score;
              pro_node->g_score = tmp_g_score;
              pro_node->parent = cur_node;
              if (corridor_edges) {
                pro_node->node_state = IN_OPEN_SET;
                push_open(pro_node);
              }
              if (dynamic)
                pro_node->time = cur_node->time + 1.0;
            }
          }
          else
          {
            cout << "error type in searching: " << pro_node->node_state << endl;
          }

          /* ----------  ---------- */
        }
    }

    if (goal_parent) return finish(goal_parent);

    /* ---------- open set empty, no path ---------- */
    cout << "open set empty, no path!" << endl;
    cout << "use node num: " << use_node_num_ << endl;
    cout << "iter num: " << iter_num_ << endl;
    ROS_WARN("[Astar debug] %s expanded_bbox=[%.2f %.2f]x[%.2f %.2f], closest_to_goal=(%.2f, %.2f), closest_dist=%.2f, goal=(%.2f, %.2f), rejects bounds=%d closed=%d collision=%d",
             use_static ? "static" : "sdf",
             expanded_min(0), expanded_max(0), expanded_min(1), expanded_max(1),
             closest_to_goal(0), closest_to_goal(1), closest_goal_dist,
             end_pt(0), end_pt(1), reject_bounds, reject_closed, reject_collision);
    return false;
  }

  void Astar::setParam(ros::NodeHandle &nh)
  {

    // resolution 可以理解成最小分辨率
    nh.param("astar/resolution_astar", resolution_, -1.0);
    // 这里的astar可以加上时间维度
    nh.param("astar/time_resolution", time_resolution_, -1.0);
    // 用于放大f_score的一个倍速；
    nh.param("astar/lambda_heu", lambda_heu_, -1.0);
    // 分配的最大可以搜索的数量；
    nh.param("astar/allocate_num", allocate_num_, 1);
    // 搜索允许的最大范围（例如 10 米内）
    nh.param("astar/horizon", horizon_, -1.0);
    // Dimensionless planner-3 penalty per metre (launch default 1; 0 = length only).
    nh.param("astar/w_clearance", w_clearance_, 0.0);
    // Preferred ADDITIONAL distance from selected blocked bins, not a safety radius.
    // Preserve the omitted-parameter legacy fallback; planner-3 launches supply 0.8.
    nh.param("astar/clearance_sigma", clearance_sigma_, 0.5);
    nh.param("astar/use_static_global_map", use_static_global_map_, false);
    // tie_breaker 见路径规划课程
    tie_breaker_ = 1.0 + 1.0 / 10000;
  }

  void Astar::retrievePath(NodePtr end_node)
  {
    NodePtr cur_node = end_node;
    path_nodes_.push_back(cur_node);

    while (cur_node->parent != NULL)
    {
      cur_node = cur_node->parent;
      path_nodes_.push_back(cur_node);
    }

    reverse(path_nodes_.begin(), path_nodes_.end());
  }

  std::vector<Eigen::Vector2d> Astar::getPath()
  {
    vector<Eigen::Vector2d> path;
    for (size_t i = 0; i < path_nodes_.size(); ++i)
    {
      path.push_back(path_nodes_[i]->position);
    }
    if (append_goal_) path.push_back(exact_goal_);
    return path;
  }

  double Astar::getDiagHeu(Eigen::Vector2d x1, Eigen::Vector2d x2)
  {
    double dx = fabs(x1(0) - x2(0));
    double dy = fabs(x1(1) - x2(1));

    double h = (dx + dy) + (sqrt(2.0) - 2) * min(dx, dy);

    return tie_breaker_ * h;
  }

  double Astar::getManhHeu(Eigen::Vector2d x1, Eigen::Vector2d x2)
  {
    double dx = fabs(x1(0) - x2(0));
    double dy = fabs(x1(1) - x2(1));
    // double dz = fabs(x1(2) - x2(2));

    // return tie_breaker_ * (dx + dy + dz);
    return tie_breaker_ * (dx + dy);
  }

  double Astar::getEuclHeu(Eigen::Vector2d x1, Eigen::Vector2d x2)
  {
    return tie_breaker_ * (x2 - x1).norm();
  }

  void Astar::init()
  {
    /* ---------- map params ---------- */
    this->inv_resolution_ = 1.0 / resolution_;
    inv_time_resolution_ = time_resolution_ > 0.0 ? 1.0 / time_resolution_ : 0.0;
    Eigen::Vector3d ori, map_size_3d;
    collision_->getMapRegion(ori, map_size_3d);
    origin_ << ori(0), ori(1);
    map_size_2d_ << map_size_3d(0), map_size_3d(1);
    map_max_2d_ = origin_ + map_size_2d_;

    cout << "origin_: " << origin_.transpose() << endl;
    cout << "map bounds: " << origin_.transpose() << " -> " << map_max_2d_.transpose() << endl;

    /* ---------- pre-allocated node ---------- */
    path_node_pool_.resize(allocate_num_);
    for (int i = 0; i < allocate_num_; i++)
    {
      path_node_pool_[i] = new Node;
    }

    use_node_num_ = 0;
    iter_num_ = 0;
  }

  // void Astar::setEnvironment(const EDTEnvironment::Ptr &env)
  // {
  //   this->edt_environment_ = env;
  // }

  void Astar::setCollision(const CollisionDetection::Ptr &col)
  {
    this->collision_ = col;
  }

  void Astar::reset()
  {
    append_goal_ = false;
    has_path_ = false;
    expanded_nodes_.clear();
    path_nodes_.clear();

    std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> empty_queue;
    open_set_.swap(empty_queue);

    for (int i = 0; i < use_node_num_; i++)
    {
      NodePtr node = path_node_pool_[i];
      node->parent = NULL;
      node->node_state = NOT_EXPAND;
    }

    use_node_num_ = 0;
    iter_num_ = 0;
  }

  std::vector<NodePtr> Astar::getVisitedNodes()
  {
    vector<NodePtr> visited;
    visited.assign(path_node_pool_.begin(), path_node_pool_.begin() + use_node_num_ - 1);
    return visited;
  }

  Eigen::Vector2i Astar::posToIndex(Eigen::Vector2d pt)
  {
    Vector2i idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();

    // idx << floor((pt(0) - origin_(0)) * inv_resolution_), floor((pt(1) -
    // origin_(1)) * inv_resolution_),
    //     floor((pt(2) - origin_(2)) * inv_resolution_);

    return idx;
  }

  int Astar::timeToIndex(double time)
  {
    if (inv_time_resolution_ <= 0.0)
      return 0;
    int idx = floor((time - time_origin_) * inv_time_resolution_);
    return idx;
  }
} // namespace fast_planner
