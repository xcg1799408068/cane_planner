#include <gtest/gtest.h>
#include <plan_manager.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <atomic>
#include <thread>
#include <future>

namespace cane_planner
{
// Reuse LFPC's existing test access to observe gait time and control state.
class StaticMpcTestAccess
{
public:
    static CollisionDetection::Ptr map()
    {
        auto c=std::make_shared<CollisionDetection>();
        c->static_global_map_ready_=true; c->static_global_esdf_ready_=false;
        c->static_origin_=Eigen::Vector2d(-10,-10); c->static_size_=Eigen::Vector2i(200,200);
        c->static_map_resolution_=.1; c->static_map_inv_resolution_=10.; c->static_require_known_region_=false;
        c->static_inflated_.assign(40000,0); c->static_distance_.assign(40000,10.);
        return c;
    }
    static void narrow(CollisionDetection& c) {
        for(int x=0;x<200;++x)for(int y=0;y<200;++y) {
            const double py=-10+(y+.5)*.1;
            if(std::abs(py)>.3)c.static_inflated_[c.staticToAddress(Eigen::Vector2i(x,y))]=1;
        }
    }
    static std::vector<double> state(const LFPC& m)
    {
        return {double(m.support_leg_), m.t_sup_, m.delta_t_, m.h_, m.t_c_,
                double(m.step_num_), m.x_0_, m.vx_0_, m.y_0_, m.vy_0_, m.t_,
                m.x_t_, m.vx_t_, m.y_t_, m.vy_t_, m.al_, m.aw_, m.theta_, m.b_};
    }
};

class AstarEdgeTestAccess {
public:
    static void initMap(Astar& a) {
        a.inv_resolution_=1./a.resolution_;a.inv_time_resolution_=0.;
        a.origin_=Eigen::Vector2d(-10,-10);a.map_max_2d_=Eigen::Vector2d(10,10);
        a.path_node_pool_.resize(a.allocate_num_);for(auto& p:a.path_node_pool_)p=new Node;
        a.use_node_num_=a.iter_num_=0;
    }
};
class SimOdomTestAccess
{
public:
    static void setup(PlannerManager& m, ros::NodeHandle& nh)
    {
        m.initial_route_active_ = true; // existing step fixtures model an already activated route
        m.simulation_ = true;
        m.gazebo_sim_ = false;
        m.mpc_debug_enable_ = false;
        m.transaction_diag_pub_=nh.advertise<std_msgs::Float64MultiArray>("transaction",20);
        m.lfpc_model_.reset(new LFPC);
        m.lfpc_model_->initializeModel(nh);
        m.lfpc_model_->reset(Eigen::Vector3d(0.4, -0.2, 0.7),
                             Eigen::Vector3d(5.4, 2.6, 0.0), LEFT_LEG, 5);
        m.lfpc_model_->SetCtrlParams(Eigen::Vector3d(0.25, 0.03, 0.1));
        // A real moving state, including nonzero phase and output velocity.
        m.lfpc_model_->updateOneStep();
        m.odom_pos_ = m.lfpc_model_->getCOMPos();
        m.odom_vel_ = Eigen::Vector3d(0.6, -0.4, 0.0);
        m.start_state_ = Eigen::Vector3d(m.odom_pos_.x(), m.odom_pos_.y(), 0.8);
        m.end_pt_ = Eigen::Vector2d(20.0, 20.0);
        m.mpc_sim_goal_ = Eigen::Vector3d(20.0, 20.0, 0.0);
        m.global_wp_idx_ = 0;
        m.last_com_pos_ = m.odom_pos_.head(2);
        m.last_theta_ = 0.8;
        m.mpc_stuck_steps_ = 0;
        m.mpc_step_count_ = 0;
        m.mpc_reached_goal_ = false;
        m.mpc_controller_.reset(new MpcController);
        // Missing corridor exercises the real fail-closed plan and STOP branch.
        m.mpc_controller_->init();
        m.sim_odom_pub_ = nh.advertise<nav_msgs::Odometry>("sim_odom", 20);
        m.mpc_stop_advice_pub_ = nh.advertise<std_msgs::Bool>("stop", 20);
        m.mpc_stop_reason_pub_ = nh.advertise<std_msgs::String>("reason", 20);
        m.mpc_best_traj_pub_ = nh.advertise<visualization_msgs::Marker>("best", 20);
        m.mpc_fov_pub_ = nh.advertise<visualization_msgs::Marker>("fov", 20);
        m.mpc_wp_pub_ = nh.advertise<visualization_msgs::Marker>("wp", 20);
        m.mpc_wps_pub_ = nh.advertise<visualization_msgs::Marker>("wps", 20);
        m.mpc_vis_pub_ = nh.advertise<visualization_msgs::Marker>("com", 20);
        m.mpc_foot_pub_ = nh.advertise<visualization_msgs::Marker>("feet", 20);
        m.mpc_path_pub_ = nh.advertise<nav_msgs::Path>("path", 20);
        m.mpc_convex_corridor_pub_ = nh.advertise<visualization_msgs::MarkerArray>("corridor", 20);
        m.cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 20);
    }

    static void dynamicSetup(PlannerManager& m)
    {
        m.pedestrians_enabled_=true;
        m.convex_corridor_.reset(new ConvexCorridor);
        m.collision_=StaticMpcTestAccess::map(); m.mpc_controller_->setCollision(m.collision_);
        const Eigen::Vector2d p=m.lfpc_model_->getCOMPos().head<2>();
        m.global_waypoints_={p,p+Eigen::Vector2d(1,0),p+Eigen::Vector2d(2,0)};
        m.global_wp_idx_=0;
    }
    static void recoverySetup(PlannerManager&m,ros::NodeHandle&nh)
    {
        m.lfpc_model_->reset(Eigen::Vector3d(.4,0,0),Eigen::Vector3d(0,0,0),LEFT_LEG,5);
        m.odom_pos_=m.lfpc_model_->getCOMPos();m.last_com_pos_=m.odom_pos_.head<2>();
        m.start_state_=Eigen::Vector3d::Zero();m.mpc_sim_goal_=Eigen::Vector3d(5,0,0);
        m.end_pt_=Eigen::Vector2d(5,0);m.global_waypoints_={Eigen::Vector2d(0,0),Eigen::Vector2d(1,0),Eigen::Vector2d(2,0),Eigen::Vector2d(3,0),Eigen::Vector2d(4,0)};
        m.global_wp_idx_=0;m.mpc_controller_->setParam(nh);m.mpc_controller_->init();
    }
    static void astarSetup(PlannerManager&m,ros::NodeHandle&nh,bool narrow=false) {
        if(narrow)StaticMpcTestAccess::narrow(*m.collision_);
        nh.setParam("astar/resolution_astar",.1);nh.setParam("astar/lambda_heu",1.);
        nh.setParam("astar/horizon",200.);nh.setParam("astar/allocate_num",100000);
        nh.setParam("astar/w_clearance",1.);nh.setParam("astar/clearance_sigma",.8);
        m.astar_finder_.reset(new Astar);m.astar_finder_->setParam(nh);m.astar_finder_->setCollision(m.collision_);
        m.astar_finder_->setCorridorEdgeClearance(m.convex_corridor_->getConfig().clearance);AstarEdgeTestAccess::initMap(*m.astar_finder_);
        m.a_path_pub_=nh.advertise<nav_msgs::Path>("astar",20);
    }
    static void invalidateRebuild(PlannerManager&m) {auto c=m.convex_corridor_->getConfig();c.iterations=0;m.convex_corridor_->setConfig(c);}
    static void afterSearch(PlannerManager&m,std::function<void()> hook) {m.after_recovery_search_test_hook_=std::move(hook);}
    static uint64_t attempts(PlannerManager&m) {return m.dynamic_recovery_attempts_;}
    static std::string recoveryReason(PlannerManager&m) {return m.dynamic_recovery_reason_;}
    static std::vector<Eigen::Vector2d> route(PlannerManager&m) {return m.global_waypoints_;}
    static void initialSetup(PlannerManager&m) {m.have_odom_=m.have_target_=true;m.exec_state_=PlannerManager::GEN_NEW_TRAJ;}
    static void fsm(PlannerManager&m) {m.execFSMCallback(ros::TimerEvent());}
    static bool active(PlannerManager&m) {return m.exec_state_==PlannerManager::MPC_STEP && m.initial_route_active_;}
    static void initialHook(PlannerManager&m,std::function<void()> h) {m.after_initial_search_test_hook_=std::move(h);}
    static void goalEntry(PlannerManager&m,std::function<void()> h) {m.goal_entry_test_hook_=std::move(h);}
    static void goalB(PlannerManager&m) {geometry_msgs::PoseStamped::Ptr g(new geometry_msgs::PoseStamped);g->pose.position.x=4.;g->pose.position.y=1.;g->pose.orientation.w=1.;m.GoalCallback(g);}
    static bool planValid(PlannerManager&m) {return m.mpc_controller_->lastPlanValid();}
    static void receive(PlannerManager& m,const onboard_detector::DynamicObstacles::Ptr& msg) {m.dynamicObstaclesCallback(msg);}
    static ConvexCorridor::Result corridor(PlannerManager&m) {
        std::lock_guard<std::mutex> lock(m.dynObsMutex_);
        return m.updateAndPublishConvexCorridorLocked(m.lfpc_model_->getCOMPos());
    }
    static void callbackEntry(PlannerManager&m,std::function<void()> hook) {m.pedestrian_callback_entry_test_hook_=std::move(hook);}
    static ros::Time receipt(PlannerManager&m) {std::lock_guard<std::mutex> lock(m.dynObsMutex_);return m.pedestrian_receipt_;}
    static void enabled(PlannerManager&m,bool enabled) {
        std::lock_guard<std::mutex> lock(m.dynObsMutex_);
        ++m.pedestrian_generation_;m.pedestrians_enabled_=enabled;
    }
    static void afterMppi(PlannerManager&m,std::function<void()> hook) {m.after_mppi_test_hook_=std::move(hook);}
    static void duringGeometry(PlannerManager&m,std::function<void()> hook) {m.geometry_locked_test_hook_=std::move(hook);}
    static void goalReset(PlannerManager&m) {
        m.have_odom_=false; // no FSM transition; exercise both real goal callbacks
        geometry_msgs::PoseStamped::Ptr goal(new geometry_msgs::PoseStamped);
        goal->pose.position.x=10.;goal->pose.orientation.w=1.;
        m.duplicate_goal_time_window_=0.;
        m.GoalCallback(goal);
        nav_msgs::Path::Ptr path(new nav_msgs::Path);
        goal->pose.position.x=11.;path->poses.push_back(*goal);
        m.waypointCallback(path);
    }
    static std::string reason(PlannerManager&m) {std::lock_guard<std::mutex> lock(m.dynObsMutex_);return m.pedestrian_data_reason_;}
    static std::vector<PedestrianPolygon::Observation> observations(PlannerManager&m) {
        std::lock_guard<std::mutex> lock(m.dynObsMutex_);return m.pedestrian_observations_;
    }
    static void transform(PlannerManager&m,const tf::StampedTransform&t) {ASSERT_TRUE(m.tf_listener_.setTransform(t,"test"));}
    static bool geometryEmpty(PlannerManager&m) {std::lock_guard<std::mutex> lock(m.dynObsMutex_);return m.pedestrian_geometry_.empty();}
    static bool step(PlannerManager& m) { return m.mpcSimStep(); }
    static void publishMoving(PlannerManager& m) { m.publishSimOdom(); }
    static LFPC& model(PlannerManager& m) { return *m.lfpc_model_; }
    static void mode(PlannerManager& m, bool simulation, bool gazebo)
    {
        m.simulation_ = simulation;
        m.gazebo_sim_ = gazebo;
    }
    static void expectHeld(const PlannerManager& m, const Eigen::Vector3d& pos)
    {
        EXPECT_EQ(pos, m.odom_pos_);
        EXPECT_EQ(Eigen::Vector3d(0.6, -0.4, 0.0), m.odom_vel_);
        EXPECT_EQ(Eigen::Vector3d(pos.x(), pos.y(), 0.8), m.start_state_);
        EXPECT_DOUBLE_EQ(0.8, m.last_theta_);
        EXPECT_TRUE(m.mpc_step_path_.empty());
        EXPECT_FALSE(m.mpc_controller_->lastPlanValid());
        EXPECT_EQ(0, m.mpc_stuck_steps_);
    }
};

class SimOdomTest : public ::testing::Test
{
protected:
    ros::NodeHandle nh_{"~"};
    PlannerManager manager_{3};
    std::vector<nav_msgs::Odometry> odom_;
    std::vector<bool> stops_;
    std::vector<int> best_actions_;
    ros::Subscriber odom_sub_, stop_sub_, best_sub_;

    void SetUp() override
    {
        SimOdomTestAccess::setup(manager_, nh_);
        odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("sim_odom", 20,
            [this](const nav_msgs::Odometry::ConstPtr& msg) { odom_.push_back(*msg); });
        stop_sub_ = nh_.subscribe<std_msgs::Bool>("stop", 20,
            [this](const std_msgs::Bool::ConstPtr& msg) { stops_.push_back(msg->data); });
        best_sub_ = nh_.subscribe<visualization_msgs::Marker>("best", 20,
            [this](const visualization_msgs::Marker::ConstPtr& msg) {
                best_actions_.push_back(msg->action);
            });
        const auto deadline = ros::WallTime::now() + ros::WallDuration(3.0);
        while ((odom_sub_.getNumPublishers() == 0 || stop_sub_.getNumPublishers() == 0 ||
                best_sub_.getNumPublishers() == 0) && ros::WallTime::now() < deadline)
        {
            ros::spinOnce();
            ros::WallDuration(0.005).sleep();
        }
        ASSERT_EQ(1u, odom_sub_.getNumPublishers());
        ASSERT_EQ(1u, stop_sub_.getNumPublishers());
        ASSERT_EQ(1u, best_sub_.getNumPublishers());
    }

    // Force callback entry during real MPPI's transaction, but never join a
    // callback while holding that transaction. The pre-lock entry barrier proves
    // actual contention; timeout must fail on the old geometry/MPPI lock gap.
    void queuedFrameStep(const onboard_detector::DynamicObstacles::Ptr& msg, bool expire=false)
    {
        std::promise<void> entered;
        auto entry=entered.get_future();
        std::future<void> receiving;
        std::atomic<bool> done{false};std::thread clock;
        SimOdomTestAccess::callbackEntry(manager_,[&]{entered.set_value();});
        SimOdomTestAccess::afterMppi(manager_,[&] {
            EXPECT_TRUE(SimOdomTestAccess::planValid(manager_));
            receiving=std::async(std::launch::async,[&]{SimOdomTestAccess::receive(manager_,msg);});
            EXPECT_EQ(std::future_status::ready,entry.wait_for(std::chrono::seconds(2)));
            EXPECT_EQ(std::future_status::timeout,receiving.wait_for(std::chrono::milliseconds(20)));
            if(expire)ros::Time::setNow(ros::Time::now()+ros::Duration(1.));
            else clock=std::thread([&]{double t=ros::Time::now().toSec();while(!done){t+=.001;ros::Time::setNow(ros::Time(t));ros::WallDuration(.002).sleep();}});
        });
        EXPECT_FALSE(SimOdomTestAccess::step(manager_));
        done=true;if(clock.joinable())clock.join();
        receiving.get(); // now outside execution lock, genuinely accepted after commit
        SimOdomTestAccess::afterMppi(manager_,{});SimOdomTestAccess::callbackEntry(manager_,{});
    }
    void waitForDecision(size_t count,bool stop)
    {
        const auto end=ros::WallTime::now()+ros::WallDuration(2.);
        while(stops_.size()<count && ros::WallTime::now()<end){ros::spinOnce();ros::WallDuration(.005).sleep();}
        ASSERT_EQ(count,stops_.size());EXPECT_EQ(stop,stops_.back());
        waitForOdom(count);
    }
    void waitForStops(size_t count)
    {
        const auto deadline = ros::WallTime::now() + ros::WallDuration(2.0);
        while ((stops_.size() < count || best_actions_.size() < count) &&
               ros::WallTime::now() < deadline)
        {
            ros::spinOnce();
            ros::WallDuration(0.005).sleep();
        }
        ASSERT_EQ(count, stops_.size());
        ASSERT_EQ(count, best_actions_.size());
        EXPECT_TRUE(stops_.back());
        EXPECT_EQ(visualization_msgs::Marker::DELETE, best_actions_.back());
    }

    void waitForOdom(size_t count)
    {
        const auto deadline = ros::WallTime::now() + ros::WallDuration(2.0);
        while (odom_.size() < count && ros::WallTime::now() < deadline)
        {
            ros::spinOnce();
            ros::WallDuration(0.005).sleep();
        }
        ASSERT_EQ(count, odom_.size());
    }
};

TEST_F(SimOdomTest, RepeatedRealStopPublishesFrozenPoseAndZeroTwist)
{
    LFPC& model = SimOdomTestAccess::model(manager_);
    const auto state = StaticMpcTestAccess::state(model);
    const auto pos = model.getCOMPos();
    const auto foot = model.getFootPosition();
    const auto path = model.getStepCOMPath();
    const auto velocity = model.getNextIterState();
    ASSERT_GT(velocity.head(2).norm(), 0.01);
    ASSERT_GT(state[10], 0.0);  // nonzero LFPC time detects accidental phase resets

    for (size_t i = 0; i < 12; ++i)
    {
        ros::Time::setNow(ros::Time(100.0 + 0.1 * i));
        EXPECT_FALSE(SimOdomTestAccess::step(manager_));
        waitForStops(i + 1);
        waitForOdom(i + 1);
        ASSERT_EQ(i + 1, odom_.size());
        const auto& msg = odom_.back();
        EXPECT_EQ(ros::Time::now(), msg.header.stamp);
        if (i)
        {
            EXPECT_GT(msg.header.stamp, odom_[i - 1].header.stamp);
        }
        EXPECT_EQ("world", msg.header.frame_id);
        EXPECT_TRUE(msg.child_frame_id.empty());
        EXPECT_DOUBLE_EQ(pos.x(), msg.pose.pose.position.x);
        EXPECT_DOUBLE_EQ(pos.y(), msg.pose.pose.position.y);
        EXPECT_DOUBLE_EQ(0.0, msg.pose.pose.position.z);
        EXPECT_EQ(tf::createQuaternionMsgFromYaw(velocity.z()), msg.pose.pose.orientation);
        EXPECT_EQ(geometry_msgs::Twist(), msg.twist.twist);
        EXPECT_EQ(state, StaticMpcTestAccess::state(model));
        EXPECT_EQ(pos, model.getCOMPos());
        EXPECT_EQ(foot, model.getFootPosition());
        EXPECT_EQ(path, model.getStepCOMPath());
        SimOdomTestAccess::expectHeld(manager_, pos);
    }
    // Default/active serialization must still expose the original moving values.
    ros::Time::setNow(ros::Time(102.0));
    SimOdomTestAccess::publishMoving(manager_);
    waitForOdom(13);
    ASSERT_EQ(13u, odom_.size());
    EXPECT_DOUBLE_EQ(velocity.x(), odom_.back().twist.twist.linear.x);
    EXPECT_DOUBLE_EQ(velocity.y(), odom_.back().twist.twist.linear.y);
    EXPECT_EQ(odom_.front().pose, odom_.back().pose);
    EXPECT_EQ(state, StaticMpcTestAccess::state(model));
}

TEST_F(SimOdomTest, HardwareAndGazeboStopsDoNotPublishSimHeartbeat)
{
    size_t count = 0;
    for (const auto& mode : {std::make_pair(false, false), std::make_pair(true, true),
                             std::make_pair(false, true)})
    {
        SimOdomTestAccess::mode(manager_, mode.first, mode.second);
        ros::Time::setNow(ros::Time(110.0 + count));
        EXPECT_FALSE(SimOdomTestAccess::step(manager_));
        waitForStops(++count);
        const auto deadline = ros::WallTime::now() + ros::WallDuration(0.1);
        while (ros::WallTime::now() < deadline)
        {
            ros::spinOnce();
            ros::WallDuration(0.005).sleep();
        }
        EXPECT_TRUE(odom_.empty());
    }
}
onboard_detector::DynamicObstacles::Ptr frame(const ros::Time& stamp, const std::string& name="world")
{
    onboard_detector::DynamicObstacles::Ptr msg(new onboard_detector::DynamicObstacles);
    msg->header.stamp=stamp;msg->header.frame_id=name;msg->num=0;return msg;
}
void addPedestrian(onboard_detector::DynamicObstacles& msg,const Eigen::Vector2d& p)
{
    geometry_msgs::Vector3 pos,vel,size;pos.x=p.x();pos.y=p.y();vel.x=1.;size.x=.4;size.y=.6;size.z=1.7;
    msg.position.push_back(pos);msg.velocity.push_back(vel);msg.size.push_back(size);msg.num=1;
}
TEST_F(SimOdomTest, DynamicValidityTransitionsAndFrozenStop)
{
    ros::Time::setNow(ros::Time(200));SimOdomTestAccess::dynamicSetup(manager_);
    using R=ConvexCorridor::FailureReason;
    EXPECT_EQ(R::DYNAMIC_DATA_INVALID,SimOdomTestAccess::corridor(manager_).failure_reason);
    const auto state=StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_));
    EXPECT_FALSE(SimOdomTestAccess::step(manager_));waitForStops(1);waitForOdom(1);
    EXPECT_EQ(state,StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_)));
    EXPECT_EQ(geometry_msgs::Twist(),odom_.back().twist.twist);
    auto msg=frame(ros::Time::now());SimOdomTestAccess::receive(manager_,msg);
    EXPECT_TRUE(SimOdomTestAccess::corridor(manager_).feasible); // empty != unavailable
    ros::Time::setNow(ros::Time(201));
    EXPECT_EQ(R::DYNAMIC_DATA_INVALID,SimOdomTestAccess::corridor(manager_).failure_reason);
    msg=frame(ros::Time::now());msg->num=1;SimOdomTestAccess::receive(manager_,msg);
    EXPECT_EQ(R::DYNAMIC_DATA_INVALID,SimOdomTestAccess::corridor(manager_).failure_reason);
    msg=frame(ros::Time(202));SimOdomTestAccess::receive(manager_,msg);
    EXPECT_EQ(R::DYNAMIC_DATA_INVALID,SimOdomTestAccess::corridor(manager_).failure_reason);
    msg=frame(ros::Time::now(),"no_such_frame");SimOdomTestAccess::receive(manager_,msg);
    EXPECT_EQ(R::DYNAMIC_DATA_INVALID,SimOdomTestAccess::corridor(manager_).failure_reason);
    msg=frame(ros::Time::now());addPedestrian(*msg,Eigen::Vector2d(2,2));msg->size[0].x=NAN;
    SimOdomTestAccess::receive(manager_,msg);
    EXPECT_EQ(R::DYNAMIC_DATA_INVALID,SimOdomTestAccess::corridor(manager_).failure_reason);
    EXPECT_TRUE(SimOdomTestAccess::geometryEmpty(manager_));
    SimOdomTestAccess::enabled(manager_,false);EXPECT_TRUE(SimOdomTestAccess::corridor(manager_).feasible);
}
TEST_F(SimOdomTest, DynamicBlockedThenFreshClearRecoversCorridor)
{
    ros::Time::setNow(ros::Time(210));SimOdomTestAccess::dynamicSetup(manager_);
    const auto p=SimOdomTestAccess::model(manager_).getCOMPos().head<2>().eval();
    auto msg=frame(ros::Time::now());addPedestrian(*msg,p+Eigen::Vector2d(1,0));
    SimOdomTestAccess::receive(manager_,msg);
    auto blocked=SimOdomTestAccess::corridor(manager_);
    EXPECT_EQ(ConvexCorridor::FailureReason::DYNAMIC_REFERENCE_BLOCKED,blocked.failure_reason);
    EXPECT_TRUE(blocked.segments.empty());
    const auto state=StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_));
    EXPECT_FALSE(SimOdomTestAccess::step(manager_));waitForStops(1);waitForOdom(1);
    EXPECT_EQ(state,StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_)));
    ros::Time::setNow(ros::Time(210.1));msg=frame(ros::Time::now());
    addPedestrian(*msg,p+Eigen::Vector2d(1,2));SimOdomTestAccess::receive(manager_,msg);
    const auto clear=SimOdomTestAccess::corridor(manager_);EXPECT_TRUE(clear.feasible)<<ConvexCorridor::failureReasonName(clear.failure_reason);
    msg=frame(ros::Time::now());SimOdomTestAccess::receive(manager_,msg);
    EXPECT_TRUE(SimOdomTestAccess::corridor(manager_).feasible);
    EXPECT_TRUE(SimOdomTestAccess::geometryEmpty(manager_));
}
TEST_F(SimOdomTest, FreshEmptyAfterDynamicBlockResumesActualIntegration)
{
    ros::Time::setNow(ros::Time(230));SimOdomTestAccess::dynamicSetup(manager_);
    SimOdomTestAccess::recoverySetup(manager_,nh_);
    auto msg=frame(ros::Time::now());addPedestrian(*msg,Eigen::Vector2d(1,0));
    SimOdomTestAccess::receive(manager_,msg);
    const auto before=StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_));
    EXPECT_FALSE(SimOdomTestAccess::step(manager_));waitForStops(1);waitForOdom(1);
    EXPECT_EQ(before,StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_)));
    msg=frame(ros::Time::now());SimOdomTestAccess::receive(manager_,msg);
    // Production visualization uses a ROS-duration sleep; advance only the test
    // clock while exercising the actual successful manager branch.
    std::atomic<bool> done{false};
    std::thread clock([&] {double t=230.;while(!done) {t+=.001;ros::Time::setNow(ros::Time(t));ros::WallDuration(.002).sleep();}});
    const bool finished=SimOdomTestAccess::step(manager_);
    done=true;clock.join();
    EXPECT_FALSE(finished);EXPECT_TRUE(SimOdomTestAccess::planValid(manager_));
    EXPECT_NE(before,StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_)));
    waitForOdom(2);
    // Existing prepareNextStep resets serialized instantaneous twist to zero;
    // actual recovery is certified by successful plan and integrated displacement.
    EXPECT_GT(std::hypot(odom_.back().pose.pose.position.x-odom_.front().pose.pose.position.x,
                         odom_.back().pose.pose.position.y-odom_.front().pose.pose.position.y),1e-6);
}
TEST_F(SimOdomTest, DynamicTransformsPositionVelocityAndCompleteBox)
{
    ros::Time::setNow(ros::Time(220));SimOdomTestAccess::dynamicSetup(manager_);
    tf::Transform t;t.setOrigin(tf::Vector3(3,4,0));t.setRotation(tf::createQuaternionFromYaw(1.5707963267948966));
    SimOdomTestAccess::transform(manager_,tf::StampedTransform(t,ros::Time(220),"world","ped_sensor"));
    auto msg=frame(ros::Time::now(),"ped_sensor");addPedestrian(*msg,Eigen::Vector2d(1,2));
    SimOdomTestAccess::receive(manager_,msg);
    const auto& obs=SimOdomTestAccess::observations(manager_);ASSERT_EQ(1u,obs.size());
    EXPECT_NEAR(1,obs[0].position.x(),1e-9);EXPECT_NEAR(5,obs[0].position.y(),1e-9);
    EXPECT_NEAR(0,obs[0].velocity.x(),1e-9);EXPECT_NEAR(1,obs[0].velocity.y(),1e-9);
    EXPECT_NEAR(.6,obs[0].full_size.x(),1e-9);EXPECT_NEAR(.4,obs[0].full_size.y(),1e-9);
    EXPECT_TRUE(SimOdomTestAccess::corridor(manager_).feasible);
}
TEST_F(SimOdomTest, DisabledSourceMessageAfterMppiDoesNotStopStaticIntegration)
{
    ros::Time::setNow(ros::Time(350));SimOdomTestAccess::dynamicSetup(manager_);
    SimOdomTestAccess::recoverySetup(manager_,nh_);SimOdomTestAccess::enabled(manager_,false);
    const auto position=SimOdomTestAccess::model(manager_).getCOMPos();
    auto blocker=frame(ros::Time::now());addPedestrian(*blocker,Eigen::Vector2d(1,0));
    queuedFrameStep(blocker);waitForDecision(1,false);
    EXPECT_GT((SimOdomTestAccess::model(manager_).getCOMPos()-position).norm(),1e-6);
    EXPECT_TRUE(SimOdomTestAccess::observations(manager_).empty());
}
TEST_F(SimOdomTest, QueuedSameStampBlockerIsAcceptedAfterCommitThenStopsNextCycle)
{
    ros::Time::setNow(ros::Time(300));SimOdomTestAccess::dynamicSetup(manager_);
    SimOdomTestAccess::recoverySetup(manager_,nh_);
    auto a=frame(ros::Time::now());SimOdomTestAccess::receive(manager_,a);
    auto b=frame(a->header.stamp);addPedestrian(*b,Eigen::Vector2d(1,0));
    const auto position=SimOdomTestAccess::model(manager_).getCOMPos();
    queuedFrameStep(b);waitForDecision(1,false);
    EXPECT_GT((SimOdomTestAccess::model(manager_).getCOMPos()-position).norm(),1e-6);
    EXPECT_EQ(ConvexCorridor::FailureReason::DYNAMIC_REFERENCE_BLOCKED,SimOdomTestAccess::corridor(manager_).failure_reason);
    const auto before=StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_));
    EXPECT_FALSE(SimOdomTestAccess::step(manager_));waitForDecision(2,true);
    EXPECT_EQ(before,StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_)));
    auto clear=frame(ros::Time::now());SimOdomTestAccess::receive(manager_,clear);
    queuedFrameStep(clear);waitForDecision(3,false);
    EXPECT_NE(before,StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_)));
}
TEST_F(SimOdomTest, QueuedFrameCannotMaskExpiryAndRetainsOriginalReceipt)
{
    ros::Time::setNow(ros::Time(310));SimOdomTestAccess::dynamicSetup(manager_);
    SimOdomTestAccess::recoverySetup(manager_,nh_);
    auto a=frame(ros::Time::now());SimOdomTestAccess::receive(manager_,a);
    const auto before=StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_));
    queuedFrameStep(frame(ros::Time::now()),true);waitForDecision(1,true);
    EXPECT_EQ(before,StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_)));
    EXPECT_EQ(ros::Time(310),SimOdomTestAccess::receipt(manager_));
    EXPECT_EQ(ConvexCorridor::FailureReason::DYNAMIC_DATA_INVALID,SimOdomTestAccess::corridor(manager_).failure_reason);
}
TEST_F(SimOdomTest, UnreplacedPlannedFrameMustStillBeFreshAtExecution)
{
    ros::Time::setNow(ros::Time(320));SimOdomTestAccess::dynamicSetup(manager_);
    SimOdomTestAccess::recoverySetup(manager_,nh_);
    auto a=frame(ros::Time::now());SimOdomTestAccess::receive(manager_,a);
    const auto before=StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_));
    SimOdomTestAccess::afterMppi(manager_,[&] {ros::Time::setNow(ros::Time(321));});
    EXPECT_FALSE(SimOdomTestAccess::step(manager_));waitForStops(1);waitForOdom(1);
    EXPECT_EQ("PLANNED_FRAME_EXPIRED",SimOdomTestAccess::reason(manager_));
    EXPECT_EQ(before,StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_)));
}
TEST_F(SimOdomTest, AcceptedInvalidFrameAndEnableTransitionRequireFreshValidData)
{
    ros::Time::setNow(ros::Time(330));SimOdomTestAccess::dynamicSetup(manager_);
    SimOdomTestAccess::recoverySetup(manager_,nh_);
    const auto before=StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_));
    auto invalid=frame(ros::Time::now());invalid->num=1;
    SimOdomTestAccess::receive(manager_,invalid);
    EXPECT_FALSE(SimOdomTestAccess::step(manager_));waitForDecision(1,true);
    SimOdomTestAccess::enabled(manager_,false);SimOdomTestAccess::enabled(manager_,true);
    EXPECT_FALSE(SimOdomTestAccess::step(manager_));waitForDecision(2,true);
    EXPECT_EQ(before,StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_)));
    auto clear=frame(ros::Time::now());SimOdomTestAccess::receive(manager_,clear);
    queuedFrameStep(clear);waitForDecision(3,false);
}
TEST_F(SimOdomTest, RepeatedTenHzEmptyUpdatesDoNotStarveExecution)
{
    ros::Time::setNow(ros::Time(400));SimOdomTestAccess::dynamicSetup(manager_);
    SimOdomTestAccess::recoverySetup(manager_,nh_);
    auto first=frame(ros::Time::now());SimOdomTestAccess::receive(manager_,first);
    const auto origin=SimOdomTestAccess::model(manager_).getCOMPos();
    double total=0.,maximum=0.;
    for(int i=0;i<8;++i) {
        ros::Time::setNow(ros::Time(400.+.1*i));
        const auto start=ros::WallTime::now();
        queuedFrameStep(frame(ros::Time::now()));waitForDecision(i+1,false);
        const double ms=(ros::WallTime::now()-start).toSec()*1000.;total+=ms;maximum=std::max(maximum,ms);
    }
    EXPECT_GT((SimOdomTestAccess::model(manager_).getCOMPos()-origin).norm(),.1);
    std::cout<<"TRAFFIC empty cycles=8 mean_ms="<<total/8<<" max_ms="<<maximum<<std::endl;
}
TEST_F(SimOdomTest, RepeatedTenHzMovingPedestrianUpdatesDoNotStarveExecution)
{
    ros::Time::setNow(ros::Time(410));SimOdomTestAccess::dynamicSetup(manager_);
    SimOdomTestAccess::recoverySetup(manager_,nh_);
    auto first=frame(ros::Time::now());addPedestrian(*first,Eigen::Vector2d(1,2));SimOdomTestAccess::receive(manager_,first);
    const auto origin=SimOdomTestAccess::model(manager_).getCOMPos();
    double total=0.,maximum=0.;
    for(int i=0;i<8;++i) {
        ros::Time::setNow(ros::Time(410.+.1*i));
        auto next=frame(ros::Time::now());addPedestrian(*next,Eigen::Vector2d(1.+.1*i,2));
        const auto start=ros::WallTime::now();queuedFrameStep(next);waitForDecision(i+1,false);
        const double ms=(ros::WallTime::now()-start).toSec()*1000.;total+=ms;maximum=std::max(maximum,ms);
    }
    EXPECT_GT((SimOdomTestAccess::model(manager_).getCOMPos()-origin).norm(),.1);
    std::cout<<"TRAFFIC moving cycles=8 mean_ms="<<total/8<<" max_ms="<<maximum<<std::endl;
}
TEST_F(SimOdomTest, GoalResetSerializesWithProductionGeometryAndPublication)
{
    ros::Time::setNow(ros::Time(340));SimOdomTestAccess::dynamicSetup(manager_);
    auto msg=frame(ros::Time::now());
    addPedestrian(*msg,SimOdomTestAccess::model(manager_).getCOMPos().head<2>()+Eigen::Vector2d(1,2));
    SimOdomTestAccess::receive(manager_,msg);
    std::promise<void> held, release, reset_entered;
    auto released=release.get_future().share();
    SimOdomTestAccess::duringGeometry(manager_,[&] {held.set_value();released.wait();});
    auto generating=std::async(std::launch::async,[&] {return SimOdomTestAccess::corridor(manager_);});
    held.get_future().wait();
    auto resetting=std::async(std::launch::async,[&] {reset_entered.set_value();SimOdomTestAccess::goalReset(manager_);});
    reset_entered.get_future().wait();
    EXPECT_EQ(std::future_status::timeout,resetting.wait_for(std::chrono::milliseconds(30)));
    release.set_value();
    EXPECT_TRUE(generating.get().feasible);
    resetting.get();
    EXPECT_TRUE(SimOdomTestAccess::geometryEmpty(manager_));
    SimOdomTestAccess::duringGeometry(manager_,{});
    // Exercise repeated resets against the real build + marker iteration path.
    std::atomic<bool> valid{true};
    std::thread builds([&] {for(int i=0;i<30;++i)if(!SimOdomTestAccess::corridor(manager_).feasible)valid=false;});
    std::thread resets([&] {for(int i=0;i<30;++i)SimOdomTestAccess::goalReset(manager_);});
    builds.join();resets.join();EXPECT_TRUE(valid);
}
TEST_F(SimOdomTest, DynamicBlockedReferenceReroutesFinalGoalAndExecutes)
{
    ros::Time::setNow(ros::Time(500));SimOdomTestAccess::dynamicSetup(manager_);
    SimOdomTestAccess::recoverySetup(manager_,nh_);SimOdomTestAccess::astarSetup(manager_,nh_);
    auto blocker=frame(ros::Time::now());addPedestrian(*blocker,Eigen::Vector2d(2,0));
    SimOdomTestAccess::receive(manager_,blocker);
    const auto original=SimOdomTestAccess::route(manager_);
    const auto pos=SimOdomTestAccess::model(manager_).getCOMPos();
    queuedFrameStep(blocker);waitForDecision(1,false);
    EXPECT_EQ("SUCCESS",SimOdomTestAccess::recoveryReason(manager_));
    EXPECT_EQ(1u,SimOdomTestAccess::attempts(manager_));
    const auto detour=SimOdomTestAccess::route(manager_);ASSERT_FALSE(detour.empty());
    EXPECT_EQ(Eigen::Vector2d(5,0),detour.back());EXPECT_NE(original,detour);
    EXPECT_GT((SimOdomTestAccess::model(manager_).getCOMPos()-pos).norm(),1e-6);
    for(int i=0;i<3;++i) {
        auto clear=frame(ros::Time::now());
        if(i==0)addPedestrian(*clear,Eigen::Vector2d(2,0)); // fresh same blocker, valid detour: no search
        SimOdomTestAccess::receive(manager_,clear);
        queuedFrameStep(clear);waitForDecision(i+2,false);
        EXPECT_EQ(detour,SimOdomTestAccess::route(manager_));EXPECT_EQ(1u,SimOdomTestAccess::attempts(manager_));
    }
}
TEST_F(SimOdomTest, FullyBlockedRecoveryStopsAndFreshClearResumes)
{
    ros::Time::setNow(ros::Time(510));SimOdomTestAccess::dynamicSetup(manager_);
    SimOdomTestAccess::recoverySetup(manager_,nh_);SimOdomTestAccess::astarSetup(manager_,nh_,true);
    auto blocker=frame(ros::Time::now());addPedestrian(*blocker,Eigen::Vector2d(2,0));
    SimOdomTestAccess::receive(manager_,blocker);
    const auto before=StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_));
    const auto route=SimOdomTestAccess::route(manager_);const auto start=ros::WallTime::now();
    EXPECT_FALSE(SimOdomTestAccess::step(manager_));waitForDecision(1,true);
    EXPECT_EQ(before,StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_)));
    EXPECT_EQ(route,SimOdomTestAccess::route(manager_));EXPECT_EQ(1u,SimOdomTestAccess::attempts(manager_));
    std::cout<<"MANAGER no_path_cycle_ms="<<(ros::WallTime::now()-start).toSec()*1000<<std::endl;
    auto clear=frame(ros::Time::now());SimOdomTestAccess::receive(manager_,clear);
    queuedFrameStep(clear);waitForDecision(2,false);
}
TEST_F(SimOdomTest, RecoveryCandidateExpiryCannotCommitOrExecute)
{
    ros::Time::setNow(ros::Time(520));SimOdomTestAccess::dynamicSetup(manager_);
    SimOdomTestAccess::recoverySetup(manager_,nh_);SimOdomTestAccess::astarSetup(manager_,nh_);
    auto blocker=frame(ros::Time::now());addPedestrian(*blocker,Eigen::Vector2d(2,0));
    SimOdomTestAccess::receive(manager_,blocker);
    const auto route=SimOdomTestAccess::route(manager_);const auto state=StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_));
    SimOdomTestAccess::afterSearch(manager_,[]{ros::Time::setNow(ros::Time(521));});
    EXPECT_FALSE(SimOdomTestAccess::step(manager_));waitForDecision(1,true);
    EXPECT_EQ(route,SimOdomTestAccess::route(manager_));EXPECT_EQ(state,StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_)));
    EXPECT_EQ("EXPIRED",SimOdomTestAccess::recoveryReason(manager_));
}
TEST_F(SimOdomTest, RecoveryRebuildFailureCannotCommitOrExecute)
{
    ros::Time::setNow(ros::Time(530));SimOdomTestAccess::dynamicSetup(manager_);
    SimOdomTestAccess::recoverySetup(manager_,nh_);SimOdomTestAccess::astarSetup(manager_,nh_);
    auto blocker=frame(ros::Time::now());addPedestrian(*blocker,Eigen::Vector2d(2,0));
    SimOdomTestAccess::receive(manager_,blocker);
    const auto route=SimOdomTestAccess::route(manager_);const auto state=StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_));
    SimOdomTestAccess::afterSearch(manager_,[&]{SimOdomTestAccess::invalidateRebuild(manager_);});
    EXPECT_FALSE(SimOdomTestAccess::step(manager_));waitForDecision(1,true);
    EXPECT_EQ(route,SimOdomTestAccess::route(manager_));EXPECT_EQ(state,StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_)));
    EXPECT_EQ("REBUILD_FAILED",SimOdomTestAccess::recoveryReason(manager_));
}
TEST_F(SimOdomTest, InitialSearchActivationQueuesNewGoalAndRequiresItsOwnSearch)
{
    ros::Time::setNow(ros::Time(540));SimOdomTestAccess::dynamicSetup(manager_);
    SimOdomTestAccess::recoverySetup(manager_,nh_);SimOdomTestAccess::astarSetup(manager_,nh_);
    SimOdomTestAccess::initialSetup(manager_);
    std::promise<void> entered;auto entry=entered.get_future();std::future<void> goal;
    SimOdomTestAccess::goalEntry(manager_,[&]{entered.set_value();});
    SimOdomTestAccess::initialHook(manager_,[&]{
        goal=std::async(std::launch::async,[&]{SimOdomTestAccess::goalB(manager_);});
        EXPECT_EQ(std::future_status::ready,entry.wait_for(std::chrono::seconds(2)));
        EXPECT_EQ(std::future_status::timeout,goal.wait_for(std::chrono::milliseconds(20)));
    });
    const auto pos=SimOdomTestAccess::model(manager_).getCOMPos();
    SimOdomTestAccess::fsm(manager_);goal.get();
    EXPECT_FALSE(SimOdomTestAccess::active(manager_));
    const auto state=StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_));
    EXPECT_FALSE(SimOdomTestAccess::step(manager_));
    EXPECT_EQ(state,StaticMpcTestAccess::state(SimOdomTestAccess::model(manager_)));
    EXPECT_EQ(pos,SimOdomTestAccess::model(manager_).getCOMPos());
    SimOdomTestAccess::initialHook(manager_,{});SimOdomTestAccess::goalEntry(manager_,{});
    SimOdomTestAccess::fsm(manager_);
    EXPECT_TRUE(SimOdomTestAccess::active(manager_));
    ASSERT_FALSE(SimOdomTestAccess::route(manager_).empty());
    EXPECT_EQ(Eigen::Vector2d(4,1),SimOdomTestAccess::route(manager_).back());
    EXPECT_EQ(pos,SimOdomTestAccess::model(manager_).getCOMPos());
}
TEST_F(SimOdomTest, TransactionDiagnosticIdentityClockAndSkippedStageReset)
{
    std::vector<std_msgs::Float64MultiArray> records;
    auto sub=nh_.subscribe<std_msgs::Float64MultiArray>("transaction",20,
        [&](const std_msgs::Float64MultiArray::ConstPtr& m){records.push_back(*m);});
    const auto connected=ros::WallTime::now()+ros::WallDuration(2.);
    while(sub.getNumPublishers()==0 && ros::WallTime::now()<connected){ros::spinOnce();ros::WallDuration(.005).sleep();}
    ASSERT_EQ(1u,sub.getNumPublishers());
    auto wait=[&](size_t n){const auto end=ros::WallTime::now()+ros::WallDuration(2.);
        while(records.size()<n && ros::WallTime::now()<end){ros::spinOnce();ros::WallDuration(.005).sleep();}};
    ros::Time::setNow(ros::Time(600));SimOdomTestAccess::dynamicSetup(manager_);
    SimOdomTestAccess::recoverySetup(manager_,nh_);
    auto clear=frame(ros::Time::now());SimOdomTestAccess::receive(manager_,clear);
    queuedFrameStep(clear);waitForDecision(1,false);wait(1);
    ASSERT_EQ(1u,records.size());ASSERT_EQ(33u,records[0].data.size());
    const auto a=records[0].data;
    EXPECT_EQ(1.,a[0]);EXPECT_EQ(600.,a[1]);EXPECT_TRUE(std::isnan(a[3]));EXPECT_TRUE(std::isnan(a[4]));
    EXPECT_EQ(600.,a[7]);EXPECT_EQ(600.,a[8]);EXPECT_EQ(0.,a[9]);EXPECT_EQ(0.,a[10]);
    EXPECT_NE(std::string::npos,records[0].layout.dim[0].label.find("source_frame=world"));
    EXPECT_GE(a[13],0.);EXPECT_TRUE(std::isnan(a[14]));EXPECT_TRUE(std::isnan(a[15]));
    EXPECT_GE(a[16],0.);EXPECT_GE(a[17],0.);EXPECT_GE(a[32],0.);EXPECT_EQ(0.,a[21]);
    EXPECT_GE(a[19],a[18]);EXPECT_EQ(0.,a[29]);EXPECT_EQ(1.,a[30]);
    // Large ROS jump, small wall interval: diagnose without asserting its cause.
    ros::Time::setNow(ros::Time(603));auto malformed=frame(ros::Time::now());malformed->num=1;
    SimOdomTestAccess::receive(manager_,malformed);
    EXPECT_FALSE(SimOdomTestAccess::step(manager_));waitForDecision(2,true);wait(2);
    ASSERT_EQ(2u,records.size());const auto b=records[1].data;
    EXPECT_EQ(3.,b[3]);EXPECT_GE(b[4],0.);EXPECT_EQ(603.,b[7]);EXPECT_EQ(603.,b[8]);EXPECT_GT(b[6],a[6]);
    EXPECT_TRUE(std::isnan(b[13]));EXPECT_TRUE(std::isnan(b[14]));EXPECT_TRUE(std::isnan(b[15]));
    EXPECT_TRUE(std::isnan(b[22]));EXPECT_TRUE(std::isnan(b[28]));EXPECT_TRUE(std::isnan(b[32]));
    EXPECT_EQ(1.,b[21]);EXPECT_EQ(0.,b[30]);
    EXPECT_EQ(static_cast<double>(ConvexCorridor::FailureReason::DYNAMIC_DATA_INVALID),b[31]);
    SimOdomTestAccess::enabled(manager_,false);queuedFrameStep(clear);waitForDecision(3,false);wait(3);
    ASSERT_EQ(3u,records.size());EXPECT_EQ(0.,records[2].data[5]);EXPECT_TRUE(std::isnan(records[2].data[7]));
    EXPECT_NE(std::string::npos,records[2].layout.dim[0].label.find("source_frame=DISABLED"));
}
}  // namespace cane_planner

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_sim_odom_stop");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
