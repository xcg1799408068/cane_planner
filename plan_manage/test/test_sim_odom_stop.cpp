#include <gtest/gtest.h>
#include <plan_manager.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

namespace cane_planner
{
// Reuse LFPC's existing test access to observe gait time and control state.
class StaticMpcTestAccess
{
public:
    static std::vector<double> state(const LFPC& m)
    {
        return {double(m.support_leg_), m.t_sup_, m.delta_t_, m.h_, m.t_c_,
                double(m.step_num_), m.x_0_, m.vx_0_, m.y_0_, m.vy_0_, m.t_,
                m.x_t_, m.vx_t_, m.y_t_, m.vy_t_, m.al_, m.aw_, m.theta_, m.b_};
    }
};

class SimOdomTestAccess
{
public:
    static void setup(PlannerManager& m, ros::NodeHandle& nh)
    {
        m.simulation_ = true;
        m.gazebo_sim_ = false;
        m.mpc_debug_enable_ = false;
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
        m.cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 20);
    }

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
}  // namespace cane_planner

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_sim_odom_stop");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
