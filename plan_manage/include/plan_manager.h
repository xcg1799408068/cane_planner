#ifndef __PLAN_MANAGE__
#define __PLAN_MANAGE__

#include <iostream>
#include <limits>
#include <vector>
#include <mutex>

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <onboard_detector/DynamicObstacles.h>
#include <bspline/non_uniform_bspline.h>
#include <bspline_opt/bspline_optimizer.h>

#include <path_searching/astar.h>
#include <path_searching/kinodynamic_astar.h>
#include <path_searching/mpc_controller.h>
#include <path_searching/kinematic_mppi_controller.h>
#include <path_searching/dynamic_walking_corridor.h>
#include <path_searching/timed_trajectory_builder.h>
#include <path_searching/convex_corridor.h>
#include <path_searching/path_smoother.h>
#include <path_searching/lfpc.h>
#include <plan_env/collision_detection.h>
#include <plan_container.hpp>

namespace cane_planner
{
    class PlannerManager
    {
    private:
        /* data */
        enum FSM_STATE
        {
            INIT,
            WAIT_TARGET,
            GEN_NEW_TRAJ,
            EXEC_TRAJ,
            REPLAN_TRAJ,
            MPC_STEP
        };

        enum TRAJECTORY_PLANNING_ID {
            GOAL = 1,
            PATH = 200,
            BSPLINE = 300,
            BSPLINE_CTRL_PT = 400,
            POLY_TRAJ = 500
        };
        /*---------- data -----------*/
        fast_planner::SDFMap::Ptr sdf_map_;
        CollisionDetection::Ptr collision_;
        LFPC::Ptr lfpc_model_;

        unique_ptr<Astar> astar_finder_;
        unique_ptr<KinodynamicAstar> kin_finder_;
        unique_ptr<MpcController> mpc_controller_;
        unique_ptr<KinematicMppiController> kinematic_mppi_controller_;
        unique_ptr<DynamicWalkingCorridor> dynamic_walking_corridor_;
        unique_ptr<ConvexCorridor> convex_corridor_;
        BsplineOptimizer::Ptr bspline_optimizers_;
        NonUniformBspline::Ptr bspline_init_;

        FSM_STATE exec_state_;
        ros::NodeHandle nh_;             // 节点私有句柄 (用于读param)
        bool have_odom_, have_target_;
        bool simulation_;                 // 里程计来源：true=仿真odom, false=真实TF
        int planner_;                     // 1=A*, 2=kinodynamic, 3=MPC
        bool gazebo_sim_;                 // true=Gazebo mode (publish cmd_vel)
        double no_replan_thresh_, replan_thresh_;

        Eigen::Vector3d odom_pos_, odom_vel_;
        Eigen::Quaterniond odom_ori_;

        Eigen::Vector2d start_pt_;
        Eigen::Vector2d end_pt_;

        Eigen::Vector3d start_state_;
        Eigen::Vector3d end_state_;

        // 动态障碍物缓存
        std::vector<Eigen::Vector3d> dynObsPos_;
        std::vector<Eigen::Vector3d> dynObsVel_;
        std::vector<Eigen::Vector3d> dynObsSize_;
        std::mutex dynObsMutex_;

        // MPC 规划结果缓存 (用于可视化)
        std::vector<Eigen::Vector3d> mpc_com_path_;
        std::vector<Eigen::Vector3d> mpc_feet_path_;
        std::vector<Eigen::Vector3d> mpc_step_path_;

        // 全局路径层 waypoints (A* → MPC 追踪)
        std::vector<Eigen::Vector2d> global_path_dense_;
        std::vector<Eigen::Vector2d> global_waypoints_;
        size_t global_wp_idx_;
        double global_wp_spacing_ = 1.0;
        double global_wp_arrival_radius_ = 0.1;
        bool global_wp_smoothing_enable_ = true;
        double global_wp_smoothing_radius_ = 0.45;
        double global_wp_smoothing_min_turn_angle_ = 0.55;
        int global_wp_smoothing_samples_ = 4;
        double lookahead_dist_ = 1.0;
        Eigen::Vector2d last_goal_cmd_ = Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN());
        double last_goal_yaw_ = std::numeric_limits<double>::quiet_NaN();
        ros::Time last_goal_cmd_time_;
        double duplicate_goal_xy_thresh_ = 0.05;
        double duplicate_goal_yaw_thresh_ = 0.05;
        double duplicate_goal_time_window_ = 1.5;
        double mpc_fov_range_ = 5.0;
        bool mpc_debug_enable_ = true;
        bool mpc_stop_advice_enable_ = true;
        bool mpc_stop_advice_enforce_ = true;
        double mpc_stop_hold_time_ = 0.8;
        double mpc_stop_release_clear_time_ = 0.5;
        bool mpc_corridor_stop_enable_ = true;
        double mpc_corridor_stop_valid_ratio_threshold_ = 0.2;
        bool mpc_interaction_enable_ = false;
        bool mpc_interaction_enable_yield_ = false;
        double mpc_interaction_st_horizon_ = 4.0;
        double mpc_interaction_yield_trigger_time_ = 2.5;
        double mpc_interaction_robot_radius_ = 0.25;
        double mpc_interaction_yield_safety_margin_ = 0.20;
        double mpc_interaction_front_min_ = 0.3;
        double mpc_interaction_front_max_ = 4.0;
        double mpc_interaction_corridor_width_ = 0.7;
        double mpc_interaction_cross_speed_ = 0.15;
        double mpc_interaction_time_gap_ = 0.8;
        double mpc_interaction_min_robot_speed_ = 0.15;
        double mpc_interaction_cpa_horizon_ = 3.0;
        double mpc_interaction_cpa_dist_ = 0.8;
        bool mpc_interaction_use_cpa_check_ = true;
        double mpc_interaction_stop_release_clear_time_ = 0.15;
        double mpc_interaction_post_yield_grace_time_ = 0.6;
        double mpc_nominal_al_ = 0.40;
        double lfpc_t_sup_ = 0.35;
        double lfpc_delta_t_ = 0.07;
        bool mpc_stop_state_active_ = false;
        ros::Time mpc_stop_enter_time_;
        ros::Time mpc_stop_clear_since_;
        std::string mpc_latched_stop_reason_ = "OK";

        // 仿真路径推进 (planner=1,2 沿规划路径移动 odom)
        std::vector<Eigen::Vector2d> sim_path_;
        size_t sim_path_idx_;
        double sim_speed_;    // 仿真行走速度 m/s

        // MPC 步进状态
        enum MpcSimState { MPC_IDLE, MPC_ACTIVE, MPC_DONE };
        enum InteractionScene { SCENE_NONE = 0, SCENE_CROSSING = 1 };
        enum InteractionMode { MODE_CONTINUE = 0, MODE_YIELD = 2 };
        std::vector<Eigen::Vector3d> last_corridor_feasible_mppi_path_;
        struct InteractionDebug
        {
            bool candidate_valid = false;
            int obs_idx = -1;
            double front = 0.0;
            double lateral = 0.0;
            double v_front = 0.0;
            double v_lateral = 0.0;
            double t_ped_to_path = -1.0;
            double signed_t_ped_to_path = -1.0;
            bool ped_before_path = false;
            bool ped_at_or_after_path = false;
            double t_robot_to_cross = -1.0;
            double time_gap = 0.0;
            double t_cpa = -1.0;
            double d_cpa = -1.0;
            bool cpa_conflict = false;
            double robot_speed_used = 0.0;
            double r_crossing = 0.0;
            int crossing_confirm_count = 0;
            int crossing_clear_count = 0;
            double risk_front = 0.0;
            bool yield_required = false;
            bool st_conflict = false;
            double st_t_conflict = -1.0;
            double st_d_conflict = -1.0;
            double st_safety_radius = 0.0;
            double st_robot_s = 0.0;
            double st_ped_s = 0.0;
            bool st_path_occupied = false;
            double st_path_t_enter = -1.0;
            double st_path_t_exit = -1.0;
            Eigen::Vector2d path_forward = Eigen::Vector2d::Zero();
        };
        MpcSimState mpc_sim_state_;
        InteractionScene mpc_interaction_scene_ = SCENE_NONE;
        InteractionMode mpc_interaction_mode_ = MODE_CONTINUE;
        InteractionDebug mpc_interaction_debug_;
        bool mpc_interaction_st_yield_latched_ = false;
        int mpc_interaction_st_latched_obs_idx_ = -1;
        int mpc_step_count_;             // 总步数计数 (仅用于日志)
        int mpc_stuck_steps_;            // waypoint 未推进的连续步数
        Eigen::Vector2d last_com_pos_;   // 上一帧CoM位置，用于检测是否实际移动
        double last_theta_;              // 上一帧航向角，用于计算cmd_vel角速度
        static constexpr int STUCK_THRESHOLD = 30;
        bool mpc_reached_goal_;

        Eigen::Vector3d mpc_sim_goal_;

        /*---------- Ros utils -----------*/
        ros::Timer exec_timer_;
        ros::Timer replan_timer_;
        ros::Subscriber odom_sub_, goal_sub, waypoint_sub_;
        ros::Subscriber start_sub_;
        ros::Subscriber dyn_obs_sub_;
        ros::Publisher astar_pub_, kin_vis_pub_, kin_foot_pub_;
        ros::Publisher kin_path_pub_, a_path_pub_;
        ros::Publisher traj_pub_;
        ros::Publisher mpc_vis_pub_, mpc_foot_pub_, mpc_path_pub_;
        ros::Publisher mpc_fov_pub_, mpc_wp_pub_, mpc_wps_pub_;
        ros::Publisher mpc_best_traj_pub_; // MPC predicted optimal CoM path (LINE_STRIP)
        ros::Publisher mpc_debug_metrics_pub_; // std_msgs/Float64MultiArray
        ros::Publisher mpc_stop_advice_pub_;   // std_msgs/Bool
        ros::Publisher mpc_stop_reason_pub_;   // std_msgs/String
        ros::Publisher mpc_interaction_scene_pub_; // std_msgs/String
        ros::Publisher mpc_interaction_mode_pub_;  // std_msgs/String
        ros::Publisher mpc_interaction_debug_pub_; // std_msgs/Float64MultiArray
        ros::Publisher mpc_dynamic_body_pub_;      // visualization_msgs/MarkerArray
        ros::Publisher mpc_walking_corridor_pub_;  // visualization_msgs/MarkerArray
        ros::Publisher mpc_walking_corridor_debug_pub_; // std_msgs/String
        ros::Publisher mpc_convex_corridor_pub_;   // visualization_msgs/MarkerArray
        ros::Publisher mpc_convex_corridor_debug_pub_; // std_msgs/String
        ros::Publisher risk_field_pub_;   // sensor_msgs::PointCloud2 (risk > hard_threshold)
        ros::Publisher risk_halo_pub_;    // sensor_msgs::PointCloud2 (halo component only)
        ros::Publisher cmd_vel_pub_;      // geometry_msgs::Twist for Gazebo
        ros::Publisher steer_pub_;        // std_msgs::Float64 steering joint angle
        ros::Publisher sim_odom_pub_;
        tf::TransformListener tf_listener_;

        /*---------- helper function -----------*/
        bool callAstarPlan();
        bool callKinodynamicAstarPlan();
        void loadSimPath();     // 从当前planner提取路径到sim_path_
        void stepSimMotion();   // 沿sim_path_推进odom (所有planner通用)
        void mpcSimInit();
        bool mpcSimStep();
        bool kinematicMppiSimStep();
        void publishSimOdom();
        void generateGlobalWaypoints();
        void reanchorWaypoint(const Eigen::Vector2d& robot_pos);
        void publishFovRange();
        void publishCurrentWaypoint();
        void publishWaypointsList();
        void publishRiskField(const std::vector<Eigen::Vector3d>& obs_pos,
                              const std::vector<Eigen::Vector3d>& obs_vel);
        void displayAstar();
        void displayKinastar();
        void displayMpcPlan();
        void publishMpcPath();
        double getPathLen(vector<Eigen::Vector2d> list);
        double getPathLen(vector<Eigen::Vector3d> list);
        void publishKinodynamicAstarPath();
        void publishAstarPath();

        void changeFSMExecState(FSM_STATE new_state);
        double QuatenionToYaw(geometry_msgs::Quaternion ori);
        double QuatenionToYaw(Eigen::Quaterniond ori);
        bool shouldIgnoreDuplicateGoal(const Eigen::Vector2d& goal, double yaw, const char* source);
        Eigen::Vector2d computeInteractionPathForward(const Eigen::Vector3d& current_com) const;
        double estimateInteractionRobotSpeed() const;
        void updateInteractionDebug(const Eigen::Vector3d& current_com,
                                    const std::vector<Eigen::Vector3d>& obs_pos,
                                    const std::vector<Eigen::Vector3d>& obs_vel,
                                    const std::vector<Eigen::Vector3d>& obs_size);
        void publishInteractionState();
        DynamicWalkingCorridor::Result updateAndPublishWalkingCorridor(
                                             const Eigen::Vector3d& current_pose,
                                             const std::vector<Eigen::Vector3d>& obs_pos,
                                             const std::vector<Eigen::Vector3d>& obs_vel,
                                             const std::vector<Eigen::Vector3d>& obs_size);
        TimedTrajectory buildWalkingCorridorNominalTrajectory(
                                             const Eigen::Vector3d& current_pose) const;
        std::vector<Eigen::Vector2d> buildWalkingCorridorReferencePath(
                                             const Eigen::Vector3d& current_pose) const;
        void publishWalkingCorridor(const DynamicWalkingCorridor::Result& result);
        ConvexCorridor::Result updateAndPublishConvexCorridor(
                                             const Eigen::Vector3d& current_pose,
                                             const std::vector<Eigen::Vector3d>& obs_pos,
                                             const std::vector<Eigen::Vector3d>& obs_vel,
                                             const std::vector<Eigen::Vector3d>& obs_size);
        void publishConvexCorridor(const ConvexCorridor::Result& result);
        const char* interactionSceneName(InteractionScene scene) const;
        const char* interactionModeName(InteractionMode mode) const;

        /*---------- ROS function -----------*/
        void execFSMCallback(const ros::TimerEvent &e);
        void checkCollisionCallback(const ros::TimerEvent &e);
        void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void waypointCallback(const nav_msgs::PathConstPtr &msg);
        void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
        void startCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &start);
        void dynamicObstaclesCallback(const onboard_detector::DynamicObstacles::ConstPtr &msg);
        void publishDynamicObstacleBodies(const onboard_detector::DynamicObstacles::ConstPtr &msg);

    public:
        PlannerManager(int planner = 2)
        {
            planner_ = planner;
            simulation_ = false;
            mpc_sim_state_ = MPC_IDLE;
            sim_path_idx_ = 0;
            sim_speed_ = 0.5;
        }
        ~PlannerManager();
        void init(ros::NodeHandle &nh);
        void Param_init(ros::NodeHandle &nh);

        void drawBspline(NonUniformBspline& bspline, double size, const Eigen::Vector4d& color,
                   bool show_ctrl_pts = false, double size2 = 0.1,
                   const Eigen::Vector4d& color2 = Eigen::Vector4d(1, 1, 0, 1), int id1 = 0,
                   int id2 = 0);

        void displaySphereList(const vector<Eigen::Vector3d>& list);

        PlanParameters pp_;
    };

} // namespace cane_planner

#endif
