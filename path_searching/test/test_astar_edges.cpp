#include <gtest/gtest.h>
#include <path_searching/astar.h>
#include <iomanip>
#include <path_searching/corridor_failure_snapshot.h>
#include <chrono>
#ifndef ASTAR_EDGE_FIXTURE_DIR
#define ASTAR_EDGE_FIXTURE_DIR "path_searching/test/fixtures/corridor_failures/"
#endif

namespace fast_planner {
class SdfProducerTestAccess {
public:
    static void update(SDFMap& map) { map.clearAndInflateLocalMap(); map.updateESDF3d(); }
};
}
namespace cane_planner {
class StaticMpcTestAccess {
public:
    static CollisionDetection::Ptr map(const ConvexCorridor::Grid& g) {
        auto c = std::make_shared<CollisionDetection>();
        c->static_global_map_ready_ = true;
        c->static_global_esdf_ready_ = false;
        c->static_origin_ = g.origin;
        c->static_size_ = Eigen::Vector2i(g.width,g.height);
        c->static_map_resolution_ = g.resolution;
        c->static_map_inv_resolution_ = 1./g.resolution;
        c->static_require_known_region_ = false;
        c->static_inflated_.resize(g.blocked.size());
        c->static_distance_.assign(g.blocked.size(), 1.0);
        for(int y=0;y<g.height;++y) for(int x=0;x<g.width;++x)
            c->static_inflated_[c->staticToAddress(Eigen::Vector2i(x,y))] = g.blocked[y*g.width+x];
        return c;
    }
    static CollisionDetection::Ptr local() {
        ros::NodeHandle nh("/astar_local_clearance_test");
        nh.setParam("sdf_map/resolution",.1);
        nh.setParam("sdf_map/map_size_x",6.);nh.setParam("sdf_map/map_size_y",6.);nh.setParam("sdf_map/map_size_z",4.);
        nh.setParam("sdf_map/ground_height",-1.);
        nh.setParam("sdf_map/local_update_range_x",5.5);nh.setParam("sdf_map/local_update_range_y",5.5);nh.setParam("sdf_map/local_update_range_z",4.5);
        nh.setParam("sdf_map/max_ray_length",5.);nh.setParam("sdf_map/input_min_height",.15);nh.setParam("sdf_map/input_max_height",2.6);
        nh.setParam("sdf_map/obstacles_inflation",.1);nh.setParam("map_ros/is_simulation",false);nh.setParam("map_ros/skip_pixel",2);
        auto c=std::make_shared<CollisionDetection>();c->sdf_map_.reset(new SDFMap);c->sdf_map_->initMap(nh);
        c->slice_height_=0.;c->margin_=.1;
        pcl::PointCloud<pcl::PointXYZ> cloud;cloud.push_back(pcl::PointXYZ(1.05,.05,.45));
        cloud.push_back(pcl::PointXYZ(1.05,1.05,.05)); // Default-filtered, not an extra unknown obstacle.
        for(int i=0;i<3;++i)c->sdf_map_->inputPointCloud(cloud,cloud.size(),Eigen::Vector3d(.05,.05,.45));
        fast_planner::SdfProducerTestAccess::update(*c->sdf_map_);return c;
    }
    static void knownOnly(CollisionDetection& c) {
        c.static_require_known_region_=true;
        c.static_known_inflated_.resize(c.static_inflated_.size());
        for(size_t i=0;i<c.static_inflated_.size();++i) {
            c.static_known_inflated_[i]=!c.static_inflated_[i]; c.static_inflated_[i]=0;
        }
        c.static_distance_.assign(c.static_inflated_.size(),100.);
    }
    static void selectEsdf(CollisionDetection& c, const ConvexCorridor::Grid& g) {
        c.static_global_esdf_ready_=true;
        c.global_esdf_resolution_=g.resolution; c.global_esdf_inv_resolution_=1./g.resolution;
        c.global_esdf_origin_=Eigen::Vector3d(g.origin.x(),g.origin.y(),0.);
        c.global_esdf_size_=Eigen::Vector3i(g.width,g.height,1);
        c.global_esdf_query_min_height_=c.global_esdf_query_max_height_=g.resolution*.5;
        c.global_esdf_safe_distance_=.2;
        c.global_esdf_distance_.assign(g.blocked.size(),100.);
        for(int y=0;y<g.height;++y) for(int x=0;x<g.width;++x)
            c.global_esdf_distance_[c.globalToAddress(Eigen::Vector3i(x,y,0))]=g.blocked[y*g.width+x] ? .1 : 100.;
        c.static_inflated_.assign(g.blocked.size(),1); // Opposite lower-priority backend.
    }
};
class AstarEdgeTestAccess {
public:
    static void init(Astar& a, const ConvexCorridor::Grid& g) {
        a.setCollision(StaticMpcTestAccess::map(g));
        a.resolution_=.1; a.inv_resolution_=10.; a.inv_time_resolution_=0.;
        a.origin_=g.origin; a.map_max_2d_=g.origin+g.resolution*Eigen::Vector2d(g.width,g.height);
        a.lambda_heu_=1.; a.tie_breaker_=1.0001; a.horizon_=100.; a.w_clearance_=0.;
        a.allocate_num_=100000; a.path_node_pool_.resize(a.allocate_num_);
        for(auto& p:a.path_node_pool_) p=new Node;
        a.use_node_num_=a.iter_num_=0;
        a.setCorridorEdgeClearance(.002);
    }
    static void preference(Astar& a, double weight=1., double sigma=.8, double resolution=.1, double lambda=1.) {
        a.w_clearance_=weight; a.clearance_sigma_=sigma; a.resolution_=resolution;
        a.inv_resolution_=1./resolution; a.lambda_heu_=lambda;
    }
    static void horizon(Astar& a, double value) { a.horizon_=value; }
    static int expanded(const Astar& a) { return a.iter_num_; }
    static double sigma(const Astar& a) { return a.clearance_sigma_; }
    static double cost(Astar& a, const Eigen::Vector2d& u, const Eigen::Vector2d& v) { return a.corridorEdgeCost(u,v); }
    static double heuristic(Astar& a, const Eigen::Vector2d& u, const Eigen::Vector2d& v) { return a.corridorHeuristic(u,v); }
    static bool edge(Astar& a, const Eigen::Vector2d& u, const Eigen::Vector2d& v) { return a.corridorEdgeFree(u,v); }
};
}
using namespace cane_planner;
static ConvexCorridor::Grid grid() {
    ConvexCorridor::Grid g; g.origin=Eigen::Vector2d(-4,-2); g.resolution=.1;g.width=60;g.height=40;
    g.blocked.assign(g.width*g.height,0);g.blocked[12*g.width+13]=1;return g;
}
TEST(AstarEdges, ExactDiagonalAndClearance) {
    auto g=grid(); Astar a; AstarEdgeTestAccess::init(a,g);
    const Eigen::Vector2d u(-2.6340717792510984,-.66673145294189384),v(-2.5340717792510983,-.7667314529418938);
    EXPECT_FALSE(AstarEdgeTestAccess::edge(a,u,v));
    EXPECT_TRUE(AstarEdgeTestAccess::edge(a,u,u+Eigen::Vector2d(.1,0)));
    EXPECT_TRUE(AstarEdgeTestAccess::edge(a,u,u+Eigen::Vector2d(.1,.1)));
    EXPECT_FALSE(AstarEdgeTestAccess::edge(a,{-2.8,-.6},{-2.6,-.8}));
    EXPECT_FALSE(AstarEdgeTestAccess::edge(a,{-2.8,-.699},{-2.5,-.699}));
    EXPECT_TRUE(AstarEdgeTestAccess::edge(a,{-2.8,-.697},{-2.5,-.697}));
    EXPECT_FALSE(AstarEdgeTestAccess::edge(a,{-2.8,-.75},{-2.5,-.75}));
    EXPECT_FALSE(AstarEdgeTestAccess::edge(a,{-4,-1},{-3.9,-1}));
    EXPECT_TRUE(AstarEdgeTestAccess::edge(a,u,u));
    a.setCollision(std::make_shared<CollisionDetection>());
    EXPECT_FALSE(a.search(u,v));
}
TEST(AstarEdges, SearchReroutesAndCorridorAccepts) {
    auto g=grid(); Astar a; AstarEdgeTestAccess::init(a,g);
    const Eigen::Vector2d u(-2.6340717792510984,-.66673145294189384),v(-2.5340717792510983,-.7667314529418938);
    ASSERT_TRUE(a.search(u,v));auto path=a.getPath();ASSERT_GE(path.size(),3u);
    EXPECT_EQ(path.front(),u);EXPECT_EQ(path.back(),v);
    for(size_t i=1;i<path.size();++i) EXPECT_TRUE(AstarEdgeTestAccess::edge(a,path[i-1],path[i]));
    ConvexCorridor c; EXPECT_FALSE(c.buildStatic({u,v},g).feasible); EXPECT_TRUE(c.buildStatic(path,g).feasible);
    a.reset();ASSERT_TRUE(a.search(u,u));EXPECT_EQ(a.getPath().size(),1u);
}
TEST(AstarEdges, LatestCapturedGridReroute) {
    CorridorFailureSnapshot s;std::string error;
    ASSERT_TRUE(readCorridorFailureSnapshot(ASTAR_EDGE_FIXTURE_DIR "failure-9ziyMV",s,error))<<error;
    ConvexCorridor c;c.setConfig(s.config);EXPECT_FALSE(c.buildStatic(s.path,s.grid).feasible);
    Astar a;AstarEdgeTestAccess::init(a,s.grid);a.setCorridorEdgeClearance(s.config.clearance);
    ASSERT_GT(s.path.size(),59u);EXPECT_FALSE(AstarEdgeTestAccess::edge(a,s.path[58],s.path[59]));
    auto begin=std::chrono::steady_clock::now();
    ASSERT_TRUE(a.search(s.path.front(),s.path.back()));auto path=a.getPath();
    double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
    ASSERT_EQ(path.front(),s.path.front());ASSERT_EQ(path.back(),s.path.back());
    for(size_t i=1;i<path.size();++i) ASSERT_TRUE(AstarEdgeTestAccess::edge(a,path[i-1],path[i]));
    auto result=c.buildStatic(path,s.grid);EXPECT_TRUE(result.feasible)<<ConvexCorridor::diagnosticsText(result);
    std::cout<<"reroute old_edges="<<s.path.size()-1<<" new_edges="<<path.size()-1<<" regions="<<result.segments.size()<<" search_ms="<<ms<<"\n";
    for(const auto& p:path)std::cout<<p.transpose()<<"\n";
}
static ConvexCorridor::Grid passage(double native=.1, double width=2.) {
    ConvexCorridor::Grid g; g.origin={-1.,-1.};g.resolution=native;
    g.width=static_cast<int>(8./native);g.height=static_cast<int>(4./native);
    g.blocked.assign(g.width*g.height,0);
    for(int y=0;y<g.height;++y) for(int x=0;x<g.width;++x) {
        const double py=g.origin.y()+(y+.5)*native;
        if(py<0. || py>width)g.blocked[y*g.width+x]=1;
    }
    return g;
}
static double length(const std::vector<Eigen::Vector2d>& path) {
    double result=0.;for(size_t i=1;i<path.size();++i)result+=(path[i]-path[i-1]).norm();return result;
}
static double cost(Astar& a,const std::vector<Eigen::Vector2d>& path) {
    double result=0.;for(size_t i=1;i<path.size();++i)result+=AstarEdgeTestAccess::cost(a,path[i-1],path[i]);return result;
}
TEST(AstarClearance, MetreCostHeuristicAndFullEdgeIntegral) {
    auto g=passage(); Astar a;AstarEdgeTestAccess::init(a,g);AstarEdgeTestAccess::preference(a);
    EXPECT_DOUBLE_EQ(AstarEdgeTestAccess::cost(a,{1.,.85},{1.4,1.15}),.5);
    EXPECT_DOUBLE_EQ(AstarEdgeTestAccess::heuristic(a,{1.,1.},{1.3,1.4}),.5);
    const double t=1.-(.2-.002)/.8;
    EXPECT_NEAR(AstarEdgeTestAccess::cost(a,{1.,.2},{2.,.2}),1.+t*t,1e-12);
    double fine=0.; const Eigen::Vector2d u(1.,.1),v(2.,1.);
    for(int i=0;i<1000;++i)fine+=AstarEdgeTestAccess::cost(a,u+(v-u)*(i/1000.),u+(v-u)*((i+1)/1000.));
    EXPECT_NEAR(AstarEdgeTestAccess::cost(a,u,v),fine,1e-4);
    EXPECT_NEAR(AstarEdgeTestAccess::cost(a,u,v),AstarEdgeTestAccess::cost(a,v,u),1e-12);
    AstarEdgeTestAccess::preference(a,0.);
    EXPECT_NEAR(AstarEdgeTestAccess::cost(a,u,v),(v-u).norm(),1e-12);
}
TEST(AstarClearance, OpenSpaceLengthAndExactGoal) {
    auto g=passage();Astar a;AstarEdgeTestAccess::init(a,g);AstarEdgeTestAccess::preference(a);
    const Eigen::Vector2d u(.13,1.),v(5.37,1.);
    ASSERT_TRUE(a.search(u,v));const auto p=a.getPath();EXPECT_EQ(p.front(),u);EXPECT_EQ(p.back(),v);
    EXPECT_NEAR(length(p),(v-u).norm(),1e-10);
    for(size_t i=1;i<p.size();++i)EXPECT_GT((p[i]-p[i-1]).norm(),1e-8);
}
TEST(AstarClearance, WidePassagePrefersInteriorAndNarrowRemainsOpen) {
    for(double width:{2.,.4}) {
        auto g=passage(.1,width);Astar a;AstarEdgeTestAccess::init(a,g);
        const Eigen::Vector2d u(.15,.15),v(5.15,.15);
        ASSERT_TRUE(a.search(u,v));const auto base=a.getPath();a.reset();AstarEdgeTestAccess::preference(a);
        ASSERT_TRUE(a.search(u,v));const auto preferred=a.getPath();
        EXPECT_LE(cost(a,preferred),cost(a,base)+1e-8);
        double middle=0.;int n=0;
        for(const auto& p:preferred)if(p.x()>2. && p.x()<3.){middle+=std::min(p.y(),width-p.y());++n;}
        ASSERT_GT(n,0);
        if(width>1.) {EXPECT_GT(middle/n,.55);EXPECT_LT(length(preferred),length(base)*1.15);}
        else EXPECT_LT(middle/n,.8);
        for(size_t i=1;i<preferred.size();++i)EXPECT_TRUE(AstarEdgeTestAccess::edge(a,preferred[i-1],preferred[i]));
    }
}
TEST(AstarClearance, NativeAndSearchResolutionSensitivity) {
    double reference_cost=0.,reference_length=0.;
    for(double native:{.1,.05}) for(double step:{.1,.05}) {
        auto g=passage(native);Astar a;AstarEdgeTestAccess::init(a,g);AstarEdgeTestAccess::preference(a,1.,.8,step);
        const double c=AstarEdgeTestAccess::cost(a,{1.,.2},{2.,.2});
        if(reference_cost==0.)reference_cost=c;EXPECT_NEAR(c,reference_cost,1e-12);
        ASSERT_TRUE(a.search({.15,.15},{5.15,.15}));const double l=length(a.getPath());
        if(reference_length==0.)reference_length=l;EXPECT_NEAR(l,reference_length,.15);
    }
}
TEST(AstarClearance, SameSelectedBlockedLayerNotRawDistance) {
    auto g=passage();Astar a;AstarEdgeTestAccess::init(a,g);AstarEdgeTestAccess::preference(a);
    const double expected=AstarEdgeTestAccess::cost(a,{1.,.2},{2.,.2});
    auto c=StaticMpcTestAccess::map(g);StaticMpcTestAccess::knownOnly(*c);a.setCollision(c);
    EXPECT_NEAR(AstarEdgeTestAccess::cost(a,{1.,.2},{2.,.2}),expected,1e-12);
    EXPECT_FALSE(AstarEdgeTestAccess::edge(a,{1.,-.1},{2.,-.1}));
    StaticMpcTestAccess::selectEsdf(*c,g);
    EXPECT_NEAR(AstarEdgeTestAccess::cost(a,{1.,.2},{2.,.2}),expected,1e-12);
    EXPECT_TRUE(AstarEdgeTestAccess::edge(a,{1.,.2},{2.,.2}));
    EXPECT_FALSE(AstarEdgeTestAccess::edge(a,{1.,-.1},{2.,-.1}));
}
TEST(AstarClearance, GoalConnectorComparedWithDijkstraAndInvalidConfig) {
    auto g=passage();Astar a;AstarEdgeTestAccess::init(a,g);AstarEdgeTestAccess::preference(a);
    ASSERT_TRUE(a.search({.15,.15},{3.173,.263}));const double with_heuristic=cost(a,a.getPath());
    a.reset();AstarEdgeTestAccess::preference(a,1.,.8,.1,0.);
    ASSERT_TRUE(a.search({.15,.15},{3.173,.263}));EXPECT_NEAR(cost(a,a.getPath()),with_heuristic,1e-9);
    for(double sigma:{0.,-.1,6.,std::numeric_limits<double>::quiet_NaN()}) {
        a.reset();AstarEdgeTestAccess::preference(a,1.,sigma);EXPECT_FALSE(a.search({.15,.15},{3.,.2}));
    }
}

TEST(AstarClearance, RealLocalProducerCostMatchesSelectedBins) {
    auto local=StaticMpcTestAccess::local();
    ConvexCorridor::Grid g;Eigen::Vector2i size;std::string reason;
    ASSERT_TRUE(local->getStaticCorridorGrid({-3.,-3.},{3.,3.},g.origin,g.resolution,size,g.blocked,reason));
    g.width=size.x();g.height=size.y();
    Astar a;AstarEdgeTestAccess::init(a,g);AstarEdgeTestAccess::preference(a);
    const Eigen::Vector2d u(.25,.25),v(.35,.25);
    const double expected=AstarEdgeTestAccess::cost(a,u,v);
    EXPECT_GT(expected,(v-u).norm());
    a.setCollision(local);
    EXPECT_NEAR(AstarEdgeTestAccess::cost(a,u,v),expected,1e-12);
    EXPECT_FALSE(AstarEdgeTestAccess::edge(a,{1.05,.05},{1.05,.05}));
    EXPECT_TRUE(AstarEdgeTestAccess::edge(a,{-1.05,-1.05},{-1.05,-1.05}));
}

TEST(AstarClearance, PreferenceGridBudgetRejectsBeforeExpansion) {
    ConvexCorridor::Grid g;g.origin={-6.,-6.};g.resolution=.01;g.width=g.height=1200;
    g.blocked.assign(g.width*g.height,0);
    Astar a;AstarEdgeTestAccess::init(a,g);AstarEdgeTestAccess::preference(a,1.,5.);
    const Eigen::Vector2d u(.005,.005),v(.105,.005);
    ASSERT_TRUE(AstarEdgeTestAccess::edge(a,u,v));
    EXPECT_FALSE(a.search(u,v));
    EXPECT_EQ(AstarEdgeTestAccess::expanded(a),0);
    // The same map/route works with a supported preference configuration.
    a.reset();AstarEdgeTestAccess::preference(a,1.,.8);
    ASSERT_TRUE(a.search(u,v));EXPECT_EQ(a.getPath().back(),v);
}
TEST(AstarClearance, HorizonRetainsCertifiedOffLatticeGoalWithPenalty) {
    auto g=passage(.1,.6);Astar a;AstarEdgeTestAccess::init(a,g);
    AstarEdgeTestAccess::preference(a);AstarEdgeTestAccess::horizon(a,5.);
    const Eigen::Vector2d u(.15,.15),v(5.23,.163);
    ASSERT_GT(AstarEdgeTestAccess::cost(a,{5.15,.15},v),(v-Eigen::Vector2d(5.15,.15)).norm());
    ASSERT_TRUE(a.search(u,v));const auto p=a.getPath();
    EXPECT_EQ(p.front(),u);EXPECT_EQ(p.back(),v);
    for(size_t i=1;i<p.size();++i)EXPECT_TRUE(AstarEdgeTestAccess::edge(a,p[i-1],p[i]));
}
TEST(AstarClearance, OmittedLegacySigmaFallback) {
    ros::NodeHandle nh("/astar_legacy_sigma_test");nh.deleteParam("astar/clearance_sigma");
    nh.setParam("astar/allocate_num",0); // This parameter-only test does not call init().
    Astar a;a.setParam(nh);EXPECT_DOUBLE_EQ(AstarEdgeTestAccess::sigma(a),.5);
    nh.setParam("astar/clearance_sigma",.8);a.setParam(nh);
    EXPECT_DOUBLE_EQ(AstarEdgeTestAccess::sigma(a),.8);
}

TEST(AstarRecovery, WholePolygonCrossingTangencyAndContainment) {
    const std::vector<Eigen::Vector2d> p{{0,0},{1,0},{1,1},{0,1}};
    ASSERT_TRUE(Astar::validRecoveryPolygon(p));
    EXPECT_FALSE(Astar::polygonEdgeFree({-.5,.5},{1.5,.5},p,.002));
    EXPECT_FALSE(Astar::polygonEdgeFree({.5,.5},{.5,.5},p,.002));
    EXPECT_FALSE(Astar::polygonEdgeFree({-.5,0},{1.5,0},p,0.));
    EXPECT_FALSE(Astar::polygonEdgeFree({-.5,-.001},{1.5,-.001},p,.002));
    EXPECT_TRUE(Astar::polygonEdgeFree({-.5,-.002},{1.5,-.002},p,.002));
    EXPECT_FALSE(Astar::validRecoveryPolygon({{0,0},{0,1},{1,0}}));
}
TEST(AstarRecovery, ExactGoalDetourAndEmptyOverlayEquivalence) {
    auto g=passage(.1,3.);Astar a;AstarEdgeTestAccess::init(a,g);AstarEdgeTestAccess::preference(a);
    const Eigen::Vector2d start(.13,1.5),goal(5.37,1.5);
    ASSERT_TRUE(a.search(start,goal));const auto plain=a.getPath();
    const auto deadline=[] {return std::chrono::steady_clock::now()+std::chrono::seconds(2);};
    const auto empty=a.searchRecovery(start,goal,{},deadline());
    ASSERT_EQ(Astar::RecoveryStatus::FOUND,empty.status);EXPECT_EQ(plain,empty.path);
    const std::vector<Eigen::Vector2d> p{{2.,1.},{3.,1.},{3.,2.},{2.,2.}};
    const auto r=a.searchRecovery(start,goal,p,deadline());
    ASSERT_EQ(Astar::RecoveryStatus::FOUND,r.status);EXPECT_EQ(start,r.path.front());EXPECT_EQ(goal,r.path.back());
    for(size_t i=1;i<r.path.size();++i) {
        EXPECT_TRUE(Astar::polygonEdgeFree(r.path[i-1],r.path[i],p,.002));
        EXPECT_TRUE(AstarEdgeTestAccess::edge(a,r.path[i-1],r.path[i]));
    }
    std::cout<<"PROFILE hard_s="<<r.profile.hard_seconds<<" soft_s="<<r.profile.soft_seconds<<" grid_s="<<r.profile.grid_seconds<<" hard_calls="<<r.profile.hard_calls<<" soft_calls="<<r.profile.soft_calls<<" grid_calls="<<r.profile.grid_calls<<" expanded="<<r.profile.expanded<<std::endl;
    std::cout<<"RECOVERY grid="<<g.width<<"x"<<g.height<<" seconds="<<r.seconds<<" edges="<<r.path.size()-1<<std::endl;
    a.reset();ASSERT_TRUE(a.search(start,goal));EXPECT_EQ(plain,a.getPath()); // no persistent overlay
}
TEST(AstarRecovery, BlockedEndpointsImpossiblePassageDeadlineAndPartialReject) {
    auto g=passage(.1,1.);Astar a;AstarEdgeTestAccess::init(a,g);
    const std::vector<Eigen::Vector2d> p{{2.,-.1},{3.,-.1},{3.,1.1},{2.,1.1}};
    auto end=[] {return std::chrono::steady_clock::now()+std::chrono::milliseconds(200);};
    for(const auto& pair:{std::make_pair(Eigen::Vector2d(2.5,.5),Eigen::Vector2d(5,.5)),std::make_pair(Eigen::Vector2d(0,.5),Eigen::Vector2d(2.5,.5))}) {
        const auto r=a.searchRecovery(pair.first,pair.second,p,end());EXPECT_NE(Astar::RecoveryStatus::FOUND,r.status);EXPECT_TRUE(r.path.empty());
    }
    auto r=a.searchRecovery({0,.5},{5,.5},p,end());EXPECT_NE(Astar::RecoveryStatus::FOUND,r.status);EXPECT_TRUE(r.path.empty());
    std::cout<<"NO_PATH seconds="<<r.seconds<<std::endl;
    r=a.searchRecovery({0,.5},{5,.5},p,std::chrono::steady_clock::now());EXPECT_EQ(Astar::RecoveryStatus::TIMEOUT,r.status);
    AstarEdgeTestAccess::horizon(a,.3);r=a.searchRecovery({0,.5},{5,.5},{},end());
    EXPECT_EQ(Astar::RecoveryStatus::INCOMPLETE_PATH,r.status);EXPECT_TRUE(r.path.empty());
}
TEST(AstarRecovery, SoftPruningFrozenInputsAndFloatTies) {
    for(double weight:{0.,1.}) {
        auto g=passage(.1,3.);Astar a;AstarEdgeTestAccess::init(a,g);AstarEdgeTestAccess::preference(a,weight);
        const std::vector<Eigen::Vector2d> polygon{{2.,1.},{3.,1.},{3.,2.},{2.,2.}};
        const auto r=a.searchRecovery({.13,1.5},{5.37,1.5},polygon,std::chrono::steady_clock::now()+std::chrono::seconds(2));
        ASSERT_EQ(Astar::RecoveryStatus::FOUND,r.status);
        EXPECT_GT(r.profile.soft_skipped,0u);
        EXPECT_GT(r.profile.soft_calls,0u); // improving/unseen candidates still costed
        std::cout<<std::setprecision(17)<<"PRUNING weight="<<weight<<" seconds="<<r.seconds<<" expanded="<<r.profile.expanded
                 <<" hard="<<r.profile.hard_calls<<" soft="<<r.profile.soft_calls<<" skipped="<<r.profile.soft_skipped
                 <<" cost="<<cost(a,r.path)<<" route=";
        for(const auto& p:r.path)std::cout<<p.x()<<","<<p.y()<<";";
        std::cout<<std::endl;
        for(size_t i=1;i<r.path.size();++i)EXPECT_TRUE(AstarEdgeTestAccess::edge(a,r.path[i-1],r.path[i]));
    }
    // Equality and adjacent floating values: monotonic lower sum cannot hide an improvement.
    for(double g:{0.,.1,1e10})for(double length:{0.,.1,std::nextafter(.1,1.)}) {
        const double lower=g+length;
        for(double factor:{1.,std::nextafter(1.,2.),2.})EXPECT_GE(g+length*factor,lower);
        EXPECT_FALSE(lower<std::nextafter(lower,-std::numeric_limits<double>::infinity()));
        EXPECT_TRUE(lower<std::nextafter(lower,std::numeric_limits<double>::infinity()));
    }
}
int main(int argc,char** argv) { ros::init(argc,argv,"test_astar_edges",ros::init_options::AnonymousName);testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS(); }
