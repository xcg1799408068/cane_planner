#include <gtest/gtest.h>
#include <path_searching/pedestrian_polygon.h>
#include <path_searching/corridor_failure_snapshot.h>
#include <chrono>
#include <iostream>
using namespace cane_planner;
using V=Eigen::Vector2d;
using P=PedestrianPolygon;
using C=ConvexCorridor;
namespace {
C::Grid grid() {C::Grid g;g.origin=V(-5,-5);g.resolution=.1;g.width=g.height=100;g.blocked.assign(10000,0);return g;}
P::Observation observation(V p=V(0,1.5)) {P::Observation o;o.position=p;o.velocity=V(1,0);o.full_size=V(.5,.7);return o;}
bool inside(const P::Polygon& h,const V&p) {
    for(size_t i=0;i<h.size();++i) {const V a=h[(i+1)%h.size()]-h[i],b=p-h[i];if(a.x()*b.y()-a.y()*b.x() < -1e-9)return false;}return true;
}
void excluded(const C::Result&r,const P::Polygon&p,double clearance) {
    ASSERT_TRUE(r.feasible)<<C::failureReasonName(r.failure_reason);
    for(const auto&s:r.segments) {bool common=false;for(const auto&h:s.halfspaces) {
        bool all=true;for(const auto&v:p)all=all && h.normal.dot(v)-h.offset>=clearance-1e-7;common=common||all;
    }EXPECT_TRUE(common);}
}
}
TEST(PedestrianPolygon, ReverseAndZeroForceAlwaysContainBodyAndSafetyDisk) {
    P::Config c;const auto o=observation();
    for(const V& f:{V(-4,0),V(0,0),V(4,0)}) {
        const auto r=P::fromForce(o,f,c);ASSERT_TRUE(r.valid);
        for(const auto&p:r.body)EXPECT_TRUE(inside(r.hull,p));
        for(int i=0;i<360;++i)EXPECT_TRUE(inside(r.hull,o.position+c.safety_radius*V(std::cos(i*.0174532925199433),std::sin(i*.0174532925199433))));
        if(f.x()<0) { EXPECT_TRUE(inside(r.hull,o.position+V(-1.6,0))); }
    }
}
TEST(PedestrianPolygon, StationaryIsSymmetricRegardlessOfForce) {
    auto o=observation();o.velocity.setZero();P::Config c;
    const auto a=P::fromForce(o,V(-4,2),c),b=P::fromForce(o,V::Zero(),c);
    ASSERT_TRUE(a.valid);EXPECT_EQ(a.hull,b.hull);
    for(const auto&p:a.hull)EXPECT_TRUE(inside(a.hull,2*o.position-p));
}
TEST(PedestrianPolygon, InvalidInputAndBoundsReject) {
    auto o=observation();P::Config c;
    o.full_size.x()=-1;EXPECT_FALSE(P::fromForce(o,V::Zero(),c).valid);
    o=observation();o.velocity.x()=NAN;EXPECT_FALSE(P::fromForce(o,V::Zero(),c).valid);
    c.alpha=INFINITY;EXPECT_FALSE(P::validConfig(c));
    EXPECT_FALSE(P::build(observation(),Eigen::Vector3d::Zero(),C::Grid(),P::Config()).valid);
}
TEST(PedestrianPolygon, WallAndSystemRepulsionHaveCorrectDirectionAndFov) {
    auto g=grid();auto o=observation(V(0,0));P::Config c;c.system_gain=0;
    for(int y=0;y<g.height;++y)g.blocked[y*g.width+55]=1;
    auto r=P::build(o,Eigen::Vector3d(-3,0,0),g,c);ASSERT_TRUE(r.valid);EXPECT_LT(r.force.x(),0);
    EXPECT_NEAR(0,r.force.y(),1e-9);
    g=grid();c.wall_gain=0;c.system_gain=1;
    auto front=P::build(o,Eigen::Vector3d(1,0,0),g,c);
    auto rear=P::build(o,Eigen::Vector3d(-1,0,0),g,c);
    ASSERT_TRUE(front.valid);ASSERT_TRUE(rear.valid);
    EXPECT_LT(front.force.x(),0);EXPECT_GT(rear.force.x(),0);
    EXPECT_NEAR(front.force.norm(),2*rear.force.norm(),1e-9);
}
TEST(DynamicConvexCorridor, EmptyInputIsExactlyStatic) {
    C c;const std::vector<V> path{V(-2,0),V(0,0),V(2,0)};auto g=grid();
    const auto a=c.buildStatic(path,g),b=c.buildStatic(path,g,{});
    ASSERT_TRUE(a.feasible);ASSERT_TRUE(b.feasible);ASSERT_EQ(a.segments.size(),b.segments.size());
    for(size_t i=0;i<a.segments.size();++i)EXPECT_EQ(a.segments[i].vertices,b.segments[i].vertices);
}
TEST(DynamicConvexCorridor, WholeHullExclusionApproachBlockedDepartRecovery) {
    C c;auto g=grid();const std::vector<V> path{V(-2,0),V(0,0),V(2,0)};
    for(double y:{1.5,.9,0.,-.9,-1.5}) {
        auto o=observation(V(.5,y));o.velocity=V(0,-.4);
        auto hull=P::fromForce(o,V::Zero(),P::Config());ASSERT_TRUE(hull.valid);
        const auto r=c.buildStatic(path,g,{hull.hull});
        if(y==0) {EXPECT_FALSE(r.feasible);EXPECT_TRUE(r.segments.empty());EXPECT_EQ(C::FailureReason::DYNAMIC_REFERENCE_BLOCKED,r.failure_reason);}
        else excluded(r,hull.hull,c.getConfig().clearance);
    }
}
TEST(DynamicConvexCorridor, ContainingAndCrossingPolygonRejectWithoutVertexOnPath) {
    C c;const std::vector<V> path{V(-2,0),V(2,0)};auto g=grid();
    for(const P::Polygon& p:{P::Polygon{V(-3,-1),V(3,-1),V(3,1),V(-3,1)},
                            P::Polygon{V(-.1,-2),V(.1,-2),V(.1,2),V(-.1,2)}}) {
        auto r=c.buildStatic(path,g,{p});EXPECT_FALSE(r.feasible);EXPECT_TRUE(r.segments.empty());
        EXPECT_EQ(C::FailureReason::DYNAMIC_REFERENCE_BLOCKED,r.failure_reason);
    }
}
TEST(DynamicConvexCorridor, InvalidPolygonsFailClosed) {
    C c;const std::vector<V> path{V(-2,0),V(2,0)};auto g=grid();
    for(const P::Polygon& p:{P::Polygon{},P::Polygon{V(0,1),V(1,2),V(1,1)},
                            P::Polygon{V(0,1),V(NAN,2),V(1,1)}}) {
        const auto r=c.buildStatic(path,g,{p});EXPECT_FALSE(r.feasible);EXPECT_TRUE(r.segments.empty());
        EXPECT_EQ(C::FailureReason::DYNAMIC_DATA_INVALID,r.failure_reason);
    }
}
TEST(DynamicConvexCorridor, BoundedSinglePedestrianTiming) {
    C c;auto g=grid();const std::vector<V> path{V(-2,0),V(-1,0),V(0,0),V(1,0),V(2,0)};
    double static_ms=0,dynamic_ms=0;
    for(int i=0;i<10;++i) {
        const auto a=c.buildStatic(path,g);ASSERT_TRUE(a.feasible);static_ms+=1000*a.elapsed_seconds;
        const auto h=P::build(observation(),Eigen::Vector3d(-2,0,0),g,P::Config());ASSERT_TRUE(h.valid);
        const auto b=c.buildStatic(path,g,{h.hull});ASSERT_TRUE(b.feasible);dynamic_ms+=1000*b.elapsed_seconds;
    }
    std::cout<<"TIMING mean_static_ms="<<static_ms/10<<" mean_one_pedestrian_ms="<<dynamic_ms/10<<std::endl;
}
TEST(DynamicConvexCorridor, DynamicEvidenceCannotBecomeStaticReplay)
{
    C c;auto g=grid();const std::vector<V> path{V(-2,0),V(2,0)};
    const auto h=P::fromForce(observation(V(0,0)),V::Zero(),P::Config());
    auto result=c.buildStatic(path,g,{h.hull});ASSERT_TRUE(result.has_dynamic_input);
    // Even a numerical failure (ordinarily eligible) must not lose dynamic input.
    result.failure_reason=C::FailureReason::NUMERICAL_FAILURE;result.failure_position_valid=true;
    CorridorFailureCapture capture;std::string file,error;
    EXPECT_EQ(CorridorFailureCapture::Status::SKIPPED,capture.saveOnce("/invalid/not-used",g,path,c.getConfig(),result,file,error));
    EXPECT_EQ("DYNAMIC_INPUT_NOT_CAPTURED",error);EXPECT_TRUE(file.empty());
}
int main(int argc,char** argv) {
    ::testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();
}
