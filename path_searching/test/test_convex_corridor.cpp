#include <gtest/gtest.h>
#include <path_searching/convex_corridor.h>
#include <iostream>
using C=cane_planner::ConvexCorridor;
using V=Eigen::Vector2d;
using Path=std::vector<V>;
static C::Grid grid(double offset=0.) { C::Grid g;g.origin=V::Constant(offset);g.width=48;g.height=24;g.resolution=.25;g.blocked.assign(48*24,0);return g; }
static C::Result run(const Path&p,const C::Grid&g,C::Config cfg=C::Config()) { C c;c.setConfig(cfg);return c.buildStatic(p,g); }
static void good(const C::Result&r) {ASSERT_TRUE(r.feasible)<<C::failureReasonName(r.failure_reason)<<" s="<<r.failure_s;ASSERT_FALSE(r.segments.empty());for(auto&s:r.segments){EXPECT_GE(s.vertices.size(),3u);EXPECT_GT(C::polygonArea(s.vertices),0.);} }
TEST(Firi, DenseStraight) {Path dense;for(int i=0;i<=500;++i)dense.push_back(V(1+i*.02,3));C::Config c;c.max_regions=600;auto a=run({V(1,3),V(11,3)},grid()),b=run(dense,grid(),c);good(a);good(b);EXPECT_LE(a.segments.size(),4u);EXPECT_LE(b.segments.size(),4u);std::cout<<"straight raw=10/500 final="<<a.segments.size()<<"/"<<b.segments.size()<<" seconds="<<a.elapsed_seconds<<"/"<<b.elapsed_seconds<<'\n';}
TEST(Firi, Corners) {for(auto&p:std::vector<Path>{{V(1,1),V(1,5),V(11,5)},{V(1,1),V(5,5),V(10,1)},{V(1,1),V(4,1),V(4,5),V(8,5),V(8,1),V(11,1)}}){auto r=run(p,grid());good(r);EXPECT_GT(r.min_overlap_area,0.);EXPECT_GE(r.min_overlap_depth,1e-6);}}
TEST(Firi, WholeCellDetour) {auto g=grid();for(int y=6;y<17;++y)for(int x=20;x<24;++x)g.blocked[y*48+x]=1;auto r=run({V(1,2),V(4,2),V(4,5),V(8,5),V(11,2)},g);good(r);for(auto&s:r.segments){for(int y=6;y<17;++y)for(int x=20;x<24;++x){bool sep=false;for(auto&h:s.halfspaces){double gap=1e9;for(auto&p:Path{V(x*.25,y*.25),V((x+1)*.25,y*.25),V((x+1)*.25,(y+1)*.25),V(x*.25,(y+1)*.25)})gap=std::min(gap,h.normal.dot(p)-h.offset);sep|=gap>=.002-1e-7;}EXPECT_TRUE(sep);}}std::cout<<"detour seconds="<<r.elapsed_seconds<<" regions="<<r.segments.size()<<std::endl;}
TEST(Firi, NearWallNarrow) {auto g=grid();for(int y=0;y<24;++y)if(y<10||y>=13)for(int x=0;x<48;++x)g.blocked[y*48+x]=1;good(run({V(1,2.65),V(11,2.65)},g));}
TEST(Firi, GeometricOnlyThinPassage) {C::Grid g;g.width=220;g.height=10;g.resolution=.05;g.blocked.assign(2200,1);for(int x=0;x<220;++x)g.blocked[4*220+x]=0;good(run({V(.5,.225),V(10,.225)},g));}
TEST(Firi, CellCrossingNotCenter) {auto g=grid();g.blocked[8*48+20]=1;auto r=run({V(1,2.02),V(11,2.02)},g);EXPECT_FALSE(r.feasible);EXPECT_TRUE(r.segments.empty());}
TEST(Firi, UnknownAndMapEdge) {auto g=grid();for(int y=0;y<24;++y)g.blocked[y*48+20]=255;EXPECT_FALSE(run({V(1,3),V(11,3)},g).feasible);for(auto&p:std::vector<Path>{{V(-.1,1),V(3,1)},{V(1,1),V(13,1)},{V(0,1),V(1,1)}})EXPECT_FALSE(run(p,grid()).feasible);}
TEST(Firi, InvalidAndBudgets) {for(auto&p:std::vector<Path>{{},{V(1,1)},{V(1,1),V(1,1)},{V(1,1),V(NAN,2)}})EXPECT_FALSE(run(p,grid()).feasible);C::Config c;c.max_regions=1;EXPECT_EQ(run({V(1,3),V(11,3)},grid(),c).failure_reason,C::FailureReason::BUDGET);c=C::Config();c.max_seconds=1e-12;EXPECT_EQ(run({V(1,3),V(11,3)},grid(),c).failure_reason,C::FailureReason::BUDGET);c=C::Config();c.max_segment_length=10;EXPECT_EQ(run({V(1,3),V(11,3)},grid(),c).failure_reason,C::FailureReason::INVALID_INPUT);c=C::Config();c.optimizer_iterations=1;EXPECT_FALSE(run({V(1,3),V(11,3)},grid(),c).feasible);}
TEST(Firi, NoCutCornerGap) {auto g=grid();g.blocked[8*48+20]=1;EXPECT_FALSE(run({V(1,2.02),V(11,2.02),V(1,2.02)},g).feasible);}
TEST(Firi, TranslationAndAreaThreshold) {auto base=run({V(1,3),V(11,3)},grid());good(base);for(double o:{-9000.,9000.}){auto r=run({V(1+o,3+o),V(11+o,3+o)},grid(o));good(r);EXPECT_EQ(r.segments.size(),base.segments.size());EXPECT_NEAR(r.min_overlap_area,base.min_overlap_area,1e-6);}for(double o:{0.,9000.,1e9}){Path v{V(o,o),V(o+3.098,o),V(o+3.098,o+5.996),V(o,o+5.996)};EXPECT_NEAR(C::polygonArea(v),3.098*5.996,2e-6);EXPECT_EQ(C::polygonArea({V(o,o),V(o+1,o+1),V(o+2,o+2)}),0.);}}
TEST(Firi, ScaleEnvelope) {EXPECT_EQ(run({V(1e9+1,1e9+3),V(1e9+11,1e9+3)},grid(1e9)).failure_reason,C::FailureReason::UNSUPPORTED_SCALE);auto g=grid();g.resolution=.001;EXPECT_EQ(run({V(1,1),V(2,1)},g).failure_reason,C::FailureReason::UNSUPPORTED_SCALE);}

TEST(Firi, OriginalEdgeEndpointCertificates) {
    const Path path{V(1,1),V(1,4),V(4,4),V(5,3),V(5,1)};
    auto r=run(path,grid());good(r);
    std::vector<double> arc{0.};for(size_t i=1;i<path.size();++i)arc.push_back(arc.back()+(path[i]-path[i-1]).norm());
    double covered=0.;
    for(const auto& region:r.segments) {
        EXPECT_GE(region.s0,covered);covered=region.s1;
        if(region.s1==region.s0)continue;
        size_t edge=0;while(edge+1<arc.size()&&arc[edge+1]<=region.s0+1e-12)++edge;
        ASSERT_LT(edge+1,path.size());EXPECT_LE(region.s1,arc[edge+1]+1e-12);
        EXPECT_LE(region.s1-region.s0,1.+1e-12);
        for(double s:{region.s0,region.s1}) {
            const V point=path[edge]+(path[edge+1]-path[edge])*((s-arc[edge])/(arc[edge+1]-arc[edge]));
            EXPECT_TRUE(C::contains(region,point,1e-8));
        }
    }
    EXPECT_NEAR(covered,arc.back(),1e-12);
}
TEST(Firi, DensePathBudgetRemainsBounded) {
    Path path;for(int i=0;i<=500;++i)path.push_back(V(1+i*.02,3));
    const auto r=run(path,grid());EXPECT_FALSE(r.feasible);
    EXPECT_EQ(r.failure_reason,C::FailureReason::BUDGET);EXPECT_TRUE(r.segments.empty());
}
int main(int argc,char**argv){testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
