// Include the implementation only in this isolated test target to exercise
// private solver/certificate boundaries without a production injection API.
#include "../src/convex_corridor.cpp"
#include <path_searching/corridor_failure_snapshot.h>
#include <gtest/gtest.h>
#include <thread>
namespace cane_planner {
namespace {
// Inject the optimizer boundary, not an objective exception: NLopt translates
// callback exceptions through FORCED_STOP, which is not a native -4 result.
struct RoundoffOptimizer {
    bool expired;
    void set_maxtime(double) {}
    nlopt::result last_optimize_result() const {return nlopt::ROUNDOFF_LIMITED;}
    nlopt::result optimize(std::vector<double>&,double&) {
        if(expired)std::this_thread::sleep_for(std::chrono::milliseconds(20));
        throw nlopt::roundoff_limited();
    }
};
std::vector<H> box() {return {{V(1,0),2},{V(-1,0),2},{V(0,1),2},{V(0,-1),2}};}
void candidate(std::vector<double> x,bool ellipse=true, bool expired=false) {
    auto end=Clock::now()+std::chrono::milliseconds(expired?10:1000);
    RoundoffOptimizer opt{expired};
    C::Diagnostics diag;
    try {
        solve(opt,x,end,diag,ellipse);
        auto hs=box();V center;M L;certifyMvie(hs,x,center,L);
        for(auto& h:hs)EXPECT_GE(slack(h,center)-(L.transpose()*h.normal).norm(),-1e-7);
        EXPECT_GT(L.determinant(),0.);
    } catch(const Failure&) {
        EXPECT_TRUE(diag.solver_status_valid);EXPECT_EQ(diag.solver_status,nlopt::ROUNDOFF_LIMITED);
        throw;
    }
    EXPECT_EQ(diag.solver_status,nlopt::ROUNDOFF_LIMITED);
    EXPECT_FALSE(diag.solver_exception.empty());
}
C::Segment rectangle(double x0,double x1,double y0,double y1,double s0,double s1) {
    C::Segment r;r.center=V((x0+x1)/2,(y0+y1)/2);r.s0=s0;r.s1=s1;
    r.halfspaces={{V(1,0),x1},{V(-1,0),-x0},{V(0,1),y1},{V(0,-1),-y0}};
    return r;
}
TEST(RegionPruning, OverlapCannotShortcutUncoveredCornerOrExcursion) {
    // End regions overlap and contain every original vertex, but not the
    // middle of the long diagonal edge: endpoint-only checks would pass.
    const Poly path{V(0,0),V(0,2),V(2,0)};
    const std::vector<double> arc{0,2,2+std::sqrt(8.)};
    std::vector<C::Segment> r{
        rectangle(-.2,.2,-.2,2.2,0,2),
        rectangle(-.2,2.2,-.2,2.2,2,arc.back()),
        rectangle(-.2,2.2,-.2,.2,arc.back(),arc.back())};
    auto end=Clock::now()+std::chrono::seconds(1);C::Diagnostics d;
    EXPECT_NO_THROW(overlap(r.front(),r.back()));
    EXPECT_FALSE(coversPath(path,arc,{&r.front(),&r.back()},0,arc.back(),end));
    pruneRegions(r,path,arc,end,d);EXPECT_EQ(r.size(),3u);
}
TEST(RegionPruning, DisjointAndPointTouchPairsCannotSkipMiddle) {
    for(double start:{1.,1.1}) {
        const Poly path{V(0,0),V(2,0)};const std::vector<double> arc{0,2};
        std::vector<C::Segment> r{rectangle(-.2,1.,-.2,.2,0,.5),
            rectangle(.4,1.6,-.2,.2,.5,1.5),rectangle(start,2.2,-.2,.2,1.5,2)};
        C::Diagnostics d;pruneRegions(r,path,arc,Clock::now()+std::chrono::seconds(1),d);
        EXPECT_EQ(r.size(),3u);
    }
}
TEST(RegionPruning, InflationStillProducesArbitraryPolygons) {
    C::Grid g;g.origin=V::Zero();g.resolution=.25;g.width=48;g.height=24;g.blocked.assign(48*24,0);
    for(int y=6;y<17;++y)for(int x=20;x<24;++x)g.blocked[y*48+x]=1;
    C::Diagnostics d;C::Config cfg;
    auto r=inflate(g,V(4,4),V(4,5),cfg,Clock::now()+std::chrono::seconds(1),d);
    EXPECT_GT(r.vertices.size(),4u);
}
TEST(RegionPruning, RemovesRedundancyAndHonorsGlobalDeadline) {
    const Poly path{V(0,0),V(2,0)};const std::vector<double> arc{0,2};
    std::vector<C::Segment> r{rectangle(-.2,1.2,-.2,.2,0,.5),
        rectangle(.4,1.6,-.2,.2,.5,1.5),rectangle(.8,2.2,-.2,.2,1.5,2)};
    C::Diagnostics d;
    try {pruneRegions(r,path,arc,Clock::now(),d);FAIL()<<"expired pruning accepted";}
    catch(const Failure& f){EXPECT_EQ(f.reason,C::FailureReason::BUDGET);}
    pruneRegions(r,path,arc,Clock::now()+std::chrono::seconds(1),d);
    ASSERT_EQ(r.size(),2u);EXPECT_EQ(r.front().s1,.5);EXPECT_EQ(r.back().s0,1.5);
    EXPECT_GE(overlap(r.front(),r.back()).second,1e-6);
}
TEST(SegmentConnection, PointTouchRequiresSameFiriConnection) {
    C::Grid grid;grid.origin=V(-3,-3);grid.resolution=.1;grid.width=grid.height=60;grid.blocked.assign(3600,0);
    C::Segment left,right;left.center=V(-.5,0);right.center=V(.5,0);
    left.halfspaces={{V(1,0),0},{V(-1,0),1},{V(0,1),1},{V(0,-1),1}};
    right.halfspaces={{V(1,0),1},{V(-1,0),0},{V(0,1),1},{V(0,-1),1}};
    EXPECT_THROW(overlap(left,right),Failure);
    C::Diagnostics diagnostics;C::Config cfg;
    auto connection=inflate(grid,V::Zero(),V::Zero(),cfg,Clock::now()+std::chrono::seconds(1),diagnostics);
    EXPECT_GE(overlap(left,connection).second,1e-6);
    EXPECT_GE(overlap(connection,right).second,1e-6);
    // Below numerical interior resolution remains rejected even with a connector.
    right.halfspaces[2].offset=1e-7;right.halfspaces[3].offset=1e-7;
    EXPECT_THROW(overlap(connection,right),Failure);
    grid.blocked[30*60+30]=1;
    EXPECT_THROW(inflate(grid,V::Zero(),V::Zero(),cfg,Clock::now()+std::chrono::seconds(1),diagnostics),Failure);
    EXPECT_THROW(inflate(grid,V::Zero(),V::Zero(),cfg,Clock::now(),diagnostics),Failure);
}
TEST(MvieRoundoff, ValidCandidateContinuesThroughShrinkCertificate) {
    EXPECT_NO_THROW(candidate({0,0,1,0,1}));
}
TEST(MvieRoundoff, InvalidCandidateRejected) {
    for(const auto& x:std::vector<std::vector<double>>{
        {0,0,0,0,1},{0,0,-1,0,1},{0,0,1,0,0},{0,0,1e-8,0,1},
        {0,0,3,0,1},{3,0,1,0,1},{0,0,1,4,1},
        {NAN,0,1,0,1},{0,0,INFINITY,0,1},{0,0,1,NAN,1}})
        EXPECT_THROW(candidate(x),Failure);
}
TEST(MvieRoundoff, SeparationStillRejectsRoundoff) {
    EXPECT_THROW(candidate({0,0,1,0,1},false),Failure);
}
TEST(MvieRoundoff, RoundoffDoesNotBypassDeadline) {
    try {candidate({0,0,1,0,1},true,true);FAIL()<<"deadline accepted";}
    catch(const Failure& f){EXPECT_EQ(f.reason,C::FailureReason::BUDGET);}
}
TEST(SeparationInitialization, RecordedRerouteEdge72) {
    const V seed(0.,.049999999999999822), other(0.,-.050000000000000266);
    const V center(-.71685447497541055,-1.5486267561859837);
    M L; L<<2.2811455022130658,0.,-.16279338537179217,1.4402017468221249;
    const Cell cell{{V(-1.2659282207489015,-.083268547058104048),
                     V(-1.1659282207489015,-.083268547058104048),
                     V(-1.1659282207489015,.016731452941895958),
                     V(-1.2659282207489015,.016731452941895958)}};
    C::Config cfg; C::Diagnostics diag;
    // Independent distance: vertical segment overlaps the cell's y interval.
    EXPECT_GT(1.1659282207489015,cfg.clearance);
    const auto plane=separate(cell,seed,other,center,L,cfg,Clock::now()+std::chrono::seconds(1),diag);
    EXPECT_FALSE(diag.solver_status_valid); // finite geometry, not an NLopt solve
    EXPECT_LE(plane.normal.dot(seed),plane.offset+1e-9);
    EXPECT_LE(plane.normal.dot(other),plane.offset+1e-9);
    for(const auto& v:cell) EXPECT_GE(plane.normal.dot(v)-plane.offset,cfg.clearance-1e-12);
}
TEST(AngularSeparation, SignedSupportDistantCellsAndActiveClearance) {
    const Cell cell{{V(0,0),V(.1,0),V(.1,.1),V(0,.1)}};
    C::Config cfg;
    for(double gap:{.002,.002000001,.115,1.165928}) {
        const V a(-gap,.02),b(-gap,.08);
        // Center can lie on the obstacle side of BOTH seed endpoints, even
        // inside the cell: signed support is a representation, not feasibility.
        for(const V& center:{V(-2,1),V(.05,.05),V(2,-1)}) {
            M L;L<<2,0,.3,.7; C::Diagnostics d;
            const auto end=Clock::now()+std::chrono::seconds(1);
            const auto h=separate(cell,a,b,center,L,cfg,end,d);
            EXPECT_LE(h.normal.dot(a),h.offset+1e-9);
            EXPECT_LE(h.normal.dot(b),h.offset+1e-9);
            for(const auto& v:cell) EXPECT_GE(h.normal.dot(v)-h.offset,cfg.clearance-1e-12);
            const auto again=separate(cell,a,b,center,L,cfg,end,d);
            EXPECT_EQ(h.normal,again.normal);EXPECT_EQ(h.offset,again.offset);
            EXPECT_FALSE(d.solver_status_valid);
        }
    }
}
TEST(AngularSeparation, FrozenM6wfx3InitialAndTerminalWitnesses) {
    const V origin(-2.8653592586517335761,-.64098930358886585523);
    const V a=V(-2.9153592586517333984,-.59098930358886581082)-origin;
    const V b=V(-2.8153592586517333096,-.69098930358886578862)-origin;
    const V center(.51152480504599062705,1.4704946517944332829);
    M L;L<<1.051834443087377835,0,.071635895908856425662,1.5258246427910688858;
    const Cell cell{{V(-2.6999999999999997335,-.79999999999999937828)-origin,
                     V(-2.5999999999999996447,-.79999999999999937828)-origin,
                     V(-2.5999999999999996447,-.69999999999999940048)-origin,
                     V(-2.6999999999999997335,-.69999999999999940048)-origin}};
    C::Config cfg;C::Diagnostics d;
    // The horizontal witness proves separation independently of either
    // optimizer iterate: the selected cell is >115 mm to the right of b.
    EXPECT_GT(cell[0].x()-b.x(),.115);
    const auto h=separate(cell,a,b,center,L,cfg,Clock::now()+std::chrono::seconds(1),d);
    EXPECT_LE(h.normal.dot(a),h.offset+1e-9);EXPECT_LE(h.normal.dot(b),h.offset+1e-9);
    for(const auto& v:cell) EXPECT_GE(h.normal.dot(v)-h.offset,cfg.clearance-1e-12);
    // Frozen initial and terminal directions: recomputed whole-cell support
    // diagnoses the initial iterate's 61 mm endpoint violation, while the
    // terminal iterate is metrically acceptable despite NLopt returning -4.
    for(const V& n:{V(-.31472252329701660978,-.94918371948193402954),
                    V(-.032531349607491245879,-.99947071557535649422)}) {
        double support=INFINITY;for(const auto& v:cell)support=std::min(support,n.dot(v));
        const double violation=std::max(n.dot(a),n.dot(b))-(support-cfg.clearance);
        if(n.x()<-.1) EXPECT_GT(violation,.06);
        else EXPECT_LT(violation,1e-9);
    }
}
TEST(AngularSeparation, IndependentUnshiftedObjectiveExtrema) {
    const Cell corner{{V(1,1),V(2,1),V(2,2),V(1,2)}};
    const Cell face{{V(1,-1),V(2,-1),V(2,1),V(1,1)}};
    C::Config cfg;M anisotropic;anisotropic<<2,0,0,1;
    auto check=[&](const Cell& cell,const V& seed,const V& center,const M& L,
                   double expected,const V& normal) {
        C::Diagnostics d;
        const auto h=separate(cell,seed,seed,center,L,cfg,Clock::now()+std::chrono::seconds(1),d);
        double support=INFINITY;
        for(const auto& v:cell) support=std::min(support,h.normal.dot(v-center));
        EXPECT_NEAR(support/(L.transpose()*h.normal).norm(),expected,1e-12);
        if(normal.norm()>0) { EXPECT_LT((h.normal-normal).norm(),1e-12); }
        EXPECT_LE(h.normal.dot(seed),h.offset+1e-9);
    };
    // Cauchy-Schwarz gives max (nx+ny)/sqrt(4 nx^2+ny^2)=sqrt(5)/2.
    // The optimum is strictly inside the corner's support sector. Subtracting
    // clearance in the objective would change this anisotropic optimum.
    check(corner,V::Zero(),V::Zero(),anisotropic,std::sqrt(5.)/2.,V(1,4).normalized());
    // min cell support = nx-|ny| <= 1, attained at the support switch ny=0.
    check(face,V::Zero(),V::Zero(),M::Identity(),1.,V(1,0));
    // The unconstrained 45-degree optimum violates this endpoint constraint.
    // The feasible-arc endpoint solves nx-.999 ny=clearance analytically.
    const V delta(1,-.999),axis=delta.normalized(),tangent(-axis.y(),axis.x());
    const double cosine=cfg.clearance/delta.norm();
    const V active=cosine*axis+std::sqrt(1.-cosine*cosine)*tangent;
    check(corner,V(0,1.999),V::Zero(),M::Identity(),active.x()+active.y(),active);
    // When center is a cell vertex, support is <=0 for every direction and
    // equals zero throughout the positive sector: the stationary vector is
    // zero, but sector/clearance boundaries still attain the optimum.
    check(corner,V::Zero(),V(1,1),anisotropic,0.,V::Zero());
}
TEST(AngularSeparation, WholeCellInteriorCrossingAndTimeoutReject) {
    const Cell cell{{V(0,0),V(1,0),V(1,1),V(0,1)}};
    C::Config cfg;C::Diagnostics d;
    // Both endpoints and every corner are away from the crossing location.
    try {separate(cell,V(-1,.01),V(2,.01),V(-2,0),M::Identity(),cfg,
                  Clock::now()+std::chrono::seconds(1),d);FAIL();}
    catch(const Failure& f){EXPECT_EQ(f.reason,C::FailureReason::REFERENCE_OCCUPIED);}
    try {separate(cell,V(-1,0),V(-1,1),V(-2,0),M::Identity(),cfg,Clock::now(),d);FAIL();}
    catch(const Failure& f){EXPECT_EQ(f.reason,C::FailureReason::BUDGET);}
}
TEST(MvieRoundoff, RecordedTurnCompletesAllCorridorCertificates) {
    CorridorFailureSnapshot s;std::string error;
    const std::string fixture=std::string(__FILE__).substr(0,std::string(__FILE__).find_last_of('/'))+
        "/fixtures/mvie_roundoff_turn.snapshot";
    ASSERT_TRUE(readCorridorFailureSnapshot(fixture,s,error))<<error;
    ASSERT_EQ(s.failure.diagnostics.solver_status,nlopt::ROUNDOFF_LIMITED);
    C builder;builder.setConfig(s.config);const auto r=builder.buildStatic(s.path,s.grid);
    ASSERT_TRUE(r.feasible)<<C::failureReasonName(r.failure_reason)<<C::diagnosticsText(r);
    EXPECT_GT(r.min_overlap_area,0.);
    EXPECT_GE(r.min_overlap_depth,1e-6);
    ASSERT_FALSE(r.segments.empty());EXPECT_TRUE(C::contains(r.segments.back(),s.path.back()));
    std::cout<<"recorded turn seconds="<<r.elapsed_seconds<<" regions="<<r.segments.size()<<'\n';
}
}
}
int main(int argc,char** argv){testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
