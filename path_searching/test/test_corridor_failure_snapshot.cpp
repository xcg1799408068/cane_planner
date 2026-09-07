#include <gtest/gtest.h>
#include <path_searching/corridor_failure_snapshot.h>
#include <nlopt.hpp>
#include <cmath>
#include <fstream>
#include <unistd.h>
using C=cane_planner::ConvexCorridor;
using V=Eigen::Vector2d;
using Capture=cane_planner::CorridorFailureCapture;
TEST(HistoricalSnapshot, EdgeSeeds) {
    for(const char* name:{"failure-T08XXn","failure-tXyExX","failure-inIHM2"}) {
        cane_planner::CorridorFailureSnapshot snapshot;std::string error;
        const std::string source=__FILE__;
        const std::string root=source.substr(0,source.find_last_of('/'))+"/fixtures/corridor_failures/";
        ASSERT_TRUE(cane_planner::readCorridorFailureSnapshot(root+name,snapshot,error))<<error;
        EXPECT_DOUBLE_EQ(snapshot.config.max_segment_length,1.);
        C corridor;corridor.setConfig(snapshot.config);
        const auto r=corridor.buildStatic(snapshot.path,snapshot.grid);
        {
            std::cout<<name<<" original_edges="<<snapshot.path.size()-1<<" baseline_raw=81 final="<<r.segments.size()<<" seconds="<<r.elapsed_seconds<<std::endl;
            ASSERT_TRUE(r.feasible)<<C::failureReasonName(r.failure_reason);
            EXPECT_GE(r.min_overlap_depth,1e-6);
            for(const auto& point:snapshot.path) {
                bool covered=false;for(const auto& region:r.segments)covered|=C::contains(region,point,1e-8);
                EXPECT_TRUE(covered);
            }
        }
    }
}
namespace {
C::Grid grid() { C::Grid g;g.origin=V(-.125,-.25);g.resolution=.25;g.width=48;g.height=24;g.blocked.assign(48*24,0);return g; }
const std::vector<V> path{V(1.123456789012345,3),V(2,3),V(11,3)};
C::Result failure(const C::Grid& g,C::Config c) { C builder;builder.setConfig(c);return builder.buildStatic(path,g); }
class Snapshot : public testing::Test {
protected:
    std::string directory;
    std::vector<std::string> files;
    void SetUp() override {char name[]="/tmp/corridor-snapshot-test-XXXXXX";auto p=::mkdtemp(name);ASSERT_NE(p,nullptr);directory=p;}
    void TearDown() override {for(const auto& f:files)::unlink(f.c_str());::rmdir(directory.c_str());}
};
}
TEST(CorridorDiagnostics, SolverStagesAndUnavailableValues) {
    C::Config c;c.optimizer_iterations=1;
    auto g=grid();auto r=failure(g,c);
    EXPECT_EQ(r.failure_reason,C::FailureReason::NUMERICAL_FAILURE);
    EXPECT_EQ(r.diagnostics.stage,C::FailureStage::MVIE_SOLVE);
    EXPECT_EQ(r.diagnostics.outer_iteration,0);
    EXPECT_TRUE(r.diagnostics.solver_status_valid);EXPECT_EQ(r.diagnostics.solver_status,nlopt::MAXEVAL_REACHED);
    EXPECT_TRUE(r.diagnostics.min_slack_valid);EXPECT_GT(r.diagnostics.min_slack,0.);
    EXPECT_TRUE(r.diagnostics.constraint_violation_valid);
    g.blocked[12*48+14]=255;r=failure(g,c);
    EXPECT_EQ(r.failure_reason,C::FailureReason::NUMERICAL_FAILURE);
    // Separation no longer consumes NLopt evaluations; the limited budget
    // now reaches MVIE even when a blocked cell is present.
    EXPECT_EQ(r.diagnostics.stage,C::FailureStage::MVIE_SOLVE);
    EXPECT_EQ(r.diagnostics.solver_status,nlopt::MAXEVAL_REACHED);
    EXPECT_TRUE(r.diagnostics.min_slack_valid);EXPECT_GT(r.diagnostics.min_slack,0.);
    EXPECT_TRUE(r.diagnostics.constraint_violation_valid);
    C builder;r=builder.buildStatic({},g);
    EXPECT_EQ(r.diagnostics.stage,C::FailureStage::INPUT);
    EXPECT_FALSE(r.diagnostics.solver_status_valid);EXPECT_EQ(r.diagnostics.solver_status,0);
    EXPECT_FALSE(r.diagnostics.constraint_violation_valid);
    EXPECT_NE(C::diagnosticsText(r).find("max_constraint_violation=NA"),std::string::npos);
    r=failure(grid(),C::Config());ASSERT_TRUE(r.feasible);
    EXPECT_EQ(r.diagnostics.stage,C::FailureStage::NONE);EXPECT_FALSE(r.diagnostics.solver_status_valid);
}
TEST_F(Snapshot, ExactRoundTripAndSameBuilder) {
    auto g=grid();g.blocked[12*48+14]=255;C::Config c;
    c.local_radius=2.987654321012345;c.clearance=.003;c.max_segment_length=.6;
    c.max_seconds=.75;c.iterations=3;c.optimizer_iterations=1;c.max_regions=99;
    auto r=failure(g,c);ASSERT_FALSE(r.feasible);
    // Exercise quoted exception/optional metadata independently of NLopt's platform-specific exceptions.
    r.diagnostics.solver_exception="roundoff \"detail\"\\test";
    Capture capture;std::string file,error;
    ASSERT_EQ(capture.saveOnce(directory,g,path,c,r,file,error),Capture::Status::SAVED)<<error;files.push_back(file);
    cane_planner::CorridorFailureSnapshot s;
    ASSERT_TRUE(cane_planner::readCorridorFailureSnapshot(file,s,error))<<error;
    EXPECT_EQ(s.grid.blocked,g.blocked);EXPECT_EQ(s.grid.width,g.width);EXPECT_EQ(s.grid.height,g.height);
    EXPECT_EQ(s.grid.origin,g.origin);EXPECT_EQ(s.grid.resolution,g.resolution);ASSERT_EQ(s.path.size(),path.size());
    for(size_t i=0;i<path.size();++i)EXPECT_EQ(s.path[i],path[i]);
#define SAME(field) EXPECT_EQ(s.config.field,c.field)
    SAME(local_radius);SAME(clearance);SAME(max_segment_length);
    SAME(max_seconds);SAME(iterations);SAME(optimizer_iterations);SAME(max_regions);
#undef SAME
    EXPECT_EQ(s.failure.failure_reason,r.failure_reason);EXPECT_EQ(s.failure.failure_segment_index,r.failure_segment_index);
    EXPECT_EQ(s.failure.failure_s,r.failure_s);EXPECT_EQ(s.failure.failure_position,r.failure_position);
    EXPECT_EQ(s.failure.elapsed_seconds,r.elapsed_seconds);
    EXPECT_EQ(C::diagnosticsText(s.failure),C::diagnosticsText(r));
    const auto replay=failure(s.grid,s.config);EXPECT_EQ(replay.failure_reason,r.failure_reason);
    EXPECT_EQ(replay.diagnostics.stage,r.diagnostics.stage);EXPECT_EQ(replay.diagnostics.solver_status,r.diagnostics.solver_status);
    // A damaged/version-mismatched file must not be accepted as a replay input.
    const auto bad=directory+"/bad";files.push_back(bad);{std::ofstream out(bad);out<<"CANE_CORRIDOR_FAILURE 2\n";}
    EXPECT_FALSE(cane_planner::readCorridorFailureSnapshot(bad,s,error));EXPECT_FALSE(error.empty());
}
TEST_F(Snapshot, FirstEligibleOnlyAndReset) {
    C::Config c;c.optimizer_iterations=1;auto g=grid();const auto r=failure(g,c);
    Capture capture;std::string file,error;
    C::Result unavailable;unavailable.failure_reason=C::FailureReason::UNSUPPORTED_BACKEND;
    EXPECT_EQ(capture.saveOnce(directory,g,path,c,unavailable,file,error),Capture::Status::SKIPPED);
    EXPECT_EQ(capture.saveOnce(directory,C::Grid(),path,c,r,file,error),Capture::Status::SKIPPED);
    EXPECT_EQ(capture.saveOnce(directory,g,{},c,r,file,error),Capture::Status::SKIPPED);
    ASSERT_EQ(capture.saveOnce(directory,g,path,c,r,file,error),Capture::Status::SAVED)<<error;files.push_back(file);
    EXPECT_EQ(capture.saveOnce(directory,g,path,c,r,file,error),Capture::Status::SKIPPED);
    capture.resetForGoal();
    ASSERT_EQ(capture.saveOnce(directory,g,path,c,r,file,error),Capture::Status::SAVED)<<error;files.push_back(file);
    EXPECT_NE(files[0],files[1]);
    cane_planner::CorridorFailureSnapshot s;
    EXPECT_TRUE(cane_planner::readCorridorFailureSnapshot(files[0],s,error));
}
TEST_F(Snapshot, IoFailureConsumesLatchUntilNewGoal) {
    C::Config c;c.optimizer_iterations=1;const auto g=grid();const auto r=failure(g,c);
    Capture capture;std::string file,error;
    EXPECT_EQ(capture.saveOnce("relative/not/allowed",g,path,c,r,file,error),Capture::Status::IO_ERROR);
    EXPECT_FALSE(error.empty());EXPECT_TRUE(file.empty());
    EXPECT_EQ(capture.saveOnce(directory,g,path,c,r,file,error),Capture::Status::SKIPPED);
    capture.resetForGoal();
    ASSERT_EQ(capture.saveOnce(directory,g,path,c,r,file,error),Capture::Status::SAVED)<<error;files.push_back(file);
}
int main(int argc,char** argv){testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}

TEST(HistoricalSnapshot, M6wfx3PredeclaredSameMapPerturbations) {
    cane_planner::CorridorFailureSnapshot snapshot;std::string error;
    const std::string source=__FILE__;
    ASSERT_TRUE(cane_planner::readCorridorFailureSnapshot(
        source.substr(0,source.find_last_of('/'))+"/fixtures/corridor_failures/failure-M6wfx3",snapshot,error))<<error;
    std::vector<V> shifts{V::Zero()};
    for(double d:{-.05,-.02,-.01,-.005,.005,.01,.02,.05})shifts.push_back(V(d,0));
    for(double d:{-.05,-.02,-.01,-.005,.005,.01,.02,.05})shifts.push_back(V(0,d));
    shifts.push_back(V::Zero()); // fresh original after the complete sweep
    for(size_t i=0;i<shifts.size();++i) {
        SCOPED_TRACE(i);auto path=snapshot.path;for(auto& p:path)p+=shifts[i];
        C builder;builder.setConfig(snapshot.config);const auto result=builder.buildStatic(path,snapshot.grid);
        // Frozen independent long-double segment/rectangle distance analysis:
        // cases 9..11 cross a full cell; all others have >=4.010696 mm gap.
        const bool valid=!(i>=9 && i<=11);
        ASSERT_EQ(result.feasible,valid)<<C::diagnosticsText(result);
        if(valid) {
            EXPECT_GE(result.min_overlap_depth,1e-6);
            EXPECT_TRUE(C::contains(result.segments.front(),path.front()));
            EXPECT_TRUE(C::contains(result.segments.back(),path.back()));
        } else {EXPECT_EQ(result.failure_reason,C::FailureReason::REFERENCE_OCCUPIED);EXPECT_TRUE(result.segments.empty());}
    }
}
