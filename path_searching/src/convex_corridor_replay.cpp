#include <path_searching/corridor_failure_snapshot.h>
#include <iomanip>
#include <iostream>
int main(int argc,char** argv) {
    if(argc!=2){std::cerr << "Usage: convex_corridor_replay SNAPSHOT\n";return 2;}
    cane_planner::CorridorFailureSnapshot s;std::string error;
    if(!cane_planner::readCorridorFailureSnapshot(argv[1],s,error)){std::cerr << error << '\n';return 2;}
    using C=cane_planner::ConvexCorridor;C corridor;corridor.setConfig(s.config);
    const auto replay=corridor.buildStatic(s.path,s.grid);
    std::cout << std::setprecision(17);
    const std::pair<const char*,const C::Result*> entries[]={{"recorded",&s.failure},{"replayed",&replay}};
    for(const auto& entry: entries) {
        const auto& r=*entry.second;
        std::cout << entry.first << " feasible=" << r.feasible << " reason=" << C::failureReasonName(r.failure_reason)
                  << " segment=" << r.failure_segment_index << " s=" << r.failure_s
                  << " position_valid=" << r.failure_position_valid << " x=" << r.failure_position.x()
                  << " y=" << r.failure_position.y() << C::diagnosticsText(r) << " seconds=" << r.elapsed_seconds << '\n';
    }
    std::cout << "Same buildStatic and recorded config; wall-clock budgets and solver/platform differences may change the outcome.\n";
    return 0; // Successful replay, not a claim that geometry was feasible or reproduced identically.
}
