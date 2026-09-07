#include <path_searching/corridor_failure_snapshot.h>
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <sys/stat.h>
#include <unistd.h>

namespace cane_planner {
namespace {
using C = ConvexCorridor;
// Adapter's existing 1M-cell cap. Path cap is diagnostic-only; never truncates planner input.
constexpr size_t kMaxEntries = 1000000;
bool validInput(const C::Grid& g, const std::vector<Eigen::Vector2d>& p) {
    if(g.width<=0||g.height<=0||g.width>10000||g.height>10000||
       size_t(g.width)*size_t(g.height)>kMaxEntries||
       g.blocked.size()!=size_t(g.width)*size_t(g.height)||!g.origin.allFinite()||
       !std::isfinite(g.resolution)||g.resolution<.01||
       g.resolution*std::max(g.width,g.height)>100.||p.size()<2||p.size()>kMaxEntries) return false;
    if(g.origin.cwiseAbs().maxCoeff()>1e4||
       (g.origin+g.resolution*Eigen::Vector2d(g.width,g.height)).cwiseAbs().maxCoeff()>1e4)return false;
    for(size_t i=0;i<p.size();++i)
        if(!p[i].allFinite()||p[i].cwiseAbs().maxCoeff()>1e4||(i&&(p[i]-p[i-1]).norm()<1e-9))return false;
    return true;
}
bool eligible(const C::Result& r) {
    if(r.feasible||!r.failure_position_valid)return false;
    switch(r.failure_reason) {
        case C::FailureReason::NUMERICAL_FAILURE: case C::FailureReason::OUTPUT_CERTIFICATE:
        case C::FailureReason::REFERENCE_OCCUPIED: case C::FailureReason::NO_OVERLAP:
        case C::FailureReason::NO_PROGRESS: return true;
        default: return false; // Pre-map/config errors and time/resource budgets do not consume the latch.
    }
}
void optionalNumber(std::ostream& out, bool valid, double value) {
    out << ' ' << valid << ' '; if(valid)out << value; else out << "NA";
}
void writeSnapshot(std::ostream& o,const C::Grid& g,const std::vector<Eigen::Vector2d>& p,
                   const C::Config& c,const C::Result& r) {
    o.imbue(std::locale::classic()); o << std::setprecision(17);
    o << "CANE_CORRIDOR_FAILURE 2\nconfig " << c.local_radius << ' ' << c.clearance << ' '
      << c.max_segment_length << ' ' << c.max_seconds
      << ' ' << c.iterations << ' ' << c.optimizer_iterations << ' ' << c.max_regions;
    o << "\ngrid " << g.origin.x() << ' ' << g.origin.y() << ' ' << g.resolution << ' ' << g.width << ' ' << g.height;
    o << "\nblocked " << g.blocked.size() << '\n';
    // Full row-major byte mask, not obstacle centers or a resampled map.
    for(size_t i=0;i<g.blocked.size();++i)o << unsigned(g.blocked[i]) << ((i+1)%g.width?' ':'\n');
    o << "path " << p.size() << '\n';for(const auto& v:p)o << v.x() << ' ' << v.y() << '\n';
    o << "failure " << C::failureReasonName(r.failure_reason) << ' ' << r.feasible << ' '
      << r.failure_segment_index << ' ' << r.failure_s << ' ' << r.failure_position_valid << ' '
      << r.failure_position.x() << ' ' << r.failure_position.y() << ' '
      << r.min_overlap_area << ' ' << r.min_overlap_depth << ' ' << r.elapsed_seconds;
    const auto& d=r.diagnostics;
    o << "\ndiagnostics " << C::failureStageName(d.stage) << ' ' << d.outer_iteration << ' '
      << d.solver_status_valid << ' ' << d.solver_status << ' ' << std::quoted(d.solver_exception);
    optionalNumber(o,d.min_slack_valid,d.min_slack);
    optionalNumber(o,d.constraint_violation_valid,d.max_constraint_violation);
    o << "\nEND\n";
}
void check(bool ok) { if(!ok)throw std::runtime_error("invalid/truncated corridor snapshot"); }
void token(std::istream& in,const char* expected) { std::string s; in>>s;check(in&&s==expected); }
void readOptional(std::istream& in,bool& valid,double& value) {
    in>>valid;check(bool(in));
    if(valid){in>>value;check(in&&std::isfinite(value));}
    else {token(in,"NA");value=std::numeric_limits<double>::quiet_NaN();}
}
bool makeDirectory(const std::string& directory,std::string& error) {
    if(directory.empty()||directory[0]!='/'){error="snapshot directory must be an absolute path";return false;}
    for(size_t i=1;i<=directory.size();++i) {
        if(i!=directory.size()&&directory[i]!='/')continue;
        const auto part=directory.substr(0,i);
        if(::mkdir(part.c_str(),0700)!=0&&errno!=EEXIST){error=std::strerror(errno);return false;}
        struct stat st;
        if(::stat(part.c_str(),&st)!=0||!S_ISDIR(st.st_mode)){error="snapshot path is not a directory: "+part;return false;}
    }
    return true;
}
}
std::string defaultCorridorFailureDirectory() {
    const char* home=std::getenv("ROS_HOME");
    std::string root;
    if(home&&*home)root=home;
    else {home=std::getenv("HOME");if(home&&*home)root=std::string(home)+"/.ros";}
    if(root.empty())return {}; // Caller logs unavailable instead of silently writing elsewhere.
    if(root.front()!='/') {
        char* cwd=::getcwd(nullptr,0);if(!cwd)return {};
        root=std::string(cwd)+"/"+root;std::free(cwd);
    }
    return root+"/cane_planner/corridor_failures";
}
CorridorFailureCapture::Status CorridorFailureCapture::saveOnce(
    const std::string& directory,const C::Grid& grid,const std::vector<Eigen::Vector2d>& path,
    const C::Config& config,const C::Result& failure,std::string& file,std::string& error) {
    file.clear();error.clear();
    if(attempted_||!eligible(failure)||!validInput(grid,path))return Status::SKIPPED;
    attempted_=true;
    if(!makeDirectory(directory,error))return Status::IO_ERROR;
    std::ostringstream text;writeSnapshot(text,grid,path,config,failure);
    if(!text){error="snapshot serialization failed";return Status::IO_ERROR;}
    std::string name=directory+"/failure-XXXXXX";
    std::vector<char> pattern(name.begin(),name.end());pattern.push_back('\0');
    const int fd=::mkstemp(pattern.data()); // O_EXCL and mode 0600; never overwrites a previous goal/run.
    if(fd<0){error=std::strerror(errno);return Status::IO_ERROR;}
    name=pattern.data();const std::string data=text.str();size_t done=0;bool ok=true;
    while(done<data.size()) {
        const ssize_t n=::write(fd,data.data()+done,data.size()-done);
        if(n<0&&errno==EINTR)continue;
        if(n<=0){error=n<0?std::strerror(errno):"zero-byte snapshot write";ok=false;break;}
        done+=size_t(n);
    }
    if(ok&&::fsync(fd)!=0){error=std::strerror(errno);ok=false;}
    if(::close(fd)!=0){error=std::strerror(errno);ok=false;}
    if(!ok){if(::unlink(name.c_str())!=0)error+="; could not remove partial file "+name;return Status::IO_ERROR;}
    file=name;return Status::SAVED;
}
bool readCorridorFailureSnapshot(const std::string& file,CorridorFailureSnapshot& snapshot,std::string& error) {
    error.clear();
    try {
        struct stat st;check(::stat(file.c_str(),&st)==0&&S_ISREG(st.st_mode)&&st.st_size<=64*1024*1024);
        std::ifstream in(file);in.imbue(std::locale::classic());check(bool(in));
        CorridorFailureSnapshot s;auto& c=s.config;auto& g=s.grid;auto& r=s.failure;auto& d=r.diagnostics;
        token(in,"CANE_CORRIDOR_FAILURE");int version;in>>version;check(in&&(version==1||version==2));
        token(in,"config");in>>c.local_radius>>c.clearance;
        if(version==1) {
            // Historical geometric-only progress/depth/area fields are consumed,
            // not reinterpreted as segment length. Replay uses the new 1 m default.
            double progress,depth,area;in>>progress>>depth>>area;
            check(in&&std::isfinite(progress)&&std::isfinite(depth)&&std::isfinite(area));
        } else in>>c.max_segment_length;
        in>>c.max_seconds>>c.iterations>>c.optimizer_iterations>>c.max_regions;check(bool(in));
        token(in,"grid");in>>g.origin.x()>>g.origin.y()>>g.resolution>>g.width>>g.height;
        check(in&&g.width>0&&g.height>0&&g.width<=10000&&g.height<=10000&&size_t(g.width)*size_t(g.height)<=kMaxEntries);
        token(in,"blocked");size_t n;in>>n;check(in&&n==size_t(g.width)*size_t(g.height));g.blocked.resize(n);
        for(auto& v:g.blocked){unsigned b;in>>b;check(in&&b<=255);v=static_cast<uint8_t>(b);}
        token(in,"path");in>>n;check(in&&n>=2&&n<=kMaxEntries);s.path.resize(n);
        for(auto& v:s.path){in>>v.x()>>v.y();check(bool(in));}
        check(validInput(g,s.path));
        token(in,"failure");std::string reason;in>>reason;bool found=false;
        for(int i=0;i<=static_cast<int>(C::FailureReason::UNSUPPORTED_BACKEND);++i)
            if(reason==C::failureReasonName(static_cast<C::FailureReason>(i))){r.failure_reason=static_cast<C::FailureReason>(i);found=true;}
        check(found);in>>r.feasible>>r.failure_segment_index>>r.failure_s>>r.failure_position_valid
          >>r.failure_position.x()>>r.failure_position.y()>>r.min_overlap_area>>r.min_overlap_depth>>r.elapsed_seconds;check(bool(in));
        token(in,"diagnostics");std::string stage;in>>stage;found=false;
        for(int i=0;i<=static_cast<int>(C::FailureStage::OVERLAP);++i)
            if(stage==C::failureStageName(static_cast<C::FailureStage>(i))){d.stage=static_cast<C::FailureStage>(i);found=true;}
        check(found);in>>d.outer_iteration>>d.solver_status_valid>>d.solver_status>>std::quoted(d.solver_exception);
        check(in&&d.solver_exception.size()<=256);
        readOptional(in,d.min_slack_valid,d.min_slack);readOptional(in,d.constraint_violation_valid,d.max_constraint_violation);
        token(in,"END");in>>std::ws;check(in.eof());snapshot=std::move(s);return true;
    } catch(const std::exception& e) {error=e.what();return false;}
}
}
