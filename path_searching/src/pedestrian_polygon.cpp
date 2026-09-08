#include <path_searching/pedestrian_polygon.h>
#include <algorithm>
#include <chrono>
#include <cmath>
namespace cane_planner {
namespace {
using V = Eigen::Vector2d;
double cross(const V& a,const V& b) { return a.x()*b.y()-a.y()*b.x(); }
constexpr double pi = 3.14159265358979323846;
}
bool PedestrianPolygon::validConfig(const Config& c) {
    for(double x:{c.safety_radius,c.wall_gain,c.system_gain,c.decay_length,c.fov_amplification,
                  c.alpha,c.max_force,c.max_extension,c.influence_distance,c.max_seconds})
        if(!std::isfinite(x))return false;
    return c.safety_radius>=.01 && c.safety_radius<=2. && c.wall_gain>=0 && c.wall_gain<=20 &&
        c.system_gain>=0 && c.system_gain<=20 && c.decay_length>=.01 && c.decay_length<=5 &&
        c.fov_amplification>=1 && c.fov_amplification<=10 && c.alpha>=0 && c.alpha<=5 &&
        c.max_force>0 && c.max_force<=20 && c.max_extension>=0 && c.max_extension<=5 &&
        c.influence_distance>=.01 && c.influence_distance<=10 && c.max_seconds>0 && c.max_seconds<=.1;
}
bool PedestrianPolygon::validObservation(const Observation& o) {
    return o.position.allFinite() && o.position.cwiseAbs().maxCoeff()<=1e4 &&
        o.velocity.allFinite() && o.velocity.norm()<=10 && o.full_size.allFinite() &&
        o.full_size.minCoeff()>=.01 && o.full_size.maxCoeff()<=4;
}
PedestrianPolygon::Polygon PedestrianPolygon::convexHull(Polygon p) {
    for(const auto& v:p)if(!v.allFinite())return {};
    std::sort(p.begin(),p.end(),[](const V&a,const V&b){return a.x()<b.x() || (a.x()==b.x() && a.y()<b.y());});
    p.erase(std::unique(p.begin(),p.end(),[](const V&a,const V&b){return a==b;}),p.end());
    if(p.size()<3)return {};
    Polygon h(2*p.size());size_t k=0;
    for(const auto& v:p) {while(k>=2 && cross(h[k-1]-h[k-2],v-h[k-1])<=0)--k;h[k++]=v;}
    const size_t lower=k+1;
    for(size_t i=p.size()-1;i>0;--i) {const auto&v=p[i-1];while(k>=lower && cross(h[k-1]-h[k-2],v-h[k-1])<=0)--k;h[k++]=v;}
    h.resize(k-1);return h;
}
PedestrianPolygon::Result PedestrianPolygon::fromForce(const Observation&o,const V& force,const Config&c) {
    Result r;
    if(!validConfig(c)||!validObservation(o)||!force.allFinite()||!std::isfinite(force.norm())) {r.reason="INVALID_POLYGON_INPUT";return r;}
    r.force=force;
    if(r.force.norm()>c.max_force)r.force*=c.max_force/r.force.norm();
    const V half=o.full_size*.5;
    for(const V& p:{V(-half.x(),-half.y()),V(half.x(),-half.y()),V(half.x(),half.y()),V(-half.x(),half.y())})r.body.push_back(o.position+p);
    Polygon points=r.body;
    const double ds=std::max(c.safety_radius,half.norm());
    // Circumscribed regular octagon contains the ENTIRE safety disk, including
    // the physical rectangle. No inscribed polygon masquerading as disk coverage.
    const double radius=ds/std::cos(pi/8.);
    for(int i=0;i<8;++i)points.push_back(o.position+radius*V(std::cos(i*pi/4.),std::sin(i*pi/4.)));
    if(o.velocity.norm()>1e-6) {
        const double heading=std::atan2(o.velocity.y(),o.velocity.x());
        for(double angle:{heading-110*pi/180.,heading+110*pi/180.})points.push_back(o.position+ds*V(std::cos(angle),std::sin(angle)));
        if(r.force.norm()>1e-9)points.push_back(o.position+(ds+std::min(c.max_extension,c.alpha*r.force.norm()))*r.force.normalized());
    }
    r.hull=convexHull(std::move(points));
    r.valid=r.hull.size()>=3 && ConvexCorridor::polygonArea(r.hull)>0;
    r.reason=r.valid?"OK":"INVALID_HULL";return r;
}
PedestrianPolygon::Result PedestrianPolygon::build(const Observation&o,const Eigen::Vector3d& system,
                                                   const ConvexCorridor::Grid&g,const Config&c) {
    Result invalid;invalid.reason="INVALID_FORCE_INPUT";
    if(!validConfig(c)||!validObservation(o)||!system.allFinite() ||
       !g.origin.allFinite()||!std::isfinite(g.resolution)||g.resolution<.01 ||
       g.width<=0||g.height<=0||g.width>10000||g.height>10000 ||
       g.resolution*std::max(g.width,g.height)>100 ||
       g.blocked.size()!=size_t(g.width)*size_t(g.height))return invalid;
    const auto began=std::chrono::steady_clock::now();
    V away=V::Zero();double nearest=c.influence_distance;
    auto consider=[&](const V& q,const V& inside_direction) {
        const V d=o.position-q;const double length=d.norm();
        if(length<nearest) {nearest=length;away=length>1e-9?V(d/length):inside_direction;}
    };
    const V upper=g.origin+g.resolution*V(g.width,g.height);
    // Finite cropped-domain exterior is blocked by the same model as FIRI.
    const V clamped=o.position.cwiseMax(g.origin).cwiseMin(upper);
    if(clamped!=o.position) {
        const double distance=(clamped-o.position).norm();
        if(distance<nearest){nearest=distance;away=(clamped-o.position)/distance;}
    }
    else {
        consider(V(g.origin.x(),o.position.y()),V(1,0));consider(V(upper.x(),o.position.y()),V(-1,0));
        consider(V(o.position.x(),g.origin.y()),V(0,1));consider(V(o.position.x(),upper.y()),V(0,-1));
    }
    for(int y=0;y<g.height;++y) {
        if(std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()>=c.max_seconds) {invalid.reason="FORCE_BUDGET";return invalid;}
        for(int x=0;x<g.width;++x) {
            if(!g.blocked[y*g.width+x])continue;
            const V lo=g.origin+g.resolution*V(x,y), hi=lo+V::Constant(g.resolution);
            V q=o.position.cwiseMax(lo).cwiseMin(hi);
            if(q==o.position) {
                // Inside a blocked bin: nearest face gives an outward direction.
                double best=std::numeric_limits<double>::infinity();V direction=V::Zero();
                for(const V& n:{V(-1,0),V(1,0),V(0,-1),V(0,1)}) {
                    const double d=n.x()<0?o.position.x()-lo.x():n.x()>0?hi.x()-o.position.x():n.y()<0?o.position.y()-lo.y():hi.y()-o.position.y();
                    if(d<best){best=d;direction=n;}
                }
                if(best<nearest){nearest=best;away=direction;}
            } else consider(q,V::Zero());
        }
    }
    const double ds=std::max(c.safety_radius,.5*o.full_size.norm());
    auto magnitude=[&](double gain,double distance) {return gain*std::exp(std::min(0.,(ds-distance)/c.decay_length));};
    V force=magnitude(c.wall_gain,nearest)*away;
    const V delta=o.position-system.head<2>();const double distance=delta.norm();
    if(distance<c.influence_distance && distance>1e-9) {
        double gain=c.system_gain;
        if(o.velocity.norm()>1e-6 && o.velocity.normalized().dot(-delta/distance)>=std::cos(110*pi/180.))gain*=c.fov_amplification;
        force+=magnitude(gain,distance)*delta/distance;
    }
    return fromForce(o,force,c);
}
}
