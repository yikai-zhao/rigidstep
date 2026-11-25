#include "rigidstep/physics.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>
#include <stdexcept>

namespace rigidstep {
namespace {
constexpr double epsilon = 1e-9;
constexpr Vec3 axes[] = {{1,0,0}, {0,1,0}, {0,0,1}};

double component(Vec3 v, int i) {
    return i == 0 ? v.x : (i == 1 ? v.y : v.z);
}
Vec3 clamp_vec(Vec3 v, Vec3 lo, Vec3 hi) {
    return {std::clamp(v.x,lo.x,hi.x),std::clamp(v.y,lo.y,hi.y),std::clamp(v.z,lo.z,hi.z)};
}
Vec3 abs_vec(Vec3 v) { return {std::abs(v.x),std::abs(v.y),std::abs(v.z)}; }
Vec3 world_inv_inertia(const Body& b, Vec3 v) {
    if (b.inv_mass == 0) return {};
    Vec3 local = b.rotation.inverse_rotate(v);
    local = {local.x*b.inv_inertia_local.x,local.y*b.inv_inertia_local.y,
             local.z*b.inv_inertia_local.z};
    return b.rotation.rotate(local);
}
Vec3 support(const Body& b, Vec3 direction) {
    if (b.shape == Shape::Sphere) return b.position + normalized(direction)*b.radius;
    const Vec3 d = b.rotation.inverse_rotate(direction);
    const Vec3 h = b.half_extents;
    return b.position + b.rotation.rotate({d.x>=0?h.x:-h.x,d.y>=0?h.y:-h.y,d.z>=0?h.z:-h.z});
}
struct Contact {
    std::size_t a{}, b{};
    Vec3 n{}, point{}, tangent_impulse{};
    double penetration{}, normal_impulse{}, bounce_target{};
};

bool overlap(const Aabb& a, const Aabb& b) {
    return a.min.x<=b.max.x && b.min.x<=a.max.x &&
           a.min.y<=b.max.y && b.min.y<=a.max.y &&
           a.min.z<=b.max.z && b.min.z<=a.max.z;
}

bool collide_spheres(const Body& a, const Body& b, Contact& c) {
    const Vec3 delta = b.position-a.position;
    const double r = a.radius+b.radius;
    const double d2 = dot(delta,delta);
    if (d2 >= r*r) return false;
    const double d = std::sqrt(d2);
    c.n = d > epsilon ? delta/d : Vec3{1,0,0};
    c.penetration = r-d;
    c.point = a.position + c.n*(a.radius-c.penetration*0.5);
    return true;
}

// c.n always points from sphere to box; a sphere fully inside a box is pushed out.
bool collide_sphere_box(const Body& sphere, const Body& box, Contact& c) {
    const Vec3 local = box.rotation.inverse_rotate(sphere.position-box.position);
    const Vec3 closest = clamp_vec(local,-box.half_extents,box.half_extents);
    Vec3 outward = local-closest;
    const double dist2 = dot(outward,outward);
    if (dist2 > sphere.radius*sphere.radius) return false;
    if (dist2 > epsilon*epsilon) {
        const double d = std::sqrt(dist2);
        const Vec3 from_box_to_sphere = box.rotation.rotate(outward/d);
        c.n = -from_box_to_sphere;
        c.penetration = sphere.radius-d;
        c.point = box.position+box.rotation.rotate(closest);
    } else {
        Vec3 clearances = box.half_extents-abs_vec(local);
        int axis = 0;
        if (clearances.y < clearances.x) axis = 1;
        if (component(clearances,2) < component(clearances,axis)) axis = 2;
        const double sign = component(local,axis)>=0?1.0:-1.0;
        const Vec3 out = box.rotation.rotate(axes[axis]*sign);
        c.n = -out;
        c.penetration = sphere.radius + component(clearances,axis);
        c.point = sphere.position+out*component(clearances,axis);
    }
    return true;
}

bool collide_boxes(const Body& a, const Body& b, Contact& c) {
    const Vec3 delta=b.position-a.position;
    const Vec3 pa[]={a.rotation.rotate(axes[0]),a.rotation.rotate(axes[1]),a.rotation.rotate(axes[2])};
    const Vec3 pb[]={b.rotation.rotate(axes[0]),b.rotation.rotate(axes[1]),b.rotation.rotate(axes[2])};
    double min_depth=std::numeric_limits<double>::infinity();
    Vec3 min_axis{};
    auto test_axis=[&](Vec3 axis) {
        const double l=norm(axis);
        if (l<1e-7) return true; // Degenerate cross-product; not a separating direction.
        axis=axis/l;
        double ra=0,rb=0;
        for(int i=0;i<3;++i) {
            ra+=component(a.half_extents,i)*std::abs(dot(pa[i],axis));
            rb+=component(b.half_extents,i)*std::abs(dot(pb[i],axis));
        }
        const double overlap_depth=ra+rb-std::abs(dot(delta,axis));
        if(overlap_depth<=0) return false;
        if(overlap_depth<min_depth) {
            min_depth=overlap_depth;
            min_axis=dot(delta,axis)>=0?axis:-axis;
        }
        return true;
    };
    for(const auto& axis:pa) if(!test_axis(axis)) return false;
    for(const auto& axis:pb) if(!test_axis(axis)) return false;
    for(const auto& x:pa) for(const auto& y:pb) if(!test_axis(cross(x,y))) return false;
    c.n=min_axis;
    c.penetration=min_depth;
    // Single support-point approximation: not a production contact manifold.
    c.point=(support(a,c.n)+support(b,-c.n))*0.5;
    return true;
}

void ground_contacts(const Body& plane, const Body& b, std::size_t pi,
                     std::size_t bi, std::vector<Contact>& out) {
    const double height=plane.position.y;
    if (b.shape==Shape::Sphere) {
        const double depth=height-(b.position.y-b.radius);
        if (depth>0) out.push_back({pi,bi,{0,1,0},{b.position.x,height,b.position.z},{},depth});
    } else if (b.shape==Shape::Box) {
        // Up to four bottom-support candidates. Duplicate projected vertices are
        // intentionally suppressed for axis-aligned boxes in a stable order.
        for (int x : {-1,1}) for (int y : {-1,1}) for (int z : {-1,1}) {
            const Vec3 p=b.position+b.rotation.rotate({x*b.half_extents.x,
                                                       y*b.half_extents.y,z*b.half_extents.z});
            if (p.y>=height-1e-8) continue;
            const Vec3 on_plane{p.x,height,p.z};
            bool duplicate=false;
            for(const auto& existing:out) {
                if(existing.a!=pi || existing.b!=bi) continue;
                const Vec3 diff=existing.point-on_plane;
                if(dot(diff,diff)<1e-12) {duplicate=true;break;}
            }
            if(!duplicate) out.push_back({pi,bi,{0,1,0},on_plane,{},height-p.y});
        }
    }
}

std::vector<Contact> contacts_for(std::vector<Body>& bodies, Broadphase mode,
                                  std::size_t& candidate_count) {
    std::vector<Contact> out;
    const auto candidates=broadphase_pairs(bodies,mode);
    candidate_count=candidates.size();
    for (const auto& pair:candidates) {
        Body& a=bodies[pair.a]; Body& b=bodies[pair.b];
        if (a.shape==Shape::Plane || b.shape==Shape::Plane) continue;
        Contact c{}; c.a=pair.a; c.b=pair.b;
        bool hit=false;
        if(a.shape==Shape::Sphere && b.shape==Shape::Sphere) hit=collide_spheres(a,b,c);
        if(a.shape==Shape::Sphere && b.shape==Shape::Box) hit=collide_sphere_box(a,b,c);
        if(a.shape==Shape::Box && b.shape==Shape::Sphere) {
            hit=collide_sphere_box(b,a,c); c.n=-c.n;
        }
        if(a.shape==Shape::Box && b.shape==Shape::Box) hit=collide_boxes(a,b,c);
        if(hit) out.push_back(c);
    }
    for(std::size_t i=0;i<bodies.size();++i) {
        if(bodies[i].shape!=Shape::Plane) continue;
        for(std::size_t j=0;j<bodies.size();++j) {
            if(i==j || bodies[j].inv_mass==0) continue;
            ground_contacts(bodies[i],bodies[j],i,j,out);
        }
    }
    return out;
}

Vec3 velocity_at(const Body& b, Vec3 r) {
    return b.velocity+cross(b.angular_velocity,r);
}
void impulse(Body& b, Vec3 j, Vec3 r) {
    b.velocity+=j*b.inv_mass;
    b.angular_velocity+=world_inv_inertia(b,cross(r,j));
}
double effective_mass(const Body& a, const Body& b, Vec3 ra, Vec3 rb, Vec3 dir) {
    return a.inv_mass+b.inv_mass+
        dot(dir,cross(world_inv_inertia(a,cross(ra,dir)),ra)+
                cross(world_inv_inertia(b,cross(rb,dir)),rb));
}
void solve_contact(Contact& c, std::vector<Body>& bodies, double dt) {
    Body& a=bodies[c.a]; Body& b=bodies[c.b];
    const Vec3 ra=c.point-a.position, rb=c.point-b.position;
    const double k=effective_mass(a,b,ra,rb,c.n);
    if(k<epsilon) return;
    const Vec3 dv=velocity_at(b,rb)-velocity_at(a,ra);
    const double vn=dot(dv,c.n);
    const double bias=std::clamp(0.14*std::max(0.0,c.penetration-0.002)/dt,0.0,2.0);
    const double target=std::max(bias,c.bounce_target);
    const double next=std::max(0.0,c.normal_impulse+(target-vn)/k);
    const double dl=next-c.normal_impulse;
    c.normal_impulse=next;
    const Vec3 j=c.n*dl;
    impulse(a,-j,ra); impulse(b,j,rb);

    const Vec3 relative=velocity_at(b,rb)-velocity_at(a,ra);
    const Vec3 tangent=relative-c.n*dot(relative,c.n);
    const double speed=norm(tangent);
    if(speed<epsilon) return;
    const Vec3 t=tangent/speed;
    const double kt=effective_mass(a,b,ra,rb,t);
    if(kt<epsilon) return;
    Vec3 proposed=c.tangent_impulse-t*(speed/kt);
    const double friction=std::sqrt(a.friction*b.friction)*c.normal_impulse;
    const double len=norm(proposed);
    if(len>friction && len>epsilon) proposed*=friction/len;
    const Vec3 dj=proposed-c.tangent_impulse;
    c.tangent_impulse=proposed;
    impulse(a,-dj,ra); impulse(b,dj,rb);
}

void hash_u64(std::uint64_t& h, std::uint64_t v) {
    for(int n=0;n<8;++n) {h^=(v>>(8*n))&0xffU;h*=1099511628211ULL;}
}
void hash_double(std::uint64_t& h, double v) {hash_u64(h,std::bit_cast<std::uint64_t>(v));}
} // namespace

double norm(Vec3 v) {return std::sqrt(dot(v,v));}
Vec3 normalized(Vec3 v, Vec3 fallback) {const double l=norm(v);return l>epsilon?v/l:fallback;}
Quat Quat::normalized() const {
    const double l=std::sqrt(w*w+x*x+y*y+z*z);
    return l>epsilon?Quat{w/l,x/l,y/l,z/l}:Quat{};
}
Quat Quat::operator*(Quat q) const {
    return {w*q.w-x*q.x-y*q.y-z*q.z,
            w*q.x+x*q.w+y*q.z-z*q.y,
            w*q.y-x*q.z+y*q.w+z*q.x,
            w*q.z+x*q.y-y*q.x+z*q.w};
}
Vec3 Quat::rotate(Vec3 v) const {
    const Vec3 q{x,y,z};const Vec3 t=2*cross(q,v);
    return v+w*t+cross(q,t);
}
Quat Quat::from_axis_angle(Vec3 axis, double radians) {
    const double half=radians*0.5;
    const Vec3 n=rigidstep::normalized(axis)*std::sin(half);
    return {std::cos(half),n.x,n.y,n.z};
}
Aabb bounds(const Body& b) {
    if(b.shape==Shape::Sphere) {
        const Vec3 r{b.radius,b.radius,b.radius};
        return {b.position-r,b.position+r};
    }
    if(b.shape==Shape::Plane) return {};
    const Vec3 ux=b.rotation.rotate(axes[0]);
    const Vec3 uy=b.rotation.rotate(axes[1]);
    const Vec3 uz=b.rotation.rotate(axes[2]);
    const Vec3 h=abs_vec(ux)*b.half_extents.x+
                 abs_vec(uy)*b.half_extents.y+abs_vec(uz)*b.half_extents.z;
    return {b.position-h,b.position+h};
}
std::vector<Pair> broadphase_pairs(const std::vector<Body>& bodies, Broadphase mode) {
    std::vector<Pair> pairs;
    std::vector<std::size_t> order;
    std::vector<Aabb> aa(bodies.size());
    for(std::size_t i=0;i<bodies.size();++i) {
        if(bodies[i].shape==Shape::Plane) continue;
        order.push_back(i);aa[i]=bounds(bodies[i]);
    }
    if(mode==Broadphase::SweepAndPrune) {
        std::sort(order.begin(),order.end(),[&](auto i,auto j) {
            if(aa[i].min.x!=aa[j].min.x) return aa[i].min.x<aa[j].min.x;
            return bodies[i].id<bodies[j].id;
        });
        for(std::size_t i=0;i<order.size();++i) {
            const auto a=order[i];
            for(std::size_t j=i+1;j<order.size() && aa[order[j]].min.x<=aa[a].max.x;++j) {
                const auto b=order[j];
                if(bodies[a].inv_mass==0 && bodies[b].inv_mass==0) continue;
                if(overlap(aa[a],aa[b])) pairs.push_back({std::uint32_t(std::min(a,b)),std::uint32_t(std::max(a,b))});
            }
        }
    } else {
        for(std::size_t i=0;i<order.size();++i) for(std::size_t j=i+1;j<order.size();++j) {
            auto a=order[i],b=order[j];
            if(bodies[a].inv_mass==0 && bodies[b].inv_mass==0) continue;
            if(overlap(aa[a],aa[b])) pairs.push_back({std::uint32_t(a),std::uint32_t(b)});
        }
    }
    std::sort(pairs.begin(),pairs.end());
    return pairs;
}
World::World(Settings s):settings_(s) {
    if(!(s.dt>0 && std::isfinite(s.dt)) || s.solver_iterations==0)
        throw std::invalid_argument("dt must be positive finite and iterations > 0");
}
std::uint32_t World::add_sphere(Vec3 p,double r,double mass,Vec3 velocity,double restitution) {
    if(!(r>0 && std::isfinite(r)) || mass<0 || !std::isfinite(mass))
        throw std::invalid_argument("invalid sphere dimensions or mass");
    Body b;
    b.id=std::uint32_t(bodies_.size());b.shape=Shape::Sphere;
    b.position=p;b.radius=r;b.velocity=velocity;b.inv_mass=mass>0?1/mass:0;
    b.restitution=std::clamp(restitution,0.0,1.0);
    const double i=mass>0?2.5/(mass*r*r):0;
    b.inv_inertia_local={i,i,i};
    bodies_.push_back(b);return b.id;
}
std::uint32_t World::add_box(Vec3 p,Vec3 h,double mass,Quat q,Vec3 velocity,double restitution) {
    if(!(h.x>0&&h.y>0&&h.z>0) || !std::isfinite(h.x+h.y+h.z) || mass<0 || !std::isfinite(mass))
        throw std::invalid_argument("invalid box dimensions or mass");
    Body b;
    b.id=std::uint32_t(bodies_.size());b.shape=Shape::Box;
    b.position=p;b.half_extents=h;b.velocity=velocity;b.rotation=q.normalized();
    b.inv_mass=mass>0?1/mass:0;b.restitution=std::clamp(restitution,0.0,1.0);
    if(mass>0) b.inv_inertia_local={3/(mass*(h.y*h.y+h.z*h.z)),
                                  3/(mass*(h.x*h.x+h.z*h.z)),
                                  3/(mass*(h.x*h.x+h.y*h.y))};
    bodies_.push_back(b);return b.id;
}
std::uint32_t World::add_ground(double height) {
    Body b;
    b.id=std::uint32_t(bodies_.size());b.shape=Shape::Plane;
    b.position={0,height,0};b.friction=0.8;b.restitution=0;
    bodies_.push_back(b);return b.id;
}
StepStats World::step() {
    const double dt=settings_.dt;
    for(auto& b:bodies_) {
        if(b.inv_mass==0) continue;
        b.velocity+=settings_.gravity*dt;
        b.position+=b.velocity*dt;
        const double w=norm(b.angular_velocity);
        if(w>epsilon) b.rotation=(Quat::from_axis_angle(b.angular_velocity/w,w*dt)*b.rotation).normalized();
    }
    std::size_t candidates=0;
    auto cs=contacts_for(bodies_,settings_.broadphase,candidates);
    double max_depth=0;
    for(auto& c:cs) {
        max_depth=std::max(max_depth,c.penetration);
        const Body& a=bodies_[c.a];const Body& b=bodies_[c.b];
        const Vec3 relative=velocity_at(b,c.point-b.position)-velocity_at(a,c.point-a.position);
        const double approach=dot(relative,c.n);
        if(approach<-0.5) c.bounce_target=-approach*std::max(a.restitution,b.restitution);
    }
    for(std::size_t iter=0;iter<settings_.solver_iterations;++iter)
        for(auto& c:cs) solve_contact(c,bodies_,dt);
    // Conservative, mass-weighted positional stabilization; orientation is not corrected.
    for(const auto& c:cs) {
        auto& a=bodies_[c.a];auto& b=bodies_[c.b];
        const double inv=a.inv_mass+b.inv_mass;
        if(inv<=0) continue;
        const Vec3 correction=c.n*(0.25*std::max(0.0,c.penetration-0.002)/inv);
        a.position-=correction*a.inv_mass;
        b.position+=correction*b.inv_mass;
    }
    double kinetic=0;
    for(const auto& b:bodies_) if(b.inv_mass>0) {
        kinetic+=dot(b.velocity,b.velocity)/(2*b.inv_mass);
        const Vec3 wlocal=b.rotation.inverse_rotate(b.angular_velocity);
        if(b.inv_inertia_local.x>0) kinetic+=0.5*wlocal.x*wlocal.x/b.inv_inertia_local.x;
        if(b.inv_inertia_local.y>0) kinetic+=0.5*wlocal.y*wlocal.y/b.inv_inertia_local.y;
        if(b.inv_inertia_local.z>0) kinetic+=0.5*wlocal.z*wlocal.z/b.inv_inertia_local.z;
    }
    return {candidates,cs.size(),max_depth,kinetic,state_hash()};
}
std::uint64_t World::state_hash() const {
    std::uint64_t h=14695981039346656037ULL;
    for(const auto& b:bodies_) {
        hash_u64(h,b.id); hash_u64(h,std::uint64_t(b.shape));
        for(double value:{b.position.x,b.position.y,b.position.z,
                          b.velocity.x,b.velocity.y,b.velocity.z,
                          b.angular_velocity.x,b.angular_velocity.y,b.angular_velocity.z,
                          b.rotation.w,b.rotation.x,b.rotation.y,b.rotation.z}) hash_double(h,value);
    }
    return h;
}
World make_scene(const std::string& name,std::size_t count,std::uint32_t seed,Settings settings) {
    if(count>50000) throw std::invalid_argument("count exceeds safety limit 50000");
    World w(settings);w.add_ground();
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> jitter(-0.045,0.045);
    std::uniform_real_distribution<double> angle(-0.25,0.25);
    if(name!="rain" && name!="stack" && name!="sparse")
        throw std::invalid_argument("scene must be rain, stack or sparse");
    for(std::size_t i=0;i<count;++i) {
        const std::size_t width=name=="stack"?4:std::size_t(std::ceil(std::sqrt(double(count))));
        const double x=(double(i%width)-double(width)/2)*1.5+jitter(rng);
        const double z=(double((i/width)%width)-double(width)/2)*1.5+jitter(rng);
        const double y=(name=="stack"?0.65+double(i/(width*width))*1.08:
             (name=="sparse"?1.5:3.0+double(i/(width*width))*1.3))+jitter(rng);
        const Vec3 p{x,y,z};
        if(name=="sparse") {
            w.add_sphere(p,0.35,1.0,{0.04*double(i%3),0,-0.05*double(i%4)});
        } else if(i%3==0) {
            const Quat q=Quat::from_axis_angle({0,1,0},angle(rng));
            w.add_box(p,{0.48,0.48,0.48},1.4,q);
        } else {
            w.add_sphere(p,0.45,1.0);
        }
    }
    return w;
}
} // namespace rigidstep
