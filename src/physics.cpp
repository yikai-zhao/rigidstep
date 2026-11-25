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
    ContactFeature feature{ContactFeature::SphereSphere};
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
    c.feature=ContactFeature::SphereSphere;
    c.penetration = r-d;
    c.point = a.position + c.n*(a.radius-c.penetration*0.5);
    return true;
}

// c.n always points from sphere to box; a sphere fully inside a box is pushed out.
bool collide_sphere_box(const Body& sphere, const Body& box, Contact& c) {
    c.feature=ContactFeature::SphereBox;
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

// SAT identifies the minimum-separation face or edge axis. For a face axis,
// Sutherland-Hodgman clips the incident quad against the reference box's four
// side planes, retaining up to four spread-out points. Cross-axis cases fall
// back to a single approximate contact, which remains a documented limitation.
void collide_boxes(const Body& a, const Body& b, std::size_t ai, std::size_t bi,
                   std::vector<Contact>& out, bool precise_edges) {
    const Vec3 delta=b.position-a.position;
    const Vec3 pa[]={a.rotation.rotate(axes[0]),a.rotation.rotate(axes[1]),a.rotation.rotate(axes[2])};
    const Vec3 pb[]={b.rotation.rotate(axes[0]),b.rotation.rotate(axes[1]),b.rotation.rotate(axes[2])};
    double min_depth=std::numeric_limits<double>::infinity();
    Vec3 min_axis{};
    int feature=-1; // 0..2: A faces, 3..5: B faces, 6..14: edge-edge.
    auto test_axis=[&](Vec3 axis, int feature_id) {
        const double l=norm(axis);
        if(l<1e-7) return true;
        axis=axis/l;
        double ra=0,rb=0;
        for(int i=0;i<3;++i) {
            ra+=component(a.half_extents,i)*std::abs(dot(pa[i],axis));
            rb+=component(b.half_extents,i)*std::abs(dot(pb[i],axis));
        }
        const double depth=ra+rb-std::abs(dot(delta,axis));
        if(depth<=0) return false;
        // Prefer face normals in near ties, reducing jitter from tiny cross axes.
        const double tie_bias=feature_id>=6 ? 1e-5 : 0.0;
        if(depth+tie_bias<min_depth) {
            min_depth=depth;
            min_axis=dot(delta,axis)>=0?axis:-axis;
            feature=feature_id;
        }
        return true;
    };
    for(int i=0;i<3;++i) if(!test_axis(pa[i],i)) return;
    for(int i=0;i<3;++i) if(!test_axis(pb[i],i+3)) return;
    for(int i=0;i<3;++i) for(int j=0;j<3;++j)
        if(!test_axis(cross(pa[i],pb[j]),6+3*i+j)) return;
    auto fallback=[&]() {
        Contact c{};c.a=ai;c.b=bi;c.n=min_axis;c.penetration=min_depth;
        c.feature=feature>=6?ContactFeature::BoxEdge:ContactFeature::BoxFace;
        if(feature>=6 && precise_edges) {
            // SAT cross product is an edge/edge axis. Choose the two most
            // extreme *support edges*, then use segment closest points rather
            // than arbitrary support vertices (old approximation).
            const int edge_a=(feature-6)/3, edge_b=(feature-6)%3;
            Vec3 center_a=a.position,center_b=b.position;
            for(int k=0;k<3;++k) {
                if(k!=edge_a)
                    center_a+=pa[k]*(component(a.half_extents,k)*
                                        (dot(pa[k],min_axis)>=0?1.0:-1.0));
                if(k!=edge_b)
                    center_b+=pb[k]*(component(b.half_extents,k)*
                                        (dot(pb[k],min_axis)>=0?-1.0:1.0));
            }
            const Vec3 da=pa[edge_a]*component(a.half_extents,edge_a);
            const Vec3 db=pb[edge_b]*component(b.half_extents,edge_b);
            const auto witness=closest_segment_points(center_a-da,center_a+da,
                                                      center_b-db,center_b+db);
            c.point=(witness.on_a+witness.on_b)*0.5;
        } else {
            // Safety fallback only for a degenerate clipped face polygon.
            c.point=(support(a,c.n)+support(b,-c.n))*0.5;
        }
        out.push_back(c);
    };
    if(feature>=6 || feature<0) {fallback();return;}

    const bool ref_is_a=feature<3;
    const Body& ref=ref_is_a?a:b;
    const Body& inc=ref_is_a?b:a;
    const int ref_axis=feature%3;
    const Vec3 normal=ref_is_a?min_axis:-min_axis; // reference -> incident
    const Vec3 ref_basis[]={ref.rotation.rotate(axes[0]),
                            ref.rotation.rotate(axes[1]),ref.rotation.rotate(axes[2])};
    const Vec3 inc_basis[]={inc.rotation.rotate(axes[0]),
                            inc.rotation.rotate(axes[1]),inc.rotation.rotate(axes[2])};
    int inc_axis=0;
    for(int i=1;i<3;++i)
        if(std::abs(dot(inc_basis[i],normal))>std::abs(dot(inc_basis[inc_axis],normal))) inc_axis=i;
    const double inc_sign=dot(inc_basis[inc_axis],normal)>0?-1.0:1.0;
    const int u=(inc_axis+1)%3,v=(inc_axis+2)%3;
    const Vec3 inc_face=inc.position+inc_basis[inc_axis]*
                         (inc_sign*component(inc.half_extents,inc_axis));
    std::vector<Vec3> poly;
    // Cyclic quad winding; the clipping routine does not require a winding normal.
    const std::pair<int,int> corners[]={{1,1},{1,-1},{-1,-1},{-1,1}};
    for(const auto& sign : corners) {
        poly.push_back(inc_face+inc_basis[u]*(double(sign.first)*component(inc.half_extents,u))+
                       inc_basis[v]*(double(sign.second)*component(inc.half_extents,v)));
    }
    const auto clip=[&](Vec3 axis,double limit) {
        if(poly.empty()) return;
        std::vector<Vec3> next;
        Vec3 prev=poly.back();
        double dp=dot(prev-ref.position,axis)-limit;
        for(const Vec3 curr:poly) {
            const double dc=dot(curr-ref.position,axis)-limit;
            if((dp<=0)!=(dc<=0)) {
                const double denom=dp-dc;
                const double t=std::abs(denom)>epsilon?std::clamp(dp/denom,0.0,1.0):0.0;
                next.push_back(prev+(curr-prev)*t);
            }
            if(dc<=0) next.push_back(curr);
            prev=curr;dp=dc;
        }
        poly.swap(next);
    };
    for(int j=0;j<3;++j) if(j!=ref_axis) {
        clip(ref_basis[j],component(ref.half_extents,j));
        clip(-ref_basis[j],component(ref.half_extents,j));
    }
    const Vec3 face=ref.position+normal*component(ref.half_extents,ref_axis);
    std::vector<Contact> manifold;
    for(const Vec3 p:poly) {
        const double depth=dot(face-p,normal);
        if(depth<-1e-6) continue;
        Contact c{};c.a=ai;c.b=bi;c.n=min_axis;
        c.penetration=std::max(0.0,depth);
        c.feature=ContactFeature::BoxFace;
        c.point=p+normal*(depth*0.5);
        bool duplicate=false;
        for(const auto& prior:manifold) {
            const Vec3 d=c.point-prior.point;
            if(dot(d,d)<1e-12) {duplicate=true;break;}
        }
        if(!duplicate) manifold.push_back(c);
    }
    if(manifold.empty()) {fallback();return;}
    // Keep spatially separated contacts instead of blindly keeping the deepest
    // four (which may all cluster around one edge in an oblique overlap).
    std::vector<Contact> selected;
    while(!manifold.empty() && selected.size()<4) {
        std::size_t best=0;
        double value=-1.0;
        for(std::size_t i=0;i<manifold.size();++i) {
            double score=manifold[i].penetration;
            if(!selected.empty()) {
                double distance=std::numeric_limits<double>::infinity();
                for(const auto& old:selected) {
                    const Vec3 d=old.point-manifold[i].point;
                    distance=std::min(distance,dot(d,d));
                }
                score=distance;
            }
            if(score>value) {value=score;best=i;}
        }
        selected.push_back(manifold[best]);
        manifold.erase(manifold.begin()+static_cast<std::ptrdiff_t>(best));
    }
    out.insert(out.end(),selected.begin(),selected.end());
}

void ground_contacts(const Body& plane, const Body& b, std::size_t pi,
                     std::size_t bi, std::vector<Contact>& out) {
    const double height=plane.position.y;
    if (b.shape==Shape::Sphere) {
        const double depth=height-(b.position.y-b.radius);
        if (depth>0) {
            Contact c{};c.a=pi;c.b=bi;c.n={0,1,0};
            c.point={b.position.x,height,b.position.z};c.penetration=depth;
            c.feature=ContactFeature::Ground;
            out.push_back(c);
        }
    } else if (b.shape==Shape::Box) {
        const auto start=out.size();
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
            if(!duplicate) {
                Contact c{};c.a=pi;c.b=bi;c.n={0,1,0};
                c.point=on_plane;c.penetration=height-p.y;
                c.feature=ContactFeature::Ground;
                out.push_back(c);
            }
        }
        // A completely submerged box can have 8 vertices below the plane.
        // Limit this *one body/plane pair* to its four deepest constraints;
        // do not accumulate 8 impulses against the same static surface.
        if(out.size()-start>4) {
            std::stable_sort(out.begin()+static_cast<std::ptrdiff_t>(start),out.end(),
                             [](const Contact& lhs,const Contact& rhs) {
                                 return lhs.penetration>rhs.penetration;
                             });
            out.resize(start+4);
        }
    }
}

std::vector<Contact> contacts_for(const std::vector<Body>& bodies, Broadphase mode,
                                  std::size_t& candidate_count, bool precise_edges) {
    std::vector<Contact> out;
    const auto candidates=broadphase_pairs(bodies,mode);
    candidate_count=candidates.size();
    for (const auto& pair:candidates) {
        const Body& a=bodies[pair.a]; const Body& b=bodies[pair.b];
        if (a.shape==Shape::Plane || b.shape==Shape::Plane) continue;
        Contact c{}; c.a=pair.a; c.b=pair.b;
        bool hit=false;
        if(a.shape==Shape::Sphere && b.shape==Shape::Sphere) hit=collide_spheres(a,b,c);
        if(a.shape==Shape::Sphere && b.shape==Shape::Box) hit=collide_sphere_box(a,b,c);
        if(a.shape==Shape::Box && b.shape==Shape::Sphere) {
            hit=collide_sphere_box(b,a,c); c.n=-c.n;
        }
        if(a.shape==Shape::Box && b.shape==Shape::Box) {
            collide_boxes(a,b,pair.a,pair.b,out,precise_edges);
        } else if(hit) out.push_back(c);
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

// This is the clamped Ericson segment-segment minimizer. No normalization
// by near-zero segment length, and the parallel branch is deterministic.
SegmentWitness closest_segment_points(Vec3 a0,Vec3 a1,Vec3 b0,Vec3 b1) {
    const Vec3 d1=a1-a0,d2=b1-b0,r=a0-b0;
    const double a=dot(d1,d1),e=dot(d2,d2),f=dot(d2,r);
    double s=0,t=0;
    constexpr double tiny=1e-20;
    if(a<=tiny && e<=tiny) return {a0,b0};
    if(a<=tiny) t=std::clamp(f/e,0.0,1.0);
    else {
        const double c=dot(d1,r);
        if(e<=tiny) s=std::clamp(-c/a,0.0,1.0);
        else {
            const double b=dot(d1,d2),denom=a*e-b*b;
            if(denom>tiny*std::max(a*e,1.0)) s=std::clamp((b*f-c*e)/denom,0.0,1.0);
            t=(b*s+f)/e;
            if(t<0) {t=0;s=std::clamp(-c/a,0.0,1.0);}
            else if(t>1) {t=1;s=std::clamp((b-c)/a,0.0,1.0);}
        }
    }
    return {a0+d1*s,b0+d2*t};
}

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
    if(!(s.dt>0 && std::isfinite(s.dt)) || s.solver_iterations==0 ||
       !std::isfinite(s.gravity.x)||!std::isfinite(s.gravity.y)||!std::isfinite(s.gravity.z))
        throw std::invalid_argument("dt must be positive finite and iterations > 0");
}
std::uint32_t World::add_sphere(Vec3 p,double r,double mass,Vec3 velocity,double restitution) {
    if(!(r>0 && std::isfinite(r)) || mass<0 || !std::isfinite(mass) ||
       !std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)||
       !std::isfinite(velocity.x)||!std::isfinite(velocity.y)||!std::isfinite(velocity.z)||
       !std::isfinite(restitution))
        throw std::invalid_argument("invalid sphere dimensions or mass");
    Body b;
    b.id=std::uint32_t(bodies_.size());b.shape=Shape::Sphere;
    b.position=p;b.radius=r;b.velocity=velocity;b.inv_mass=mass>0?1/mass:0;
    b.restitution=std::clamp(restitution,0.0,1.0);
    const double i=mass>0?2.5/(mass*r*r):0;
    b.inv_inertia_local={i,i,i};
    contact_cache_.clear();
    bodies_.push_back(b);return b.id;
}
std::uint32_t World::add_box(Vec3 p,Vec3 h,double mass,Quat q,Vec3 velocity,double restitution) {
    if(!(h.x>0&&h.y>0&&h.z>0) || !std::isfinite(h.x+h.y+h.z) ||
       mass<0 || !std::isfinite(mass) || !std::isfinite(restitution) ||
       !std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)||
       !std::isfinite(velocity.x)||!std::isfinite(velocity.y)||!std::isfinite(velocity.z)||
       !std::isfinite(q.w)||!std::isfinite(q.x)||!std::isfinite(q.y)||!std::isfinite(q.z))
        throw std::invalid_argument("invalid box dimensions or mass");
    Body b;
    b.id=std::uint32_t(bodies_.size());b.shape=Shape::Box;
    b.position=p;b.half_extents=h;b.velocity=velocity;b.rotation=q.normalized();
    b.inv_mass=mass>0?1/mass:0;b.restitution=std::clamp(restitution,0.0,1.0);
    if(mass>0) b.inv_inertia_local={3/(mass*(h.y*h.y+h.z*h.z)),
                                  3/(mass*(h.x*h.x+h.z*h.z)),
                                  3/(mass*(h.x*h.x+h.y*h.y))};
    contact_cache_.clear();
    bodies_.push_back(b);return b.id;
}
std::uint32_t World::add_ground(double height) {
    if(!std::isfinite(height)) throw std::invalid_argument("ground height must be finite");
    Body b;
    b.id=std::uint32_t(bodies_.size());b.shape=Shape::Plane;
    b.position={0,height,0};b.friction=0.8;b.restitution=0;
    contact_cache_.clear();
    bodies_.push_back(b);return b.id;
}
std::vector<ContactDebug> World::collision_contacts() const {
    std::size_t unused=0;
    const auto contacts=contacts_for(bodies_,settings_.broadphase,unused,settings_.precise_edge_contacts);
    std::vector<ContactDebug> result;
    result.reserve(contacts.size());
    for(const auto& c:contacts) {
        result.push_back({static_cast<std::uint32_t>(c.a),static_cast<std::uint32_t>(c.b),
                          c.n,c.point,c.penetration,c.feature});
    }
    return result;
}
StepStats World::step() {
    const double dt=settings_.dt;
    double highest_ground=-std::numeric_limits<double>::infinity();
    if(settings_.sphere_plane_ccd) {
        for(const auto& b:bodies_) if(b.shape==Shape::Plane)
            highest_ground=std::max(highest_ground,b.position.y);
    }
    for(auto& b:bodies_) {
        if(b.inv_mass==0) continue;
        b.velocity+=settings_.gravity*dt;
        // Limited CCD: sweep a sphere against the topmost horizontal plane only.
        // This does not handle sphere/box, sphere/sphere or rotating body sweeps.
        const double floor=highest_ground+b.radius;
        if(settings_.sphere_plane_ccd && b.shape==Shape::Sphere &&
           std::isfinite(highest_ground) && b.velocity.y<0 &&
           b.position.y>=floor && b.position.y+b.velocity.y*dt<floor) {
            const double toi=(floor-b.position.y)/b.velocity.y;
            b.position+=b.velocity*toi;
            b.position.y=floor;
            b.velocity.y=-b.velocity.y*b.restitution;
            b.position+=b.velocity*(dt-toi);
        } else {
            b.position+=b.velocity*dt;
        }
        const double w=norm(b.angular_velocity);
        if(w>epsilon) b.rotation=(Quat::from_axis_angle(b.angular_velocity/w,w*dt)*b.rotation).normalized();
    }
    std::size_t candidates=0;
    auto cs=contacts_for(bodies_,settings_.broadphase,candidates,settings_.precise_edge_contacts);
    double max_depth=0;
    std::size_t edge_count=0,face_count=0;
    for(auto& c:cs) {
        edge_count+=c.feature==ContactFeature::BoxEdge;
        face_count+=c.feature==ContactFeature::BoxFace;
        max_depth=std::max(max_depth,c.penetration);
        const Body& a=bodies_[c.a];const Body& b=bodies_[c.b];
        const Vec3 relative=velocity_at(b,c.point-b.position)-velocity_at(a,c.point-a.position);
        const double approach=dot(relative,c.n);
        if(approach<-0.5) c.bounce_target=-approach*std::max(a.restitution,b.restitution);
    }
    // Warm-start contact constraints using stable body-local anchors, rather
    // than transient clipped polygon slot indices. Every prior contact can be
    // matched at most once, and only when both anchors and the normal agree.
    std::size_t matched=0, warm_applied=0;
    if(settings_.warm_starting) {
        std::vector<bool> used(contact_cache_.size(),false);
        constexpr double match_distance_sq=0.10*0.10; // world units; scope is meter-scale scenes
        for(auto& c:cs) {
            const Body& a=bodies_[c.a];const Body& b=bodies_[c.b];
            const Vec3 la=a.rotation.inverse_rotate(c.point-a.position);
            const Vec3 lb=b.rotation.inverse_rotate(c.point-b.position);
            std::size_t best=contact_cache_.size();
            double nearest=2.0*match_distance_sq;
            for(std::size_t i=0;i<contact_cache_.size();++i) {
                const auto& old=contact_cache_[i];
                if(used[i] || old.a!=c.a || old.b!=c.b || dot(old.normal,c.n)<0.95) continue;
                const Vec3 da=la-old.anchor_a,db=lb-old.anchor_b;
                const double d1=dot(da,da),d2=dot(db,db);
                if(d1>match_distance_sq || d2>match_distance_sq) continue;
                const double score=d1+d2;
                if(score<nearest) {nearest=score;best=i;}
            }
            if(best==contact_cache_.size()) continue;
            used[best]=true;
            ++matched;
            const auto& old=contact_cache_[best];
            // Bounce events are new impacts, not a quasi-static constraint.
            if(c.bounce_target>0) continue;
            // Under-relaxed experimental warm start: full reuse caused
            // inconsistent dense-stack results in the ablation.
            c.normal_impulse=0.5*std::max(0.0,old.normal_impulse);
            c.tangent_impulse=0.5*(old.tangent_impulse-c.n*dot(old.tangent_impulse,c.n));
            const double mu=std::sqrt(a.friction*b.friction)*c.normal_impulse;
            const double tangent_len=norm(c.tangent_impulse);
            if(tangent_len>mu && tangent_len>epsilon) c.tangent_impulse*=mu/tangent_len;
            if(c.normal_impulse<=epsilon && norm(c.tangent_impulse)<=epsilon) continue;
            const Vec3 j=c.n*c.normal_impulse+c.tangent_impulse;
            impulse(bodies_[c.a],-j,c.point-bodies_[c.a].position);
            impulse(bodies_[c.b],j,c.point-bodies_[c.b].position);
            ++warm_applied;
        }
    }
    for(std::size_t iter=0;iter<settings_.solver_iterations;++iter)
        for(auto& c:cs) solve_contact(c,bodies_,dt);
    // Rebuild from actual solved contacts. Disappeared pairs are evicted each
    // frame; no unbounded cache, no reuse after separation or a new body add.
    contact_cache_.clear();
    if(settings_.warm_starting) {
        contact_cache_.reserve(cs.size());
        for(const auto& c:cs) {
            if(c.normal_impulse<=epsilon) continue;
            const Body& a=bodies_[c.a];const Body& b=bodies_[c.b];
            contact_cache_.push_back({static_cast<std::uint32_t>(c.a),
                static_cast<std::uint32_t>(c.b),
                a.rotation.inverse_rotate(c.point-a.position),
                b.rotation.inverse_rotate(c.point-b.position),c.n,c.tangent_impulse,c.normal_impulse});
        }
    }

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
    return {candidates,cs.size(),max_depth,kinetic,state_hash(),matched,warm_applied,
            contact_cache_.size(),edge_count,face_count};
}
std::uint64_t World::state_hash() const {
    std::uint64_t h=14695981039346656037ULL;
    hash_double(h,settings_.dt);
    for(double v:{settings_.gravity.x,settings_.gravity.y,settings_.gravity.z}) hash_double(h,v);
    hash_u64(h,settings_.solver_iterations);
    hash_u64(h,settings_.sphere_plane_ccd?1U:0U);
    hash_u64(h,settings_.warm_starting?1U:0U);
    hash_u64(h,settings_.precise_edge_contacts?1U:0U);
    for(const auto& b:bodies_) {
        hash_u64(h,b.id); hash_u64(h,std::uint64_t(b.shape));
        for(double value:{b.position.x,b.position.y,b.position.z,
                          b.velocity.x,b.velocity.y,b.velocity.z,
                          b.angular_velocity.x,b.angular_velocity.y,b.angular_velocity.z,
                          b.rotation.w,b.rotation.x,b.rotation.y,b.rotation.z,
                          b.half_extents.x,b.half_extents.y,b.half_extents.z,b.radius,
                          b.inv_mass,b.restitution,b.friction,
                          b.inv_inertia_local.x,b.inv_inertia_local.y,b.inv_inertia_local.z}) hash_double(h,value);
    }
    // Cached impulses affect the next step, so they are part of the replay
    // state (unlike broadphase choice). This detects a hidden-state mismatch.
    for(const auto& c:contact_cache_) {
        hash_u64(h,c.a);hash_u64(h,c.b);
        for(double v:{c.anchor_a.x,c.anchor_a.y,c.anchor_a.z,
                      c.anchor_b.x,c.anchor_b.y,c.anchor_b.z,
                      c.normal.x,c.normal.y,c.normal.z,
                      c.tangent_impulse.x,c.tangent_impulse.y,c.tangent_impulse.z,
                      c.normal_impulse}) hash_double(h,v);
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
