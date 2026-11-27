#include "rigidstep/physics.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

using namespace rigidstep;
namespace {
std::size_t assertions = 0;
void require(bool ok, const std::string& msg) {
    if (!ok) throw std::runtime_error(msg);
    ++assertions;
}
void near(double got, double expected, double eps, const std::string& msg) {
    require(std::isfinite(got) && std::abs(got - expected) <= eps,
            msg + " got=" + std::to_string(got) + " expected=" + std::to_string(expected));
}
double clamp(double x, double h) {return std::max(-h,std::min(h,x));}

void semi_implicit_ballistic_reference() {
    // Derived independently from p_{k+1}=p_k+(v_k+g*dt)*dt, without contacts.
    for (int seed=1;seed<=30;++seed) {
        const double dt=0.001*(seed%9+1);
        const Vec3 p0{seed*0.03, 7.0+seed*0.11, -0.15*seed};
        const Vec3 v0{0.2*seed, -0.1*seed, 0.25};
        const Vec3 gravity{0.1,-9.81,0.23};
        Settings s; s.dt=dt;s.gravity=gravity;
        World world(s);
        const auto id=world.add_sphere(p0,0.5,1.2,v0);
        const int n=7+seed*3;
        for(int i=0;i<n;++i) (void)world.step();
        const auto& b=world.bodies()[id];
        const Vec3 want_v=v0+gravity*(dt*n);
        const Vec3 want_p=p0+v0*(dt*n)+gravity*(dt*dt*n*(n+1)/2.0);
        near(b.velocity.x,want_v.x,2e-11,"ballistic vx");
        near(b.velocity.y,want_v.y,2e-11,"ballistic vy");
        near(b.velocity.z,want_v.z,2e-11,"ballistic vz");
        near(b.position.x,want_p.x,2e-10,"ballistic px");
        near(b.position.y,want_p.y,2e-10,"ballistic py");
        near(b.position.z,want_p.z,2e-10,"ballistic pz");
    }
}
void unequal_mass_elastic_head_on() {
    // Closed-form 1D two-body collision; restitution=1, no external force.
    struct Case {double ma,mb,va,vb;};
    const std::vector<Case> cases={{1,3,3,-1},{3,1,1,-2},{2,5,4,-2},{1,1,2,-2}};
    for(const auto& t:cases) {
        Settings s;s.dt=1.0/480;s.gravity={0,0,0};
        World w(s);
        const auto a=w.add_sphere({-1,0,0},0.5,t.ma,{t.va,0,0},1.0);
        const auto b=w.add_sphere({1,0,0},0.5,t.mb,{t.vb,0,0},1.0);
        bool collision=false;
        for(int step=0;step<650;++step) collision|=w.step().contacts>0;
        require(collision,"must actually collide");
        const double expected_a=((t.ma-t.mb)*t.va+2*t.mb*t.vb)/(t.ma+t.mb);
        const double expected_b=(2*t.ma*t.va+(t.mb-t.ma)*t.vb)/(t.ma+t.mb);
        near(w.bodies()[a].velocity.x,expected_a,0.025,"elastic unequal mass a");
        near(w.bodies()[b].velocity.x,expected_b,0.025,"elastic unequal mass b");
        near(t.ma*w.bodies()[a].velocity.x+t.mb*w.bodies()[b].velocity.x,
             t.ma*t.va+t.mb*t.vb,1e-8,"1D total linear momentum");
    }
}
void rotated_box_bounds_vertex_oracle() {
    std::mt19937 rng(9981);
    std::uniform_real_distribution<double> p(-5,5),h(0.1,1.7),theta(-3.1,3.1);
    for(int i=0;i<400;++i) {
        Settings s;s.gravity={0,0,0};World w(s);
        const Vec3 pos{p(rng),p(rng),p(rng)},half{h(rng),h(rng),h(rng)};
        const Quat q=Quat::from_axis_angle({p(rng),p(rng),p(rng)},theta(rng));
        const auto id=w.add_box(pos,half,1.5,q);
        const Aabb a=bounds(w.bodies()[id]);
        Vec3 lo{1e20,1e20,1e20},hi{-1e20,-1e20,-1e20};
        for(int bit=0;bit<8;++bit) {
            const Vec3 corner=pos+w.bodies()[id].rotation.rotate({(bit&1)?half.x:-half.x,
                (bit&2)?half.y:-half.y,(bit&4)?half.z:-half.z});
            lo.x=std::min(lo.x,corner.x);lo.y=std::min(lo.y,corner.y);lo.z=std::min(lo.z,corner.z);
            hi.x=std::max(hi.x,corner.x);hi.y=std::max(hi.y,corner.y);hi.z=std::max(hi.z,corner.z);
        }
        near(a.min.x,lo.x,1e-10,"obb min x");near(a.min.y,lo.y,1e-10,"obb min y");
        near(a.min.z,lo.z,1e-10,"obb min z");near(a.max.x,hi.x,1e-10,"obb max x");
        near(a.max.y,hi.y,1e-10,"obb max y");near(a.max.z,hi.z,1e-10,"obb max z");
    }
}
void sphere_obb_distance_oracle() {
    // Independent closest-point distance against a rotated box.
    std::mt19937 rng(1729);
    std::uniform_real_distribution<double> pos(-2.5,2.5),theta(-2.0,2.0),radius(0.15,0.65);
    std::size_t separated=0, touching=0;
    for(int i=0;i<650;++i) {
        const Quat q=Quat::from_axis_angle({1,1,0.4},theta(rng));
        const Vec3 center{pos(rng)*0.3,pos(rng)*0.3,pos(rng)*0.3};
        const Vec3 h{0.5,0.8,0.35},sphere_center{pos(rng),pos(rng),pos(rng)};
        const double r=radius(rng);
        const Vec3 local=q.inverse_rotate(sphere_center-center);
        const Vec3 nearest{clamp(local.x,h.x),clamp(local.y,h.y),clamp(local.z,h.z)};
        const double dist=norm(local-nearest);
        if(std::abs(dist-r)<1e-7)continue; // explicitly exclude tolerance boundary
        Settings s;s.gravity={0,0,0};World w(s);
        w.add_box(center,h,0,q);
        w.add_sphere(sphere_center,r,1.0);
        const bool contact=!w.collision_contacts().empty();
        require(contact==(dist<r),"rotated sphere/OBB independent closest-point oracle");
        touching+=contact;separated+=!contact;
    }
    require(touching>10&&separated>100,"oracle contains both positive and negative examples");
}
}
int main() {
    try {
        semi_implicit_ballistic_reference();
        unequal_mass_elastic_head_on();
        rotated_box_bounds_vertex_oracle();
        sphere_obb_distance_oracle();
        std::cout<<"PASS: independent analytic reference oracles; assertions="<<assertions<<"\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr<<"FAIL: "<<e.what()<<" after "<<assertions<<" assertions\n";
        return 1;
    }
}
