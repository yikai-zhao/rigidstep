#include "rigidstep/physics.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

using namespace rigidstep;
namespace {
std::size_t passed=0;
void check(bool b,const std::string& message) {
    if(!b) throw std::runtime_error(message);
    ++passed;
}
void near(double a,double b,double tolerance,const std::string& message) {
    check(std::isfinite(a)&&std::abs(a-b)<=tolerance,
          message+": "+std::to_string(a)+" vs "+std::to_string(b));
}
void test_math() {
    const Vec3 x{1,0,0},y{0,1,0};
    const auto z=cross(x,y);
    check(dot(z,Vec3{0,0,1})==1,"cross product handedness");
    const auto q=Quat::from_axis_angle({0,0,1},3.14159265358979323846/2);
    near(q.rotate(x).x,0,1e-12,"quaternion x");
    near(q.rotate(x).y,1,1e-12,"quaternion y");
    near(q.inverse_rotate(q.rotate({1,2,3})).z,3,1e-12,"inverse rotation");
}
void test_broadphase_oracle() {
    for(std::uint32_t seed=1;seed<=20;++seed) {
        std::mt19937 gen(seed);
        std::uniform_real_distribution<double> p(-10,10),h(0.1,1.5),angle(-3,3);
        World w;
        for(std::size_t i=0;i<90;++i) {
            const Vec3 pos{p(gen),p(gen),p(gen)};
            if(i%3==0) w.add_sphere(pos,h(gen),1);
            else w.add_box(pos,{h(gen),h(gen),h(gen)},1,
                           Quat::from_axis_angle({1,1,1},angle(gen)));
        }
        const auto sap=broadphase_pairs(w.bodies(),Broadphase::SweepAndPrune);
        const auto brute=broadphase_pairs(w.bodies(),Broadphase::BruteForce);
        check(sap==brute,"SAP/brute pair mismatch seed "+std::to_string(seed));
    }
}
void test_free_flight() {
    Settings s;s.gravity={0,0,0};
    World w(s);
    const auto id=w.add_sphere({0,0,0},0.5,1,{1,2,3});
    for(int i=0;i<120;++i) (void)w.step();
    near(w.bodies()[id].position.x,1,1e-12,"free flight x");
    near(w.bodies()[id].position.y,2,1e-12,"free flight y");
    near(w.bodies()[id].position.z,3,1e-12,"free flight z");
}
void test_equal_sphere_collision() {
    Settings s;s.gravity={0,0,0};
    World w(s);
    auto a=w.add_sphere({-1,0,0},0.5,1,{2,0,0},1);
    auto b=w.add_sphere({1,0,0},0.5,1,{-2,0,0},1);
    const double momentum_before=w.bodies()[a].velocity.x+w.bodies()[b].velocity.x;
    bool collided=false;
    for(int i=0;i<110;++i) {
        const auto stats=w.step();collided|=stats.contacts>0;
    }
    check(collided,"spheres must collide");
    near(w.bodies()[a].velocity.x,-2,0.1,"elastic sphere 1");
    near(w.bodies()[b].velocity.x,2,0.1,"elastic sphere 2");
    near(w.bodies()[a].velocity.x+w.bodies()[b].velocity.x,momentum_before,1e-9,"linear momentum");
}
void test_ground_stability() {
    World w;
    w.add_ground();
    const auto b=w.add_sphere({0,3,0},0.5,1);
    for(int i=0;i<1200;++i) {
        const auto s=w.step();
        check(std::isfinite(s.kinetic_energy),"finite kinetic energy");
    }
    near(w.bodies()[b].position.y,0.5,0.02,"sphere rests on ground");
    check(std::abs(w.bodies()[b].velocity.y)<0.25,"ground settling speed");
}
void test_rotated_box_ground() {
    World w;
    w.add_ground();
    const auto b=w.add_box({0,2,0},{0.6,0.4,0.5},2,
        Quat::from_axis_angle({0,0,1},0.12));
    for(int i=0;i<800;++i) (void)w.step();
    check(std::isfinite(w.bodies()[b].position.y),"rotated box finite");
    check(w.bodies()[b].position.y>0.2 && w.bodies()[b].position.y<1.0,
          "rotated box above floor");
}
void test_sphere_static_box() {
    Settings s;s.gravity={0,0,0};
    World w(s);
    w.add_box({0,0,0},{0.5,0.5,0.5},0);
    const auto b=w.add_sphere({-2,0,0},0.5,1,{3,0,0},0.7);
    bool collided=false;
    for(int i=0;i<130;++i) collided|=w.step().contacts>0;
    check(collided,"sphere-box collision detected");
    check(w.bodies()[b].velocity.x<0,"sphere reflected by static box");
}
void test_box_box_sat() {
    Settings s;s.gravity={0,0,0};
    World w(s);
    w.add_box({0,0,0},{0.5,0.5,0.5},0,
              Quat::from_axis_angle({0,1,0},0.25));
    const auto b=w.add_box({-2,0,0},{0.5,0.5,0.5},1,{}, {3,0,0},0.3);
    bool collided=false;
    for(int i=0;i<140;++i) collided|=w.step().contacts>0;
    check(collided,"OBB-OBB SAT must detect collision");
    check(std::isfinite(w.bodies()[b].angular_velocity.y),"box angular velocity finite");
}
void test_deterministic_replay() {
    World a=make_scene("rain",48,21);
    World b=make_scene("rain",48,21);
    check(a.state_hash()==b.state_hash(),"identical initial state");
    for(int i=0;i<240;++i) {
        const auto x=a.step();const auto y=b.step();
        check(x.state_hash==y.state_hash,"determinism at tick "+std::to_string(i));
        check(std::isfinite(x.kinetic_energy),"finite replay energy");
    }
    World c=make_scene("rain",48,22);
    check(c.state_hash()!=a.state_hash(),"different seed differs");
}

void test_cross_algorithm_trajectory() {
    Settings brute; brute.broadphase=Broadphase::BruteForce;
    Settings sap; sap.broadphase=Broadphase::SweepAndPrune;
    for(std::uint32_t seed:{3U,11U,27U}) {
        World a=make_scene("rain",36,seed,brute);
        World b=make_scene("rain",36,seed,sap);
        for(int tick=0;tick<180;++tick) {
            const StepStats left=a.step();const StepStats right=b.step();
            check(left.state_hash==right.state_hash,
                 "broadphase mode must not change physics state, tick "+std::to_string(tick));
            check(left.contacts==right.contacts,"contact count parity");
        }
    }
}
void test_ground_bounce() {
    World w;
    w.add_ground();
    const auto b=w.add_sphere({0,1,0},0.3,1,{},0.85);
    bool saw_contact=false,saw_rebound=false;
    for(int i=0;i<250;++i) {
        const auto stats=w.step();
        saw_contact|=stats.contacts>0;
        if(saw_contact && w.bodies()[b].velocity.y>0.1) saw_rebound=true;
    }
    check(saw_contact && saw_rebound,"restitution causes upward rebound");
}

void test_box_face_manifold() {
    Settings opt;opt.gravity={0,0,0};
    World w(opt);
    w.add_box({0,0,0},{0.5,0.5,0.5},0);
    const auto id=w.add_box({0,0.9,0},{0.5,0.5,0.5},1);
    const auto result=w.step();
    check(result.contacts==4,"axis-aligned OBB face-face should clip to four contacts");
    check(std::isfinite(result.kinetic_energy),"contact manifold energy finite");
    check(w.bodies()[id].position.y>0.9,"contact positional correction separates boxes");
}
void test_rotated_manifold_stress() {
    Settings opt;opt.gravity={0,0,0};
    for(std::uint32_t seed=1;seed<=15;++seed) {
        World a(opt),b(opt);
        const double theta=0.01*double(seed);
        a.add_box({0,0,0},{1,0.4,0.8},0,Quat::from_axis_angle({0,1,0},theta));
        b.add_box({0,0,0},{1,0.4,0.8},0,Quat::from_axis_angle({0,1,0},theta));
        a.add_box({0,0.7,0},{0.8,0.4,0.8},1,Quat::from_axis_angle({1,1,0},theta));
        b.add_box({0,0.7,0},{0.8,0.4,0.8},1,Quat::from_axis_angle({1,1,0},theta));
        const auto first=a.step();
        check(first.contacts>=1 && first.contacts<=4,"bounded rotated box contact manifold");
        check(first.state_hash==b.step().state_hash,"rotated box deterministic initial collision");
        for(int t=0;t<180;++t) {
            const auto x=a.step();const auto y=b.step();
            check(x.state_hash==y.state_hash,"rotated manifold replay");
            check(std::isfinite(x.kinetic_energy),"rotated manifold finite energy");
        }
    }
}
void test_sphere_plane_ccd() {
    Settings s;s.gravity={0,0,0};s.dt=1.0/60.0;s.sphere_plane_ccd=true;
    World w(s);
    w.add_ground();
    const auto id=w.add_sphere({0,2,0},0.25,1,{0,-400,0},0.8);
    const auto tick=w.step();
    check(tick.contacts==0,"CCD should keep discrete solver from spurious below-floor state");
    check(w.bodies()[id].position.y>=0.25,"high-speed CCD sphere above floor");
    check(w.bodies()[id].velocity.y>0,"CCD reflected high-speed sphere");
    // Non-CCD negative control reproduces tunneling: discrete collision may
    // correct penetration, but does not guarantee no passage below a plane.
    s.sphere_plane_ccd=false;
    World control(s);control.add_ground();
    const auto control_id=control.add_sphere({0,2,0},0.25,1,{0,-400,0},0.8);
    (void)control.step();
    check(control.bodies()[control_id].position.y<0.25,
          "discrete negative control is vulnerable to high-speed tunneling");
}
void test_ccd_replay_and_scope() {
    Settings s;s.sphere_plane_ccd=true;
    World a(s),b(s);
    a.add_ground();b.add_ground();
    for(int i=0;i<16;++i) {
        Vec3 p{double(i%4),3+double(i/4),0};
        a.add_sphere(p,0.2,1,{0,-20-double(i),0},0.6);
        b.add_sphere(p,0.2,1,{0,-20-double(i),0},0.6);
    }
    for(int t=0;t<200;++t) {
        const auto x=a.step(),y=b.step();
        check(x.state_hash==y.state_hash,"CCD replay state parity");
        check(std::isfinite(x.kinetic_energy),"CCD finite energy");
    }
}
void test_invalid_inputs() {
    bool caught=false;
    try {World w;w.add_sphere({0,0,0},-1,1);} catch(const std::invalid_argument&) {caught=true;}
    check(caught,"reject negative sphere radius");
    caught=false;
    try {World w;w.add_box({0,0,0},{0,1,1},1);} catch(const std::invalid_argument&) {caught=true;}
    check(caught,"reject zero box extent");
    caught=false;
    try {World w;w.add_sphere({std::nan(""),0,0},0.5,1);} catch(const std::invalid_argument&) {caught=true;}
    check(caught,"reject NaN position");
    caught=false;
    try {World w;w.add_box({0,0,0},{0.5,0.5,0.5},1,{1,std::nan(""),0,0});}
    catch(const std::invalid_argument&) {caught=true;}
    check(caught,"reject non-finite quaternion");
    caught=false;
    try {Settings s;s.gravity={std::nan(""),0,0};World w(s);}
    catch(const std::invalid_argument&) {caught=true;}
    check(caught,"reject NaN gravity");
}
}
int main() {
    try {
        test_math();test_broadphase_oracle();test_free_flight();
        test_equal_sphere_collision();test_ground_stability();
        test_rotated_box_ground();test_sphere_static_box();
        test_box_box_sat();test_deterministic_replay();test_invalid_inputs();
        test_cross_algorithm_trajectory();test_ground_bounce();
        test_box_face_manifold();test_rotated_manifold_stress();
        test_sphere_plane_ccd();test_ccd_replay_and_scope();
        std::cout<<"PASS "<<passed<<" assertions across 15 suites\n";
    } catch(const std::exception& e) {
        std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;
    }
}
