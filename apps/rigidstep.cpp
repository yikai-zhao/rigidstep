#include "rigidstep/physics.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

using namespace rigidstep;
namespace {
struct Options {
    std::string command="demo", scene="rain", broadphase="sap", out="frames.jsonl";
    std::size_t bodies=64, frames=360, repeats=5;
    std::uint32_t seed=11;
    bool sphere_plane_ccd=false, warm_starting=false, precise_edge_contacts=false;
    bool contact_overlay=false;
    std::size_t iterations=16;
};
Options parse(int argc,char** argv) {
    Options o;
    if(argc>1) o.command=argv[1];
    for(int i=2;i<argc;++i) {
        const std::string k=argv[i];
        if(i+1>=argc) throw std::invalid_argument("missing value for "+k);
        const std::string v=argv[++i];
        if(k=="--scene") o.scene=v;
        else if(k=="--broadphase") o.broadphase=v;
        else if(k=="--out") o.out=v;
        else if(k=="--bodies") o.bodies=std::stoul(v);
        else if(k=="--frames") o.frames=std::stoul(v);
        else if(k=="--repeats") o.repeats=std::stoul(v);
        else if(k=="--seed") o.seed=std::stoul(v);
        else if(k=="--iterations") o.iterations=std::stoul(v);
        else if(k=="--contact-overlay") {
            if(v!="on" && v!="off") throw std::invalid_argument("--contact-overlay on|off");
            o.contact_overlay=(v=="on");
        }
        else if(k=="--edge-contact") {
            if(v!="on" && v!="off") throw std::invalid_argument("--edge-contact on|off");
            o.precise_edge_contacts=(v=="on");
        }
        else if(k=="--warm-start") {
            if(v!="on" && v!="off") throw std::invalid_argument("--warm-start on|off");
            o.warm_starting=(v=="on");
        }
        else if(k=="--sphere-plane-ccd") {
            if(v!="on" && v!="off") throw std::invalid_argument("--sphere-plane-ccd on|off");
            o.sphere_plane_ccd=(v=="on");
        }
        else throw std::invalid_argument("unknown option: "+k);
    }
    if(o.frames<1 || o.frames>200000 || o.repeats<1 || o.repeats>1000 || o.iterations<1 || o.iterations>100)
        throw std::invalid_argument("frames/repeats out of allowed range");
    if(o.broadphase!="sap" && o.broadphase!="brute")
        throw std::invalid_argument("--broadphase must be sap or brute");
    return o;
}
Settings options_settings(const Options& o) {
    Settings s; s.sphere_plane_ccd=o.sphere_plane_ccd;
    s.warm_starting=o.warm_starting;s.precise_edge_contacts=o.precise_edge_contacts;s.solver_iterations=o.iterations; s.broadphase=o.broadphase=="sap"?Broadphase::SweepAndPrune:Broadphase::BruteForce;
    return s;
}
void vec(std::ostream& out, Vec3 v) {out<<'['<<v.x<<','<<v.y<<','<<v.z<<']';}
void quat(std::ostream& out,Quat q) {out<<'['<<q.w<<','<<q.x<<','<<q.y<<','<<q.z<<']';}
const char* feature_name(ContactFeature f) {
    switch(f) {
      case ContactFeature::SphereSphere:return "sphere-sphere";
      case ContactFeature::SphereBox:return "sphere-box";
      case ContactFeature::BoxFace:return "box-face";
      case ContactFeature::BoxEdge:return "box-edge";
      case ContactFeature::Ground:return "ground";
    }
    return "unknown";
}
void emit_frame(std::ostream& f,std::size_t tick,const World& w,const StepStats& s,
                bool contact_overlay) {
    f<<"{\"tick\":"<<tick<<",\"dt\":"<<w.settings().dt
     <<",\"hash\":\""<<std::hex<<s.state_hash<<std::dec<<"\",\"candidates\":"<<s.candidates
     <<",\"contacts\":"<<s.contacts<<",\"edge_contacts\":"<<s.edge_contacts
     <<",\"face_contacts\":"<<s.face_contacts<<",\"warm_matched\":"<<s.matched_contacts
     <<",\"warm_applied\":"<<s.warm_started_contacts<<",\"cached_contacts\":"<<s.cached_contacts<<",\"max_penetration\":"<<s.max_penetration<<",\"energy\":"<<s.kinetic_energy<<",\"bodies\":[";
    bool first=true;
    for(const auto& b:w.bodies()) {
        if(b.shape==Shape::Plane) continue;
        if(!first) f<<',';
        first=false;
        f<<"{\"id\":"<<b.id<<",\"type\":\""<<(b.shape==Shape::Sphere?"sphere":"box")
         <<"\",\"static\":"<<(b.inv_mass==0?"true":"false")<<",\"p\":";
        vec(f,b.position);f<<",\"q\":";quat(f,b.rotation);
        f<<",\"v\":";vec(f,b.velocity);
        if(b.shape==Shape::Sphere) f<<",\"radius\":"<<b.radius;
        else {f<<",\"half\":";vec(f,b.half_extents);}
        f<<'}';
    }
    f<<']';
    if(contact_overlay) {
        // These geometry probes are evaluated at the *serialized end-of-step*
        // configuration. They are deliberately distinct from pre-solve StepStats.
        f<<",\"contact_snapshot\":[";
        bool first_contact=true;
        for(const auto& c:w.collision_contacts()) {
            if(!first_contact) f<<',';
            first_contact=false;
            f<<"{\"a\":"<<c.a<<",\"b\":"<<c.b
             <<",\"feature\":\""<<feature_name(c.feature)<<"\",\"p\":";
            vec(f,c.point);f<<",\"n\":";vec(f,c.normal);
            f<<",\"depth\":"<<c.penetration<<'}';
        }
        f<<']';
    }
    f<<"}\n";
}
double percentile(std::vector<double> data,double fraction) {
    std::sort(data.begin(),data.end());
    const double location=(data.size()-1)*fraction;
    const auto low=std::size_t(location);
    const double t=location-double(low);
    if(low+1<data.size()) return data[low]*(1-t)+data[low+1]*t;
    return data.back();
}
void demo(const Options& o) {
    World w=make_scene(o.scene,o.bodies,o.seed,options_settings(o));
    std::ofstream f(o.out);
    if(!f) throw std::runtime_error("cannot create output: "+o.out);
    f<<std::setprecision(17);
    emit_frame(f,0,w,StepStats{0,0,0,0,w.state_hash()},o.contact_overlay);
    std::size_t max_contacts=0,total_warm_applied=0;
    for(std::size_t tick=1;tick<=o.frames;++tick) {
        const StepStats s=w.step();
        max_contacts=std::max(max_contacts,s.contacts);
        total_warm_applied+=s.warm_started_contacts;
        emit_frame(f,tick,w,s,o.contact_overlay);
    }
    std::cout<<"scene="<<o.scene<<" bodies="<<o.bodies<<" ticks="<<o.frames
             <<" dt="<<w.settings().dt<<" max_contacts="<<max_contacts
             <<" warm_applied_total="<<total_warm_applied<<" final_state_hash="
             <<std::hex<<w.state_hash()<<std::dec<<" frames="<<o.out<<'\n';
}
void benchmark(const Options& o) {
    std::ofstream f(o.out);
    if(!f) throw std::runtime_error("cannot create output: "+o.out);
    f<<"mode,scene,bodies,seed,repeat,frames,p50_step_us,p95_step_us,p99_step_us,mean_step_us,final_hash\n";
    f<<std::fixed<<std::setprecision(3);
    for(std::size_t repetition=0;repetition<o.repeats;++repetition) {
        // Alternate order to reduce systematic drift from heat/frequency effects.
        const std::vector<std::string> modes = (repetition%2==0) ?
            std::vector<std::string>{"brute","sap"} : std::vector<std::string>{"sap","brute"};
        for(const auto& mode:modes) {
            Options current=o; current.broadphase=mode;
            World w=make_scene(o.scene,o.bodies,o.seed,options_settings(current));
            std::vector<double> costs;costs.reserve(o.frames);
            // Each mode starts from the same seeded state; timing includes all step work.
            for(std::size_t t=0;t<o.frames;++t) {
                const auto begin=std::chrono::steady_clock::now();
                const auto stats=w.step();
                const auto end=std::chrono::steady_clock::now();
                (void)stats;
                costs.push_back(std::chrono::duration<double,std::micro>(end-begin).count());
            }
            const double mean=std::accumulate(costs.begin(),costs.end(),0.0)/costs.size();
            f<<mode<<','<<o.scene<<','<<o.bodies<<','<<o.seed<<','<<repetition<<','<<o.frames<<','
             <<percentile(costs,0.5)<<','<<percentile(costs,0.95)<<','<<percentile(costs,0.99)<<','
             <<mean<<','<<std::hex<<w.state_hash()<<std::dec<<'\n';
            std::cout<<mode<<" rep="<<repetition<<" mean_step_us="<<mean<<" p99_step_us="
                     <<percentile(costs,0.99)<<" hash="<<std::hex<<w.state_hash()<<std::dec<<'\n';
        }
    }
    std::cout<<"wrote="<<o.out<<" (same seed for both algorithms; do not interpret as general hardware performance)\n";
}
}
int main(int argc,char** argv) {
    try {
        Options o=parse(argc,argv);
        if(o.command=="demo") demo(o);
        else if(o.command=="benchmark") benchmark(o);
        else if(o.command=="--help" || o.command=="help") {
            std::cout<<"rigidstep demo|benchmark [--scene rain|stack|sparse] [--bodies N] "
                        "[--frames N] [--seed N] [--broadphase sap|brute] [--sphere-plane-ccd on|off] [--warm-start on|off] [--edge-contact on|off] [--contact-overlay on|off] [--iterations N] [--repeats N] [--out PATH]\n";
        } else throw std::invalid_argument("unknown command: "+o.command);
    } catch(const std::exception& e) {
        std::cerr<<"error: "<<e.what()<<'\n';return 2;
    }
}
