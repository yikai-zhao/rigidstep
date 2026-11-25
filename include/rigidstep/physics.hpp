#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace rigidstep {

struct Vec3 {
    double x{}, y{}, z{};
    constexpr Vec3() = default;
    constexpr Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    [[nodiscard]] constexpr Vec3 operator+(Vec3 b) const { return {x+b.x,y+b.y,z+b.z}; }
    [[nodiscard]] constexpr Vec3 operator-(Vec3 b) const { return {x-b.x,y-b.y,z-b.z}; }
    [[nodiscard]] constexpr Vec3 operator-() const { return {-x,-y,-z}; }
    [[nodiscard]] constexpr Vec3 operator*(double s) const { return {x*s,y*s,z*s}; }
    [[nodiscard]] constexpr Vec3 operator/(double s) const { return {x/s,y/s,z/s}; }
    Vec3& operator+=(Vec3 b) { x+=b.x;y+=b.y;z+=b.z;return *this; }
    Vec3& operator-=(Vec3 b) { x-=b.x;y-=b.y;z-=b.z;return *this; }
    Vec3& operator*=(double s) { x*=s;y*=s;z*=s;return *this; }
};
[[nodiscard]] inline constexpr Vec3 operator*(double s, Vec3 v) { return v*s; }
[[nodiscard]] inline constexpr double dot(Vec3 a, Vec3 b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
[[nodiscard]] inline constexpr Vec3 cross(Vec3 a, Vec3 b) {
    return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};
}
[[nodiscard]] double norm(Vec3 v);
[[nodiscard]] Vec3 normalized(Vec3 v, Vec3 fallback={1,0,0});

struct Quat {
    double w{1}, x{}, y{}, z{};
    [[nodiscard]] Quat normalized() const;
    [[nodiscard]] Quat conjugate() const { return {w,-x,-y,-z}; }
    [[nodiscard]] Quat operator*(Quat q) const;
    [[nodiscard]] Vec3 rotate(Vec3 v) const;
    [[nodiscard]] Vec3 inverse_rotate(Vec3 v) const { return conjugate().rotate(v); }
    [[nodiscard]] static Quat from_axis_angle(Vec3 axis, double radians);
};

enum class Shape { Sphere, Box, Plane };
enum class Broadphase { SweepAndPrune, BruteForce };

struct Body {
    std::uint32_t id{};
    Shape shape{Shape::Sphere};
    Vec3 position{}, velocity{}, angular_velocity{};
    Quat rotation{};
    Vec3 half_extents{0.5,0.5,0.5};
    double radius{0.5}, inv_mass{0}, restitution{0.1}, friction{0.55};
    Vec3 inv_inertia_local{};
    // A plane represents the infinite horizontal half-space y >= position.y.
};

struct Aabb { Vec3 min{}, max{}; };
struct Pair {
    std::uint32_t a{}, b{};
    [[nodiscard]] bool operator==(const Pair&) const = default;
    [[nodiscard]] bool operator<(const Pair& rhs) const {
        return (a < rhs.a) || (a == rhs.a && b < rhs.b);
    }
};

[[nodiscard]] Aabb bounds(const Body& b);
[[nodiscard]] std::vector<Pair> broadphase_pairs(const std::vector<Body>& bodies, Broadphase mode);

// Geometric primitive: shortest segment pair, including degeneracy/parallel cases.
struct SegmentWitness { Vec3 on_a{}, on_b{}; };
[[nodiscard]] SegmentWitness closest_segment_points(Vec3 a0, Vec3 a1, Vec3 b0, Vec3 b1);

enum class ContactFeature { SphereSphere, SphereBox, BoxFace, BoxEdge, Ground };
struct ContactDebug {
    std::uint32_t a{}, b{};
    Vec3 normal{}, point{};
    double penetration{};
    ContactFeature feature{ContactFeature::SphereSphere};
};

struct Settings {
    double dt{1.0/120.0};
    Vec3 gravity{0,-9.81,0};
    std::size_t solver_iterations{16};
    Broadphase broadphase{Broadphase::SweepAndPrune};
    bool sphere_plane_ccd{false}; // Optional, limited TOI sweep; not general CCD.
    bool warm_starting{false}; // Match persistent contact anchors and reuse prior impulses.
    bool precise_edge_contacts{false}; // Experimental SAT edge closest points; default legacy fallback.
};

struct StepStats {
    std::size_t candidates{}, contacts{};
    double max_penetration{}, kinetic_energy{};
    std::uint64_t state_hash{};
    std::size_t matched_contacts{}, warm_started_contacts{}, cached_contacts{};
    std::size_t edge_contacts{}, face_contacts{};
};

class World {
public:
    explicit World(Settings settings={});
    std::uint32_t add_sphere(Vec3 position, double radius, double mass,
                             Vec3 velocity={}, double restitution=0.1);
    std::uint32_t add_box(Vec3 position, Vec3 half_extents, double mass,
                          Quat rotation={}, Vec3 velocity={}, double restitution=0.1);
    std::uint32_t add_ground(double height=0.0);
    [[nodiscard]] const std::vector<Body>& bodies() const { return bodies_; }
    [[nodiscard]] std::vector<Body>& bodies() { return bodies_; }
    [[nodiscard]] Settings settings() const { return settings_; }
    // Call after directly editing the mutable body vector or teleports.
    void clear_contact_cache() { contact_cache_.clear(); }
    [[nodiscard]] std::size_t cached_contact_count() const { return contact_cache_.size(); }
    // Read-only narrowphase diagnostics for interactive visualization and geometry tests.
    [[nodiscard]] std::vector<ContactDebug> collision_contacts() const;
    [[nodiscard]] StepStats step();
    [[nodiscard]] std::uint64_t state_hash() const;
private:
    Settings settings_;
    std::vector<Body> bodies_;
    struct CachedContact {
        std::uint32_t a{}, b{};
        Vec3 anchor_a{}, anchor_b{}, normal{}, tangent_impulse{};
        double normal_impulse{};
    };
    std::vector<CachedContact> contact_cache_;
};

[[nodiscard]] World make_scene(const std::string& name, std::size_t count,
                               std::uint32_t seed, Settings settings={});
} // namespace rigidstep
