#pragma once

#include "ik/vec.hpp"

#include <cmath>

namespace ik
{
namespace detail
{

inline constexpr float kPi = 3.14159265358979323846f;

template <unsigned D>
inline vec<float, D> default_forward() {
    vec<float, D> v{};
    v[0] = 1.0f;
    return v;
}

template <unsigned D>
inline vec<float, D> safe_normalize(vec<float, D> v, vec<float, D> fallback = default_forward<D>()) {
    float len = length(v);
    if (len <= 1e-8f) {
        return fallback;
    }
    return v / len;
}

// Rodrigues rotation for 3D.
inline vec3 rotate_axis(vec3 v, vec3 axis, float angle_rad) {
    axis = safe_normalize(axis, vec3{0, 0, 1});
    float c = std::cos(angle_rad);
    float s = std::sin(angle_rad);
    return v * c + cross(axis, v) * s + axis * (dot(axis, v) * (1.0f - c));
}

// Build right-handed basis from a forward axis. Prefer world +Z as up.
inline void basis_from_forward(vec3 forward, vec3& right, vec3& up) {
    forward = safe_normalize(forward);
    vec3 world_up{0, 0, 1};
    if (std::abs(dot(forward, world_up)) > 0.999f) {
        world_up = vec3{0, 1, 0};
    }
    right = safe_normalize(cross(world_up, forward), vec3{0, 1, 0});
    up = cross(forward, right);
}

// Map local spherical angles (yaw, pitch) to a world direction.
// local +X aligns with parent_forward; yaw around parent up, pitch around parent right.
inline vec3 direction_from_local(vec3 parent_forward, vec2 local) {
    vec3 right, up;
    basis_from_forward(parent_forward, right, up);
    float yaw = local[0];
    float pitch = local[1];
    float cp = std::cos(pitch);
    float sp = std::sin(pitch);
    float cy = std::cos(yaw);
    float sy = std::sin(yaw);
    // local spherical point on unit sphere
    float lx = cp * cy;
    float ly = cp * sy;
    float lz = sp;
    return safe_normalize(parent_forward * lx + right * ly + up * lz);
}

inline vec2 local_from_direction(vec3 parent_forward, vec3 child_dir) {
    vec3 right, up;
    basis_from_forward(parent_forward, right, up);
    child_dir = safe_normalize(child_dir);
    float lx = dot(child_dir, parent_forward);
    float ly = dot(child_dir, right);
    float lz = dot(child_dir, up);
    float yaw = std::atan2(ly, lx);
    float pitch = std::atan2(lz, std::sqrt(lx * lx + ly * ly));
    return vec2{yaw, pitch};
}

inline vec2 direction_from_local(vec2 parent_forward, vec<float, 1> local) {
    return rotate(safe_normalize(parent_forward), local[0]);
}

inline vec<float, 1> local_from_direction(vec2 parent_forward, vec2 child_dir) {
    parent_forward = safe_normalize(parent_forward);
    child_dir = safe_normalize(child_dir);
    float sin_a = parent_forward[0] * child_dir[1] - parent_forward[1] * child_dir[0];
    float cos_a = dot(parent_forward, child_dir);
    return vec<float, 1>{std::atan2(sin_a, cos_a)};
}

template <unsigned D>
inline vec<float, D> direction_from_local_angles(vec<float, D> parent_forward,
                                                 vec<float, D - 1> local) {
    if constexpr (D == 2) {
        return direction_from_local(parent_forward, local);
    } else if constexpr (D == 3) {
        return direction_from_local(parent_forward, local);
    } else {
        return safe_normalize(parent_forward);
    }
}

template <unsigned D>
inline vec<float, D - 1> local_angles_from_direction(vec<float, D> parent_forward,
                                                     vec<float, D> child_dir) {
    if constexpr (D == 2) {
        return local_from_direction(parent_forward, child_dir);
    } else if constexpr (D == 3) {
        return local_from_direction(parent_forward, child_dir);
    } else {
        return vec<float, D - 1>{};
    }
}

template <unsigned N>
inline vec<float, N> clamp_local_angles(vec<float, N> local,
                                        vec<float, N> min_rot,
                                        vec<float, N> max_rot) {
    vec<float, N> out;
    for (unsigned i = 0; i < N; ++i) {
        out[i] = clamp_angle(wrap_angle(local[i]), min_rot[i], max_rot[i]);
    }
    return out;
}

template <unsigned D>
inline vec<float, D> parent_forward_of(const vec<float, D>& parent_orientation,
                                       const vec<float, D>& parent_position,
                                       const vec<float, D>* grandparent_position) {
    if (length_squared(parent_orientation) > 1e-12f) {
        return safe_normalize(parent_orientation);
    }
    if (grandparent_position) {
        return safe_normalize(parent_position - *grandparent_position);
    }
    return default_forward<D>();
}

} // namespace detail
} // namespace ik
