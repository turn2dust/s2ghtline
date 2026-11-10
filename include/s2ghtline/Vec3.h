#pragma once

#include <cmath>

// Vec3.h — the library's 3D vector (game units, Source axes: z up).
namespace s2ghtline {

    struct Vec3 {
        float x = 0.f, y = 0.f, z = 0.f;

        Vec3 operator+(const Vec3& o) const { return { x + o.x, y + o.y, z + o.z }; }
        Vec3 operator-(const Vec3& o) const { return { x - o.x, y - o.y, z - o.z }; }
        Vec3 operator*(float s)       const { return { x * s, y * s, z * s }; }

        float dot(const Vec3& o)   const { return x * o.x + y * o.y + z * o.z; }
        Vec3  cross(const Vec3& o) const { return { y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x }; }
        float length()             const { return std::sqrt(dot(*this)); }
    };
    // Parsed vertex blobs are copied straight into Vec3 arrays (three packed floats).
    static_assert(sizeof(Vec3) == 3 * sizeof(float));

} // namespace s2ghtline
