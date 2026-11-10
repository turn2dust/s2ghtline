#pragma once

// BvhHelpers.h — shared by test_bvh and test_bvh_maps: brute-force references
// (every triangle, the same triangle test) and random segments.
#include "Bvh.h"

#include <chrono>
#include <cstdint>
#include <cmath>
#include <optional>
#include <random>
#include <utility>
#include <vector>

namespace bvh_helpers {

    using s2ghtline::Bvh;
    using s2ghtline::Hit;
    using s2ghtline::Tri;
    using s2ghtline::Vec3;
    using Clock = std::chrono::steady_clock;

    inline double msSince(Clock::time_point t) { return std::chrono::duration<double, std::milli>(Clock::now() - t).count(); }

    // ── Brute force: every triangle, Bvh::hit ────────────────
    inline std::optional<Hit> bruteClosest(const Bvh& bvh, Vec3 a, Vec3 b, uint64_t mask) {
        std::optional<Hit> best;
        for (uint32_t i = 0; i < bvh.triangleCount(); ++i) {
            if (!(mask & s2ghtline::layerBit(bvh.tri(i).layer)))
                continue;
            if (const auto t = bvh.hit(a, b, i); t && (!best || *t < best->t))
                best = Hit{ *t, i };
        }
        return best;
    }
    inline size_t bruteCount(const Bvh& bvh, Vec3 a, Vec3 b, uint64_t mask) {
        size_t n = 0;
        for (uint32_t i = 0; i < bvh.triangleCount(); ++i)
            if ((mask & s2ghtline::layerBit(bvh.tri(i).layer)) && bvh.hit(a, b, i))
                ++n;
        return n;
    }

    // Compares every query with brute force on `segments`; returns mismatches.
    inline int compare(const Bvh& bvh, const std::vector<std::pair<Vec3, Vec3>>& segments, uint64_t mask) {
        int bad = 0;
        std::vector<Hit> all;
        for (const auto& [a, b] : segments) {
            const auto ref  = bruteClosest(bvh, a, b, mask);
            const auto got  = bvh.closest(a, b, mask);
            const bool occl = bvh.occluded(a, b, mask);
            bvh.traceAll(a, b, mask, all);
            // Closest: same t (two triangles can tie at a shared edge, so compare t, not the index).
            const bool closestOk = ref.has_value() == got.has_value() && (!ref || ref->t == got->t);
            if (!closestOk || occl != ref.has_value() || all.size() != bruteCount(bvh, a, b, mask))
                ++bad;
        }
        return bad;
    }

    inline std::vector<std::pair<Vec3, Vec3>> randomSegments(std::mt19937& rng, const s2ghtline::Aabb& box, int n,
                                                             float maxLength) {
        std::uniform_real_distribution<float> ux(box.min.x, box.max.x), uy(box.min.y, box.max.y),
            uz(box.min.z, box.max.z), unit(-1.f, 1.f), len(0.f, maxLength);
        std::vector<std::pair<Vec3, Vec3>> out;
        for (int i = 0; i < n; ++i) {
            const Vec3 a{ ux(rng), uy(rng), uz(rng) };
            Vec3 d{ unit(rng), unit(rng), unit(rng) };
            if (i % 10 == 0) d = { 0.f, 0.f, -1.f };   // axis-aligned too (zero direction components)
            const float l = d.length();
            out.push_back({ a, a + d * (len(rng) / (l > 0 ? l : 1.f)) });
        }
        return out;
    }

} // namespace bvh_helpers
