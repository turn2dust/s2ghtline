// test_bvh: Bvh against brute force (every triangle, the same triangle
// test), the triangle test against an independent double-precision one,
// watertightness on shared edges / vertices, and layer masks - all on
// generated geometry.  test_bvh_maps does the same on real maps.
#include "support/BvhHelpers.h"
#include "support/Check.h"

#include <cmath>
#include <cstdio>
#include <random>

using namespace bvh_helpers;
using check::expect;

namespace {

    // ── Independent reference: Moller-Trumbore in double ────
    std::optional<double> mollerTrumbore(Vec3 a, Vec3 b, Vec3 v0, Vec3 v1, Vec3 v2) {
        const double o[3] = { a.x, a.y, a.z }, d[3] = { b.x - a.x, b.y - a.y, b.z - a.z };
        const double e1[3] = { v1.x - v0.x, v1.y - v0.y, v1.z - v0.z }, e2[3] = { v2.x - v0.x, v2.y - v0.y, v2.z - v0.z };
        const double p[3] = { d[1] * e2[2] - d[2] * e2[1], d[2] * e2[0] - d[0] * e2[2], d[0] * e2[1] - d[1] * e2[0] };
        const double det = e1[0] * p[0] + e1[1] * p[1] + e1[2] * p[2];
        if (std::fabs(det) < 1e-12) return std::nullopt;
        const double s[3] = { o[0] - v0.x, o[1] - v0.y, o[2] - v0.z };
        const double u = (s[0] * p[0] + s[1] * p[1] + s[2] * p[2]) / det;
        if (u < 0 || u > 1) return std::nullopt;
        const double q[3] = { s[1] * e1[2] - s[2] * e1[1], s[2] * e1[0] - s[0] * e1[2], s[0] * e1[1] - s[1] * e1[0] };
        const double v = (d[0] * q[0] + d[1] * q[1] + d[2] * q[2]) / det;
        if (v < 0 || u + v > 1) return std::nullopt;
        const double t = (e2[0] * q[0] + e2[1] * q[1] + e2[2] * q[2]) / det;
        if (t < 0 || t > 1) return std::nullopt;
        return t;
    }

    // Random triangle soup: small ones everywhere, a few large, layers 0-3.
    Bvh randomSoup(std::mt19937& rng, int count) {
        std::uniform_real_distribution<float> pos(-1000.f, 1000.f), off(-40.f, 40.f), big(-600.f, 600.f);
        std::vector<Vec3> v;
        std::vector<Tri> tris;
        for (int i = 0; i < count; ++i) {
            const Vec3 c{ pos(rng), pos(rng), pos(rng) };
            const bool large = i % 100 == 0;
            for (int k = 0; k < 3; ++k)
                v.push_back(large ? Vec3{ c.x + big(rng), c.y + big(rng), c.z + big(rng) }
                                  : Vec3{ c.x + off(rng), c.y + off(rng), c.z + off(rng) });
            const auto base = static_cast<uint32_t>(v.size() - 3);
            tris.push_back({ base, base + 1, base + 2, 0, static_cast<uint16_t>(i % 4) });
        }
        return Bvh::build(std::move(v), std::move(tris));
    }

    // A flat grid of n x n unit-64 quads at z = 0, each split along a diagonal.
    Bvh grid(int n) {
        std::vector<Vec3> v;
        std::vector<Tri> tris;
        for (int y = 0; y <= n; ++y)
            for (int x = 0; x <= n; ++x)
                v.push_back({ static_cast<float>(x) * 64.f, static_cast<float>(y) * 64.f, 0.f });
        const auto at = [&](int x, int y) { return static_cast<uint32_t>(y * (n + 1) + x); };
        for (int y = 0; y < n; ++y)
            for (int x = 0; x < n; ++x) {
                tris.push_back({ at(x, y), at(x + 1, y), at(x + 1, y + 1), 0, 0 });
                tris.push_back({ at(x, y), at(x + 1, y + 1), at(x, y + 1), 0, 0 });
            }
        return Bvh::build(std::move(v), std::move(tris));
    }

    void testSynthetic() {
        check::section("synthetic");
        std::mt19937 rng(7);
        const Bvh soup = randomSoup(rng, 20000);
        std::printf("  soup: %zu tris, %zu nodes\n", soup.triangleCount(), soup.nodeCount());
        const auto segs = randomSegments(rng, soup.bounds(), 3000, 1500.f);
        expect(compare(soup, segs, s2ghtline::kAllLayers) == 0, "all layers: closest / occluded / traceAll = brute force");
        const uint64_t mask = s2ghtline::layerBit(0) | s2ghtline::layerBit(2);
        expect(compare(soup, segs, mask) == 0, "layers 0+2: = brute force (masked layers ignored)");
        expect(compare(soup, segs, 0) == 0, "empty mask: no hits");

        // The triangle test vs an independent double Moller-Trumbore (away from edges they must agree).
        int disagree = 0, checked = 0;
        for (const auto& [a, b] : segs) {
            for (uint32_t i = 0; i < 2000; ++i) {
                const Tri& t = soup.tri(i);
                const auto ref = mollerTrumbore(a, b, soup.vertex(t.a), soup.vertex(t.b), soup.vertex(t.c));
                const auto got = soup.hit(a, b, i);
                ++checked;
                if (ref.has_value() != got.has_value() || (ref && std::fabs(*ref - *got) > 1e-4))
                    ++disagree;
            }
        }
        std::printf("  triangle test vs double Moller-Trumbore: %d / %d disagree\n", disagree, checked);
        expect(disagree <= checked / 100000, "triangle test agrees with the double reference");

        const Bvh empty = Bvh::build({}, {});
        expect(!empty.occluded({ 0, 0, 0 }, { 1, 1, 1 }, s2ghtline::kAllLayers) && !empty.closest({}, { 1, 1, 1 }, 1),
               "empty BVH: no hits");
        expect(!soup.closest({ 5, 5, 5 }, { 5, 5, 5 }, s2ghtline::kAllLayers), "zero-length segment: no hit");
    }

    void testWatertight() {
        check::section("watertight");
        const int n = 64;
        const Bvh g = grid(n);
        std::mt19937 rng(3);
        std::uniform_int_distribution<int> cell(1, n - 1);
        std::uniform_real_distribution<float> frac(0.f, 1.f), tilt(-300.f, 300.f);
        int misses = 0, rays = 0;
        for (int i = 0; i < 200000; ++i) {
            // A point exactly on a grid vertex, an axis edge or a diagonal.
            const float x = static_cast<float>(cell(rng)) * 64.f, y = static_cast<float>(cell(rng)) * 64.f, f = frac(rng) * 64.f;
            Vec3 p;
            switch (i % 4) {
            case 0: p = { x, y, 0.f }; break;           // vertex (6 triangles meet)
            case 1: p = { x + f, y, 0.f }; break;       // horizontal edge
            case 2: p = { x, y + f, 0.f }; break;       // vertical edge
            case 3: p = { x + f, y + f, 0.f }; break;   // diagonal edge
            }
            const Vec3 a{ p.x + tilt(rng), p.y + tilt(rng), 200.f };
            const Vec3 b = p + (p - a);   // through p, as far below
            ++rays;
            if (!g.occluded(a, b, s2ghtline::kAllLayers))
                ++misses;
        }
        std::printf("  %d segments through shared vertices / edges, %d leaked\n", rays, misses);
        expect(misses == 0, "no segment leaks through a shared edge or vertex");
    }

} // namespace

int main() {
    testSynthetic();
    testWatertight();
    return check::result();
}
