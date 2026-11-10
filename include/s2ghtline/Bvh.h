#pragma once

#include "PhysParser.h"
#include "Vec3.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <optional>
#include <stop_token>
#include <utility>
#include <vector>

// Bvh.h — bounding volume hierarchy over a map's collision triangles,
// for segment queries: any hit (line of sight), the first hit, every hit.
//
// Memory (maps reach 5M triangles): triangles stay indices into the shared
// vertex array and are reordered into leaf order, so a leaf is a contiguous
// run — no extra index array.  Nodes are 32 bytes, children stored as a pair.
// Built top-down with binned SAH (16 bins per axis), leaves of 1-8 triangles.
//
// Queries take a segment a → b (t = 0..1 along it) and a layer mask (one bit
// per collision layer, layerBit()); triangles of layers not in the mask are
// ignored.  The ray-triangle test is watertight (Woop, Benthin, Wald 2013): a
// segment through an edge or vertex shared by two triangles always hits one
// of them — the world is built of hulls touching each other, and a gap there
// would let a line of sight leak through a wall.  Triangles are two-sided.
namespace s2ghtline {

    // A query's layer mask: bit i = layer i (layers past 63 share bit 63).
    [[nodiscard]] constexpr uint64_t layerBit(uint16_t layer) { return 1ull << (layer < 63 ? layer : 63); }
    constexpr uint64_t kAllLayers = ~0ull;

    struct Aabb {
        Vec3 min{  std::numeric_limits<float>::max(),  std::numeric_limits<float>::max(),
                   std::numeric_limits<float>::max() };
        Vec3 max{ -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(),
                  -std::numeric_limits<float>::max() };

        void grow(const Vec3& p) {
            min = { std::min(min.x, p.x), std::min(min.y, p.y), std::min(min.z, p.z) };
            max = { std::max(max.x, p.x), std::max(max.y, p.y), std::max(max.z, p.z) };
        }
        void grow(const Aabb& b) { grow(b.min); grow(b.max); }

        [[nodiscard]] bool empty() const { return min.x > max.x; }
        [[nodiscard]] float area() const {
            if (empty())
                return 0.f;
            const Vec3 e = max - min;
            return 2.f * (e.x * e.y + e.y * e.z + e.z * e.x);
        }
    };

    // A segment hit: t along a → b (0..1) and the triangle (index into Bvh::tri()).
    struct Hit {
        float    t   = 0.f;
        uint32_t tri = 0;
    };

    class Bvh {
    public:
        Bvh() = default;

        // Takes the geometry's vertices and triangles; reorders the triangles
        // into leaf order.  A stop request (the map changed mid-build) ends it
        // early with an empty BVH.
        [[nodiscard]] static Bvh build(std::vector<Vec3> vertices, std::vector<Tri> tris,
                                       std::stop_token stop = {});

        [[nodiscard]] bool empty() const { return m_nodes.empty(); }

        // Every triangle whose box overlaps `box` (f(triangle index)).
        template <class F>
        void forEachInBox(const Aabb& box, F&& f) const;

        // Anything of the masked layers between a and b?
        [[nodiscard]] bool occluded(const Vec3& a, const Vec3& b, uint64_t mask) const {
            return blocker(a, b, mask).has_value();
        }
        // A triangle of the masked layers between a and b (the first one
        // found, not necessarily the nearest), if any.
        [[nodiscard]] std::optional<uint32_t> blocker(const Vec3& a, const Vec3& b, uint64_t mask) const;
        // The first surface from a toward b.
        [[nodiscard]] std::optional<Hit> closest(const Vec3& a, const Vec3& b, uint64_t mask) const;
        // Every surface between a and b, nearest first (a segment through a
        // shared edge can report both triangles at the same t).
        void traceAll(const Vec3& a, const Vec3& b, uint64_t mask, std::vector<Hit>& out) const;
        // Where the segment hits one triangle (the same test the queries use).
        [[nodiscard]] std::optional<float> hit(const Vec3& a, const Vec3& b, uint32_t tri) const {
            return intersect(Ray(a, b), tri, 1.f);
        }

        [[nodiscard]] const Tri&  tri(uint32_t i)    const { return m_tris[i]; }
        [[nodiscard]] const Vec3& vertex(uint32_t i) const { return m_vertices[i]; }
        [[nodiscard]] size_t triangleCount() const { return m_tris.size(); }
        [[nodiscard]] size_t nodeCount()     const { return m_nodes.size(); }
        [[nodiscard]] Aabb   bounds()        const { return m_nodes.empty() ? Aabb{} : Aabb{ m_nodes[0].min, m_nodes[0].max }; }
        [[nodiscard]] size_t memoryBytes()   const {
            return m_nodes.size() * sizeof(Node) + m_tris.size() * sizeof(Tri) + m_vertices.size() * sizeof(Vec3);
        }

    private:
        // count == 0: interior node, children at first and first + 1.
        // count  > 0: leaf, triangles [first, first + count).
        struct Node {
            Vec3     min;
            uint32_t first = 0;
            Vec3     max;
            uint32_t count = 0;
        };
        static_assert(sizeof(Node) == 32);

        static constexpr uint32_t kMaxLeaf   = 8;
        static constexpr int      kBins      = 16;
        static constexpr int      kMaxDepth  = 96;
        static constexpr int      kStackSize = kMaxDepth + 4;

        std::vector<Node> m_nodes;
        std::vector<Tri>  m_tris;
        std::vector<Vec3> m_vertices;

        class Builder;

        // A segment prepared for the slab and watertight triangle tests.
        struct Ray {
            Vec3  origin, dir, invDir;
            int   kx = 0, ky = 1, kz = 2;    // axes: kz = the dominant direction axis
            float sx = 0.f, sy = 0.f, sz = 0.f;

            Ray(const Vec3& a, const Vec3& b);
        };

        [[nodiscard]] static float axis(const Vec3& v, int i) { return i == 0 ? v.x : i == 1 ? v.y : v.z; }

        // Entry t of the segment into a node's box, or +inf if it misses within [0, tMax].
        [[nodiscard]] static float enter(const Node& n, const Ray& r, float tMax);
        // t of the hit on triangle i within [0, tMax], if any.
        [[nodiscard]] std::optional<float> intersect(const Ray& r, uint32_t i, float tMax) const;

        // Visit every leaf the segment's box test reaches, nearest child first;
        // `leaf(node)` returns the new tMax (or a negative value to stop).
        template <class LeafFn>
        void traverse(const Ray& r, float tMax, LeafFn&& leaf) const;
    };

    // ── Build ────────────────────────────────────────────────
    class Bvh::Builder {
    public:
        Builder(const std::vector<Vec3>& vertices, const std::vector<Tri>& tris, std::vector<Node>& nodes,
                std::stop_token stop)
            : m_vertices(vertices), m_tris(tris), m_nodes(nodes), m_stop(std::move(stop)) {}

        // Builds the nodes; returns the triangle order (leaf runs), or nothing if stopped.
        std::optional<std::vector<uint32_t>> run() {
            const auto n = static_cast<uint32_t>(m_tris.size());
            m_order.resize(n);
            std::iota(m_order.begin(), m_order.end(), 0u);
            m_boxes.resize(n);
            for (uint32_t i = 0; i < n; ++i) {
                const Tri& t = m_tris[i];
                m_boxes[i].grow(m_vertices[t.a]);
                m_boxes[i].grow(m_vertices[t.b]);
                m_boxes[i].grow(m_vertices[t.c]);
            }

            m_nodes.reserve(n / 2 + 2);
            m_nodes.push_back(makeNode(0, n));
            struct Pending { uint32_t node; int depth; };
            std::vector<Pending> stack{ { 0, 0 } };
            for (uint32_t splits = 0; !stack.empty(); ++splits) {
                if (splits % 4096 == 0 && m_stop.stop_requested())
                    return std::nullopt;
                const Pending p = stack.back();
                stack.pop_back();
                if (split(p.node, p.depth)) {
                    const uint32_t left = m_nodes[p.node].first;
                    stack.push_back({ left + 1, p.depth + 1 });
                    stack.push_back({ left, p.depth + 1 });
                }
            }
            return std::move(m_order);
        }

    private:
        const std::vector<Vec3>& m_vertices;
        const std::vector<Tri>&  m_tris;
        std::vector<Node>&       m_nodes;
        std::stop_token          m_stop;
        std::vector<uint32_t>    m_order;   // triangle indices; a node owns a contiguous range
        std::vector<Aabb>        m_boxes;   // per triangle (by original index)

        [[nodiscard]] Vec3 centre(uint32_t tri) const { return (m_boxes[tri].min + m_boxes[tri].max) * 0.5f; }

        Node makeNode(uint32_t first, uint32_t count) const {
            Aabb box;
            for (uint32_t i = first; i < first + count; ++i)
                box.grow(m_boxes[m_order[i]]);
            Node node;
            node.min   = box.min;
            node.max   = box.max;
            node.first = first;
            node.count = count;
            return node;
        }

        // Splits a leaf into two children (true) or keeps it a leaf.
        bool split(uint32_t index, int depth) {
            const uint32_t first = m_nodes[index].first;
            const uint32_t count = m_nodes[index].count;
            if (count <= 1 || depth >= kMaxDepth)
                return false;

            Aabb centres;
            for (uint32_t i = first; i < first + count; ++i)
                centres.grow(centre(m_order[i]));

            // Binned SAH: cost of a split = (area_L * n_L + area_R * n_R) / area + 1 traversal,
            // against n for keeping a leaf.
            const float parentArea = Aabb{ m_nodes[index].min, m_nodes[index].max }.area();
            float bestCost = std::numeric_limits<float>::max();
            int   bestAxis = -1, bestBin = 0;
            for (int a = 0; a < 3; ++a) {
                const float lo = axis(centres.min, a), extent = axis(centres.max, a) - lo;
                if (!(extent > 0.f))
                    continue;
                Aabb     bins[kBins];
                uint32_t counts[kBins] = {};
                const float scale = kBins / extent;
                for (uint32_t i = first; i < first + count; ++i) {
                    const uint32_t t = m_order[i];
                    const int b = std::min(kBins - 1, static_cast<int>((axis(centre(t), a) - lo) * scale));
                    bins[b].grow(m_boxes[t]);
                    ++counts[b];
                }
                // Sweep: areas / counts left of each plane, then right of it.
                float    leftArea[kBins - 1];
                uint32_t leftCount[kBins - 1];
                Aabb box;
                uint32_t sum = 0;
                for (int b = 0; b < kBins - 1; ++b) {
                    box.grow(bins[b]);
                    sum += counts[b];
                    leftArea[b]  = box.area();
                    leftCount[b] = sum;
                }
                box = Aabb{};
                sum = 0;
                for (int b = kBins - 1; b > 0; --b) {
                    box.grow(bins[b]);
                    sum += counts[b];
                    if (leftCount[b - 1] == 0 || sum == 0)
                        continue;
                    const float cost = leftArea[b - 1] * static_cast<float>(leftCount[b - 1]) + box.area() * static_cast<float>(sum);
                    if (cost < bestCost) {
                        bestCost = cost;
                        bestAxis = a;
                        bestBin  = b;
                    }
                }
            }

            uint32_t mid = first;
            if (bestAxis >= 0) {
                const float splitCost = parentArea > 0.f ? bestCost / parentArea + 1.f : 0.f;
                if (splitCost >= static_cast<float>(count) && count <= kMaxLeaf)
                    return false;
                const float lo    = axis(centres.min, bestAxis);
                const float scale = kBins / (axis(centres.max, bestAxis) - lo);
                const auto  it    = std::partition(m_order.begin() + first, m_order.begin() + first + count,
                    [&](uint32_t t) {
                        return std::min(kBins - 1, static_cast<int>((axis(centre(t), bestAxis) - lo) * scale)) < bestBin;
                    });
                mid = static_cast<uint32_t>(it - m_order.begin());
            }
            if (mid == first || mid == first + count) {
                // All centres in one spot (or one bin): split the run in half.
                if (count <= kMaxLeaf)
                    return false;
                mid = first + count / 2;
            }

            const auto left = static_cast<uint32_t>(m_nodes.size());
            m_nodes.push_back(makeNode(first, mid - first));
            m_nodes.push_back(makeNode(mid, first + count - mid));
            m_nodes[index].first = left;
            m_nodes[index].count = 0;
            return true;
        }
    };

    inline Bvh Bvh::build(std::vector<Vec3> vertices, std::vector<Tri> tris, std::stop_token stop) {
        Bvh bvh;
        if (tris.empty())
            return bvh;
        bvh.m_vertices = std::move(vertices);

        const auto order = Builder(bvh.m_vertices, tris, bvh.m_nodes, std::move(stop)).run();
        if (!order)
            return Bvh{};
        bvh.m_tris.reserve(tris.size());
        for (uint32_t i : *order)
            bvh.m_tris.push_back(tris[i]);
        bvh.m_nodes.shrink_to_fit();
        return bvh;
    }

    template <class F>
    void Bvh::forEachInBox(const Aabb& box, F&& f) const {
        if (m_nodes.empty())
            return;
        const auto overlaps = [&](const Node& n) {
            return n.min.x <= box.max.x && n.max.x >= box.min.x && n.min.y <= box.max.y && n.max.y >= box.min.y &&
                   n.min.z <= box.max.z && n.max.z >= box.min.z;
        };
        uint32_t stack[kStackSize * 2];
        int top = 0;
        stack[top++] = 0;
        while (top > 0) {
            const Node& n = m_nodes[stack[--top]];
            if (!overlaps(n))
                continue;
            if (n.count > 0) {
                for (uint32_t i = n.first; i < n.first + n.count; ++i)
                    f(i);
            }
            else {
                stack[top++] = n.first;
                stack[top++] = n.first + 1;
            }
        }
    }

    // ── Queries ──────────────────────────────────────────────
    inline Bvh::Ray::Ray(const Vec3& a, const Vec3& b) : origin(a), dir(b - a) {
        // A zero component would give 0 * inf = NaN in the slab test: use a huge finite inverse.
        const auto inv = [](float d) { return std::fabs(d) > 1e-12f ? 1.f / d : std::copysign(1e30f, d); };
        invDir = { inv(dir.x), inv(dir.y), inv(dir.z) };

        const float ax = std::fabs(dir.x), ay = std::fabs(dir.y), az = std::fabs(dir.z);
        kz = ax > ay ? (ax > az ? 0 : 2) : (ay > az ? 1 : 2);
        kx = (kz + 1) % 3;
        ky = (kx + 1) % 3;
        if (axis(dir, kz) < 0.f)
            std::swap(kx, ky);   // keep the winding (orientation) consistent
        const float dz = axis(dir, kz);
        if (dz != 0.f) {
            sx = axis(dir, kx) / dz;
            sy = axis(dir, ky) / dz;
            sz = 1.f / dz;
        }
    }

    // The slab exit is widened by the rounding error of its own computation
    // (1 + 2 gamma(3), Ize 2013 "Robust BVH Ray Traversal"): without it a
    // segment exactly on the face shared by two boxes can round outside both
    // while the triangle test puts its hit inside one, and the hit is lost.
    inline float Bvh::enter(const Node& n, const Ray& r, float tMax) {
        constexpr float kEps   = std::numeric_limits<float>::epsilon() * 0.5f;
        constexpr float kSlack = 1.f + 2.f * (3.f * kEps / (1.f - 3.f * kEps));
        const float tx1 = (n.min.x - r.origin.x) * r.invDir.x, tx2 = (n.max.x - r.origin.x) * r.invDir.x;
        const float ty1 = (n.min.y - r.origin.y) * r.invDir.y, ty2 = (n.max.y - r.origin.y) * r.invDir.y;
        const float tz1 = (n.min.z - r.origin.z) * r.invDir.z, tz2 = (n.max.z - r.origin.z) * r.invDir.z;
        const float tNear = std::max({ std::min(tx1, tx2), std::min(ty1, ty2), std::min(tz1, tz2), 0.f });
        const float tFar  = std::min({ std::max(tx1, tx2) * kSlack, std::max(ty1, ty2) * kSlack,
                                       std::max(tz1, tz2) * kSlack, tMax });
        return tNear <= tFar ? tNear : std::numeric_limits<float>::infinity();
    }

    inline std::optional<float> Bvh::intersect(const Ray& r, uint32_t i, float tMax) const {
        if (r.sz == 0.f)
            return std::nullopt;   // zero-length segment
        const Tri& t = m_tris[i];
        const Vec3 A = m_vertices[t.a] - r.origin, B = m_vertices[t.b] - r.origin, C = m_vertices[t.c] - r.origin;

        // Shear / scale so the ray is the +z axis; edge functions in 2D.
        const float az = axis(A, r.kz), bz = axis(B, r.kz), cz = axis(C, r.kz);
        const float ax = axis(A, r.kx) - r.sx * az, ay = axis(A, r.ky) - r.sy * az;
        const float bx = axis(B, r.kx) - r.sx * bz, by = axis(B, r.ky) - r.sy * bz;
        const float cx = axis(C, r.kx) - r.sx * cz, cy = axis(C, r.ky) - r.sy * cz;
        float u = cx * by - cy * bx;
        float v = ax * cy - ay * cx;
        float w = bx * ay - by * ax;
        if (u == 0.f || v == 0.f || w == 0.f) {
            // On an edge in float: decide in double so neighbours agree (watertightness).
            u = static_cast<float>(static_cast<double>(cx) * by - static_cast<double>(cy) * bx);
            v = static_cast<float>(static_cast<double>(ax) * cy - static_cast<double>(ay) * cx);
            w = static_cast<float>(static_cast<double>(bx) * ay - static_cast<double>(by) * ax);
        }
        if ((u < 0.f || v < 0.f || w < 0.f) && (u > 0.f || v > 0.f || w > 0.f))
            return std::nullopt;
        const float det = u + v + w;
        if (det == 0.f)
            return std::nullopt;

        // Hit distance, scaled by det (two-sided: det may be negative).
        const float T = r.sz * (u * az + v * bz + w * cz);
        if (det > 0.f ? (T < 0.f || T > tMax * det) : (T > 0.f || T < tMax * det))
            return std::nullopt;
        return T / det;
    }

    template <class LeafFn>
    void Bvh::traverse(const Ray& r, float tMax, LeafFn&& leaf) const {
        constexpr float kMiss = std::numeric_limits<float>::infinity();
        if (m_nodes.empty() || enter(m_nodes[0], r, tMax) == kMiss)
            return;

        // Deferred far children with their entry t: skipped once a nearer hit lowered tMax.
        struct Deferred { uint32_t node; float t; };
        Deferred stack[kStackSize];
        int      top  = 0;
        uint32_t node = 0;
        const auto pop = [&]() {
            while (top > 0) {
                const Deferred d = stack[--top];
                if (d.t <= tMax) {
                    node = d.node;
                    return true;
                }
            }
            return false;
        };

        for (;;) {
            const Node& n = m_nodes[node];
            if (n.count > 0) {
                tMax = leaf(n);
                if (tMax < 0.f || !pop())
                    return;
                continue;
            }
            uint32_t nearChild = n.first, farChild = n.first + 1;
            float    tNear = enter(m_nodes[nearChild], r, tMax), tFar = enter(m_nodes[farChild], r, tMax);
            if (tFar < tNear) {
                std::swap(nearChild, farChild);
                std::swap(tNear, tFar);
            }
            if (tNear == kMiss) {
                if (!pop())
                    return;
                continue;
            }
            if (tFar != kMiss)
                stack[top++] = { farChild, tFar };
            node = nearChild;
        }
    }

    inline std::optional<uint32_t> Bvh::blocker(const Vec3& a, const Vec3& b, uint64_t mask) const {
        const Ray r(a, b);
        std::optional<uint32_t> found;
        traverse(r, 1.f, [&](const Node& n) {
            for (uint32_t i = n.first; i < n.first + n.count; ++i) {
                if ((mask & layerBit(m_tris[i].layer)) && intersect(r, i, 1.f)) {
                    found = i;
                    return -1.f;
                }
            }
            return 1.f;
        });
        return found;
    }

    inline std::optional<Hit> Bvh::closest(const Vec3& a, const Vec3& b, uint64_t mask) const {
        const Ray r(a, b);
        std::optional<Hit> best;
        float tMax = 1.f;
        traverse(r, tMax, [&](const Node& n) {
            for (uint32_t i = n.first; i < n.first + n.count; ++i) {
                if (!(mask & layerBit(m_tris[i].layer)))
                    continue;
                if (const auto t = intersect(r, i, tMax)) {
                    tMax = *t;
                    best = Hit{ *t, i };
                }
            }
            return tMax;
        });
        return best;
    }

    inline void Bvh::traceAll(const Vec3& a, const Vec3& b, uint64_t mask, std::vector<Hit>& out) const {
        out.clear();
        const Ray r(a, b);
        traverse(r, 1.f, [&](const Node& n) {
            for (uint32_t i = n.first; i < n.first + n.count; ++i)
                if (mask & layerBit(m_tris[i].layer))
                    if (const auto t = intersect(r, i, 1.f))
                        out.push_back({ *t, i });
            return 1.f;
        });
        std::sort(out.begin(), out.end(), [](const Hit& x, const Hit& y) { return x.t < y.t; });
    }

} // namespace s2ghtline
