#pragma once

#include "Bytes.h"
#include "Kv3.h"
#include "Vec3.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

// PhysParser.h — a map's collision (the decoded PHYS block of
// world_physics.vmdl_c) as one triangle list.
//
// The collision is made of shapes, each with ONE surface material and ONE
// collision layer (attribute): convex hulls (half-edge polygons, triangulated
// here as fans), triangle meshes, and spheres / capsules (poles, railings,
// trunks: thousands on some maps), tessellated.  Static props are baked in.
// Each triangle keeps its shape's material and layer, so layers can be
// filtered and materials looked up later.  Hull and mesh triangles come first,
// sphere / capsule triangles last (stats.primitiveTris of them).  Compound
// shapes (none on current maps) are counted, not read.
namespace s2ghtline {

    struct Tri {
        uint32_t a = 0, b = 0, c = 0;   // vertex indices
        uint16_t surface = 0;           // index into Geometry::surfaceHashes (kNoSurface if the shape had none)
        uint16_t layer   = 0;           // index into Geometry::layers
        uint32_t solid   = 0;           // its shape (hull / mesh / sphere / capsule), numbered in parse order
    };

    // A collision layer: what its shapes interact as ("playerclip", "passbullets", ...)
    // and what they exclude ("player").  Empty interactAs = plain world geometry.
    struct Layer {
        std::string              group;
        std::vector<std::string> interactAs;
        std::vector<std::string> exclude;
    };

    struct GeometryStats {
        uint32_t hulls = 0, meshes = 0, spheres = 0, capsules = 0;
        uint32_t compounds = 0;           // compound shapes (groups of shapes): counted, not read
        uint32_t hullTris = 0, meshTris = 0;   // triangles read, before dropping degenerate ones
        uint32_t primitiveTris = 0;       // sphere / capsule triangles kept (the last ones in Geometry::tris)
        uint32_t degenerate = 0;          // zero-area triangles dropped
        uint32_t missingSurface = 0;      // shapes whose surface index was out of range
        uint32_t meshMaterialsIgnored = 0;   // meshes whose per-triangle materials didn't match their triangles
        bool     bindPose = false;        // parts carry transforms (not applied; world physics has none)
    };

    enum class SolidKind : uint8_t { Hull, Mesh, Primitive };
    inline const char* toString(SolidKind k) { return k == SolidKind::Mesh ? "mesh" : k == SolidKind::Primitive ? "primitive" : "hull"; }

    struct Geometry {
        static constexpr uint16_t kNoSurface = 0xFFFF;

        std::vector<Vec3>      vertices;
        std::vector<Tri>       tris;
        std::vector<Layer>     layers;
        std::vector<uint32_t>  surfaceHashes;   // MurmurHash2 of each surface property's name
        std::vector<SolidKind> solidKinds;      // by Tri::solid
        GeometryStats          stats;
    };

    // PhysParser ---------------------------------------------------
    class PhysParser {
    public:
        [[nodiscard]] static std::optional<Geometry> parse(Kv3Value root, std::string& why) {
            PhysParser p(why);
            if (!p.readTables(root) || !p.readParts(root) || !p.readPrimitives(root))
                return std::nullopt;
            if (p.m_geo.tris.empty()) {
                why = "the physics data has no triangles";
                return std::nullopt;
            }
            return std::move(p.m_geo);
        }

    private:
        static constexpr float    kMaxCoord        = 100000.f;   // CS2 maps are within +-16384 (sanity bound)
        static constexpr float    kMinTwiceArea    = 1e-6f;
        static constexpr uint32_t kMaxShapeVertices = 1u << 24;

        struct HalfEdge {
            uint8_t next = 0, twin = 0, origin = 0, face = 0;
        };

        Geometry     m_geo;
        std::string& m_why;
        uint32_t     m_solid = 0;   // the current shape's number (Tri::solid)
        void nextSolid(SolidKind kind) {
            ++m_solid;
            m_geo.solidKinds.resize(m_solid + 1, SolidKind::Hull);
            m_geo.solidKinds[m_solid] = kind;
        }

        explicit PhysParser(std::string& why) : m_why(why) {}

        bool fail(std::string why) {
            m_why = std::move(why);
            return false;
        }

        // ── Surface materials and collision layers ───────────
        bool readTables(Kv3Value root) {
            using namespace kv3_literals;

            const Kv3Value hashes = root["m_surfacePropertyHashes"_kv];
            for (size_t i = 0; i < hashes.size(); ++i)
                m_geo.surfaceHashes.push_back(static_cast<uint32_t>(hashes.at(i).integer().value_or(0)));

            const Kv3Value attributes = root["m_collisionAttributes"_kv];
            if (!attributes.isArray() || attributes.size() == 0 || attributes.size() > 0xFFFF)
                return fail("no collision layers in the physics data");
            for (size_t i = 0; i < attributes.size(); ++i) {
                const Kv3Value a = attributes.at(i);
                Layer layer;
                layer.group = std::string(a["m_CollisionGroupString"_kv].string().value_or(""));
                Kv3Value tags = a["m_InteractAsStrings"_kv];
                if (!tags.isArray())
                    tags = a["m_PhysicsTagStrings"_kv];   // older name
                layer.interactAs = strings(tags);
                layer.exclude    = strings(a["m_InteractExcludeStrings"_kv]);
                m_geo.layers.push_back(std::move(layer));
            }

            m_geo.stats.bindPose = root["m_bindPose"_kv].size() > 0;
            return true;
        }

        static std::vector<std::string> strings(Kv3Value array) {
            std::vector<std::string> out;
            for (size_t i = 0; i < array.size(); ++i)
                if (auto s = array.at(i).string())
                    out.emplace_back(*s);
            return out;
        }

        // ── Shapes ───────────────────────────────────────────
        bool readParts(Kv3Value root) {
            using namespace kv3_literals;

            const Kv3Value parts = root["m_parts"_kv];
            if (parts.size() == 0)
                return fail("no parts in the physics data");

            for (size_t p = 0; p < parts.size(); ++p) {
                const Kv3Value shape = parts.at(p)["m_rnShape"_kv];
                m_geo.stats.compounds += static_cast<uint32_t>(shape["m_compounds"_kv].size());

                const Kv3Value hulls = shape["m_hulls"_kv];
                for (size_t i = 0; i < hulls.size(); ++i)
                    if (!readHull(hulls.at(i)))
                        return false;

                const Kv3Value meshes = shape["m_meshes"_kv];
                for (size_t i = 0; i < meshes.size(); ++i)
                    if (!readMesh(meshes.at(i)))
                        return false;
            }
            return true;
        }

        // Spheres and capsules, after every part's hulls and meshes.
        bool readPrimitives(Kv3Value root) {
            using namespace kv3_literals;
            const Kv3Value parts = root["m_parts"_kv];
            const size_t trisBefore = m_geo.tris.size();
            for (size_t p = 0; p < parts.size(); ++p) {
                const Kv3Value shape = parts.at(p)["m_rnShape"_kv];

                const Kv3Value spheres = shape["m_spheres"_kv];
                for (size_t i = 0; i < spheres.size(); ++i) {
                    const Kv3Value s = spheres.at(i)["m_Sphere"_kv];
                    const auto centre = readVec3(s["m_vCenter"_kv]);
                    if (!addPrimitive(spheres.at(i), centre, centre, s["m_flRadius"_kv]))
                        return false;
                    ++m_geo.stats.spheres;
                }

                const Kv3Value capsules = shape["m_capsules"_kv];
                for (size_t i = 0; i < capsules.size(); ++i) {
                    const Kv3Value c = capsules.at(i)["m_Capsule"_kv];
                    const Kv3Value centres = c["m_vCenter"_kv];
                    if (!addPrimitive(capsules.at(i), readVec3(centres.at(0)), readVec3(centres.at(1)), c["m_flRadius"_kv]))
                        return false;
                    ++m_geo.stats.capsules;
                }
            }
            m_geo.stats.primitiveTris = static_cast<uint32_t>(m_geo.tris.size() - trisBefore);
            return true;
        }

        bool addPrimitive(Kv3Value shape, std::optional<Vec3> c0, std::optional<Vec3> c1, Kv3Value radius) {
            nextSolid(SolidKind::Primitive);
            uint16_t layer = 0, surface = 0;
            if (!shapeTags(shape, layer, surface))
                return false;
            const float r = static_cast<float>(radius.number().value_or(-1.0));
            if (!c0 || !c1 || !(r > 0.f && r <= kMaxCoord) || !inRange(*c0) || !inRange(*c1))
                return fail("a sphere / capsule is unreadable");
            addCapsule(*c0, *c1, r, surface, layer);
            return true;
        }

        // A capsule (a sphere when c0 == c1) as triangles: rings of kSides
        // vertices - the two equators, kCapRings latitudes on each cap - and a
        // fan to each pole.  Vertices lie on the surface (polygon edges cut
        // up to 1 - cos(18 deg) = 5% of the radius inside it).
        void addCapsule(const Vec3& c0, const Vec3& c1, float r, uint16_t surface, uint16_t layer) {
            constexpr int   kSides    = 10;
            constexpr int   kCapRings = 1;
            constexpr float kHalfPi   = 1.57079633f;
            constexpr float kTwoPi    = 6.28318531f;

            // Axis and a perpendicular basis.
            const Vec3  axisVec = c1 - c0;
            const float length  = axisVec.length();
            const Vec3  u = length > 1e-3f ? axisVec * (1.f / length) : Vec3{ 0.f, 0.f, 1.f };
            const Vec3 helper = std::fabs(u.z) < 0.9f ? Vec3{ 0.f, 0.f, 1.f } : Vec3{ 1.f, 0.f, 0.f };
            Vec3 v = helper.cross(u);
            v = v * (1.f / v.length());
            const Vec3 w = u.cross(v);
            const bool sphere = length <= 1e-3f;

            // Rings from the c0 pole to the c1 pole: (centre, radius).
            struct Ring { Vec3 centre; float radius; };
            std::vector<Ring> rings;
            for (int k = kCapRings; k >= 0; --k) {
                const float lat = kHalfPi * static_cast<float>(k) / (kCapRings + 1);
                rings.push_back({ c0 - u * (r * std::sin(lat)), r * std::cos(lat) });
            }
            for (int k = sphere ? 1 : 0; k <= kCapRings; ++k) {   // a sphere shares one equator
                const float lat = kHalfPi * static_cast<float>(k) / (kCapRings + 1);
                rings.push_back({ c1 + u * (r * std::sin(lat)), r * std::cos(lat) });
            }

            const auto pole0 = static_cast<uint32_t>(m_geo.vertices.size());
            m_geo.vertices.push_back(c0 - u * r);
            const uint32_t firstRing = pole0 + 1;
            for (const Ring& ring : rings) {
                for (int s = 0; s < kSides; ++s) {
                    const float a = kTwoPi * static_cast<float>(s) / kSides;
                    m_geo.vertices.push_back(ring.centre + v * (ring.radius * std::cos(a)) + w * (ring.radius * std::sin(a)));
                }
            }
            const auto pole1 = static_cast<uint32_t>(m_geo.vertices.size());
            m_geo.vertices.push_back(c1 + u * r);

            const auto at = [&](size_t ring, int side) {
                return firstRing + static_cast<uint32_t>(ring * kSides + static_cast<size_t>(side % kSides));
            };
            for (int s = 0; s < kSides; ++s) {
                addTri(pole0, at(0, s + 1), at(0, s), surface, layer);
                for (size_t k = 0; k + 1 < rings.size(); ++k) {
                    addTri(at(k, s), at(k, s + 1), at(k + 1, s + 1), surface, layer);
                    addTri(at(k, s), at(k + 1, s + 1), at(k + 1, s), surface, layer);
                }
                addTri(pole1, at(rings.size() - 1, s), at(rings.size() - 1, s + 1), surface, layer);
            }
        }

        [[nodiscard]] static bool inRange(const Vec3& p) {
            return std::fabs(p.x) <= kMaxCoord && std::fabs(p.y) <= kMaxCoord && std::fabs(p.z) <= kMaxCoord;   // also NaN
        }

        [[nodiscard]] static std::optional<Vec3> readVec3(Kv3Value v) {
            if (v.size() != 3)
                return std::nullopt;
            const auto x = v.at(0).number(), y = v.at(1).number(), z = v.at(2).number();
            if (!x || !y || !z)
                return std::nullopt;
            return Vec3{ static_cast<float>(*x), static_cast<float>(*y), static_cast<float>(*z) };
        }

        // The shape's layer and surface (every shape carries both).
        bool shapeTags(Kv3Value shape, uint16_t& layer, uint16_t& surface) {
            using namespace kv3_literals;
            const int64_t l = shape["m_nCollisionAttributeIndex"_kv].integer().value_or(-1);
            const int64_t s = shape["m_nSurfacePropertyIndex"_kv].integer().value_or(-1);
            if (l < 0 || static_cast<size_t>(l) >= m_geo.layers.size())
                return fail("a shape refers to a missing collision layer");
            layer = static_cast<uint16_t>(l);
            if (s >= 0 && static_cast<size_t>(s) < m_geo.surfaceHashes.size()) {
                surface = static_cast<uint16_t>(s);
            }
            else {
                surface = Geometry::kNoSurface;
                ++m_geo.stats.missingSurface;
            }
            return true;
        }

        // Convex hull: vertex positions + half-edges; each face is the loop of
        // half-edges from its first edge, triangulated as a fan.
        bool readHull(Kv3Value shape) {
            using namespace kv3_literals;
            nextSolid(SolidKind::Hull);
            uint16_t layer = 0, surface = 0;
            if (!shapeTags(shape, layer, surface))
                return false;
            const Kv3Value hull = shape["m_Hull"_kv];

            // With explicit vertex indices (current format) the positions are
            // m_VertexPositions and m_Vertices holds per-vertex edge indices.
            const Kv3Value positions = hull["m_VertexPositions"_kv].valid() ? hull["m_VertexPositions"_kv]
                                                                            : hull["m_Vertices"_kv];
            const auto base  = static_cast<uint32_t>(m_geo.vertices.size());
            const auto count = readVec3s(positions);
            const auto edges = readEdges(hull["m_Edges"_kv]);
            const auto faces = readFaces(hull["m_Faces"_kv]);
            if (!count || !edges || !faces)
                return fail("a hull's data is unreadable");

            for (uint8_t first : *faces) {
                if (first >= edges->size())
                    return fail("a hull face refers to a missing edge");
                // Walk the loop; a face can't have more edges than the hull.
                uint32_t corner[3];
                uint32_t n = 0;
                uint8_t  e = first;
                for (size_t steps = 0; steps <= edges->size(); ++steps) {
                    const HalfEdge& he = (*edges)[e];
                    if (he.origin >= *count || he.next >= edges->size())
                        return fail("a hull edge is out of range");
                    if (n < 2) {
                        corner[n++] = base + he.origin;
                    }
                    else {
                        corner[2] = base + he.origin;
                        addTri(corner[0], corner[1], corner[2], surface, layer);
                        ++m_geo.stats.hullTris;
                        corner[1] = corner[2];
                    }
                    e = he.next;
                    if (e == first)
                        break;
                    if (steps == edges->size())
                        return fail("a hull face doesn't close");
                }
            }
            ++m_geo.stats.hulls;
            return true;
        }

        // Triangle mesh: vertex positions + index triples.
        bool readMesh(Kv3Value shape) {
            using namespace kv3_literals;
            nextSolid(SolidKind::Mesh);
            uint16_t layer = 0, surface = 0;
            if (!shapeTags(shape, layer, surface))
                return false;
            const Kv3Value mesh = shape["m_Mesh"_kv];

            const auto base  = static_cast<uint32_t>(m_geo.vertices.size());
            const auto count = readVec3s(mesh["m_Vertices"_kv]);
            if (!count)
                return fail("a mesh's vertices are unreadable");

            const Kv3Value triangles = mesh["m_Triangles"_kv];
            const auto     triBlob   = triangles.blob();
            const size_t   triCount  = triBlob ? triBlob->size() / 12 : triangles.size();

            // Per-triangle materials (indices into Geometry::surfaceHashes; a
            // mesh mixes several - e.g. concrete and metal sheets), when there's
            // one per triangle; else the shape's own.
            std::vector<uint16_t> materials;
            const Kv3Value        mats = mesh["m_Materials"_kv];
            if (const auto mb = mats.blob()) {
                for (size_t i = 0; i < mb->size(); ++i)
                    materials.push_back(mb->at<uint8_t>(i).value_or(0));
            }
            else {
                for (size_t i = 0; i < mats.size(); ++i)
                    materials.push_back(static_cast<uint16_t>(mats.at(i).integer().value_or(0)));
            }
            if (materials.size() != triCount) {
                if (!materials.empty())
                    ++m_geo.stats.meshMaterialsIgnored;
                materials.clear();
            }

            size_t triIndex = 0;
            const auto add = [&](int32_t a, int32_t b, int32_t c) {
                if (a < 0 || b < 0 || c < 0 || static_cast<uint32_t>(a) >= *count ||
                    static_cast<uint32_t>(b) >= *count || static_cast<uint32_t>(c) >= *count)
                    return fail("a mesh triangle is out of range");
                uint16_t s = surface;
                if (triIndex < materials.size() && materials[triIndex] < m_geo.surfaceHashes.size())
                    s = materials[triIndex];
                ++triIndex;
                addTri(base + a, base + b, base + c, s, layer);
                ++m_geo.stats.meshTris;
                return true;
            };

            if (const auto blob = triangles.blob()) {
                if (blob->size() % 12 != 0)
                    return fail("a mesh's triangles are unreadable");
                ByteCursor c(*blob);
                while (c.remaining()) {
                    const int32_t a = c.read<int32_t>(), b = c.read<int32_t>(), d = c.read<int32_t>();
                    if (!add(a, b, d))
                        return false;
                }
            }
            else {
                for (size_t i = 0; i < triangles.size(); ++i) {   // older: objects {m_nIndex: [a, b, c]}
                    const Kv3Value idx = triangles.at(i)["m_nIndex"_kv];
                    if (idx.size() != 3)
                        return fail("a mesh's triangles are unreadable");
                    const auto get = [&](size_t k) { return static_cast<int32_t>(idx.at(k).integer().value_or(-1)); };
                    if (!add(get(0), get(1), get(2)))
                        return false;
                }
            }
            ++m_geo.stats.meshes;
            return true;
        }

        void addTri(uint32_t a, uint32_t b, uint32_t c, uint16_t surface, uint16_t layer) {
            const auto& v = m_geo.vertices;
            const Vec3 ab = v[b] - v[a], ac = v[c] - v[a];
            if (ab.cross(ac).length() < kMinTwiceArea) {
                ++m_geo.stats.degenerate;
                return;
            }
            m_geo.tris.push_back({ a, b, c, surface, layer, m_solid });
        }

        // ── Shape data: a blob (current format) or an array of values (older) ──
        // Appends the positions to the vertex list; returns how many.
        std::optional<uint32_t> readVec3s(Kv3Value v) {
            std::vector<Vec3> points;
            if (const auto blob = v.blob()) {
                if (blob->size() % sizeof(float[3]) != 0)
                    return std::nullopt;
                points.resize(blob->size() / sizeof(float[3]));
                std::memcpy(points.data(), blob->data(), blob->size());
            }
            else if (v.isArray()) {
                for (size_t i = 0; i < v.size(); ++i) {
                    const Kv3Value p = v.at(i);
                    if (p.size() != 3)
                        return std::nullopt;
                    points.push_back({ static_cast<float>(p.at(0).number().value_or(NAN)),
                                       static_cast<float>(p.at(1).number().value_or(NAN)),
                                       static_cast<float>(p.at(2).number().value_or(NAN)) });
                }
            }
            else {
                return std::nullopt;
            }

            if (points.size() > kMaxShapeVertices)
                return std::nullopt;
            for (const auto& p : points)
                if (!inRange(p))
                    return std::nullopt;
            m_geo.vertices.insert(m_geo.vertices.end(), points.begin(), points.end());
            return static_cast<uint32_t>(points.size());
        }

        static std::optional<std::vector<HalfEdge>> readEdges(Kv3Value v) {
            using namespace kv3_literals;
            std::vector<HalfEdge> edges;
            if (const auto blob = v.blob()) {
                if (blob->size() % sizeof(HalfEdge) != 0)
                    return std::nullopt;
                edges.resize(blob->size() / sizeof(HalfEdge));
                std::memcpy(edges.data(), blob->data(), blob->size());
            }
            else if (v.isArray()) {
                for (size_t i = 0; i < v.size(); ++i) {
                    const Kv3Value e = v.at(i);
                    const auto get = [&](uint32_t key) { return static_cast<uint8_t>(e[key].integer().value_or(0)); };
                    edges.push_back({ get("m_nNext"_kv), get("m_nTwin"_kv), get("m_nOrigin"_kv), get("m_nFace"_kv) });
                }
            }
            else {
                return std::nullopt;
            }
            return edges;
        }

        static std::optional<std::vector<uint8_t>> readFaces(Kv3Value v) {
            using namespace kv3_literals;
            if (const auto blob = v.blob())
                return std::vector<uint8_t>(blob->data(), blob->data() + blob->size());
            if (!v.isArray())
                return std::nullopt;
            std::vector<uint8_t> faces;
            for (size_t i = 0; i < v.size(); ++i)
                faces.push_back(static_cast<uint8_t>(v.at(i)["m_nEdge"_kv].integer().value_or(0)));
            return faces;
        }
    };

} // namespace s2ghtline
