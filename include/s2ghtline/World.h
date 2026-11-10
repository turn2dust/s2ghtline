#pragma once

#include "Bvh.h"
#include "PhysParser.h"
#include "SurfaceProperties.h"
#include "Vec3.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <stop_token>
#include <string>
#include <string_view>
#include <unordered_set>
#include <utility>
#include <vector>

// World.h — a loaded map: its collision in a BVH, every collision
// layer classed for line of sight, ready for queries (read-only once built,
// shared between threads as shared_ptr<const World>).
//
// Classes (from each layer's InteractAs tags, LayerRules):
//   Solid        world geometry and props: blocks line of sight
//   SeeThrough   chain-link fences (passbullets), glass (window), water
//   Clip         invisible walls (player / npc / grenade clip), ladders, sky
// Every class stays in the BVH (clip layers stay queryable); line of sight
// queries only the Solid layers (visionMask).
//
// Bullet penetration: segmentContacts() lists every surface a segment
// crosses, with each surface's values from the game's surface properties
// (SurfaceTable; "default"'s for a surface it doesn't know, and for every
// surface when the table couldn't be loaded - surfaceNote() says why), and
// whether a triangle's layer interacts as "passbullets".
namespace s2ghtline {

    enum class LayerClass : uint8_t { Solid, SeeThrough, Clip };
    constexpr size_t kLayerClasses = 3;

    [[nodiscard]] inline const char* toString(LayerClass c) {
        switch (c) {
        case LayerClass::Solid:      return "solid";
        case LayerClass::SeeThrough: return "see-through";
        case LayerClass::Clip:       return "clip";
        }
        return "?";
    }

    // LayerRules ---------------------------------------------------
    // Which InteractAs tags make a layer a clip or see-through.  Three lists,
    // defaulting to the tags below; set them to class layers differently:
    //   clip        tags of invisible walls, ladders and sky
    //   seeThrough  tags of surfaces line of sight passes (fences, glass, water)
    //   solid       tags known to be plain solid - listed only so that they
    //               aren't reported as unknown
    // A tag in no list is reported as unknown and treated as solid.
    struct LayerRules {
        std::vector<std::string> clip       { "playerclip", "npcclip", "csgo_grenadeclip", "ladder", "sky" };
        std::vector<std::string> seeThrough { "passbullets", "window", "water" };
        std::vector<std::string> solid      { "solid", "blocklight", "blocksound" };

        // Clip if any tag is a clip tag, else see-through if any tag is, else
        // solid.  Tags in no list go to `unknown` (the layer counts as solid:
        // treating a see-through layer as solid is the safe mistake for line
        // of sight).
        [[nodiscard]] LayerClass classify(const Layer& layer, std::vector<std::string>* unknown = nullptr) const {
            const auto in = [](const std::vector<std::string>& list, const std::string& tag) {
                return std::find(list.begin(), list.end(), tag) != list.end();
            };
            bool isClip = false, isSeeThrough = false;
            for (const auto& tag : layer.interactAs) {
                if (in(clip, tag))
                    isClip = true;
                else if (in(seeThrough, tag))
                    isSeeThrough = true;
                else if (!in(solid, tag) && unknown && !in(*unknown, tag))
                    unknown->push_back(tag);
            }
            return isClip ? LayerClass::Clip : isSeeThrough ? LayerClass::SeeThrough : LayerClass::Solid;
        }
    };

    struct WorldStats {
        GeometryStats            geometry;
        std::array<size_t, kLayerClasses> trisByClass{};
        std::vector<size_t>      trisPerLayer;
        std::vector<std::string> unknownTags;
        size_t                   memoryBytes = 0;
        double                   buildMs     = 0.0;   // classing + BVH
    };

    // World ---------------------------------------------------
    class World {
    public:
        [[nodiscard]] const std::string& map()   const { return m_map; }
        [[nodiscard]] const Bvh&         bvh()   const { return m_bvh; }
        [[nodiscard]] const WorldStats&  stats() const { return m_stats; }

        // Line of sight: nothing solid between a and b.
        [[nodiscard]] bool visible(const Vec3& a, const Vec3& b) const {
            return !m_bvh.occluded(a, b, m_visionMask);
        }
        // A solid triangle blocking a -> b, if any (for occluder caches) ...
        [[nodiscard]] std::optional<uint32_t> blocker(const Vec3& a, const Vec3& b) const {
            return m_bvh.blocker(a, b, m_visionMask);
        }
        // ... and whether that one solid triangle still blocks a -> b (a few
        // ns).  Any index is safe: true only for a solid triangle of THIS
        // world that really is in the way.
        [[nodiscard]] bool blocks(uint32_t tri, const Vec3& a, const Vec3& b) const {
            return tri < m_bvh.triangleCount() && (m_visionMask & layerBit(m_bvh.tri(tri).layer)) &&
                   m_bvh.hit(a, b, tri).has_value();
        }
        // The first surface of the masked classes from a toward b.
        [[nodiscard]] std::optional<Hit> trace(const Vec3& a, const Vec3& b, uint64_t mask) const {
            return m_bvh.closest(a, b, mask);
        }

        [[nodiscard]] uint64_t visionMask() const { return m_visionMask; }
        [[nodiscard]] uint64_t classMask(LayerClass c) const { return m_classMask[static_cast<size_t>(c)]; }

        // The map's collision layers and their classes.
        [[nodiscard]] size_t       layerCount()           const { return m_layers.size(); }
        [[nodiscard]] const Layer& layer(size_t i)        const { return m_layers[i]; }
        [[nodiscard]] LayerClass   layerClass(size_t i)   const { return m_classes[i]; }

        // A BVH triangle's layer (its Tri::layer is a mask bit, not a layer
        // index): nothing when several layers share the bit (maps with more
        // than 64 layers - none so far).
        [[nodiscard]] std::optional<uint16_t> layerOfTri(uint32_t tri) const {
            const int32_t l = m_bitLayer[m_bvh.tri(tri).layer];
            return l >= 0 ? std::optional<uint16_t>(static_cast<uint16_t>(l)) : std::nullopt;
        }
        [[nodiscard]] LayerClass classOfTri(uint32_t tri) const { return m_bitClass[m_bvh.tri(tri).layer]; }
        [[nodiscard]] std::optional<uint32_t> surfaceHashOfTri(uint32_t tri) const {
            const uint16_t s = m_bvh.tri(tri).surface;
            return s < m_surfaceHashes.size() ? std::optional<uint32_t>(m_surfaceHashes[s]) : std::nullopt;
        }

        // A triangle's penetration values, and whether its layer interacts as "passbullets".
        [[nodiscard]] const SurfaceProps& surfaceOfTri(uint32_t tri) const {
            const uint16_t s = m_bvh.tri(tri).surface;
            return s < m_surfaceProps.size() ? m_surfaceProps[s] : m_defaultSurface;
        }
        [[nodiscard]] bool onPassBulletsLayer(uint32_t tri) const { return m_bitPassBullets[m_bvh.tri(tri).layer]; }
        // A solid's kind (by Tri::solid; Hull when out of range).
        [[nodiscard]] SolidKind solidKind(uint32_t solid) const {
            return solid < m_solidKinds.size() ? m_solidKinds[solid] : SolidKind::Hull;
        }
        // A triangle's surface name (the surface table's; "" unknown).
        [[nodiscard]] const std::string& surfaceNameOfTri(uint32_t tri) const {
            static const std::string none;
            const uint16_t s = m_bvh.tri(tri).surface;
            return s < m_surfaceNames.size() ? m_surfaceNames[s] : none;
        }
        // Empty = the game's surface table was loaded; else why every surface
        // has "default"'s values.
        [[nodiscard]] const std::string& surfaceNote() const { return m_surfaceNote; }

        // The layers a bullet meets (segmentContacts() reports these): the
        // solid ones, and see-through layers that interact as "passbullets"
        // or "window" (fences, glass); not clips or water.
        [[nodiscard]] uint64_t segmentMask() const { return m_segmentMask; }

        // One surface a segment crosses: where (t, 0..1 along a -> b), which
        // triangle and solid (Tri::solid), and whether it enters the solid
        // (meets the triangle's front face: the parsed winding faces out).
        struct SegmentContact {
            float    t        = 0.f;
            uint32_t tri      = 0;
            uint32_t solid    = 0;
            bool     entering = true;
        };
        // Every surface of the segmentMask() layers between a and b, nearest first.
        void segmentContacts(const Vec3& a, const Vec3& b, std::vector<SegmentContact>& out,
                             std::vector<Hit>& scratch) const {
            out.clear();
            m_bvh.traceAll(a, b, m_segmentMask, scratch);
            const Vec3 dir = b - a;
            for (const Hit& h : scratch) {
                const Tri& t  = m_bvh.tri(h.tri);
                const Vec3 p0 = m_bvh.vertex(t.a), e1 = m_bvh.vertex(t.b) - p0, e2 = m_bvh.vertex(t.c) - p0;
                out.push_back({ h.t, h.tri, t.solid, e1.cross(e2).dot(dir) < 0.f });
            }
        }

    private:
        friend class WorldBuilder;

        std::string                            m_map;
        Bvh                                    m_bvh;
        std::vector<Layer>                     m_layers;
        std::vector<LayerClass>                m_classes;         // per layer
        std::array<int32_t, 64>                m_bitLayer{};      // mask bit -> its layer, -1 = shared
        std::array<LayerClass, 64>             m_bitClass{};      // mask bit -> class
        std::array<uint64_t, kLayerClasses>    m_classMask{};
        uint64_t                               m_visionMask = 0;
        std::vector<uint32_t>                  m_surfaceHashes;
        std::vector<SurfaceProps>              m_surfaceProps;    // per surface (m_surfaceHashes order)
        std::vector<SolidKind>                 m_solidKinds;      // by Tri::solid
        std::vector<std::string>               m_surfaceNames;    // ... and its name
        SurfaceProps                           m_defaultSurface;
        std::string                            m_surfaceNote;
        std::array<bool, 64>                   m_bitPassBullets{};   // mask bit -> its layer interacts as "passbullets"
        uint64_t                               m_segmentMask = 0;          // the layers segmentContacts() reports
        WorldStats                             m_stats;
    };

    // WorldBuilder ---------------------------------------------------
    // Geometry + layer rules -> World.  Each layer gets a mask bit (its own
    // index; past 64 layers, layers 61+ share one bit per class).
    class WorldBuilder {
    public:
        // Nothing if `stop` was requested (the map changed mid-build).
        // `surfaces` null: every surface gets the defaults (`surfaceNote` why).
        [[nodiscard]] static std::optional<World> build(std::string map, Geometry geo, const LayerRules& rules,
                                                        std::stop_token stop = {}, const SurfaceTable* surfaces = nullptr,
                                                        std::string surfaceNote = "no surface table") {
            const auto start = std::chrono::steady_clock::now();
            World w;
            w.m_map           = std::move(map);
            w.m_layers        = std::move(geo.layers);
            w.m_surfaceHashes = std::move(geo.surfaceHashes);
            w.m_solidKinds    = std::move(geo.solidKinds);
            w.m_stats.geometry = geo.stats;
            if (surfaces) {
                w.m_defaultSurface = surfaces->defaults();
                for (const uint32_t h : w.m_surfaceHashes) {
                    w.m_surfaceProps.push_back(surfaces->get(h));
                    w.m_surfaceNames.push_back(surfaces->name(h));
                }
            }
            else {
                w.m_surfaceNote = std::move(surfaceNote);
                w.m_surfaceProps.assign(w.m_surfaceHashes.size(), w.m_defaultSurface);
            }

            // Classes and mask bits.
            std::vector<uint8_t> bitOf(w.m_layers.size());
            w.m_bitLayer.fill(-1);
            w.m_bitClass.fill(LayerClass::Solid);
            const bool shared = w.m_layers.size() > 64;
            for (size_t i = 0; i < w.m_layers.size(); ++i) {
                const LayerClass c = rules.classify(w.m_layers[i], &w.m_stats.unknownTags);
                w.m_classes.push_back(c);
                const uint8_t bit = !shared || i < kSharedBase ? static_cast<uint8_t>(i)
                                                                : static_cast<uint8_t>(kSharedBase + static_cast<size_t>(c));
                bitOf[i] = bit;
                w.m_bitLayer[bit] = shared && bit >= kSharedBase ? -1 : static_cast<int32_t>(i);
                w.m_bitClass[bit] = c;
                w.m_classMask[static_cast<size_t>(c)] |= 1ull << bit;
                const bool own = !shared || bit < kSharedBase;   // (a shared bit: by class only)
                bool contactLayer = c == LayerClass::Solid;
                for (const std::string& tag : w.m_layers[i].interactAs) {
                    if (equalsLower(tag, "passbullets"))
                        w.m_bitPassBullets[bit] = own;
                    if (c == LayerClass::SeeThrough && own && (equalsLower(tag, "passbullets") || equalsLower(tag, "window")))
                        contactLayer = true;
                }
                if (contactLayer)
                    w.m_segmentMask |= 1ull << bit;
            }
            w.m_visionMask = w.m_classMask[static_cast<size_t>(LayerClass::Solid)];

            w.m_stats.trisPerLayer.assign(w.m_layers.size(), 0);
            for (Tri& t : geo.tris) {
                ++w.m_stats.trisByClass[static_cast<size_t>(w.m_classes[t.layer])];
                ++w.m_stats.trisPerLayer[t.layer];
                t.layer = bitOf[t.layer];
            }

            w.m_bvh = Bvh::build(std::move(geo.vertices), std::move(geo.tris), stop);
            if (stop.stop_requested())
                return std::nullopt;
            w.m_stats.memoryBytes = w.m_bvh.memoryBytes();
            w.m_stats.buildMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
            return w;
        }

    private:
        static constexpr size_t kSharedBase = 61;   // bits 61-63: solid / see-through / clip when shared

        static bool equalsLower(std::string_view a, std::string_view lower) {
            return a.size() == lower.size() && std::equal(a.begin(), a.end(), lower.begin(), [](char x, char y) {
                       return static_cast<char>(x >= 'A' && x <= 'Z' ? x - 'A' + 'a' : x) == y;
                   });
        }
    };

    // OpenGeometry ---------------------------------------------------------
    // Where solids aren't closed: an edge no other triangle of its solid uses
    // the other way round (one-sided sheets in the big world meshes - a bullet
    // crossing one is "unresolved" for a penetration walk).  Every shape has
    // its own vertices, so a directed edge (a, b) belongs to one solid: it's
    // open when (b, a) isn't there.  Bit k of a triangle = its edge from
    // vertex k (a->b, b->c, c->a).  ~0.5-2 s on big maps: run it off the
    // caller's latency-sensitive thread.
    struct OpenGeometry {
        std::vector<uint8_t>         edges;    // per triangle (BVH order)
        std::unordered_set<uint32_t> solids;   // solids with any open edge
        size_t                       count = 0;
    };
    [[nodiscard]] inline OpenGeometry findOpenGeometry(const World& world) {
        const Bvh& bvh = world.bvh();
        const auto key = [](uint32_t a, uint32_t b) { return (static_cast<uint64_t>(a) << 32) | b; };
        std::vector<uint64_t> directed;
        directed.reserve(bvh.triangleCount() * 3);
        for (uint32_t i = 0; i < bvh.triangleCount(); ++i) {
            const Tri& t = bvh.tri(i);
            directed.push_back(key(t.a, t.b));
            directed.push_back(key(t.b, t.c));
            directed.push_back(key(t.c, t.a));
        }
        std::sort(directed.begin(), directed.end());
        const auto has = [&](uint64_t k) { return std::binary_search(directed.begin(), directed.end(), k); };
        OpenGeometry out;
        out.edges.assign(bvh.triangleCount(), 0);
        for (uint32_t i = 0; i < bvh.triangleCount(); ++i) {
            const Tri&     t    = bvh.tri(i);
            const uint32_t v[3] = { t.a, t.b, t.c };
            uint8_t        bits = 0;
            for (int k = 0; k < 3; ++k)
                if (!has(key(v[(k + 1) % 3], v[k])))
                    bits |= static_cast<uint8_t>(1u << k);
            if (bits) {
                out.edges[i] = bits;
                out.solids.insert(t.solid);
                out.count += static_cast<size_t>((bits & 1) + ((bits >> 1) & 1) + ((bits >> 2) & 1));
            }
        }
        return out;
    }

} // namespace s2ghtline
