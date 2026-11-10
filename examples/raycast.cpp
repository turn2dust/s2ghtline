// raycast — loads a map's collision, builds the World and queries it.
//
//   raycast <map>          a map of the installed game, by name ("de_mirage",
//                          a workshop map's name, "workshop/<id>/<map>")
//   raycast <vpk> <map>    a map VPK (or a workshop item's VPK) by path
//
// The install is S2GHTLINE_CS2_DIR, else found through Steam; it's also where
// the surface properties come from (without it every surface gets the
// defaults).  Prints the world's stats, a few segments' first hits, and the
// cost of 100k random queries.
#include "MapLocator.h"
#include "MapPhysics.h"
#include "World.h"

#include "support/Cs2Install.h"

#include <chrono>
#include <cstdio>
#include <random>
#include <string>
#include <utility>
#include <vector>

using namespace s2ghtline;
using Clock = std::chrono::steady_clock;

namespace {

    double msSince(Clock::time_point t) { return std::chrono::duration<double, std::milli>(Clock::now() - t).count(); }

    std::optional<SurfaceTable> loadSurfaces(const std::filesystem::path& csgo, std::string& why) {
        const auto pak   = Vpk::open(csgo / "pak01_dir.vpk", why);
        const auto vsurf = pak ? pak->read("surfaceproperties/surfaceproperties.vsurf_c", why) : std::nullopt;
        const auto text  = vsurf ? pak->read("scripts/surfaceproperties_game.txt", why) : std::nullopt;
        if (!text)
            return std::nullopt;
        return SurfaceTable::parse(ByteView(*vsurf), std::string_view(reinterpret_cast<const char*>(text->data()), text->size()),
                                   why);
    }

    void printHit(const World& w, const char* what, Vec3 a, Vec3 b, uint64_t mask) {
        const auto hit = w.trace(a, b, mask);
        std::printf("  %-26s ", what);
        if (!hit) {
            std::printf("no hit\n");
            return;
        }
        const float distance = (b - a).length() * hit->t;
        const auto  layer    = w.layerOfTri(hit->tri);
        const auto& surface  = w.surfaceOfTri(hit->tri);
        const auto& name     = w.surfaceNameOfTri(hit->tri);
        std::printf("hit at %7.1f units  layer %-3s %-11s surface %-16s (penetration %.2f)\n", distance,
                    layer ? std::to_string(*layer).c_str() : "-", toString(w.classOfTri(hit->tri)),
                    name.empty() ? "?" : name.c_str(), surface.penetration);
    }

} // namespace

int main(int argc, char** argv) {
    if (argc != 2 && argc != 3) {
        std::printf("usage: raycast <map>\n       raycast <vpk> <map>\n");
        return 2;
    }

    std::string why;
    const auto csgo = cs2::find(why);
    std::filesystem::path vpkPath;
    std::string           map;
    if (argc == 3) {
        vpkPath = argv[1];
        map     = argv[2];
    }
    else {
        if (!csgo) {
            std::printf("%s\n", why.c_str());
            return 1;
        }
        MapLocator locator(*csgo);
        const auto source = locator.find(argv[1], why);
        if (!source) {
            std::printf("%s\n", why.c_str());
            return 1;
        }
        vpkPath = source->vpk;
        map     = source->map;
    }
    std::printf("map %s from %s\n", map.c_str(), vpkPath.string().c_str());

    // Read + parse the physics.
    auto t = Clock::now();
    const auto vpk = openMapVpk(vpkPath, map, why);
    auto       geo = vpk ? loadMapGeometry(*vpk, map, why) : std::nullopt;
    if (!geo) {
        std::printf("can't load the map: %s\n", why.c_str());
        return 1;
    }
    const double readMs = msSince(t);

    // Surface properties, from the install when there is one.
    std::string surfaceWhy = "no CS2 install";
    const auto  surfaces   = csgo ? loadSurfaces(*csgo, surfaceWhy) : std::nullopt;

    const auto world = WorldBuilder::build(map, std::move(*geo), LayerRules{}, {}, surfaces ? &*surfaces : nullptr, surfaceWhy);
    if (!world)
        return 1;

    const WorldStats& s = world->stats();
    std::printf("\nworld\n");
    std::printf("  read + parse        %8.0f ms\n", readMs);
    std::printf("  classes + BVH       %8.0f ms\n", s.buildMs);
    std::printf("  triangles           %8zu solid, %zu see-through, %zu clip\n", s.trisByClass[0], s.trisByClass[1],
                s.trisByClass[2]);
    std::printf("  shapes              %8u hulls, %u meshes, %u spheres, %u capsules\n", s.geometry.hulls, s.geometry.meshes,
                s.geometry.spheres, s.geometry.capsules);
    std::printf("  layers              %8zu\n", world->layerCount());
    std::printf("  memory              %8.1f MB (%.1f bytes per triangle)\n", static_cast<double>(s.memoryBytes) / 1048576.0,
                static_cast<double>(s.memoryBytes) / static_cast<double>(world->bvh().triangleCount()));
    std::printf("  surfaces            %s\n", world->surfaceNote().empty() ? "the game's surface properties"
                                                                            : ("defaults (" + world->surfaceNote() + ")").c_str());
    for (const auto& tag : s.unknownTags)
        std::printf("  unknown tag         %s (treated as solid)\n", tag.c_str());

    // A few segments through the middle of the map.
    const Aabb box = world->bvh().bounds();
    const Vec3 mid = (box.min + box.max) * 0.5f;
    std::printf("\nsegments from the centre of the bounds (%.0f, %.0f, %.0f)\n", mid.x, mid.y, mid.z);
    printHit(*world, "down, solid only", mid, { mid.x, mid.y, box.min.z }, world->visionMask());
    printHit(*world, "down, every layer", mid, { mid.x, mid.y, box.min.z }, kAllLayers);
    printHit(*world, "east, solid only", mid, { box.max.x, mid.y, mid.z }, world->visionMask());
    printHit(*world, "north, every layer", mid, { mid.x, box.max.y, mid.z }, kAllLayers);

    std::vector<World::SegmentContact> contacts;
    std::vector<Hit>                   scratch;
    world->segmentContacts({ box.min.x, mid.y, mid.z }, { box.max.x, mid.y, mid.z }, contacts, scratch);
    std::printf("  %-26s %zu surfaces crossed\n", "west to east, contacts", contacts.size());

    // 100k random segments of up to 3000 units.
    std::mt19937 rng(1);
    std::uniform_real_distribution<float> ux(box.min.x, box.max.x), uy(box.min.y, box.max.y), uz(box.min.z, box.max.z),
        unit(-1.f, 1.f), len(0.f, 3000.f);
    std::vector<std::pair<Vec3, Vec3>> segments;
    for (int i = 0; i < 100000; ++i) {
        const Vec3 a{ ux(rng), uy(rng), uz(rng) };
        Vec3 d{ unit(rng), unit(rng), unit(rng) };
        const float l = d.length();
        d = l > 0.f ? d * (len(rng) / l) : Vec3{};
        segments.push_back({ a, a + d });
    }
    std::printf("\n100k random segments\n");
    t = Clock::now();
    size_t visible = 0;
    for (const auto& [a, b] : segments)
        visible += world->visible(a, b);
    std::printf("  visible()           %8.2f us each (%.0f%% visible)\n", msSince(t) * 1000.0 / 100000.0,
                static_cast<double>(visible) / 1000.0);
    t = Clock::now();
    size_t hits = 0;
    for (const auto& [a, b] : segments)
        hits += world->trace(a, b, kAllLayers).has_value();
    std::printf("  trace()             %8.2f us each (%.0f%% hit)\n", msSince(t) * 1000.0 / 100000.0,
                static_cast<double>(hits) / 1000.0);
    t = Clock::now();
    size_t crossed = 0;
    for (const auto& [a, b] : segments) {
        world->segmentContacts(a, b, contacts, scratch);
        crossed += contacts.size();
    }
    std::printf("  segmentContacts()   %8.2f us each (%.2f surfaces each)\n", msSince(t) * 1000.0 / 100000.0,
                static_cast<double>(crossed) / 100000.0);
    return 0;
}
