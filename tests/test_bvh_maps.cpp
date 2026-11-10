// test_bvh_maps: the BVH on real maps - brute-force agreement on random
// segments, and the numbers the README quotes: build time (single thread),
// memory, and per-segment query cost of occluded / closest / traceAll.
// Maps: the arguments, else de_mirage, de_dust2, de_inferno, de_boulder.
// Skips without a CS2 install (support/Cs2Install.h).
#include "support/BvhHelpers.h"
#include "support/Check.h"
#include "support/Cs2Install.h"

#include "MapLocator.h"
#include "MapPhysics.h"

#include <cstdio>
#include <random>
#include <string>
#include <vector>

using namespace bvh_helpers;
using namespace s2ghtline;

namespace {

    int bruteRaysFor(size_t tris) { return tris < 200000 ? 300 : tris < 1000000 ? 60 : 30; }

    void testMap(MapLocator& locator, const std::string& map) {
        std::string why;
        const auto source = locator.find(map, why);
        const auto vpk    = source ? openMapVpk(source->vpk, source->map, why) : std::nullopt;
        auto       geo    = vpk ? loadMapGeometry(*vpk, source->map, why) : std::nullopt;
        if (!geo) {
            std::printf("  %-12s FAIL can't load (%s)\n", map.c_str(), why.c_str());
            ++check::failures();
            return;
        }
        const size_t tris = geo->tris.size();
        const auto t0 = Clock::now();
        const Bvh bvh = Bvh::build(std::move(geo->vertices), std::move(geo->tris));
        const double buildMs = msSince(t0);

        std::mt19937 rng(11);
        const auto queries = randomSegments(rng, bvh.bounds(), 200000, 3000.f);
        const auto timeUs = [&](auto&& query) {
            const auto t = Clock::now();
            size_t n = 0;
            for (const auto& [a, b] : queries)
                n += query(a, b);
            return std::pair{ msSince(t) * 1000.0 / static_cast<double>(queries.size()), n };
        };
        const auto [occludedUs, blocked] = timeUs([&](Vec3 a, Vec3 b) { return bvh.occluded(a, b, kAllLayers) ? 1u : 0u; });
        const auto [closestUs, found]    = timeUs([&](Vec3 a, Vec3 b) { return bvh.closest(a, b, kAllLayers) ? 1u : 0u; });
        std::vector<Hit> all;
        const auto [traceAllUs, hits]    = timeUs([&](Vec3 a, Vec3 b) { bvh.traceAll(a, b, kAllLayers, all); return all.size(); });

        const int  bruteRays = bruteRaysFor(tris);
        const auto check     = randomSegments(rng, bvh.bounds(), bruteRays, 3000.f);
        const int  bad       = compare(bvh, check, kAllLayers);

        std::printf("  %-12s %8zu tris  build %6.0f ms  %6.1f MB (%4.1f B/tri)  occluded %.2f us  closest %.2f us"
                    "  traceAll %.2f us  (%.0f%% blocked, %.1f hits/segment)  brute force %d/%d mismatches\n",
                    map.c_str(), tris, buildMs, static_cast<double>(bvh.memoryBytes()) / 1048576.0,
                    static_cast<double>(bvh.memoryBytes()) / static_cast<double>(tris), occludedUs, closestUs, traceAllUs,
                    100.0 * static_cast<double>(blocked) / static_cast<double>(queries.size()),
                    static_cast<double>(hits) / static_cast<double>(queries.size()), bad, bruteRays);
        (void)found;
        if (bad)
            ++check::failures();
    }

} // namespace

int main(int argc, char** argv) {
    std::string why;
    const auto csgo = cs2::find(why);
    if (!csgo)
        return check::noGame(why, cs2::named());
    std::printf("CS2: %s\n", csgo->string().c_str());

    std::vector<std::string> maps = { "de_mirage", "de_dust2", "de_inferno", "de_boulder" };
    if (argc > 1)
        maps.assign(argv + 1, argv + argc);

    MapLocator locator(*csgo);
    check::section("maps");
    for (const auto& map : maps)
        testMap(locator, map);
    return check::result();
}
