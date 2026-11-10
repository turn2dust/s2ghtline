// map_check <vpk> <map> [<vpk> <map> ...]
// Loads each map's collision with s2ghtline::loadMapGeometry and prints a
// canonical summary - the C++ side of the oracle comparison: tools/MapOracle
// prints the same lines from ValveResourceFormat, and
// `python tests/compare_oracle.py` diffs the two over every installed map.
// Hull + mesh data is compared exactly; spheres / capsules by count (their
// tessellation is ours).  Lines starting "  info" are not compared.
#include "MapPhysics.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <map>

using Clock = std::chrono::steady_clock;

static double msSince(Clock::time_point t) {
    return std::chrono::duration<double, std::milli>(Clock::now() - t).count();
}

static int check(const char* vpkPath, const char* map) {
    std::string why;
    const auto t0  = Clock::now();
    const auto vpk = s2ghtline::openMapVpk(vpkPath, map, why);
    const auto geo = vpk ? s2ghtline::loadMapGeometry(*vpk, map, why) : std::nullopt;
    if (!geo) {
        std::printf("map %s\n  FAILED %s\n", map, why.c_str());
        return 1;
    }
    const double ms = msSince(t0);

    // Hull + mesh triangles come first, sphere / capsule triangles last.
    const auto& s = geo->stats;
    const size_t solidTris = geo->tris.size() - s.primitiveTris;
    uint32_t solidVerts = static_cast<uint32_t>(geo->vertices.size());
    for (size_t i = solidTris; i < geo->tris.size(); ++i)
        solidVerts = std::min({ solidVerts, geo->tris[i].a, geo->tris[i].b, geo->tris[i].c });

    std::printf("map %s\n", map);
    std::printf("  info load %.0f ms, %zu triangles total, %u primitive triangles\n", ms, geo->tris.size(), s.primitiveTris);
    std::printf("  shapes hulls %u meshes %u spheres %u capsules %u compounds %u\n",
                s.hulls, s.meshes, s.spheres, s.capsules, s.compounds);
    std::printf("  hull+mesh vertices %u triangles %zu degenerate %u\n", solidVerts, solidTris, s.degenerate);

    double sum[3] = {};
    for (uint32_t i = 0; i < solidVerts; ++i) {
        sum[0] += geo->vertices[i].x; sum[1] += geo->vertices[i].y; sum[2] += geo->vertices[i].z;
    }
    std::printf("  vertex sum %.3f %.3f %.3f\n", sum[0], sum[1], sum[2]);

    std::vector<size_t> solidPerLayer(geo->layers.size()), primPerLayer(geo->layers.size());
    std::map<uint16_t, size_t> perSurface;
    for (size_t i = 0; i < geo->tris.size(); ++i) {
        const auto& t = geo->tris[i];
        if (i < solidTris) {
            ++solidPerLayer[t.layer];
            ++perSurface[t.surface];
        }
        else {
            ++primPerLayer[t.layer];
        }
    }
    for (size_t i = 0; i < geo->layers.size(); ++i) {
        const auto& l = geo->layers[i];
        std::string as, ex;
        for (const auto& a : l.interactAs) as += " " + a;
        for (const auto& e : l.exclude)    ex += " " + e;
        std::printf("  layer %zu %s as[%s ] exclude[%s ] tris %zu primitive-tris %zu\n", i, l.group.c_str(), as.c_str(),
                    ex.c_str(), solidPerLayer[i], primPerLayer[i]);
    }
    std::printf("  surfaces %zu\n", geo->surfaceHashes.size());
    for (const auto& [surface, n] : perSurface) {
        if (surface == s2ghtline::Geometry::kNoSurface)
            std::printf("  surface none tris %zu\n", n);
        else
            std::printf("  surface %u %u tris %zu\n", surface, geo->surfaceHashes[surface], n);
    }
    return 0;
}

int main(int argc, char** argv) {
    if (argc < 3 || argc % 2 == 0) {
        std::printf("usage: map_check <vpk> <map> [<vpk> <map> ...]\n");
        return 2;
    }
    int failures = 0;
    for (int i = 1; i + 1 < argc; i += 2)
        failures += check(argv[i], argv[i + 1]);
    return failures ? 1 : 0;
}
