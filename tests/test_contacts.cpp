// test_contacts: World::segmentContacts on real maps.
//   1. Winding: every CLOSED solid (each edge used by exactly two of its
//      triangles, in opposite directions) must have a positive signed volume
//      (front faces point out) - entering / leaving depends on it.
//   2. Random segments: per solid the contacts should alternate enter / leave
//      (a closed solid can't be entered twice without leaving; open sheets in
//      the world meshes can, so repeats are reported, not failed).
//   3. The game's surface table reached the world (no surface note).
// Maps: the arguments, else nine official maps.
// Skips without a CS2 install (support/Cs2Install.h).
#include "support/Check.h"
#include "support/Cs2Install.h"

#include "MapManager.h"

#include <chrono>
#include <cstdio>
#include <map>
#include <random>
#include <thread>
#include <unordered_map>

using namespace s2ghtline;
using Clock = std::chrono::steady_clock;

int main(int argc, char** argv) {
    std::string why;
    const auto csgo = cs2::find(why);
    if (!csgo)
        return check::noGame(why, cs2::named());
    std::printf("CS2: %s\n", csgo->string().c_str());

    MapManager maps(LayerRules{});
    std::vector<std::string> names = { "de_mirage", "de_dust2", "de_inferno", "de_nuke", "de_ancient", "de_overpass",
                                       "de_vertigo", "de_anubis", "de_train" };
    if (argc > 1)
        names.assign(argv + 1, argv + argc);
    for (const std::string& map : names) {
        const auto t0 = Clock::now();
        while (maps.update(map, *csgo), !maps.world() && maps.status().state != MapManager::State::Failed &&
                                        Clock::now() - t0 < std::chrono::seconds(90))
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        const auto w = maps.world();
        if (!w) {
            std::printf("%-12s not loaded: %s\n", map.c_str(), maps.status().reason.c_str());
            continue;
        }
        const Bvh& bvh = w->bvh();
        const auto openStart = Clock::now();
        const OpenGeometry og = findOpenGeometry(*w);
        std::printf("%-12s open geometry: %zu solids, %zu edges (%.0f ms; BVH build %.0f ms)\n", map.c_str(),
                    og.solids.size(), og.count,
                    std::chrono::duration<double, std::milli>(Clock::now() - openStart).count(), w->stats().buildMs);
        if (!w->surfaceNote().empty()) {
            std::printf("%-12s FAIL surface table: %s\n", map.c_str(), w->surfaceNote().c_str());
            ++check::failures();
        }

        // 1. Closed solids' winding.
        struct Solid { double volume = 0.0; std::map<std::pair<uint32_t, uint32_t>, int> edges; };
        std::unordered_map<uint32_t, Solid> solids;
        for (uint32_t i = 0; i < bvh.triangleCount(); ++i) {
            const Tri& t = bvh.tri(i);
            Solid& s = solids[t.solid];
            const Vec3 a = bvh.vertex(t.a), b = bvh.vertex(t.b), c = bvh.vertex(t.c);
            s.volume += (static_cast<double>(a.x) * (static_cast<double>(b.y) * c.z - static_cast<double>(b.z) * c.y) -
                         static_cast<double>(a.y) * (static_cast<double>(b.x) * c.z - static_cast<double>(b.z) * c.x) +
                         static_cast<double>(a.z) * (static_cast<double>(b.x) * c.y - static_cast<double>(b.y) * c.x)) / 6.0;
            for (auto [p, q] : { std::pair{ t.a, t.b }, std::pair{ t.b, t.c }, std::pair{ t.c, t.a } })
                s.edges[{ p, q }] += 1;
        }
        int closed = 0, open = 0, inward = 0;
        for (auto& [id, s] : solids) {
            bool isClosed = true;
            for (const auto& [e, n] : s.edges) {
                const auto back = s.edges.find({ e.second, e.first });
                if (n != 1 || back == s.edges.end() || back->second != 1) {
                    isClosed = false;
                    break;
                }
            }
            if (!isClosed) { ++open; continue; }
            ++closed;
            if (s.volume <= 0.0)
                ++inward;
        }

        // 2. Random segments through the map: per solid, contacts alternate.
        const Aabb box = bvh.bounds();
        std::mt19937 rng(7);
        std::uniform_real_distribution<float> ux(box.min.x, box.max.x), uy(box.min.y, box.max.y), uz(box.min.z, box.max.z);
        std::vector<World::SegmentContact> contacts;
        std::vector<Hit> scratch;
        long long segments = 0, total = 0, unpaired = 0;
        const auto start = Clock::now();
        for (int i = 0; i < 20000; ++i) {
            const Vec3 a{ ux(rng), uy(rng), uz(rng) };
            const Vec3 b{ a.x + (ux(rng) - a.x) * 0.3f, a.y + (uy(rng) - a.y) * 0.3f, uz(rng) };
            w->segmentContacts(a, b, contacts, scratch);
            ++segments;
            total += static_cast<long long>(contacts.size());
            std::unordered_map<uint32_t, int> last;   // solid -> +1 entered, -1 left
            for (const auto& c : contacts) {
                int& state = last[c.solid];
                const int now = c.entering ? 1 : -1;
                if (state == now)
                    ++unpaired;   // entered twice / left twice in a row
                state = now;
            }
        }
        const double us = std::chrono::duration<double, std::micro>(Clock::now() - start).count() /
                          static_cast<double>(segments);
        std::printf("%-12s solids: %d closed (%d wound inward), %d open | %lld segments, %.1f contacts each, %lld repeated"
                    " enter/leave (%.3f%%), %.1f us each\n",
                    map.c_str(), closed, inward, open, segments, static_cast<double>(total) / static_cast<double>(segments),
                    unpaired, total ? 100.0 * static_cast<double>(unpaired) / static_cast<double>(total) : 0.0, us);
        if (inward) {
            std::printf("%-12s FAIL %d closed solids wound inward\n", map.c_str(), inward);
            ++check::failures();
        }
    }
    return check::result();
}
