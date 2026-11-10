// test_world_maps: MapLocator and MapManager on the installed game -
// official and workshop maps, a switch mid-load (a big map cancelled), reuse
// of the cached world, failure - and line of sight on a real map.
// Skips without a CS2 install (support/Cs2Install.h); the workshop
// checks use the first installed workshop map that parses (skipped if none).
#include "support/Check.h"
#include "support/Cs2Install.h"

#include "MapManager.h"

#include <chrono>
#include <cstdio>
#include <thread>

using namespace s2ghtline;
using check::expect;
using Clock = std::chrono::steady_clock;

namespace {

    double msSince(Clock::time_point t) { return std::chrono::duration<double, std::milli>(Clock::now() - t).count(); }

    MapManager::Status run(MapManager& m, std::string_view map, const std::filesystem::path& csgo, double timeoutS = 90.0) {
        const auto start = Clock::now();
        for (;;) {
            m.update(map, csgo);
            const auto s = m.status();
            if (s.state != MapManager::State::Loading ||
                std::chrono::duration<double>(Clock::now() - start).count() > timeoutS)
                return s;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    using WorkshopMap = std::optional<std::pair<std::string, std::string>>;   // item id, map

    // The first installed workshop map that parses.
    WorkshopMap pickWorkshopMap(const std::filesystem::path& csgo) {
        MapLocator loc(csgo);
        for (const auto& [id, map] : cs2::workshopMaps(csgo)) {
            std::string why;
            const auto source = loc.find("workshop/" + id + "/" + map, why);
            const auto vpk    = source ? openMapVpk(source->vpk, source->map, why) : std::nullopt;
            if (vpk && loadMapGeometry(*vpk, map, why))
                return std::pair{ id, map };
        }
        return std::nullopt;
    }

    void testLocator(const std::filesystem::path& csgo, const WorkshopMap& workshop) {
        check::section("locator (real install)");
        MapLocator loc(csgo);
        std::string why;
        const auto dust = loc.find("de_dust2", why);
        expect(dust && dust->kind == MapSource::Kind::Official && dust->vpk.filename() == "de_dust2.vpk", "official map");
        const auto mirage = loc.find("maps/de_mirage.vpk", why);
        expect(mirage && mirage->map == "de_mirage", "\"maps/<map>.vpk\" form");
        expect(!loc.find("no_such_map_x", why) && !why.empty(), "unknown map: not found, with a reason");
        const auto again = loc.find("de_dust2", why);
        expect(dust && again && dust->sameAs(*again), "same file twice: sameAs");
        if (!workshop)
            return;
        const auto& [id, map] = *workshop;
        const auto byName = loc.find(map, why);
        expect(byName && byName->kind == MapSource::Kind::Workshop && byName->workshopId == id, "workshop map by name");
        const auto byPath = loc.find("workshop/" + id + "/" + map, why);
        expect(byName && byPath && byPath->vpk == byName->vpk, "workshop map by workshop path");
    }

    void testManager(const std::filesystem::path& csgo, const WorkshopMap& workshop) {
        check::section("manager (real maps)");
        MapManager m(LayerRules{});

        auto t = Clock::now();
        auto s = run(m, "de_mirage", csgo);
        std::printf("  de_mirage: %s in %.0f ms (locate %.0f, read %.0f, build %.0f)\n", toString(s.state), msSince(t),
                    s.locateMs, s.readMs, m.world() ? m.world()->stats().buildMs : 0.0);
        expect(s.state == MapManager::State::Ready && m.world() && m.world()->map() == "de_mirage" &&
               m.world()->bvh().triangleCount() > 100000, "loads de_mirage");
        expect(m.world() && m.world()->surfaceNote().empty(), "the game's surface table is used");
        if (m.world()) {
            const auto& st = m.world()->stats();
            std::printf("  de_mirage: %zu solid, %zu see-through, %zu clip triangles; %zu unknown tags\n",
                        st.trisByClass[0], st.trisByClass[1], st.trisByClass[2], st.unknownTags.size());
            for (const auto& tag : st.unknownTags)
                std::printf("    unknown tag: %s\n", tag.c_str());
        }

        // Switch to a big map and away again at once: the big job is cancelled / discarded.
        m.update("de_boulder", csgo);
        expect(!m.world() && m.status().state == MapManager::State::Loading, "map change: world cleared, loading");
        s = run(m, "de_dust2", csgo);
        expect(s.state == MapManager::State::Ready && m.world() && m.world()->map() == "de_dust2",
               "switching mid-load ends on the new map (boulder discarded)");

        // Line of sight across a real map: some segments blocked, some not.
        if (const auto w = m.world()) {
            const Aabb box = w->bvh().bounds();
            int blocked = 0, total = 0;
            for (int i = 0; i < 1000; ++i) {
                const float f = static_cast<float>(i) / 1000.f;
                const Vec3 a{ box.min.x + (box.max.x - box.min.x) * f, box.min.y + (box.max.y - box.min.y) * 0.5f,
                              box.min.z + (box.max.z - box.min.z) * 0.5f };
                const Vec3 b{ a.x, box.min.y + (box.max.y - box.min.y) * (1.f - f), a.z };
                blocked += !w->visible(a, b);
                ++total;
            }
            std::printf("  de_dust2: %d / %d segments blocked\n", blocked, total);
            expect(blocked > 0 && blocked < total, "line of sight on a real map: some segments blocked, not all");
        }

        // Leave and come back: the cached world is reused.
        m.update("", csgo);
        expect(!m.world() && m.status().state == MapManager::State::Idle, "no map wanted: no world");
        t = Clock::now();
        s = run(m, "de_dust2", csgo);
        std::printf("  de_dust2 again: %.0f ms, reused %d\n", msSince(t), s.reused);
        expect(s.state == MapManager::State::Ready && s.reused && m.world(), "asking again reuses the cached world");

        if (workshop) {
            s = run(m, workshop->second, csgo);
            expect(s.state == MapManager::State::Ready && s.source && s.source->kind == MapSource::Kind::Workshop,
                   "workshop map loads");
        }

        s = run(m, "no_such_map_x", csgo);
        expect(s.state == MapManager::State::Failed && !s.reason.empty() && !m.world(), "unknown map: failed, with a reason");
        std::printf("    (reason: %s)\n", s.reason.c_str());
    }

} // namespace

int main() {
    std::string why;
    const auto csgo = cs2::find(why);
    if (!csgo)
        return check::noGame(why, cs2::named());
    std::printf("CS2: %s\n", csgo->string().c_str());

    const WorkshopMap workshop = pickWorkshopMap(*csgo);
    if (workshop)
        std::printf("workshop map: %s (item %s)\n", workshop->second.c_str(), workshop->first.c_str());
    else
        std::printf("(no installed workshop map parses: workshop checks skipped)\n");

    testLocator(*csgo, workshop);
    testManager(*csgo, workshop);
    return check::result();
}
