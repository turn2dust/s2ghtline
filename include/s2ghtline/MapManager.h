#pragma once

#include "MapLocator.h"
#include "MapPhysics.h"
#include "World.h"

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include <atomic>
#include <chrono>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <stop_token>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

// MapManager.h — loads a map's collision in the background and hands out the
// finished World.  Nothing is written to disk.
//
// Owner thread: call update(mapName, csgoDir) whenever the wanted map may have
// changed ("" = none), and again to pick up a finished build: world() only
// changes inside update().  A new map starts a job on a worker thread
// (below-normal priority on Windows): locate the VPK, read + parse the
// physics, class the layers, build the BVH.  world() is null until the build
// finishes.  A change cancels the job in flight; a job that finishes for a map
// no longer wanted is discarded (job ids).  The last world is cached: asking
// for the same map again, unchanged on disk, reuses it at once.  Workers only
// read files.
namespace s2ghtline {

    class MapManager {
    public:
        enum class State { Idle, Loading, Ready, Failed };

        struct Status {
            State                    state = State::Idle;
            std::string              map;      // the wanted map's name
            std::string              stage;    // Loading: locating / reading / building
            std::string              reason;   // Failed: why
            std::optional<MapSource> source;
            bool                     reused   = false;   // the cached world was reused
            double                   locateMs = 0.0, readMs = 0.0;
        };

        explicit MapManager(LayerRules rules) : m_rules(std::move(rules)) {}

        MapManager(const MapManager&)            = delete;
        MapManager& operator=(const MapManager&) = delete;

        // Owner thread; cheap when nothing changed, so it can be polled.
        void update(std::string_view mapName, const std::filesystem::path& csgoDir) {
            const std::string key = std::string(mapName) + '|' + csgoDir.string();
            if (key != m_key) {
                m_key = key;
                restart(std::string(mapName), csgoDir);
            }
            {
                std::lock_guard lock(m_mutex);
                m_world = m_ready;
            }
            std::erase_if(m_workers, [](const Worker& w) { return w.done->load(); });   // joins finished ones
        }

        // The wanted map's world as of the last update(); null while loading,
        // after a failure and when no map is wanted.
        [[nodiscard]] const std::shared_ptr<const World>& world() const { return m_world; }

        [[nodiscard]] Status status() const {
            std::lock_guard lock(m_mutex);
            return m_status;
        }

    private:
        using Clock = std::chrono::steady_clock;

        struct Worker {
            std::shared_ptr<std::atomic<bool>> done;
            std::jthread                       thread;
        };

        const LayerRules m_rules;   // read by workers, never changed

        // Shared with the workers, guarded by m_mutex.
        mutable std::mutex           m_mutex;
        uint64_t                     m_job = 0;        // the current job; older ones' results are discarded
        Status                       m_status;
        std::shared_ptr<const World> m_ready;          // the current map's world, once built
        std::optional<MapSource>     m_cacheSource;    // the last world built, and what from
        std::shared_ptr<const World> m_cache;

        // One locator per game install (its workshop index), used by one worker at a time.
        std::mutex                   m_locatorMutex;
        std::optional<MapLocator>    m_locator;

        // The surface table, per game install (surfaceTable()).
        std::mutex                          m_surfaceMutex;
        std::filesystem::path               m_surfaceDir;
        std::shared_ptr<const SurfaceTable> m_surfaces;
        std::string                         m_surfaceWhy;

        // Owner thread.
        std::string                  m_key;            // map name + install the current job is for
        std::shared_ptr<const World> m_world;

        std::vector<Worker>          m_workers;        // last member: joined first on destruction

        void restart(std::string map, std::filesystem::path csgoDir) {
            for (auto& w : m_workers)
                w.thread.request_stop();
            m_world.reset();

            uint64_t job = 0;
            {
                std::lock_guard lock(m_mutex);
                job      = ++m_job;
                m_ready.reset();
                m_status = {};
                m_status.map = map;
                // A different map starts: free the old world now ("" keeps it, for asking again).
                if (m_cache && !map.empty() && m_cache->map() != map) {
                    m_cache.reset();
                    m_cacheSource.reset();
                }
                if (!map.empty() && !csgoDir.empty()) {
                    m_status.state = State::Loading;
                    m_status.stage = "locating";
                }
            }
            if (map.empty() || csgoDir.empty())
                return;

            auto done = std::make_shared<std::atomic<bool>>(false);
            m_workers.push_back({ done, std::jthread([this, job, map = std::move(map), csgo = std::move(csgoDir), done]
                                                     (std::stop_token stop) {
#ifdef _WIN32
                SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL);
#endif
                load(stop, job, map, csgo);
                done->store(true);
            }) });
        }

        // ── Worker thread ────────────────────────────────────
        void load(const std::stop_token& stop, uint64_t job, const std::string& map, const std::filesystem::path& csgo) {
            std::string why;
            auto t = Clock::now();
            std::optional<MapSource> source;
            {
                std::lock_guard lock(m_locatorMutex);
                if (!m_locator || m_locator->csgoDir() != csgo)
                    m_locator.emplace(csgo);
                source = m_locator->find(map, why);
            }
            if (!source)
                return fail(job, why);
            const double locateMs = msSince(t);

            if (reuse(job, *source, locateMs) || stop.stop_requested())
                return;

            setStage(job, "reading", source, locateMs);
            t = Clock::now();
            const auto vpk = openMapVpk(source->vpk, source->map, why);
            auto geo = vpk ? loadMapGeometry(*vpk, source->map, why) : std::nullopt;
            if (stop.stop_requested())
                return;
            if (!geo)
                return fail(job, why);
            const double readMs = msSince(t);

            setStage(job, "building", source, locateMs, readMs);
            std::string surfaceWhy;
            const auto  surfaces = surfaceTable(csgo, surfaceWhy);
            auto built = WorldBuilder::build(map, std::move(*geo), m_rules, stop, surfaces.get(), surfaceWhy);
            if (!built)
                return;   // stopped
            publish(job, std::make_shared<const World>(std::move(*built)), *source);
        }

        // The game's surface properties (bullet penetration), read from pak01
        // once per install; null (and `why`) when unreadable.
        std::shared_ptr<const SurfaceTable> surfaceTable(const std::filesystem::path& csgo, std::string& why) {
            std::lock_guard lock(m_surfaceMutex);
            if (m_surfaceDir != csgo || (!m_surfaces && m_surfaceWhy.empty())) {
                m_surfaceDir = csgo;
                m_surfaces.reset();
                m_surfaceWhy.clear();
                const auto pak   = Vpk::open(csgo / "pak01_dir.vpk", m_surfaceWhy);
                const auto vsurf = pak ? pak->read("surfaceproperties/surfaceproperties.vsurf_c", m_surfaceWhy) : std::nullopt;
                const auto game  = vsurf ? pak->read("scripts/surfaceproperties_game.txt", m_surfaceWhy) : std::nullopt;
                if (game) {
                    auto t = SurfaceTable::parse(ByteView(*vsurf),
                                                 std::string_view(reinterpret_cast<const char*>(game->data()), game->size()),
                                                 m_surfaceWhy);
                    if (t)
                        m_surfaces = std::make_shared<const SurfaceTable>(std::move(*t));
                }
                if (!m_surfaces && m_surfaceWhy.empty())
                    m_surfaceWhy = "surface properties unreadable";
            }
            why = m_surfaceWhy;
            return m_surfaces;
        }

        // The cached world, if it was built from this same file.
        bool reuse(uint64_t job, const MapSource& source, double locateMs) {
            std::lock_guard lock(m_mutex);
            if (job != m_job || !m_cache || !m_cacheSource || !m_cacheSource->sameAs(source))
                return false;
            m_ready           = m_cache;
            m_status.state    = State::Ready;
            m_status.stage.clear();
            m_status.source   = source;
            m_status.reused   = true;
            m_status.locateMs = locateMs;
            return true;
        }

        void setStage(uint64_t job, const char* stage, const std::optional<MapSource>& source, double locateMs,
                      double readMs = 0.0) {
            std::lock_guard lock(m_mutex);
            if (job != m_job)
                return;
            m_status.stage    = stage;
            m_status.source   = source;
            m_status.locateMs = locateMs;
            m_status.readMs   = readMs;
        }

        void publish(uint64_t job, std::shared_ptr<const World> world, const MapSource& source) {
            std::lock_guard lock(m_mutex);
            if (job != m_job)
                return;   // we left that map meanwhile
            m_ready        = world;
            m_cache        = std::move(world);
            m_cacheSource  = source;
            m_status.state = State::Ready;
            m_status.stage.clear();
        }

        void fail(uint64_t job, const std::string& why) {
            std::lock_guard lock(m_mutex);
            if (job != m_job)
                return;
            m_status.state  = State::Failed;
            m_status.stage.clear();
            m_status.reason = why;
        }

        static double msSince(Clock::time_point t) {
            return std::chrono::duration<double, std::milli>(Clock::now() - t).count();
        }
    };

    [[nodiscard]] inline const char* toString(MapManager::State s) {
        switch (s) {
        case MapManager::State::Idle:    return "idle";
        case MapManager::State::Loading: return "loading";
        case MapManager::State::Ready:   return "ready";
        case MapManager::State::Failed:  return "failed";
        }
        return "?";
    }

} // namespace s2ghtline
