#pragma once

#include "Vpk.h"

#include <cctype>
#include <chrono>
#include <filesystem>
#include <optional>
#include <regex>
#include <string>
#include <string_view>
#include <system_error>
#include <unordered_map>

// MapLocator.h — finds the VPK holding a map's collision in a CS2 install:
// an official map (game/csgo/maps/<map>.vpk) or a workshop item (in the same
// Steam library's workshop/content/730, its VPK nesting maps/<map>.vpk).
namespace s2ghtline {

    struct MapSource {
        enum class Kind { Official, Workshop };

        Kind                            kind = Kind::Official;
        std::filesystem::path           vpk;
        std::string                     map;          // the name inside the VPK: maps/<map>/...
        std::string                     workshopId;   // workshop items
        uintmax_t                       size = 0;
        std::filesystem::file_time_type modified{};

        // Same file, unchanged since: a world built from it can be reused.
        [[nodiscard]] bool sameAs(const MapSource& o) const {
            return vpk == o.vpk && map == o.map && size == o.size && modified == o.modified;
        }
    };

    class MapLocator {
    public:
        // `csgoDir` = <install>/game/csgo.
        explicit MapLocator(std::filesystem::path csgoDir) : m_csgo(std::move(csgoDir)) {
            // <library>/steamapps/common/<game>/game/csgo -> <library>/steamapps
            const auto steamapps = m_csgo.parent_path().parent_path().parent_path().parent_path();
            m_workshop = steamapps / "workshop" / "content" / "730";
        }

        [[nodiscard]] const std::filesystem::path& csgoDir() const { return m_csgo; }

        [[nodiscard]] std::optional<MapSource> find(std::string_view gameMapName, std::string& why) {
            const Name name = parseName(gameMapName);
            if (name.map.empty()) {
                why = "no map name";
                return std::nullopt;
            }

            if (name.workshopId.empty()) {
                const auto official = m_csgo / "maps" / (name.map + ".vpk");
                if (auto s = describe(MapSource::Kind::Official, official, name.map, {}))
                    return s;
            }
            else if (auto s = findInItem(name.workshopId, name.map)) {
                return s;
            }

            if (auto s = findInWorkshop(name.map))
                return s;
            why = "map file not found for " + std::string(gameMapName);
            return std::nullopt;
        }

    private:
        static constexpr auto kRescanInterval = std::chrono::seconds(10);

        struct Name {
            std::string map;
            std::string workshopId;
        };

        struct Indexed {
            std::filesystem::path vpk;
            std::string           id;
        };

        std::filesystem::path                    m_csgo, m_workshop;
        std::unordered_map<std::string, Indexed> m_index;   // map name -> workshop item VPK
        std::optional<std::chrono::steady_clock::time_point> m_indexedAt;

        // "de_mirage", "maps/de_mirage.vpk", "workshop/123456789/some_map" -> map + workshop id.
        [[nodiscard]] static Name parseName(std::string_view raw) {
            std::string s(raw);
            for (char& ch : s) {
                ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
                if (ch == '\\')
                    ch = '/';
            }
            Name name;
            static const std::regex workshop(R"(^workshop/(\d+)/(.+)$)");
            std::smatch m;
            if (std::regex_match(s, m, workshop)) {
                name.workshopId = m[1].str();
                s = m[2].str();
            }
            if (s.starts_with("maps/"))
                s = s.substr(5);
            for (std::string_view ext : { ".vpk", ".bsp" })
                if (s.ends_with(ext))
                    s.resize(s.size() - ext.size());
            name.map = s.substr(s.find_last_of('/') + 1);   // npos + 1 = 0
            return name;
        }

        [[nodiscard]] static std::optional<MapSource> describe(MapSource::Kind kind, const std::filesystem::path& vpk,
                                                               const std::string& map, const std::string& id) {
            std::error_code ec;
            if (!std::filesystem::is_regular_file(vpk, ec))
                return std::nullopt;
            MapSource s;
            s.kind       = kind;
            s.vpk        = vpk;
            s.map        = map;
            s.workshopId = id;
            s.size       = std::filesystem::file_size(vpk, ec);
            s.modified   = std::filesystem::last_write_time(vpk, ec);
            return s;
        }

        // The VPK files of one workshop item: <id>.vpk, or <id>_dir.vpk (not the _NNN archives).
        template <class F>
        static void forEachItemVpk(const std::filesystem::path& item, F&& f) {
            std::error_code ec;
            static const std::regex archive(R"(_\d{3}\.vpk$)");
            for (const auto& e : std::filesystem::directory_iterator(item, ec)) {
                const std::string file = e.path().filename().string();
                if (e.is_regular_file(ec) && file.ends_with(".vpk") && !std::regex_search(file, archive))
                    f(e.path());
            }
        }

        // Map names inside one item's VPK (maps/<name>.vpk, skybox VPKs aside).
        template <class F>
        static void forEachNestedMap(const std::filesystem::path& vpkPath, F&& f) {
            std::string why;
            const auto vpk = Vpk::open(vpkPath, why);
            if (!vpk)
                return;
            vpk->forEachPath([&](std::string_view p) {
                if (!p.starts_with("maps/") || !p.ends_with(".vpk") || p.find('/', 5) != std::string_view::npos)
                    return;
                const std::string_view name = p.substr(5, p.size() - 9);
                if (!name.ends_with("_3dsky"))
                    f(std::string(name));
            });
        }

        [[nodiscard]] std::optional<MapSource> findInItem(const std::string& id, const std::string& map) const {
            std::optional<MapSource> found;
            forEachItemVpk(m_workshop / id, [&](const std::filesystem::path& vpk) {
                forEachNestedMap(vpk, [&](const std::string& name) {
                    if (!found && name == map)
                        found = describe(MapSource::Kind::Workshop, vpk, map, id);
                });
            });
            return found;
        }

        [[nodiscard]] std::optional<MapSource> findInWorkshop(const std::string& map) {
            const auto now = std::chrono::steady_clock::now();
            auto it = m_index.find(map);
            if (it == m_index.end() && (!m_indexedAt || now - *m_indexedAt >= kRescanInterval)) {
                indexWorkshop();
                m_indexedAt = now;
                it = m_index.find(map);
            }
            if (it == m_index.end())
                return std::nullopt;
            return describe(MapSource::Kind::Workshop, it->second.vpk, map, it->second.id);
        }

        void indexWorkshop() {
            m_index.clear();
            std::error_code ec;
            for (const auto& item : std::filesystem::directory_iterator(m_workshop, ec)) {
                if (!item.is_directory(ec))
                    continue;
                const std::string id = item.path().filename().string();
                forEachItemVpk(item.path(), [&](const std::filesystem::path& vpk) {
                    forEachNestedMap(vpk, [&](const std::string& name) { m_index.try_emplace(name, Indexed{ vpk, id }); });
                });
            }
        }
    };

} // namespace s2ghtline
