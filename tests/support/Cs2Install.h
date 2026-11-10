#pragma once

// Cs2Install.h — finds an installed CS2 for the real-map tests and the
// example.  Order:
//   1. S2GHTLINE_CS2_DIR: the game's folder ("Counter-Strike Global
//      Offensive") or its game/csgo folder.
//   2. Every Steam library listed in Steam's libraryfolders.vdf, Steam itself
//      found in its default places (Program Files on Windows; ~/.steam,
//      ~/.local/share/Steam and the Flatpak folder elsewhere).
// Returns the game/csgo folder (holding pak01_dir.vpk and maps/);
// workshopMaps() lists the workshop maps installed beside it.

#include "Vpk.h"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <optional>
#include <regex>
#include <sstream>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace cs2 {

    namespace fs = std::filesystem;

    inline std::optional<std::string> env(const char* name) {
#ifdef _MSC_VER
        char*  value = nullptr;
        size_t size  = 0;
        if (_dupenv_s(&value, &size, name) != 0 || !value)
            return std::nullopt;
        std::string s(value);
        std::free(value);
#else
        const char* value = std::getenv(name);
        if (!value)
            return std::nullopt;
        std::string s(value);
#endif
        if (s.empty())
            return std::nullopt;
        return s;
    }

    // S2GHTLINE_CS2_DIR is set (find() then looks nowhere else).
    inline bool named() { return env("S2GHTLINE_CS2_DIR").has_value(); }

    inline bool isCsgoDir(const fs::path& p) {
        std::error_code ec;
        return fs::is_regular_file(p / "pak01_dir.vpk", ec) && fs::is_directory(p / "maps", ec);
    }

    // A game folder or its game/csgo folder -> game/csgo.
    inline std::optional<fs::path> csgoDirOf(const fs::path& p) {
        if (isCsgoDir(p))
            return p;
        if (isCsgoDir(p / "game" / "csgo"))
            return p / "game" / "csgo";
        return std::nullopt;
    }

    inline std::vector<fs::path> steamRoots() {
        std::vector<fs::path> roots;
#ifdef _WIN32
        for (const char* var : { "ProgramFiles(x86)", "ProgramFiles" })
            if (const auto p = env(var))
                roots.push_back(fs::path(*p) / "Steam");
#else
        if (const auto home = env("HOME")) {
            roots.push_back(fs::path(*home) / ".steam" / "steam");
            roots.push_back(fs::path(*home) / ".local" / "share" / "Steam");
            roots.push_back(fs::path(*home) / ".var" / "app" / "com.valvesoftware.Steam" / ".local" / "share" / "Steam");
        }
#endif
        return roots;
    }

    // A Steam root and every library its libraryfolders.vdf lists.
    inline std::vector<fs::path> steamLibraries(const fs::path& root) {
        std::vector<fs::path> libs{ root };
        std::ifstream in(root / "steamapps" / "libraryfolders.vdf");
        if (!in)
            return libs;
        std::stringstream text;
        text << in.rdbuf();
        const std::string s = text.str();
        static const std::regex path(R"re("path"\s+"([^"]+)")re");
        for (std::sregex_iterator it(s.begin(), s.end(), path), end; it != end; ++it) {
            std::string p = (*it)[1].str();
            for (size_t i = p.find("\\\\"); i != std::string::npos; i = p.find("\\\\", i + 1))
                p.erase(i, 1);   // the file escapes backslashes
            libs.emplace_back(p);
        }
        return libs;
    }

    // The game/csgo folder, or nothing (`why` says where it looked).
    inline std::optional<fs::path> find(std::string& why) {
        if (const auto dir = env("S2GHTLINE_CS2_DIR")) {
            if (const auto csgo = csgoDirOf(*dir))
                return csgo;
            why = "S2GHTLINE_CS2_DIR=" + *dir + " is not a CS2 install (no game/csgo/pak01_dir.vpk)";
            return std::nullopt;
        }
        for (const fs::path& root : steamRoots())
            for (const fs::path& lib : steamLibraries(root))
                if (const auto csgo = csgoDirOf(lib / "steamapps" / "common" / "Counter-Strike Global Offensive"))
                    return csgo;
        why = "no CS2 install found: set S2GHTLINE_CS2_DIR to the game's folder";
        return std::nullopt;
    }

    // <library>/steamapps/workshop/content/730 for an install's game/csgo folder.
    inline fs::path workshopDir(const fs::path& csgo) {
        return csgo.parent_path().parent_path().parent_path().parent_path() / "workshop" / "content" / "730";
    }

    // Every installed workshop map: {item id, map name}, by item.
    inline std::vector<std::pair<std::string, std::string>> workshopMaps(const fs::path& csgo) {
        std::vector<std::pair<std::string, std::string>> out;
        std::error_code ec;
        static const std::regex archive(R"(_\d{3}\.vpk$)");
        for (const auto& item : fs::directory_iterator(workshopDir(csgo), ec)) {
            if (!item.is_directory(ec))
                continue;
            for (const auto& f : fs::directory_iterator(item.path(), ec)) {
                const std::string name = f.path().filename().string();
                if (!name.ends_with(".vpk") || std::regex_search(name, archive))
                    continue;
                std::string why;
                const auto vpk = s2ghtline::Vpk::open(f.path(), why);
                if (!vpk)
                    continue;
                vpk->forEachPath([&](std::string_view p) {
                    if (p.starts_with("maps/") && p.ends_with(".vpk") && p.find('/', 5) == std::string_view::npos &&
                        !p.ends_with("_3dsky.vpk"))
                        out.emplace_back(item.path().filename().string(), std::string(p.substr(5, p.size() - 9)));
                });
            }
        }
        std::sort(out.begin(), out.end());
        return out;
    }

} // namespace cs2
