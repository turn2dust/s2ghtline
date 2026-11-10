#pragma once

#include "Kv3.h"
#include "PhysParser.h"
#include "Resource.h"
#include "Vpk.h"

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>

// MapPhysics.h — a map's collision geometry from its VPK.
//
// Official maps: game/csgo/maps/<map>.vpk holds maps/<map>/world_physics.vmdl_c.
// Workshop items: the item's VPK holds the map's own VPK (maps/<map>.vpk),
// which is read into memory and opened from there.
// The collision is the vmdl_c's PHYS block; a few maps also ship a
// world_physics.vphys_c (DATA block), used only when there's no vmdl_c.
namespace s2ghtline {

    [[nodiscard]] inline std::string physicsModelPath(std::string_view map) {
        return "maps/" + std::string(map) + "/world_physics.vmdl_c";
    }
    [[nodiscard]] inline std::string physicsDataPath(std::string_view map) {
        return "maps/" + std::string(map) + "/world_physics.vphys_c";
    }

    // The VPK holding the map's physics: the file itself, or the map VPK nested in it.
    [[nodiscard]] inline std::optional<Vpk> openMapVpk(const std::filesystem::path& file, std::string_view map,
                                                       std::string& why) {
        auto vpk = Vpk::open(file, why);
        if (!vpk)
            return std::nullopt;
        if (vpk->contains(physicsModelPath(map)) || vpk->contains(physicsDataPath(map)))
            return vpk;

        const std::string nested = "maps/" + std::string(map) + ".vpk";
        if (!vpk->contains(nested)) {
            why = "no physics for " + std::string(map) + " in " + file.filename().string();
            return std::nullopt;
        }
        auto bytes = vpk->read(nested, why);
        if (!bytes)
            return std::nullopt;
        return Vpk::fromMemory(std::move(*bytes), why);
    }

    [[nodiscard]] inline std::optional<Geometry> loadMapGeometry(const Vpk& vpk, std::string_view map,
                                                                 std::string& why) {
        const bool model = vpk.contains(physicsModelPath(map));
        const auto file  = vpk.read(model ? physicsModelPath(map) : physicsDataPath(map), why);
        if (!file)
            return std::nullopt;

        const auto block = findBlock(ByteView(*file), model ? fourCC("PHYS") : fourCC("DATA"));
        if (!block) {
            why = "no physics block in the map's physics file";
            return std::nullopt;
        }
        const auto doc = Kv3Doc::parse(*block, why);
        if (!doc)
            return std::nullopt;
        return PhysParser::parse(doc->root(), why);
    }

} // namespace s2ghtline
