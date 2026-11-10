#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string_view>

// SurfaceNames.h — surface property (material) names from the hashes a
// map's physics stores (Geometry::surfaceHashes): MurmurHash2 of the
// lowercase name, seed 0x31415926 (checked: "default" = 1977497166, as the
// collision group "Default" stores it).  The names below are common CS2
// surface properties, for readable output; the full list is the game's
// surface properties file, which SurfaceTable reads (with names).
namespace s2ghtline {

    [[nodiscard]] constexpr uint32_t surfaceHash(std::string_view name) {
        constexpr uint32_t m = 0x5bd1e995;
        const auto lower = [](char ch) {
            return static_cast<uint8_t>(ch >= 'A' && ch <= 'Z' ? ch - 'A' + 'a' : ch);
        };
        uint32_t h = 0x31415926u ^ static_cast<uint32_t>(name.size());
        size_t i = 0;
        for (; name.size() - i >= 4; i += 4) {
            uint32_t k = lower(name[i]) | (lower(name[i + 1]) << 8) | (lower(name[i + 2]) << 16) |
                         (static_cast<uint32_t>(lower(name[i + 3])) << 24);
            k *= m;
            k ^= k >> 24;
            k *= m;
            h = h * m ^ k;
        }
        switch (name.size() - i) {
        case 3: h ^= static_cast<uint32_t>(lower(name[i + 2])) << 16; [[fallthrough]];
        case 2: h ^= static_cast<uint32_t>(lower(name[i + 1])) << 8;  [[fallthrough]];
        case 1: h ^= lower(name[i]); h *= m;
        }
        h ^= h >> 13;
        h *= m;
        h ^= h >> 15;
        return h;
    }
    static_assert(surfaceHash("default") == 1977497166u);

    // The name of a known surface property hash (for readable output).
    [[nodiscard]] inline std::optional<std::string_view> knownSurfaceName(uint32_t hash) {
        static constexpr std::array<std::string_view, 48> kNames{
            "default", "concrete", "concrete_block", "brick", "rock", "stone", "gravel", "dirt", "mud", "sand",
            "grass", "foliage", "snow", "ice", "water", "slime", "tile", "ceramic", "plaster", "drywall",
            "sheetrock", "carpet", "cloth", "fabric", "rubber", "rubbertire", "plastic", "plastic_barrel",
            "cardboard", "paper", "computer", "glass", "glassbottle", "metal", "metalpanel", "metalgrate",
            "metalvent", "metalvehicle", "metal_box", "metal_barrel", "metal_sand_barrel", "chainlink", "wood",
            "wood_box", "wood_crate", "wood_plank", "wood_panel", "wood_solid",
        };
        for (std::string_view n : kNames)
            if (surfaceHash(n) == hash)
                return n;
        return std::nullopt;
    }

} // namespace s2ghtline
