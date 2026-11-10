// test_kv3_maps: the KV3 decoder + physics parser on real PHYS blocks from
// the installed game - an official map (v5, zstd) and the installed workshop
// maps (LZ4: no checksum, so corruption reaches the tree parser) - and
// mutated copies of them, which must fail cleanly.  Run it under ASan / UBSan
// too.  Workshop maps that don't parse are listed with the reason; at least
// one must parse.  Skips without a CS2 install (support/Cs2Install.h).
#include "support/Check.h"
#include "support/Cs2Install.h"

#include "MapLocator.h"
#include "MapPhysics.h"

#include <cstdio>
#include <random>

using namespace s2ghtline;
using check::expect;

namespace {

    std::optional<std::vector<uint8_t>> physBlock(const MapSource& source) {
        std::string why;
        const auto vpk  = openMapVpk(source.vpk, source.map, why);
        const auto file = vpk ? vpk->read(physicsModelPath(source.map), why) : std::nullopt;
        const auto block = file ? findBlock(ByteView(*file), fourCC("PHYS")) : std::nullopt;
        if (!block) {
            std::printf("  %s: no PHYS block (%s)\n", source.map.c_str(), why.c_str());
            return std::nullopt;
        }
        return std::vector<uint8_t>(block->data(), block->data() + block->size());
    }

    bool tryParse(const std::vector<uint8_t>& bytes) {
        std::string why;
        const auto doc = Kv3Doc::parse(ByteView(bytes), why);
        return doc && PhysParser::parse(doc->root(), why).has_value();
    }

    void fuzz(const char* name, const std::vector<uint8_t>& original, int rounds) {
        std::mt19937 rng(12345);
        int parsed = 0, rejected = 0;
        for (int r = 0; r < rounds; ++r) {
            std::vector<uint8_t> bytes = original;
            switch (r % 4) {
            case 0: {   // flip a few bytes anywhere
                const int flips = 1 + static_cast<int>(rng() % 8);
                for (int i = 0; i < flips; ++i)
                    bytes[rng() % bytes.size()] ^= static_cast<uint8_t>(1 + rng() % 255);
                break;
            }
            case 1:     // truncate
                bytes.resize(rng() % bytes.size());
                break;
            case 2: {   // corrupt the header (sizes, counts)
                const size_t header = std::min<size_t>(bytes.size(), 120);
                bytes[rng() % header] ^= static_cast<uint8_t>(1 + rng() % 255);
                break;
            }
            case 3: {   // overwrite a run with random bytes
                const size_t at = rng() % bytes.size();
                const size_t n  = std::min<size_t>(bytes.size() - at, 1 + rng() % 64);
                for (size_t i = 0; i < n; ++i)
                    bytes[at + i] = static_cast<uint8_t>(rng());
                break;
            }
            }
            (tryParse(bytes) ? parsed : rejected)++;
        }
        std::printf("  fuzz %s: %d rounds, %d parsed, %d rejected, no crash\n", name, rounds, parsed, rejected);
    }

} // namespace

int main() {
    std::string why;
    const auto csgo = cs2::find(why);
    if (!csgo)
        return check::noGame(why, cs2::named());
    std::printf("CS2: %s\n", csgo->string().c_str());

    MapLocator locator(*csgo);
    check::section("real PHYS blocks");
    const auto mirage = locator.find("de_mirage", why);
    const auto zstd   = mirage ? physBlock(*mirage) : std::nullopt;
    expect(zstd && tryParse(*zstd), "de_mirage (v5 zstd) parses");

    // Workshop maps: the first that parses is fuzzed below.
    std::optional<std::vector<uint8_t>> lz4;
    std::string                         lz4Map;
    const auto workshop = cs2::workshopMaps(*csgo);
    int parsedMaps = 0;
    for (const auto& [id, map] : workshop) {
        const auto source = locator.find("workshop/" + id + "/" + map, why);
        const auto block  = source ? physBlock(*source) : std::nullopt;
        std::string reason = source ? "no PHYS block" : why;
        const auto  doc = block ? Kv3Doc::parse(ByteView(*block), reason) : std::nullopt;
        const auto  geo = doc ? PhysParser::parse(doc->root(), reason) : std::nullopt;
        if (geo) {
            ++parsedMaps;
            if (!lz4) {
                lz4    = block;
                lz4Map = map;
            }
        }
        else {
            std::printf("  note: workshop map %s (%s) doesn't parse: %s\n", map.c_str(), id.c_str(), reason.c_str());
        }
    }
    if (workshop.empty())
        std::printf("  skipped: no workshop maps installed\n");
    else
        expect(parsedMaps > 0, "installed workshop maps (LZ4) parse - at least one");
    std::printf("  %d / %zu workshop maps parse\n", parsedMaps, workshop.size());

    check::section("mutations");
    if (lz4)
        fuzz(lz4Map.c_str(), *lz4, 4000);
    if (zstd)
        fuzz("de_mirage", *zstd, 400);
    return check::result();
}
