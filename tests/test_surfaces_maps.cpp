// test_surfaces_maps: SurfaceTable from the installed game's pak01 - a few
// known surfaces' resolved values.
//   test_surfaces_maps <names.txt> <out.txt>   also writes, for each surface
//   name in names.txt (one per line), "name penetration damage type known",
//   for comparison with another reader of the same files.
// Skips without a CS2 install (support/Cs2Install.h).
#include "support/Check.h"
#include "support/Cs2Install.h"

#include "SurfaceProperties.h"
#include "Vpk.h"

#include <cstdio>
#include <fstream>
#include <string>

using namespace s2ghtline;
using check::expect;

int main(int argc, char** argv) {
    std::string why;
    const auto csgo = cs2::find(why);
    if (!csgo)
        return check::noGame(why, cs2::named());
    std::printf("CS2: %s\n", csgo->string().c_str());

    const auto pak = Vpk::open(*csgo / "pak01_dir.vpk", why);
    const auto vsurf = pak ? pak->read("surfaceproperties/surfaceproperties.vsurf_c", why) : std::nullopt;
    const auto game  = vsurf ? pak->read("scripts/surfaceproperties_game.txt", why) : std::nullopt;
    const auto table = game ? SurfaceTable::parse(ByteView(*vsurf),
                                                  std::string_view(reinterpret_cast<const char*>(game->data()), game->size()), why)
                            : std::nullopt;
    if (!expect(table.has_value(), "the game's surface table parses")) {
        std::printf("    (%s)\n", why.c_str());
        return check::result();
    }
    std::printf("  %zu surfaces\n", table->size());

    const auto show = [&](const char* name) {
        const auto& p  = table->get(surfaceHash(name));
        const char  t0 = static_cast<char>(p.type & 0xFF), t1 = static_cast<char>(p.type >> 8);
        std::printf("  %-16s pen %.2f  dmg %.2f  type %c%c%s\n", name, p.penetration, p.damage, t0, t1 ? t1 : ' ',
                    table->knows(surfaceHash(name)) ? "" : "   (unknown: default)");
        return p;
    };
    const auto def   = show("default");
    const auto metal = show("metal");
    const auto grate = show("metalgrate");
    const auto wood  = show("Wood_Plank");
    const auto glass = show("glass");
    show("chainlink");
    show("concrete");
    show("brick");
    show("plaster_drywall");
    show("no_such_surface");
    expect(def.type == 'C' && def.penetration == 0.5f, "default: C, 0.5");
    expect(metal.penetration == 0.4f && metal.type == 'M', "metal: its own 0.4, the type from solidmetal (M)");
    expect(grate.type == 'G', "metal grate: G");
    expect(wood.type == 'W', "wood plank: W (from Wood)");
    expect(glass.type == 'Y', "glass: Y");
    expect(table->size() > 50 && !table->knows(surfaceHash("no_such_surface")), "many surfaces; unknown ones aren't known");

    if (argc > 2) {
        std::ifstream in(argv[1]);
        std::ofstream out(argv[2]);
        for (std::string name; in && out && std::getline(in, name);) {
            while (!name.empty() && (name.back() == '\n' || name.back() == '\r'))
                name.pop_back();
            const auto& p = table->get(surfaceHash(name));
            char line[512];
            std::snprintf(line, sizeof(line), "%s %.4f %.4f %u %d\n", name.c_str(), p.penetration, p.damage,
                          static_cast<unsigned>(p.type), table->knows(surfaceHash(name)) ? 1 : 0);
            out << line;
        }
    }
    return check::result();
}
