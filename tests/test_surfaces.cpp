// test_surfaces: SurfaceTable on generated files (support/Fixtures.h) -
// inheritance (self, base, base's base, then "default"), names, unknown
// surfaces - and the text reader and the vsurf reader on malformed input.
// test_surfaces_maps does the same on the installed game's files.
#include "support/Check.h"
#include "support/Fixtures.h"

#include "SurfaceProperties.h"

#include <cmath>
#include <cstdio>
#include <string>

using namespace s2ghtline;
using check::expect;
using fixtures::Bytes;
using fixtures::Kv;

namespace {

    bool approx(float a, float b) { return std::fabs(a - b) < 1e-6f; }

    void testResolution() {
        check::section("resolution");
        const Bytes vsurf = fixtures::surfaceVsurfFile();
        std::string why;
        const auto t = SurfaceTable::parse(ByteView(vsurf), fixtures::kSurfaceGameText, why);
        if (!expect(t.has_value(), "parses")) {
            std::printf("    (%s)\n", why.c_str());
            return;
        }
        expect(t->size() == 5, "five surfaces");
        const auto& def   = t->get(surfaceHash("default"));
        const auto& metal = t->get(surfaceHash("metal"));
        const auto& grate = t->get(surfaceHash("metalgrate"));
        const auto& conc  = t->get(surfaceHash("concrete"));
        expect(def.type == 'C' && approx(def.penetration, 0.5f) && approx(def.damage, 0.5f), "default: C, 0.5, 0.5");
        expect(metal.type == 'M' && approx(metal.penetration, 0.4f) && approx(metal.damage, 0.3f), "metal: its own values");
        expect(grate.type == 'G' && approx(grate.penetration, 0.4f) && approx(grate.damage, 0.3f),
               "metalgrate: its own type, the rest from its base (metal)");
        expect(conc.type == 'C' && approx(conc.penetration, 0.25f) && approx(conc.damage, 0.5f),
               "concrete: its own penetration, the rest from default");
        expect(!t->knows(surfaceHash("no_such_surface")) && &t->get(surfaceHash("no_such_surface")) == &t->defaults(),
               "unknown surface: default's values");
        expect(t->name(surfaceHash("metalgrate")) == "metalgrate" && t->name(12345).empty(), "names by hash");
        expect(gameMaterialType("12") == ('1' | ('2' << 8)) && gameMaterialType("") == 0, "gamematerial packing");
    }

    void testMalformed() {
        check::section("malformed input");
        const Bytes vsurf = fixtures::surfaceVsurfFile();
        for (const char* bad : { "", "{", "{ SurfacePropertiesList = [ { surfacePropertyName = \"x\" ", "{ a = }",
                                 "{ SurfacePropertiesList = [ { surfacePropertyName = \"x\" } ] }",   // no default
                                 "{ SurfacePropertiesList = { } }", "{ x = \"unterminated }", "<!-- unterminated" }) {
            std::string why;
            const auto t = SurfaceTable::parse(ByteView(vsurf), bad, why);
            char name[128];
            std::snprintf(name, sizeof(name), "rejected: %.40s", bad);
            expect(!t && !why.empty(), name);
        }
        std::string why;
        const auto minimal = SurfaceTable::parse(
            ByteView(vsurf), "<!-- kv3 --> { SurfacePropertiesList = [ { surfacePropertyName = \"default\" gamematerial = \"C\" } ] }", why);
        expect(minimal && approx(minimal->get(surfaceHash("metal")).penetration, 0.5f),
               "a minimal file: everything from default (built-in values where it sets none)");

        const Bytes empty;
        expect(!SurfaceTable::parse(ByteView(empty), fixtures::kSurfaceGameText, why) && !why.empty(), "no vsurf: rejected");
        const Bytes noList = fixtures::resourceFile({ { fourCC("DATA"), fixtures::encodeKv3(Kv::object({}), 5, fixtures::Compression::None) } });
        expect(!SurfaceTable::parse(ByteView(noList), fixtures::kSurfaceGameText, why) && !why.empty(),
               "vsurf without surfaces: rejected");
        Bytes truncated = vsurf;
        truncated.resize(truncated.size() / 2);
        expect(!SurfaceTable::parse(ByteView(truncated), fixtures::kSurfaceGameText, why) && !why.empty(),
               "truncated vsurf: rejected");
    }

} // namespace

int main() {
    testResolution();
    testMalformed();
    return check::result();
}
