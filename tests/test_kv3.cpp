// test_kv3: the KV3 decoder and the physics parser on hand-built input -
// every value encoding, versions 2-5, uncompressed / LZ4 / zstd - then
// corrupted copies of it, which must fail cleanly (nullopt), never crash or
// read out of bounds.  Run it under ASan / UBSan (the asan preset).
#include "support/Check.h"
#include "support/Fixtures.h"

#include "MapPhysics.h"

#include <cmath>
#include <cstdio>
#include <random>
#include <string>

using namespace s2ghtline;
using namespace s2ghtline::kv3_literals;
using fixtures::Bytes;
using fixtures::Compression;
using fixtures::Kv;
using check::expect;

namespace {

    constexpr Compression kCompressions[] = { Compression::None, Compression::Lz4, Compression::Zstd };

    std::optional<Kv3Doc> decode(const Bytes& bytes, std::string& why) { return Kv3Doc::parse(ByteView(bytes), why); }

    // Decode + parse; true if it produced geometry.
    bool tryParse(const Bytes& bytes) {
        std::string why;
        const auto doc = decode(bytes, why);
        return doc && PhysParser::parse(doc->root(), why).has_value();
    }

    // Every kind of value, in every lane.
    Kv valueDoc() {
        Bytes big(40000);   // > 2 LZ4 frames
        for (size_t i = 0; i < big.size(); ++i)
            big[i] = static_cast<uint8_t>((i * 7) ^ (i >> 5));
        std::vector<Kv> deep = { Kv::integer(42) };
        Kv nested = Kv::array(deep);
        for (int i = 0; i < 20; ++i)
            nested = Kv::array({ nested });
        return Kv::object({
            { "null", Kv::null() },
            { "yes", Kv::boolean(true) }, { "no", Kv::boolean(false) },
            { "zero", Kv::integer(0) }, { "one", Kv::integer(1) },
            { "i8", Kv::integer(-100) }, { "i16", Kv::integer(-30000) }, { "i32", Kv::integer(-2000000000) },
            { "i64", Kv::integer(-9000000000000000000ll) },
            { "u8", Kv::uinteger(200) }, { "u16", Kv::uinteger(60000) }, { "u32", Kv::uinteger(4000000000u) },
            { "u64", Kv::uinteger(18000000000000000000ull) },
            { "d0", Kv::real(0.0) }, { "d1", Kv::real(1.0) }, { "pi", Kv::real(3.141592653589793) },
            { "f", Kv::f32(-2.5f) },
            { "s", Kv::str("hello") }, { "empty", Kv::str("") },
            { "blob", Kv::bytes({ 1, 2, 3, 4, 5 }) }, { "emptyBlob", Kv::bytes({}) }, { "bigBlob", Kv::bytes(big) },
            { "array", Kv::array({ Kv::integer(7), Kv::str("x"), Kv::real(0.5), Kv::array({}) }) },
            { "typed4", Kv::typedArray({ Kv::f32(1.f), Kv::f32(2.f), Kv::f32(3.f) }) },
            { "typed1", Kv::typedArray({ Kv::integer(-1), Kv::integer(5) }, Kv::Typed::Count1) },
            { "typedAux", Kv::typedArray({ Kv::real(0.25), Kv::real(-8.0) }, Kv::Typed::Auxiliary) },
            { "typedStr", Kv::typedArray({ Kv::str("a"), Kv::str("b") }) },
            { "typedBool", Kv::typedArray({ Kv::boolean(true), Kv::boolean(false), Kv::boolean(true) }) },
            { "nested", nested },
            { "object", Kv::object({ { "inner", Kv::str("hello") } }) },
        });
    }

    bool sameBytes(std::optional<ByteView> got, const Bytes& want) {
        return got && got->size() == want.size() && std::equal(want.begin(), want.end(), got->data());
    }

    void testFixed() {
        check::section("fixed cases");
        expect(!tryParse({}), "empty input rejected");
        expect(!tryParse({ 0x05, 0x33, 0x56, 0x4B }), "magic only rejected");
        expect(!tryParse({ 0x56, 0x4B, 0x56, 0x03, 0, 0, 0, 0 }), "legacy VKV3 rejected");
        expect(!tryParse({ 0x09, 0x33, 0x56, 0x4B, 0, 0, 0, 0 }), "unknown version rejected");
        expect(!tryParse({ 0x01, 0x33, 0x56, 0x4B, 0, 0, 0, 0 }), "version 1 rejected");
    }

    void testValues(int version, Compression c) {
        char name[96];
        std::snprintf(name, sizeof(name), "v%d %s: every value round-trips", version, toString(c));
        std::string why;
        const auto doc = decode(fixtures::encodeKv3(valueDoc(), version, c), why);
        if (!doc) {
            std::printf("    (%s)\n", why.c_str());
            expect(false, name);
            return;
        }
        const Kv3Value r = doc->root();
        Bytes big(40000);
        for (size_t i = 0; i < big.size(); ++i)
            big[i] = static_cast<uint8_t>((i * 7) ^ (i >> 5));
        Kv3Value deep = r["nested"_kv];
        for (int i = 0; i < 21; ++i)
            deep = deep.at(0);

        const bool ok =
            r.isObject() && r.size() == 30 && r["null"_kv].valid() && r["null"_kv].type() == Kv3Type::Null &&
            r["yes"_kv].integer() == 1 && r["no"_kv].integer() == 0 && r["yes"_kv].type() == Kv3Type::Bool &&
            r["zero"_kv].integer() == 0 && r["one"_kv].integer() == 1 && r["i8"_kv].integer() == -100 &&
            r["i16"_kv].integer() == -30000 && r["i32"_kv].integer() == -2000000000 &&
            r["i64"_kv].integer() == -9000000000000000000ll &&
            r["u8"_kv].integer() == 200 && r["u16"_kv].integer() == 60000 && r["u32"_kv].integer() == 4000000000ll &&
            !r["u64"_kv].integer() && r["u64"_kv].number() == 18000000000000000000.0 &&   // doesn't fit int64
            r["d0"_kv].number() == 0.0 && r["d1"_kv].number() == 1.0 && r["pi"_kv].number() == 3.141592653589793 &&
            r["f"_kv].number() == -2.5 &&
            r["s"_kv].string() == "hello" && r["empty"_kv].string() == "" && r["object"_kv]["inner"_kv].string() == "hello" &&
            sameBytes(r["blob"_kv].blob(), { 1, 2, 3, 4, 5 }) && sameBytes(r["emptyBlob"_kv].blob(), {}) &&
            sameBytes(r["bigBlob"_kv].blob(), big) &&
            r["array"_kv].size() == 4 && r["array"_kv].at(0).integer() == 7 && r["array"_kv].at(1).string() == "x" &&
            r["array"_kv].at(2).number() == 0.5 && r["array"_kv].at(3).isArray() && r["array"_kv].at(3).size() == 0 &&
            r["typed4"_kv].size() == 3 && r["typed4"_kv].at(2).number() == 3.0 &&
            r["typed1"_kv].size() == 2 && r["typed1"_kv].at(0).integer() == -1 && r["typed1"_kv].at(1).integer() == 5 &&
            r["typedAux"_kv].size() == 2 && r["typedAux"_kv].at(1).number() == -8.0 &&
            r["typedStr"_kv].at(1).string() == "b" && r["typedBool"_kv].at(2).integer() == 1 &&
            deep.integer() == 42 &&
            // Misuse: wrong types and missing members are nothing, not a crash.
            !r["missing"_kv].valid() && !r["s"_kv].integer() && !r["pi"_kv].string() && !r["s"_kv].blob() &&
            !r["array"_kv].at(99).valid() && r["s"_kv].size() == 0 && !r["s"_kv]["x"_kv].valid();
        expect(ok, name);
    }

    void testPhysics(int version, Compression c) {
        char name[96];
        std::snprintf(name, sizeof(name), "v%d %s: the box world parses to the expected geometry", version, toString(c));
        std::string why;
        const auto doc = decode(fixtures::encodeKv3(fixtures::physicsDoc(), version, c), why);
        const auto geo = doc ? PhysParser::parse(doc->root(), why) : std::nullopt;
        if (!geo) {
            std::printf("    (%s)\n", why.c_str());
            expect(false, name);
            return;
        }
        using E = fixtures::PhysicsExpect;
        const auto& s = geo->stats;
        bool ok = s.hulls == E::hulls && s.meshes == E::meshes && s.spheres == E::spheres && s.capsules == E::capsules &&
                  s.hullTris == E::hullTris && s.meshTris == E::meshTris && s.primitiveTris == E::primitiveTris &&
                  s.degenerate == 0 && s.missingSurface == 0 && s.meshMaterialsIgnored == 0 && !s.bindPose &&
                  geo->tris.size() == E::tris && geo->layers.size() == E::layers &&
                  geo->surfaceHashes.size() == E::surfaces && geo->surfaceHashes[1] == surfaceHash("metal") &&
                  geo->layers[1].interactAs == std::vector<std::string>{ "passbullets" } &&
                  geo->layers[1].exclude == std::vector<std::string>{ "player" } && geo->layers[0].group == "default";
        // Hull A's 12 triangles on layer 0 / concrete, hull B's on layer 1 / metal;
        // mesh 1's two materials, mesh 2's glass; solids numbered in parse order.
        ok = ok && geo->tris[0].layer == 0 && geo->tris[0].surface == 0 && geo->tris[12].layer == 1 &&
             geo->tris[12].surface == 1 && geo->tris[24].surface == 0 && geo->tris[25].surface == 1 &&
             geo->tris[26].layer == 2 && geo->tris[27].surface == 2 && geo->tris[0].solid != geo->tris[12].solid &&
             geo->solidKinds[geo->tris[0].solid] == SolidKind::Hull && geo->solidKinds[geo->tris[24].solid] == SolidKind::Mesh &&
             geo->solidKinds[geo->tris.back().solid] == SolidKind::Primitive;
        expect(ok, name);
    }

    void testPhysicsErrors() {
        check::section("physics errors");
        const auto parseDoc = [](const Kv& doc, std::string& why) {
            const auto d = decode(fixtures::encodeKv3(doc, 5, Compression::None), why);
            return d ? PhysParser::parse(d->root(), why) : std::nullopt;
        };
        std::string why;
        expect(!parseDoc(Kv::object({}), why) && !why.empty(), "empty document: no collision layers");
        const Kv layers = Kv::array({ fixtures::layerAttributes({}) });
        expect(!parseDoc(Kv::object({ { "m_collisionAttributes", layers } }), why) && !why.empty(), "no parts");
        const auto withShape = [&](Kv shape) {
            return Kv::object({ { "m_collisionAttributes", layers },
                                { "m_parts", Kv::array({ Kv::object({ { "m_rnShape", std::move(shape) } }) }) } });
        };
        expect(!parseDoc(withShape(Kv::object({})), why) && why == "the physics data has no triangles",
               "a part with no shapes: no triangles");
        expect(!parseDoc(withShape(Kv::object({ { "m_hulls", Kv::array({ fixtures::hullShape(fixtures::kSolidCube, 5, 0, true) }) } })),
                         why) && why == "a shape refers to a missing collision layer",
               "a shape on a missing layer is rejected");
        const auto geo = parseDoc(withShape(Kv::object({ { "m_hulls", Kv::array({ fixtures::hullShape(fixtures::kSolidCube, 0, 9, true) }) } })), why);
        expect(geo && geo->stats.missingSurface == 1 && geo->tris[0].surface == Geometry::kNoSurface,
               "a missing surface: kept, without a surface");

        // Depth limit: 100 nested arrays.
        Kv deep = Kv::integer(1);
        for (int i = 0; i < 100; ++i)
            deep = Kv::array({ deep });
        expect(!decode(fixtures::encodeKv3(deep, 5, Compression::None), why) && !why.empty(),
               "nesting past the limit is rejected");

        // A degenerate triangle (zero area) is dropped.
        const Kv flat = Kv::object({ { "m_nCollisionAttributeIndex", Kv::integer(0) }, { "m_nSurfacePropertyIndex", Kv::integer(0) },
            { "m_Mesh", Kv::object({ { "m_Vertices", Kv::array({ Kv::vec3({ 0, 0, 0 }), Kv::vec3({ 1, 0, 0 }), Kv::vec3({ 2, 0, 0 }),
                                                                 Kv::vec3({ 0, 1, 0 }) }) },
                                     { "m_Triangles", Kv::array({ Kv::object({ { "m_nIndex", Kv::typedArray({ Kv::integer(0), Kv::integer(1), Kv::integer(2) }) } }),
                                                                  Kv::object({ { "m_nIndex", Kv::typedArray({ Kv::integer(0), Kv::integer(1), Kv::integer(3) }) } }) }) } }) } });
        const auto g2 = parseDoc(withShape(Kv::object({ { "m_meshes", Kv::array({ flat }) } })), why);
        expect(g2 && g2->stats.degenerate == 1 && g2->tris.size() == 1, "a zero-area triangle is dropped");
    }

    // Mutated copies: each must decode or fail - the sanitizers catch anything else.
    void fuzz(const char* name, const Bytes& original, int rounds, uint32_t seed) {
        std::mt19937 rng(seed);
        int parsed = 0, rejected = 0;
        for (int r = 0; r < rounds; ++r) {
            Bytes bytes = original;
            switch (r % 5) {
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
            case 4: {   // a header size field set to an extreme value
                const size_t field = 28 + 4 * (rng() % 20);
                if (field + 4 <= bytes.size()) {
                    const int32_t extremes[] = { -1, 0, 1, 0x7FFFFFFF, 1 << 30, 255, 65536 };
                    const int32_t v = extremes[rng() % std::size(extremes)];
                    std::memcpy(bytes.data() + field, &v, 4);
                }
                break;
            }
            }
            (tryParse(bytes) ? parsed : rejected)++;
        }
        std::printf("  fuzz %-12s %5d rounds: %4d parsed, %4d rejected, no crash\n", name, rounds, parsed, rejected);
    }

} // namespace

int main() {
    testFixed();

    check::section("round trips");
    for (int version = 2; version <= 5; ++version)
        for (const Compression c : kCompressions)
            testValues(version, c);
    for (int version = 2; version <= 5; ++version)
        for (const Compression c : kCompressions)
            testPhysics(version, c);
    testPhysicsErrors();

    check::section("mutations");
    uint32_t seed = 1;
    for (int version = 2; version <= 5; ++version) {
        for (const Compression c : kCompressions) {
            char name[32];
            std::snprintf(name, sizeof(name), "v%d %s", version, toString(c));
            fuzz(name, fixtures::encodeKv3(fixtures::physicsDoc(), version, c), 1500, seed++);
        }
    }
    return check::result();
}
