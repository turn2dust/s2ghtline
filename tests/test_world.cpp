// test_world: everything above the BVH, on generated data - layer rules,
// World (line of sight by layer class, > 64 layers, cancellation), segment
// contacts and winding on the box world (support/Fixtures.h), open geometry,
// surface lookup, and MapLocator / MapManager end to end on a fake game
// install written to a temporary folder (official map, workshop item with a
// nested map VPK, archive VPKs, load / switch / cancel / reuse / failure).
// test_world_maps does the install-dependent part on the real game.
#include "support/Check.h"
#include "support/Fixtures.h"

#include "MapManager.h"
#include "SurfaceNames.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <map>
#include <thread>
#include <unordered_map>

using namespace s2ghtline;
using check::expect;
using fixtures::Bytes;
using fixtures::Compression;
using fixtures::Kv;
using Clock = std::chrono::steady_clock;

namespace {

    Layer layer(std::vector<std::string> as, std::vector<std::string> ex = {}) {
        return Layer{ "g", std::move(as), std::move(ex) };
    }

    // A 200 x 200 wall in the plane x = `x`, two triangles on `layerIndex`.
    void addWall(Geometry& g, float x, uint16_t layerIndex) {
        const auto base = static_cast<uint32_t>(g.vertices.size());
        g.vertices.push_back({ x, -100.f, -100.f });
        g.vertices.push_back({ x,  100.f, -100.f });
        g.vertices.push_back({ x,  100.f,  100.f });
        g.vertices.push_back({ x, -100.f,  100.f });
        g.tris.push_back({ base, base + 1, base + 2, 0, layerIndex });
        g.tris.push_back({ base, base + 2, base + 3, 0, layerIndex });
    }

    bool approx(float a, float b, float eps = 1e-4f) { return std::fabs(a - b) <= eps; }

    void testRules() {
        check::section("layer rules");
        const LayerRules r;
        std::vector<std::string> unknown;
        expect(r.classify(layer({})) == LayerClass::Solid, "no tags = solid (world)");
        expect(r.classify(layer({ "npcclip", "playerclip" })) == LayerClass::Clip, "player / npc clip = clip");
        expect(r.classify(layer({ "csgo_grenadeclip" })) == LayerClass::Clip, "grenade clip = clip");
        expect(r.classify(layer({ "sky" })) == LayerClass::Clip, "sky = clip");
        expect(r.classify(layer({ "passbullets" })) == LayerClass::SeeThrough, "passbullets = see-through");
        expect(r.classify(layer({ "passbullets", "window" })) == LayerClass::SeeThrough, "passbullets + window = see-through");
        expect(r.classify(layer({ "water" })) == LayerClass::SeeThrough, "water = see-through");
        expect(r.classify(layer({ "ladder", "passbullets" })) == LayerClass::Clip, "ladder + passbullets = clip (clip wins)");
        expect(r.classify(layer({ "blocklight", "blocksound", "solid" }), &unknown) == LayerClass::Solid && unknown.empty(),
               "blocklight blocksound solid = solid, nothing unknown");
        expect(r.classify(layer({}, { "player" })) == LayerClass::Solid, "excludes don't matter (world !player = solid)");
        expect(r.classify(layer({ "mystery" }), &unknown) == LayerClass::Solid && unknown == std::vector<std::string>{ "mystery" },
               "unknown tag = solid, reported");
        (void)r.classify(layer({ "mystery" }), &unknown);
        expect(unknown.size() == 1, "an unknown tag is reported once");

        LayerRules custom;
        custom.seeThrough = { "foo" };
        expect(custom.classify(layer({ "foo" })) == LayerClass::SeeThrough && custom.classify(layer({ "window" })) == LayerClass::Solid,
               "a replaced list replaces the default");
        expect(custom.classify(layer({ "playerclip" })) == LayerClass::Clip, "the other lists keep their defaults");
    }

    void testWorld() {
        check::section("world: line of sight by class");
        Geometry g;
        g.layers = { layer({}), layer({ "passbullets" }), layer({ "playerclip" }) };
        g.surfaceHashes = { surfaceHash("concrete") };
        addWall(g, 100.f, 1);   // fence
        addWall(g, 200.f, 2);   // clip
        addWall(g, 300.f, 0);   // solid
        const auto w = WorldBuilder::build("test", std::move(g), LayerRules{});
        expect(w.has_value(), "builds");
        if (!w) return;
        const Vec3 eye{ 0, 0, 0 };
        expect(w->visible(eye, { 150, 0, 0 }), "through a fence: visible");
        expect(w->visible(eye, { 250, 0, 0 }), "through fence + clip: visible");
        expect(!w->visible(eye, { 350, 0, 0 }), "through the solid wall: blocked");
        expect(w->visible(eye, { 350, 150, 0 }), "past the wall's edge: visible");
        const auto hit = w->trace(eye, { 400, 0, 0 }, kAllLayers);
        expect(hit && w->classOfTri(hit->tri) == LayerClass::SeeThrough && w->layerOfTri(hit->tri) == 1,
               "trace (all classes): first hit is the fence, layer 1");
        const auto solidHit = w->trace(eye, { 400, 0, 0 }, w->classMask(LayerClass::Solid));
        expect(solidHit && std::fabs(solidHit->t * 400.f - 300.f) < 0.01f, "trace (solid only): the wall at x = 300");
        expect(hit && w->surfaceHashOfTri(hit->tri) == surfaceHash("concrete"), "surface hash of a triangle");
        expect(w->stats().trisByClass[0] == 2 && w->stats().trisByClass[1] == 2 && w->stats().trisByClass[2] == 2,
               "triangles per class");
        const auto blocker = w->blocker(eye, { 350, 0, 0 });
        expect(blocker && w->blocks(*blocker, eye, { 350, 0, 0 }) && !w->blocks(*blocker, eye, { 250, 0, 0 }) &&
               !w->blocks(1u << 30, eye, { 350, 0, 0 }),
               "blocker / blocks: the solid triangle in the way, any index safe");
        expect(w->surfaceNote() == "no surface table" && w->surfaceOfTri(0).penetration == SurfaceProps{}.penetration,
               "no surface table: defaults, with a note");

        check::section("world: more than 64 layers");
        Geometry many;
        for (int i = 0; i < 70; ++i)
            many.layers.push_back(i % 2 ? layer({ "passbullets" }) : layer({}));
        for (int i = 0; i < 70; ++i)
            addWall(many, 10.f + static_cast<float>(i) * 10.f, static_cast<uint16_t>(i));
        const auto m = WorldBuilder::build("many", std::move(many), LayerRules{});
        expect(m && !m->visible({ 0, 0, 0 }, { 15, 0, 0 }), "layer 0 (solid) blocks");
        expect(m && m->visible({ 15, 0, 0 }, { 25, 0, 0 }), "layer 1 (see-through) doesn't");
        bool sharedOk = m.has_value();
        for (int i = 61; m && i < 70; ++i) {
            // Wall i alone between the two points.
            const float x = 10.f + static_cast<float>(i) * 10.f;
            const bool blocks = !m->visible({ x - 5.f, 0, 0 }, { x + 5.f, 0, 0 });
            sharedOk = sharedOk && blocks == (i % 2 == 0);
        }
        expect(sharedOk, "layers 61+ share bits by class and still block / pass correctly");
        const auto shared = m ? m->trace({ 10.f + 65 * 10.f - 5.f, 0, 0 }, { 10.f + 65 * 10.f + 5.f, 0, 0 }, kAllLayers)
                              : std::nullopt;
        expect(shared && !m->layerOfTri(shared->tri) && m->classOfTri(shared->tri) == LayerClass::SeeThrough,
               "a shared bit: no single layer, class kept");

        check::section("world: cancellation");
        Geometry big;
        big.layers = { layer({}) };
        addWall(big, 1.f, 0);
        std::stop_source stop;
        stop.request_stop();
        expect(!WorldBuilder::build("big", std::move(big), LayerRules{}, stop.get_token()), "stopped build returns nothing");
    }

    std::optional<SurfaceTable> fixtureSurfaces(std::string& why) {
        const Bytes vsurf = fixtures::surfaceVsurfFile();
        return SurfaceTable::parse(ByteView(vsurf), fixtures::kSurfaceGameText, why);
    }

    std::optional<World> boxWorld() {
        std::string why;
        const auto doc = Kv3Doc::parse(ByteView(fixtures::encodeKv3(fixtures::physicsDoc(), 5, Compression::Zstd)), why);
        auto geo = doc ? PhysParser::parse(doc->root(), why) : std::nullopt;
        const auto surfaces = fixtureSurfaces(why);
        if (!geo || !surfaces) {
            std::printf("  box world: %s\n", why.c_str());
            return std::nullopt;
        }
        return WorldBuilder::build("box_world", std::move(*geo), LayerRules{}, {}, &*surfaces);
    }

    void testBoxWorld() {
        check::section("box world: classes, traces, surfaces");
        const auto w = boxWorld();
        expect(w.has_value(), "builds from the parsed physics, with the surface table");
        if (!w) return;
        expect(w->surfaceNote().empty(), "surface table used (no note)");
        expect(w->layerCount() == 3 && w->layerClass(0) == LayerClass::Solid && w->layerClass(1) == LayerClass::SeeThrough &&
               w->layerClass(2) == LayerClass::Clip, "layers: solid, see-through (passbullets), clip");
        expect(w->stats().unknownTags.empty() && w->stats().trisByClass[1] == 12 && w->stats().trisByClass[2] == 2,
               "stats: no unknown tags, 12 fence and 2 clip triangles");

        // Down onto the cube from above: the clip quad at z = 300 first, then the cube's top.
        const Vec3 above{ 30, 60, 500 }, below{ 30, 60, -100 };
        const auto first = w->trace(above, below, kAllLayers);
        expect(first && w->classOfTri(first->tri) == LayerClass::Clip && approx(first->t, 200.f / 600.f),
               "trace (all): the clip at z = 300 first");
        const auto top = w->trace(above, below, w->visionMask());
        expect(top && approx(top->t, 400.f / 600.f) && w->surfaceNameOfTri(top->tri) == "concrete",
               "trace (solid): the cube's top at z = 100, concrete");
        expect(first && w->surfaceNameOfTri(first->tri) == "glass" && approx(w->surfaceOfTri(first->tri).penetration, 0.99f) &&
               approx(w->surfaceOfTri(first->tri).damage, 0.5f), "the clip's glass: its own penetration, default damage");
        expect(top && approx(w->surfaceOfTri(top->tri).penetration, 0.25f) && w->surfaceOfTri(top->tri).type == 'C',
               "concrete: its own penetration, the type from default");
        expect(!w->visible({ -50, 40, 60 }, { 150, 40, 60 }), "the cube blocks line of sight");
        expect(w->visible({ 150, 40, 60 }, { 250, 40, 60 }), "the fence doesn't");

        // Floor materials: per triangle.
        const auto floorA = w->trace({ 10, -400, 0 }, { 10, -400, -20 }, kAllLayers);
        const auto floorB = w->trace({ -400, 10, 0 }, { -400, 10, -20 }, kAllLayers);
        expect(floorA && floorB && w->solidKind(w->bvh().tri(floorA->tri).solid) == SolidKind::Mesh &&
               w->surfaceNameOfTri(floorA->tri) != w->surfaceNameOfTri(floorB->tri),
               "a mesh's two triangles keep their own materials");

        check::section("box world: segment contacts");
        std::vector<World::SegmentContact> contacts;
        std::vector<Hit> scratch;
        // Off the faces' diagonals, so each face is crossed in one triangle.
        const Vec3 a{ -50, 40, 60 }, b{ 250, 40, 60 };
        w->segmentContacts(a, b, contacts, scratch);
        const bool four = contacts.size() == 4;
        expect(four, "through cube and fence: four surfaces");
        if (four) {
            expect(contacts[0].entering && !contacts[1].entering && contacts[2].entering && !contacts[3].entering,
                   "enter, leave, enter, leave");
            expect(approx(contacts[0].t, 50.f / 300.f) && approx(contacts[1].t, 150.f / 300.f) &&
                   approx(contacts[2].t, 250.f / 300.f) && approx(contacts[3].t, 252.f / 300.f), "at x = 0, 100, 200, 202");
            expect(contacts[0].solid == contacts[1].solid && contacts[2].solid == contacts[3].solid &&
                   contacts[0].solid != contacts[2].solid, "two solids, each entered and left");
            expect(!w->onPassBulletsLayer(contacts[0].tri) && w->onPassBulletsLayer(contacts[2].tri) &&
                   w->surfaceNameOfTri(contacts[2].tri) == "metal" && approx(w->surfaceOfTri(contacts[2].tri).damage, 0.3f),
                   "the fence: a passbullets layer, metal");
        }
        w->segmentContacts(b, a, contacts, scratch);
        expect(contacts.size() == 4 && contacts[0].entering && contacts[0].t < contacts[1].t, "reversed: nearest first, entering");
        w->segmentContacts({ 30, 60, 500 }, { 30, 60, 200 }, contacts, scratch);
        expect(contacts.empty(), "clip layers are not reported (segmentMask has no clips)");
        expect((w->segmentMask() & w->classMask(LayerClass::Clip)) == 0 &&
               (w->segmentMask() & w->classMask(LayerClass::Solid)) == w->classMask(LayerClass::Solid),
               "segmentMask: solid and passbullets layers, no clips");

        check::section("box world: winding and open geometry");
        // Every closed solid (each edge used once each way) must have a positive signed volume.
        struct Solid { double volume = 0.0; std::map<std::pair<uint32_t, uint32_t>, int> edges; };
        std::unordered_map<uint32_t, Solid> solids;
        const Bvh& bvh = w->bvh();
        for (uint32_t i = 0; i < bvh.triangleCount(); ++i) {
            const Tri& t = bvh.tri(i);
            Solid& s = solids[t.solid];
            const Vec3 p = bvh.vertex(t.a), q = bvh.vertex(t.b), r = bvh.vertex(t.c);
            s.volume += static_cast<double>(p.dot(q.cross(r))) / 6.0;
            for (auto [x, y] : { std::pair{ t.a, t.b }, std::pair{ t.b, t.c }, std::pair{ t.c, t.a } })
                s.edges[{ x, y }] += 1;
        }
        int closed = 0, inward = 0;
        for (const auto& [id, s] : solids) {
            bool isClosed = true;
            for (const auto& [e, n] : s.edges) {
                const auto back = s.edges.find({ e.second, e.first });
                isClosed = isClosed && n == 1 && back != s.edges.end() && back->second == 1;
            }
            closed += isClosed;
            inward += isClosed && s.volume <= 0.0;
        }
        std::printf("  %zu solids, %d closed, %d wound inward\n", solids.size(), closed, inward);
        expect(solids.size() == 6 && closed == 4 && inward == 0, "hulls, sphere and capsule closed and wound outward");
        const OpenGeometry open = findOpenGeometry(*w);
        expect(open.solids.size() == 2 && open.count == 8, "open geometry: the two mesh quads, 4 edges each");
    }

    // A big flat mesh (2 n^2 triangles) to keep a build busy.
    Bytes bigMapPhysics(int n) {
        Bytes vertices, triangles;
        for (int y = 0; y <= n; ++y)
            for (int x = 0; x <= n; ++x) {
                fixtures::put(vertices, static_cast<float>(x) * 8.f);
                fixtures::put(vertices, static_cast<float>(y) * 8.f);
                fixtures::put(vertices, static_cast<float>((x * 7 + y * 13) % 50));
            }
        const auto at = [&](int x, int y) { return static_cast<int32_t>(y * (n + 1) + x); };
        for (int y = 0; y < n; ++y)
            for (int x = 0; x < n; ++x)
                for (const int32_t v : { at(x, y), at(x + 1, y), at(x + 1, y + 1), at(x, y), at(x + 1, y + 1), at(x, y + 1) })
                    fixtures::put(triangles, v);
        const Kv mesh = Kv::object({ { "m_nCollisionAttributeIndex", Kv::integer(0) }, { "m_nSurfacePropertyIndex", Kv::integer(0) },
                                     { "m_Mesh", Kv::object({ { "m_Vertices", Kv::bytes(vertices) },
                                                              { "m_Triangles", Kv::bytes(triangles) } }) } });
        const Kv doc = Kv::object({
            { "m_surfacePropertyHashes", Kv::typedArray({ Kv::uinteger(surfaceHash("default")) }) },
            { "m_collisionAttributes", Kv::array({ fixtures::layerAttributes({}) }) },
            { "m_parts", Kv::array({ Kv::object({ { "m_rnShape", Kv::object({ { "m_meshes", Kv::array({ mesh }) } }) } }) }) },
        });
        return fixtures::resourceFile({ { fourCC("PHYS"), fixtures::encodeKv3(doc, 5, Compression::Lz4) } });
    }

    void testLocator(const fixtures::FakeInstall& install) {
        check::section("locator (fake install)");
        MapLocator loc(install.csgoDir());
        std::string why;
        const auto box = loc.find("box_world", why);
        expect(box && box->kind == MapSource::Kind::Official && box->vpk.filename() == "box_world.vpk" && box->map == "box_world",
               "official map");
        const auto ws = loc.find("ws_box", why);
        expect(ws && ws->kind == MapSource::Kind::Workshop && ws->workshopId == fixtures::FakeInstall::kWorkshopId,
               "workshop map by name");
        const auto ws2 = loc.find(std::string("workshop/") + fixtures::FakeInstall::kWorkshopId + "/ws_box", why);
        expect(ws && ws2 && ws2->vpk == ws->vpk, "workshop map by workshop path");
        const auto byPath = loc.find("maps/box_world.vpk", why);
        expect(byPath && byPath->map == "box_world", "\"maps/<map>.vpk\" form");
        const auto messy = loc.find("MAPS\\Box_World.vpk", why);
        expect(messy && messy->map == "box_world", "case and backslashes normalised");
        expect(!loc.find("ws_box_3dsky", why), "skybox VPKs are not maps");
        expect(!loc.find("no_such_map_x", why) && !why.empty(), "unknown map: not found, with a reason");
        expect(!loc.find("", why) && !why.empty(), "empty name: not found, with a reason");
        const auto again = loc.find("box_world", why);
        expect(box && again && box->sameAs(*again), "same file twice: sameAs");
    }

    // Polls update() until the state settles (or times out).
    MapManager::Status run(MapManager& m, std::string_view map, const std::filesystem::path& csgo, double timeoutS = 60.0) {
        const auto start = Clock::now();
        for (;;) {
            m.update(map, csgo);
            const auto s = m.status();
            if (s.state != MapManager::State::Loading ||
                std::chrono::duration<double>(Clock::now() - start).count() > timeoutS)
                return s;
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    void testManager(const fixtures::FakeInstall& install) {
        check::section("manager (fake install)");
        const auto& csgo = install.csgoDir();
        MapManager m(LayerRules{});
        m.update("", csgo);
        expect(m.status().state == MapManager::State::Idle && !m.world(), "no map: idle");
        m.update("box_world", {});
        expect(m.status().state == MapManager::State::Idle, "no game directory: idle");

        auto s = run(m, "box_world", csgo);
        expect(s.state == MapManager::State::Ready && m.world() && m.world()->map() == "box_world" &&
               m.world()->bvh().triangleCount() == fixtures::PhysicsExpect::tris, "loads box_world (every triangle)");
        expect(m.world() && m.world()->surfaceNote().empty() && m.world()->surfaceNameOfTri(0) == "concrete",
               "surface table read from pak01 (archive VPK)");
        expect(s.source && s.source->kind == MapSource::Kind::Official && !s.reused, "status: official source, built");

        // A big map, then away at once: its job is cancelled / discarded.
        m.update("big_world", csgo);
        expect(!m.world() && m.status().state == MapManager::State::Loading, "map change: world cleared, loading");
        s = run(m, "ws_box", csgo);
        expect(s.state == MapManager::State::Ready && m.world() && m.world()->map() == "ws_box" && s.source &&
               s.source->kind == MapSource::Kind::Workshop, "switching mid-load ends on the new map (workshop, nested VPK)");
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        m.update("ws_box", csgo);
        expect(m.world() && m.world()->map() == "ws_box", "the cancelled job never replaces the current world");

        // Leave and come back: the cached world is reused.
        m.update("", csgo);
        expect(!m.world() && m.status().state == MapManager::State::Idle, "no map wanted: no world");
        const auto cached = run(m, "ws_box", csgo);
        expect(cached.state == MapManager::State::Ready && cached.reused && m.world(), "asking again reuses the cached world");

        s = run(m, "big_world", csgo);
        expect(s.state == MapManager::State::Ready && m.world() && m.world()->bvh().triangleCount() == 2u * 400 * 400,
               "a big map loads");
        s = run(m, "broken", csgo);
        expect(s.state == MapManager::State::Failed && !s.reason.empty() && !m.world(), "corrupt map: failed, with a reason");
        std::printf("    (reason: %s)\n", s.reason.c_str());
        s = run(m, "no_such_map_x", csgo);
        expect(s.state == MapManager::State::Failed && !s.reason.empty() && !m.world(), "unknown map: failed, with a reason");
        expect(std::string(toString(s.state)) == "failed", "toString(State)");
    }

} // namespace

int main() {
    testRules();
    testWorld();
    testBoxWorld();

    try {
        fixtures::FakeInstall install("s2ghtline_test_world");
        install.writeMap("big_world", bigMapPhysics(400));
        Bytes broken = fixtures::physicsModelFile(5, Compression::Lz4);
        for (size_t i = broken.size() / 2; i < broken.size(); i += 7)
            broken[i] ^= 0x5A;
        install.writeMap("broken", broken);
        testLocator(install);
        testManager(install);
    }
    catch (const std::exception& e) {
        std::printf("  FAIL fake install: %s\n", e.what());
        ++check::failures();
    }
    return check::result();
}
