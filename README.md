# s2ghtline

A header-only C++20 library that reads a Counter-Strike 2 map's collision straight from the game's files and answers line-of-sight and ray queries against it in a few microseconds.

s2ghtline reads files from an installed copy of the game. It ships no game files: no maps, no VPKs, no extracted data, in the library or its tests.

## What it reads

| Format | Header | Notes |
|---|---|---|
| VPK v1 / v2 | `Vpk.h` | single-file paks, `_dir.vpk` + `_NNN.vpk` archives, preload data; opened from disk or memory (workshop items nest the map's VPK inside their own) |
| Compiled resources (`*_c`) | `Resource.h` | the block table (`PHYS`, `DATA`, ...) |
| Binary KeyValues3 v2-v5 | `Kv3.h` | uncompressed, LZ4 (chained blob frames), zstd; every value type up to type 25, typed / auxiliary arrays, blobs; keys looked up by compile-time hash (`"m_parts"_kv`) |
| Map physics (`maps/<map>/world_physics.vmdl_c` → `PHYS`) | `PhysParser.h`, `MapPhysics.h` | convex hulls (half-edge polygons, both encodings), triangle meshes with per-triangle materials, spheres and capsules (tessellated), each shape's surface and collision layer (`InteractAs` tags) |
| Surface properties (`surfaceproperties.vsurf_c`, `surfaceproperties_game.txt`) | `SurfaceProperties.h` | bullet penetration values per surface, inheritance resolved |

Every offset and length in these files is checked before use: a corrupt file fails with a reason (`std::optional` + `why`), never a crash. There are no exceptions.

## What it does with it

- **`Bvh.h`**: a bounding volume hierarchy over the triangles (binned SAH, 32-byte nodes, triangles as indices into shared vertices). Segment queries take a layer mask: `occluded` (any hit), `closest` (first hit), `traceAll` (every hit). The triangle test is watertight (Woop, Benthin, Wald 2013), so a segment through an edge shared by two hulls can't slip between them.
- **`World.h`**: a loaded map. Each collision layer is classed **Solid**, **SeeThrough** (fences, glass, water) or **Clip** (player / grenade clips, ladders, sky) from its tags (`LayerRules`). It offers line of sight (`visible`), traces, and `segmentContacts`: every surface a segment crosses, entering or leaving, with that surface's penetration values.
- **`MapLocator.h`** (optional): finds a map's VPK in an install by name: official maps, workshop maps by name or `workshop/<id>/<map>`.
- **`MapManager.h`** (optional): loads the wanted map on a background thread, cancels a load when the wanted map changes, and reuses the last world when its file is unchanged.

## Usage

```cpp
#include "MapPhysics.h"
#include "World.h"

using namespace s2ghtline;

std::string why;
const auto vpk = openMapVpk("<cs2>/game/csgo/maps/de_mirage.vpk", "de_mirage", why);
auto geometry  = vpk ? loadMapGeometry(*vpk, "de_mirage", why) : std::nullopt;
if (!geometry)
    return fail(why);   // why the map couldn't be read

const auto world = WorldBuilder::build("de_mirage", std::move(*geometry), LayerRules{});

const Vec3 a{ -1000.f, -900.f, 64.f }, b{ 200.f, -300.f, 64.f };
if (world->visible(a, b)) { /* nothing solid between a and b */ }

if (const auto hit = world->trace(a, b, world->visionMask())) {
    const float distance = (b - a).length() * hit->t;
    const auto& surface  = world->surfaceOfTri(hit->tri);   // penetration, damage, material type
}

std::vector<World::SegmentContact> contacts;
std::vector<Hit> scratch;
world->segmentContacts(a, b, contacts, scratch);   // every surface crossed, nearest first
```

To load maps in the background by name:

```cpp
#include "MapManager.h"

s2ghtline::MapManager maps(s2ghtline::LayerRules{});
// Call whenever the wanted map may have changed, and again to pick up a finished build:
maps.update("de_mirage", "<cs2>/game/csgo");
if (const auto& world = maps.world())   // null until the build finishes
    world->visible(a, b);               // a, b as above
```

`LayerRules` holds three tag lists (`clip`, `seeThrough`, `solid`) with built-in defaults; set them to class layers differently. A tag in no list is reported in `WorldStats::unknownTags` and treated as solid. Pass a `SurfaceTable` to `WorldBuilder::build` for the game's penetration values (`MapManager` reads it from `pak01` itself); without one every surface gets the defaults and `surfaceNote()` says why.

`examples/raycast.cpp` does all of this end to end: `raycast de_mirage` prints the world's stats, a few traces, and the cost of 100k random queries.

## Building

The library is the headers in `include/s2ghtline/`, plus two C files for the decompressors (`third_party/`). The headers include each other by bare name, so put `include/s2ghtline` and `third_party` on the include path and compile `third_party/lz4/lz4.c` and `third_party/zstd/zstddeclib.c`. Define `NOMINMAX` on Windows.

**CMake** (3.25+ for the presets):

```sh
cmake --preset release            # or debug, asan; vs2022 for a Visual Studio solution
cmake --build --preset release
ctest --preset release            # every test; release-synthetic for the game-free ones only
```

As a dependency: `add_subdirectory(s2ghtline)` then `target_link_libraries(app PRIVATE s2ghtline::s2ghtline)`. The target carries the include paths, C++20, `NOMINMAX` and the decompressors (`s2ghtline_third_party`, a static library).

**Visual Studio**: `s2ghtline.sln` builds the example (`build/vs/<platform>/<configuration>/raycast.exe`) with C++20, `/W4` and `/permissive-`.

The Ninja presets need the compiler in the environment: on Windows, run them from a Developer Command Prompt (`cl`), or with MinGW / Clang. The `asan` preset uses `/fsanitize=address` with MSVC and `-fsanitize=address,undefined` with GCC / Clang. The code builds without warnings at `/W4` (MSVC 19.44) and `-Wall -Wextra -Wpedantic -Wshadow -Wconversion` (GCC 16); `-DS2GHTLINE_WERROR=ON` makes warnings errors, as CI does.

## Tests and verification

| Test | Checks | Needs CS2 |
|---|---|---|
| `test_bvh` | every query against brute force (random triangle soups, layer masks), the triangle test against an independent double-precision one, watertightness (200k segments through shared vertices and edges) | no |
| `test_kv3` | a KV3 writer encodes every value type in versions 2-5, uncompressed / LZ4 / zstd, and the decoder must read them back exactly; a generated map's physics (hulls in both encodings, meshes, spheres, capsules) parses to the expected geometry; 18,000 corrupted copies must fail cleanly | no |
| `test_world` | layer classes, line of sight, more than 64 layers, cancellation, segment contacts (enter / leave), winding of closed solids, open geometry, surface lookup; `MapLocator` and `MapManager` end to end on a fake install written to a temp folder (official map, workshop item with a nested VPK, archive VPKs) | no |
| `test_surfaces` | surface property inheritance, malformed text and resource files | no |
| `test_bvh_maps` | brute-force agreement on real maps; the performance numbers below | yes |
| `test_kv3_maps` | real PHYS blocks (official zstd, workshop LZ4) and corrupted copies of them | yes |
| `test_world_maps` | `MapLocator` / `MapManager` on the install: switching mid-load, reuse, workshop maps | yes |
| `test_contacts` | closed solids of real maps wound outward; contacts alternate enter / leave | yes |
| `test_surfaces_maps` | the game's surface table: known surfaces' values | yes |
| `compare_oracle.py` | every installed map (official and workshop, every Steam library) through `map_check` and through [ValveResourceFormat](https://github.com/ValveResourceFormat/ValveResourceFormat) (`tools/MapOracle`), summaries diffed | yes |

The game-free tests (CTest label `synthetic`) build all their input with `tests/support/Fixtures.h`, so they run anywhere. CI runs them on Windows (MSVC, plus the Visual Studio project) and Linux (GCC, Clang), each also under ASan / UBSan.

The real-map tests (label `cs2`) look for the game in `S2GHTLINE_CS2_DIR` (the "Counter-Strike Global Offensive" folder or its `game/csgo` folder), else in every Steam library listed in Steam's `libraryfolders.vdf`. Without an install they report themselves skipped; if `S2GHTLINE_CS2_DIR` names a folder that isn't one, they fail.

```sh
S2GHTLINE_CS2_DIR="/path/to/Counter-Strike Global Offensive" ctest --preset release -L cs2
python tests/compare_oracle.py      # needs the .NET 10 SDK; finds map_check under build/
```

## Measured

Release build (MSVC 19.44, `cmake --preset release`), one thread, Intel Core Ultra 7 265KF, CS2 build of November 2026. Queries are 200k random segments of up to 3000 units inside the map's bounds, against every layer (`test_bvh_maps`).

| Map | Triangles | BVH build | Memory | Bytes / triangle | `occluded` | `closest` | `traceAll` |
|---|---:|---:|---:|---:|---:|---:|---:|
| de_mirage  |   133,133 |    98 ms |   4.8 MB | 38.2 | 1.57 µs | 3.25 µs | 5.63 µs |
| de_dust2   |   435,649 |   319 ms |  15.8 MB | 37.9 | 2.05 µs | 3.83 µs | 6.63 µs |
| de_inferno | 2,733,603 | 2,287 ms |  97.8 MB | 37.5 | 1.56 µs | 2.66 µs | 6.77 µs |
| de_boulder | 5,290,740 | 4,758 ms | 188.8 MB | 37.4 | 0.95 µs | 1.38 µs | 3.62 µs |

Reading and parsing de_mirage takes about 30 ms on top of the build. Memory is the BVH's nodes, triangles (20 bytes: three vertex indices, surface, layer, solid) and vertices.

`compare_oracle.py` against ValveResourceFormat 20.0.6980: **39 / 39 installed maps match**: 33 official maps (a current install and an older one in a second Steam library) and 6 workshop maps. Shape counts, every hull and mesh vertex and triangle, per-layer and per-surface triangle counts are compared exactly; spheres and capsules by count. Two of the workshop maps match because both readers reject them (see below).

## Limitations

- **KV3 value type 26.** Some recently published workshop maps use a KV3 value type newer than the decoder (and ValveResourceFormat 20.0.6980) knows. Such maps fail to load with "corrupt KV3 data"; official maps are unaffected.
- Compound shapes are counted (`GeometryStats::compounds`), not read: no current map has any.
- Part transforms (`m_bindPose`) are not applied: world physics has none (`GeometryStats::bindPose` reports it).

## Layout

```
include/s2ghtline/   the library (header-only)
third_party/         lz4, zstd decoder (C)
examples/            raycast.cpp
tests/               tests (support/: fixtures, CS2 discovery), map_check, compare_oracle.py
tools/MapOracle/     the ValveResourceFormat reference (C#, .NET 10)
s2ghtline/           the Visual Studio project (s2ghtline.sln builds it)
```

## Third-party code

LZ4 1.10.0 (BSD 2-Clause) and the Zstandard 1.5.7 decoder (BSD 3-Clause) are included in `third_party/`, each with its license; see [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md). `tools/MapOracle` uses ValveResourceFormat (MIT) from NuGet; it is a development tool and not part of the library.
