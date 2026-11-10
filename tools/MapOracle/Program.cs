// MapOracle <vpk> <map> [<vpk> <map> ...]
// MapOracle --dump <vpk> <path in vpk> <out file>   (a compiled resource as KV3 text; other files as they are)
//
// The reference side of the map parser check. Loads a map's collision with
// ValveResourceFormat (the library behind Source 2 Viewer) and prints the same
// canonical summary as tests/map_check.cpp (the s2ghtline C++ parser).
// `python tests/compare_oracle.py` runs both over every installed map and
// diffs them. A development tool: not part of the library.
//
// Compared exactly: shape counts, hull + mesh vertex / triangle counts, the
// vertex checksum, per-layer and per-surface triangle counts. Spheres and
// capsules are tessellated by our parser, so only their count per layer is
// compared (kSphereTris / kCapsuleTris triangles each). "  info" lines are not
// compared.
using System.Globalization;
using System.Numerics;
using SteamDatabase.ValvePak;
using ValveKeyValue;
using ValveResourceFormat;
using ValveResourceFormat.ResourceTypes;
using ValveResourceFormat.Serialization.KeyValues;

internal static class Program
{
    // Triangles per tessellated sphere / capsule in include/s2ghtline/PhysParser.h.
    private const int kSphereTris = 60;
    private const int kCapsuleTris = 80;
    private const ushort kNoSurface = 0xFFFF;

    private static int Main(string[] args)
    {
        CultureInfo.DefaultThreadCurrentCulture = CultureInfo.InvariantCulture;
        if (args.Length == 4 && args[0] == "--dump")
            return Dump(args[1], args[2], args[3]);
        if (args.Length < 2 || args.Length % 2 != 0)
        {
            Console.WriteLine("usage: MapOracle <vpk> <map> [<vpk> <map> ...]");
            return 2;
        }

        var failures = 0;
        for (var i = 0; i + 1 < args.Length; i += 2)
        {
            try
            {
                Check(args[i], args[i + 1]);
            }
            catch (Exception e)
            {
                Console.WriteLine($"map {args[i + 1]}\n  FAILED {e.Message}");
                failures++;
            }
        }
        return failures > 0 ? 1 : 0;
    }

    // One file out of a VPK (e.g. the surface property files, to inspect them).
    private static int Dump(string vpk, string path, string outFile)
    {
        using var package = new Package();
        package.Read(vpk);
        var entry = package.FindEntry(path);
        if (entry == null)
        {
            Console.WriteLine($"{path}: not in {vpk}");
            return 1;
        }
        package.ReadEntry(entry, out var bytes);
        if (!path.EndsWith("_c", StringComparison.Ordinal))
        {
            File.WriteAllBytes(outFile, bytes);
            return 0;
        }
        using var resource = new Resource();
        resource.Read(new MemoryStream(bytes));
        // A model's physics (PHYS) when it has one, else its data block.
        var block = resource.GetBlockByType(BlockType.PHYS) ?? resource.DataBlock;
        File.WriteAllText(outFile, block!.ToString());
        return 0;
    }

    private static string ModelPath(string map) => $"maps/{map}/world_physics.vmdl_c";
    private static string DataPath(string map) => $"maps/{map}/world_physics.vphys_c";

    // The VPK holding the map's physics: the file itself, or the map VPK a workshop item nests in it.
    private static Package OpenMapPackage(string path, string map)
    {
        var package = new Package();
        package.Read(path);
        if (package.FindEntry(ModelPath(map)) != null || package.FindEntry(DataPath(map)) != null)
        {
            return package;
        }

        var nested = package.FindEntry($"maps/{map}.vpk")
            ?? throw new InvalidDataException($"no physics for {map} in {Path.GetFileName(path)}");
        package.ReadEntry(nested, out var bytes);
        package.Dispose();

        var inner = new Package();
        inner.SetFileName($"{map}.vpk");
        inner.Read(new MemoryStream(bytes));
        return inner;
    }

    private static PhysAggregateData LoadPhysics(Package package, string map, Resource resource)
    {
        var model = package.FindEntry(ModelPath(map));
        var entry = model ?? package.FindEntry(DataPath(map)) ?? throw new InvalidDataException("no physics file");
        package.ReadEntry(entry, out var bytes);
        resource.Read(new MemoryStream(bytes));
        var block = model != null ? resource.GetBlockByType(BlockType.PHYS) : resource.DataBlock;
        return block as PhysAggregateData ?? throw new InvalidDataException("no physics block");
    }

    private sealed class Summary
    {
        public int Hulls, Meshes, Spheres, Capsules, Compounds, Degenerate;
        public long Vertices, Triangles;
        public double SumX, SumY, SumZ;
        public long[] LayerTris = [];
        public long[] LayerPrimitiveTris = [];
        public SortedDictionary<int, long> SurfaceTris = [];
    }

    private static void Check(string vpkPath, string map)
    {
        using var package = OpenMapPackage(vpkPath, map);
        using var resource = new Resource();
        var phys = LoadPhysics(package, map, resource);

        var layers = phys.CollisionAttributes;
        var hashes = phys.SurfacePropertyHashes;
        var s = new Summary
        {
            LayerTris = new long[layers.Count],
            LayerPrimitiveTris = new long[layers.Count],
        };

        // Compound shapes aren't typed in this VRF version: count them from the raw data.
        foreach (var part in phys.Data.GetArray("m_parts") ?? [])
        {
            s.Compounds += part.GetSubCollection("m_rnShape")?.GetArray("m_compounds")?.Count() ?? 0;
        }

        foreach (var part in phys.Parts)
        {
            var shape = part.Shape;

            foreach (var h in shape.Hulls)
            {
                var hull = h.Shape;
                var positions = hull.GetVertexPositions().ToArray();
                var edges = hull.GetEdges().ToArray();
                AddVertices(s, positions);
                foreach (var face in hull.GetFaces())
                {
                    // The face's edge loop, triangulated as a fan from its first vertex.
                    var corners = new List<int>();
                    var e = (int)face.Edge;
                    do
                    {
                        corners.Add(edges[e].Origin);
                        e = edges[e].Next;
                    }
                    while (e != face.Edge);

                    for (var k = 2; k < corners.Count; k++)
                    {
                        AddTriangle(s, positions[corners[0]], positions[corners[k - 1]], positions[corners[k]],
                            h.CollisionAttributeIndex, SurfaceIndex(h.SurfacePropertyIndex, hashes.Length));
                    }
                }
                s.Hulls++;
            }

            foreach (var m in shape.Meshes)
            {
                var vertices = m.Shape.GetVertices().ToArray();
                AddVertices(s, vertices);
                var triangles = m.Shape.GetTriangles().ToArray();
                // Per-triangle materials (indices into the surface hashes) when there's one per triangle.
                var materials = m.Shape.Materials.ToArray();
                var perTriangle = materials.Length == triangles.Length;
                for (var i = 0; i < triangles.Length; i++)
                {
                    var t = triangles[i];
                    var surface = perTriangle && materials[i] < hashes.Length ? materials[i] : SurfaceIndex(m.SurfacePropertyIndex, hashes.Length);
                    AddTriangle(s, vertices[t.X], vertices[t.Y], vertices[t.Z], m.CollisionAttributeIndex, surface);
                }
                s.Meshes++;
            }

            foreach (var sphere in shape.Spheres)
            {
                s.LayerPrimitiveTris[sphere.CollisionAttributeIndex] += kSphereTris;
                s.Spheres++;
            }
            foreach (var capsule in shape.Capsules)
            {
                s.LayerPrimitiveTris[capsule.CollisionAttributeIndex] += kCapsuleTris;
                s.Capsules++;
            }
        }

        Print(map, s, layers, hashes);
    }

    private static int SurfaceIndex(int index, int count) => index >= 0 && index < count ? index : kNoSurface;

    private static void AddVertices(Summary s, Vector3[] points)
    {
        foreach (var p in points)
        {
            s.SumX += p.X;
            s.SumY += p.Y;
            s.SumZ += p.Z;
        }
        s.Vertices += points.Length;
    }

    // Same rule as PhysParser.h addTri: zero-area triangles are dropped.
    private static void AddTriangle(Summary s, Vector3 a, Vector3 b, Vector3 c, int layer, int surface)
    {
        float abx = b.X - a.X, aby = b.Y - a.Y, abz = b.Z - a.Z;
        float acx = c.X - a.X, acy = c.Y - a.Y, acz = c.Z - a.Z;
        float nx = aby * acz - abz * acy, ny = abz * acx - abx * acz, nz = abx * acy - aby * acx;
        if (MathF.Sqrt(nx * nx + ny * ny + nz * nz) < 1e-6f)
        {
            s.Degenerate++;
            return;
        }
        s.Triangles++;
        s.LayerTris[layer]++;
        s.SurfaceTris[surface] = s.SurfaceTris.GetValueOrDefault(surface) + 1;
    }

    private static void Print(string map, Summary s, IReadOnlyList<KVObject> layers, uint[] hashes)
    {
        var o = Console.Out;
        o.Write($"map {map}\n");
        o.Write($"  shapes hulls {s.Hulls} meshes {s.Meshes} spheres {s.Spheres} capsules {s.Capsules} compounds {s.Compounds}\n");
        o.Write($"  hull+mesh vertices {s.Vertices} triangles {s.Triangles} degenerate {s.Degenerate}\n");
        o.Write($"  vertex sum {s.SumX:F3} {s.SumY:F3} {s.SumZ:F3}\n");
        for (var i = 0; i < layers.Count; i++)
        {
            var group = layers[i].GetStringProperty("m_CollisionGroupString") ?? "";
            var interactAs = layers[i].GetArray<string>("m_InteractAsStrings")
                ?? layers[i].GetArray<string>("m_PhysicsTagStrings") ?? [];
            var exclude = layers[i].GetArray<string>("m_InteractExcludeStrings") ?? [];
            var asText = string.Concat(interactAs.Select(a => " " + a));
            var exText = string.Concat(exclude.Select(e => " " + e));
            o.Write($"  layer {i} {group} as[{asText} ] exclude[{exText} ] tris {s.LayerTris[i]} primitive-tris {s.LayerPrimitiveTris[i]}\n");
        }
        o.Write($"  surfaces {hashes.Length}\n");
        foreach (var (surface, n) in s.SurfaceTris)
        {
            o.Write(surface == kNoSurface ? $"  surface none tris {n}\n" : $"  surface {surface} {hashes[surface]} tris {n}\n");
        }
    }
}
