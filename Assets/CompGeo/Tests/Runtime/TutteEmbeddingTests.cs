using System.Collections.Generic;
using NUnit.Framework;
using Unity.Collections;
using Unity.Mathematics;
using CompGeo.Core;
using CompGeo.MeshProcessing.Parameterization;

namespace CompGeo.Tests
{
    /// <summary>
    /// Tests for the Tutte/Floater uniform-Laplacian parameterization (docs/MIGRATION.md §6 HW2).
    /// Correctness is checked structurally — independent of mesh geometry — via the two defining
    /// properties of the embedding: the boundary lands on the unit circle, and every interior vertex
    /// sits at the average of its neighbours (the discrete harmonic / barycentric condition).
    /// </summary>
    public class TutteEmbeddingTests
    {
        [Test]
        public void HexagonFanCentresInteriorVertex()
        {
            // Center vertex 0 surrounded by a 6-vertex ring; every spoke is interior, every ring edge
            // is boundary. The lone interior vertex must map to the centroid of the circle ≈ (0,0).
            using var mesh = BuildHexagonFan();
            var uv = TutteEmbedding.Compute(mesh, Allocator.Temp);
            try
            {
                for (int v = 1; v <= 6; v++)
                    Assert.AreEqual(1f, math.length(uv[v]), 1e-4f, $"boundary vertex {v} not on unit circle");

                Assert.AreEqual(0f, uv[0].x, 1e-3f, "interior x should be the circle centroid");
                Assert.AreEqual(0f, uv[0].y, 1e-3f, "interior y should be the circle centroid");
            }
            finally { uv.Dispose(); }
        }

        [Test]
        public void GridSatisfiesHarmonicProperty()
        {
            using var mesh = BuildGrid(6); // 36 verts: 20 boundary + 16 interior (with interior neighbours)
            var uv = TutteEmbedding.Compute(mesh, Allocator.Temp);

            var boundary = new HashSet<int>();
            using (var loop = MeshBoundary.ExtractLoop(mesh, Allocator.Temp))
                for (int i = 0; i < loop.Length; i++) boundary.Add(loop[i]);

            try
            {
                bool sawInteriorWithInteriorNeighbour = false;
                for (int v = 0; v < mesh.VertexCount; v++)
                {
                    Assert.IsFalse(math.any(math.isnan(uv[v])), $"NaN UV at vertex {v}");
                    if (boundary.Contains(v))
                    {
                        Assert.AreEqual(1f, math.length(uv[v]), 1e-4f, $"boundary vertex {v} not on unit circle");
                        continue;
                    }

                    // Interior: uv[v] must equal the average of its neighbours' UVs (L·u = 0).
                    mesh.GetNeighbours(v, out int start, out int count);
                    float2 mean = float2.zero;
                    for (int k = 0; k < count; k++)
                    {
                        int w = mesh.AdjNeighbours[start + k];
                        mean += uv[w];
                        if (!boundary.Contains(w)) sawInteriorWithInteriorNeighbour = true;
                    }
                    mean /= count;
                    Assert.AreEqual(mean.x, uv[v].x, 5e-3f, $"harmonic x violated at {v}");
                    Assert.AreEqual(mean.y, uv[v].y, 5e-3f, $"harmonic y violated at {v}");
                }

                Assert.IsTrue(sawInteriorWithInteriorNeighbour,
                    "grid should exercise interior-interior coupling (off-diagonal solve)");
            }
            finally { uv.Dispose(); }
        }

        [Test]
        public void ClosedMeshThrows()
        {
            // A tetrahedron has no boundary — Tutte must reject it.
            using var mesh = BuildTetrahedron();
            Assert.Throws<System.ArgumentException>(() =>
            {
                var uv = TutteEmbedding.Compute(mesh, Allocator.Temp);
                uv.Dispose();
            });
        }

        static MeshData BuildHexagonFan()
        {
            var positions = new List<float3> { float3.zero };
            for (int i = 0; i < 6; i++)
            {
                float a = 2f * math.PI * i / 6f;
                positions.Add(new float3(math.cos(a), 0f, math.sin(a)));
            }
            var triangles = new List<int3>();
            for (int i = 0; i < 6; i++)
                triangles.Add(new int3(0, 1 + i, 1 + (i + 1) % 6));
            return MeshBuilder.Build(positions, triangles, Allocator.Persistent);
        }

        static MeshData BuildGrid(int s)
        {
            var positions = new List<float3>(s * s);
            for (int j = 0; j < s; j++)
            for (int i = 0; i < s; i++)
                positions.Add(new float3(i, 0f, j));

            var triangles = new List<int3>((s - 1) * (s - 1) * 2);
            for (int j = 0; j < s - 1; j++)
            for (int i = 0; i < s - 1; i++)
            {
                int a = j * s + i, b = a + 1, c = (j + 1) * s + i, d = c + 1;
                triangles.Add(new int3(a, b, d));
                triangles.Add(new int3(a, d, c));
            }
            return MeshBuilder.Build(positions, triangles, Allocator.Persistent);
        }

        static MeshData BuildTetrahedron()
        {
            var positions = new List<float3>
            {
                new float3(0, 0, 0), new float3(1, 0, 0),
                new float3(0, 1, 0), new float3(0, 0, 1),
            };
            var triangles = new List<int3>
            {
                new int3(0, 2, 1), new int3(0, 1, 3),
                new int3(0, 3, 2), new int3(1, 2, 3),
            };
            return MeshBuilder.Build(positions, triangles, Allocator.Persistent);
        }
    }
}
