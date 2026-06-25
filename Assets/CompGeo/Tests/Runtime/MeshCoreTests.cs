using System.IO;
using NUnit.Framework;
using Unity.Collections;
using Unity.Mathematics;
using CompGeo.Core;
using CompGeo.Core.IO;

namespace CompGeo.Tests
{
    public class MeshCoreTests
    {
        // A unit quad split into two triangles sharing the 0-2 diagonal:
        //
        //   3 ---- 2
        //   |    / |
        //   |  /   |
        //   0 ---- 1
        //
        const string QuadOff =
            "OFF\n" +
            "4 2 0\n" +
            "0 0 0\n" +
            "1 0 0\n" +
            "1 1 0\n" +
            "0 1 0\n" +
            "3 0 1 2\n" +
            "3 0 2 3\n";

        static MeshData ReadQuad() => OffReader.Read(new StringReader(QuadOff), Allocator.Persistent);

        [Test]
        public void ReadsOffVertexAndTriangleCounts()
        {
            using var mesh = ReadQuad();
            Assert.AreEqual(4, mesh.VertexCount);
            Assert.AreEqual(2, mesh.TriangleCount);
            Assert.AreEqual(new float3(1, 1, 0), mesh.Positions[2]);
        }

        [Test]
        public void FanTriangulatesPolygonFaces()
        {
            // A single quad face (k = 4) must fan-triangulate into two triangles.
            const string quadFaceOff =
                "OFF\n4 1 0\n0 0 0\n1 0 0\n1 1 0\n0 1 0\n4 0 1 2 3\n";
            using var mesh = OffReader.Read(new StringReader(quadFaceOff), Allocator.Persistent);
            Assert.AreEqual(2, mesh.TriangleCount);
        }

        [Test]
        public void BuildsCsrAdjacencyWithExpectedDegrees()
        {
            using var mesh = ReadQuad();

            Assert.IsTrue(mesh.HasAdjacency);
            Assert.AreEqual(mesh.VertexCount + 1, mesh.AdjOffsets.Length);

            // Diagonal vertices 0 and 2 are shared by both triangles -> degree 3;
            // off-diagonal vertices 1 and 3 belong to one triangle -> degree 2.
            Assert.AreEqual(3, mesh.Degree(0));
            Assert.AreEqual(2, mesh.Degree(1));
            Assert.AreEqual(3, mesh.Degree(2));
            Assert.AreEqual(2, mesh.Degree(3));

            // Total directed neighbour entries == 2 * unique edge count (5 edges here).
            Assert.AreEqual(10, mesh.AdjNeighbours.Length);
        }

        [Test]
        public void AdjacencyIsSymmetricAndSorted()
        {
            using var mesh = ReadQuad();

            for (int a = 0; a < mesh.VertexCount; a++)
            {
                mesh.GetNeighbours(a, out int start, out int count);

                for (int k = 0; k < count; k++)
                {
                    int b = mesh.AdjNeighbours[start + k];
                    Assert.IsTrue(IsNeighbour(mesh, b, a), $"adjacency not symmetric for ({a}, {b})");
                    if (k > 0)
                        Assert.Less(mesh.AdjNeighbours[start + k - 1], b, "neighbours must be sorted ascending");
                }
            }
        }

        [Test]
        public void AabbFromMeshCoversAllVertices()
        {
            using var mesh = ReadQuad();
            Aabb box = Aabb.FromMesh(mesh);
            Assert.AreEqual(new float3(0, 0, 0), box.Min);
            Assert.AreEqual(new float3(1, 1, 0), box.Max);
        }

        [Test]
        public void Orient2DSignsAreCorrect()
        {
            Assert.Greater(GeometryPredicates.Orient2D(new float2(0, 0), new float2(1, 0), new float2(0, 1)), 0f);
            Assert.Less(GeometryPredicates.Orient2D(new float2(0, 0), new float2(0, 1), new float2(1, 0)), 0f);
            Assert.AreEqual(0f, GeometryPredicates.Orient2D(new float2(0, 0), new float2(1, 1), new float2(2, 2)));
        }

        static bool IsNeighbour(MeshData mesh, int v, int target)
        {
            mesh.GetNeighbours(v, out int start, out int count);
            for (int k = 0; k < count; k++)
                if (mesh.AdjNeighbours[start + k] == target) return true;
            return false;
        }
    }
}
