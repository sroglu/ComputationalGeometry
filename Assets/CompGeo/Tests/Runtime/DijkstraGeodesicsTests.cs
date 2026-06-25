using System.IO;
using System.Text;
using NUnit.Framework;
using Unity.Collections;
using Unity.Mathematics;
using CompGeo.Core;
using CompGeo.Core.IO;
using CompGeo.MeshProcessing.Geodesics;

namespace CompGeo.Tests
{
    public class DijkstraGeodesicsTests
    {
        // Unit quad, two triangles sharing the 0-2 diagonal (see MeshCoreTests):
        //   3 ---- 2
        //   |    / |
        //   0 ---- 1
        const string QuadOff =
            "OFF\n4 2 0\n0 0 0\n1 0 0\n1 1 0\n0 1 0\n3 0 1 2\n3 0 2 3\n";

        static MeshData ReadQuad() => OffReader.Read(new StringReader(QuadOff), Allocator.Persistent);

        static void Compute(MeshData mesh, int source, out NativeArray<float> dist, out NativeArray<int> pred)
        {
            dist = new NativeArray<float>(mesh.VertexCount, Allocator.Persistent);
            pred = new NativeArray<int>(mesh.VertexCount, Allocator.Persistent);
            DijkstraGeodesics.Compute(mesh, source, dist, pred);
        }

        [Test]
        public void QuadDistancesMatchHandComputed()
        {
            using var mesh = ReadQuad();
            Compute(mesh, 0, out var dist, out var pred);
            try
            {
                Assert.AreEqual(0f, dist[0], 1e-5f);
                Assert.AreEqual(1f, dist[1], 1e-5f);
                Assert.AreEqual(math.SQRT2, dist[2], 1e-5f);   // shortest is the 0-2 diagonal, not 0->1->2
                Assert.AreEqual(1f, dist[3], 1e-5f);

                Assert.AreEqual(DijkstraGeodesics.NoPredecessor, pred[0]);
                Assert.AreEqual(0, pred[1]);
                Assert.AreEqual(0, pred[2]);                   // reached directly across the diagonal
                Assert.AreEqual(0, pred[3]);
            }
            finally
            {
                dist.Dispose();
                pred.Dispose();
            }
        }

        [Test]
        public void PathReconstructionIsSourceToTarget()
        {
            using var mesh = ReadQuad();
            Compute(mesh, 0, out var dist, out var pred);
            using var path = DijkstraGeodesics.ReconstructPath(pred, 0, 2, Allocator.Persistent);
            try
            {
                Assert.AreEqual(2, path.Length);
                Assert.AreEqual(0, path[0]);
                Assert.AreEqual(2, path[1]);
            }
            finally
            {
                dist.Dispose();
                pred.Dispose();
            }
        }

        [Test]
        public void UnreachableVertexHasInfiniteDistanceAndEmptyPath()
        {
            // Two disjoint triangles -> vertex 0 cannot reach vertex 3..5.
            const string twoIslandsOff =
                "OFF\n6 2 0\n" +
                "0 0 0\n1 0 0\n0 1 0\n" +
                "5 0 0\n6 0 0\n5 1 0\n" +
                "3 0 1 2\n3 3 4 5\n";
            using var mesh = OffReader.Read(new StringReader(twoIslandsOff), Allocator.Persistent);
            Compute(mesh, 0, out var dist, out var pred);
            using var path = DijkstraGeodesics.ReconstructPath(pred, 0, 4, Allocator.Persistent);
            try
            {
                Assert.IsTrue(float.IsPositiveInfinity(dist[4]));
                Assert.AreEqual(DijkstraGeodesics.NoPredecessor, pred[4]);
                Assert.AreEqual(0, path.Length);
            }
            finally
            {
                dist.Dispose();
                pred.Dispose();
            }
        }

        [Test]
        public void MatchesIndependentFloydWarshallOnGrid()
        {
            // Independent oracle: all-pairs Floyd-Warshall over the same CSR edge graph with Euclidean
            // weights, computed by a separate code path. Single-source Dijkstra must reproduce each row.
            using var mesh = OffReader.Read(new StringReader(BuildGridOff(4)), Allocator.Persistent);
            int n = mesh.VertexCount;
            float[,] reference = FloydWarshall(mesh);

            for (int s = 0; s < n; s++)
            {
                Compute(mesh, s, out var dist, out var pred);
                try
                {
                    for (int t = 0; t < n; t++)
                        Assert.AreEqual(reference[s, t], dist[t], 1e-3f, $"distance mismatch ({s} -> {t})");
                }
                finally
                {
                    dist.Dispose();
                    pred.Dispose();
                }
            }
        }

        // --- helpers ------------------------------------------------------------------------------

        /// <summary>An m×m grid of unit-spaced vertices, each cell split into two triangles.</summary>
        static string BuildGridOff(int m)
        {
            int verts = m * m;
            int faces = (m - 1) * (m - 1) * 2;
            var sb = new StringBuilder();
            sb.Append("OFF\n").Append(verts).Append(' ').Append(faces).Append(" 0\n");

            for (int j = 0; j < m; j++)
            for (int i = 0; i < m; i++)
                sb.Append(i).Append(' ').Append(j).Append(" 0\n");

            for (int j = 0; j < m - 1; j++)
            for (int i = 0; i < m - 1; i++)
            {
                int a = j * m + i;
                int b = j * m + i + 1;
                int c = (j + 1) * m + i;
                int d = (j + 1) * m + i + 1;
                sb.Append("3 ").Append(a).Append(' ').Append(b).Append(' ').Append(d).Append('\n');
                sb.Append("3 ").Append(a).Append(' ').Append(d).Append(' ').Append(c).Append('\n');
            }
            return sb.ToString();
        }

        static float[,] FloydWarshall(MeshData mesh)
        {
            int n = mesh.VertexCount;
            var d = new float[n, n];
            for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                d[i, j] = i == j ? 0f : float.PositiveInfinity;

            for (int u = 0; u < n; u++)
            {
                mesh.GetNeighbours(u, out int start, out int count);
                for (int k = 0; k < count; k++)
                {
                    int w = mesh.AdjNeighbours[start + k];
                    d[u, w] = math.distance(mesh.Positions[u], mesh.Positions[w]);
                }
            }

            for (int k = 0; k < n; k++)
            for (int i = 0; i < n; i++)
            {
                float dik = d[i, k];
                if (float.IsPositiveInfinity(dik)) continue;
                for (int j = 0; j < n; j++)
                {
                    float through = dik + d[k, j];
                    if (through < d[i, j]) d[i, j] = through;
                }
            }
            return d;
        }
    }
}
