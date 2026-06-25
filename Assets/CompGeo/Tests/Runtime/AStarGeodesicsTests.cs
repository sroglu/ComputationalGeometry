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
    public class AStarGeodesicsTests
    {
        const string QuadOff =
            "OFF\n4 2 0\n0 0 0\n1 0 0\n1 1 0\n0 1 0\n3 0 1 2\n3 0 2 3\n";

        [Test]
        public void QuadAStarMatchesHandComputed()
        {
            using var mesh = OffReader.Read(new StringReader(QuadOff), Allocator.Persistent);
            var dist = new NativeArray<float>(mesh.VertexCount, Allocator.Persistent);
            var pred = new NativeArray<int>(mesh.VertexCount, Allocator.Persistent);
            try
            {
                AStarGeodesics.Compute(mesh, 0, 2, dist, pred);
                Assert.AreEqual(math.SQRT2, dist[2], 1e-5f); // optimal across the 0-2 diagonal

                using var path = DijkstraGeodesics.ReconstructPath(pred, 0, 2, Allocator.Persistent);
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
        public void AStarTargetCostEqualsDijkstraForEveryTarget()
        {
            // A* with an admissible+consistent heuristic must settle each target at the same optimal
            // distance Dijkstra computes — verified against the full single-source sweep as the oracle.
            using var mesh = OffReader.Read(new StringReader(BuildGridOff(4)), Allocator.Persistent);
            int n = mesh.VertexCount;
            const int source = 0;

            var dijkstraDist = new NativeArray<float>(n, Allocator.Persistent);
            var dijkstraPred = new NativeArray<int>(n, Allocator.Persistent);
            try
            {
                DijkstraGeodesics.Compute(mesh, source, dijkstraDist, dijkstraPred);

                for (int target = 0; target < n; target++)
                {
                    var aDist = new NativeArray<float>(n, Allocator.Persistent);
                    var aPred = new NativeArray<int>(n, Allocator.Persistent);
                    try
                    {
                        AStarGeodesics.Compute(mesh, source, target, aDist, aPred);
                        Assert.AreEqual(dijkstraDist[target], aDist[target], 1e-3f, $"A* cost differs at target {target}");

                        // The reconstructed path's accumulated edge length must equal the reported cost.
                        using var path = DijkstraGeodesics.ReconstructPath(aPred, source, target, Allocator.Persistent);
                        if (target == source)
                            Assert.AreEqual(1, path.Length);
                        else
                            Assert.GreaterOrEqual(path.Length, 2);
                        Assert.AreEqual(source, path[0]);
                        Assert.AreEqual(target, path[path.Length - 1]);
                        Assert.AreEqual(dijkstraDist[target], PathLength(mesh, path), 1e-3f, $"path length != cost at target {target}");
                    }
                    finally
                    {
                        aDist.Dispose();
                        aPred.Dispose();
                    }
                }
            }
            finally
            {
                dijkstraDist.Dispose();
                dijkstraPred.Dispose();
            }
        }

        [Test]
        public void FindPathReturnsCostAndPath()
        {
            using var mesh = OffReader.Read(new StringReader(BuildGridOff(4)), Allocator.Persistent);
            int target = mesh.VertexCount - 1;
            float cost = AStarGeodesics.FindPath(mesh, 0, target, Allocator.Persistent, out var path);
            try
            {
                Assert.AreEqual(0, path[0]);
                Assert.AreEqual(target, path[path.Length - 1]);
                Assert.AreEqual(cost, PathLength(mesh, path), 1e-3f);
            }
            finally
            {
                path.Dispose();
            }
        }

        // --- helpers ------------------------------------------------------------------------------

        static float PathLength(MeshData mesh, NativeList<int> path)
        {
            float total = 0f;
            for (int i = 1; i < path.Length; i++)
                total += math.distance(mesh.Positions[path[i - 1]], mesh.Positions[path[i]]);
            return total;
        }

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
    }
}
