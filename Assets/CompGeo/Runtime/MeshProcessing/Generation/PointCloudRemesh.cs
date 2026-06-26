using System.Collections.Generic;
using System.Threading.Tasks;
using Unity.Collections;
using Unity.Mathematics;
using CompGeo.Core;
using CompGeo.Collections;

namespace CompGeo.MeshProcessing
{
    /// <summary>
    /// Rebuilds surface connectivity from a raw point set — a faithful port of the original CENG789
    /// homework's <c>Model.RemeshModel</c> (the "Mesh Generation" part). The algorithm, unchanged:
    /// <list type="number">
    /// <item>Build a k-d tree over all points.</item>
    /// <item>Greedily partition the points into neighbourhoods: repeatedly take the first unprocessed
    /// point, gather its <c>k</c> nearest neighbours (k = 8 originally), and mark them processed.</item>
    /// <item>For each neighbourhood, derive a local plane from the covariance rows
    /// (<see cref="Covariance.PlaneFromRows"/> — the homework's heuristic, not a true eigen-frame),
    /// project the points onto it, ear-clip the 2D polygon, and lift the triangles back to global
    /// vertex indices.</item>
    /// </list>
    /// Neighbourhoods overlap (each is the k-NN over <i>all</i> points, processed only gates seeding), so
    /// the result is the same union of local triangulations the original produced.
    /// </summary>
    public static class PointCloudRemesh
    {
        public const int DefaultK = 8;

        /// <summary>
        /// Remesh <paramref name="positions"/> into a new <see cref="MeshData"/> (positions copied,
        /// triangles regenerated) using the given local-plane <paramref name="method"/>. The caller owns
        /// and must dispose the result.
        /// </summary>
        public static MeshData Remesh(NativeArray<float3> positions, int k, PlaneMethod method, Allocator allocator)
        {
            int n = positions.Length;
            var posList = new List<float3>(n);
            for (int i = 0; i < n; i++) posList.Add(positions[i]);

            var triangles = new List<int3>();
            if (n >= 3)
            {
                k = math.clamp(k, 3, n);
                using var tree = KdTree3.Build(positions, Allocator.Persistent);
                var nbrIdx = new NativeArray<int>(k, Allocator.Persistent);
                var nbrDst = new NativeArray<float>(k, Allocator.Persistent);
                var processed = new bool[n];
                var groupPts = new List<float3>(k);

                for (int seed = 0; seed < n; seed++)
                {
                    if (processed[seed]) continue;

                    int got = tree.KNearest(positions[seed], nbrIdx, nbrDst);
                    groupPts.Clear();
                    var members = new int[got];
                    for (int i = 0; i < got; i++)
                    {
                        members[i] = nbrIdx[i];
                        groupPts.Add(positions[nbrIdx[i]]);
                        processed[nbrIdx[i]] = true;
                    }

                    TriangulatePatch(groupPts, members, method, triangles);
                }

                nbrIdx.Dispose();
                nbrDst.Dispose();
            }

            return MeshBuilder.Build(posList, triangles, allocator);
        }

        /// <summary>
        /// Same algorithm and <b>bit-identical result</b> as <see cref="Remesh(NativeArray{float3},int,PlaneMethod,Allocator)"/>,
        /// but parallelised: every vertex's k-NN is precomputed in one Burst-parallel pass
        /// (<see cref="KdTree3.KNearestAll"/>), the greedy grouping stays sequential (so the partition is
        /// unchanged), and the independent per-group plane/project/ear-clip work runs across threads. The
        /// triangle list is concatenated back in group order, so output equals the serial path exactly.
        /// </summary>
        public static MeshData RemeshParallel(NativeArray<float3> positions, int k, PlaneMethod method, Allocator allocator)
        {
            int n = positions.Length;
            var posList = new List<float3>(n);
            for (int i = 0; i < n; i++) posList.Add(positions[i]);

            var triangles = new List<int3>();
            if (n >= 3)
            {
                k = math.clamp(k, 3, n);
                using var tree = KdTree3.Build(positions, Allocator.Persistent);

                var knn = new NativeArray<int>(n * k, Allocator.Persistent);
                var knd = new NativeArray<float>(n * k, Allocator.Persistent);
                tree.KNearestAll(positions, k, knn, knd);

                // Managed copies so the parallel per-group work touches no NativeArray off the main thread.
                var pos = new float3[n];
                for (int i = 0; i < n; i++) pos[i] = positions[i];
                var knnM = new int[n * k];
                NativeArray<int>.Copy(knn, knnM);
                knn.Dispose();
                knd.Dispose();

                // Sequential greedy grouping (identical seed order and partition to the serial path).
                var processed = new bool[n];
                var groups = new List<int[]>();
                for (int seed = 0; seed < n; seed++)
                {
                    if (processed[seed]) continue;
                    var members = new int[k];
                    for (int i = 0; i < k; i++)
                    {
                        int idx = knnM[seed * k + i];
                        members[i] = idx;
                        processed[idx] = true;
                    }
                    groups.Add(members);
                }

                int g = groups.Count;
                var perGroup = new List<int3>[g];
                Parallel.For(0, g, gi =>
                {
                    int[] members = groups[gi];
                    var gp = new List<float3>(members.Length);
                    for (int i = 0; i < members.Length; i++) gp.Add(pos[members[i]]);

                    var list = new List<int3>();
                    TriangulatePatch(gp, members, method, list);
                    perGroup[gi] = list;
                });

                for (int gi = 0; gi < g; gi++) triangles.AddRange(perGroup[gi]);
            }

            return MeshBuilder.Build(posList, triangles, allocator);
        }

        /// <summary>
        /// Triangulate one neighbourhood: project onto its covariance plane, then — before ear-clipping —
        /// order the points by angle about the centroid so they form a simple (star-shaped) polygon. The
        /// raw k-NN distance order self-intersects, which is what made the surface a tangle of slivers;
        /// the angular order turns each patch into a clean fan that tiles with its neighbours.
        /// <paramref name="global"/> maps local point index → global vertex index (length = group size).
        /// </summary>
        static void TriangulatePatch(List<float3> groupPts, int[] global, PlaneMethod method, List<int3> outTris)
        {
            int got = groupPts.Count;
            if (got < 3) return;

            Covariance.Plane(groupPts, method, out float3 dim1, out float3 dim2, out _, out float3 center);

            var p2 = new float2[got];
            var order = new int[got];
            for (int i = 0; i < got; i++)
            {
                float3 rel = groupPts[i] - center;
                p2[i] = new float2(math.dot(dim1, rel), math.dot(dim2, rel));
                order[i] = i;
            }
            System.Array.Sort(order, (a, b) => math.atan2(p2[a].y, p2[a].x).CompareTo(math.atan2(p2[b].y, p2[b].x)));

            var poly = new List<float2>(got);
            for (int i = 0; i < got; i++) poly.Add(p2[order[i]]);

            int[] tri = EarClippingTriangulator.Triangulate(poly);
            for (int t = 0; t < tri.Length; t += 3)
                outTris.Add(new int3(global[order[tri[t]]], global[order[tri[t + 1]]], global[order[tri[t + 2]]]));
        }

        /// <summary>Remesh with the original homework method (covariance rows).</summary>
        public static MeshData Remesh(NativeArray<float3> positions, int k, Allocator allocator)
            => Remesh(positions, k, PlaneMethod.CovarianceRows, allocator);

        /// <summary>Remesh with the original neighbourhood size (k = 8) and method.</summary>
        public static MeshData Remesh(NativeArray<float3> positions, Allocator allocator)
            => Remesh(positions, DefaultK, PlaneMethod.CovarianceRows, allocator);
    }
}
