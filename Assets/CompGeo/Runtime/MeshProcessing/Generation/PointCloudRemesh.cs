using System.Collections.Generic;
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
        /// triangles regenerated). The caller owns and must dispose the result.
        /// </summary>
        public static MeshData Remesh(NativeArray<float3> positions, int k, Allocator allocator)
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
                var pts2d = new List<float2>(k);

                for (int seed = 0; seed < n; seed++)
                {
                    if (processed[seed]) continue;

                    int got = tree.KNearest(positions[seed], nbrIdx, nbrDst);
                    groupPts.Clear();
                    for (int i = 0; i < got; i++)
                    {
                        groupPts.Add(positions[nbrIdx[i]]);
                        processed[nbrIdx[i]] = true;
                    }
                    if (got < 3) continue;

                    Covariance.PlaneFromRows(groupPts, out float3 dim1, out float3 dim2, out _, out float3 center);

                    pts2d.Clear();
                    for (int i = 0; i < got; i++)
                    {
                        float3 rel = groupPts[i] - center;
                        pts2d.Add(new float2(math.dot(dim1, rel), math.dot(dim2, rel)));
                    }

                    int[] tri = EarClippingTriangulator.Triangulate(pts2d);
                    for (int t = 0; t < tri.Length; t += 3)
                        triangles.Add(new int3(nbrIdx[tri[t]], nbrIdx[tri[t + 1]], nbrIdx[tri[t + 2]]));
                }

                nbrIdx.Dispose();
                nbrDst.Dispose();
            }

            return MeshBuilder.Build(posList, triangles, allocator);
        }

        /// <summary>Remesh with the original neighbourhood size (k = 8).</summary>
        public static MeshData Remesh(NativeArray<float3> positions, Allocator allocator)
            => Remesh(positions, DefaultK, allocator);
    }
}
