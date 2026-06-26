using System.Collections.Generic;
using System.Threading.Tasks;
using Unity.Collections;
using Unity.Mathematics;
using CompGeo.Core;
using CompGeo.Collections;

namespace CompGeo.MeshProcessing
{
    /// <summary>Reconstruction variants offered by the Remesh dropdown.</summary>
    public enum RemeshMode
    {
        /// <summary>The faithful homework: union of independent per-neighbourhood ear-clipped patches (a soup).</summary>
        Original,
        /// <summary>The improved, near-manifold mutual-agreement reconstruction.</summary>
        Improved,
    }

    /// <summary>
    /// Rebuilds surface connectivity from a raw point set (the CENG789 "Mesh Generation" part). The
    /// original homework unioned an independent triangulation of every point's k-NN neighbourhood, which
    /// is a non-manifold <i>triangle soup</i>: overlapping patches never share edges, so dense meshes
    /// render as a tangle. This keeps the same per-neighbourhood primitives — k-d tree k-NN, covariance
    /// local plane, 2D triangulation — but reconstructs a coherent surface by <b>mutual agreement</b>:
    /// for every vertex it builds the Delaunay "umbrella" of its neighbourhood and keeps only the
    /// triangles that <b>two or more</b> of their vertices' umbrellas independently produced. Spurious
    /// bridging triangles (seen by a single umbrella) drop out, leaving a near-manifold mesh.
    /// </summary>
    public static class PointCloudRemesh
    {
        public const int DefaultK = 8;

        const int Bits = 21;                 // vertex indices must fit in 21 bits (≤ ~2.1M vertices)
        const long Mask = (1L << Bits) - 1;

        /// <summary>
        /// Reconstruct the surface: <see cref="RemeshMode.Original"/> = the homework's patch-union soup,
        /// <see cref="RemeshMode.Improved"/> = the mutual-agreement near-manifold mesh. Both use the
        /// homework's covariance-row local plane.
        /// </summary>
        public static MeshData Remesh(NativeArray<float3> positions, int k, RemeshMode mode, Allocator allocator)
            => mode == RemeshMode.Original
                ? BuildSoup(positions, k, allocator)
                : Build(positions, k, PlaneMethod.CovarianceRows, allocator, parallel: true);

        public static MeshData Remesh(NativeArray<float3> positions, int k, PlaneMethod method, Allocator allocator)
            => Build(positions, k, method, allocator, parallel: false);

        /// <summary>
        /// The faithful original "Mesh Generation": greedily group each unprocessed point's k-NN, fit the
        /// covariance-row plane, ear-clip the projected neighbourhood, and union all patches — overlapping,
        /// non-manifold (a triangle soup), exactly as the homework did.
        /// </summary>
        static MeshData BuildSoup(NativeArray<float3> positions, int k, Allocator allocator)
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
                    var members = new int[got];
                    for (int i = 0; i < got; i++)
                    {
                        members[i] = nbrIdx[i];
                        groupPts.Add(positions[nbrIdx[i]]);
                        processed[nbrIdx[i]] = true;
                    }
                    if (got < 3) continue;

                    Covariance.Plane(groupPts, PlaneMethod.CovarianceRows, out float3 d1, out float3 d2, out _, out float3 c);
                    pts2d.Clear();
                    for (int i = 0; i < got; i++)
                    {
                        float3 rel = groupPts[i] - c;
                        pts2d.Add(new float2(math.dot(d1, rel), math.dot(d2, rel)));
                    }

                    int[] tri = EarClippingTriangulator.Triangulate(pts2d);
                    for (int t = 0; t < tri.Length; t += 3)
                        triangles.Add(new int3(members[tri[t]], members[tri[t + 1]], members[tri[t + 2]]));
                }

                nbrIdx.Dispose();
                nbrDst.Dispose();
            }
            return MeshBuilder.Build(posList, triangles, allocator);
        }

        /// <summary>
        /// Same result as <see cref="Remesh(NativeArray{float3},int,PlaneMethod,Allocator)"/> — the
        /// per-vertex umbrellas are independent, so they run across threads while the agreement count and
        /// the final (sorted) triangle list stay deterministic.
        /// </summary>
        public static MeshData RemeshParallel(NativeArray<float3> positions, int k, PlaneMethod method, Allocator allocator)
            => Build(positions, k, method, allocator, parallel: true);

        static MeshData Build(NativeArray<float3> positions, int k, PlaneMethod method, Allocator allocator, bool parallel)
        {
            int n = positions.Length;
            var posList = new List<float3>(n);
            for (int i = 0; i < n; i++) posList.Add(positions[i]);

            var triangles = new List<int3>();
            if (n >= 3 && n <= Mask)
            {
                k = math.clamp(k, 3, n);
                using var tree = KdTree3.Build(positions, Allocator.Persistent);

                var knn = new NativeArray<int>(n * k, Allocator.Persistent);
                var knd = new NativeArray<float>(n * k, Allocator.Persistent);
                tree.KNearestAll(positions, k, knn, knd);

                var pos = new float3[n];
                for (int i = 0; i < n; i++) pos[i] = positions[i];
                var knnM = new int[n * k];
                NativeArray<int>.Copy(knn, knnM);
                knn.Dispose();
                knd.Dispose();

                // Each vertex's umbrella = the sorted-triple keys of the Delaunay triangles incident to it.
                var perVertex = new List<long>[n];
                if (parallel)
                    Parallel.For(0, n, v => perVertex[v] = UmbrellaKeys(v, k, knnM, pos, method));
                else
                    for (int v = 0; v < n; v++) perVertex[v] = UmbrellaKeys(v, k, knnM, pos, method);

                // Count how many umbrellas produced each triangle; keep the mutually-agreed ones.
                var support = new Dictionary<long, int>(n * 4);
                for (int v = 0; v < n; v++)
                {
                    var keys = perVertex[v];
                    for (int i = 0; i < keys.Count; i++)
                    {
                        support.TryGetValue(keys[i], out int c);
                        support[keys[i]] = c + 1;
                    }
                }

                var kept = new List<long>(support.Count);
                foreach (var kv in support)
                    if (kv.Value >= 2) kept.Add(kv.Key);
                kept.Sort(); // deterministic triangle order, independent of threading / dictionary order

                for (int i = 0; i < kept.Count; i++) triangles.Add(Decode(kept[i]));
            }

            return MeshBuilder.Build(posList, triangles, allocator);
        }

        /// <summary>
        /// Build vertex <paramref name="v"/>'s umbrella: project its k-NN onto their covariance plane,
        /// 2D-Delaunay them, and return the sorted global-index keys of the triangles incident to v
        /// (dropping any triangle whose longest edge far exceeds the patch's median edge — a bridge).
        /// </summary>
        static List<long> UmbrellaKeys(int v, int k, int[] knnM, float3[] pos, PlaneMethod method)
        {
            var members = new int[k];
            var gp = new List<float3>(k);
            for (int i = 0; i < k; i++) { members[i] = knnM[v * k + i]; gp.Add(pos[members[i]]); }

            Covariance.Plane(gp, method, out float3 dim1, out float3 dim2, out _, out float3 center);

            var p2 = new float2[k];
            for (int i = 0; i < k; i++)
            {
                float3 rel = gp[i] - center;
                p2[i] = new float2(math.dot(dim1, rel), math.dot(dim2, rel));
            }

            int[] tri = DelaunayTriangulator.Triangulate(p2);
            var keys = new List<long>(tri.Length / 3);
            if (tri.Length == 0) return keys;

            // Median edge length of this patch, for the bridge filter.
            var edges = new List<float>(tri.Length);
            for (int t = 0; t < tri.Length; t += 3)
            {
                edges.Add(math.distance(p2[tri[t]], p2[tri[t + 1]]));
                edges.Add(math.distance(p2[tri[t + 1]], p2[tri[t + 2]]));
                edges.Add(math.distance(p2[tri[t + 2]], p2[tri[t]]));
            }
            edges.Sort();
            float thresh = 2.5f * edges[edges.Count / 2];

            for (int t = 0; t < tri.Length; t += 3)
            {
                int la = tri[t], lb = tri[t + 1], lc = tri[t + 2];
                if (la != 0 && lb != 0 && lc != 0) continue; // v is local index 0 — keep only its triangles

                float e0 = math.distance(p2[la], p2[lb]);
                float e1 = math.distance(p2[lb], p2[lc]);
                float e2 = math.distance(p2[lc], p2[la]);
                if (math.max(e0, math.max(e1, e2)) > thresh) continue;

                keys.Add(Encode(members[la], members[lb], members[lc]));
            }
            return keys;
        }

        // Pack a triangle's three global indices (sorted ascending) into one long, so identical triangles
        // from different umbrellas hash to the same key regardless of vertex order.
        static long Encode(int a, int b, int c)
        {
            if (a > b) (a, b) = (b, a);
            if (b > c) (b, c) = (c, b);
            if (a > b) (a, b) = (b, a);
            return ((long)a << (2 * Bits)) | ((long)b << Bits) | (long)c;
        }

        static int3 Decode(long key)
            => new int3((int)(key >> (2 * Bits)), (int)((key >> Bits) & Mask), (int)(key & Mask));

        /// <summary>Remesh with the original homework method (covariance rows).</summary>
        public static MeshData Remesh(NativeArray<float3> positions, int k, Allocator allocator)
            => Remesh(positions, k, PlaneMethod.CovarianceRows, allocator);

        /// <summary>Remesh with the original neighbourhood size (k = 8) and method.</summary>
        public static MeshData Remesh(NativeArray<float3> positions, Allocator allocator)
            => Remesh(positions, DefaultK, PlaneMethod.CovarianceRows, allocator);
    }
}
