using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using CompGeo.Core;

namespace CompGeo.MeshProcessing.Geodesics
{
    /// <summary>
    /// Single-source discrete geodesics on a triangle mesh: shortest paths over the vertex/edge graph
    /// with Euclidean edge weights (Dijkstra). This is the graph-distance approximation of HW1 — an
    /// upper bound on the true surface geodesic; Fast Marching / the Heat Method are the planned
    /// upgrades for true geodesics (docs/MIGRATION.md §3).
    ///
    /// <para><b>Provenance (clean-room):</b> implemented from the textbook Dijkstra specification
    /// (Dijkstra 1959) on a deliberately data-oriented structure — CSR adjacency, <c>dist[]</c>/<c>pred[]</c>
    /// arrays over integer vertex ids, and a decrease-key indexed min-heap. It shares
    /// NO code lineage with the original course project's <c>PriorityQueue&lt;double,Path&gt;</c> +
    /// <c>HashSet</c> A*/Dijkstra (the unlicensed "THeK3nger" gist); only the public algorithm is reused.
    /// See ProjectFoundation FOUNDATION-MIGRATION-REPORT.md §DataStructures.</para>
    /// </summary>
    public static class DijkstraGeodesics
    {
        /// <summary>Sentinel <c>pred</c> value for the source and for unreachable vertices.</summary>
        public const int NoPredecessor = -1;

        /// <summary>
        /// Compute single-source shortest-path distances over the mesh's vertex graph from
        /// <paramref name="source"/>. Fills <paramref name="dist"/> (graph distance, +∞ if unreachable)
        /// and <paramref name="pred"/> (predecessor vertex on the shortest path, or
        /// <see cref="NoPredecessor"/>). Outputs must be sized to the vertex count.
        ///
        /// Core kernel: Burst-friendly (integer ids, no delegates/boxing/managed allocation), so it
        /// runs identically when called directly (tests) or from inside <see cref="DijkstraJob"/>.
        /// </summary>
        public static void Compute(in MeshData mesh, int source, NativeArray<float> dist, NativeArray<int> pred)
            => Compute(mesh.Positions, mesh.AdjOffsets, mesh.AdjNeighbours, source, dist, pred);

        /// <summary>Raw-array overload (the form a Burst job passes its <c>[ReadOnly]</c> fields to).</summary>
        public static void Compute(
            NativeArray<float3> positions,
            NativeArray<int> adjOffsets,
            NativeArray<int> adjNeighbours,
            int source,
            NativeArray<float> dist,
            NativeArray<int> pred)
            => GeodesicSearch.Search(positions, adjOffsets, adjNeighbours, source, -1, false, dist, pred);

        /// <summary>
        /// Allocate distance/predecessor outputs and run the Burst-compiled <see cref="DijkstraJob"/>
        /// on the calling thread (<c>.Run()</c>). Single-source Dijkstra is an inherently sequential
        /// frontier, so Burst-CPU is the right tool for interactive single queries (docs/MIGRATION.md §5b).
        /// The caller owns and must dispose <paramref name="dist"/> and <paramref name="pred"/>.
        /// </summary>
        public static void Run(
            in MeshData mesh,
            int source,
            Allocator allocator,
            out NativeArray<float> dist,
            out NativeArray<int> pred)
        {
            int n = mesh.VertexCount;
            dist = new NativeArray<float>(n, allocator, NativeArrayOptions.UninitializedMemory);
            pred = new NativeArray<int>(n, allocator, NativeArrayOptions.UninitializedMemory);

            new DijkstraJob
            {
                Positions = mesh.Positions,
                AdjOffsets = mesh.AdjOffsets,
                AdjNeighbours = mesh.AdjNeighbours,
                Source = source,
                Dist = dist,
                Pred = pred,
            }.Run();
        }

        /// <summary>
        /// Reconstruct the vertex path from <paramref name="source"/> to <paramref name="target"/> from a
        /// <c>pred</c> array, ordered source → target. Returns an empty list when <paramref name="target"/>
        /// is unreachable. The caller owns the returned list.
        /// </summary>
        public static NativeList<int> ReconstructPath(
            NativeArray<int> pred,
            int source,
            int target,
            Allocator allocator)
        {
            var path = new NativeList<int>(allocator);

            int cur = target;
            while (cur != NoPredecessor)
            {
                path.Add(cur);
                if (cur == source) break;
                cur = pred[cur];
            }

            // Did not trace back to the source -> target is unreachable.
            if (path.Length == 0 || path[path.Length - 1] != source)
            {
                path.Clear();
                return path;
            }

            for (int i = 0, j = path.Length - 1; i < j; i++, j--)
            {
                int tmp = path[i];
                path[i] = path[j];
                path[j] = tmp;
            }
            return path;
        }
    }

    /// <summary>
    /// Burst-compiled wrapper around <see cref="DijkstraGeodesics.Compute(NativeArray{float3},NativeArray{int},NativeArray{int},int,NativeArray{float},NativeArray{int})"/>.
    /// </summary>
    [BurstCompile]
    public struct DijkstraJob : IJob
    {
        [ReadOnly] public NativeArray<float3> Positions;
        [ReadOnly] public NativeArray<int> AdjOffsets;
        [ReadOnly] public NativeArray<int> AdjNeighbours;
        public int Source;

        public NativeArray<float> Dist;
        public NativeArray<int> Pred;

        public void Execute()
            => DijkstraGeodesics.Compute(Positions, AdjOffsets, AdjNeighbours, Source, Dist, Pred);
    }
}
