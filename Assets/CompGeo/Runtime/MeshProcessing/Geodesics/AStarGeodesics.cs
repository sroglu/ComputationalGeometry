using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using CompGeo.Core;

namespace CompGeo.MeshProcessing.Geodesics
{
    /// <summary>
    /// Point-to-point discrete geodesics via A* over the vertex/edge graph with Euclidean edge weights.
    /// A* is Dijkstra guided toward a single goal by an admissible heuristic — here the straight-line
    /// distance to the target, which can never exceed the actual graph distance (and is consistent), so
    /// the search settles the target optimally while typically touching far fewer vertices than a full
    /// Dijkstra sweep. Use this for single source→target queries; use <see cref="DijkstraGeodesics"/> for
    /// one-to-all distance fields.
    ///
    /// <para><b>Provenance (clean-room):</b> the original course project's A* (admissible Euclidean
    /// estimate over an <c>IHasNeighbours</c> graph) is reimplemented from the textbook A* spec
    /// (Hart–Nilsson–Raphael 1968) on the shared data-oriented <see cref="GeodesicSearch"/> kernel — CSR
    /// + <c>dist</c>/<c>pred</c> + decrease-key heap. No code lineage with the unlicensed gist; only the
    /// public algorithm is reused (docs/MIGRATION.md §5; FOUNDATION-MIGRATION-REPORT.md §DataStructures).</para>
    /// </summary>
    public static class AStarGeodesics
    {
        /// <summary>Sentinel <c>pred</c> value for the source and for unreached vertices.</summary>
        public const int NoPredecessor = -1;

        /// <summary>
        /// Compute the shortest path cost from <paramref name="source"/> to <paramref name="target"/>,
        /// filling <paramref name="dist"/> (g-scores; +∞ where the search did not reach) and
        /// <paramref name="pred"/>. Only the vertices A* expanded before settling the target are
        /// finalized — the rest stay +∞. Outputs must be sized to the vertex count.
        /// </summary>
        public static void Compute(in MeshData mesh, int source, int target, NativeArray<float> dist, NativeArray<int> pred)
            => GeodesicSearch.Search(mesh.Positions, mesh.AdjOffsets, mesh.AdjNeighbours, source, target, true, dist, pred);

        /// <summary>Raw-array overload (the form a Burst job passes its <c>[ReadOnly]</c> fields to).</summary>
        public static void Compute(
            NativeArray<float3> positions,
            NativeArray<int> adjOffsets,
            NativeArray<int> adjNeighbours,
            int source,
            int target,
            NativeArray<float> dist,
            NativeArray<int> pred)
            => GeodesicSearch.Search(positions, adjOffsets, adjNeighbours, source, target, true, dist, pred);

        /// <summary>
        /// Run Burst-compiled A* and return the source→target path (empty when unreachable) plus its
        /// total cost (+∞ when unreachable). The caller owns and must dispose <paramref name="path"/>.
        /// </summary>
        public static float FindPath(in MeshData mesh, int source, int target, Allocator allocator, out NativeList<int> path)
        {
            int n = mesh.VertexCount;
            var dist = new NativeArray<float>(n, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            var pred = new NativeArray<int>(n, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            try
            {
                new AStarJob
                {
                    Positions = mesh.Positions,
                    AdjOffsets = mesh.AdjOffsets,
                    AdjNeighbours = mesh.AdjNeighbours,
                    Source = source,
                    Target = target,
                    Dist = dist,
                    Pred = pred,
                }.Run();

                path = DijkstraGeodesics.ReconstructPath(pred, source, target, allocator);
                return dist[target];
            }
            finally
            {
                dist.Dispose();
                pred.Dispose();
            }
        }
    }

    /// <summary>Burst-compiled wrapper around <see cref="AStarGeodesics.Compute(NativeArray{float3},NativeArray{int},NativeArray{int},int,int,NativeArray{float},NativeArray{int})"/>.</summary>
    [BurstCompile]
    public struct AStarJob : IJob
    {
        [ReadOnly] public NativeArray<float3> Positions;
        [ReadOnly] public NativeArray<int> AdjOffsets;
        [ReadOnly] public NativeArray<int> AdjNeighbours;
        public int Source;
        public int Target;

        public NativeArray<float> Dist;
        public NativeArray<int> Pred;

        public void Execute()
            => AStarGeodesics.Compute(Positions, AdjOffsets, AdjNeighbours, Source, Target, Dist, Pred);
    }
}
