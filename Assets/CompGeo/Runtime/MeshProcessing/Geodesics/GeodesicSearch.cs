using Unity.Collections;
using Unity.Mathematics;
using CompGeo.Collections;

namespace CompGeo.MeshProcessing.Geodesics
{
    /// <summary>
    /// Shared best-first shortest-path kernel over CSR adjacency with Euclidean edge weights — the
    /// single implementation behind both <see cref="DijkstraGeodesics"/> and <see cref="AStarGeodesics"/>
    /// (CODING-STYLE §6: one source of truth). Burst-friendly: integer ids, no delegates/boxing/managed
    /// allocation beyond the scratch heap (Temp).
    ///
    /// <para>Dijkstra and A* are the same algorithm with a different priority: the heap key is
    /// <c>f = g + h</c>, where <c>g</c> is the accumulated edge distance (stored in <c>dist</c>) and
    /// <c>h</c> is a heuristic lower bound on the remaining distance to the goal. Dijkstra is the
    /// <c>h = 0</c> case; A* uses the straight-line distance to the target, which is admissible and
    /// consistent for Euclidean edge weights, so the goal is optimal the moment it is popped.</para>
    /// </summary>
    internal static class GeodesicSearch
    {
        internal const int NoPredecessor = -1;

        /// <summary>
        /// Run the search from <paramref name="source"/>. When <paramref name="target"/> is negative the
        /// whole graph is swept (single-source); otherwise the search stops once the target is settled.
        /// When <paramref name="useHeuristic"/> is true the priority adds the Euclidean estimate to the
        /// target (A*); otherwise it is plain Dijkstra. Fills <paramref name="dist"/> with g-scores
        /// (+∞ where unreached) and <paramref name="pred"/> with predecessors (<see cref="NoPredecessor"/>
        /// at the source and at unreached vertices).
        /// </summary>
        internal static void Search(
            NativeArray<float3> positions,
            NativeArray<int> adjOffsets,
            NativeArray<int> adjNeighbours,
            int source,
            int target,
            bool useHeuristic,
            NativeArray<float> dist,
            NativeArray<int> pred)
        {
            int n = positions.Length;
            for (int i = 0; i < n; i++)
            {
                dist[i] = float.PositiveInfinity;
                pred[i] = NoPredecessor;
            }

            float3 goal = useHeuristic ? positions[target] : default;

            var heap = new NativeIndexedMinHeap(n, Allocator.Temp);
            dist[source] = 0f;
            heap.PushOrDecrease(source, useHeuristic ? math.distance(positions[source], goal) : 0f);

            while (!heap.IsEmpty)
            {
                int u = heap.Pop();    // settled: dist[u] (g) is final
                if (u == target) break; // target < 0 never matches -> full single-source sweep

                float gu = dist[u];
                float3 pu = positions[u];
                int start = adjOffsets[u];
                int end = adjOffsets[u + 1];
                for (int e = start; e < end; e++)
                {
                    int w = adjNeighbours[e];
                    float ng = gu + math.distance(pu, positions[w]);
                    if (ng < dist[w])
                    {
                        dist[w] = ng;
                        pred[w] = u;
                        float f = useHeuristic ? ng + math.distance(positions[w], goal) : ng;
                        heap.PushOrDecrease(w, f);
                    }
                }
            }

            heap.Dispose();
        }
    }
}
