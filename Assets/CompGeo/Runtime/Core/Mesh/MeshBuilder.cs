using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;

namespace CompGeo.Core
{
    /// <summary>
    /// Constructs <see cref="MeshData"/> from raw geometry and computes its CSR vertex-vertex
    /// adjacency. Topology construction is inherently sequential, so this stays on the managed CPU
    /// side (see docs/MIGRATION.md §5b); the resulting native arrays are then Burst-consumable.
    /// </summary>
    public static class MeshBuilder
    {
        /// <summary>
        /// Build a <see cref="MeshData"/> from <paramref name="positions"/> and
        /// <paramref name="triangles"/>, allocating native storage with <paramref name="allocator"/>
        /// and computing CSR vertex-vertex adjacency.
        /// </summary>
        public static MeshData Build(
            IReadOnlyList<float3> positions,
            IReadOnlyList<int3> triangles,
            Allocator allocator)
        {
            var mesh = new MeshData
            {
                Positions = new NativeArray<float3>(positions.Count, allocator),
                Triangles = new NativeArray<int3>(triangles.Count, allocator),
            };
            for (int i = 0; i < positions.Count; i++) mesh.Positions[i] = positions[i];
            for (int i = 0; i < triangles.Count; i++) mesh.Triangles[i] = triangles[i];

            BuildAdjacency(ref mesh, allocator);
            return mesh;
        }

        /// <summary>
        /// Compute the undirected vertex-vertex adjacency of <paramref name="mesh"/> from its
        /// triangles and store it as CSR. Neighbour lists are sorted ascending so the layout is
        /// deterministic (important for regression fixtures). Any pre-existing adjacency arrays are
        /// disposed and replaced. Runs in O(V + E) up to the per-vertex neighbour sort.
        /// </summary>
        public static void BuildAdjacency(ref MeshData mesh, Allocator allocator)
        {
            int v = mesh.VertexCount;

            // Collect the unique neighbour set of every vertex.
            var sets = new HashSet<int>[v];
            for (int i = 0; i < v; i++) sets[i] = new HashSet<int>();

            for (int t = 0; t < mesh.TriangleCount; t++)
            {
                int3 tri = mesh.Triangles[t];
                AddEdge(sets, tri.x, tri.y);
                AddEdge(sets, tri.y, tri.z);
                AddEdge(sets, tri.z, tri.x);
            }

            // CSR row offsets (prefix sum of degrees).
            var offsets = new NativeArray<int>(v + 1, allocator);
            int total = 0;
            for (int i = 0; i < v; i++)
            {
                offsets[i] = total;
                total += sets[i].Count;
            }
            offsets[v] = total;

            // Flatten sorted neighbour lists into the CSR neighbour array.
            var neighbours = new NativeArray<int>(total, allocator);
            var scratch = new List<int>();
            for (int i = 0; i < v; i++)
            {
                scratch.Clear();
                scratch.AddRange(sets[i]);
                scratch.Sort();

                int w = offsets[i];
                for (int k = 0; k < scratch.Count; k++) neighbours[w + k] = scratch[k];
            }

            if (mesh.AdjOffsets.IsCreated) mesh.AdjOffsets.Dispose();
            if (mesh.AdjNeighbours.IsCreated) mesh.AdjNeighbours.Dispose();
            mesh.AdjOffsets = offsets;
            mesh.AdjNeighbours = neighbours;
        }

        static void AddEdge(HashSet<int>[] sets, int a, int b)
        {
            if (a == b) return;
            sets[a].Add(b);
            sets[b].Add(a);
        }
    }
}
