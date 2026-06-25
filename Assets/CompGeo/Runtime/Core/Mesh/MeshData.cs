using System;
using Unity.Collections;
using Unity.Mathematics;

namespace CompGeo.Core
{
    /// <summary>
    /// Render-independent, Burst-friendly triangle mesh in Structure-of-Arrays (SoA) form.
    ///
    /// Layout:
    ///  - <see cref="Positions"/> : one <see cref="float3"/> per vertex.
    ///  - <see cref="Triangles"/> : one <see cref="int3"/> (vertex indices) per face.
    ///  - Vertex-vertex adjacency in CSR (Compressed Sparse Row) form: the neighbours of vertex
    ///    <c>v</c> are <c>AdjNeighbours[AdjOffsets[v] .. AdjOffsets[v + 1]]</c>.
    ///
    /// All storage is held in <see cref="NativeArray{T}"/> so the data can be consumed directly by
    /// Burst jobs without GC pressure or pointer-chasing (this replaces the old class-per-element
    /// AoS layout — see docs/MIGRATION.md §2.B / §3). The owner must call <see cref="Dispose"/>.
    /// </summary>
    public struct MeshData : IDisposable
    {
        /// <summary>One position per vertex.</summary>
        public NativeArray<float3> Positions;

        /// <summary>One triangle (three vertex indices) per face.</summary>
        public NativeArray<int3> Triangles;

        /// <summary>CSR row offsets into <see cref="AdjNeighbours"/>; length is <c>VertexCount + 1</c>.</summary>
        public NativeArray<int> AdjOffsets;

        /// <summary>
        /// CSR neighbour indices. Each undirected edge (a, b) is stored in both directions, so the
        /// total length is twice the unique edge count.
        /// </summary>
        public NativeArray<int> AdjNeighbours;

        public int VertexCount => Positions.IsCreated ? Positions.Length : 0;

        public int TriangleCount => Triangles.IsCreated ? Triangles.Length : 0;

        public bool HasAdjacency => AdjOffsets.IsCreated && AdjNeighbours.IsCreated;

        /// <summary>Number of neighbours of vertex <paramref name="v"/>.</summary>
        public int Degree(int v) => AdjOffsets[v + 1] - AdjOffsets[v];

        /// <summary>
        /// Neighbour slice of vertex <paramref name="v"/> as a (<paramref name="start"/>,
        /// <paramref name="count"/>) range into <see cref="AdjNeighbours"/>.
        /// </summary>
        public void GetNeighbours(int v, out int start, out int count)
        {
            start = AdjOffsets[v];
            count = AdjOffsets[v + 1] - start;
        }

        public void Dispose()
        {
            if (Positions.IsCreated) Positions.Dispose();
            if (Triangles.IsCreated) Triangles.Dispose();
            if (AdjOffsets.IsCreated) AdjOffsets.Dispose();
            if (AdjNeighbours.IsCreated) AdjNeighbours.Dispose();
        }
    }
}
