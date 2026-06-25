using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using CompGeo.Core;

namespace CompGeo.Samples
{
    /// <summary>
    /// Shared helpers for the sample mesh sources: a self-contained procedural disk surface (used when no
    /// file is chosen) and normalisation/orientation of an arbitrary loaded mesh so it sits sensibly in
    /// view. Single source of truth for both (CODING-STYLE §6).
    /// </summary>
    public static class MeshFactory
    {
        /// <summary>Build a centred, non-planar disk-topology grid (a stand-in surface with a boundary).</summary>
        public static MeshData BuildProceduralDisk(Allocator allocator, int gridSize = 48, float spacing = 0.045f, float amp = 0.25f)
        {
            float half = (gridSize - 1) * spacing * 0.5f;
            var positions = new List<float3>(gridSize * gridSize);
            for (int j = 0; j < gridSize; j++)
            for (int i = 0; i < gridSize; i++)
            {
                float y = amp * math.sin(i * 0.5f) * math.cos(j * 0.5f);
                positions.Add(new float3(i * spacing - half, y, j * spacing - half));
            }

            var triangles = new List<int3>((gridSize - 1) * (gridSize - 1) * 2);
            for (int j = 0; j < gridSize - 1; j++)
            for (int i = 0; i < gridSize - 1; i++)
            {
                int a = j * gridSize + i;
                int b = j * gridSize + i + 1;
                int c = (j + 1) * gridSize + i;
                int d = (j + 1) * gridSize + i + 1;
                triangles.Add(new int3(a, b, d));
                triangles.Add(new int3(a, d, c));
            }

            return MeshBuilder.Build(positions, triangles, allocator);
        }

        /// <summary>
        /// Lay an arbitrary mesh down for viewing: rotate its dominant (area-weighted) face normal to +Y,
        /// then centre on the origin and uniformly scale to fit ~2 units. Topology/indices untouched.
        /// </summary>
        public static void NormalizeAndOrient(ref MeshData m)
        {
            int n = m.VertexCount;

            // Dominant (area-weighted) normal vs total area. For an open relief surface (a face) the
            // normals reinforce, so the sum is a large fraction of the area → reorient it to face +Y.
            // For a closed surface (man0, horse0) the normals cancel (sum ≈ 0) → leave the orientation as
            // authored rather than rotating by a meaningless near-zero vector.
            float3 nrmSum = float3.zero;
            float areaSum = 0f;
            for (int t = 0; t < m.TriangleCount; t++)
            {
                int3 tri = m.Triangles[t];
                float3 c = math.cross(m.Positions[tri.y] - m.Positions[tri.x], m.Positions[tri.z] - m.Positions[tri.x]);
                nrmSum += c;
                areaSum += math.length(c);
            }
            if (areaSum > 1e-9f && math.length(nrmSum) / areaSum > 0.5f)
            {
                Quaternion q = Quaternion.FromToRotation((Vector3)math.normalize(nrmSum), Vector3.up);
                for (int i = 0; i < n; i++) m.Positions[i] = (float3)(q * (Vector3)(float3)m.Positions[i]);
            }

            float3 mn = m.Positions[0], mx = m.Positions[0];
            for (int i = 1; i < n; i++) { mn = math.min(mn, m.Positions[i]); mx = math.max(mx, m.Positions[i]); }
            float3 centre = (mn + mx) * 0.5f;
            float maxExtent = math.cmax(mx - mn);
            float scale = maxExtent > 1e-9f ? 2f / maxExtent : 1f;
            for (int i = 0; i < n; i++) m.Positions[i] = (m.Positions[i] - centre) * scale;
        }
    }
}
