using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;
using CompGeo.Core;

namespace CompGeo.MeshProcessing
{
    /// <summary>
    /// Flatten a mesh by projecting every vertex onto its best-fit plane — a faithful port of the original
    /// CENG789 homework's <c>Model.UnfoldModel</c>. The plane comes from the global covariance rows
    /// (<see cref="Covariance.PlaneFromRows"/>, the homework's heuristic), and each vertex maps to
    /// <c>(dot(dim1, p−center), dot(dim2, p−center))</c>. This is the linear PCA-style flattening — distinct
    /// from <see cref="TutteEmbedding"/>, which solves a Laplacian system and preserves connectivity far
    /// better; both are kept so the original method stays available alongside the newer one.
    /// </summary>
    public static class PcaUnfold
    {
        /// <summary>
        /// Project all <paramref name="positions"/> onto the global covariance plane, writing the 2D result
        /// into <paramref name="outUv"/> (must be the same length as <paramref name="positions"/>).
        /// </summary>
        public static void Compute(NativeArray<float3> positions, NativeArray<float2> outUv)
        {
            int n = positions.Length;
            var pts = new List<float3>(n);
            for (int i = 0; i < n; i++) pts.Add(positions[i]);

            Covariance.PlaneFromRows(pts, out float3 dim1, out float3 dim2, out _, out float3 center);

            for (int i = 0; i < n; i++)
            {
                float3 rel = positions[i] - center;
                outUv[i] = new float2(math.dot(dim1, rel), math.dot(dim2, rel));
            }
        }
    }
}
