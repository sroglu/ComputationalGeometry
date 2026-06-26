using System.Collections.Generic;
using Unity.Mathematics;

namespace CompGeo.Core
{
    /// <summary>
    /// Covariance of a 3D point set and the local plane the original CENG789 homework derived from it.
    /// <para/>
    /// This is a faithful port of that homework's method, not an optimisation of it. The original used
    /// Accord.Statistics' <c>double[n,3].Covariance()</c> (the unbiased sample covariance, divisor n−1)
    /// and then — instead of a true eigen-decomposition — took the <b>rows</b> of the symmetric 3×3
    /// covariance as candidate directions, ordered them by their largest component, and built a frame
    /// from the top two rows. <see cref="Compute"/> reproduces that covariance; <see cref="PlaneFromRows"/>
    /// reproduces that covariance-row heuristic exactly (see <c>Model.cs:CalculatePlane / UnfoldModel</c>
    /// in the original repo).
    /// </summary>
    public static class Covariance
    {
        /// <summary>
        /// Unbiased sample covariance (divisor n−1) of <paramref name="points"/>, returned as the three
        /// rows of the symmetric 3×3 matrix plus the mean. Matches Accord's <c>.Covariance()</c>.
        /// </summary>
        public static void Compute(IReadOnlyList<float3> points, out float3 row0, out float3 row1, out float3 row2, out float3 mean)
        {
            int n = points.Count;
            mean = float3.zero;
            for (int i = 0; i < n; i++) mean += points[i];
            if (n > 0) mean /= n;

            double cxx = 0, cxy = 0, cxz = 0, cyy = 0, cyz = 0, czz = 0;
            for (int i = 0; i < n; i++)
            {
                float3 d = points[i] - mean;
                cxx += (double)d.x * d.x; cxy += (double)d.x * d.y; cxz += (double)d.x * d.z;
                cyy += (double)d.y * d.y; cyz += (double)d.y * d.z; czz += (double)d.z * d.z;
            }
            double inv = n > 1 ? 1.0 / (n - 1) : 0.0;
            row0 = new float3((float)(cxx * inv), (float)(cxy * inv), (float)(cxz * inv));
            row1 = new float3((float)(cxy * inv), (float)(cyy * inv), (float)(cyz * inv));
            row2 = new float3((float)(cxz * inv), (float)(cyz * inv), (float)(czz * inv));
        }

        /// <summary>
        /// Build the local frame the homework used: take the covariance rows, order them by their largest
        /// signed component descending (the original <c>OrderByDescending(v =&gt; Max(v.x,v.y,v.z))</c>),
        /// then <paramref name="dim1"/> = normalize(row0), <paramref name="normal"/> = normalize(row0 × row1),
        /// <paramref name="dim2"/> = dim1 × normal. <paramref name="center"/> is the point-set mean used as
        /// the projection origin. This reproduces the covariance-row heuristic, not a true PCA eigenframe.
        /// </summary>
        public static void PlaneFromRows(IReadOnlyList<float3> points, out float3 dim1, out float3 dim2, out float3 normal, out float3 center)
        {
            Compute(points, out float3 r0, out float3 r1, out float3 r2, out center);

            // Order the three rows by largest (signed) component, descending — the original heuristic.
            SortByMaxComponentDescending(ref r0, ref r1, ref r2);

            normal = math.normalizesafe(math.cross(r0, r1));
            dim1 = math.normalizesafe(r0);
            dim2 = math.cross(dim1, normal);
        }

        static void SortByMaxComponentDescending(ref float3 a, ref float3 b, ref float3 c)
        {
            if (MaxComp(b) > MaxComp(a)) (a, b) = (b, a);
            if (MaxComp(c) > MaxComp(b)) (b, c) = (c, b);
            if (MaxComp(b) > MaxComp(a)) (a, b) = (b, a);
        }

        // Matches UnityEngine Mathf.Max(v.x, v.y, v.z): the largest signed component (not magnitude).
        static float MaxComp(float3 v) => math.max(v.x, math.max(v.y, v.z));
    }
}
