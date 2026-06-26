using System.Collections.Generic;
using Unity.Mathematics;

namespace CompGeo.Core
{
    /// <summary>How to turn a point set's covariance into a local plane frame.</summary>
    public enum PlaneMethod
    {
        /// <summary>The original homework's covariance-row heuristic (<see cref="Covariance.PlaneFromRows"/>).</summary>
        CovarianceRows,
        /// <summary>True PCA: eigenvectors of the covariance (<see cref="Covariance.PlaneFromEigen"/>).</summary>
        Eigenvectors,
    }

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

        /// <summary>
        /// The "true PCA" frame the user originally intended by "eigenvector": eigenvectors of the
        /// symmetric covariance (via Jacobi). The two largest-eigenvalue directions span the tangent plane
        /// (<paramref name="dim1"/>, <paramref name="dim2"/>); the smallest is the <paramref name="normal"/>.
        /// Mathematically correct, unlike <see cref="PlaneFromRows"/> — offered as the alternative method.
        /// </summary>
        public static void PlaneFromEigen(IReadOnlyList<float3> points, out float3 dim1, out float3 dim2, out float3 normal, out float3 center)
        {
            Compute(points, out float3 r0, out float3 r1, out float3 r2, out center);
            JacobiEigenSymmetric(r0, r1, r2, out float3 v0, out float3 v1, out float3 v2);
            dim1 = math.normalizesafe(v0);
            normal = math.normalizesafe(v2);
            dim2 = math.cross(normal, dim1); // complete a right-handed frame in the tangent plane
        }

        /// <summary>Dispatch to the covariance-row heuristic or the eigenvector PCA per <paramref name="method"/>.</summary>
        public static void Plane(IReadOnlyList<float3> points, PlaneMethod method, out float3 dim1, out float3 dim2, out float3 normal, out float3 center)
        {
            if (method == PlaneMethod.Eigenvectors) PlaneFromEigen(points, out dim1, out dim2, out normal, out center);
            else PlaneFromRows(points, out dim1, out dim2, out normal, out center);
        }

        /// <summary>
        /// Eigen-decomposition of the symmetric 3×3 with rows <paramref name="r0"/>,<paramref name="r1"/>,
        /// <paramref name="r2"/> via cyclic Jacobi rotations; outputs the eigenvectors ordered by
        /// eigenvalue descending (v0 largest … v2 smallest).
        /// </summary>
        static void JacobiEigenSymmetric(float3 r0, float3 r1, float3 r2, out float3 v0, out float3 v1, out float3 v2)
        {
            double a00 = r0.x, a01 = r0.y, a02 = r0.z, a11 = r1.y, a12 = r1.z, a22 = r2.z;
            double v00 = 1, v01 = 0, v02 = 0, v10 = 0, v11 = 1, v12 = 0, v20 = 0, v21 = 0, v22 = 1;

            for (int sweep = 0; sweep < 50; sweep++)
            {
                double off = math.abs(a01) + math.abs(a02) + math.abs(a12);
                if (off < 1e-12) break;

                Rotate(ref a00, ref a11, ref a01, ref a02, ref a12, ref v00, ref v10, ref v20, ref v01, ref v11, ref v21); // (0,1)
                Rotate(ref a00, ref a22, ref a02, ref a01, ref a12, ref v00, ref v10, ref v20, ref v02, ref v12, ref v22); // (0,2)
                Rotate(ref a11, ref a22, ref a12, ref a01, ref a02, ref v01, ref v11, ref v21, ref v02, ref v12, ref v22); // (1,2)
            }

            var pairs = new[]
            {
                (val: a00, vec: new float3((float)v00, (float)v10, (float)v20)),
                (val: a11, vec: new float3((float)v01, (float)v11, (float)v21)),
                (val: a22, vec: new float3((float)v02, (float)v12, (float)v22)),
            };
            if (pairs[1].val > pairs[0].val) (pairs[0], pairs[1]) = (pairs[1], pairs[0]);
            if (pairs[2].val > pairs[1].val) (pairs[1], pairs[2]) = (pairs[2], pairs[1]);
            if (pairs[1].val > pairs[0].val) (pairs[0], pairs[1]) = (pairs[1], pairs[0]);
            v0 = pairs[0].vec; v1 = pairs[1].vec; v2 = pairs[2].vec;
        }

        // One Jacobi rotation that zeroes the off-diagonal app/aqq coupling apq, updating the diagonal
        // (app,aqq), the three off-diagonals (apq, and the spectators apr,aqr), and the eigenvector columns.
        static void Rotate(ref double app, ref double aqq, ref double apq, ref double apr, ref double aqr,
                           ref double vp0, ref double vp1, ref double vp2, ref double vq0, ref double vq1, ref double vq2)
        {
            if (math.abs(apq) < 1e-300) return;
            double theta = (aqq - app) / (2.0 * apq);
            double t = math.sign(theta) / (math.abs(theta) + math.sqrt(theta * theta + 1.0));
            if (theta == 0.0) t = 1.0;
            double c = 1.0 / math.sqrt(t * t + 1.0);
            double s = t * c;

            double newApp = app - t * apq;
            double newAqq = aqq + t * apq;
            app = newApp; aqq = newAqq; apq = 0.0;

            double pr = apr, qr = aqr;
            apr = c * pr - s * qr;
            aqr = s * pr + c * qr;

            double p0 = vp0, p1 = vp1, p2 = vp2, q0 = vq0, q1 = vq1, q2 = vq2;
            vp0 = c * p0 - s * q0; vq0 = s * p0 + c * q0;
            vp1 = c * p1 - s * q1; vq1 = s * p1 + c * q1;
            vp2 = c * p2 - s * q2; vq2 = s * p2 + c * q2;
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
