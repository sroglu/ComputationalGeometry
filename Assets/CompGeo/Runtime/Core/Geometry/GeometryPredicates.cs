using Unity.Mathematics;

namespace CompGeo.Core
{
    /// <summary>
    /// Basic geometric predicates and triangle helpers used across the core.
    ///
    /// The 2D predicates use the standard determinant formulations in single precision; they are
    /// fast but not exact/robust near-degenerate. A robust (adaptive-precision) variant can replace
    /// these later when convex-hull / Delaunay land — see docs/MIGRATION.md §5c (&lt;future&gt;).
    /// </summary>
    public static class GeometryPredicates
    {
        /// <summary>
        /// Orientation of the ordered triple (a, b, c) in 2D.
        /// &gt; 0 : counter-clockwise, &lt; 0 : clockwise, == 0 : collinear.
        /// </summary>
        public static float Orient2D(float2 a, float2 b, float2 c)
            => (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);

        /// <summary>
        /// In-circle test in 2D. Assuming (a, b, c) are counter-clockwise:
        /// &gt; 0 when d lies strictly inside the circumcircle of (a, b, c),
        /// &lt; 0 when outside, == 0 when cocircular.
        /// </summary>
        public static float InCircle2D(float2 a, float2 b, float2 c, float2 d)
        {
            float2 ad = a - d, bd = b - d, cd = c - d;
            float adSq = math.lengthsq(ad);
            float bdSq = math.lengthsq(bd);
            float cdSq = math.lengthsq(cd);

            return ad.x * (bd.y * cdSq - bdSq * cd.y)
                 - ad.y * (bd.x * cdSq - bdSq * cd.x)
                 + adSq * (bd.x * cd.y - bd.y * cd.x);
        }

        /// <summary>Unnormalized face normal of triangle (a, b, c) in 3D (right-hand rule).</summary>
        public static float3 TriangleNormal(float3 a, float3 b, float3 c)
            => math.cross(b - a, c - a);

        /// <summary>Area of triangle (a, b, c) in 3D.</summary>
        public static float TriangleArea(float3 a, float3 b, float3 c)
            => 0.5f * math.length(math.cross(b - a, c - a));
    }
}
