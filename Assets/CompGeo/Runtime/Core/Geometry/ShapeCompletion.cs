using System.Collections.Generic;
using Unity.Mathematics;

namespace CompGeo.Core
{
    /// <summary>
    /// 2D shape completion (ported from the original CENG789 <c>ShapeCompletion</c> practice): given an
    /// open chain of boundary points, fill the gap back to the start with an arc of evenly turned points.
    /// The chain's accumulated turning angle (about its centroid) tells us how much of a full turn is
    /// still missing; the new points sweep that remaining angle while their radius eases from the last
    /// point's to the first point's, so the closed outline stays smooth. Pure and allocation-light — the
    /// MonoBehaviour demo just draws what this returns.
    /// </summary>
    public static class ShapeCompletion
    {
        /// <summary>
        /// Compute <paramref name="extraPoints"/> arc points that close the open chain
        /// <paramref name="open"/> from its last point back toward its first. The original points are not
        /// included in the result; the caller appends these and then closes back to <c>open[0]</c>.
        /// Returns an empty array when the input is too short or no points are requested.
        /// </summary>
        public static float2[] CompleteArc(IReadOnlyList<float2> open, int extraPoints)
        {
            if (open == null || open.Count < 2 || extraPoints <= 0)
                return System.Array.Empty<float2>();

            float2 center = float2.zero;
            for (int i = 0; i < open.Count; i++) center += open[i];
            center /= open.Count;

            // Accumulated turning angle of the existing chain about the centroid (signed).
            float turned = 0f;
            for (int i = 0; i < open.Count - 1; i++)
                turned += SignedAngle(open[i] - center, open[i + 1] - center);

            // Spread the remaining turn (a full revolution minus what is already there) over the new points.
            float increment = (2f * math.PI - math.abs(turned)) / (extraPoints + 1) * math.sign(turned);

            float2 first = open[0];
            float2 last = open[open.Count - 1];
            float baseAngle = SignedAngle(new float2(1f, 0f), last - center);
            float rLast = math.distance(center, last);
            float rFirst = math.distance(center, first);

            var result = new float2[extraPoints];
            for (int i = 0; i < extraPoints; i++)
            {
                float t = (float)(i + 1) / (extraPoints + 1);
                float radius = math.lerp(rLast, rFirst, t);
                float angle = (i + 1) * increment + baseAngle;
                result[i] = center + new float2(radius * math.cos(angle), radius * math.sin(angle));
            }
            return result;
        }

        /// <summary>Signed angle (radians) rotating <paramref name="a"/> to <paramref name="b"/>, in (-π, π].</summary>
        static float SignedAngle(float2 a, float2 b)
        {
            float cross = a.x * b.y - a.y * b.x;
            float dot = a.x * b.x + a.y * b.y;
            return math.atan2(cross, dot);
        }
    }
}
