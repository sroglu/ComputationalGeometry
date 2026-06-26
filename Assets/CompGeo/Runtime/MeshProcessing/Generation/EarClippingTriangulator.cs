using System;
using System.Collections.Generic;
using Unity.Mathematics;

namespace CompGeo.MeshProcessing
{
    /// <summary>
    /// Ear-clipping triangulation of a simple 2D polygon — a faithful port of the original CENG789
    /// homework's <c>Triangulator</c> (the public-domain ear-clipping library it vendored). The polygon
    /// is wound to counter-clockwise, then convex "ear" corners that contain no reflex vertex are clipped
    /// one at a time. The point-cloud remesh feeds it each neighbourhood's projected 2D points, so inputs
    /// are tiny (≈8 vertices) and the O(n²) ear search is irrelevant.
    /// <para/>
    /// One deliberate fidelity fix versus the original: results are returned as indices <b>into the input
    /// polygon array</b>. The original returned indices into a possibly winding-reversed copy while the
    /// caller mapped them through the input order, so clockwise patches mapped to the wrong vertices; here
    /// the reversal is mapped back, keeping the same algorithm but correcting that latent bug.
    /// </summary>
    public static class EarClippingTriangulator
    {
        public enum WindingOrder { Clockwise, CounterClockwise }

        /// <summary>
        /// Triangulate <paramref name="polygon"/>; returns triangle indices (triplets) into the input
        /// array, counter-clockwise. Returns an empty array for fewer than three vertices.
        /// </summary>
        public static int[] Triangulate(IReadOnlyList<float2> polygon)
        {
            int n = polygon?.Count ?? 0;
            if (n < 3) return Array.Empty<int>();

            // A counter-clockwise ordering of the original indices (mirrors the original ReverseWindingOrder:
            // keep [0], reverse the rest) so the clipped triangles always reference input positions.
            var ring = new List<int>(n);
            if (DetermineWindingOrder(polygon) == WindingOrder.Clockwise)
            {
                ring.Add(0);
                for (int i = 1; i < n; i++) ring.Add(n - i);
            }
            else
            {
                for (int i = 0; i < n; i++) ring.Add(i);
            }

            var tris = new List<int>(3 * (n - 2));
            int guard = n * n;
            while (ring.Count > 3 && guard-- > 0)
            {
                bool clipped = false;
                int count = ring.Count;
                for (int i = 0; i < count; i++)
                {
                    int prev = ring[(i - 1 + count) % count];
                    int cur = ring[i];
                    int next = ring[(i + 1) % count];

                    if (!IsConvex(polygon[prev], polygon[cur], polygon[next])) continue;
                    if (!IsEar(polygon, ring, i, prev, cur, next)) continue;

                    tris.Add(cur); tris.Add(next); tris.Add(prev); // ear, next, prev — CCW (matches original)
                    ring.RemoveAt(i);
                    clipped = true;
                    break;
                }
                if (!clipped) break; // no ear found (degenerate/non-simple input) — stop instead of looping
            }

            if (ring.Count == 3) { tris.Add(ring[0]); tris.Add(ring[1]); tris.Add(ring[2]); }
            return tris.ToArray();
        }

        /// <summary>Winding order via the original's sign-of-cross vote across consecutive corners.</summary>
        public static WindingOrder DetermineWindingOrder(IReadOnlyList<float2> v)
        {
            int n = v.Count;
            int cw = 0, ccw = 0;
            float2 p1 = v[0];
            for (int i = 1; i < n; i++)
            {
                float2 p2 = v[i];
                float2 p3 = v[(i + 1) % n];
                float2 e1 = p1 - p2;
                float2 e2 = p3 - p2;
                if (e1.x * e2.y - e1.y * e2.x >= 0f) cw++;
                else ccw++;
                p1 = p2;
            }
            return cw > ccw ? WindingOrder.Clockwise : WindingOrder.CounterClockwise;
        }

        // A corner is convex when turning from edge (prev->cur) to (cur->next) keeps the interior on the
        // left — the original's dot(d1, perp(d2)) <= 0 test for a CCW polygon.
        static bool IsConvex(float2 prev, float2 cur, float2 next)
        {
            float2 d1 = math.normalizesafe(cur - prev);
            float2 d2 = math.normalizesafe(next - cur);
            float2 perp = new float2(-d2.y, d2.x);
            return math.dot(d1, perp) <= 0f;
        }

        // An ear is a convex corner whose triangle (prev,cur,next) contains no reflex vertex of the ring.
        static bool IsEar(IReadOnlyList<float2> poly, List<int> ring, int earPos, int prev, int cur, int next)
        {
            int count = ring.Count;
            for (int j = 0; j < count; j++)
            {
                int v = ring[j];
                if (v == prev || v == cur || v == next) continue;

                int pj = ring[(j - 1 + count) % count];
                int nj = ring[(j + 1) % count];
                if (IsConvex(poly[pj], poly[v], poly[nj])) continue; // only reflex vertices can spoil an ear

                if (PointInTriangle(poly[v], poly[prev], poly[cur], poly[next])) return false;
            }
            return true;
        }

        static bool PointInTriangle(float2 p, float2 a, float2 b, float2 c)
        {
            float d1 = Cross(p, a, b);
            float d2 = Cross(p, b, c);
            float d3 = Cross(p, c, a);
            bool hasNeg = d1 < 0f || d2 < 0f || d3 < 0f;
            bool hasPos = d1 > 0f || d2 > 0f || d3 > 0f;
            return !(hasNeg && hasPos); // inside or on an edge
        }

        static float Cross(float2 p1, float2 p2, float2 p3)
            => (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
    }
}
