using System;
using System.Collections.Generic;
using Unity.Mathematics;

namespace CompGeo.MeshProcessing
{
    /// <summary>
    /// 2D Delaunay triangulation of a point set (Bowyer–Watson). Used to triangulate each remesh
    /// neighbourhood: unlike polygon ear-clipping, it treats the projected k-NN as a <i>point set</i> and
    /// produces the well-shaped, sliver-free triangulation that tiles cleanly — fixing the long, crossing
    /// triangles the polygon approach produced. Pure and thread-safe (only local allocations).
    /// </summary>
    public static class DelaunayTriangulator
    {
        /// <summary>
        /// Triangulate <paramref name="pts"/>; returns triangle indices (triplets) into the input array.
        /// Returns an empty array for fewer than three points.
        /// </summary>
        public static int[] Triangulate(IReadOnlyList<float2> pts)
        {
            int n = pts == null ? 0 : pts.Count;
            if (n < 3) return Array.Empty<int>();

            // Vertices 0..n-1 are the input; n, n+1, n+2 are a super-triangle enclosing them all.
            var p = new float2[n + 3];
            float2 lo = pts[0], hi = pts[0];
            for (int i = 0; i < n; i++) { p[i] = pts[i]; lo = math.min(lo, pts[i]); hi = math.max(hi, pts[i]); }
            float2 c = (lo + hi) * 0.5f;
            float d = math.max(hi.x - lo.x, hi.y - lo.y);
            if (d < 1e-9f) d = 1f;
            p[n] = c + new float2(-20f * d, -20f * d);
            p[n + 1] = c + new float2(0f, 20f * d);
            p[n + 2] = c + new float2(20f * d, -20f * d);

            var tris = new List<int3> { new int3(n, n + 1, n + 2) };
            var bad = new List<int>();
            var poly = new List<int2>(); // boundary edges of the cavity

            for (int i = 0; i < n; i++)
            {
                bad.Clear();
                for (int t = 0; t < tris.Count; t++)
                {
                    int3 tr = tris[t];
                    if (InCircumcircle(p[tr.x], p[tr.y], p[tr.z], p[i])) bad.Add(t);
                }

                // Boundary of the cavity = edges that belong to exactly one bad triangle.
                poly.Clear();
                for (int bi = 0; bi < bad.Count; bi++)
                {
                    int3 tr = tris[bad[bi]];
                    AddEdge(poly, tr.x, tr.y);
                    AddEdge(poly, tr.y, tr.z);
                    AddEdge(poly, tr.z, tr.x);
                }

                // Remove bad triangles (back to front so indices stay valid).
                bad.Sort();
                for (int bi = bad.Count - 1; bi >= 0; bi--) tris.RemoveAt(bad[bi]);

                // Re-triangulate the cavity by joining each boundary edge to the new point.
                for (int e = 0; e < poly.Count; e++)
                    if (poly[e].x >= 0) tris.Add(new int3(poly[e].x, poly[e].y, i));
            }

            var result = new List<int>(tris.Count * 3);
            for (int t = 0; t < tris.Count; t++)
            {
                int3 tr = tris[t];
                if (tr.x < n && tr.y < n && tr.z < n) // drop anything still touching the super-triangle
                {
                    result.Add(tr.x); result.Add(tr.y); result.Add(tr.z);
                }
            }
            return result.ToArray();
        }

        // Track each undirected edge once; a second occurrence cancels it (interior edge, not boundary).
        static void AddEdge(List<int2> poly, int a, int b)
        {
            for (int i = 0; i < poly.Count; i++)
            {
                int2 e = poly[i];
                if (e.x < 0) continue;
                if ((e.x == a && e.y == b) || (e.x == b && e.y == a)) { poly[i] = new int2(-1, -1); return; }
            }
            poly.Add(new int2(a, b));
        }

        // True when p lies inside the circumcircle of triangle (a,b,c), orientation-independent.
        static bool InCircumcircle(float2 a, float2 b, float2 c, float2 p)
        {
            double ax = a.x - p.x, ay = a.y - p.y;
            double bx = b.x - p.x, by = b.y - p.y;
            double cx = c.x - p.x, cy = c.y - p.y;
            double det = (ax * ax + ay * ay) * (bx * cy - cx * by)
                       - (bx * bx + by * by) * (ax * cy - cx * ay)
                       + (cx * cx + cy * cy) * (ax * by - bx * ay);
            double orient = (double)(b.x - a.x) * (c.y - a.y) - (double)(c.x - a.x) * (b.y - a.y);
            if (orient > 0) return det > 0;
            if (orient < 0) return det < 0;
            return false; // degenerate (collinear) triangle: no containment
        }
    }
}
