using System;
using System.Collections.Generic;
using Unity.Collections;
using CompGeo.Core;

namespace CompGeo.MeshProcessing.Parameterization
{
    /// <summary>
    /// Extracts the boundary of a triangle mesh: the edges that belong to exactly one face. For a
    /// disk-topology mesh these form a single closed loop, returned as an ordered vertex cycle — the
    /// input that parameterization methods (<see cref="TutteEmbedding"/>) pin to a convex polygon.
    /// </summary>
    public static class MeshBoundary
    {
        /// <summary>
        /// Return the boundary vertices of <paramref name="mesh"/> in cyclic order. Assumes a single
        /// boundary loop (manifold disk topology); the walk starts at the lowest-indexed boundary vertex
        /// for determinism. Throws <see cref="ArgumentException"/> when the mesh is closed (no boundary).
        /// The caller owns and must dispose the returned list.
        /// </summary>
        public static NativeList<int> ExtractLoop(in MeshData mesh, Allocator allocator)
        {
            // Count how many triangles use each undirected edge; boundary edges are used exactly once.
            var edgeUse = new Dictionary<long, int>();
            for (int t = 0; t < mesh.TriangleCount; t++)
            {
                var tri = mesh.Triangles[t];
                Bump(edgeUse, tri.x, tri.y);
                Bump(edgeUse, tri.y, tri.z);
                Bump(edgeUse, tri.z, tri.x);
            }

            // Boundary-edge adjacency: each boundary vertex links to its two boundary-edge neighbours.
            var boundaryAdj = new Dictionary<int, List<int>>();
            foreach (var kv in edgeUse)
            {
                if (kv.Value != 1) continue;
                DecodeKey(kv.Key, out int a, out int b);
                Link(boundaryAdj, a, b);
                Link(boundaryAdj, b, a);
            }

            if (boundaryAdj.Count == 0)
                throw new ArgumentException(
                    "Mesh has no boundary (closed surface); Tutte/Floater embedding requires disk topology.",
                    nameof(mesh));

            // Deterministic start: lowest boundary vertex id; sort each neighbour pair for a stable walk.
            int start = int.MaxValue;
            foreach (var v in boundaryAdj.Keys) if (v < start) start = v;
            foreach (var list in boundaryAdj.Values) list.Sort();

            var loop = new NativeList<int>(boundaryAdj.Count, allocator);
            int prev = -1;
            int cur = start;
            for (int guard = 0; guard <= boundaryAdj.Count; guard++)
            {
                loop.Add(cur);
                var nbrs = boundaryAdj[cur];
                int next = (nbrs[0] != prev) ? nbrs[0] : nbrs[1];
                prev = cur;
                cur = next;
                if (cur == start) break;
            }
            return loop;
        }

        static void Bump(Dictionary<long, int> edgeUse, int a, int b)
        {
            if (a == b) return;
            long key = Key(a, b);
            edgeUse.TryGetValue(key, out int c);
            edgeUse[key] = c + 1;
        }

        static void Link(Dictionary<int, List<int>> adj, int v, int w)
        {
            if (!adj.TryGetValue(v, out var list)) { list = new List<int>(2); adj[v] = list; }
            if (!list.Contains(w)) list.Add(w);
        }

        static long Key(int a, int b)
        {
            int lo = a < b ? a : b;
            int hi = a < b ? b : a;
            return ((long)lo << 32) | (uint)hi;
        }

        static void DecodeKey(long key, out int a, out int b)
        {
            a = (int)(key >> 32);
            b = (int)(key & 0xffffffff);
        }
    }
}
