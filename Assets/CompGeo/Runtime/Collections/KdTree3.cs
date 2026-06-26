using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace CompGeo.Collections
{
    /// <summary>
    /// A static, balanced 3D k-d tree over a fixed point set, in flat <see cref="NativeArray{T}"/>
    /// storage. Built once (median split, O(n log n)); supports nearest-neighbour to a point and
    /// nearest-to-a-ray queries with branch-and-bound pruning over per-node subtree bounding boxes.
    ///
    /// This is the clean-room replacement for the old vendored KD-tree, and the spatial structure
    /// behind picking — replacing per-vertex physics colliders + name parsing (docs/MIGRATION.md §3,§D).
    /// Queries are read-only and allocation-free; the tree is immutable after <see cref="Build"/>.
    /// </summary>
    public struct KdTree3 : IDisposable
    {
        const int None = -1;

        NativeArray<float3> _pos;    // node -> point position
        NativeArray<int> _index;     // node -> original point index
        NativeArray<int> _axis;      // node -> split axis (0/1/2)
        NativeArray<int> _left;      // node -> left child node, or None
        NativeArray<int> _right;     // node -> right child node, or None
        NativeArray<float3> _bbMin;  // node -> subtree AABB min
        NativeArray<float3> _bbMax;  // node -> subtree AABB max
        int _root;
        int _count;

        public int Count => _count;

        /// <summary>Build a balanced k-d tree over <paramref name="points"/>.</summary>
        public static KdTree3 Build(NativeArray<float3> points, Allocator allocator)
        {
            int n = points.Length;
            var t = new KdTree3
            {
                _pos = new NativeArray<float3>(n, allocator, NativeArrayOptions.UninitializedMemory),
                _index = new NativeArray<int>(n, allocator, NativeArrayOptions.UninitializedMemory),
                _axis = new NativeArray<int>(n, allocator, NativeArrayOptions.UninitializedMemory),
                _left = new NativeArray<int>(n, allocator, NativeArrayOptions.UninitializedMemory),
                _right = new NativeArray<int>(n, allocator, NativeArrayOptions.UninitializedMemory),
                _bbMin = new NativeArray<float3>(n, allocator, NativeArrayOptions.UninitializedMemory),
                _bbMax = new NativeArray<float3>(n, allocator, NativeArrayOptions.UninitializedMemory),
                _count = 0,
                _root = None,
            };

            if (n > 0)
            {
                var order = new int[n];
                for (int i = 0; i < n; i++) order[i] = i;
                t._root = t.BuildNode(points, order, 0, n, 0);
            }
            return t;
        }

        int BuildNode(NativeArray<float3> points, int[] order, int lo, int hi, int depth)
        {
            if (lo >= hi) return None;

            int axis = depth % 3;
            int mid = (lo + hi) >> 1;
            QuickSelect(points, order, lo, hi - 1, mid, axis);

            int node = _count++;
            int p = order[mid];
            float3 pos = points[p];

            _index[node] = p;
            _pos[node] = pos;
            _axis[node] = axis;
            _left[node] = BuildNode(points, order, lo, mid, depth + 1);
            _right[node] = BuildNode(points, order, mid + 1, hi, depth + 1);

            float3 mn = pos, mx = pos;
            int l = _left[node];
            int r = _right[node];
            if (l != None) { mn = math.min(mn, _bbMin[l]); mx = math.max(mx, _bbMax[l]); }
            if (r != None) { mn = math.min(mn, _bbMin[r]); mx = math.max(mx, _bbMax[r]); }
            _bbMin[node] = mn;
            _bbMax[node] = mx;
            return node;
        }

        /// <summary>Index (into the original points) of the nearest point to <paramref name="query"/>, or -1 if empty.</summary>
        public int Nearest(float3 query)
        {
            if (_root == None) return None;
            int best = None;
            float bestSq = float.PositiveInfinity;
            NearestPoint(_root, query, ref best, ref bestSq);
            return best == None ? None : _index[best];
        }

        void NearestPoint(int node, float3 q, ref int best, ref float bestSq)
        {
            while (node != None)
            {
                float d2 = math.distancesq(q, _pos[node]);
                if (d2 < bestSq) { bestSq = d2; best = node; }

                int axis = _axis[node];
                float diff = q[axis] - _pos[node][axis];
                int near = diff < 0f ? _left[node] : _right[node];
                int far = diff < 0f ? _right[node] : _left[node];

                NearestPoint(near, q, ref best, ref bestSq);
                if (diff * diff < bestSq) node = far; // only the far side can still beat best
                else return;
            }
        }

        /// <summary>
        /// Fill <paramref name="outIndices"/> with the original indices of the up-to-k nearest points to
        /// <paramref name="query"/>, sorted ascending by distance (k = <c>outIndices.Length</c>); returns
        /// how many were written. <paramref name="outDistSq"/> (same length) receives their squared
        /// distances and doubles as the working buffer. Allocation-free. This is the k-NN query the
        /// point-cloud remesh uses to gather each vertex's neighbourhood.
        /// </summary>
        public int KNearest(float3 query, NativeArray<int> outIndices, NativeArray<float> outDistSq)
        {
            int k = outIndices.Length;
            int count = 0;
            if (_root != None && k > 0)
                KNearestSearch(_root, query, outIndices, outDistSq, ref count, k);
            return count;
        }

        void KNearestSearch(int node, float3 q, NativeArray<int> oi, NativeArray<float> od, ref int count, int k)
        {
            while (node != None)
            {
                ConsiderNeighbour(node, q, oi, od, ref count, k);

                int axis = _axis[node];
                float diff = q[axis] - _pos[node][axis];
                int near = diff < 0f ? _left[node] : _right[node];
                int far = diff < 0f ? _right[node] : _left[node];

                KNearestSearch(near, q, oi, od, ref count, k);

                float worst = count < k ? float.PositiveInfinity : od[k - 1];
                if (diff * diff < worst) node = far; // the far side may still hold a closer-than-worst point
                else return;
            }
        }

        void ConsiderNeighbour(int node, float3 q, NativeArray<int> oi, NativeArray<float> od, ref int count, int k)
        {
            float d2 = math.distancesq(q, _pos[node]);
            if (count == k && d2 >= od[k - 1]) return;

            int hi = count < k ? count - 1 : k - 2; // last slot to shift
            int i = hi;
            while (i >= 0 && od[i] > d2) { od[i + 1] = od[i]; oi[i + 1] = oi[i]; i--; }
            od[i + 1] = d2;
            oi[i + 1] = _index[node];
            if (count < k) count++;
        }

        /// <summary>
        /// Index of the point closest to the ray <c>origin + t·direction, t ≥ 0</c> (by perpendicular
        /// distance), within <paramref name="maxDistance"/>, or -1 if none. <paramref name="direction"/>
        /// must be normalized. Used for click-picking: build a screen ray, get the vertex under it.
        /// </summary>
        public int NearestToRay(float3 origin, float3 direction, float maxDistance)
        {
            if (_root == None) return None;
            int best = None;
            float bestSq = maxDistance * maxDistance;
            NearestRay(_root, origin, direction, ref best, ref bestSq);
            return best == None ? None : _index[best];
        }

        void NearestRay(int node, float3 o, float3 d, ref int best, ref float bestSq)
        {
            if (node == None) return;
            if (LowerBoundSq(node, o, d) >= bestSq) return; // whole subtree can't beat best

            float d2 = PointRaySq(_pos[node], o, d);
            if (d2 < bestSq) { bestSq = d2; best = node; }

            int l = _left[node];
            int r = _right[node];
            float lbL = l == None ? float.PositiveInfinity : LowerBoundSq(l, o, d);
            float lbR = r == None ? float.PositiveInfinity : LowerBoundSq(r, o, d);

            // Descend into the more promising child first so the bound tightens sooner.
            if (lbL <= lbR)
            {
                if (lbL < bestSq) NearestRay(l, o, d, ref best, ref bestSq);
                if (lbR < bestSq) NearestRay(r, o, d, ref best, ref bestSq);
            }
            else
            {
                if (lbR < bestSq) NearestRay(r, o, d, ref best, ref bestSq);
                if (lbL < bestSq) NearestRay(l, o, d, ref best, ref bestSq);
            }
        }

        /// <summary>Squared perpendicular distance from point <paramref name="p"/> to the ray (t ≥ 0).</summary>
        static float PointRaySq(float3 p, float3 o, float3 d)
        {
            float3 w = p - o;
            float s = math.dot(w, d);
            if (s <= 0f) return math.dot(w, w);   // closest approach is behind the origin
            return math.dot(w, w) - s * s;
        }

        /// <summary>
        /// A valid lower bound on the squared point-to-ray distance achievable by any point in node's
        /// AABB. With L = min |p-o|² over the box and sMax = max dot(p-o, d) over the box: points with
        /// s ≤ 0 give f = |p-o|² ≥ L; points with s &gt; 0 give f = |p-o|² - s² ≥ L - sMax². Hence
        /// max(0, L - sMax²) (or L when sMax ≤ 0) is a true lower bound — safe to prune against.
        /// </summary>
        float LowerBoundSq(int node, float3 o, float3 d)
        {
            float3 mn = _bbMin[node];
            float3 mx = _bbMax[node];

            // L: squared distance from o to the box.
            float3 cl = math.clamp(o, mn, mx);
            float L = math.distancesq(o, cl);

            // sMax: max of dot(p - o, d) over the box (pick each axis corner by the sign of d).
            float3 corner = math.select(mn, mx, d >= 0f);
            float sMax = math.dot(corner - o, d);

            if (sMax <= 0f) return L;
            return math.max(0f, L - sMax * sMax);
        }

        // Hoare-style quickselect: arranges `order[lo..hi]` so that the element at `k` is the one that
        // would be there if the slice were sorted by the given axis (median split for the build).
        static void QuickSelect(NativeArray<float3> points, int[] order, int lo, int hi, int k, int axis)
        {
            while (lo < hi)
            {
                float pivot = points[order[(lo + hi) >> 1]][axis];
                int i = lo, j = hi;
                while (i <= j)
                {
                    while (points[order[i]][axis] < pivot) i++;
                    while (points[order[j]][axis] > pivot) j--;
                    if (i <= j)
                    {
                        (order[i], order[j]) = (order[j], order[i]);
                        i++;
                        j--;
                    }
                }
                if (k <= j) hi = j;
                else if (k >= i) lo = i;
                else return;
            }
        }

        /// <summary>
        /// Batched, Burst-parallel k-NN: for every query in <paramref name="queries"/> writes its k nearest
        /// original indices (k = <c>outIdx.Length / queries.Length</c>) into the matching slice of
        /// <paramref name="outIdx"/>, sorted ascending by distance, with squared distances in
        /// <paramref name="outDst"/>. Each query's result is identical to a single <see cref="KNearest"/>
        /// call — this just runs them across worker threads. Used to precompute every vertex's
        /// neighbourhood for the point-cloud remesh.
        /// </summary>
        public void KNearestAll(NativeArray<float3> queries, int k, NativeArray<int> outIdx, NativeArray<float> outDst)
        {
            if (_root == None || queries.Length == 0 || k <= 0) return;
            new KnnAllJob
            {
                pos = _pos, index = _index, axis = _axis, left = _left, right = _right,
                queries = queries, root = _root, k = k, outIdx = outIdx, outDst = outDst,
            }.Schedule(queries.Length, 64).Complete();
        }

        [BurstCompile]
        struct KnnAllJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<float3> pos;
            [ReadOnly] public NativeArray<int> index;
            [ReadOnly] public NativeArray<int> axis;
            [ReadOnly] public NativeArray<int> left;
            [ReadOnly] public NativeArray<int> right;
            [ReadOnly] public NativeArray<float3> queries;
            public int root;
            public int k;
            [NativeDisableParallelForRestriction] public NativeArray<int> outIdx;
            [NativeDisableParallelForRestriction] public NativeArray<float> outDst;

            public void Execute(int v)
            {
                int count = 0;
                Search(root, queries[v], v * k, ref count);
            }

            void Search(int node, float3 q, int baseOff, ref int count)
            {
                while (node != None)
                {
                    float d2 = math.distancesq(q, pos[node]);
                    if (!(count == k && d2 >= outDst[baseOff + k - 1]))
                    {
                        int i = count < k ? count - 1 : k - 2;
                        while (i >= 0 && outDst[baseOff + i] > d2)
                        {
                            outDst[baseOff + i + 1] = outDst[baseOff + i];
                            outIdx[baseOff + i + 1] = outIdx[baseOff + i];
                            i--;
                        }
                        outDst[baseOff + i + 1] = d2;
                        outIdx[baseOff + i + 1] = index[node];
                        if (count < k) count++;
                    }

                    int ax = axis[node];
                    float diff = q[ax] - pos[node][ax];
                    int near = diff < 0f ? left[node] : right[node];
                    int far = diff < 0f ? right[node] : left[node];

                    Search(near, q, baseOff, ref count);
                    float worst = count < k ? float.PositiveInfinity : outDst[baseOff + k - 1];
                    if (diff * diff < worst) node = far;
                    else return;
                }
            }
        }

        public void Dispose()
        {
            if (_pos.IsCreated) _pos.Dispose();
            if (_index.IsCreated) _index.Dispose();
            if (_axis.IsCreated) _axis.Dispose();
            if (_left.IsCreated) _left.Dispose();
            if (_right.IsCreated) _right.Dispose();
            if (_bbMin.IsCreated) _bbMin.Dispose();
            if (_bbMax.IsCreated) _bbMax.Dispose();
        }
    }
}
