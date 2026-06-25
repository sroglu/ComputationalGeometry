using System;
using Unity.Collections;

namespace CompGeo.Collections
{
    /// <summary>
    /// A Burst-compatible binary min-heap addressed by integer node id, supporting <b>decrease-key</b>.
    ///
    /// Designed for single-source shortest-path (Dijkstra / A*) over a graph with a fixed node range
    /// <c>[0, capacity)</c>: each node appears at most once, and lowering its key re-sifts it in place
    /// rather than inserting a duplicate. This is the deliberately data-oriented alternative to the
    /// classic "lazy re-insertion + closed-set" heap — no boxing, no per-op allocation (see
    /// docs/MIGRATION.md §3 and ProjectFoundation FOUNDATION-MIGRATION-REPORT.md §DataStructures).
    ///
    /// Backing storage:
    ///  - <c>_heap[i]</c>  : node id stored at heap position i.
    ///  - <c>_pos[node]</c>: heap position of <c>node</c>, or <see cref="NotInHeap"/>.
    ///  - <c>_key[node]</c>: current priority key of <c>node</c>.
    /// </summary>
    public struct NativeIndexedMinHeap : IDisposable
    {
        const int NotInHeap = -1;

        NativeArray<int> _heap;
        NativeArray<int> _pos;
        NativeArray<float> _key;
        int _count;

        /// <summary>Create a heap able to hold node ids in <c>[0, capacity)</c>.</summary>
        public NativeIndexedMinHeap(int capacity, Allocator allocator)
        {
            _heap = new NativeArray<int>(capacity, allocator, NativeArrayOptions.UninitializedMemory);
            _pos = new NativeArray<int>(capacity, allocator, NativeArrayOptions.UninitializedMemory);
            _key = new NativeArray<float>(capacity, allocator, NativeArrayOptions.UninitializedMemory);
            for (int i = 0; i < capacity; i++) _pos[i] = NotInHeap;
            _count = 0;
        }

        public int Count => _count;
        public bool IsEmpty => _count == 0;

        /// <summary>True while <paramref name="node"/> is currently in the heap.</summary>
        public bool Contains(int node) => _pos[node] != NotInHeap;

        /// <summary>Current key of <paramref name="node"/> (valid while it is in the heap).</summary>
        public float KeyOf(int node) => _key[node];

        /// <summary>
        /// Insert <paramref name="node"/> with <paramref name="key"/>, or — if it is already present —
        /// lower its key to <paramref name="key"/>. A key that is not smaller than the current one is
        /// ignored (a min-heap never raises a key).
        /// </summary>
        public void PushOrDecrease(int node, float key)
        {
            int p = _pos[node];
            if (p == NotInHeap)
            {
                _key[node] = key;
                _heap[_count] = node;
                _pos[node] = _count;
                _count++;
                SiftUp(_count - 1);
            }
            else if (key < _key[node])
            {
                _key[node] = key;
                SiftUp(p);
            }
        }

        /// <summary>Remove and return the node with the smallest key. Call only when not empty.</summary>
        public int Pop()
        {
            int min = _heap[0];
            _count--;
            if (_count > 0)
            {
                int last = _heap[_count];
                _heap[0] = last;
                _pos[last] = 0;
                SiftDown(0);
            }
            _pos[min] = NotInHeap;
            return min;
        }

        void SiftUp(int i)
        {
            int node = _heap[i];
            float key = _key[node];
            while (i > 0)
            {
                int parent = (i - 1) >> 1;
                int parentNode = _heap[parent];
                if (_key[parentNode] <= key) break;
                _heap[i] = parentNode;
                _pos[parentNode] = i;
                i = parent;
            }
            _heap[i] = node;
            _pos[node] = i;
        }

        void SiftDown(int i)
        {
            int node = _heap[i];
            float key = _key[node];
            int half = _count >> 1; // nodes with at least one child
            while (i < half)
            {
                int child = (i << 1) + 1;
                int right = child + 1;
                if (right < _count && _key[_heap[right]] < _key[_heap[child]]) child = right;

                int childNode = _heap[child];
                if (_key[childNode] >= key) break;
                _heap[i] = childNode;
                _pos[childNode] = i;
                i = child;
            }
            _heap[i] = node;
            _pos[node] = i;
        }

        public void Dispose()
        {
            if (_heap.IsCreated) _heap.Dispose();
            if (_pos.IsCreated) _pos.Dispose();
            if (_key.IsCreated) _key.Dispose();
        }
    }
}
