using NUnit.Framework;
using Unity.Collections;
using CompGeo.Collections;

namespace CompGeo.Tests
{
    public class NativeIndexedMinHeapTests
    {
        [Test]
        public void PopsInAscendingKeyOrder()
        {
            using var heap = new NativeIndexedMinHeap(8, Allocator.Persistent);
            heap.PushOrDecrease(3, 3.0f);
            heap.PushOrDecrease(1, 1.0f);
            heap.PushOrDecrease(5, 5.0f);
            heap.PushOrDecrease(2, 2.0f);
            heap.PushOrDecrease(4, 4.0f);

            Assert.AreEqual(1, heap.Pop());
            Assert.AreEqual(2, heap.Pop());
            Assert.AreEqual(3, heap.Pop());
            Assert.AreEqual(4, heap.Pop());
            Assert.AreEqual(5, heap.Pop());
            Assert.IsTrue(heap.IsEmpty);
        }

        [Test]
        public void DecreaseKeyReordersAndDoesNotDuplicate()
        {
            using var heap = new NativeIndexedMinHeap(8, Allocator.Persistent);
            heap.PushOrDecrease(0, 10.0f);
            heap.PushOrDecrease(1, 20.0f);
            heap.PushOrDecrease(2, 30.0f);
            Assert.AreEqual(3, heap.Count);

            heap.PushOrDecrease(2, 5.0f); // decrease-key: node 2 jumps to the front, no new entry
            Assert.AreEqual(3, heap.Count);
            Assert.AreEqual(5.0f, heap.KeyOf(2), 1e-6f);

            Assert.AreEqual(2, heap.Pop());
            Assert.AreEqual(0, heap.Pop());
            Assert.AreEqual(1, heap.Pop());
        }

        [Test]
        public void IgnoresNonDecreasingKey()
        {
            using var heap = new NativeIndexedMinHeap(4, Allocator.Persistent);
            heap.PushOrDecrease(0, 5.0f);
            heap.PushOrDecrease(0, 9.0f); // larger key must be ignored by a min-heap
            Assert.AreEqual(1, heap.Count);
            Assert.AreEqual(5.0f, heap.KeyOf(0), 1e-6f);
        }

        [Test]
        public void ContainsTracksMembership()
        {
            using var heap = new NativeIndexedMinHeap(4, Allocator.Persistent);
            Assert.IsFalse(heap.Contains(2));
            heap.PushOrDecrease(2, 1.0f);
            Assert.IsTrue(heap.Contains(2));
            heap.Pop();
            Assert.IsFalse(heap.Contains(2));
        }
    }
}
