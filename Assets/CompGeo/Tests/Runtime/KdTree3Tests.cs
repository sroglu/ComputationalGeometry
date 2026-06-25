using System;
using NUnit.Framework;
using Unity.Collections;
using Unity.Mathematics;
using CompGeo.Collections;

namespace CompGeo.Tests
{
    public class KdTree3Tests
    {
        const int N = 200;

        // Deterministic pseudo-random point set (no RNG dependency, reproducible).
        static NativeArray<float3> MakePoints(Allocator allocator)
        {
            var pts = new NativeArray<float3>(N, allocator);
            for (int i = 0; i < N; i++)
                pts[i] = new float3(Hash(i * 3 + 0), Hash(i * 3 + 1), Hash(i * 3 + 2)) * 10f;
            return pts;
        }

        static float Hash(int n)
        {
            double x = math.sin(n * 12.9898 + 4.1414) * 43758.5453;
            return (float)(x - math.floor(x));
        }

        [Test]
        public void NearestMatchesBruteForce()
        {
            using var pts = MakePoints(Allocator.Persistent);
            using var tree = KdTree3.Build(pts, Allocator.Persistent);

            for (int q = 0; q < 25; q++)
            {
                float3 query = new float3(Hash(1000 + q * 3), Hash(1001 + q * 3), Hash(1002 + q * 3)) * 12f - 1f;

                int expected = BruteNearest(pts, query);
                int actual = tree.Nearest(query);
                Assert.AreEqual(expected, actual, $"point NN mismatch for query {q}");
            }
        }

        [Test]
        public void NearestOfAnExistingPointIsItself()
        {
            using var pts = MakePoints(Allocator.Persistent);
            using var tree = KdTree3.Build(pts, Allocator.Persistent);
            for (int i = 0; i < N; i += 17)
                Assert.AreEqual(i, tree.Nearest(pts[i]));
        }

        [Test]
        public void NearestToRayMatchesBruteForce()
        {
            using var pts = MakePoints(Allocator.Persistent);
            using var tree = KdTree3.Build(pts, Allocator.Persistent);

            for (int q = 0; q < 25; q++)
            {
                float3 origin = new float3(Hash(2000 + q * 3), Hash(2001 + q * 3), Hash(2002 + q * 3)) * 14f - 2f;
                float3 dir = math.normalize(new float3(
                    Hash(3000 + q * 3) - 0.5f,
                    Hash(3001 + q * 3) - 0.5f,
                    Hash(3002 + q * 3) - 0.5f));

                int expected = BruteNearestToRay(pts, origin, dir);
                int actual = tree.NearestToRay(origin, dir, 1000f);
                Assert.AreEqual(expected, actual, $"ray NN mismatch for ray {q}");
            }
        }

        [Test]
        public void NearestToRayRespectsMaxDistance()
        {
            using var pts = MakePoints(Allocator.Persistent);
            using var tree = KdTree3.Build(pts, Allocator.Persistent);
            // A ray far from every point with a tiny radius must find nothing.
            int hit = tree.NearestToRay(new float3(1000f, 1000f, 1000f), new float3(0, 0, 1), 0.5f);
            Assert.AreEqual(-1, hit);
        }

        static int BruteNearest(NativeArray<float3> pts, float3 q)
        {
            int best = -1;
            float bestSq = float.PositiveInfinity;
            for (int i = 0; i < pts.Length; i++)
            {
                float d2 = math.distancesq(q, pts[i]);
                if (d2 < bestSq) { bestSq = d2; best = i; }
            }
            return best;
        }

        static int BruteNearestToRay(NativeArray<float3> pts, float3 o, float3 d)
        {
            int best = -1;
            float bestSq = float.PositiveInfinity;
            for (int i = 0; i < pts.Length; i++)
            {
                float3 w = pts[i] - o;
                float s = math.dot(w, d);
                float d2 = s <= 0f ? math.dot(w, w) : math.dot(w, w) - s * s;
                if (d2 < bestSq) { bestSq = d2; best = i; }
            }
            return best;
        }
    }
}
