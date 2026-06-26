using NUnit.Framework;
using Unity.Mathematics;
using CompGeo.Core;

namespace CompGeo.Tests
{
    public class ShapeCompletionTests
    {
        // An open chain of points along the upper unit circle (angles 180°..0°), centred near the origin.
        static float2[] UpperArc(int n)
        {
            var pts = new float2[n];
            for (int i = 0; i < n; i++)
            {
                float a = math.PI - math.PI * i / (n - 1); // π .. 0
                pts[i] = new float2(math.cos(a), math.sin(a));
            }
            return pts;
        }

        [Test]
        public void ReturnsRequestedCount()
        {
            float2[] arc = ShapeCompletion.CompleteArc(UpperArc(6), 10);
            Assert.AreEqual(10, arc.Length);
        }

        [Test]
        public void DegenerateInputReturnsEmpty()
        {
            Assert.AreEqual(0, ShapeCompletion.CompleteArc(new[] { new float2(0, 0) }, 5).Length);
            Assert.AreEqual(0, ShapeCompletion.CompleteArc(UpperArc(6), 0).Length);
            Assert.AreEqual(0, ShapeCompletion.CompleteArc(null, 5).Length);
        }

        [Test]
        public void CompletionPointsLieBelowTheOpenArc()
        {
            // The open chain spans the upper half-circle; the closing arc should sweep the lower half,
            // so every synthesised point sits at negative Y (within numerical tolerance).
            float2[] arc = ShapeCompletion.CompleteArc(UpperArc(9), 12);
            foreach (float2 p in arc)
                Assert.LessOrEqual(p.y, 1e-3f, $"completion point {p} should be in the lower half");
        }

        [Test]
        public void CompletionRadiusMatchesEndpointsAboutTheCentroid()
        {
            // The arc eases its radius from the last point's to the first point's, measured about the
            // chain's centroid. Here both endpoints are equidistant from the centroid, so every
            // synthesised point should sit at that same radius about it.
            float2[] chain = UpperArc(9);
            float2 centroid = float2.zero;
            foreach (float2 p in chain) centroid += p;
            centroid /= chain.Length;

            float expected = math.distance(centroid, chain[0]);
            float2[] arc = ShapeCompletion.CompleteArc(chain, 16);
            foreach (float2 p in arc)
                Assert.That(math.distance(centroid, p), Is.EqualTo(expected).Within(1e-3f));
        }
    }
}
