using System.Globalization;
using System.IO;
using NUnit.Framework;
using Unity.Collections;
using UnityEngine;
using CompGeo.Core;
using CompGeo.Core.IO;
using CompGeo.MeshProcessing.Geodesics;

namespace CompGeo.Tests
{
    /// <summary>
    /// Regression against the CENG789 instructor-provided all-pairs geodesic distance matrix for
    /// <c>man0.off</c> (the "M for man0" ground truth; docs/MIGRATION.md §1). The data is
    /// all-rights-reserved course material kept local and git-ignored — see Fixtures/README.md.
    /// When the files are absent the test is skipped so a clean checkout stays green.
    /// </summary>
    public class Man0GroundTruthTests
    {
        const string MeshFile = "man0.off";
        const string MatrixFile = "M_for_man0.txt";

        // Graph Dijkstra is an upper bound on the true surface geodesic; the instructor matrix is the
        // validation target for the HW1 Dijkstra output, so allow a small relative tolerance.
        const float RelTolerance = 1e-2f;

        static string FixtureDir => Path.Combine(Application.dataPath, "CompGeo/Tests/Runtime/Fixtures");

        [Test]
        public void DijkstraMatchesGroundTruthMatrix()
        {
            string meshPath = Path.Combine(FixtureDir, MeshFile);
            string matrixPath = Path.Combine(FixtureDir, MatrixFile);

            if (!File.Exists(meshPath) || !File.Exists(matrixPath))
            {
                Assert.Ignore($"man0 fixture not present in {FixtureDir} — see Fixtures/README.md.");
                return;
            }

            using var mesh = OffReader.ReadFile(meshPath, Allocator.Persistent);
            int n = mesh.VertexCount;

            float[] matrix = ReadMatrix(matrixPath, n);

            // Validate a spread of source vertices (full 502×502 sweep is unnecessary for a regression).
            int[] sources = { 0, n / 4, n / 2, (3 * n) / 4, n - 1 };
            foreach (int s in sources)
            {
                var dist = new NativeArray<float>(n, Allocator.Persistent);
                var pred = new NativeArray<int>(n, Allocator.Persistent);
                try
                {
                    DijkstraGeodesics.Compute(mesh, s, dist, pred);
                    for (int t = 0; t < n; t++)
                    {
                        float expected = matrix[s * n + t];
                        float tol = RelTolerance * Mathf.Max(1f, expected);
                        Assert.AreEqual(expected, dist[t], tol, $"man0 distance mismatch ({s} -> {t})");
                    }
                }
                finally
                {
                    dist.Dispose();
                    pred.Dispose();
                }
            }
        }

        static float[] ReadMatrix(string path, int n)
        {
            var values = new float[n * n];
            int count = 0;
            using var reader = new StreamReader(path);
            string line;
            while ((line = reader.ReadLine()) != null)
            {
                if (string.IsNullOrWhiteSpace(line)) continue;
                foreach (string tok in line.Split((char[])null, System.StringSplitOptions.RemoveEmptyEntries))
                {
                    Assert.Less(count, values.Length, "matrix has more entries than n*n");
                    values[count++] = float.Parse(tok, CultureInfo.InvariantCulture);
                }
            }
            Assert.AreEqual(n * n, count, "matrix entry count does not match n*n");
            return values;
        }
    }
}
