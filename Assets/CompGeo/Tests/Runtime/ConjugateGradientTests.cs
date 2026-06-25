using NUnit.Framework;
using Unity.Collections;
using CompGeo.Numerics;

namespace CompGeo.Tests
{
    /// <summary>
    /// Unit tests for the sparse CSR matrix and the Conjugate-Gradient SPD solver
    /// (docs/MIGRATION.md §6.C — the sparse replacement for the old dense inverse).
    /// </summary>
    public class ConjugateGradientTests
    {
        [Test]
        public void BuilderSumsDuplicatesAndSortsColumns()
        {
            // Row 0: two entries at column 0 (2 + 2) must merge to 4, plus a column-1 entry, kept sorted.
            var builder = new SparseMatrixBuilder(2, 2);
            builder.Add(0, 1, 5f);
            builder.Add(0, 0, 2f);
            builder.Add(0, 0, 2f);
            builder.Add(1, 1, 3f);

            var a = builder.Build(Allocator.Temp);
            try
            {
                Assert.AreEqual(3, a.NonZeros, "duplicate (0,0) entries should merge");
                // Row 0 spans [0,2): columns ascending 0 then 1.
                Assert.AreEqual(0, a.RowOffsets[0]);
                Assert.AreEqual(2, a.RowOffsets[1]);
                Assert.AreEqual(0, a.ColumnIndices[0]);
                Assert.AreEqual(4f, a.Values[0], 1e-6f); // 2 + 2 merged
                Assert.AreEqual(1, a.ColumnIndices[1]);
                Assert.AreEqual(5f, a.Values[1], 1e-6f);
            }
            finally { a.Dispose(); }
        }

        [Test]
        public void SpMvMatchesDenseProduct()
        {
            // A = [[4,1],[1,3]] ; x = [1,1] -> A·x = [5,4]
            var builder = new SparseMatrixBuilder(2, 2);
            builder.Add(0, 0, 4f); builder.Add(0, 1, 1f);
            builder.Add(1, 0, 1f); builder.Add(1, 1, 3f);
            var a = builder.Build(Allocator.Temp);

            var x = new NativeArray<float>(new[] { 1f, 1f }, Allocator.Temp);
            var y = new NativeArray<float>(2, Allocator.Temp);
            try
            {
                a.Multiply(x, y);
                Assert.AreEqual(5f, y[0], 1e-6f);
                Assert.AreEqual(4f, y[1], 1e-6f);
            }
            finally { a.Dispose(); x.Dispose(); y.Dispose(); }
        }

        [Test]
        public void SolvesSpdSystem()
        {
            // 4x + y = 1 ; x + 3y = 2  ->  x = 1/11, y = 7/11
            var builder = new SparseMatrixBuilder(2, 2);
            builder.Add(0, 0, 4f); builder.Add(0, 1, 1f);
            builder.Add(1, 0, 1f); builder.Add(1, 1, 3f);
            var a = builder.Build(Allocator.Temp);

            var b = new NativeArray<float>(new[] { 1f, 2f }, Allocator.Temp);
            var x = new NativeArray<float>(2, Allocator.Temp); // zero initial guess
            try
            {
                int iters = ConjugateGradient.Solve(
                    a.RowOffsets, a.ColumnIndices, a.Values, b, x, maxIterations: 100, tolerance: 1e-8f);

                Assert.LessOrEqual(iters, 2, "CG must converge in <= n steps on a 2x2 SPD system");
                Assert.AreEqual(1f / 11f, x[0], 1e-5f);
                Assert.AreEqual(7f / 11f, x[1], 1e-5f);
            }
            finally { a.Dispose(); b.Dispose(); x.Dispose(); }
        }

        [Test]
        public void SolvesLargerDiagonallyDominantSystem()
        {
            // Tridiagonal SPD: diagonal 2, off-diagonals -1 (1D Laplacian) with a known RHS.
            const int n = 64;
            var builder = new SparseMatrixBuilder(n, n);
            for (int i = 0; i < n; i++)
            {
                builder.Add(i, i, 2f);
                if (i > 0) builder.Add(i, i - 1, -1f);
                if (i < n - 1) builder.Add(i, i + 1, -1f);
            }
            var a = builder.Build(Allocator.Temp);

            // Pick a known solution xTrue, set b = A·xTrue, solve, compare.
            var xTrue = new NativeArray<float>(n, Allocator.Temp);
            for (int i = 0; i < n; i++) xTrue[i] = (i % 7) - 3;
            var b = new NativeArray<float>(n, Allocator.Temp);
            a.Multiply(xTrue, b);

            var x = new NativeArray<float>(n, Allocator.Temp);
            try
            {
                ConjugateGradient.Solve(a.RowOffsets, a.ColumnIndices, a.Values, b, x, maxIterations: 500, tolerance: 1e-8f);
                for (int i = 0; i < n; i++)
                    Assert.AreEqual(xTrue[i], x[i], 1e-3f, $"solution mismatch at {i}");
            }
            finally { a.Dispose(); xTrue.Dispose(); b.Dispose(); x.Dispose(); }
        }
    }
}
