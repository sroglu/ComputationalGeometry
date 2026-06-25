using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;

namespace CompGeo.Numerics
{
    /// <summary>
    /// Conjugate-Gradient solver for a sparse <b>symmetric positive-definite</b> system <c>A·x = b</c>
    /// over CSR (Hestenes &amp; Stiefel 1952). This is the HW2 linear-algebra fix (docs/MIGRATION.md §6.C):
    /// the original dense <c>Matrix.Inverse(L)</c> — O(N³) time, O(N²) memory — is replaced by an
    /// iterative solve whose per-iteration cost is one CSR SpMV plus a few vector ops, i.e. O(nnz).
    ///
    /// <para>Matrix-free interface: the solver only ever multiplies by <c>A</c>, so it needs no
    /// factorization and touches no managed state. The core kernel is Burst-friendly (raw arrays, scratch
    /// in <see cref="Allocator.Temp"/>, dot-products accumulated in <c>double</c> for stability), so it
    /// runs identically when called directly (tests) or from inside <see cref="ConjugateGradientJob"/>.</para>
    /// </summary>
    public static class ConjugateGradient
    {
        /// <summary>Default relative residual tolerance (‖b − A·x‖ / ‖b‖) for convergence.</summary>
        public const float DefaultTolerance = 1e-6f;

        /// <summary>
        /// Solve <c>A·x = b</c> in place. <paramref name="x"/> is both the initial guess (use a zeroed
        /// array for none) and the output. Iterates until the relative residual falls below
        /// <paramref name="tolerance"/> or <paramref name="maxIterations"/> is reached; returns the number
        /// of iterations performed. <c>A</c> must be SPD — the kernel bails out if a non-positive
        /// curvature <c>pᵀA·p ≤ 0</c> is encountered (indefinite matrix / numerical breakdown).
        /// </summary>
        public static int Solve(
            NativeArray<int> rowOffsets,
            NativeArray<int> columnIndices,
            NativeArray<float> values,
            NativeArray<float> b,
            NativeArray<float> x,
            int maxIterations,
            float tolerance)
        {
            int n = b.Length;
            var r = new NativeArray<float>(n, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var p = new NativeArray<float>(n, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var ap = new NativeArray<float>(n, Allocator.Temp, NativeArrayOptions.UninitializedMemory);

            // r = b - A x ; p = r ; rsold = rᵀr
            SparseMatrixCsr.Multiply(rowOffsets, columnIndices, values, n, x, ap);
            double rsold = 0.0;
            double bnorm2 = 0.0;
            for (int i = 0; i < n; i++)
            {
                float ri = b[i] - ap[i];
                r[i] = ri;
                p[i] = ri;
                rsold += (double)ri * ri;
                bnorm2 += (double)b[i] * b[i];
            }

            // Relative convergence threshold on the squared residual norm.
            double threshold = (double)tolerance * tolerance * (bnorm2 > 1e-30 ? bnorm2 : 1.0);

            int it = 0;
            if (rsold > threshold)
            {
                for (; it < maxIterations; it++)
                {
                    SparseMatrixCsr.Multiply(rowOffsets, columnIndices, values, n, p, ap);

                    double pAp = 0.0;
                    for (int i = 0; i < n; i++) pAp += (double)p[i] * ap[i];
                    if (pAp <= 0.0) break; // non-SPD / breakdown — stop rather than diverge

                    double alpha = rsold / pAp;
                    double rsnew = 0.0;
                    for (int i = 0; i < n; i++)
                    {
                        x[i] += (float)(alpha * p[i]);
                        float ri = r[i] - (float)(alpha * ap[i]);
                        r[i] = ri;
                        rsnew += (double)ri * ri;
                    }

                    if (rsnew <= threshold) { it++; break; }

                    double beta = rsnew / rsold;
                    for (int i = 0; i < n; i++) p[i] = r[i] + (float)(beta * p[i]);
                    rsold = rsnew;
                }
            }

            r.Dispose();
            p.Dispose();
            ap.Dispose();
            return it;
        }

        /// <summary>
        /// Allocate a zeroed solution vector and solve <c>A·x = b</c> on the calling thread via the
        /// Burst-compiled <see cref="ConjugateGradientJob"/>. The caller owns and must dispose the result.
        /// <para><paramref name="allocator"/> must be job-compatible (<see cref="Allocator.TempJob"/> or
        /// <see cref="Allocator.Persistent"/>) — and so must the storage of <paramref name="a"/> and
        /// <paramref name="b"/>, since they are bound to the job. <see cref="Allocator.Temp"/> is rejected
        /// by the job scheduler. To solve with Temp scratch instead, call <see cref="Solve"/> directly.</para>
        /// </summary>
        public static NativeArray<float> Run(
            in SparseMatrixCsr a,
            NativeArray<float> b,
            Allocator allocator,
            int maxIterations,
            float tolerance = DefaultTolerance)
        {
            var x = new NativeArray<float>(b.Length, allocator); // zeroed -> zero initial guess
            new ConjugateGradientJob
            {
                RowOffsets = a.RowOffsets,
                ColumnIndices = a.ColumnIndices,
                Values = a.Values,
                B = b,
                X = x,
                MaxIterations = maxIterations,
                Tolerance = tolerance,
            }.Run();
            return x;
        }
    }

    /// <summary>Burst-compiled wrapper around <see cref="ConjugateGradient.Solve"/>.</summary>
    [BurstCompile]
    public struct ConjugateGradientJob : IJob
    {
        [ReadOnly] public NativeArray<int> RowOffsets;
        [ReadOnly] public NativeArray<int> ColumnIndices;
        [ReadOnly] public NativeArray<float> Values;
        [ReadOnly] public NativeArray<float> B;

        public NativeArray<float> X;
        public int MaxIterations;
        public float Tolerance;

        public void Execute()
            => ConjugateGradient.Solve(RowOffsets, ColumnIndices, Values, B, X, MaxIterations, Tolerance);
    }
}
