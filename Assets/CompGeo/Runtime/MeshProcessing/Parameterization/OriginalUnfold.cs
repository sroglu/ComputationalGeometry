using System;
using Unity.Collections;
using Unity.Mathematics;
using CompGeo.Core;

namespace CompGeo.MeshProcessing.Parameterization
{
    /// <summary>
    /// The original CENG789 homework's unfold (<c>Model.UniformUnfold</c>) reproduced faithfully: build the
    /// <b>dense</b> uniform Laplacian, pin the boundary loop to the unit circle, and solve the system
    /// directly (the homework used <c>Accord.Math.Matrix.Inverse</c>; here a dense Gaussian elimination,
    /// same O(N³) cost). Mathematically identical to <see cref="TutteEmbedding"/> — both give the round
    /// result — but kept as the authentic, slow original next to the performant sparse-CG version.
    /// Capped at <see cref="MaxVertices"/> because the dense solve is cubic.
    /// </summary>
    public static class OriginalUnfold
    {
        public const int MaxVertices = 2800;

        public static NativeArray<float2> Compute(in MeshData mesh, Allocator allocator)
        {
            int n = mesh.VertexCount;
            if (n > MaxVertices)
                throw new ArgumentException(
                    $"Original (dense O(N^3)) unfold is capped at {MaxVertices} vertices; this mesh has {n}. Use Tutte (performant).");

            var uv = new NativeArray<float2>(n, allocator);

            // Boundary loop pinned to the unit circle (throws if the mesh is closed — no boundary).
            var loop = MeshBoundary.ExtractLoop(mesh, Allocator.Temp);
            int b = loop.Length;
            var isB = new bool[n];
            for (int k = 0; k < b; k++)
            {
                int v = loop[k];
                isB[v] = true;
                float ang = 2f * math.PI * k / b;
                uv[v] = new float2(math.cos(ang), math.sin(ang));
            }
            loop.Dispose();

            // Dense system A·x = rhs: boundary rows fix x to the circle point; interior rows are the
            // uniform Laplacian (diag = -degree, +1 per neighbour) with zero right-hand side.
            var A = new double[n, n];
            var bx = new double[n];
            var by = new double[n];
            for (int i = 0; i < n; i++)
            {
                if (isB[i])
                {
                    A[i, i] = 1.0;
                    bx[i] = uv[i].x;
                    by[i] = uv[i].y;
                }
                else
                {
                    mesh.GetNeighbours(i, out int s, out int c);
                    A[i, i] = -c;
                    for (int k = 0; k < c; k++) A[i, mesh.AdjNeighbours[s + k]] += 1.0;
                }
            }

            Solve2(A, bx, by, n);

            for (int v = 0; v < n; v++)
                if (!isB[v]) uv[v] = new float2((float)bx[v], (float)by[v]);
            return uv;
        }

        // Gaussian elimination with partial pivoting, two right-hand sides sharing the matrix.
        static void Solve2(double[,] A, double[] bx, double[] by, int n)
        {
            for (int k = 0; k < n; k++)
            {
                int piv = k;
                double best = Math.Abs(A[k, k]);
                for (int r = k + 1; r < n; r++)
                {
                    double v = Math.Abs(A[r, k]);
                    if (v > best) { best = v; piv = r; }
                }
                if (piv != k)
                {
                    for (int c = k; c < n; c++) (A[k, c], A[piv, c]) = (A[piv, c], A[k, c]);
                    (bx[k], bx[piv]) = (bx[piv], bx[k]);
                    (by[k], by[piv]) = (by[piv], by[k]);
                }

                double d = A[k, k];
                if (Math.Abs(d) < 1e-12) continue;
                for (int r = k + 1; r < n; r++)
                {
                    double f = A[r, k] / d;
                    if (f == 0.0) continue;
                    for (int c = k; c < n; c++) A[r, c] -= f * A[k, c];
                    bx[r] -= f * bx[k];
                    by[r] -= f * by[k];
                }
            }

            for (int i = n - 1; i >= 0; i--)
            {
                double sx = bx[i], sy = by[i];
                for (int c = i + 1; c < n; c++) { sx -= A[i, c] * bx[c]; sy -= A[i, c] * by[c]; }
                double d = A[i, i];
                if (Math.Abs(d) < 1e-12) d = 1.0;
                bx[i] = sx / d;
                by[i] = sy / d;
            }
        }
    }
}
