using Unity.Collections;
using Unity.Mathematics;
using CompGeo.Core;
using CompGeo.Numerics;

namespace CompGeo.MeshProcessing.Parameterization
{
    /// <summary>
    /// Tutte / Floater barycentric mesh parameterization with the <b>uniform graph Laplacian</b>
    /// (Tutte 1963). The boundary loop is pinned to a convex polygon (here the unit circle); every
    /// interior vertex is then placed at the average of its neighbours, i.e. it satisfies the discrete
    /// Laplace equation <c>L·u = 0</c> on the interior with Dirichlet boundary data. Tutte's theorem
    /// guarantees the result is a valid (fold-free) planar embedding for a 3-connected planar graph.
    ///
    /// <para>This is the HW2 "mesh unfolding" of docs/MIGRATION.md §6, re-cast correctly: the system is
    /// assembled <b>sparse</b> in O(E) (<see cref="SparseMatrixBuilder"/>) and solved with a sparse SPD
    /// <see cref="ConjugateGradient"/> — replacing the original dense N×N Laplacian + O(N³)
    /// <c>Matrix.Inverse</c>. The reduced interior Laplacian is symmetric positive-definite, so CG is
    /// the right solver. Two right-hand sides (u and v) share the same matrix.</para>
    /// </summary>
    public static class TutteEmbedding
    {
        /// <summary>
        /// Compute a planar UV per vertex. Boundary vertices land on the unit circle (ordered by the
        /// boundary loop); interior vertices solve the uniform-Laplacian system via CG. Pass
        /// <paramref name="maxIterations"/> &lt;= 0 to size it automatically. The caller owns and must
        /// dispose the returned array.
        /// </summary>
        public static NativeArray<float2> Compute(
            in MeshData mesh,
            Allocator allocator,
            int maxIterations = 0,
            float tolerance = ConjugateGradient.DefaultTolerance)
        {
            int n = mesh.VertexCount;
            var uv = new NativeArray<float2>(n, allocator);

            // 1. Pin the boundary loop to the unit circle, spaced uniformly by loop index.
            var loop = MeshBoundary.ExtractLoop(mesh, Allocator.Temp);
            int bCount = loop.Length;
            var isBoundary = new NativeArray<bool>(n, Allocator.Temp); // zero-init -> all false
            for (int k = 0; k < bCount; k++)
            {
                int v = loop[k];
                isBoundary[v] = true;
                float angle = 2f * math.PI * k / bCount;
                uv[v] = new float2(math.cos(angle), math.sin(angle));
            }

            // 2. Re-index interior vertices 0..m-1 (the unknowns).
            var interiorIndex = new NativeArray<int>(n, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            int m = 0;
            for (int v = 0; v < n; v++)
                interiorIndex[v] = isBoundary[v] ? -1 : m++;

            if (m == 0)
            {
                loop.Dispose();
                isBoundary.Dispose();
                interiorIndex.Dispose();
                return uv; // degenerate: every vertex is on the boundary
            }

            // 3. Assemble the reduced interior Laplacian A (m×m, SPD) and the two right-hand sides.
            //    Interior row i: deg(i)·u_i - Σ_{interior j∈N(i)} u_j = Σ_{boundary j∈N(i)} u_j^fixed.
            // These three are passed to the Burst CG job, so they must use a job-compatible allocator
            // (TempJob), not Allocator.Temp. They are disposed below within this same synchronous call.
            var builder = new SparseMatrixBuilder(m, m);
            var bU = new NativeArray<float>(m, Allocator.TempJob);
            var bV = new NativeArray<float>(m, Allocator.TempJob);
            for (int v = 0; v < n; v++)
            {
                if (isBoundary[v]) continue;
                int i = interiorIndex[v];
                mesh.GetNeighbours(v, out int start, out int count);
                builder.Add(i, i, count); // diagonal = full degree (uniform weights)
                for (int k = 0; k < count; k++)
                {
                    int w = mesh.AdjNeighbours[start + k];
                    if (isBoundary[w])
                    {
                        bU[i] += uv[w].x;
                        bV[i] += uv[w].y;
                    }
                    else
                    {
                        builder.Add(i, interiorIndex[w], -1f);
                    }
                }
            }

            var a = builder.Build(Allocator.TempJob);

            // 4. Solve A·u = bU and A·v = bV (same matrix, two RHS).
            int maxIt = maxIterations > 0 ? maxIterations : math.max(1000, 4 * m);
            var xu = ConjugateGradient.Run(a, bU, Allocator.TempJob, maxIt, tolerance);
            var xv = ConjugateGradient.Run(a, bV, Allocator.TempJob, maxIt, tolerance);

            // 5. Scatter the interior solution back into the per-vertex UV array.
            for (int v = 0; v < n; v++)
            {
                if (isBoundary[v]) continue;
                int i = interiorIndex[v];
                uv[v] = new float2(xu[i], xv[i]);
            }

            loop.Dispose();
            isBoundary.Dispose();
            interiorIndex.Dispose();
            bU.Dispose();
            bV.Dispose();
            xu.Dispose();
            xv.Dispose();
            a.Dispose();
            return uv;
        }
    }
}
