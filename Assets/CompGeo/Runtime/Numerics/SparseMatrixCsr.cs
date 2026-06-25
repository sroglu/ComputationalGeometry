using System;
using Unity.Collections;

namespace CompGeo.Numerics
{
    /// <summary>
    /// Sparse matrix in CSR (Compressed Sparse Row) form, held in <see cref="NativeArray{T}"/> so the
    /// sparse matrix-vector product (SpMV) is Burst-consumable with zero managed allocation
    /// (docs/MIGRATION.md §6.C: the original HW2 used a dense N×N matrix + explicit inverse — this is the
    /// sparse, O(nnz) replacement). Build instances with <see cref="SparseMatrixBuilder"/>; the owner must
    /// call <see cref="Dispose"/>.
    ///
    /// Layout: row <c>r</c> occupies <c>[RowOffsets[r], RowOffsets[r + 1])</c> in
    /// <see cref="ColumnIndices"/> / <see cref="Values"/>. Columns within a row are ascending and unique.
    /// </summary>
    public struct SparseMatrixCsr : IDisposable
    {
        /// <summary>CSR row offsets; length is <see cref="Rows"/> + 1.</summary>
        public NativeArray<int> RowOffsets;

        /// <summary>Column index of each stored entry; length is the non-zero count.</summary>
        public NativeArray<int> ColumnIndices;

        /// <summary>Value of each stored entry; length is the non-zero count.</summary>
        public NativeArray<float> Values;

        public int Rows;
        public int Cols;

        public int NonZeros => Values.IsCreated ? Values.Length : 0;

        /// <summary>
        /// Compute <c>result = A · x</c>. <paramref name="x"/> has length <see cref="Cols"/>,
        /// <paramref name="result"/> has length <see cref="Rows"/>.
        /// </summary>
        public void Multiply(NativeArray<float> x, NativeArray<float> result)
            => Multiply(RowOffsets, ColumnIndices, Values, Rows, x, result);

        /// <summary>
        /// Raw-array SpMV (the form a Burst job passes its <c>[ReadOnly]</c> CSR fields to): Burst-friendly,
        /// no delegates/boxing/managed allocation.
        /// </summary>
        public static void Multiply(
            NativeArray<int> rowOffsets,
            NativeArray<int> columnIndices,
            NativeArray<float> values,
            int rows,
            NativeArray<float> x,
            NativeArray<float> result)
        {
            for (int r = 0; r < rows; r++)
            {
                int start = rowOffsets[r];
                int end = rowOffsets[r + 1];
                float sum = 0f;
                for (int k = start; k < end; k++)
                    sum += values[k] * x[columnIndices[k]];
                result[r] = sum;
            }
        }

        public void Dispose()
        {
            if (RowOffsets.IsCreated) RowOffsets.Dispose();
            if (ColumnIndices.IsCreated) ColumnIndices.Dispose();
            if (Values.IsCreated) Values.Dispose();
        }
    }
}
