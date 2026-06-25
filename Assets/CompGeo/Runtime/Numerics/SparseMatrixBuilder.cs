using System;
using System.Collections.Generic;
using Unity.Collections;

namespace CompGeo.Numerics
{
    /// <summary>
    /// Assembles a <see cref="SparseMatrixCsr"/> from <c>(row, column, value)</c> triplets (COO). Matrix
    /// assembly is inherently sequential and one-shot, so it stays on the managed side (CODING-STYLE §5:
    /// construction may allocate; the hot path — the CG iterations / SpMV — does not). Duplicate
    /// <c>(row, column)</c> entries are <b>summed</b>, matching finite-element / Laplacian accumulation,
    /// and columns within each row are sorted ascending so the resulting CSR is canonical.
    /// </summary>
    public sealed class SparseMatrixBuilder
    {
        public readonly int Rows;
        public readonly int Cols;

        readonly List<int> _rows = new List<int>();
        readonly List<int> _cols = new List<int>();
        readonly List<float> _values = new List<float>();

        public SparseMatrixBuilder(int rows, int cols)
        {
            if (rows <= 0) throw new ArgumentOutOfRangeException(nameof(rows));
            if (cols <= 0) throw new ArgumentOutOfRangeException(nameof(cols));
            Rows = rows;
            Cols = cols;
        }

        /// <summary>Accumulate <paramref name="value"/> at <c>(row, column)</c>. Zero values are kept.</summary>
        public void Add(int row, int column, float value)
        {
            if ((uint)row >= (uint)Rows) throw new ArgumentOutOfRangeException(nameof(row));
            if ((uint)column >= (uint)Cols) throw new ArgumentOutOfRangeException(nameof(column));
            _rows.Add(row);
            _cols.Add(column);
            _values.Add(value);
        }

        /// <summary>
        /// Build the CSR matrix, allocating its native storage with <paramref name="allocator"/>. Sorts
        /// each row by column and sums duplicate entries. The caller owns and must dispose the result.
        /// </summary>
        public SparseMatrixCsr Build(Allocator allocator)
        {
            int nnzRaw = _rows.Count;

            // Bucket the triplets by row (counting sort on the row index).
            var rowStart = new int[Rows + 1];
            for (int i = 0; i < nnzRaw; i++) rowStart[_rows[i] + 1]++;
            for (int r = 0; r < Rows; r++) rowStart[r + 1] += rowStart[r];

            var colTmp = new int[nnzRaw];
            var valTmp = new float[nnzRaw];
            var fill = new int[Rows];
            for (int i = 0; i < nnzRaw; i++)
            {
                int r = _rows[i];
                int p = rowStart[r] + fill[r]++;
                colTmp[p] = _cols[i];
                valTmp[p] = _values[i];
            }

            // Per row: sort by column, then merge duplicate columns.
            var offsets = new NativeArray<int>(Rows + 1, allocator);
            var outCols = new List<int>(nnzRaw);
            var outVals = new List<float>(nnzRaw);
            for (int r = 0; r < Rows; r++)
            {
                offsets[r] = outCols.Count;
                int s = rowStart[r];
                int e = rowStart[r + 1];
                if (e > s)
                {
                    Array.Sort(colTmp, valTmp, s, e - s);
                    int j = s;
                    while (j < e)
                    {
                        int col = colTmp[j];
                        float acc = valTmp[j];
                        j++;
                        while (j < e && colTmp[j] == col) { acc += valTmp[j]; j++; }
                        outCols.Add(col);
                        outVals.Add(acc);
                    }
                }
            }
            offsets[Rows] = outCols.Count;

            int nnz = outCols.Count;
            var columnIndices = new NativeArray<int>(nnz, allocator, NativeArrayOptions.UninitializedMemory);
            var values = new NativeArray<float>(nnz, allocator, NativeArrayOptions.UninitializedMemory);
            for (int i = 0; i < nnz; i++) { columnIndices[i] = outCols[i]; values[i] = outVals[i]; }

            return new SparseMatrixCsr
            {
                RowOffsets = offsets,
                ColumnIndices = columnIndices,
                Values = values,
                Rows = Rows,
                Cols = Cols,
            };
        }
    }
}
