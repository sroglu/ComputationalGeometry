using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using Unity.Collections;
using Unity.Mathematics;

namespace CompGeo.Core.IO
{
    /// <summary>
    /// Reader for the Object File Format (.off) — the format used by the CENG789 meshes
    /// (man0.off, centaur, dragon, faces...). Parses vertices and faces into a <see cref="MeshData"/>
    /// with CSR adjacency. Polygonal faces (k &gt; 3) are fan-triangulated. Header variants that end
    /// in "OFF" (e.g. "COFF", "NOFF") are accepted; the optional per-vertex colour/normal columns
    /// they imply are not parsed.
    /// </summary>
    public static class OffReader
    {
        /// <summary>Read an OFF mesh from a file path.</summary>
        public static MeshData ReadFile(string path, Allocator allocator)
        {
            using var stream = File.OpenRead(path);
            return Read(stream, allocator);
        }

        /// <summary>Read an OFF mesh from a stream.</summary>
        public static MeshData Read(Stream stream, Allocator allocator)
        {
            using var reader = new StreamReader(stream);
            return Read(reader, allocator);
        }

        /// <summary>Read an OFF mesh from a text reader.</summary>
        public static MeshData Read(TextReader reader, Allocator allocator)
        {
            List<string> tokens = Tokenize(reader);
            int cursor = 0;

            if (tokens.Count == 0)
                throw new FormatException("Empty OFF stream.");

            string header = tokens[cursor++];
            if (!header.EndsWith("OFF", StringComparison.Ordinal))
                throw new FormatException($"Not an OFF file: header '{header}'.");

            int vCount = ParseInt(tokens, ref cursor);
            int fCount = ParseInt(tokens, ref cursor);
            ParseInt(tokens, ref cursor); // declared edge count — informational, often 0; ignored.

            var positions = new List<float3>(vCount);
            for (int i = 0; i < vCount; i++)
            {
                float x = ParseFloat(tokens, ref cursor);
                float y = ParseFloat(tokens, ref cursor);
                float z = ParseFloat(tokens, ref cursor);
                positions.Add(new float3(x, y, z));
            }

            var triangles = new List<int3>(fCount);
            for (int f = 0; f < fCount; f++)
            {
                int k = ParseInt(tokens, ref cursor);
                if (k < 3)
                {
                    cursor += k; // skip degenerate face indices
                    continue;
                }

                int first = ParseInt(tokens, ref cursor);
                int prev = ParseInt(tokens, ref cursor);
                for (int j = 2; j < k; j++)
                {
                    int next = ParseInt(tokens, ref cursor);
                    triangles.Add(new int3(first, prev, next)); // triangle fan around the first vertex
                    prev = next;
                }
            }

            return MeshBuilder.Build(positions, triangles, allocator);
        }

        /// <summary>
        /// Split the stream into whitespace-delimited tokens, dropping blank lines and
        /// <c>#</c> comments. OFF allows counts and coordinates to wrap arbitrarily across lines, so
        /// a flat token stream is the robust way to parse it.
        /// </summary>
        static List<string> Tokenize(TextReader reader)
        {
            var tokens = new List<string>();
            string line;
            while ((line = reader.ReadLine()) != null)
            {
                int hash = line.IndexOf('#');
                if (hash >= 0) line = line.Substring(0, hash);
                if (string.IsNullOrWhiteSpace(line)) continue;

                string[] parts = line.Split((char[])null, StringSplitOptions.RemoveEmptyEntries);
                tokens.AddRange(parts);
            }
            return tokens;
        }

        static int ParseInt(List<string> tokens, ref int cursor)
        {
            if (cursor >= tokens.Count)
                throw new FormatException("Unexpected end of OFF data while reading an integer.");
            return int.Parse(tokens[cursor++], CultureInfo.InvariantCulture);
        }

        static float ParseFloat(List<string> tokens, ref int cursor)
        {
            if (cursor >= tokens.Count)
                throw new FormatException("Unexpected end of OFF data while reading a float.");
            return float.Parse(tokens[cursor++], CultureInfo.InvariantCulture);
        }
    }
}
