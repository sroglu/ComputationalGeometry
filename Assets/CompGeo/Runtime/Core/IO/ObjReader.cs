using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using Unity.Collections;
using Unity.Mathematics;

namespace CompGeo.Core.IO
{
    /// <summary>
    /// Reader for the Wavefront Object Format (.obj) — used by the larger CENG789 benchmark meshes
    /// (e.g. dragon.obj). Parses <c>v</c> vertex lines and <c>f</c> face lines into a
    /// <see cref="MeshData"/> with CSR adjacency; only geometry is read. Face vertex references may be
    /// plain (<c>f 1 2 3</c>) or carry texture/normal indices (<c>f 1/4/7 ...</c>) — only the position
    /// index is used. Indices are 1-based and negative (relative) indices are supported. Polygonal faces
    /// (k &gt; 3) are fan-triangulated; everything other than <c>v</c>/<c>f</c> is ignored.
    /// </summary>
    public static class ObjReader
    {
        /// <summary>Read an OBJ mesh from a file path.</summary>
        public static MeshData ReadFile(string path, Allocator allocator)
        {
            using var stream = File.OpenRead(path);
            return Read(stream, allocator);
        }

        /// <summary>Read an OBJ mesh from a stream.</summary>
        public static MeshData Read(Stream stream, Allocator allocator)
        {
            using var reader = new StreamReader(stream);
            return Read(reader, allocator);
        }

        /// <summary>Read an OBJ mesh from a text reader.</summary>
        public static MeshData Read(TextReader reader, Allocator allocator)
        {
            var positions = new List<float3>();
            var triangles = new List<int3>();

            string line;
            while ((line = reader.ReadLine()) != null)
            {
                int hash = line.IndexOf('#');
                if (hash >= 0) line = line.Substring(0, hash);
                if (string.IsNullOrWhiteSpace(line)) continue;

                string[] parts = line.Split((char[])null, StringSplitOptions.RemoveEmptyEntries);
                if (parts.Length == 0) continue;

                if (parts[0] == "v")
                {
                    float x = ParseFloat(parts, 1);
                    float y = ParseFloat(parts, 2);
                    float z = ParseFloat(parts, 3);
                    positions.Add(new float3(x, y, z));
                }
                else if (parts[0] == "f" && parts.Length >= 4)
                {
                    // Fan-triangulate, resolving each "v/vt/vn" token to a 0-based position index.
                    int first = ResolveIndex(parts[1], positions.Count);
                    int prev = ResolveIndex(parts[2], positions.Count);
                    for (int j = 3; j < parts.Length; j++)
                    {
                        int next = ResolveIndex(parts[j], positions.Count);
                        triangles.Add(new int3(first, prev, next));
                        prev = next;
                    }
                }
            }

            if (positions.Count == 0)
                throw new FormatException("No vertices found in OBJ stream.");

            return MeshBuilder.Build(positions, triangles, allocator);
        }

        /// <summary>
        /// Resolve an OBJ face vertex reference ("v", "v/vt", "v/vt/vn", "v//vn") to a 0-based position
        /// index. OBJ indices are 1-based; negative values count back from the current vertex total.
        /// </summary>
        static int ResolveIndex(string token, int vertexCount)
        {
            int slash = token.IndexOf('/');
            string vPart = slash >= 0 ? token.Substring(0, slash) : token;
            int idx = int.Parse(vPart, CultureInfo.InvariantCulture);
            return idx > 0 ? idx - 1 : vertexCount + idx;
        }

        static float ParseFloat(string[] parts, int i)
        {
            if (i >= parts.Length)
                throw new FormatException("Unexpected end of OBJ vertex line while reading a float.");
            return float.Parse(parts[i], CultureInfo.InvariantCulture);
        }
    }
}
