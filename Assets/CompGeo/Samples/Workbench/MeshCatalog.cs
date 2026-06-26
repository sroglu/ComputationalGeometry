using System;
using System.Collections.Generic;
using System.IO;
using Unity.Collections;
using UnityEngine;
using CompGeo.Core;
using CompGeo.Core.IO;

namespace CompGeo.Samples
{
    /// <summary>
    /// Inspector-editable list of selectable sample meshes (the runtime mesh-picker source), replacing the
    /// original project's scene-baked mesh objects. Each entry is either an <c>.off</c> under
    /// <see cref="Application.streamingAssetsPath"/> or — when the path is blank — the procedural disk.
    /// </summary>
    [Serializable]
    public sealed class MeshCatalog
    {
        [Serializable]
        public struct Entry
        {
            public string name;

            [Tooltip("Path under StreamingAssets, e.g. meshes1/faces/face.off. Leave empty for the procedural disk.")]
            public string streamingPath;
        }

        public List<Entry> entries = DefaultEntries();

        public int Count => entries.Count;

        public string NameAt(int index) => entries[index].name;

        public string[] Names()
        {
            var names = new string[entries.Count];
            for (int i = 0; i < entries.Count; i++) names[i] = entries[i].name;
            return names;
        }

        /// <summary>
        /// Build the mesh at <paramref name="index"/>: load the OFF from StreamingAssets (normalised and
        /// oriented for viewing) or fall back to the procedural disk when the path is blank. The caller
        /// owns and must dispose the result.
        /// </summary>
        public MeshData Build(int index, Allocator allocator)
        {
            Entry e = entries[index];
            if (string.IsNullOrEmpty(e.streamingPath))
                return MeshFactory.BuildProceduralDisk(allocator);

            string path = Path.Combine(Application.streamingAssetsPath, e.streamingPath);
            MeshData mesh = path.EndsWith(".obj", StringComparison.OrdinalIgnoreCase)
                ? ObjReader.ReadFile(path, allocator)
                : OffReader.ReadFile(path, allocator);
            MeshFactory.NormalizeAndOrient(ref mesh);
            return mesh;
        }

        static List<Entry> DefaultEntries() => new List<Entry>
        {
            new Entry { name = "Face",        streamingPath = "meshes1/faces/face.off" },
            new Entry { name = "Face (alt)",  streamingPath = "meshes1/faces/facem.off" },
            new Entry { name = "Face (low)",  streamingPath = "meshes1/faces/face-low.off" },
            new Entry { name = "man0",        streamingPath = "meshes1/geodesics/man0.off" },
            new Entry { name = "horse0",      streamingPath = "meshes1/geodesics/horse0.off" },
            new Entry { name = "Dragon",      streamingPath = "meshes1/timing/dragon.obj" },
            new Entry { name = "Procedural disk", streamingPath = "" },
        };
    }
}
