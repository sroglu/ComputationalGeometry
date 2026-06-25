using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Rendering;
using CompGeo.Core;

namespace CompGeo.Visualization
{
    /// <summary>
    /// Draws a <see cref="MeshData"/> on the GPU as (1) an instanced-free point cloud — one mesh with
    /// <see cref="MeshTopology.Points"/> — and (2) an edge mesh with <see cref="MeshTopology.Lines"/>,
    /// plus an optional highlighted path. Per-vertex colour carries the geodesic heatmap / selection.
    ///
    /// This replaces the original project's GameObject-per-vertex / LineRenderer-per-edge rendering
    /// (docs/MIGRATION.md §2.A, §3): one mesh and one draw call per layer, no per-element material.
    /// The view owns its <see cref="Mesh"/>/<see cref="Material"/> objects — call <see cref="Dispose"/>.
    /// </summary>
    public sealed class MeshGpuView : IDisposable
    {
        const string ShaderName = "CompGeo/VertexColorUnlit";

        readonly Material _material;
        Mesh _pointsMesh;
        Mesh _edgesMesh;
        Mesh _pathMesh;
        Bounds _bounds;
        int _vertexCount;

        public MeshGpuView()
        {
            Shader shader = Shader.Find(ShaderName);
            if (shader == null)
                throw new InvalidOperationException($"Shader '{ShaderName}' not found — ensure CompGeo.Visualization/Shaders is imported under URP.");
            _material = new Material(shader);
        }

        /// <summary>Build the point and edge meshes from <paramref name="mesh"/>. Colours start white.</summary>
        public void Build(in MeshData mesh)
        {
            _vertexCount = mesh.VertexCount;
            var vertices = mesh.Positions.Reinterpret<Vector3>();

            var colors = new NativeArray<Color>(_vertexCount, Allocator.Temp);
            for (int i = 0; i < _vertexCount; i++) colors[i] = Color.white;

            // Points: one index per vertex.
            var pointIndices = new int[_vertexCount];
            for (int i = 0; i < _vertexCount; i++) pointIndices[i] = i;

            // Edges: each undirected CSR edge once (v < w), two indices per edge.
            var edgeIndices = new List<int>(mesh.AdjNeighbours.Length);
            for (int v = 0; v < _vertexCount; v++)
            {
                mesh.GetNeighbours(v, out int start, out int count);
                for (int k = 0; k < count; k++)
                {
                    int w = mesh.AdjNeighbours[start + k];
                    if (w > v) { edgeIndices.Add(v); edgeIndices.Add(w); }
                }
            }

            _pointsMesh = NewMesh("CompGeo Points");
            _pointsMesh.SetVertices(vertices);
            _pointsMesh.SetColors(colors);
            _pointsMesh.SetIndices(pointIndices, MeshTopology.Points, 0);
            _pointsMesh.RecalculateBounds();

            _edgesMesh = NewMesh("CompGeo Edges");
            _edgesMesh.SetVertices(vertices);
            _edgesMesh.SetColors(colors);
            _edgesMesh.SetIndices(edgeIndices.ToArray(), MeshTopology.Lines, 0);
            _edgesMesh.RecalculateBounds();

            _bounds = _pointsMesh.bounds;
            colors.Dispose();
        }

        /// <summary>
        /// Replace the vertex positions of the point and edge meshes in place (topology/colours unchanged).
        /// Used to animate per-frame geometry — e.g. morphing a surface to its planar parameterization
        /// (<c>UnfoldDemo</c>). <paramref name="positions"/> length must equal the vertex count.
        /// </summary>
        public void UpdatePositions(NativeArray<float3> positions)
        {
            var vertices = positions.Reinterpret<Vector3>();
            _pointsMesh.SetVertices(vertices);
            _edgesMesh.SetVertices(vertices);
            _pointsMesh.RecalculateBounds();
            _edgesMesh.RecalculateBounds();
            _bounds = _pointsMesh.bounds;
        }

        /// <summary>Recolour every vertex from a scalar field (e.g. a geodesic distance field) via <see cref="Heatmap"/>.</summary>
        public void SetHeatmap(NativeArray<float> field, Color unreachable)
        {
            var colors = new NativeArray<Color>(_vertexCount, Allocator.Temp);
            Heatmap.Apply(field, colors, unreachable);
            SetColors(colors);
            colors.Dispose();
        }

        /// <summary>Set per-vertex colours directly (length must equal the vertex count).</summary>
        public void SetColors(NativeArray<Color> colors)
        {
            _pointsMesh.SetColors(colors);
            _edgesMesh.SetColors(colors);
        }

        /// <summary>Build/replace the highlighted path from a vertex sequence (e.g. a reconstructed shortest path).</summary>
        public void SetPath(in MeshData mesh, NativeArray<int> path, Color color)
        {
            if (_pathMesh == null) _pathMesh = NewMesh("CompGeo Path");

            int n = path.Length;
            var verts = new Vector3[n];
            var cols = new Color[n];
            for (int i = 0; i < n; i++)
            {
                verts[i] = (Vector3)(float3)mesh.Positions[path[i]];
                cols[i] = color;
            }

            var lines = new int[n > 1 ? (n - 1) * 2 : 0];
            for (int i = 0; i < n - 1; i++) { lines[i * 2] = i; lines[i * 2 + 1] = i + 1; }

            _pathMesh.Clear();
            _pathMesh.SetVertices(verts);
            _pathMesh.SetColors(cols);
            _pathMesh.SetIndices(lines, MeshTopology.Lines, 0);
            _pathMesh.RecalculateBounds();
        }

        /// <summary>Issue the draw calls for this frame at <paramref name="objectToWorld"/>. Call from Update/OnRenderObject.</summary>
        public void DrawNow(Matrix4x4 objectToWorld)
        {
            var rp = new RenderParams(_material)
            {
                worldBounds = _bounds,
                shadowCastingMode = ShadowCastingMode.Off,
                receiveShadows = false,
            };
            if (_pointsMesh != null) Graphics.RenderMesh(rp, _pointsMesh, 0, objectToWorld);
            if (_edgesMesh != null) Graphics.RenderMesh(rp, _edgesMesh, 0, objectToWorld);
            if (_pathMesh != null) Graphics.RenderMesh(rp, _pathMesh, 0, objectToWorld);
        }

        static Mesh NewMesh(string name)
        {
            var m = new Mesh { name = name, indexFormat = IndexFormat.UInt32 };
            m.MarkDynamic();
            return m;
        }

        public void Dispose()
        {
            DestroyObject(_pointsMesh);
            DestroyObject(_edgesMesh);
            DestroyObject(_pathMesh);
            DestroyObject(_material);
            _pointsMesh = _edgesMesh = _pathMesh = null;
        }

        static void DestroyObject(UnityEngine.Object o)
        {
            if (o == null) return;
            if (Application.isPlaying) UnityEngine.Object.Destroy(o);
            else UnityEngine.Object.DestroyImmediate(o);
        }
    }
}
