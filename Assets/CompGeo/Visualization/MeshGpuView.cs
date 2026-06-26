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
        Material _surfaceMaterialOverride; // optional, externally owned (e.g. the UV-checker material)
        Mesh _pointsMesh;
        Mesh _edgesMesh;
        Mesh _surfaceMesh;
        Mesh _normalsMesh;
        Mesh _pathMesh;
        Bounds _bounds;
        int _vertexCount;

        /// <summary>
        /// When true, <see cref="DrawNow"/> also draws the filled triangle surface (per-vertex coloured),
        /// not just the point cloud and wireframe. Off by default so the heatmap/point views stay as-is;
        /// surfaces (e.g. the unfold demo) turn it on for a legible solid render.
        /// </summary>
        public bool ShowSurface;

        /// <summary>Draw the vertices as small instanced spheres (default true). Off for a clean surface view.</summary>
        public bool ShowPoints = true;

        /// <summary>Draw the wireframe edges (default true). Turn off for a clean surface-only view.</summary>
        public bool ShowEdges = true;

        /// <summary>Draw per-vertex normal spikes (the original "Show Normals" view). Off by default.</summary>
        public bool ShowNormals;

        /// <summary>Colour of the normal spikes.</summary>
        public Color NormalColor = new Color(0.95f, 0.25f, 0.4f, 1f); // red/pink, like the original

        /// <summary>Normal-spike length as a fraction of the mesh's bounding size.</summary>
        public float NormalLengthScale = 0.03f;

        /// <summary>Colour of the vertex spheres.</summary>
        public Color PointColor = new Color(0f, 0.85f, 1f, 1f); // cyan

        /// <summary>Uniform wireframe colour. Edges keep this colour and are <b>not</b> recoloured by the
        /// heatmap (so the wireframe reads as a clean cyan mesh, like the original "Show Edges").</summary>
        public Color EdgeColor = new Color(0f, 0.85f, 1f, 1f); // cyan

        /// <summary>Vertex sphere radius as a fraction of the mesh's bounding size.</summary>
        public float PointRadiusScale = 0.008f;

        /// <summary>Path tube radius as a fraction of the mesh's bounding size (live: re-call SetPath to apply).</summary>
        public float PathRadiusScale = 0.006f;

        static Mesh s_sphere;
        readonly Material _pointMaterial;
        Vector3[] _pointPositions;
        readonly Matrix4x4[] _pointMatrices = new Matrix4x4[1023];
        float _pointRadius;

        public MeshGpuView()
        {
            Shader shader = Shader.Find(ShaderName);
            if (shader == null)
                throw new InvalidOperationException($"Shader '{ShaderName}' not found — ensure CompGeo.Visualization/Shaders is imported under URP.");
            _material = new Material(shader);

            // Cyan instanced material for the vertex spheres (URP Unlit supports GPU instancing).
            _pointMaterial = new Material(Shader.Find("Universal Render Pipeline/Unlit")) { enableInstancing = true };
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
            var edgeColors = new NativeArray<Color>(_vertexCount, Allocator.Temp);
            for (int i = 0; i < _vertexCount; i++) edgeColors[i] = EdgeColor;
            _edgesMesh.SetColors(edgeColors);
            edgeColors.Dispose();
            _edgesMesh.SetIndices(edgeIndices.ToArray(), MeshTopology.Lines, 0);
            _edgesMesh.RecalculateBounds();

            // Filled surface: three indices per triangle (drawn only when ShowSurface).
            var triIndices = new int[mesh.TriangleCount * 3];
            for (int t = 0; t < mesh.TriangleCount; t++)
            {
                int3 tri = mesh.Triangles[t];
                triIndices[t * 3] = tri.x;
                triIndices[t * 3 + 1] = tri.y;
                triIndices[t * 3 + 2] = tri.z;
            }
            _surfaceMesh = NewMesh("CompGeo Surface");
            _surfaceMesh.SetVertices(vertices);
            _surfaceMesh.SetColors(colors);
            _surfaceMesh.SetIndices(triIndices, MeshTopology.Triangles, 0);
            _surfaceMesh.RecalculateBounds();

            _bounds = _pointsMesh.bounds;
            colors.Dispose();

            BuildNormalsMesh(mesh);

            // Vertex spheres: cache positions + size for instanced drawing.
            _pointPositions = vertices.ToArray();
            _pointRadius = PointRadiusScale * _bounds.size.magnitude;
            _pointMaterial.SetColor("_BaseColor", PointColor);
        }

        /// <summary>
        /// Build the red normal-spike line mesh: one line per vertex from its position along its
        /// area-weighted vertex normal (the original homework's "Show Normals" view). Drawn only when
        /// <see cref="ShowNormals"/>.
        /// </summary>
        void BuildNormalsMesh(in MeshData mesh)
        {
            int vc = mesh.VertexCount;
            var nrm = new Vector3[vc];
            for (int t = 0; t < mesh.TriangleCount; t++)
            {
                int3 tri = mesh.Triangles[t];
                Vector3 p0 = (Vector3)(float3)mesh.Positions[tri.x];
                Vector3 p1 = (Vector3)(float3)mesh.Positions[tri.y];
                Vector3 p2 = (Vector3)(float3)mesh.Positions[tri.z];
                Vector3 fn = Vector3.Cross(p1 - p0, p2 - p0); // area-weighted (not normalized)
                nrm[tri.x] += fn; nrm[tri.y] += fn; nrm[tri.z] += fn;
            }

            float len = NormalLengthScale * _bounds.size.magnitude;
            Vector3 centre = _bounds.center;
            var verts = new Vector3[vc * 2];
            var cols = new Color[vc * 2];
            var idx = new int[vc * 2];
            for (int i = 0; i < vc; i++)
            {
                Vector3 p = (Vector3)(float3)mesh.Positions[i];
                Vector3 n = nrm[i].sqrMagnitude > 1e-12f ? nrm[i].normalized : (p - centre).normalized;
                if (Vector3.Dot(n, p - centre) < 0f) n = -n; // orient outward so the spikes read consistently
                verts[2 * i] = p;
                verts[2 * i + 1] = p + n * len;
                cols[2 * i] = NormalColor;
                cols[2 * i + 1] = NormalColor;
                idx[2 * i] = 2 * i;
                idx[2 * i + 1] = 2 * i + 1;
            }

            _normalsMesh = NewMesh("CompGeo Normals");
            _normalsMesh.SetVertices(verts);
            _normalsMesh.SetColors(cols);
            _normalsMesh.SetIndices(idx, MeshTopology.Lines, 0);
            _normalsMesh.RecalculateBounds();
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
            _surfaceMesh.SetVertices(vertices);
            _pointsMesh.RecalculateBounds();
            _edgesMesh.RecalculateBounds();
            _surfaceMesh.RecalculateBounds();
            _bounds = _pointsMesh.bounds;

            if (_pointPositions == null || _pointPositions.Length != vertices.Length)
                _pointPositions = new Vector3[vertices.Length];
            vertices.CopyTo(_pointPositions);
        }

        /// <summary>Recolour every vertex from a scalar field (e.g. a geodesic distance field) via <see cref="Heatmap"/>.</summary>
        public void SetHeatmap(NativeArray<float> field, Color unreachable)
        {
            var colors = new NativeArray<Color>(_vertexCount, Allocator.Temp);
            Heatmap.Apply(field, colors, unreachable);
            SetColors(colors);
            colors.Dispose();
        }

        /// <summary>
        /// Set per-vertex colours for the point and surface layers (e.g. the geodesic heatmap). The
        /// wireframe keeps its uniform <see cref="EdgeColor"/> so it stays a clean cyan mesh.
        /// </summary>
        public void SetColors(NativeArray<Color> colors)
        {
            _pointsMesh.SetColors(colors);
            _surfaceMesh.SetColors(colors);
        }

        /// <summary>
        /// Override the material used for the filled surface (e.g. a UV-checker shader that paints a crisp,
        /// tessellation-independent pattern). Pass <c>null</c> to fall back to the per-vertex-colour
        /// material. The override is owned by the caller, not disposed here.
        /// </summary>
        public void SetSurfaceMaterial(Material material) => _surfaceMaterialOverride = material;

        /// <summary>Set the surface mesh's UV0 channel (length must equal the vertex count).</summary>
        public void SetSurfaceUVs(NativeArray<float2> uv) => _surfaceMesh.SetUVs(0, uv.Reinterpret<Vector2>());

        /// <summary>
        /// Build/replace the highlighted path from a vertex sequence as a thick coloured <b>tube</b> (a
        /// 1px line is nearly invisible). The tube pokes out of the surface so it reads clearly over the
        /// heatmap.
        /// </summary>
        public void SetPath(in MeshData mesh, NativeArray<int> path, Color color)
        {
            if (_pathMesh == null) _pathMesh = NewMesh("CompGeo Path");
            _pathMesh.Clear();

            int n = path.Length;
            if (n < 2) return;

            var center = new Vector3[n];
            for (int i = 0; i < n; i++) center[i] = (Vector3)(float3)mesh.Positions[path[i]];

            const int sides = 6;
            float radius = Mathf.Max(1e-5f, PathRadiusScale * _bounds.size.magnitude);
            var verts = new Vector3[n * sides];
            var cols = new Color[n * sides];

            Vector3 u = Vector3.zero;
            for (int i = 0; i < n; i++)
            {
                Vector3 tangent = center[Mathf.Min(i + 1, n - 1)] - center[Mathf.Max(i - 1, 0)];
                if (tangent.sqrMagnitude < 1e-12f) tangent = Vector3.forward;
                tangent.Normalize();

                if (i == 0)
                    u = Vector3.Cross(tangent, Mathf.Abs(tangent.y) < 0.9f ? Vector3.up : Vector3.right);
                else
                    u -= tangent * Vector3.Dot(u, tangent); // parallel-transport to avoid twist
                if (u.sqrMagnitude < 1e-12f) u = Vector3.Cross(tangent, Vector3.right);
                u.Normalize();
                Vector3 v = Vector3.Cross(tangent, u);

                for (int k = 0; k < sides; k++)
                {
                    float a = 2f * Mathf.PI * k / sides;
                    verts[i * sides + k] = center[i] + (Mathf.Cos(a) * u + Mathf.Sin(a) * v) * radius;
                    cols[i * sides + k] = color;
                }
            }

            var tris = new int[(n - 1) * sides * 6];
            int ti = 0;
            for (int i = 0; i < n - 1; i++)
            for (int k = 0; k < sides; k++)
            {
                int k2 = (k + 1) % sides;
                int a = i * sides + k, b = i * sides + k2, c = (i + 1) * sides + k, d = (i + 1) * sides + k2;
                tris[ti++] = a; tris[ti++] = c; tris[ti++] = b;
                tris[ti++] = b; tris[ti++] = c; tris[ti++] = d;
            }

            _pathMesh.SetVertices(verts);
            _pathMesh.SetColors(cols);
            _pathMesh.SetTriangles(tris, 0);
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
            if (ShowSurface && _surfaceMesh != null)
            {
                Material surfaceMat = _surfaceMaterialOverride != null ? _surfaceMaterialOverride : _material;
                var rpSurface = new RenderParams(surfaceMat)
                {
                    worldBounds = _bounds,
                    shadowCastingMode = ShadowCastingMode.Off,
                    receiveShadows = false,
                };
                Graphics.RenderMesh(rpSurface, _surfaceMesh, 0, objectToWorld);
            }
            if (ShowPoints) DrawPointSpheres(objectToWorld);
            if (ShowEdges && _edgesMesh != null) Graphics.RenderMesh(rp, _edgesMesh, 0, objectToWorld);
            if (ShowNormals && _normalsMesh != null) Graphics.RenderMesh(rp, _normalsMesh, 0, objectToWorld);
            if (_pathMesh != null) Graphics.RenderMesh(rp, _pathMesh, 0, objectToWorld);
        }

        /// <summary>
        /// Draw the vertices as small cyan spheres via GPU instancing (one batched draw per ≤1023 verts).
        /// Solid 3D markers replace 1px points, which on some backends rendered as huge quads and
        /// z-fought the surface.
        /// </summary>
        void DrawPointSpheres(Matrix4x4 objectToWorld)
        {
            if (_pointPositions == null || _pointPositions.Length == 0 || _pointMaterial == null) return;

            Mesh sphere = SphereMesh();
            var prp = new RenderParams(_pointMaterial)
            {
                worldBounds = _bounds,
                shadowCastingMode = ShadowCastingMode.Off,
                receiveShadows = false,
            };
            Vector3 scale = Vector3.one * _pointRadius;
            int n = _pointPositions.Length;
            for (int i = 0; i < n;)
            {
                int count = Mathf.Min(_pointMatrices.Length, n - i);
                for (int k = 0; k < count; k++)
                    _pointMatrices[k] = objectToWorld * Matrix4x4.TRS(_pointPositions[i + k], Quaternion.identity, scale);
                Graphics.RenderMeshInstanced(prp, sphere, 0, _pointMatrices, count);
                i += count;
            }
        }

        static Mesh SphereMesh()
        {
            if (s_sphere != null) return s_sphere;

            float t = (1f + Mathf.Sqrt(5f)) / 2f; // icosahedron (20 tris) — round enough at marker scale
            var v = new[]
            {
                new Vector3(-1, t, 0), new Vector3(1, t, 0), new Vector3(-1, -t, 0), new Vector3(1, -t, 0),
                new Vector3(0, -1, t), new Vector3(0, 1, t), new Vector3(0, -1, -t), new Vector3(0, 1, -t),
                new Vector3(t, 0, -1), new Vector3(t, 0, 1), new Vector3(-t, 0, -1), new Vector3(-t, 0, 1),
            };
            for (int i = 0; i < v.Length; i++) v[i] = v[i].normalized;
            var tris = new[]
            {
                0, 11, 5, 0, 5, 1, 0, 1, 7, 0, 7, 10, 0, 10, 11,
                1, 5, 9, 5, 11, 4, 11, 10, 2, 10, 7, 6, 7, 1, 8,
                3, 9, 4, 3, 4, 2, 3, 2, 6, 3, 6, 8, 3, 8, 9,
                4, 9, 5, 2, 4, 11, 6, 2, 10, 8, 6, 7, 9, 8, 1,
            };
            s_sphere = new Mesh { name = "CompGeo PointSphere" };
            s_sphere.vertices = v;
            s_sphere.triangles = tris;
            s_sphere.RecalculateNormals();
            s_sphere.RecalculateBounds();
            return s_sphere;
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
            DestroyObject(_surfaceMesh);
            DestroyObject(_normalsMesh);
            DestroyObject(_pathMesh);
            DestroyObject(_material);
            DestroyObject(_pointMaterial);
            _pointsMesh = _edgesMesh = _surfaceMesh = _normalsMesh = _pathMesh = null;
        }

        static void DestroyObject(UnityEngine.Object o)
        {
            if (o == null) return;
            if (Application.isPlaying) UnityEngine.Object.Destroy(o);
            else UnityEngine.Object.DestroyImmediate(o);
        }
    }
}
