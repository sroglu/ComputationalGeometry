using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using CompGeo.Core;
using CompGeo.MeshProcessing.Parameterization;
using CompGeo.Visualization;

namespace CompGeo.Samples
{
    /// <summary>
    /// Static Step 4–5 demo: a non-planar disk-topology surface (left) and its flattened
    /// <see cref="TutteEmbedding"/> parameterization on the unit disk (right), drawn side by side and
    /// shaded with the same checkerboard. The checker is computed from the flat UV, so the same cells
    /// line up on both — you can read off how each patch of the surface maps into the plane (and that
    /// the flat result is fold-free, Tutte's guarantee). Nothing animates.
    ///
    /// Drop on a GameObject at the origin; a tagged MainCamera looking at it. Requires the URP pipeline
    /// asset assigned (see README) so <c>CompGeo/VertexColorUnlit</c> renders.
    /// </summary>
    public sealed class UnfoldDemo : MonoBehaviour
    {
        [Header("Procedural disk surface")]
        [Min(2)] public int gridSize = 48;
        public float spacing = 0.045f;
        public float heightAmplitude = 0.25f;

        [Header("Checkerboard / layout")]
        // Keep well below gridSize (≈ 4 vertices per checker cell) so the per-vertex checker is
        // properly sampled — a frequency near the vertex resolution aliases into coarse blobs.
        [Min(1)] public int checkerFrequency = 12;
        public Color checkerA = new Color(0.95f, 0.95f, 0.95f, 1f);
        public Color checkerB = new Color(0.15f, 0.35f, 0.85f, 1f);
        public float separation = 1.35f;

        MeshData _mesh;
        MeshData _flatMesh;
        MeshGpuView _view;
        MeshGpuView _flatView;

        void Start()
        {
            BuildGridData(gridSize, spacing, heightAmplitude, out var positions, out var triangles);
            _mesh = MeshBuilder.Build(positions, triangles, Allocator.Persistent);

            var uv = TutteEmbedding.Compute(_mesh, Allocator.Persistent);

            // Flat mesh: same topology, positions taken from the UV plane (y = 0).
            var flat = new List<float3>(uv.Length);
            for (int i = 0; i < uv.Length; i++) flat.Add(new float3(uv[i].x, 0f, uv[i].y));
            _flatMesh = MeshBuilder.Build(flat, triangles, Allocator.Persistent);

            // Shared checkerboard from the UV, so the cells correspond on both meshes.
            var colors = new NativeArray<Color>(uv.Length, Allocator.Temp);
            for (int i = 0; i < uv.Length; i++) colors[i] = Checker(uv[i]);

            _view = new MeshGpuView { ShowSurface = true, ShowPoints = false, ShowEdges = false };
            _view.Build(_mesh);
            _view.SetColors(colors);

            _flatView = new MeshGpuView { ShowSurface = true, ShowPoints = false, ShowEdges = false };
            _flatView.Build(_flatMesh);
            _flatView.SetColors(colors);

            colors.Dispose();
            uv.Dispose();
        }

        void Update()
        {
            Matrix4x4 baseM = transform.localToWorldMatrix;
            _view.DrawNow(baseM * Matrix4x4.Translate(new Vector3(-separation, 0f, 0f)));
            _flatView.DrawNow(baseM * Matrix4x4.Translate(new Vector3(separation, 0f, 0f)));
        }

        Color Checker(float2 uv)
        {
            int cx = (int)math.floor((uv.x * 0.5f + 0.5f) * checkerFrequency);
            int cy = (int)math.floor((uv.y * 0.5f + 0.5f) * checkerFrequency);
            return ((cx + cy) & 1) == 0 ? checkerA : checkerB;
        }

        void OnDestroy()
        {
            _view?.Dispose();
            _flatView?.Dispose();
            _mesh.Dispose();
            _flatMesh.Dispose();
        }

        static void BuildGridData(int m, float spacing, float amp, out List<float3> positions, out List<int3> triangles)
        {
            float half = (m - 1) * spacing * 0.5f; // centre the surface on the origin
            positions = new List<float3>(m * m);
            for (int j = 0; j < m; j++)
            for (int i = 0; i < m; i++)
            {
                float y = amp * math.sin(i * 0.5f) * math.cos(j * 0.5f);
                positions.Add(new float3(i * spacing - half, y, j * spacing - half));
            }

            triangles = new List<int3>((m - 1) * (m - 1) * 2);
            for (int j = 0; j < m - 1; j++)
            for (int i = 0; i < m - 1; i++)
            {
                int a = j * m + i;
                int b = j * m + i + 1;
                int c = (j + 1) * m + i;
                int d = (j + 1) * m + i + 1;
                triangles.Add(new int3(a, b, d));
                triangles.Add(new int3(a, d, c));
            }
        }
    }
}
