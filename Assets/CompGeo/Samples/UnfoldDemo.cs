using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using CompGeo.Core;
using CompGeo.MeshProcessing.Parameterization;
using CompGeo.Visualization;

namespace CompGeo.Samples
{
    /// <summary>
    /// Step 4–5 demo: builds a non-planar disk-topology surface, computes its <see cref="TutteEmbedding"/>
    /// (uniform-Laplacian unfold to the unit circle), and continuously morphs the GPU view between the
    /// 3D surface and its flattened 2D parameterization. Vertices are checkerboard-coloured by UV, so the
    /// parameter grid is visible both wrapped on the surface and laid flat — and the flat state is visibly
    /// fold-free (Tutte's guarantee).
    ///
    /// Drop on a GameObject at the origin; ensure a tagged MainCamera looks at it. Requires the URP
    /// pipeline asset assigned (see README) so <c>CompGeo/VertexColorUnlit</c> renders.
    /// </summary>
    public sealed class UnfoldDemo : MonoBehaviour
    {
        [Header("Procedural disk surface")]
        [Min(2)] public int gridSize = 28;
        public float spacing = 0.08f;
        public float heightAmplitude = 0.35f;

        [Header("Unfold morph")]
        [Min(0.1f)] public float morphPeriod = 4f;
        [Min(1)] public int checkerFrequency = 10;
        public Color checkerA = new Color(0.95f, 0.95f, 0.95f, 1f);
        public Color checkerB = new Color(0.15f, 0.35f, 0.85f, 1f);

        MeshData _mesh;
        MeshGpuView _view;
        NativeArray<float3> _surface; // original 3D positions (centred)
        NativeArray<float3> _flat;    // UV positions on the y = 0 plane
        NativeArray<float3> _morph;   // per-frame interpolation

        void Start()
        {
            _mesh = BuildGrid(gridSize, spacing, heightAmplitude);
            _view = new MeshGpuView();
            _view.Build(_mesh);

            var uv = TutteEmbedding.Compute(_mesh, Allocator.Persistent);

            int n = _mesh.VertexCount;
            _surface = new NativeArray<float3>(n, Allocator.Persistent);
            _flat = new NativeArray<float3>(n, Allocator.Persistent);
            _morph = new NativeArray<float3>(n, Allocator.Persistent);

            // Centre the surface on the origin so it morphs in place over the flat disk.
            float half = 0.5f * (gridSize - 1) * spacing;
            for (int i = 0; i < n; i++)
            {
                float3 p = _mesh.Positions[i];
                _surface[i] = new float3(p.x - half, p.y, p.z - half);
                _flat[i] = new float3(uv[i].x, 0f, uv[i].y);
            }

            // Checkerboard colour from the UV coordinates (fixed for the run).
            var colors = new NativeArray<Color>(n, Allocator.Temp);
            for (int i = 0; i < n; i++) colors[i] = Checker(uv[i]);
            _view.SetColors(colors);
            colors.Dispose();

            uv.Dispose();
        }

        void Update()
        {
            // Smooth ping-pong in [0,1]: 0 = 3D surface, 1 = flat parameterization.
            float t = 0.5f - 0.5f * math.cos(Time.time / morphPeriod * 2f * math.PI);
            for (int i = 0; i < _morph.Length; i++)
                _morph[i] = math.lerp(_surface[i], _flat[i], t);

            _view.UpdatePositions(_morph);
            _view.DrawNow(transform.localToWorldMatrix);
        }

        Color Checker(float2 uv)
        {
            // Map the unit-disk UV from [-1,1] to [0,1], then tile.
            int cx = (int)math.floor((uv.x * 0.5f + 0.5f) * checkerFrequency);
            int cy = (int)math.floor((uv.y * 0.5f + 0.5f) * checkerFrequency);
            return ((cx + cy) & 1) == 0 ? checkerA : checkerB;
        }

        void OnDestroy()
        {
            _view?.Dispose();
            if (_surface.IsCreated) _surface.Dispose();
            if (_flat.IsCreated) _flat.Dispose();
            if (_morph.IsCreated) _morph.Dispose();
            _mesh.Dispose();
        }

        static MeshData BuildGrid(int m, float spacing, float amp)
        {
            var positions = new System.Collections.Generic.List<float3>(m * m);
            for (int j = 0; j < m; j++)
            for (int i = 0; i < m; i++)
            {
                float y = amp * math.sin(i * 0.5f) * math.cos(j * 0.5f);
                positions.Add(new float3(i * spacing, y, j * spacing));
            }

            var triangles = new System.Collections.Generic.List<int3>((m - 1) * (m - 1) * 2);
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

            return MeshBuilder.Build(positions, triangles, Allocator.Persistent);
        }
    }
}
