using System;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using CompGeo.Core;
using CompGeo.MeshProcessing.Parameterization;

namespace CompGeo.Samples
{
    /// <summary>
    /// HW2 unfold mode (the new-core form of the original <c>UI2.cs</c> + <c>Model.UnfoldModel</c>):
    /// computes the <see cref="TutteEmbedding"/> of the active mesh and continuously morphs the surface
    /// between its 3D shape and the flat parameterization. Vertices are checkerboard-coloured by their flat
    /// UV so the mapping stays legible, and the flat target is Procrustes-aligned to the surface so the
    /// morph reads as a clean flatten-in-place (no global spin/scale). Meshes without a boundary (closed
    /// surfaces like man0) can't be parameterized — the mode logs that and just shows the mesh.
    /// </summary>
    [RequireComponent(typeof(Workbench))]
    public sealed class UnfoldMode : MonoBehaviour
    {
        [Min(1)] public int checkerFrequency = 12;
        public Color checkerA = new Color(0.95f, 0.95f, 0.95f, 1f);
        public Color checkerB = new Color(0.15f, 0.35f, 0.85f, 1f);
        [Min(0.1f)] public float morphPeriod = 4f;
        public bool morphing = true;

        Workbench _wb;
        NativeArray<float3> _surface;
        NativeArray<float3> _flat;
        NativeArray<float3> _morph;
        bool _ready;

        void Awake() => _wb = GetComponent<Workbench>();
        void OnEnable() => _wb.MeshChanged += OnMeshChanged;
        void OnDisable() { _wb.MeshChanged -= OnMeshChanged; Free(); }

        public void SetMorphing(bool on) => morphing = on;
        public void ToggleMorphing() => morphing = !morphing;

        void OnMeshChanged()
        {
            Free();
            MeshData mesh = _wb.Mesh;
            int n = mesh.VertexCount;

            NativeArray<float2> uv;
            try
            {
                uv = TutteEmbedding.Compute(mesh, Allocator.Persistent);
            }
            catch (ArgumentException ex)
            {
                Debug.LogWarning($"[UnfoldMode] {_wb.Catalog.NameAt(_wb.SelectedIndex)} can't be unfolded: {ex.Message}");
                return; // leave the mesh shown as-is (no morph)
            }

            _surface = new NativeArray<float3>(n, Allocator.Persistent);
            _flat = new NativeArray<float3>(n, Allocator.Persistent);
            _morph = new NativeArray<float3>(n, Allocator.Persistent);
            for (int i = 0; i < n; i++) _surface[i] = mesh.Positions[i];

            // Procrustes-align the flat UV to the surface's XZ footprint (rotation + uniform scale +
            // translation) so the unfold is a clean flatten-in-place, not a global spin/grow.
            float2 meanP = float2.zero, meanQ = float2.zero;
            for (int i = 0; i < n; i++) { meanP += uv[i]; meanQ += new float2(_surface[i].x, _surface[i].z); }
            meanP /= n; meanQ /= n;
            float a = 0f, b = 0f, den = 0f;
            for (int i = 0; i < n; i++)
            {
                float2 p = uv[i] - meanP;
                float2 q = new float2(_surface[i].x, _surface[i].z) - meanQ;
                a += p.x * q.x + p.y * q.y;
                b += p.x * q.y - p.y * q.x;
                den += p.x * p.x + p.y * p.y;
            }
            float m00 = a / den, m01 = -b / den, m10 = b / den, m11 = a / den;

            var colors = new NativeArray<Color>(n, Allocator.Temp);
            for (int i = 0; i < n; i++)
            {
                float2 p = uv[i] - meanP;
                _flat[i] = new float3(m00 * p.x + m01 * p.y + meanQ.x, 0f, m10 * p.x + m11 * p.y + meanQ.y);
                colors[i] = Checker(uv[i]);
            }
            _wb.View.SetColors(colors);
            colors.Dispose();
            uv.Dispose();

            _ready = true;
            Apply(0f);
        }

        void Update()
        {
            if (!_ready) return;
            float t = morphing ? 0.5f - 0.5f * math.cos(Time.time / morphPeriod * 2f * math.PI) : 1f;
            Apply(t);
        }

        void Apply(float t)
        {
            for (int i = 0; i < _morph.Length; i++) _morph[i] = math.lerp(_surface[i], _flat[i], t);
            _wb.View.UpdatePositions(_morph);
        }

        Color Checker(float2 uv)
        {
            int cx = (int)math.floor((uv.x * 0.5f + 0.5f) * checkerFrequency);
            int cy = (int)math.floor((uv.y * 0.5f + 0.5f) * checkerFrequency);
            return ((cx + cy) & 1) == 0 ? checkerA : checkerB;
        }

        void Free()
        {
            _ready = false;
            if (_surface.IsCreated) _surface.Dispose();
            if (_flat.IsCreated) _flat.Dispose();
            if (_morph.IsCreated) _morph.Dispose();
        }
    }
}
