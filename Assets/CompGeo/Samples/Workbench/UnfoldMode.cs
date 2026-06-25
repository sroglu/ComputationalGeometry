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
        Material _checkerMat;
        NativeArray<float3> _surface;
        NativeArray<float3> _flat;
        NativeArray<float3> _morph;
        bool _ready;

        void Awake()
        {
            _wb = GetComponent<Workbench>();
            _checkerMat = new Material(Shader.Find("CompGeo/UvCheckerUnlit"));
        }

        void OnEnable() => _wb.MeshChanged += OnMeshChanged;
        void OnDisable() { _wb.MeshChanged -= OnMeshChanged; Free(); }
        void OnDestroy() { if (_checkerMat != null) Destroy(_checkerMat); }

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

            // Tutte's boundary winding can come out mirrored relative to the surface on some meshes (e.g.
            // the low-poly face), which makes the flat map look inside-out and collapses the alignment
            // below to a tiny patch. Detect it by comparing per-triangle orientation in UV vs the surface's
            // XZ projection, and un-mirror the UV so the two windings agree.
            int agree = 0, disagree = 0;
            for (int t = 0; t < mesh.TriangleCount; t++)
            {
                int3 tr = mesh.Triangles[t];
                float aUv = Cross2(uv[tr.y] - uv[tr.x], uv[tr.z] - uv[tr.x]);
                float3 pa = mesh.Positions[tr.x], pb = mesh.Positions[tr.y], pc = mesh.Positions[tr.z];
                float aXz = Cross2(new float2(pb.x - pa.x, pb.z - pa.z), new float2(pc.x - pa.x, pc.z - pa.z));
                if (aUv * aXz >= 0f) agree++; else disagree++;
            }
            if (disagree > agree)
                for (int i = 0; i < n; i++) uv[i] = new float2(-uv[i].x, uv[i].y);

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

            for (int i = 0; i < n; i++)
            {
                float2 p = uv[i] - meanP;
                _flat[i] = new float3(m00 * p.x + m01 * p.y + meanQ.x, 0f, m10 * p.x + m11 * p.y + meanQ.y);
            }

            // Paint the checker per-pixel from the UV in the shader (crisp, tessellation-independent),
            // instead of per-vertex colours which blur/alias on coarse or irregular meshes.
            _checkerMat.SetColor("_ColorA", checkerA);
            _checkerMat.SetColor("_ColorB", checkerB);
            _checkerMat.SetFloat("_Frequency", checkerFrequency);
            _wb.View.SetSurfaceUVs(uv);
            _wb.View.SetSurfaceMaterial(_checkerMat);
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

        public void Apply(float t)
        {
            for (int i = 0; i < _morph.Length; i++) _morph[i] = math.lerp(_surface[i], _flat[i], t);
            _wb.View.UpdatePositions(_morph);
        }

        static float Cross2(float2 a, float2 b) => a.x * b.y - a.y * b.x;

        void Free()
        {
            _ready = false;
            if (_surface.IsCreated) _surface.Dispose();
            if (_flat.IsCreated) _flat.Dispose();
            if (_morph.IsCreated) _morph.Dispose();
        }
    }
}
