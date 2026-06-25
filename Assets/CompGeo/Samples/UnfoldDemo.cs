using System.Collections.Generic;
using System.IO;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using CompGeo.Core;
using CompGeo.Core.IO;
using CompGeo.MeshProcessing.Parameterization;
using CompGeo.Visualization;

namespace CompGeo.Samples
{
    /// <summary>
    /// Interactive Step 4–5 demo: a non-planar disk-topology surface and its flat
    /// <see cref="TutteEmbedding"/> parameterization. The surface <b>continuously unfolds</b> onto the unit
    /// disk and folds back on its own; <b>hold the left mouse button and drag to rotate</b> the view and
    /// inspect it from any angle. Vertices are checkerboard-coloured by their flat UV, so the parameter grid
    /// stays attached as the surface flattens (and the flat state is visibly fold-free, Tutte's guarantee).
    ///
    /// Drop on a GameObject at the origin; a tagged MainCamera looking at it. Requires the URP pipeline
    /// asset assigned (see README) so <c>CompGeo/VertexColorUnlit</c> renders.
    ///
    /// <para>Input note: this uses the legacy <c>UnityEngine.Input</c> for now (the project currently ships
    /// with Active Input Handling = Input Manager). Migrating the samples to the new Input System is a
    /// separate, pending step.</para>
    /// </summary>
    public sealed class UnfoldDemo : MonoBehaviour
    {
        [Header("Source mesh")]
        [Tooltip("Absolute path to a disk-topology (open / single-boundary) .off mesh. " +
                 "If empty or missing, the procedural grid below is used instead.")]
        public string meshOffPath = "";

        [Header("Procedural disk surface (fallback)")]
        [Min(2)] public int gridSize = 48;
        public float spacing = 0.045f;
        public float heightAmplitude = 0.25f;

        [Header("Checkerboard")]
        // Keep well below gridSize (≈ 4 vertices per checker cell) so the per-vertex checker is
        // properly sampled — a frequency near the vertex resolution aliases into coarse blobs.
        [Min(1)] public int checkerFrequency = 12;
        public Color checkerA = new Color(0.95f, 0.95f, 0.95f, 1f);
        public Color checkerB = new Color(0.15f, 0.35f, 0.85f, 1f);

        [Header("Unfold morph (automatic)")]
        [Min(0.1f)] public float morphPeriod = 4f;

        [Header("Inspect")]
        [Tooltip("Hold the left mouse button and drag to rotate the view.")]
        [Min(0f)] public float rotateSpeed = 5f;

        MeshData _mesh;
        MeshGpuView _view;
        NativeArray<float3> _surface; // original 3D positions
        NativeArray<float3> _flat;    // flat UV positions (y = 0)
        NativeArray<float3> _morph;   // current interpolation

        void Start()
        {
            _mesh = LoadOrBuildMesh();

            var uv = TutteEmbedding.Compute(_mesh, Allocator.Persistent);

            int n = _mesh.VertexCount;
            _surface = new NativeArray<float3>(n, Allocator.Persistent);
            _flat = new NativeArray<float3>(n, Allocator.Persistent);
            _morph = new NativeArray<float3>(n, Allocator.Persistent);
            for (int i = 0; i < n; i++) _surface[i] = _mesh.Positions[i];

            // Procrustes-align the flat UV to the surface's XZ footprint (best rotation + uniform scale +
            // translation). This strips the global spin/size difference between the Tutte disk and the
            // grid, so the morph reads as a clean flatten-in-place — only the genuine parameter
            // distortion remains, no distracting whole-object rotation or growing/shrinking.
            float2 meanP = float2.zero, meanQ = float2.zero;
            for (int i = 0; i < n; i++)
            {
                meanP += uv[i];
                meanQ += new float2(_surface[i].x, _surface[i].z);
            }
            meanP /= n; meanQ /= n;

            float a = 0f, b = 0f, den = 0f;
            for (int i = 0; i < n; i++)
            {
                float2 p = uv[i] - meanP;
                float2 q = new float2(_surface[i].x, _surface[i].z) - meanQ;
                a += p.x * q.x + p.y * q.y; // Σ p·q
                b += p.x * q.y - p.y * q.x; // Σ p×q
                den += p.x * p.x + p.y * p.y;
            }
            // Scaled-rotation matrix M = (1/den) * [[a, -b], [b, a]] (2D Procrustes similarity).
            float m00 = a / den, m01 = -b / den, m10 = b / den, m11 = a / den;
            for (int i = 0; i < n; i++)
            {
                float2 p = uv[i] - meanP;
                float fx = m00 * p.x + m01 * p.y + meanQ.x;
                float fy = m10 * p.x + m11 * p.y + meanQ.y;
                _flat[i] = new float3(fx, 0f, fy);
            }

            var colors = new NativeArray<Color>(n, Allocator.Temp);
            for (int i = 0; i < n; i++) colors[i] = Checker(uv[i]);

            _view = new MeshGpuView { ShowSurface = true, ShowPoints = false, ShowEdges = false };
            _view.Build(_mesh);
            _view.SetColors(colors);

            colors.Dispose();
            uv.Dispose();

            ApplyUnfold(0f);
        }

        void Update()
        {
            // Automatic unfold: smooth ping-pong in [0,1] (0 = 3D surface, 1 = flat disk).
            float t = 0.5f - 0.5f * math.cos(Time.time / morphPeriod * 2f * math.PI);
            ApplyUnfold(t);

            // Hold left mouse + drag to rotate the view for inspection (legacy Input Manager axes).
            if (Input.GetMouseButton(0))
            {
                transform.Rotate(Vector3.up, -Input.GetAxis("Mouse X") * rotateSpeed, Space.World);
                transform.Rotate(Vector3.right, Input.GetAxis("Mouse Y") * rotateSpeed, Space.World);
            }

            _view.DrawNow(transform.localToWorldMatrix);
        }

        void ApplyUnfold(float t)
        {
            for (int i = 0; i < _morph.Length; i++)
                _morph[i] = math.lerp(_surface[i], _flat[i], t);
            _view.UpdatePositions(_morph);
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
            if (_surface.IsCreated) _surface.Dispose();
            if (_flat.IsCreated) _flat.Dispose();
            if (_morph.IsCreated) _morph.Dispose();
            _mesh.Dispose();
        }

        MeshData LoadOrBuildMesh()
        {
            if (!string.IsNullOrEmpty(meshOffPath) && File.Exists(meshOffPath))
            {
                var loaded = OffReader.ReadFile(meshOffPath, Allocator.Persistent);
                NormalizeAndOrient(ref loaded);
                return loaded;
            }

            BuildGridData(gridSize, spacing, heightAmplitude, out var positions, out var triangles);
            return MeshBuilder.Build(positions, triangles, Allocator.Persistent);
        }

        /// <summary>
        /// Lay an arbitrary loaded mesh down for the demo: rotate its dominant (area-weighted) face normal
        /// to +Y so relief runs vertically (like the procedural grid), then centre it on the origin and
        /// uniformly scale it to fit ~2 units. Topology/indices are untouched.
        /// </summary>
        static void NormalizeAndOrient(ref MeshData m)
        {
            int n = m.VertexCount;

            float3 nrm = float3.zero;
            for (int t = 0; t < m.TriangleCount; t++)
            {
                int3 tri = m.Triangles[t];
                nrm += math.cross(m.Positions[tri.y] - m.Positions[tri.x], m.Positions[tri.z] - m.Positions[tri.x]);
            }
            if (math.length(nrm) > 1e-9f)
            {
                Quaternion q = Quaternion.FromToRotation((Vector3)math.normalize(nrm), Vector3.up);
                for (int i = 0; i < n; i++) m.Positions[i] = (float3)(q * (Vector3)(float3)m.Positions[i]);
            }

            float3 mn = m.Positions[0], mx = m.Positions[0];
            for (int i = 1; i < n; i++) { mn = math.min(mn, m.Positions[i]); mx = math.max(mx, m.Positions[i]); }
            float3 centre = (mn + mx) * 0.5f;
            float maxExtent = math.cmax(mx - mn);
            float scale = maxExtent > 1e-9f ? 2f / maxExtent : 1f;
            for (int i = 0; i < n; i++) m.Positions[i] = (m.Positions[i] - centre) * scale;
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
