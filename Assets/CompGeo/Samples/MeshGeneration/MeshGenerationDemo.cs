using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using CompGeo.Core;
using CompGeo.MeshProcessing;
using CompGeo.Visualization;

namespace CompGeo.Samples
{
    /// <summary>
    /// Demo for the ported "Mesh Generation" homework: load a mesh as a raw <b>point cloud</b>, then
    /// rebuild its surface with <see cref="PointCloudRemesh"/> (KD-tree k-NN grouping + covariance-row
    /// local planes + ear-clipping), or flatten it with <see cref="PcaUnfold"/> (global covariance plane).
    /// Both reproduce the original method exactly; this just drives them and renders the result through
    /// <see cref="MeshGpuView"/>. Buttons are drawn with IMGUI so the scene needs no canvas wiring.
    /// </summary>
    public sealed class MeshGenerationDemo : MonoBehaviour
    {
        public MeshCatalog catalog = new MeshCatalog();
        [Tooltip("Catalog index to load on Start (3 = man0).")]
        public int selectedMeshIndex = 3;
        [Tooltip("Neighbourhood size for the remesh (original homework used 8).")]
        [Range(3, 32)] public int k = PointCloudRemesh.DefaultK;

        [Tooltip("Local-plane method: the homework's covariance rows, or true eigenvectors (PCA).")]
        public PlaneMethod method = PlaneMethod.CovarianceRows;

        [Tooltip("Use the Burst-parallel remesh (identical result, ~3x faster on large meshes).")]
        public bool parallel = true;

        enum Mode { Points, Remesh, Unfold }
        Mode _mode = Mode.Points;

        MeshData _source;
        MeshData _generated;
        bool _hasSource, _hasGenerated, _built;
        MeshGpuView _view;
        string _status = "";
        static readonly string[] MethodLabels = { "Homework (cov-rows)", "Eigenvectors (PCA)" };

        void Start()
        {
            _view = new MeshGpuView();
            LoadMesh();
        }

        void LoadMesh()
        {
            FreeGenerated();
            FreeSource();
            selectedMeshIndex = Mathf.Clamp(selectedMeshIndex, 0, catalog.Count - 1);
            _source = catalog.Build(selectedMeshIndex, Allocator.Persistent);
            _hasSource = true;
            ShowPointCloud();
        }

        void ShowPointCloud()
        {
            _view.Build(_source);
            _view.ShowPoints = true;
            _view.ShowEdges = false;
            _view.ShowSurface = false;
            _built = true;
            _mode = Mode.Points;
            _status = $"{catalog.NameAt(selectedMeshIndex)}: {_source.VertexCount} points";
        }

        void Remesh()
        {
            if (!_hasSource) return;
            FreeGenerated();
            float t0 = Time.realtimeSinceStartup;
            _generated = parallel
                ? PointCloudRemesh.RemeshParallel(_source.Positions, k, method, Allocator.Persistent)
                : PointCloudRemesh.Remesh(_source.Positions, k, method, Allocator.Persistent);
            float ms = (Time.realtimeSinceStartup - t0) * 1000f;
            _hasGenerated = true;
            _view.Build(_generated);
            _view.ShowPoints = false;
            _view.ShowEdges = true;
            _view.ShowSurface = true;
            _built = true;
            _mode = Mode.Remesh;
            _status = $"remesh (k={k}, {MethodLabels[(int)method]}, {(parallel ? "parallel" : "serial")}): {_generated.TriangleCount} tris, {ms:F0} ms";
        }

        void PcaUnfoldMesh()
        {
            if (!_hasSource) return;
            FreeGenerated();

            using var uv = new NativeArray<float2>(_source.VertexCount, Allocator.Persistent);
            PcaUnfold.Compute(_source.Positions, uv, method);

            var flat = new List<float3>(_source.VertexCount);
            for (int i = 0; i < uv.Length; i++) flat.Add(new float3(uv[i].x, 0f, uv[i].y));
            var tris = new List<int3>(_source.TriangleCount);
            for (int t = 0; t < _source.TriangleCount; t++) tris.Add(_source.Triangles[t]);

            _generated = MeshBuilder.Build(flat, tris, Allocator.Persistent);
            _hasGenerated = true;
            _view.Build(_generated);
            _view.ShowPoints = false;
            _view.ShowEdges = true;
            _view.ShowSurface = true;
            _built = true;
            _mode = Mode.Unfold;
            _status = $"PCA unfold ({MethodLabels[(int)method]})";
        }

        // Re-run whichever result is showing, after the method dropdown changes.
        void Reapply()
        {
            switch (_mode)
            {
                case Mode.Remesh: Remesh(); break;
                case Mode.Unfold: PcaUnfoldMesh(); break;
                default: ShowPointCloud(); break;
            }
        }

        void LateUpdate()
        {
            if (_built && _view != null) _view.DrawNow(transform.localToWorldMatrix);
        }

        void OnGUI()
        {
            const int w = 210;
            GUILayout.BeginArea(new Rect(8, 8, w, 320), GUI.skin.box);
            GUILayout.Label("<b>Mesh Generation</b> (eigenvector / covariance-row port)");
            GUILayout.Label(_status);

            GUILayout.Label("Plane method:");
            int sel = GUILayout.SelectionGrid((int)method, MethodLabels, 1);
            if (sel != (int)method) { method = (PlaneMethod)sel; Reapply(); }

            bool par = GUILayout.Toggle(parallel, " Parallel (Burst)");
            if (par != parallel) parallel = par;

            GUILayout.Space(4);
            if (GUILayout.Button("Point cloud")) ShowPointCloud();
            if (GUILayout.Button($"Remesh (k = {k})")) Remesh();
            if (GUILayout.Button("PCA Unfold")) PcaUnfoldMesh();

            GUILayout.Space(6);
            GUILayout.Label($"k = {k}");
            k = Mathf.RoundToInt(GUILayout.HorizontalSlider(k, 3, 32));

            GUILayout.Space(6);
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("◀ mesh")) { selectedMeshIndex--; LoadMesh(); }
            if (GUILayout.Button("mesh ▶")) { selectedMeshIndex++; LoadMesh(); }
            GUILayout.EndHorizontal();
            GUILayout.EndArea();
        }

        void FreeSource() { if (_hasSource) { _source.Dispose(); _hasSource = false; } }
        void FreeGenerated() { if (_hasGenerated) { _generated.Dispose(); _hasGenerated = false; } }

        void OnDestroy()
        {
            FreeGenerated();
            FreeSource();
            _view?.Dispose();
        }
    }
}
