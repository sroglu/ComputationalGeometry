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

        MeshData _source;
        MeshData _generated;
        bool _hasSource, _hasGenerated, _built;
        MeshGpuView _view;
        string _status = "";

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
            _status = $"{catalog.NameAt(selectedMeshIndex)}: {_source.VertexCount} points";
        }

        void Remesh()
        {
            if (!_hasSource) return;
            FreeGenerated();
            _generated = PointCloudRemesh.Remesh(_source.Positions, k, Allocator.Persistent);
            _hasGenerated = true;
            _view.Build(_generated);
            _view.ShowPoints = false;
            _view.ShowEdges = true;
            _view.ShowSurface = true;
            _built = true;
            _status = $"remesh (k={k}): {_generated.TriangleCount} triangles";
        }

        void PcaUnfoldMesh()
        {
            if (!_hasSource) return;
            FreeGenerated();

            using var uv = new NativeArray<float2>(_source.VertexCount, Allocator.Persistent);
            PcaUnfold.Compute(_source.Positions, uv);

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
            _status = "PCA unfold (global covariance plane)";
        }

        void LateUpdate()
        {
            if (_built && _view != null) _view.DrawNow(transform.localToWorldMatrix);
        }

        void OnGUI()
        {
            const int w = 210;
            GUILayout.BeginArea(new Rect(8, 8, w, 230), GUI.skin.box);
            GUILayout.Label("<b>Mesh Generation</b> (eigenvector / covariance-row port)");
            GUILayout.Label(_status);

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
