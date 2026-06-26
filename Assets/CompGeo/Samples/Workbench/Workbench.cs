using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using CompGeo.Core;
using CompGeo.MeshProcessing;
using CompGeo.MeshProcessing.Parameterization;
using CompGeo.Visualization;

namespace CompGeo.Samples
{
    /// <summary>The two unfold methods: the homework's dense solve, or the performant sparse-CG Tutte
    /// (same round result, far faster). Both pin the boundary to a circle and solve the uniform Laplacian.</summary>
    public enum UnfoldMethod { Original, Tutte }

    /// <summary>
    /// Hub for the interactive samples (the new-core stand-in for the original <c>ModelManager</c>): owns
    /// the active <see cref="MeshData"/> plus its <see cref="MeshGpuView"/> and <see cref="MeshPicker"/>,
    /// loads any mesh from the inspector-editable <see cref="MeshCatalog"/>, and exposes the vertex/edge/
    /// surface display toggles. Modes (geodesics / unfold) attach alongside and drive the shared view.
    /// The view is drawn in <see cref="LateUpdate"/> so a mode's <c>Update</c> can recolour or reshape it
    /// first.
    /// </summary>
    public sealed class Workbench : MonoBehaviour
    {
        public MeshCatalog catalog = new MeshCatalog();
        [SerializeField] int selectedMeshIndex = 0;

        [Header("Display")]
        public bool showVertices = false;
        public bool showEdges = false;
        public bool showSurface = true;
        public bool showNormals = false;

        [Header("Mesh generation")]
        [Range(3, 32)] public int remeshK = 8;
        [Tooltip("Reconstruction: the homework's patch-union soup, or the improved mutual-agreement mesh.")]
        public RemeshMode remeshMode = RemeshMode.Original;

        [Header("Unfold")]
        [Tooltip("Original (dense homework solve) or Tutte (performant sparse CG). Same round result.")]
        public UnfoldMethod unfoldMethod = UnfoldMethod.Original;

        public MeshData Mesh => _mesh;
        public MeshGpuView View => _view;
        public MeshPicker Picker => _picker;
        public int SelectedIndex => selectedMeshIndex;
        public MeshCatalog Catalog => catalog;

        /// <summary>Raised after a new mesh is loaded (modes reset their per-mesh state here).</summary>
        public event Action MeshChanged;

        MeshData _mesh;
        MeshGpuView _view;
        MeshPicker _picker;
        bool _loaded;

        void Start() => LoadMesh(selectedMeshIndex);

        /// <summary>Load and display the catalog mesh at <paramref name="index"/>; rebuilds view + picker.</summary>
        public void LoadMesh(int index)
        {
            index = Mathf.Clamp(index, 0, catalog.Count - 1);
            DisposeCurrent();

            selectedMeshIndex = index;
            _mesh = catalog.Build(index, Allocator.Persistent);
            BuildViewAndPicker();

            MeshChanged?.Invoke();
        }

        /// <summary>
        /// Regenerate the surface from the current point cloud (the original homework's "Remesh" — KD-tree
        /// k-NN grouping + covariance-row local planes + ear-clipping) and rebuild the view/picker.
        /// </summary>
        public void Remesh()
        {
            if (!_loaded) return;
            MeshData newMesh = PointCloudRemesh.Remesh(_mesh.Positions, remeshK, remeshMode, Allocator.Persistent);
            _view.Dispose();
            _picker.Dispose();
            _mesh.Dispose();
            _mesh = newMesh;
            BuildViewAndPicker();
            MeshChanged?.Invoke();
        }

        /// <summary>
        /// Flatten the current mesh to its 2D parameterization and display it: <see cref="UnfoldMethod.Tutte"/>
        /// (boundary pinned to a circle, interior solved via the Laplacian) or <see cref="UnfoldMethod.Pca"/>
        /// (project onto the eigenvector plane). Tutte needs disk topology — on a closed mesh it does nothing.
        /// </summary>
        public void Unfold()
        {
            if (!_loaded) return;
            int n = _mesh.VertexCount;
            var uv = new NativeArray<float2>(n, Allocator.Persistent);
            try
            {
                if (unfoldMethod == UnfoldMethod.Tutte)
                {
                    using var t = TutteEmbedding.Compute(_mesh, Allocator.Persistent);
                    uv.CopyFrom(t);
                }
                else
                {
                    using var t = OriginalUnfold.Compute(_mesh, Allocator.Persistent); // dense homework solve
                    uv.CopyFrom(t);
                }
            }
            catch (ArgumentException e)
            {
                Debug.LogWarning($"Unfold skipped: {e.Message}"); // Tutte on a closed mesh has no boundary
                uv.Dispose();
                return;
            }

            // Centre + scale the flat result to ~2 units so the camera frames it like the 3D mesh.
            float2 mn = uv[0], mx = uv[0];
            for (int i = 1; i < n; i++) { mn = math.min(mn, uv[i]); mx = math.max(mx, uv[i]); }
            float2 c = (mn + mx) * 0.5f;
            float ext = math.cmax(mx - mn);
            float s = ext > 1e-9f ? 2f / ext : 1f;

            var flat = new List<float3>(n);
            for (int i = 0; i < n; i++) { float2 p = (uv[i] - c) * s; flat.Add(new float3(p.x, 0f, p.y)); }
            var tris = new List<int3>(_mesh.TriangleCount);
            for (int t = 0; t < _mesh.TriangleCount; t++) tris.Add(_mesh.Triangles[t]);
            uv.Dispose();

            MeshData newMesh = MeshBuilder.Build(flat, tris, Allocator.Persistent);
            _view.Dispose();
            _picker.Dispose();
            _mesh.Dispose();
            _mesh = newMesh;
            BuildViewAndPicker();
            MeshChanged?.Invoke();
        }

        void BuildViewAndPicker()
        {
            _view = new MeshGpuView
            {
                ShowPoints = showVertices,
                ShowEdges = showEdges,
                ShowSurface = showSurface,
                ShowNormals = showNormals,
            };
            _view.Build(_mesh);
            _picker = new MeshPicker(_mesh);
            _loaded = true;
        }

        public void SetShowVertices(bool v) { showVertices = v; if (_loaded) _view.ShowPoints = v; }
        public void SetShowEdges(bool v) { showEdges = v; if (_loaded) _view.ShowEdges = v; }
        public void SetShowSurface(bool v) { showSurface = v; if (_loaded) _view.ShowSurface = v; }
        public void SetShowNormals(bool v) { showNormals = v; if (_loaded) _view.ShowNormals = v; }

        void LateUpdate()
        {
            if (_loaded) _view.DrawNow(transform.localToWorldMatrix);
        }

        void DisposeCurrent()
        {
            if (!_loaded) return;
            _view.Dispose();
            _picker.Dispose();
            _mesh.Dispose();
            _loaded = false;
        }

        void OnDestroy() => DisposeCurrent();
    }
}
