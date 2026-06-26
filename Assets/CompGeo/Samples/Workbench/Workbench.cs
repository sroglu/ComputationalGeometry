using System;
using Unity.Collections;
using UnityEngine;
using CompGeo.Core;
using CompGeo.MeshProcessing;
using CompGeo.Visualization;

namespace CompGeo.Samples
{
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
        [Tooltip("Local-plane method for Remesh: the homework's covariance rows, or true eigenvectors (PCA).")]
        public PlaneMethod remeshMethod = PlaneMethod.CovarianceRows;

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
            MeshData newMesh = PointCloudRemesh.RemeshParallel(_mesh.Positions, remeshK, remeshMethod, Allocator.Persistent);
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
