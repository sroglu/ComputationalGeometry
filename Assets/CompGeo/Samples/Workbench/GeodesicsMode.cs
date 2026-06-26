using System;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.EventSystems;
using CompGeo.Core;
using CompGeo.MeshProcessing.Geodesics;

namespace CompGeo.Samples
{
    /// <summary>
    /// HW1 geodesics mode (the new-core form of the original <c>UI.cs</c> + <c>ShotestPathSearch</c>):
    /// <b>left-click</b> a vertex to set the source and recolour the surface by its Dijkstra distance field;
    /// <b>right-click</b> to set the target and draw the shortest path with the selected algorithm
    /// (Dijkstra-reconstructed or A*). Picking goes through the workbench <see cref="MeshPicker"/>
    /// (KD-tree, no colliders); a small mouse-movement threshold separates a click-to-pick from a
    /// camera drag.
    /// </summary>
    [RequireComponent(typeof(Workbench))]
    public sealed class GeodesicsMode : MonoBehaviour
    {
        public enum Algorithm { Dijkstra, AStar }

        public Algorithm algorithm = Algorithm.Dijkstra;
        public float pickRadius = 0.12f;
        public Color unreachableColor = new Color(0.15f, 0.15f, 0.15f, 1f);
        public Color pathColor = new Color(1f, 0f, 1f, 1f); // magenta — stands out over the heatmap
        [Tooltip("Path tube radius (fraction of mesh size). Min ≈ thin line, max ≈ thick tube.")]
        [Range(0.0008f, 0.012f)] public float pathWidth = 0.005f;
        [Tooltip("Max pointer travel (pixels) still treated as a click rather than a camera drag.")]
        public float clickThreshold = 6f;

        /// <summary>Geodesic cost of the current source→target pair, or -1 when no target is set.</summary>
        public float LastCost { get; private set; } = -1f;

        /// <summary>Elapsed time (ms) of the last geodesic compute, for the UI's stopwatch readout.</summary>
        public double LastSearchMs { get; private set; }
        public int Source => _source;
        public int Target => _target;

        /// <summary>Raised whenever the source/target/distance changes (the UI reads it off this).</summary>
        public event Action PathUpdated;

        Workbench _wb;
        NativeArray<float> _dist;
        NativeArray<int> _pred;
        bool _alloc;
        int _source;
        int _target = -1;
        Vector3 _downPos;
        int _downButton = -1;
        int _armed; // 0 = none, 1 = next click sets source, 2 = next click sets target (the "Pick" buttons)

        void Awake() => _wb = GetComponent<Workbench>();
        void OnEnable() => _wb.MeshChanged += OnMeshChanged;
        void OnDisable() { _wb.MeshChanged -= OnMeshChanged; Free(); }

        void OnMeshChanged()
        {
            Free();
            int n = _wb.Mesh.VertexCount;
            _dist = new NativeArray<float>(n, Allocator.Persistent);
            _pred = new NativeArray<int>(n, Allocator.Persistent);
            _alloc = true;
            _wb.View.PathRadiusScale = pathWidth;
            _source = 0;
            _target = -1;
            Recompute();
        }

        /// <summary>Bound to the UI path-width slider; rebuilds the current path at the new thickness.</summary>
        public void SetPathWidth(float width)
        {
            pathWidth = width;
            if (_alloc) { _wb.View.PathRadiusScale = width; RefreshPath(); }
        }

        /// <summary>Bound to the UI algorithm dropdown (0 = Dijkstra, 1 = A*).</summary>
        public void SetAlgorithm(int index)
        {
            algorithm = (Algorithm)Mathf.Clamp(index, 0, 1);
            RefreshPath();
        }

        /// <summary>Set the source vertex and recompute its distance field + path (bound to the Source field).</summary>
        public void SetSource(int v)
        {
            if (!_alloc) return;
            _source = Mathf.Clamp(v, 0, _wb.Mesh.VertexCount - 1);
            Recompute();
            RefreshPath();
        }

        /// <summary>Set the target vertex and redraw the path (bound to the Dest field).</summary>
        public void SetTarget(int v)
        {
            if (!_alloc) return;
            _target = Mathf.Clamp(v, 0, _wb.Mesh.VertexCount - 1);
            RefreshPath();
        }

        /// <summary>Arm the next mesh click to set the source vertex (the "Pick" button next to Source).</summary>
        public void ArmPickSource() => _armed = 1;

        /// <summary>Arm the next mesh click to set the target vertex (the "Pick" button next to Dest).</summary>
        public void ArmPickDest() => _armed = 2;

        /// <summary>Recompute the path for the current source/target (the "Search" button).</summary>
        public void Search()
        {
            if (!_alloc) return;
            Recompute();
            RefreshPath();
        }

        /// <summary>Pick a random source/target pair and search (the "Find Random Points" path).</summary>
        public void SearchRandom()
        {
            if (!_alloc) return;
            int n = _wb.Mesh.VertexCount;
            _source = UnityEngine.Random.Range(0, n);
            _target = UnityEngine.Random.Range(0, n);
            Recompute();
            RefreshPath();
        }

        void Update()
        {
            if (!_alloc) return;

            // Track press/release to tell a click (pick) apart from a drag (camera).
            for (int b = 0; b <= 1; b++)
            {
                if (Input.GetMouseButtonDown(b)) { _downPos = Input.mousePosition; _downButton = b; }
            }
            if (_downButton >= 0 && Input.GetMouseButtonUp(_downButton))
            {
                int button = _downButton;
                _downButton = -1;
                bool overUi = EventSystem.current != null && EventSystem.current.IsPointerOverGameObject();
                bool isClick = (Input.mousePosition - _downPos).sqrMagnitude <= clickThreshold * clickThreshold;
                if (isClick && !overUi && TryPick(out int v))
                {
                    if (_armed == 1) { _armed = 0; SetSource(v); }
                    else if (_armed == 2) { _armed = 0; SetTarget(v); }
                    else if (button == 0) SetSource(v);
                    else SetTarget(v);
                }
            }
        }

        bool TryPick(out int vertex)
        {
            vertex = -1;
            Camera cam = Camera.main;
            if (cam == null) return false;

            // Screen ray into the mesh's local space (the view draws at the workbench transform).
            Ray world = cam.ScreenPointToRay(Input.mousePosition);
            Vector3 lo = transform.InverseTransformPoint(world.origin);
            Vector3 ld = transform.InverseTransformDirection(world.direction);
            return _wb.Picker.PickSurface(new Ray(lo, ld), out vertex);
        }

        void Recompute()
        {
            var sw = System.Diagnostics.Stopwatch.StartNew();
            DijkstraGeodesics.Compute(_wb.Mesh, _source, _dist, _pred);
            sw.Stop();
            LastSearchMs = sw.Elapsed.TotalMilliseconds;
            _wb.View.SetHeatmap(_dist, unreachableColor);
        }

        void RefreshPath()
        {
            if (_target < 0)
            {
                LastCost = -1f;
                PathUpdated?.Invoke();
                return;
            }

            NativeList<int> path;
            float cost;
            if (algorithm == Algorithm.AStar)
                cost = AStarGeodesics.FindPath(_wb.Mesh, _source, _target, Allocator.Temp, out path);
            else
            {
                path = DijkstraGeodesics.ReconstructPath(_pred, _source, _target, Allocator.Temp);
                cost = _dist[_target];
            }

            if (path.Length > 0) _wb.View.SetPath(_wb.Mesh, path.AsArray(), pathColor);
            path.Dispose();

            LastCost = cost;
            PathUpdated?.Invoke();
        }

        void Free()
        {
            if (!_alloc) return;
            if (_dist.IsCreated) _dist.Dispose();
            if (_pred.IsCreated) _pred.Dispose();
            _alloc = false;
        }
    }
}
