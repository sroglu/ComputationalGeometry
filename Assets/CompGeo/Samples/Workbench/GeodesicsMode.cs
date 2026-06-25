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
        public Color pathColor = Color.white;
        [Tooltip("Max pointer travel (pixels) still treated as a click rather than a camera drag.")]
        public float clickThreshold = 6f;

        Workbench _wb;
        NativeArray<float> _dist;
        NativeArray<int> _pred;
        bool _alloc;
        int _source;
        int _target = -1;
        Vector3 _downPos;
        int _downButton = -1;

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
            _source = 0;
            _target = -1;
            Recompute();
        }

        /// <summary>Bound to the UI algorithm dropdown (0 = Dijkstra, 1 = A*).</summary>
        public void SetAlgorithm(int index)
        {
            algorithm = (Algorithm)Mathf.Clamp(index, 0, 1);
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
                    if (button == 0) { _source = v; Recompute(); RefreshPath(); }
                    else { _target = v; RefreshPath(); }
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
            return _wb.Picker.Pick(new Ray(lo, ld), pickRadius, out vertex);
        }

        void Recompute()
        {
            DijkstraGeodesics.Compute(_wb.Mesh, _source, _dist, _pred);
            _wb.View.SetHeatmap(_dist, unreachableColor);
        }

        void RefreshPath()
        {
            if (_target < 0) return;

            NativeList<int> path;
            if (algorithm == Algorithm.AStar)
                AStarGeodesics.FindPath(_wb.Mesh, _source, _target, Allocator.Temp, out path);
            else
                path = DijkstraGeodesics.ReconstructPath(_pred, _source, _target, Allocator.Temp);

            if (path.Length > 0) _wb.View.SetPath(_wb.Mesh, path.AsArray(), pathColor);
            path.Dispose();
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
