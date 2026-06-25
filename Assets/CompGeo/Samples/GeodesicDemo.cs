using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using CompGeo.Core;
using CompGeo.MeshProcessing.Geodesics;
using CompGeo.Visualization;

namespace CompGeo.Samples
{
    /// <summary>
    /// End-to-end Step 1–3 demo: builds a procedural grid mesh, renders it on the GPU
    /// (<see cref="MeshGpuView"/>), and wires interaction to the geodesics —
    /// <b>left-click</b> a vertex to set the source and recolour by its Dijkstra distance field;
    /// <b>right-click</b> to set a target and highlight the A* shortest path to it.
    /// Picking goes through the KD-tree <see cref="MeshPicker"/> (no colliders).
    ///
    /// Drop this component on a GameObject at the origin; ensure a tagged MainCamera looks at it.
    /// Requires a URP pipeline asset assigned in Project Settings → Graphics (see README).
    /// </summary>
    public sealed class GeodesicDemo : MonoBehaviour
    {
        [Header("Procedural grid")]
        [Min(2)] public int gridSize = 24;
        public float spacing = 0.1f;
        public float heightAmplitude = 0.15f;

        [Header("Picking / colours")]
        public float pickRadius = 0.12f;
        public Color unreachableColor = new Color(0.15f, 0.15f, 0.15f, 1f);
        public Color pathColor = Color.white;

        MeshData _mesh;
        MeshGpuView _view;
        MeshPicker _picker;
        NativeArray<float> _dist;
        NativeArray<int> _pred;
        int _source;
        int _target = -1;

        void Start()
        {
            _mesh = BuildGrid(gridSize, spacing, heightAmplitude);
            _view = new MeshGpuView();
            _view.Build(_mesh);
            _picker = new MeshPicker(_mesh);

            _dist = new NativeArray<float>(_mesh.VertexCount, Allocator.Persistent);
            _pred = new NativeArray<int>(_mesh.VertexCount, Allocator.Persistent);

            _source = 0;
            RecomputeField();
        }

        void Update()
        {
            if (Input.GetMouseButtonDown(0) && TryPick(out int v))
            {
                _source = v;
                RecomputeField();
                RefreshPath();
            }
            else if (Input.GetMouseButtonDown(1) && TryPick(out int t))
            {
                _target = t;
                RefreshPath();
            }

            _view.DrawNow(transform.localToWorldMatrix);
        }

        bool TryPick(out int vertex)
        {
            vertex = -1;
            Camera cam = Camera.main;
            if (cam == null) return false;

            // Convert the screen ray into the mesh's local space (the view draws with this transform).
            Ray world = cam.ScreenPointToRay(Input.mousePosition);
            Vector3 lo = transform.InverseTransformPoint(world.origin);
            Vector3 ld = transform.InverseTransformDirection(world.direction);
            return _picker.Pick(new Ray(lo, ld), pickRadius, out vertex);
        }

        void RecomputeField()
        {
            DijkstraGeodesics.Compute(_mesh, _source, _dist, _pred);
            _view.SetHeatmap(_dist, unreachableColor);
        }

        void RefreshPath()
        {
            if (_target < 0) return;
            float cost = AStarGeodesics.FindPath(_mesh, _source, _target, Allocator.Temp, out var path);
            if (path.Length > 0) _view.SetPath(_mesh, path.AsArray(), pathColor);
            path.Dispose();
        }

        void OnDestroy()
        {
            _view?.Dispose();
            _picker?.Dispose();
            if (_dist.IsCreated) _dist.Dispose();
            if (_pred.IsCreated) _pred.Dispose();
            _mesh.Dispose();
        }

        static MeshData BuildGrid(int m, float spacing, float amp)
        {
            var positions = new List<float3>(m * m);
            for (int j = 0; j < m; j++)
            for (int i = 0; i < m; i++)
            {
                float y = amp * math.sin(i * 0.6f) * math.cos(j * 0.6f);
                positions.Add(new float3(i * spacing, y, j * spacing));
            }

            var triangles = new List<int3>((m - 1) * (m - 1) * 2);
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
