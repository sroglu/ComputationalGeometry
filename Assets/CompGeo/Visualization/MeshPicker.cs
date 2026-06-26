using System;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using CompGeo.Core;
using CompGeo.Collections;

namespace CompGeo.Visualization
{
    /// <summary>
    /// Vertex picking backed by a <see cref="KdTree3"/> over the mesh positions — the replacement for
    /// the original per-vertex physics colliders + <c>Int32.TryParse(hit.transform.name)</c> approach
    /// (docs/MIGRATION.md §2.D). Pick the vertex under a screen ray, or the nearest vertex to a world
    /// point. The tree is built in world-independent local space, so transform the ray into the mesh's
    /// local space before querying if the view is not at the identity.
    /// </summary>
    public sealed class MeshPicker : IDisposable
    {
        KdTree3 _tree;
        NativeArray<float3> _positions; // borrowed from the mesh (same lifetime as the picker)
        NativeArray<int3> _triangles;

        public MeshPicker(in MeshData mesh)
        {
            _tree = KdTree3.Build(mesh.Positions, Allocator.Persistent);
            _positions = mesh.Positions;
            _triangles = mesh.Triangles;
        }

        /// <summary>
        /// Pick the vertex nearest to where <paramref name="ray"/> first hits the mesh surface
        /// (ray-triangle intersection, then nearest vertex to the hit point). Forgiving — a click
        /// anywhere on a face selects that face's nearest vertex — and only the front surface is hit,
        /// unlike the perpendicular-distance <see cref="Pick(Ray,float,out int)"/>. False if the ray
        /// misses the mesh entirely.
        /// </summary>
        public bool PickSurface(Ray ray, out int vertex)
        {
            vertex = -1;
            float3 o = ray.origin;
            float3 d = math.normalizesafe(ray.direction, new float3(0, 0, 1));

            float bestT = float.PositiveInfinity;
            for (int t = 0; t < _triangles.Length; t++)
            {
                int3 tri = _triangles[t];
                if (RayTriangle(o, d, _positions[tri.x], _positions[tri.y], _positions[tri.z], out float hit) && hit < bestT)
                    bestT = hit;
            }
            if (float.IsInfinity(bestT)) return false;

            vertex = _tree.Nearest(o + d * bestT);
            return vertex >= 0;
        }

        // Möller–Trumbore, two-sided; returns the positive ray parameter of the hit.
        static bool RayTriangle(float3 o, float3 d, float3 a, float3 b, float3 c, out float t)
        {
            t = 0f;
            float3 e1 = b - a, e2 = c - a;
            float3 p = math.cross(d, e2);
            float det = math.dot(e1, p);
            if (math.abs(det) < 1e-9f) return false;
            float inv = 1f / det;
            float3 tv = o - a;
            float u = math.dot(tv, p) * inv;
            if (u < 0f || u > 1f) return false;
            float3 q = math.cross(tv, e1);
            float v = math.dot(d, q) * inv;
            if (v < 0f || u + v > 1f) return false;
            t = math.dot(e2, q) * inv;
            return t > 1e-5f;
        }

        /// <summary>Nearest vertex to <paramref name="ray"/> within <paramref name="maxDistance"/>; false if none.</summary>
        public bool Pick(Ray ray, float maxDistance, out int vertex)
        {
            float3 dir = math.normalizesafe(ray.direction, new float3(0, 0, 1));
            vertex = _tree.NearestToRay(ray.origin, dir, maxDistance);
            return vertex >= 0;
        }

        /// <summary>Nearest vertex to the ray through <paramref name="screenPosition"/> from <paramref name="camera"/>.</summary>
        public bool Pick(Camera camera, Vector2 screenPosition, float maxDistance, out int vertex)
            => Pick(camera.ScreenPointToRay(screenPosition), maxDistance, out vertex);

        /// <summary>Nearest vertex to an arbitrary world/local point; false only if the mesh is empty.</summary>
        public bool PickNearest(float3 point, out int vertex)
        {
            vertex = _tree.Nearest(point);
            return vertex >= 0;
        }

        public void Dispose() => _tree.Dispose();
    }
}
