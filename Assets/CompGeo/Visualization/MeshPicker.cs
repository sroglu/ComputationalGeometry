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

        public MeshPicker(in MeshData mesh)
        {
            _tree = KdTree3.Build(mesh.Positions, Allocator.Persistent);
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
