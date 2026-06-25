using Unity.Collections;
using Unity.Mathematics;

namespace CompGeo.Core
{
    /// <summary>A 3D line segment between two endpoints.</summary>
    public readonly struct Segment3
    {
        public readonly float3 A;
        public readonly float3 B;

        public Segment3(float3 a, float3 b)
        {
            A = a;
            B = b;
        }

        public float3 Direction => B - A;
        public float Length => math.length(B - A);

        /// <summary>Point on the segment at parameter <paramref name="t"/> in [0, 1].</summary>
        public float3 Lerp(float t) => math.lerp(A, B, t);
    }

    /// <summary>An axis-aligned bounding box.</summary>
    public readonly struct Aabb
    {
        public readonly float3 Min;
        public readonly float3 Max;

        public Aabb(float3 min, float3 max)
        {
            Min = min;
            Max = max;
        }

        public float3 Center => 0.5f * (Min + Max);
        public float3 Size => Max - Min;

        public bool Contains(float3 p) => math.all(p >= Min) && math.all(p <= Max);

        /// <summary>Compute the bounding box of all vertices in <paramref name="mesh"/>.</summary>
        public static Aabb FromMesh(MeshData mesh)
        {
            NativeArray<float3> p = mesh.Positions;
            if (!p.IsCreated || p.Length == 0)
                return new Aabb(float3.zero, float3.zero);

            float3 min = p[0];
            float3 max = p[0];
            for (int i = 1; i < p.Length; i++)
            {
                min = math.min(min, p[i]);
                max = math.max(max, p[i]);
            }
            return new Aabb(min, max);
        }
    }
}
