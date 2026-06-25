using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace CompGeo.Visualization
{
    /// <summary>
    /// Maps a per-vertex scalar field (e.g. a geodesic distance field from
    /// <c>DijkstraGeodesics</c> / <c>AStarGeodesics</c>) to per-vertex colours for the GPU view.
    /// Non-finite entries (+∞ unreachable, NaN) get a dedicated colour so they read as "no data".
    /// </summary>
    public static class Heatmap
    {
        /// <summary>Fill <paramref name="colors"/> from <paramref name="field"/>, auto-ranging over the finite values.</summary>
        public static void Apply(NativeArray<float> field, NativeArray<Color> colors, Color unreachable)
        {
            float min = float.PositiveInfinity;
            float max = float.NegativeInfinity;
            for (int i = 0; i < field.Length; i++)
            {
                float v = field[i];
                if (!float.IsFinite(v)) continue;
                if (v < min) min = v;
                if (v > max) max = v;
            }
            Apply(field, colors, min, max, unreachable);
        }

        /// <summary>Fill <paramref name="colors"/> from <paramref name="field"/> with an explicit value range.</summary>
        public static void Apply(NativeArray<float> field, NativeArray<Color> colors, float min, float max, Color unreachable)
        {
            float inv = max > min ? 1f / (max - min) : 0f;
            for (int i = 0; i < field.Length; i++)
            {
                float v = field[i];
                colors[i] = float.IsFinite(v) ? Ramp(math.saturate((v - min) * inv)) : unreachable;
            }
        }

        /// <summary>A simple perceptual-ish ramp blue → cyan → green → yellow → red for t in [0, 1].</summary>
        public static Color Ramp(float t)
        {
            t = math.saturate(t);
            // Four equal segments, each a linear interpolation between two stops.
            float4 c = t < 0.25f ? math.lerp(new float4(0, 0, 1, 1), new float4(0, 1, 1, 1), t * 4f)
                     : t < 0.50f ? math.lerp(new float4(0, 1, 1, 1), new float4(0, 1, 0, 1), (t - 0.25f) * 4f)
                     : t < 0.75f ? math.lerp(new float4(0, 1, 0, 1), new float4(1, 1, 0, 1), (t - 0.50f) * 4f)
                                 : math.lerp(new float4(1, 1, 0, 1), new float4(1, 0, 0, 1), (t - 0.75f) * 4f);
            return new Color(c.x, c.y, c.z, c.w);
        }
    }
}
