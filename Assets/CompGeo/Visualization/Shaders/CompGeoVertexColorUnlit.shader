// Minimal URP unlit shader that simply outputs interpolated vertex colour.
// Used for the point cloud (MeshTopology.Points), the edge mesh (MeshTopology.Lines) and the
// highlighted path. Per-vertex colour carries the geodesic heatmap / selection state, so a single
// mesh + single draw replaces the old GameObject-per-element rendering (docs/MIGRATION.md §3).
//
// Emits a small PSIZE for MeshTopology.Points: without it, point size is undefined on some backends
// (Metal renders huge point quads that swamp the view). PSIZE is ignored for line/triangle topology.
Shader "CompGeo/VertexColorUnlit"
{
    Properties
    {
        [Toggle] _ZWrite ("ZWrite", Float) = 1
        _PointSize ("Point Size", Float) = 5
    }

    SubShader
    {
        Tags { "RenderPipeline" = "UniversalPipeline" "Queue" = "Geometry" "RenderType" = "Opaque" }

        Pass
        {
            Name "Unlit"
            ZWrite [_ZWrite]
            Cull Off

            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag

            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"

            float _PointSize;

            struct Attributes
            {
                float4 positionOS : POSITION;
                half4  color      : COLOR;
            };

            struct Varyings
            {
                float4 positionHCS : SV_POSITION;
                half4  color       : COLOR;
                float  pointSize   : PSIZE;
            };

            Varyings vert (Attributes IN)
            {
                Varyings OUT;
                OUT.positionHCS = TransformObjectToHClip(IN.positionOS.xyz);
                OUT.color = IN.color;
                OUT.pointSize = _PointSize;
                return OUT;
            }

            half4 frag (Varyings IN) : SV_Target
            {
                return IN.color;
            }
            ENDHLSL
        }
    }

    Fallback Off
}
