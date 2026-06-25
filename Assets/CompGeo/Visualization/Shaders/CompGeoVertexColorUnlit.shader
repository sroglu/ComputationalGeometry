// Minimal URP unlit shader that simply outputs interpolated vertex colour.
// Used for the point cloud (MeshTopology.Points), the edge mesh (MeshTopology.Lines) and the
// highlighted path. Per-vertex colour carries the geodesic heatmap / selection state, so a single
// mesh + single draw replaces the old GameObject-per-element rendering (docs/MIGRATION.md §3).
//
// Note: point size is not controllable in URP without a geometry shader, so MeshTopology.Points
// renders 1px points. For larger points, render instanced quads instead (future work).
Shader "CompGeo/VertexColorUnlit"
{
    Properties
    {
        [Toggle] _ZWrite ("ZWrite", Float) = 1
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

            struct Attributes
            {
                float4 positionOS : POSITION;
                half4  color      : COLOR;
            };

            struct Varyings
            {
                float4 positionHCS : SV_POSITION;
                half4  color       : COLOR;
            };

            Varyings vert (Attributes IN)
            {
                Varyings OUT;
                OUT.positionHCS = TransformObjectToHClip(IN.positionOS.xyz);
                OUT.color = IN.color;
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
