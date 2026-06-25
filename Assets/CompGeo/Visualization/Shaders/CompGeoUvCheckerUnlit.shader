// URP unlit shader that paints a crisp checkerboard computed per-pixel from the interpolated UV, so the
// pattern resolution is independent of the mesh tessellation (per-vertex colours blur/alias on coarse or
// irregular meshes). Used by UnfoldMode to show the Tutte parameterization: UVs in [-1,1] are remapped to
// [0,1] and tiled at _Frequency cells. Cull Off so the surface reads from both sides.
Shader "CompGeo/UvCheckerUnlit"
{
    Properties
    {
        _ColorA ("Color A", Color) = (0.95, 0.95, 0.95, 1)
        _ColorB ("Color B", Color) = (0.15, 0.35, 0.85, 1)
        _Frequency ("Frequency", Float) = 12
    }

    SubShader
    {
        Tags { "RenderPipeline" = "UniversalPipeline" "Queue" = "Geometry" "RenderType" = "Opaque" }

        Pass
        {
            Name "UvChecker"
            Cull Off

            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag

            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"

            float4 _ColorA;
            float4 _ColorB;
            float _Frequency;

            struct Attributes
            {
                float4 positionOS : POSITION;
                float2 uv         : TEXCOORD0;
            };

            struct Varyings
            {
                float4 positionHCS : SV_POSITION;
                float2 uv          : TEXCOORD0;
            };

            Varyings vert (Attributes IN)
            {
                Varyings OUT;
                OUT.positionHCS = TransformObjectToHClip(IN.positionOS.xyz);
                OUT.uv = IN.uv;
                return OUT;
            }

            half4 frag (Varyings IN) : SV_Target
            {
                float2 cell = floor((IN.uv * 0.5 + 0.5) * _Frequency);
                float checker = fmod(cell.x + cell.y, 2.0);
                return checker < 1.0 ? (half4)_ColorA : (half4)_ColorB;
            }
            ENDHLSL
        }
    }

    Fallback Off
}
