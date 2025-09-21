Shader "Unlit/ClothRender"
{
    Properties
    {
        _MainTex("Texture", 2D) = "white" {}
    }
        SubShader
    {
        Tags { "RenderType" = "Opaque" }
        LOD 100

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            // make fog work
            #pragma multi_compile_fog

            #include "UnityCG.cginc"

            struct v2f
            {
                float4 pos : POSITION;
            };

            sampler2D _MainTex;
            float4 _MainTex_ST;
            float4x4 _ModelMatrix;

            struct clothData
            {
                float3 position;
            };

            StructuredBuffer<clothData> _clothDataBuffer;

            v2f vert(uint id : SV_VertexID)
            {
                v2f o;
                // 先应用模型矩阵将本地坐标转换为世界坐标，再转换为裁剪坐标
                float4 worldPos = mul(_ModelMatrix, float4(_clothDataBuffer[id].position, 1.0f));
                o.pos = UnityObjectToClipPos(worldPos);
                return o;
            }

            fixed4 frag(v2f i) : SV_Target
            {
                return float4(0.0f, 0.0f, 1.0f, 1.0f);
            }
            ENDCG
        }
    }
}

