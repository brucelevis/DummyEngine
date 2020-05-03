#version 310 es
#extension GL_EXT_texture_buffer : enable
#extension GL_OES_texture_buffer : enable
//#extension GL_EXT_control_flow_attributes : enable

$ModifyWarning

#include "common_vs.glsl"

/*
UNIFORM_BLOCKS
    SharedDataBlock : $ubSharedDataLoc
*/

layout(location = REN_VTX_POS_LOC) in vec3 aVertexPosition;
layout(location = REN_VTX_NOR_LOC) in vec4 aVertexNormal;
layout(location = REN_VTX_TAN_LOC) in vec2 aVertexTangent;
layout(location = REN_VTX_UV1_LOC) in vec2 aVertexUVs1;
layout(location = REN_VTX_AUX_LOC) in uint aVertexOcclusion;

#if defined(VULKAN) || defined(GL_SPIRV)
layout (binding = 0, std140)
#else
layout (std140)
#endif
uniform SharedDataBlock {
    SharedData shrd_data;
};

layout (location = REN_U_INSTANCES_LOC) uniform ivec4 uInstanceIndices[REN_MAX_BATCH_SIZE / 4];

layout(binding = REN_INST_BUF_SLOT) uniform highp samplerBuffer instances_buffer;

#if defined(VULKAN) || defined(GL_SPIRV)
layout(location = 0) out highp vec3 aVertexPos_;
layout(location = 1) out mediump vec2 aVertexUVs_;
layout(location = 2) out mediump vec3 aVertexNormal_;
layout(location = 3) out mediump vec3 aVertexTangent_;
layout(location = 4) out mediump vec4 aVertexOcclusion_;
layout(location = 5) out highp vec3 aVertexShUVs_[4];
#else
out highp vec3 aVertexPos_;
out mediump vec2 aVertexUVs_;
out mediump vec3 aVertexNormal_;
out mediump vec3 aVertexTangent_;
out mediump vec4 aVertexOcclusion_;
out highp vec3 aVertexShUVs_[4];
#endif

#ifdef VULKAN
    #define gl_InstanceID gl_InstanceIndex
#endif

invariant gl_Position;

void main(void) {
    int instance = uInstanceIndices[gl_InstanceID / 4][gl_InstanceID % 4];

    // load model matrix
    mat4 MMatrix;
    MMatrix[0] = texelFetch(instances_buffer, instance * 4 + 0);
    MMatrix[1] = texelFetch(instances_buffer, instance * 4 + 1);
    MMatrix[2] = texelFetch(instances_buffer, instance * 4 + 2);
    MMatrix[3] = vec4(0.0, 0.0, 0.0, 1.0);

    MMatrix = transpose(MMatrix);

    vec3 vertex_position_ws = (MMatrix * vec4(aVertexPosition, 1.0)).xyz;
    vec3 vertex_normal_ws = normalize((MMatrix * vec4(aVertexNormal.xyz, 0.0)).xyz);
    vec3 vertex_tangent_ws = normalize((MMatrix * vec4(aVertexNormal.w, aVertexTangent, 0.0)).xyz);

    aVertexPos_ = vertex_position_ws;
    aVertexNormal_ = vertex_normal_ws;
    aVertexTangent_ = vertex_tangent_ws;
    aVertexUVs_ = aVertexUVs1;
    
    aVertexOcclusion_ = unpackUnorm4x8(aVertexOcclusion);
    aVertexOcclusion_.xyz = 2.0 * aVertexOcclusion_.xyz - vec3(1.0);
    aVertexOcclusion_.xyz = normalize(mat3(aVertexTangent_, cross(aVertexNormal_, aVertexTangent_), aVertexNormal_) * aVertexOcclusion_.xyz);
    
    const vec2 offsets[4] = vec2[4](
        vec2(0.0, 0.0),
        vec2(0.25, 0.0),
        vec2(0.0, 0.5),
        vec2(0.25, 0.5)
    );
    
    /*[[unroll]]*/ for (int i = 0; i < 4; i++) {
        aVertexShUVs_[i] = (shrd_data.uShadowMapRegions[i].clip_from_world * vec4(vertex_position_ws, 1.0)).xyz;
        aVertexShUVs_[i] = 0.5 * aVertexShUVs_[i] + 0.5;
        aVertexShUVs_[i].xy *= vec2(0.25, 0.5);
        aVertexShUVs_[i].xy += offsets[i];
    }
    
    gl_Position = shrd_data.uViewProjMatrix * vec4(vertex_position_ws, 1.0);
} 
