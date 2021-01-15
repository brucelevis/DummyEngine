#version 310 es

#include "_vs_common.glsl"

layout(location = REN_VTX_POS_LOC) in vec2 aVertexPosition;
layout(location = REN_VTX_UV1_LOC) in vec2 aVertexUVs;

layout(location = 0) uniform vec4 uTransform;

#if defined(VULKAN) || defined(GL_SPIRV)
layout(location = 0)
#endif
out vec2 aVertexUVs_;


void main() {
    aVertexUVs_ = uTransform.xy + aVertexUVs * uTransform.zw;
    gl_Position = vec4(aVertexPosition, 0.5, 1.0);
} 
