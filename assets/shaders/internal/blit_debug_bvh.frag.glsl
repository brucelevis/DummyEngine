#version 310 es
#extension GL_EXT_texture_buffer : enable

#ifdef GL_ES
    precision highp float;
#endif

#include "_fs_common.glsl"

/*
UNIFORM_BLOCKS
    SharedDataBlock : $ubSharedDataLoc
PERM @MSAA_4
*/

#if defined(VULKAN) || defined(GL_SPIRV)
layout (binding = REN_UB_SHARED_DATA_LOC, std140)
#else
layout (std140)
#endif
uniform SharedDataBlock {
    SharedData shrd_data;
};

#if defined(MSAA_4)
layout(binding = 0) uniform mediump sampler2DMS depth_texture;
#else
layout(binding = 0) uniform mediump sampler2D depth_texture;
#endif
layout(binding = 1) uniform highp samplerBuffer nodes_buffer;
layout(location = 12) uniform int uRootIndex;

#if defined(VULKAN) || defined(GL_SPIRV)
layout(location = 0) in vec2 aVertexUVs_;
#else
in vec2 aVertexUVs_;
#endif

layout(location = 0) out vec4 outColor;

bool _bbox_test(vec3 o, vec3 inv_d, float t, vec3 bbox_min, vec3 bbox_max) {
    float low = inv_d.x * (bbox_min[0] - o.x);
    float high = inv_d.x * (bbox_max[0] - o.x);
    float tmin = min(low, high);
    float tmax = max(low, high);

    low = inv_d.y * (bbox_min[1] - o.y);
    high = inv_d.y * (bbox_max[1] - o.y);
    tmin = max(tmin, min(low, high));
    tmax = min(tmax, max(low, high));

    low = inv_d.z * (bbox_min[2] - o.z);
    high = inv_d.z * (bbox_max[2] - o.z);
    tmin = max(tmin, min(low, high));
    tmax = min(tmax, max(low, high));
    tmax *= 1.00000024;

    return tmin <= tmax && tmin <= t && tmax > 0.0;
}

void main() {
    vec2 norm_uvs = aVertexUVs_ / shrd_data.uResAndFRes.xy;

    float depth = texelFetch(depth_texture, ivec2(aVertexUVs_), 0).r;
    depth = 2.0 * depth - 1.0;

    vec4 ray_start_cs = vec4(aVertexUVs_ / shrd_data.uResAndFRes.xy, 0.0, 1.0);
    ray_start_cs.xy = 2.0 * ray_start_cs.xy - 1.0;

    vec4 ray_end_cs = vec4(aVertexUVs_ / shrd_data.uResAndFRes.xy, depth, 1.0);
    ray_end_cs.xy = 2.0 * ray_end_cs.xy - 1.0;

    vec4 ray_start_ws = shrd_data.uInvViewProjMatrix * ray_start_cs;
    ray_start_ws /= ray_start_ws.w;

    vec4 ray_end_ws = shrd_data.uInvViewProjMatrix * ray_end_cs;
    ray_end_ws /= ray_end_ws.w;

    vec3 ray_dir_ws = ray_end_ws.xyz - ray_start_ws.xyz;
    float ray_length = length(ray_dir_ws);
    ray_dir_ws /= ray_length;
    
    vec3 inv_dir = 1.0 / ray_dir_ws;

    int stack[32];
    int stack_size = 0;
    stack[stack_size++] = uRootIndex;

    int tree_complexity = 0;

    while (stack_size != 0) {
        int cur = stack[--stack_size];

        /*
            struct bvh_node_t {
                uvec4 node_data0;   // { prim_index  (u32), prim_count  (u32), left_child  (u32), right_child (u32) }
                xvec4 node_data1;   // { bbox_min[0] (f32), bbox_min[1] (f32), bbox_min[2] (f32), parent      (u32) }
                xvec4 node_data2;   // { bbox_max[0] (f32), bbox_max[1] (f32), bbox_max[2] (f32), space_axis  (u32) }
            };
        */

        vec4 node_data1 = texelFetch(nodes_buffer, cur * 3 + 1);
        vec4 node_data2 = texelFetch(nodes_buffer, cur * 3 + 2);

        if (!_bbox_test(ray_start_ws.xyz, inv_dir, 100.0, node_data1.xyz, node_data2.xyz)) continue;

        tree_complexity++;

        uvec4 node_data0 = floatBitsToUint(texelFetch(nodes_buffer, cur * 3 + 0));
        if (node_data0.y == 0u) {
            stack[stack_size++] = int(node_data0.w);
            stack[stack_size++] = int(node_data0.z);
        }
    }

    outColor = vec4(heatmap(float(tree_complexity) / 128.0), 0.85);
}
