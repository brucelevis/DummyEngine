#version 310 es
#extension GL_ARB_texture_multisample : enable
#extension GL_EXT_texture_buffer : enable

#ifdef GL_ES
    precision mediump float;
#endif

#include "_fs_common.glsl"

/*
UNIFORM_BLOCKS
    SharedDataBlock : $ubSharedDataLoc
PERM @MSAA_4
*/

#define Z_THICKNESS 0.05
#define STRIDE 0.0125
#define MAX_STEPS 48.0
#define BSEARCH_STEPS 4

#if defined(VULKAN) || defined(GL_SPIRV)
layout (binding = REN_UB_SHARED_DATA_LOC, std140)
#else
layout (std140)
#endif
uniform SharedDataBlock {
    SharedData shrd_data;
};

layout(binding = REN_REFL_DEPTH_TEX_SLOT) uniform mediump sampler2D depth_texture;
#if defined(MSAA_4)
layout(binding = REN_REFL_NORM_TEX_SLOT) uniform mediump sampler2DMS norm_texture;
layout(binding = REN_REFL_SPEC_TEX_SLOT) uniform mediump sampler2DMS spec_texture;
#else
layout(binding = REN_REFL_NORM_TEX_SLOT) uniform mediump sampler2D norm_texture;
layout(binding = REN_REFL_SPEC_TEX_SLOT) uniform mediump sampler2D spec_texture;
#endif

#if defined(VULKAN) || defined(GL_SPIRV)
layout(location = 0) in vec2 aVertexUVs_;
#else
in vec2 aVertexUVs_;
#endif

layout(location = 0) out vec4 outColor;

float distance2(vec2 P0, vec2 P1) {
    vec2 d = P1 - P0;
    return d.x * d.x + d.y * d.y;
}

float LinearDepthTexelFetch(ivec2 hit_pixel) {
    return texelFetch(depth_texture, hit_pixel / 2, 0).r;
}

bool IntersectRay(vec3 ray_origin_vs, vec3 ray_dir_vs, float jitter, out vec2 hit_pixel, out vec3 hit_point) {
    const float max_dist = 100.0;

    // from "Efficient GPU Screen-Space Ray Tracing"

    // Clip ray length to camera near plane
    float ray_length = (ray_origin_vs.z + ray_dir_vs.z * max_dist) > - shrd_data.uClipInfo[1] ?
                       (-ray_origin_vs.z - shrd_data.uClipInfo[1]) / ray_dir_vs.z :
                       max_dist;

    vec3 ray_end_vs = ray_origin_vs + ray_length * ray_dir_vs;

    // Project into screen space
    vec4 H0 = shrd_data.uProjMatrix * vec4(ray_origin_vs, 1.0),
         H1 = shrd_data.uProjMatrix * vec4(ray_end_vs, 1.0);
    float k0 = 1.0 / H0.w, k1 = 1.0 / H1.w;

    vec3 Q0 = ray_origin_vs * k0,
         Q1 = ray_end_vs * k1;

    // Screen-space endpoints
    vec2 P0 = H0.xy * k0, P1 = H1.xy * k1;

    P1 += vec2((distance2(P0, P1) < 0.0001) ? 0.01 : 0.0);

    P0 = 0.5 * P0 + 0.5;
    P1 = 0.5 * P1 + 0.5;

    P0 *= shrd_data.uResAndFRes.xy;
    P1 *= shrd_data.uResAndFRes.xy;

    vec2 delta = P1 - P0;

    bool permute = false;
    if (abs(delta.x) < abs(delta.y)) {
        permute = true;
        delta = delta.yx;
        P0 = P0.yx;
        P1 = P1.yx;
    }

    float step_dir = sign(delta.x);
    float inv_dx = step_dir / delta.x;
    vec2 dP = vec2(step_dir, delta.y * inv_dx);

    vec3 dQ = (Q1 - Q0) * inv_dx;
    float dk = (k1 - k0) * inv_dx;

    float stride = STRIDE * shrd_data.uResAndFRes.x;
    dP *= stride;
    dQ *= stride;
    dk *= stride;

    P0 += dP * (1.0 + jitter);
    Q0 += dQ * (1.0 + jitter);
    k0 += dk * (1.0 + jitter);

    vec3 Q = Q0;
    float k = k0;
    float step_count = 0.0;
    float end = P1.x * step_dir;
    float prev_zmax_estimate = ray_origin_vs.z + 0.1;
    hit_pixel = vec2(-1.0, -1.0);

    const float max_steps = MAX_STEPS;
        
    for (vec2 P = P0;
        ((P.x * step_dir) <= end) && (step_count < max_steps);
         P += dP, Q.z += dQ.z, k += dk, step_count += 1.0) {

        float ray_zmin = prev_zmax_estimate;
        // take half of step forward
        float ray_zmax = (dQ.z * 0.5 + Q.z) / (dk * 0.5 + k);
        prev_zmax_estimate = ray_zmax;

        if(ray_zmin > ray_zmax) {
            float temp = ray_zmin; ray_zmin = ray_zmax; ray_zmax = temp;
        }

        vec2 pixel = permute ? P.yx : P;

        float scene_zmax = -LinearDepthTexelFetch(ivec2(pixel));
        float scene_zmin = scene_zmax - Z_THICKNESS;

        if ((ray_zmax >= scene_zmin) && (ray_zmin <= scene_zmax)) {
            hit_pixel = P;
            break;
        }
    }

    vec2 test_pixel = permute ? hit_pixel.yx : hit_pixel;
    bool res = all(lessThanEqual(abs(test_pixel - (shrd_data.uResAndFRes.xy * 0.5)), shrd_data.uResAndFRes.xy * 0.5));

#if BSEARCH_STEPS != 0
    if (res) {
        Q.xy += dQ.xy * step_count;

        // perform binary search to find intersection more accurately
        for (int i = 0; i < BSEARCH_STEPS; i++) {
            vec2 pixel = permute ? hit_pixel.yx : hit_pixel;
            float scene_z = -LinearDepthTexelFetch(ivec2(pixel));
            float ray_z = Q.z / k;
    
            float depth_diff = ray_z - scene_z;
        
            dQ *= 0.5;
            dP *= 0.5;
            dk *= 0.5;
            if (depth_diff > 0.0) {
                Q += dQ;
                hit_pixel += dP;
                k += dk;
            } else {
                Q -= dQ;
                hit_pixel -= dP;
                k -= dk;
            }
        }

        hit_pixel = permute ? hit_pixel.yx : hit_pixel;
        hit_point = Q * (1.0 / k);
    }
#else
    Q.xy += dQ.xy * step_count;

    hit_pixel = permute ? hit_pixel.yx : hit_pixel;
    hit_point = Q * (1.0 / k);
#endif
    
    return res;
}


void main() {
    outColor = vec4(0.0);

    ivec2 pix_uvs = ivec2(aVertexUVs_ + vec2(0.5));
    vec2 norm_uvs = aVertexUVs_ / shrd_data.uResAndFRes.xy;

    vec4 normal_tex = texelFetch(norm_texture, pix_uvs + ivec2(0, 0), 0);
    if (normal_tex.w < 0.0001) return;

    float depth = DelinearizeDepth(texelFetch(depth_texture, pix_uvs / 2, 0).r, shrd_data.uClipInfo);

    vec3 normal_ws = 2.0 * normal_tex.xyz - 1.0;
    vec3 normal_vs = (shrd_data.uViewMatrix * vec4(normal_ws, 0.0)).xyz;

    vec4 ray_origin_cs = vec4(norm_uvs, depth, 1.0);
    ray_origin_cs.xyz = 2.0 * ray_origin_cs.xyz - vec3(1.0);

    vec4 ray_origin_vs = shrd_data.uInvProjMatrix * ray_origin_cs;
    ray_origin_vs /= ray_origin_vs.w;

    vec3 view_ray_vs = normalize(ray_origin_vs.xyz);
    vec3 refl_ray_vs = reflect(view_ray_vs, normal_vs);

    ivec2 c = ivec2(gl_FragCoord.xy);
    float jitter = float((c.x + c.y) & 1) * 0.5;    

    vec2 hit_pixel;
    vec3 hit_point;
    
    if (IntersectRay(ray_origin_vs.xyz, refl_ray_vs, jitter, hit_pixel, hit_point)) {
        hit_pixel /= shrd_data.uResAndFRes.xy;

        // reproject hitpoint into a clip space of previous frame
        vec4 hit_prev = shrd_data.uDeltaMatrix * vec4(hit_point, 1.0);
        hit_prev /= hit_prev.w;
        hit_prev.xy = 0.5 * hit_prev.xy + 0.5;

        float mm = max(abs(0.5 - hit_prev.x), abs(0.5 - hit_prev.y));
        float mix_factor = min(4.0 * (1.0 - 2.0 * mm), 1.0);

        outColor.rg = hit_prev.xy;
        outColor.b = mix_factor;
    }
}

