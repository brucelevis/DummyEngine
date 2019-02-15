R"(
#version 310 es

#ifdef GL_ES
	precision mediump float;
#endif

layout(binding = 0) uniform mediump sampler2DMS depth_texture;
layout(binding = 1) uniform mediump sampler2DMS norm_texture;
layout(binding = 2) uniform mediump sampler2DMS spec_texture;
layout(binding = 3) uniform mediump sampler2D prev_texture;

layout(location = 0) uniform mat4 proj_matrix;
layout(location = 1) uniform mat4 inv_proj_matrix;
layout(location = 2) uniform mat4 delta_matrix;
layout(location = 3) uniform vec2 zbuffer_size;

in vec2 aVertexUVs_;

out vec4 outColor;

float distance2(in vec2 P0, in vec2 P1) {
    vec2 d = P1 - P0;
    return d.x * d.x + d.y * d.y;
}

float rand(vec2 co) {
  return fract(sin(dot(co.xy, vec2(12.9898, 78.233))) * 43758.5453);
}

float LinearDepthTexelFetch(ivec2 hit_pixel) {
    const float n = 0.5;
    const float f = 10000.0;

    float depth = texelFetch(depth_texture, hit_pixel, 0).r;
    depth = 2.0 * depth - 1.0;
    depth = 2.0 * n * f / (f + n - depth * (f - n));
    return depth;
}

bool IntersectRay(in vec3 ray_origin_vs, in vec3 ray_dir_vs, out vec2 hit_pixel, out vec3 hit_point) {
    const float n = 0.5;
    const float max_dist = 100.0;

    // from "Efficient GPU Screen-Space Ray Tracing"

    // Clip ray length to camera near plane
    float ray_length = (ray_origin_vs.z + ray_dir_vs.z * max_dist) > -n ?
                       (-ray_origin_vs.z + n) / ray_dir_vs.z :
                       max_dist;

    vec3 ray_end_vs = ray_origin_vs + ray_length * ray_dir_vs;

    // Project into screen space
    vec4 H0 = proj_matrix * vec4(ray_origin_vs, 1.0),
         H1 = proj_matrix * vec4(ray_end_vs, 1.0);
    float k0 = 1.0 / H0.w, k1 = 1.0 / H1.w;

    vec3 Q0 = ray_origin_vs * k0,
         Q1 = ray_end_vs * k1;

    // Screen-space endpoints
    vec2 P0 = H0.xy * k0, P1 = H1.xy * k1;

    P1 += vec2((distance2(P0, P1) < 0.0001) ? 0.01 : 0.0);

    P0 = 0.5 * P0 + 0.5;
    P1 = 0.5 * P1 + 0.5;

    P0 *= zbuffer_size;
    P1 *= zbuffer_size;

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

    vec3 dQ = (Q1 - Q0) * inv_dx;
    float dk = (k1 - k0) * inv_dx;
    vec2 dP = vec2(step_dir, delta.y * inv_dx);

        float stride = 0.025 * zbuffer_size.x; //16.0;
        dP *= stride;
        dQ *= stride;
        dk *= stride;

    ivec2 c = ivec2(gl_FragCoord.xy);
    float jitter = rand(gl_FragCoord.xy); //float((c.x + c.y) & 1) * 0.5;    

    P0 += dP * (1.0 + jitter);
    Q0 += dQ * (1.0 + jitter);
    k0 += dk * (1.0 + jitter);

    vec3 Q = Q0;
    float k = k0;
    float step_count = 0.0f;
    float end = P1.x * step_dir;
    float prev_zmax_estimate = ray_origin_vs.z;
    hit_pixel = vec2(-1.0, -1.0);

    const float max_steps = 24.0;
        
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

        const float z_thickness = 1.0;

        vec2 pixel = permute ? P.yx : P;
        float scene_zmax = -LinearDepthTexelFetch(ivec2(pixel));
        float scene_zmin = scene_zmax - z_thickness;

        if (((ray_zmax >= scene_zmin) && (ray_zmin <= scene_zmax)) || scene_zmax >= -n) {
            hit_pixel = P;
            break;
        }
    }

    vec2 test_pixel = permute ? hit_pixel.yx : hit_pixel;
    bool res = all(lessThanEqual(abs(test_pixel - (zbuffer_size * 0.5)), zbuffer_size * 0.5));

    if (res) {
        Q.xy += dQ.xy * step_count;

        // perform binary search to find intersection more accurately
        for (int i = 0; i < 8; i++) {
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
    
    return res;
}

vec3 DecodeNormal(vec2 enc) {
    vec4 nn = vec4(2.0 * enc, 0.0, 0.0) + vec4(-1.0, -1.0, 1.0, -1.0);
    float l = dot(nn.xyz, -nn.xyw);
    nn.z = l;
    nn.xy *= sqrt(l);
    return 2.0 * nn.xyz + vec3(0.0, 0.0, -1.0);
}

void main() {
    const float n = 0.5;
    const float f = 10000.0;

    vec4 prev_color = vec4(0.0);
    float prev_depth = -1.1f;

    for (int i = 0; i < 1; i++) {
        vec4 specular = texelFetch(spec_texture, ivec2(aVertexUVs_), i);
        if (dot(specular.xyz, specular.xyz) < 0.01) continue;

        float depth = texelFetch(depth_texture, ivec2(aVertexUVs_), i).r;
        depth = 2.0 * depth - 1.0;
        //depth = 2.0 * n * f / (f + n - depth * (f - n));

        /*if (abs(depth - prev_depth) < 0.005) {
            outColor += 0.25 * prev_color;
            continue;
        }*/

        vec3 normal = DecodeNormal(texelFetch(norm_texture, ivec2(aVertexUVs_), i).xy);

        vec4 ray_origin_cs = vec4(aVertexUVs_.xy / zbuffer_size, depth, 1.0f);
        ray_origin_cs.xy = 2.0 * ray_origin_cs.xy - 1.0;

        vec4 ray_origin_vs = inv_proj_matrix * ray_origin_cs;
        ray_origin_vs /= ray_origin_vs.w;

        vec3 view_ray_vs = normalize(ray_origin_vs.xyz);
        vec3 refl_ray_vs = reflect(view_ray_vs, normal);

        vec2 hit_pixel;
        vec3 hit_point;
    
        if (IntersectRay(ray_origin_vs.xyz, refl_ray_vs, hit_pixel, hit_point)) {
            hit_pixel /= zbuffer_size;

            // reproject hitpoint in view space of previous frame
            vec4 hit_prev = delta_matrix * vec4(hit_point, 1.0);
            hit_prev = proj_matrix * hit_prev;
            hit_prev /= hit_prev.w;
            hit_prev.xy = 0.5 * hit_prev.xy + 0.5;
            
            vec4 tex_color = textureLod(prev_texture, hit_prev.xy, 0.0);

            const float R0 = 0.0f;
            float fresnel = R0 + (1.0 - R0) * pow(1.0 - dot(normal, -view_ray_vs), 5.0);;

            vec3 infl = fresnel * specular.xyz;
            infl *= max(1.0 - 2.0 * distance(hit_pixel, vec2(0.5, 0.5)), 0.0);

            prev_depth = depth;
            prev_color = vec4(infl * tex_color.xyz, 1.0);
            outColor += 1.0 * prev_color;
        }
    }
}
)"