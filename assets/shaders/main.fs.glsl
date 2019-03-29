#version 310 es
#extension GL_EXT_texture_buffer : enable

$ModifyWarning

#ifdef GL_ES
    precision mediump float;
    precision mediump sampler2DShadow;
#endif

#define LIGHT_ATTEN_CUTOFF 0.001f

layout(binding = $DiffTexSlot) uniform sampler2D diffuse_texture;
layout(binding = $NormTexSlot) uniform sampler2D normals_texture;
layout(binding = $SpecTexSlot) uniform sampler2D specular_texture;
layout(binding = $ShadTexSlot) uniform sampler2DShadow shadow_texture;
layout(binding = $LmapDirSlot) uniform sampler2D lm_direct_texture;
layout(binding = $LmapIndirSlot) uniform sampler2D lm_indirect_texture;
layout(binding = $LmapSHSlot) uniform sampler2D lm_indirect_sh_texture[4];
layout(binding = $DecalTexSlot) uniform sampler2D decals_texture;
layout(binding = $SSAOTexSlot) uniform sampler2D ao_texture;
layout(binding = $LightBufSlot) uniform mediump samplerBuffer lights_buffer;
layout(binding = $DecalBufSlot) uniform mediump samplerBuffer decals_buffer;
layout(binding = $CellsBufSlot) uniform highp usamplerBuffer cells_buffer;
layout(binding = $ItemsBufSlot) uniform highp usamplerBuffer items_buffer;

layout (std140) uniform MatricesBlock {
    mat4 uMVPMatrix;
    mat4 uVMatrix;
    mat4 uMMatrix;
    mat4 uShadowMatrix[4];
    vec4 uClipInfo;
};

layout(location = 12) uniform vec3 sun_dir;
layout(location = 13) uniform vec3 sun_col;
layout(location = 14) uniform float gamma;
layout(location = 15) uniform ivec2 res;

in vec3 aVertexPos_;
in mat3 aVertexTBN_;
in vec2 aVertexUVs1_;
in vec2 aVertexUVs2_;

in vec3 aVertexShUVs_[4];

layout(location = $OutColorIndex) out vec4 outColor;
layout(location = $OutNormIndex) out vec2 outNormal;
layout(location = $OutSpecIndex) out vec4 outSpecular;

vec3 heatmap(float t) {
    vec3 r = vec3(t) * 2.1 - vec3(1.8, 1.14, 0.3);
    return vec3(1.0) - r * r;
}

vec3 RGBMDecode(vec4 rgbm) {
    return 6.0 * rgbm.rgb * rgbm.a;
}

float GetVisibility(float frag_depth, in vec2 lm_uvs, inout vec3 additional_light) {
    const vec2 poisson_disk[16] = vec2[16](
        vec2(-0.5, 0.0),
        vec2(0.0, 0.5),
        vec2(0.5, 0.0),
        vec2(0.0, -0.5),
    
        vec2(0.0, 0.0),
        vec2(-0.1, -0.32),
        vec2(0.17, 0.31),
        vec2(0.35, 0.04),
        
        vec2(0.07, 0.7),
        vec2(-0.72, 0.09),
        vec2(0.73, 0.05),
        vec2(0.1, -0.71),
        
        vec2(0.72, 0.8),
        vec2(-0.75, 0.74),
        vec2(-0.8, -0.73),
        vec2(0.75, -0.81)
    );

    const highp float shadow_softness = 2.0 / $ShadRes.0;
    
    float visibility = 0.0;
    
    if (frag_depth < $ShadCasc0Dist) {
        const float weight = 1.0 / $ShadCasc0Samp.0;
        for (int i = 0; i < $ShadCasc0Samp; i++) {
            visibility += texture(shadow_texture, aVertexShUVs_[0] + vec3(poisson_disk[i] * shadow_softness, 0.0)) * weight;
        }
    } else if (frag_depth < $ShadCasc1Dist) {
        const float weight = 1.0 / $ShadCasc1Samp.0;
        for (int i = 0; i < 8; i++) {
            visibility += texture(shadow_texture, aVertexShUVs_[1] + vec3(poisson_disk[i] * shadow_softness * 0.25, 0.0)) * weight;
        }
    } else if (frag_depth < $ShadCasc2Dist) {
        const float weight = 1.0 / $ShadCasc2Samp.0;
        for (int i = 0; i < 4; i++) {
            visibility += texture(shadow_texture, aVertexShUVs_[2] + vec3(poisson_disk[i] * shadow_softness * 0.125, 0.0)) * weight;
        }
    } else if (frag_depth < $ShadCasc3Dist) {
        visibility += texture(shadow_texture, aVertexShUVs_[3]);
    } else {
        // use direct lightmap
        additional_light += RGBMDecode(texture(lm_direct_texture, lm_uvs));
    }
    
    return visibility;
}

vec2 EncodeNormal(vec3 n) {
    vec2 enc = normalize(n.xy) * (sqrt(-n.z * 0.5 + 0.5));
    enc = enc * 0.5 + 0.5;
    return enc;
}

vec3 DecodeNormal(vec2 enc) {
    vec4 nn = vec4(2.0 * enc, 0.0, 0.0) + vec4(-1.0, -1.0, 1.0, -1.0);
    float l = dot(nn.xyz, -nn.xyw);
    nn.z = l;
    nn.xy *= sqrt(l);
    return 2.0 * nn.xyz + vec3(0.0, 0.0, -1.0);
}

void main(void) {
    float depth = uClipInfo[0] / (gl_FragCoord.z * (uClipInfo[1] - uClipInfo[2]) + uClipInfo[2]);
    
    float k = log2(depth / uClipInfo[1]) / uClipInfo[3];
    int slice = int(floor(k * $ItemGridResZ.0));
    
    int ix = int(gl_FragCoord.x);
    int iy = int(gl_FragCoord.y);
    int cell_index = slice * $ItemGridResX * $ItemGridResY + (iy * $ItemGridResY / res.y) * $ItemGridResX + ix * $ItemGridResX / res.x;
    
    highp uvec2 cell_data = texelFetch(cells_buffer, cell_index).xy;
    highp uvec2 offset_and_lcount = uvec2(bitfieldExtract(cell_data.x, 0, 24), bitfieldExtract(cell_data.x, 24, 8));
    highp uvec2 dcount_and_pcount = uvec2(bitfieldExtract(cell_data.y, 0, 8), 0);
    
    vec3 albedo_color = pow(texture(diffuse_texture, aVertexUVs1_).rgb, vec3(gamma));
    vec3 normal_color = texture(normals_texture, aVertexUVs1_).xyz;
    vec4 specular_color = texture(specular_texture, aVertexUVs1_);
    
    vec3 dp_dx = dFdx(aVertexPos_);
    vec3 dp_dy = dFdy(aVertexPos_);
    
    for (uint i = offset_and_lcount.x; i < offset_and_lcount.x + dcount_and_pcount.x; i++) {
        highp uint item_data = texelFetch(items_buffer, int(i)).x;
        int di = int(bitfieldExtract(item_data, 12, 12));
        
        mat4 de_proj;
        de_proj[0] = texelFetch(decals_buffer, di * 6 + 0);
        de_proj[1] = texelFetch(decals_buffer, di * 6 + 1);
        de_proj[2] = texelFetch(decals_buffer, di * 6 + 2);
        de_proj[3] = vec4(0.0, 0.0, 0.0, 1.0);
        de_proj = transpose(de_proj);
        
        vec4 pp = de_proj * vec4(aVertexPos_, 1.0);
        pp /= pp[3];
        
        vec3 app = abs(pp.xyz);
        vec2 uvs = pp.xy * 0.5 + 0.5;
        
        vec2 duv_dx = 0.5 * (de_proj * vec4(dp_dx, 0.0)).xy;
        vec2 duv_dy = 0.5 * (de_proj * vec4(dp_dy, 0.0)).xy;
        
        if (app.x < 1.0 && app.y < 1.0 && app.z < 1.0) {
            vec4 diff_uvs_tr = texelFetch(decals_buffer, di * 6 + 3);
            float decal_influence = 0.0;
            
            if (diff_uvs_tr.z > 0.0) {
                vec2 diff_uvs = diff_uvs_tr.xy + diff_uvs_tr.zw * uvs;
                
                vec2 _duv_dx = diff_uvs_tr.zw * duv_dx;
                vec2 _duv_dy = diff_uvs_tr.zw * duv_dy;
            
                vec4 decal_diff = textureGrad(decals_texture, diff_uvs, _duv_dx, _duv_dy);
                decal_influence = decal_diff.a;
                albedo_color = mix(albedo_color, decal_diff.xyz, decal_influence);
            }
            
            vec4 norm_uvs_tr = texelFetch(decals_buffer, di * 6 + 4);
            
            if (norm_uvs_tr.z > 0.0) {
                vec2 norm_uvs = norm_uvs_tr.xy + norm_uvs_tr.zw * uvs;
                
                vec2 _duv_dx = norm_uvs_tr.zw * duv_dx;
                vec2 _duv_dy = norm_uvs_tr.zw * duv_dy;
            
                vec4 decal_norm = textureGrad(decals_texture, norm_uvs, _duv_dx, _duv_dy);
                normal_color = mix(normal_color, decal_norm.xyz, decal_influence);
            }
            
            vec4 spec_uvs_tr = texelFetch(decals_buffer, di * 6 + 5);
            
            if (spec_uvs_tr.z > 0.0) {
                vec2 spec_uvs = spec_uvs_tr.xy + spec_uvs_tr.zw * uvs;
                
                vec2 _duv_dx = spec_uvs_tr.zw * duv_dx;
                vec2 _duv_dy = spec_uvs_tr.zw * duv_dy;
            
                vec4 decal_spec = textureGrad(decals_texture, spec_uvs, _duv_dx, _duv_dy);
                specular_color = mix(specular_color, decal_spec, decal_influence);
            }
        }
    }
    
    vec3 normal = normalize(normal_color * 2.0 - 1.0);
    normal = aVertexTBN_ * normal;
    
    vec3 additional_light = vec3(0.0, 0.0, 0.0);
    
    for (uint i = offset_and_lcount.x; i < offset_and_lcount.x + offset_and_lcount.y; i++) {
        highp uint item_data = texelFetch(items_buffer, int(i)).x;
        int li = int(bitfieldExtract(item_data, 0, 12));

        vec4 pos_and_radius = texelFetch(lights_buffer, li * 3 + 0);
        highp vec4 col_and_brightness = texelFetch(lights_buffer, li * 3 + 1);
        vec4 dir_and_spot = texelFetch(lights_buffer, li * 3 + 2);
        
        vec3 L = pos_and_radius.xyz - aVertexPos_;
        float dist = length(L);
        float d = max(dist - pos_and_radius.w, 0.0);
        L /= dist;
        
        highp float denom = d / pos_and_radius.w + 1.0;
        highp float atten = 1.0 / (denom * denom);
        
        highp float factor = LIGHT_ATTEN_CUTOFF / col_and_brightness.w;
        atten = (atten - factor) / (1.0 - LIGHT_ATTEN_CUTOFF);
        atten = max(atten, 0.0);
        
        float _dot1 = max(dot(L, normal), 0.0);
        float _dot2 = dot(L, dir_and_spot.xyz);
        
        atten = _dot1 * atten;
        if (_dot2 > dir_and_spot.w && (col_and_brightness.w * atten) > $FltEps) {
            additional_light += col_and_brightness.xyz * atten;
        }
    }
    
    vec2 lm_uvs = aVertexUVs2_;
    
    float lambert = max(dot(normal, sun_dir), 0.0);
    float visibility = 0.0;
    if (lambert > 0.00001) {
        visibility = GetVisibility(depth, lm_uvs, additional_light);
    }
    
    vec3 indirect_col = RGBMDecode(texture(lm_indirect_texture, lm_uvs));
    
    vec3 sh_l_00 = RGBMDecode(texture(lm_indirect_sh_texture[0], lm_uvs));
    vec3 sh_l_10 = texture(lm_indirect_sh_texture[1], lm_uvs).rgb;
    vec3 sh_l_11 = texture(lm_indirect_sh_texture[2], lm_uvs).rgb;
    vec3 sh_l_12 = texture(lm_indirect_sh_texture[3], lm_uvs).rgb;
    
    //indirect_col += sh_l_00 + sh_l_10 * normal.y + sh_l_11 * normal.z + sh_l_12 * normal.x;
    indirect_col += (0.5 + (sh_l_10 - vec3(0.5)) * normal.y +
                           (sh_l_11 - vec3(0.5)) * normal.z +
                           (sh_l_12 - vec3(0.5)) * normal.x) * sh_l_00 * 2.0;
    indirect_col = max(indirect_col, vec3(0.0));
    
    vec2 ao_uvs = gl_FragCoord.xy / vec2(float(res.x), float(res.y));
    float ambient_occlusion = texture(ao_texture, ao_uvs).r;
    
    vec3 diffuse_color = albedo_color * (sun_col * lambert * visibility + ambient_occlusion * indirect_col + additional_light);
    
    outColor = vec4(diffuse_color, 1.0);
    
    vec3 normal_vs = normalize((uVMatrix * vec4(normal, 0.0)).xyz);
    outNormal = EncodeNormal(normal_vs);
    
    outSpecular = vec4(vec3(ambient_occlusion), 1.0) * specular_color;
}