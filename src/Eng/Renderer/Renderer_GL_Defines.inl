
#define _AS_STR(x) #x
#define AS_STR(x) _AS_STR(x)

//#define REN_DIRECT_DRAWING

// Resolution of frustum item grid
#define REN_GRID_RES_X 16
#define REN_GRID_RES_Y 8
#define REN_GRID_RES_Z 24

// Attribute location
#define REN_VTX_POS_LOC 0
#define REN_VTX_NOR_LOC 1
#define REN_VTX_TAN_LOC 2
#define REN_VTX_UV1_LOC 3
#define REN_VTX_UV2_LOC 4

// Texture binding
#define REN_MAT_TEX0_SLOT   0
#define REN_MAT_TEX1_SLOT   1
#define REN_MAT_TEX2_SLOT   2
#define REN_SHAD_TEX_SLOT   3
#define REN_LMAP_SH_SLOT    4
#define REN_DECAL_TEX_SLOT  8
#define REN_SSAO_TEX_SLOT   9
#define REN_BRDF_TEX_SLOT   10
#define REN_LIGHT_BUF_SLOT  11
#define REN_DECAL_BUF_SLOT  12
#define REN_CELLS_BUF_SLOT  13
#define REN_ITEMS_BUF_SLOT  14
#define REN_INST_BUF_SLOT   15
#define REN_ENV_TEX_SLOT    16
#define REN_MOMENTS0_TEX_SLOT 4 // shares slot with lightmap 0
#define REN_MOMENTS1_TEX_SLOT 5 // shares slot with lightmap 1
#define REN_MOMENTS2_TEX_SLOT 6 // shares slot with lightmap 2
#define REN_MOMENTS0_MS_TEX_SLOT 7 shares slot with lightmap 3
#define REN_MOMENTS1_MS_TEX_SLOT 9 shares slot with ssao texture
#define REN_MOMENTS2_MS_TEX_SLOT 17

#define REN_ALPHATEST_TEX_SLOT 0

#define REN_BASE0_TEX_SLOT 0
#define REN_BASE1_TEX_SLOT 1

#define REN_REFL_DEPTH_TEX_SLOT 0
#define REN_REFL_NORM_TEX_SLOT  1
#define REN_REFL_SPEC_TEX_SLOT  2
#define REN_REFL_PREV_TEX_SLOT  3
#define REN_REFL_SSR_TEX_SLOT   4
#define REN_REFL_ENV_TEX_SLOT   5
#define REN_REFL_BRDF_TEX_SLOT  6
#define REN_REFL_DEPTH_LOW_TEX_SLOT 7

#define REN_U_M_MATRIX_LOC  0
#define REN_U_INSTANCES_LOC 1

#define REN_UB_SHARED_DATA_LOC  0
#define REN_UB_BATCH_DATA_LOC   1

// Shader output location
#define REN_OUT_COLOR_INDEX 0
#define REN_OUT_NORM_INDEX  1
#define REN_OUT_SPEC_INDEX  2

// Shadow resolution
#define REN_SHAD_RES_PC         8192
#define REN_SHAD_RES_ANDROID    4096

// Shadow cascades definition
#define REN_SHAD_CASCADE0_DIST      10.0
#define REN_SHAD_CASCADE0_SAMPLES   16
#define REN_SHAD_CASCADE1_DIST      24.0
#define REN_SHAD_CASCADE1_SAMPLES   8
#define REN_SHAD_CASCADE2_DIST      48.0
#define REN_SHAD_CASCADE2_SAMPLES   4
#define REN_SHAD_CASCADE3_DIST      96.0
#define REN_SHAD_CASCADE3_SAMPLES   4
#define REN_SHAD_CASCADE_SOFT       1

#define REN_MAX_BATCH_SIZE 8

#define REN_MAX_INSTANCES_TOTAL     262144
#define REN_MAX_SHADOWMAPS_TOTAL    32
#define REN_MAX_PROBES_TOTAL        32
#define REN_SKIN_REGION_SIZE        256
#define REN_MAX_SKIN_XFORMS_TOTAL   65536
#define REN_MAX_SKIN_REGIONS_TOTAL  262144
#define REN_MAX_SKIN_VERTICES_TOTAL 1048576

#define REN_MAX_SHADOW_BATCHES 262144
#define REN_MAX_MAIN_BATCHES 262144

#define REN_CELLS_COUNT (REN_GRID_RES_X * REN_GRID_RES_Y * REN_GRID_RES_Z)

#define REN_MAX_LIGHTS_PER_CELL 255
#define REN_MAX_DECALS_PER_CELL 255
#define REN_MAX_PROBES_PER_CELL 8
#define REN_MAX_ITEMS_PER_CELL  255

#define REN_MAX_LIGHTS_TOTAL 4096
#define REN_MAX_DECALS_TOTAL 4096
#define REN_MAX_ITEMS_TOTAL (1 << 16)

#define REN_OIT_DISABLED            0
#define REN_OIT_MOMENT_BASED        1
#define REN_OIT_WEIGHTED_BLENDED    2

#define REN_OIT_MOMENT_RENORMALIZE  1

#define REN_OIT_MODE REN_OIT_DISABLED
