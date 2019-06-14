#pragma once

#include <Ren/Camera.h>
#include <Ren/RingBuffer.h>
#include <Ren/TextureSplitter.h>
extern "C" {
#include <Ren/SW/SWculling.h>
}

#include "FrameBuf.h"
#include "Renderer_GL_Defines.inl"
#include "Renderer_Structs.h"
#include "../Scene/SceneData.h"

namespace Sys {
    class ThreadPool;
}

class TextureAtlas;

enum eRenderFlags {
    EnableZFill     = (1 << 0),
    EnableCulling   = (1 << 1),
    EnableSSR       = (1 << 2),
    EnableSSAO      = (1 << 3),
    EnableLightmap  = (1 << 4),
    EnableLights    = (1 << 5),
    EnableDecals    = (1 << 6),
    EnableProbes    = (1 << 7),
    EnableShadows   = (1 << 8),
    EnableTonemap   = (1 << 9),
    EnableBloom     = (1 << 10),
    EnableFxaa      = (1 << 11),
    EnableTimers    = (1 << 12),
    DebugWireframe  = (1 << 13),
    DebugCulling    = (1 << 14),
    DebugShadow     = (1 << 15),
    DebugReduce     = (1 << 16),
    DebugLights     = (1 << 17),
    DebugDeferred   = (1 << 18),
    DebugBlur       = (1 << 19),
    DebugDecals     = (1 << 20),
    DebugSSAO       = (1 << 21),
    DebugTimings    = (1 << 22),
    DebugBVH        = (1 << 23),
    DebugProbes     = (1 << 24)
};

template <typename T>
struct DynArray {
    std::unique_ptr<T[]> data;
    uint32_t count, capacity;
};

class Renderer {
public:
    Renderer(Ren::Context &ctx, std::shared_ptr<Sys::ThreadPool> &threads);
    ~Renderer();

    uint32_t render_flags() const {
        return render_flags_;
    }

    void set_render_flags(uint32_t f) {
        render_flags_ = f;
    }

    BackendInfo backend_info() const {
        return backend_info_;
    }

    struct ShadReg {
        const LightSource *ls;
        int pos[2], size[2];
        float cam_near, cam_far; // for debugging
        uint32_t last_update, last_visible;
    };

    struct DrawList {
        uint32_t        render_flags = default_flags;
        Ren::Camera     draw_cam;
        Environment     env;
        FrontendInfo    frontend_info;
        DynArray<InstanceData>          instances;
        DynArray<ShadowDrawBatch>       shadow_batches;
        DynArray<uint32_t>              shadow_batch_indices;
        DynArray<ShadowList>            shadow_lists;
        DynArray<ShadowMapRegion>       shadow_regions;
        DynArray<MainDrawBatch>         main_batches;
        DynArray<uint32_t>              main_batch_indices;
        DynArray<SkinTransform>         skin_transforms;
        DynArray<SkinRegion>            skin_regions;
        uint32_t                        skin_vertices_count;
        DynArray<LightSourceItem>       light_sources;
        DynArray<DecalItem>             decals;
        DynArray<ProbeItem>             probes;
        DynArray<CellData>              cells;
        DynArray<ItemData>              items;
        const Ren::TextureAtlas *decals_atlas = nullptr;
        const ProbeStorage *probe_storage = nullptr;

        // for debugging only, backend does not require nodes for drawing
        std::vector<bvh_node_t> temp_nodes;
        uint32_t root_index;
        DynArray<ShadReg>               cached_shadow_regions;

        DrawList() {
            skin_transforms.data.reset(new SkinTransform[REN_MAX_SKIN_XFORMS_TOTAL]);
            skin_transforms.capacity = REN_MAX_SKIN_XFORMS_TOTAL;
            skin_transforms.count = 0;

            skin_regions.data.reset(new SkinRegion[REN_MAX_SKIN_REGIONS_TOTAL]);
            skin_regions.capacity = REN_MAX_SKIN_REGIONS_TOTAL;
            skin_regions.count = 0;

            instances.data.reset(new InstanceData[REN_MAX_INSTANCES_TOTAL]);
            instances.capacity = REN_MAX_INSTANCES_TOTAL;
            instances.count = 0;

            shadow_batches.data.reset(new ShadowDrawBatch[REN_MAX_SHADOW_BATCHES]);
            shadow_batches.capacity = REN_MAX_SHADOW_BATCHES;
            shadow_batches.count = 0;

            shadow_batch_indices.data.reset(new uint32_t[REN_MAX_SHADOW_BATCHES]);
            shadow_batch_indices.capacity = REN_MAX_SHADOW_BATCHES;
            shadow_batch_indices.count = 0;

            shadow_lists.data.reset(new ShadowList[REN_MAX_SHADOWMAPS_TOTAL]);
            shadow_lists.capacity = REN_MAX_SHADOWMAPS_TOTAL;
            shadow_lists.count = 0;

            shadow_regions.data.reset(new ShadowMapRegion[REN_MAX_SHADOWMAPS_TOTAL]);
            shadow_regions.capacity = REN_MAX_SHADOWMAPS_TOTAL;
            shadow_regions.count = 0;

            main_batches.data.reset(new MainDrawBatch[REN_MAX_MAIN_BATCHES]);
            main_batches.capacity = REN_MAX_MAIN_BATCHES;
            main_batches.count = 0;

            main_batch_indices.data.reset(new uint32_t[REN_MAX_MAIN_BATCHES]);
            main_batch_indices.capacity = REN_MAX_MAIN_BATCHES;
            main_batch_indices.count = 0;

            light_sources.data.reset(new LightSourceItem[REN_MAX_LIGHTS_TOTAL]);
            light_sources.capacity = REN_MAX_LIGHTS_TOTAL;
            light_sources.count = 0;

            decals.data.reset(new DecalItem[REN_MAX_DECALS_TOTAL]);
            decals.capacity = REN_MAX_DECALS_TOTAL;
            decals.count = 0;

            probes.data.reset(new ProbeItem[REN_MAX_PROBES_TOTAL]);
            probes.capacity = REN_MAX_PROBES_TOTAL;
            probes.count = 0;

            cells.data.reset(new CellData[REN_CELLS_COUNT]);
            cells.capacity = REN_CELLS_COUNT;
            cells.count = REN_CELLS_COUNT;

            items.data.reset(new ItemData[REN_MAX_ITEMS_TOTAL]);
            items.capacity = REN_MAX_ITEMS_TOTAL;
            items.count = 0;

            cached_shadow_regions.data.reset(new ShadReg[REN_MAX_SHADOWMAPS_TOTAL]);
            cached_shadow_regions.capacity = REN_MAX_SHADOWMAPS_TOTAL;
            cached_shadow_regions.count = 0;
        }
    };

    void PrepareDrawList(const SceneData &scene, const Ren::Camera &cam, DrawList &list);
    void ExecuteDrawList(const DrawList &list, const FrameBuf *target = nullptr);

    void BlitPixels(const void *data, int w, int h, const Ren::eTexColorFormat format);
    void BlitPixelsTonemap(const void *data, int w, int h, const Ren::eTexColorFormat format);
    void BlitBuffer(float px, float py, float sx, float sy, const FrameBuf &buf, int first_att, int att_count, float multiplier = 1.0f);
    void BlitTexture(float px, float py, float sx, float sy, uint32_t tex_id, int resx, int resy, bool is_ms = false);

    void BlitToLightProbeFace(const FrameBuf &src_buf, const ProbeStorage &dst_store, int probe_index, int face);
    bool BlitProjectSH(const ProbeStorage &store, int probe_index, int iteration, LightProbe &probe);
private:
    Ren::Context &ctx_;
    std::shared_ptr<Sys::ThreadPool> threads_;
    SWcull_ctx cull_ctx_;
    Ren::ProgramRef skydome_prog_, fillz_solid_prog_, fillz_transp_prog_, shadow_solid_prog_, shadow_transp_prog_, blit_prog_, blit_ms_prog_, blit_combine_prog_, blit_combine_ms_prog_,
        blit_red_prog_, blit_down_prog_, blit_down_ms_prog_, blit_gauss_prog_, blit_gauss_sep_prog_, blit_debug_prog_, blit_debug_ms_prog_, blit_ssr_prog_, blit_ssr_ms_prog_,
        blit_ao_prog_, blit_ao_ms_prog_, blit_multiply_prog_, blit_multiply_ms_prog_, blit_debug_bvh_prog_, blit_debug_bvh_ms_prog_, blit_depth_prog_,
        blit_rgbm_prog_, blit_mipmap_prog_, blit_project_sh_prog_, blit_fxaa_prog_, probe_prog_, skinning_prog_;
    Ren::Texture2DRef dummy_black_, dummy_white_, rand2d_8x8_;

    FrameBuf clean_buf_, refl_buf_, down_buf_, blur_buf1_, blur_buf2_, shadow_buf_, reduced_buf_, ssao_buf_, probe_sample_buf_, combined_buf_;
    int scr_w_ = 0, scr_h_ = 0, act_w_ = 0, act_h_ = 0;

    Ren::TextureSplitter shadow_splitter_;

    static const uint32_t default_flags =
#if !defined(__ANDROID__)
        (EnableZFill | EnableCulling | EnableSSR | EnableSSAO | EnableLightmap | EnableLights | EnableDecals | EnableShadows | EnableTonemap | EnableBloom | EnableFxaa | EnableTimers);
#else
        (EnableZFill | EnableCulling | EnableSSR | EnableLightmap | EnableLights | EnableDecals | EnableShadows | EnableTonemap | EnableTimers);
#endif
    uint32_t render_flags_ = default_flags;

    int frame_counter_ = 0;

    DynArray<const LightSource *> litem_to_lsource_;
    DynArray<const Decal *> ditem_to_decal_;

    struct ProcessedObjData {
        uint32_t instance_index;
        uint32_t base_vertex;
    };
    std::vector<ProcessedObjData> proc_objects_;
    DynArray<BBox> decals_boxes_;
    BackendInfo backend_info_;
    uint64_t backend_cpu_start_, backend_cpu_end_;
    int64_t backend_time_diff_;
    float reduced_average_ = 0.0f;
    Ren::Mat4f down_buf_view_from_world_;

    DynArray<Ren::Frustum> temp_sub_frustums_;
    DynArray<SortSpan32> temp_sort_spans_32_[2];
    DynArray<SortSpan64> temp_sort_spans_64_[2];

#if defined(USE_GL_RENDER)
    static const int FrameSyncWindow = 2;

    uint32_t temp_tex_;
    Ren::eTexColorFormat temp_tex_format_;
    int temp_tex_w_ = 0, temp_tex_h_ = 0;

    uint32_t temp_framebuf_, skydome_framebuf_ = 0, depth_fill_framebuf_ = 0, refl_comb_framebuf_ = 0;

    uint32_t unif_shared_data_block_, unif_batch_data_block_;
    uint32_t temp_vao_, depth_pass_solid_vao_, depth_pass_transp_vao_, draw_pass_vao_, skydome_vao_, sphere_vao_;
    uint32_t temp_buf1_vtx_offset_, temp_buf2_vtx_offset_, temp_buf_ndx_offset_,
             skydome_vtx1_offset_, skydome_vtx2_offset_, skydome_ndx_offset_,
             sphere_vtx1_offset_, sphere_vtx2_offset_, sphere_ndx_offset_,
             skinned_buf1_vtx_offset_, skinned_buf2_vtx_offset_;
    uint32_t last_vertex_buf1_ = 0, last_vertex_buf2_ = 0, last_index_buffer_ = 0;
    uint32_t instances_buf_, instances_tbo_[FrameSyncWindow],
             skin_transforms_buf_, skin_transforms_tbo_,
             skin_regions_buf_, skin_regions_tbo_;
    uint32_t lights_buf_, lights_tbo_[FrameSyncWindow],
             decals_buf_, decals_tbo_[FrameSyncWindow],
             cells_buf_, cells_tbo_[FrameSyncWindow],
             items_buf_, items_tbo_[FrameSyncWindow];
    uint32_t reduce_pbo_[FrameSyncWindow], probe_sample_pbo_;
    int cur_reduce_pbo_ = 0;

    uint32_t nodes_buf_ = 0, nodes_tbo_ = 0;

    enum { TimeDrawStart, TimeSkinningStart, TimeShadowMapStart, TimeDepthOpaqueStart,
           TimeAOPassStart, TimeOpaqueStart, TimeTranspStart, TimeReflStart, TimeBlurStart,
           TimeBlitStart, TimeDrawEnd, TimersCount };
    uint32_t queries_[FrameSyncWindow][TimersCount];
    int cur_query_ = 0;

    void *buf_range_fences_[FrameSyncWindow] = {};
    int cur_buf_chunk_ = 0;

    void CheckInitVAOs();
#endif

    DynArray<ShadReg> allocated_shadow_regions_;

    //temp
    std::vector<uint8_t> depth_pixels_[2], depth_tiles_[2];
    float debug_roughness_ = 0.0f;

    void GatherDrawables(const SceneData &scene, const Ren::Camera &cam, DrawList &list);

    void __push_skeletal_mesh(uint32_t obj_index, const AnimState *as, const Ren::Mesh *mesh, DrawList &list);

    void InitRendererInternal();
    bool InitFramebuffersInternal();
    void DestroyRendererInternal();
    void DrawObjectsInternal(const DrawList &list, const FrameBuf *target);
    uint64_t GetGpuTimeBlockingUs();

    // Parallel Jobs
    static void GatherItemsForZSlice_Job(int slice, const Ren::Frustum *sub_frustums, const LightSourceItem *lights, int lights_count,
                                         const DecalItem *decals, int decals_count, const BBox *decals_boxes, const ProbeItem *probes, int probes_count,
                                         const LightSource * const *litem_to_lsource, CellData *cells, ItemData *items, std::atomic_int &items_count);
};