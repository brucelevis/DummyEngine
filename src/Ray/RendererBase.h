#pragma once

#include <memory>

#include "SceneBase.h"
#include "Types.h"

/**
  @file RendererBase.h
*/

namespace Ray {
/// Renderer flags used to choose backend
enum eRendererType : uint32_t {
    RendererRef     = (1u << 0u),
    RendererSSE2    = (1u << 1u),
    RendererAVX     = (1u << 2u),
    RendererAVX2    = (1u << 3u),
    RendererNEON    = (1u << 4u),
    RendererOCL     = (1u << 5u),
};

/// Renderer settings
struct settings_t {
    int w, h;
#if !defined(DISABLE_OCL)
    int platform_index = -1, device_index = -1;
#endif
    bool use_wide_bvh = true;
};

/** Render region context,
    holds information for specific rectangle on image
*/
class RegionContext {
    /// Rectangle on image
    rect_t rect_;
public:
    int iteration = 0;                      ///< Number of rendered samples per pixel
    std::unique_ptr<float[]> halton_seq;    ///< Sequense of random 2D points

    explicit RegionContext(const rect_t &rect) : rect_(rect) {}

    rect_t rect() const { return rect_; }

    /// Clear region context (used to start again)
    void Clear() {
        iteration = 0;
        halton_seq = nullptr;
    }
};

/** Base class for all renderer backends
*/
class RendererBase {
public:
    virtual ~RendererBase() = default;

    /// Type of renderer
    virtual eRendererType type() const = 0;

    /// Returns size of rendered image
    virtual std::pair<int, int> size() const = 0;

    /// Returns pointer to rendered image
    virtual const pixel_color_t *get_pixels_ref() const = 0;

    /// Returns pointer to SH data
    virtual const shl1_data_t *get_sh_data_ref() const = 0;

    /** @brief Resize framebuffer
        @param w new image width
        @param h new image height
    */
    virtual void Resize(int w, int h) = 0;

    /** @brief Clear framebuffer
        @param c color used to fill image
    */
    virtual void Clear(const pixel_color_t &c = { 0, 0, 0, 0 }) = 0;

    /** @brief Create new scene
        @return shared pointer to new scene for specific backend
    */
    virtual std::shared_ptr<SceneBase> CreateScene() = 0;

    /** @brief Render image region
        @param s shared pointer to a scene
        @param region image region to render
    */
    virtual void RenderScene(const std::shared_ptr<SceneBase> &s, RegionContext &region) = 0;

    struct stats_t {
        unsigned long long time_primary_ray_gen_us;
        unsigned long long time_primary_trace_us;
        unsigned long long time_primary_shade_us;
        unsigned long long time_secondary_sort_us;
        unsigned long long time_secondary_trace_us;
        unsigned long long time_secondary_shade_us;
    };
    virtual void GetStats(stats_t &st) = 0;
    virtual void ResetStats() = 0;
};
}