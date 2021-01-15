#pragma once

#include <condition_variable>
#include <future>
#include <mutex>
#include <thread>

#include <Eng/GameBase.h>
#include <Eng/GameState.h>
#include <Eng/Utils/VideoPlayer.h>
#include <Ren/Camera.h>
#include <Ren/MVec.h>
#include <Ren/Mesh.h>
#include <Ren/Program.h>
#include <Ren/SW/SW.h>
#include <Ren/Texture.h>

#include "GSBaseState.h"

class Cmdline;
class DebugInfoUI;
class GameStateManager;
class FontStorage;
class SceneManager;

namespace Gui {
class BaseElement;
class BitmapFont;
class Renderer;
} // namespace Gui

namespace Sys {
class ThreadPool;
class QThreadPool;
class ThreadWorker;
} // namespace Sys

class GSVideoTest final : public GSBaseState {
    std::shared_ptr<Sys::ThreadPool> threads_;
    std::shared_ptr<Sys::ThreadWorker> aux_gfx_thread_;

    int view_pointer_ = 0, move_pointer_ = 0;

    Ren::Vec3f initial_view_pos_ = Ren::Vec3f{0, 1, 0},
               initial_view_dir_ = Ren::Vec3f{0, 0, -1};

    Ren::Vec3f view_origin_, view_dir_;

    float fwd_press_speed_ = 0, side_press_speed_ = 0, fwd_touch_speed_ = 0,
          side_touch_speed_ = 0;

    float max_fwd_speed_ = 0.5f, view_fov_ = 60.0f;
    float max_exposure_ = 1000.0f;

    uint64_t click_time_ = 0;

    // test test
    uint32_t wall_picture_indices_[5] = {0xffffffff};

    bool enable_video_update_ = true;
    VideoPlayer vp_[5];
    VideoPlayer::eFrUpdateResult decode_result_[5] = {
        VideoPlayer::eFrUpdateResult::Reused, VideoPlayer::eFrUpdateResult::Reused,
        VideoPlayer::eFrUpdateResult::Reused, VideoPlayer::eFrUpdateResult::Reused,
        VideoPlayer::eFrUpdateResult::Reused,
    };

    std::future<void> vid_update_done_[5], tex_update_done_[5];
    std::shared_ptr<Sys::QThreadPool> decoder_threads_;

    uint64_t video_time_us_ = 0;

    // constant that controls maximum number of allowed in-flight frames
    static const int TextureSyncWindow = 2;

    uint32_t y_pbo_[5] = {}, uv_pbo_[5] = {};
    uint8_t *y_ptr_[5] = {}, *uv_ptr_[5] = {};
    int cur_frame_index_[5] = {};
    Ren::Tex2DRef y_tex_[5][TextureSyncWindow], uv_tex_[5][TextureSyncWindow];
    void *after_tex_update_fences_[5][TextureSyncWindow] = {};

    double frame_decode_time_ms_ = 0.0, tex_update_time_ms_ = 0.0;

    Ren::MaterialRef orig_vid_mat_[5], vid_mat_[5];

    void OnPreloadScene(JsObject &js_scene) override;
    void OnPostloadScene(JsObject &js_scene) override;

    void OnUpdateScene() override;

    void SaveScene(JsObject &js_scene) override;

    void DrawUI(Gui::Renderer *r, Gui::BaseElement *root) override;

    bool OpenVideoFiles();

    // Main thread procedures
    void InitVideoTextures();
    void UpdateVideoTextures();
    void DestroyVideoTextures();
    void UpdatePBOWithDecodedFrame(int tex_index, int frame_index);

    void SetVideoTextureFence(int tex_index, int frame_index);
    void InsertVideoTextureBarrier(int tex_index, int frame_index);
    void WaitVideoTextureUpdated(int tex_index, int frame_index);
    void UpdateVideoTextureData(int tex_index, int frame_index);
    static void FlushGPUCommands();

    // Decoder thread procedures
    void UpdatePBOWithDecodedFrame_Persistent(int tex_index, int frame_index);

  public:
    explicit GSVideoTest(GameBase *game);
    ~GSVideoTest() final = default;

    void Enter() override;
    void Exit() override;

    void Draw(uint64_t dt_us) override;

    void Update(uint64_t dt_us) override;

    bool HandleInput(const InputManager::Event &evt) override;
};