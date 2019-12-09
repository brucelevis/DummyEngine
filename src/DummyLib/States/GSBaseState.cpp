﻿#include "GSBaseState.h"

#include <fstream>
#include <memory>

#include <Eng/GameStateManager.h>
#include <Gui/Renderer.h>
#include <Ren/Context.h>
#include <Ren/GL.h>
#include <Ren/Utils.h>
#include <Sys/AssetFile.h>
#include <Sys/Json.h>
#include <Sys/Log.h>
#include <Sys/MemBuf.h>
#include <Sys/Time_.h>

#include "../Gui/DebugInfoUI.h"
#include "../Gui/FontStorage.h"
#include "../Viewer.h"
#include "../Renderer/Renderer.h"
#include "../Scene/SceneManager.h"
#include "../Utils/Cmdline.h"

namespace GSBaseStateInternal {
const int MAX_CMD_LINES = 8;

#if defined(__ANDROID__)
const char SCENE_NAME[] = "assets/scenes/"
#else
const char SCENE_NAME[] = "assets_pc/scenes/"
#endif
    //"font_test.json";
    //"skin_test.json";
    //"living_room_gumroad.json";
    //"bistro.json";
    "pbr_test.json";

const bool USE_TWO_THREADS = true;
}

GSBaseState::GSBaseState(GameBase *game) : game_(game) {
    using namespace GSBaseStateInternal;

    cmdline_        = game->GetComponent<Cmdline>(CMDLINE_KEY);

    state_manager_  = game->GetComponent<GameStateManager>(STATE_MANAGER_KEY);
    ctx_            = game->GetComponent<Ren::Context>(REN_CONTEXT_KEY);

    renderer_       = game->GetComponent<Renderer>(RENDERER_KEY);
    scene_manager_  = game->GetComponent<SceneManager>(SCENE_MANAGER_KEY);

    ui_renderer_    = game->GetComponent<Gui::Renderer>(UI_RENDERER_KEY);
    ui_root_        = game->GetComponent<Gui::BaseElement>(UI_ROOT_KEY);

    const std::shared_ptr<FontStorage> fonts = game->GetComponent<FontStorage>(UI_FONTS_KEY);
    font_ = fonts->FindFont("main_font");

    debug_ui_       = game->GetComponent<DebugInfoUI>(UI_DEBUG_KEY);

    swap_interval_  = game->GetComponent<TimeInterval>(SWAP_TIMER_KEY);

    // Prepare cam for probes updating
    temp_probe_cam_.Perspective(90.0f, 1.0f, 0.1f, 10000.0f);
}

void GSBaseState::Enter() {
    using namespace GSBaseStateInternal;

    if (USE_TWO_THREADS) {
        background_thread_ = std::thread(std::bind(&GSBaseState::BackgroundProc, this));
    }

    {   // Create temporary buffer to update probes
        FrameBuf::ColorAttachmentDesc desc;
        desc.format = Ren::RawRGB16F;
        desc.filter = Ren::NoFilter;
        desc.repeat = Ren::ClampToEdge;

        const int res = scene_manager_->scene_data().probe_storage.res();
        temp_probe_buf_ = FrameBuf(res, res, &desc, 1, { FrameBuf::DepthNone });
    }

    cmdline_history_.resize(MAX_CMD_LINES, "~");

    std::shared_ptr<GameStateManager> state_manager = state_manager_.lock();
    std::weak_ptr<GSBaseState> weak_this = std::dynamic_pointer_cast<GSBaseState>(state_manager->Peek());

    cmdline_->RegisterCommand("wireframe", [weak_this](int argc, Cmdline::ArgData *argv) -> bool {
        auto shrd_this = weak_this.lock();
        if (shrd_this) {
            uint32_t flags = shrd_this->renderer_->render_flags();
            flags ^= DebugWireframe;
            shrd_this->renderer_->set_render_flags(flags);
        }
        return true;
    });

    cmdline_->RegisterCommand("culling", [weak_this](int argc, Cmdline::ArgData *argv) -> bool {
        auto shrd_this = weak_this.lock();
        if (shrd_this) {
            uint32_t flags = shrd_this->renderer_->render_flags();
            flags ^= EnableCulling;
            shrd_this->renderer_->set_render_flags(flags);
        }
        return true;
    });

    cmdline_->RegisterCommand("lightmap", [weak_this](int argc, Cmdline::ArgData *argv) -> bool {
        auto shrd_this = weak_this.lock();
        if (shrd_this) {
            uint32_t flags = shrd_this->renderer_->render_flags();
            flags ^= EnableLightmap;
            shrd_this->renderer_->set_render_flags(flags);
        }
        return true;
    });

    cmdline_->RegisterCommand("lights", [weak_this](int argc, Cmdline::ArgData *argv) -> bool {
        auto shrd_this = weak_this.lock();
        if (shrd_this) {
            uint32_t flags = shrd_this->renderer_->render_flags();
            flags ^= EnableLights;
            shrd_this->renderer_->set_render_flags(flags);
        }
        return true;
    });

    cmdline_->RegisterCommand("decals", [weak_this](int argc, Cmdline::ArgData *argv) -> bool {
        auto shrd_this = weak_this.lock();
        if (shrd_this) {
            uint32_t flags = shrd_this->renderer_->render_flags();
            flags ^= EnableDecals;
            shrd_this->renderer_->set_render_flags(flags);
        }
        return true;
    });

    cmdline_->RegisterCommand("shadows", [weak_this](int argc, Cmdline::ArgData *argv) -> bool {
        auto shrd_this = weak_this.lock();
        if (shrd_this) {
            uint32_t flags = shrd_this->renderer_->render_flags();
            flags ^= EnableShadows;
            shrd_this->renderer_->set_render_flags(flags);
        }
        return true;
    });

    cmdline_->RegisterCommand("fxaa", [weak_this](int argc, Cmdline::ArgData *argv) -> bool {
        auto shrd_this = weak_this.lock();
        if (shrd_this) {
            uint32_t flags = shrd_this->renderer_->render_flags();
            flags ^= EnableFxaa;
            shrd_this->renderer_->set_render_flags(flags);
        }
        return true;
    });

    cmdline_->RegisterCommand("pt", [weak_this](int argc, Cmdline::ArgData *argv) ->bool {
        auto shrd_this = weak_this.lock();
        if (shrd_this) {
            shrd_this->use_pt_ = !shrd_this->use_pt_;
            if (shrd_this->use_pt_) {
                shrd_this->scene_manager_->InitScene_PT();
                shrd_this->invalidate_view_ = true;
            }
            
        }
        return true;
    });

    cmdline_->RegisterCommand("lm", [weak_this](int argc, Cmdline::ArgData *argv) ->bool {
        auto shrd_this = weak_this.lock();
        if (shrd_this) {
            shrd_this->use_lm_ = !shrd_this->use_lm_;
            if (shrd_this->use_lm_) {
                shrd_this->scene_manager_->InitScene_PT();
                shrd_this->scene_manager_->ResetLightmaps_PT();
                shrd_this->invalidate_view_ = true;
            }
        }
        return true;
    });

    cmdline_->RegisterCommand("oit", [weak_this](int argc, Cmdline::ArgData *argv) -> bool {
        auto shrd_this = weak_this.lock();
        if (shrd_this) {
            uint32_t flags = shrd_this->renderer_->render_flags();
            flags ^= EnableOIT;
            shrd_this->renderer_->set_render_flags(flags);
        }
        return true;
    });

    cmdline_->RegisterCommand("debug_cull", [weak_this](int argc, Cmdline::ArgData *argv) -> bool {
        auto shrd_this = weak_this.lock();
        if (shrd_this) {
            uint32_t flags = shrd_this->renderer_->render_flags();
            flags ^= DebugCulling;
            shrd_this->renderer_->set_render_flags(flags);
        }
        return true;
    });

    cmdline_->RegisterCommand("debug_shadow", [weak_this](int argc, Cmdline::ArgData *argv) -> bool {
        auto shrd_this = weak_this.lock();
        if (shrd_this) {
            uint32_t flags = shrd_this->renderer_->render_flags();
            flags ^= DebugShadow;
            shrd_this->renderer_->set_render_flags(flags);
        }
        return true;
    });

    cmdline_->RegisterCommand("debug_reduce", [weak_this](int argc, Cmdline::ArgData *argv) -> bool {
        auto shrd_this = weak_this.lock();
        if (shrd_this) {
            uint32_t flags = shrd_this->renderer_->render_flags();
            flags ^= DebugReduce;
            shrd_this->renderer_->set_render_flags(flags);
        }
        return true;
    });

    cmdline_->RegisterCommand("debug_lights", [weak_this](int argc, Cmdline::ArgData *argv) -> bool {
        auto shrd_this = weak_this.lock();
        if (shrd_this) {
            uint32_t flags = shrd_this->renderer_->render_flags();
            flags ^= DebugLights;
            shrd_this->renderer_->set_render_flags(flags);
        }
        return true;
    });

    cmdline_->RegisterCommand("debug_decals", [weak_this](int argc, Cmdline::ArgData *argv) -> bool {
        auto shrd_this = weak_this.lock();
        if (shrd_this) {
            uint32_t flags = shrd_this->renderer_->render_flags();
            flags ^= DebugDecals;
            shrd_this->renderer_->set_render_flags(flags);
        }
        return true;
    });

    cmdline_->RegisterCommand("debug_deferred", [weak_this](int argc, Cmdline::ArgData *argv) -> bool {
        auto shrd_this = weak_this.lock();
        if (shrd_this) {
            uint32_t flags = shrd_this->renderer_->render_flags();
            flags ^= DebugDeferred;
            shrd_this->renderer_->set_render_flags(flags);
        }
        return true;
    });

    cmdline_->RegisterCommand("debug_blur", [weak_this](int argc, Cmdline::ArgData *argv) -> bool {
        auto shrd_this = weak_this.lock();
        if (shrd_this) {
            uint32_t flags = shrd_this->renderer_->render_flags();
            flags ^= DebugBlur;
            shrd_this->renderer_->set_render_flags(flags);
        }
        return true;
    });

    cmdline_->RegisterCommand("debug_ssao", [weak_this](int argc, Cmdline::ArgData *argv) -> bool {
        auto shrd_this = weak_this.lock();
        if (shrd_this) {
            uint32_t flags = shrd_this->renderer_->render_flags();
            flags ^= DebugSSAO;
            shrd_this->renderer_->set_render_flags(flags);
        }
        return true;
    });

    cmdline_->RegisterCommand("debug_timings", [weak_this](int argc, Cmdline::ArgData *argv) -> bool {
        auto shrd_this = weak_this.lock();
        if (shrd_this) {
            uint32_t flags = shrd_this->renderer_->render_flags();
            flags ^= DebugTimings;
            shrd_this->renderer_->set_render_flags(flags);
        }
        return true;
    });

    cmdline_->RegisterCommand("debug_bvh", [weak_this](int argc, Cmdline::ArgData *argv) -> bool {
        auto shrd_this = weak_this.lock();
        if (shrd_this) {
            uint32_t flags = shrd_this->renderer_->render_flags();
            flags ^= DebugBVH;
            shrd_this->renderer_->set_render_flags(flags);
        }
        return true;
    });

    cmdline_->RegisterCommand("debug_probes", [weak_this](int argc, Cmdline::ArgData *argv) -> bool {
        auto shrd_this = weak_this.lock();
        if (shrd_this) {
            uint32_t flags = shrd_this->renderer_->render_flags();
            flags ^= DebugProbes;
            shrd_this->renderer_->set_render_flags(flags);
        }
        return true;
    });
}

bool GSBaseState::LoadScene(const char *name) {
    JsObject js_scene;

    {   // Load scene data from file
        Sys::AssetFile in_scene(name);
        if (!in_scene) {
            LOGE("Can not open scene file %s", name);
            return false;
        }

        size_t scene_size = in_scene.size();

        std::unique_ptr<uint8_t[]> scene_data(new uint8_t[scene_size]);
        in_scene.Read((char *)&scene_data[0], scene_size);

        Sys::MemBuf mem(&scene_data[0], scene_size);
        std::istream in_stream(&mem);

        if (!js_scene.Read(in_stream)) {
            throw std::runtime_error("Cannot load scene!");
        }
    }

    OnPreloadScene(js_scene);

    try {
        scene_manager_->LoadScene(js_scene);
    } catch (std::exception &e) {
        LOGI("Error loading scene: %s", e.what());
    }

    OnPostloadScene(js_scene);

    return true;
}

void GSBaseState::OnPreloadScene(JsObject &js_scene) {
    
}

void GSBaseState::OnPostloadScene(JsObject &js_scene) {
    // trigger probes update
    probes_dirty_ = true;
}

void GSBaseState::Exit() {
    using namespace GSBaseStateInternal;

    if (USE_TWO_THREADS) {
        if (background_thread_.joinable()) {
            shutdown_ = notified_ = true;
            thr_notify_.notify_all();
            background_thread_.join();
        }
    }
}

void GSBaseState::Draw(uint64_t dt_us) {
    using namespace GSBaseStateInternal;

    if (cmdline_enabled_) {
        // Process comandline input
        for (const InputManager::Event &evt : cmdline_input_) {
            if (evt.key == InputManager::RAW_INPUT_BUTTON_BACKSPACE) {
                if (!cmdline_history_.back().empty()) {
                    cmdline_history_.back().pop_back();
                }

            } else if (evt.key == InputManager::RAW_INPUT_BUTTON_RETURN) {
                cmdline_->Execute(cmdline_history_.back().c_str());

                cmdline_history_.emplace_back();
                if (cmdline_history_.size() > MAX_CMD_LINES) {
                    cmdline_history_.erase(cmdline_history_.begin());
                }
            } else if (evt.raw_key == (int)'`') {
                if (!cmdline_history_.back().empty()) {
                    cmdline_history_.emplace_back();
                    if (cmdline_history_.size() > MAX_CMD_LINES) {
                        cmdline_history_.erase(cmdline_history_.begin());
                    }
                }
            } else {
                char ch = (char)evt.raw_key;
                if (shift_down_) {
                    if (ch == '-') ch = '_';
                }
                cmdline_history_.back() += ch;
            }
        }

        cmdline_input_.clear();
    }

    {
        int back_list;

        if (USE_TWO_THREADS) {
            std::unique_lock<std::mutex> lock(mtx_);
            while (notified_) {
                thr_done_.wait(lock);
            }

            if (use_lm_) {
                int w, h;
                const float *preview_pixels = nullptr;
                if (scene_manager_->PrepareLightmaps_PT(&preview_pixels, &w, &h)) {
                    if (preview_pixels) {
                        renderer_->BlitPixels(preview_pixels, w, h, Ren::RawRGBA32F);
                    }
                } else {
                    // Lightmap creation finished, convert textures
                    Viewer::PrepareAssets("pc");
                    // Reload scene
                    LoadScene(SCENE_NAME);
                    // Switch back to normal mode
                    use_lm_ = false;
                }

                back_list = -1;
            } else if (use_pt_) {
                // TODO: fix pt view setup (use current camera)
                //scene_manager_->SetupView_PT(view_origin_, (view_origin_ + view_dir_), Ren::Vec3f{ 0.0f, 1.0f, 0.0f }, view_fov_);
                if (invalidate_view_) {
                    scene_manager_->Clear_PT();
                    invalidate_view_ = false;
                }
                int w, h;
                const float *preview_pixels = scene_manager_->Draw_PT(&w, &h);
                if (preview_pixels) {
                    renderer_->BlitPixelsTonemap(preview_pixels, w, h, Ren::RawRGBA32F);
                }

                back_list = -1;
            } else {
                back_list = front_list_;
                front_list_ = (front_list_ + 1) % 2;

                if (probes_dirty_ && scene_manager_->load_complete()) {
                    // Perform first update of reflection probes
                    update_all_probes_ = true;
                    probes_dirty_ = false;
                }

                const SceneData &scene_data = scene_manager_->scene_data();

                if (probe_to_update_sh_) {
                    bool done = renderer_->BlitProjectSH(
                        scene_data.probe_storage, probe_to_update_sh_->layer_index,
                        probe_sh_update_iteration_, *probe_to_update_sh_);
                    probe_sh_update_iteration_++;

                    if (done) {
                        probe_sh_update_iteration_ = 0;
                        probe_to_update_sh_ = nullptr;
                    }
                }

                // Render probe cubemap
                if (probe_to_render_) {
                    for (int i = 0; i < 6; i++) {
                        renderer_->ExecuteDrawList(temp_probe_lists_[i], &temp_probe_buf_);
                        renderer_->BlitToTempProbeFace(temp_probe_buf_, scene_data.probe_storage, i);
                    }

                    renderer_->BlitPrefilterFromTemp(scene_data.probe_storage, probe_to_render_->layer_index);

                    probe_to_update_sh_ = probe_to_render_;
                    probe_to_render_ = nullptr;
                }
            }

            notified_ = true;
            thr_notify_.notify_one();
        } else {
            //scene_manager_->SetupView(view_origin_, (view_origin_ + view_dir_), Ren::Vec3f{ 0.0f, 1.0f, 0.0f }, view_fov_);
            // Gather drawables for list 0
            UpdateFrame(0);
            back_list = 0;
        }

        if (back_list != -1) {
            // Render current frame (from back list)
            renderer_->ExecuteDrawList(main_view_lists_[back_list]);
        }
    }

    //LOGI("{ %.7f, %.7f, %.7f } { %.7f, %.7f, %.7f }", view_origin_[0], view_origin_[1], view_origin_[2],
    //                                                  view_dir_[0], view_dir_[1], view_dir_[2]);

    {   // ui draw
        ui_renderer_->BeginDraw();

        DrawUI(ui_renderer_.get(), ui_root_.get());

        ui_renderer_->EndDraw();
    }

    ctx_->ProcessTasks();
}

void GSBaseState::DrawUI(Gui::Renderer *r, Gui::BaseElement *root) {
    using namespace GSBaseStateInternal;

    const float font_height = font_->height(root);

    if (cmdline_enabled_) {
        int ifont_height = (int)(0.5f * font_->height(root) * (float)game_->height);
#if defined(USE_GL_RENDER)
        glEnable(GL_SCISSOR_TEST);
        glScissor(0, game_->height - MAX_CMD_LINES * ifont_height - 2, game_->width, MAX_CMD_LINES * ifont_height + 2);

        glClearColor(0, 0.5f, 0.5f, 1);
        glClear(GL_COLOR_BUFFER_BIT);
        glDisable(GL_SCISSOR_TEST);
#endif
        float cur_y = 1.0f - font_->height(root);

        for (const std::string &cmd : cmdline_history_) {
            font_->DrawText(r, cmd.c_str(), { -1, cur_y }, root);
            cur_y -= font_height;
        }
    }

    if (!use_pt_ && !use_lm_) {
        int back_list = (front_list_ + 1) % 2;

        uint32_t render_flags = renderer_->render_flags();
        FrontendInfo front_info = main_view_lists_[back_list].frontend_info;
        BackendInfo back_info = renderer_->backend_info();

        uint64_t front_dur = front_info.end_timepoint_us - front_info.start_timepoint_us,
            back_dur = back_info.cpu_end_timepoint_us - back_info.cpu_start_timepoint_us;

        LOGI("Frontend: %04lld\tBackend(cpu): %04lld", (long long)front_dur, (long long)back_dur);

        //uint64_t transp_dur = back_info.gpu_end_timepoint_us - back_info.gpu_start_timepoint_us;// back_info.opaque_pass_time_us + back_info.transp_pass_time_us;
        //LOGI("Transparent draw: %04i us", (int)transp_dur);

        ItemsInfo items_info;
        items_info.light_sources_count = main_view_lists_[back_list].light_sources.count;
        items_info.decals_count = main_view_lists_[back_list].decals.count;
        items_info.probes_count = main_view_lists_[back_list].probes.count;
        items_info.items_total = main_view_lists_[back_list].items.count;

        debug_ui_->UpdateInfo(front_info, back_info, items_info, *swap_interval_, render_flags);
        debug_ui_->Draw(r);
    }
}

void GSBaseState::Update(uint64_t dt_us) {
    
}

bool GSBaseState::HandleInput(const InputManager::Event &evt) {
    using namespace Ren;
    using namespace GSBaseStateInternal;

    switch (evt.type) {
    case InputManager::RAW_INPUT_P1_DOWN: {
    } break;
    case InputManager::RAW_INPUT_P2_DOWN: {
    } break;
    case InputManager::RAW_INPUT_P1_UP: {
    } break;
    case InputManager::RAW_INPUT_P2_UP: {
    } break;
    case InputManager::RAW_INPUT_P1_MOVE: {
    } break;
    case InputManager::RAW_INPUT_P2_MOVE: {
    } break;
    case InputManager::RAW_INPUT_KEY_DOWN: {
    } break;
    case InputManager::RAW_INPUT_KEY_UP: {
        if (evt.key == InputManager::RAW_INPUT_BUTTON_SHIFT) {
            shift_down_ = false;
        } else if (evt.key == InputManager::RAW_INPUT_BUTTON_BACKSPACE) {
            if (cmdline_enabled_) {
                cmdline_input_.push_back(evt);
            }
        } else if (evt.key == InputManager::RAW_INPUT_BUTTON_RETURN) {
            if (cmdline_enabled_) {
                cmdline_input_.push_back(evt);
            }
        } else if (evt.raw_key == (int)'`') {
            cmdline_enabled_ = !cmdline_enabled_;
            if (cmdline_enabled_) {
                cmdline_input_.push_back(evt);
            }
        } else if (cmdline_enabled_) {
            cmdline_input_.push_back(evt);
        }
    }
    case InputManager::RAW_INPUT_RESIZE:
        break;
    default:
        break;
    }

    return true;
}

void GSBaseState::BackgroundProc() {
    std::unique_lock<std::mutex> lock(mtx_);
    while (!shutdown_) {
        while (!notified_) {
            thr_notify_.wait(lock);
        }

        // Gather drawables for list 1
        UpdateFrame(front_list_);

        notified_ = false;
        thr_done_.notify_one();
    }
}

void GSBaseState::UpdateFrame(int list_index) {
    {   // Update loop with fixed timestep
        auto input_manager = game_->GetComponent<InputManager>(INPUT_MANAGER_KEY);

        FrameInfo &fr = fr_info_;

        fr.cur_time_us = Sys::GetTimeUs();
        if (fr.cur_time_us < fr.prev_time_us) fr.prev_time_us = 0;
        fr.delta_time_us = fr.cur_time_us - fr.prev_time_us;
        if (fr.delta_time_us > 200000) {
            fr.delta_time_us = 200000;
        }
        fr.prev_time_us = fr.cur_time_us;
        fr.time_acc_us += fr.delta_time_us;

        uint64_t poll_time_point = fr.cur_time_us - fr.time_acc_us;

        while (fr.time_acc_us >= UPDATE_DELTA) {
            InputManager::Event evt;
            while (input_manager->PollEvent(poll_time_point, evt)) {
                this->HandleInput(evt);
            }

            this->Update(UPDATE_DELTA);
            fr.time_acc_us -= UPDATE_DELTA;

            poll_time_point += UPDATE_DELTA;
        }

        fr.time_fract = double(fr.time_acc_us) / UPDATE_DELTA;
    }

    OnUpdateScene();

    // Update invalidated objects
    scene_manager_->UpdateObjects();

    if (!use_pt_ && !use_lm_) {
        if (update_all_probes_) {
            if (probes_to_update_.empty()) {
                int obj_count = (int)scene_manager_->scene_data().objects.size();
                for (int i = 0; i < obj_count; i++) {
                    SceneObject *obj = scene_manager_->GetObject(i);
                    if (obj->comp_mask & CompProbeBit) {
                        probes_to_update_.push_back(i);
                    }
                }
            }
            update_all_probes_ = false;
        }

        if (!probes_to_update_.empty() && !probe_to_render_ && !probe_to_update_sh_) {
            LOGI("Updating probe");
            SceneObject *probe_obj = scene_manager_->GetObject(probes_to_update_.back());
            auto *probe = (LightProbe *)scene_manager_->scene_data().comp_store[CompProbe]->Get(probe_obj->components[CompProbe]);
            auto *probe_tr = (Transform *)scene_manager_->scene_data().comp_store[CompTransform]->Get(probe_obj->components[CompTransform]);

            Ren::Vec4f pos = { probe->offset[0], probe->offset[1], probe->offset[2], 1.0f };
            pos = probe_tr->mat * pos;
            pos /= pos[3];

            static const Ren::Vec3f axises[] = {
                { 1.0f,  0.0f,  0.0f },
                { -1.0f,  0.0f,  0.0f },
                { 0.0f,  1.0f,  0.0f },
                { 0.0f, -1.0f,  0.0f },
                { 0.0f,  0.0f,  1.0f },
                { 0.0f,  0.0f, -1.0f }
            };

            static const Ren::Vec3f ups[] = {
                { 0.0f, -1.0f, 0.0f },
                { 0.0f, -1.0f, 0.0f },
                { 0.0f,  0.0f, 1.0f },
                { 0.0f,  0.0f, -1.0f },
                { 0.0f, -1.0f, 0.0f },
                { 0.0f, -1.0f, 0.0f }
            };

            const Ren::Vec3f center = { pos[0], pos[1], pos[2] };

            for (int i = 0; i < 6; i++) {
                const Ren::Vec3f target = center + axises[i];
                temp_probe_cam_.SetupView(center, target, ups[i]);
                temp_probe_cam_.UpdatePlanes();

                temp_probe_lists_[i].render_flags = EnableZFill | EnableCulling | EnableLightmap | EnableLights | EnableDecals | EnableShadows | EnableProbes;

                renderer_->PrepareDrawList(scene_manager_->scene_data(), temp_probe_cam_, temp_probe_lists_[i]);
            }

            probe_to_render_ = probe;
            probes_to_update_.pop_back();
        }
        // Enable all flags, Renderer will mask out what is not enabled
        main_view_lists_[list_index].render_flags = 0xffffffff;

        renderer_->PrepareDrawList(scene_manager_->scene_data(), scene_manager_->main_cam(), main_view_lists_[list_index]);
    }
}