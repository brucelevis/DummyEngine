﻿#include "GSUITest.h"

#include <fstream>
#include <memory>

#include <Eng/GameStateManager.h>
#include <Eng/Gui/Image.h>
#include <Eng/Gui/Image9Patch.h>
#include <Eng/Gui/Renderer.h>
#include <Eng/Gui/Utils.h>
#include <Eng/Renderer/Renderer.h>
#include <Eng/Scene/SceneManager.h>
#include <Eng/Utils/Cmdline.h>
#include <Ren/Context.h>
#include <Ren/GL.h>
#include <Ren/Utils.h>
#include <Sys/AssetFile.h>
#include <Sys/Json.h>
#include <Sys/MemBuf.h>
#include <Sys/Time_.h>

#include "../Gui/FontStorage.h"
#include "../Gui/WordPuzzleUI.h"
#include "../Viewer.h"

namespace GSUITestInternal {
#if defined(__ANDROID__)
const char SCENE_NAME[] = "assets/scenes/"
#else
const char SCENE_NAME[] = "assets_pc/scenes/"
#endif
                          "corridor.json";
} // namespace GSUITestInternal

GSUITest::GSUITest(GameBase *game) : GSBaseState(game) {
    const std::shared_ptr<FontStorage> fonts =
        game->GetComponent<FontStorage>(UI_FONTS_KEY);
    dialog_font_ = fonts->FindFont("dialog_font");
    dialog_font_->set_scale(1.5f);

    word_puzzle_.reset(new WordPuzzleUI{*ren_ctx_, Ren::Vec2f{-0.995f, -0.995f},
                                        Ren::Vec2f{1.99f, 1.1f}, ui_root_.get(),
                                        *dialog_font_});
}

GSUITest::~GSUITest() = default;

void GSUITest::Enter() {
    using namespace GSUITestInternal;

    GSBaseState::Enter();

    log_->Info("GSUITest: Loading scene!");
    //GSBaseState::LoadScene(SCENE_NAME);

#if defined(__ANDROID__)
    const char *dialog_name = "assets/scenes/test/test_puzzle.json";
#else
    const char *dialog_name = "assets_pc/scenes/test/test_puzzle.json";
#endif
    JsObject js_script;

    { // Load dialog data from file
        Sys::AssetFile in_scene(dialog_name);
        if (!in_scene) {
            log_->Error("Can not open dialog file %s", dialog_name);
        }

        const size_t scene_size = in_scene.size();

        std::unique_ptr<uint8_t[]> scene_data(new uint8_t[scene_size]);
        in_scene.Read((char *)&scene_data[0], scene_size);

        Sys::MemBuf mem(&scene_data[0], scene_size);
        std::istream in_stream(&mem);

        if (!js_script.Read(in_stream)) {
            throw std::runtime_error("Cannot load dialog!");
        }
    }

    word_puzzle_->Load(js_script);
    word_puzzle_->Restart();
}

void GSUITest::OnPostloadScene(JsObject &js_scene) {
    using namespace GSUITestInternal;

    GSBaseState::OnPostloadScene(js_scene);

    Ren::Vec3f view_origin, view_dir = Ren::Vec3f{0.0f, 0.0f, 1.0f};
    float view_fov = 45.0f, max_exposure = 1000.0f;

    if (js_scene.Has("camera")) {
        const JsObject &js_cam = js_scene.at("camera").as_obj();
        if (js_cam.Has("view_origin")) {
            const JsArray &js_orig = js_cam.at("view_origin").as_arr();
            view_origin[0] = float(js_orig.at(0).as_num().val);
            view_origin[1] = float(js_orig.at(1).as_num().val);
            view_origin[2] = float(js_orig.at(2).as_num().val);
        }

        if (js_cam.Has("view_dir")) {
            const JsArray &js_dir = js_cam.at("view_dir").as_arr();
            view_dir[0] = float(js_dir.at(0).as_num().val);
            view_dir[1] = float(js_dir.at(1).as_num().val);
            view_dir[2] = float(js_dir.at(2).as_num().val);
        }

        /*if (js_cam.Has("fwd_speed")) {
            const JsNumber &js_fwd_speed = (const JsNumber &)js_cam.at("fwd_speed");
            max_fwd_speed_ = (float)js_fwd_speed.val;
        }*/

        if (js_cam.Has("fov")) {
            const JsNumber &js_fov = js_cam.at("fov").as_num();
            view_fov = float(js_fov.val);
        }

        if (js_cam.Has("max_exposure")) {
            const JsNumber &js_max_exposure = js_cam.at("max_exposure").as_num();
            max_exposure = float(js_max_exposure.val);
        }
    }

    scene_manager_->SetupView(view_origin, (view_origin + view_dir),
                              Ren::Vec3f{0.0f, 1.0f, 0.0f}, view_fov, max_exposure);

    {
        char sophia_name[] = "sophia_00";

        for (int i = 0; i < 2; i++) {
            sophia_name[8] = char('0' + i);

            uint32_t sophia_index = scene_manager_->FindObject(sophia_name);
            sophia_indices_[i] = sophia_index;
        }
    }
}

void GSUITest::OnUpdateScene() {
    using namespace GSUITestInternal;

    GSBaseState::OnUpdateScene();

    const float delta_time_s = fr_info_.delta_time_us * 0.000001f;
    test_time_counter_s += delta_time_s;

    const float char_period_s = 0.025f;

    while (test_time_counter_s > char_period_s) {
        //word_puzzle_->incr_progress();
        test_time_counter_s -= char_period_s;
    }

    const SceneData &scene = scene_manager_->scene_data();

    if (sophia_indices_[0] != 0xffffffff) {
        for (int i = 0; i < 2; i++) { // NOLINT
            if (sophia_indices_[i] == 0xffffffff) {
                break;
            }

            SceneObject *sophia = scene_manager_->GetObject(sophia_indices_[i]);

            uint32_t mask = CompDrawableBit | CompAnimStateBit;
            if ((sophia->comp_mask & mask) == mask) {
                auto *dr = (Drawable *)scene.comp_store[CompDrawable]->Get(
                    sophia->components[CompDrawable]);
                auto *as = (AnimState *)scene.comp_store[CompAnimState]->Get(
                    sophia->components[CompAnimState]);

                // keep previous palette for velocity calculation
                std::swap(as->matr_palette_curr, as->matr_palette_prev);
                as->anim_time_s += delta_time_s;

                Ren::Mesh *mesh = dr->mesh.get();
                Ren::Skeleton *skel = mesh->skel();

                const int anim_index = 0;

                skel->UpdateAnim(anim_index, as->anim_time_s);
                skel->ApplyAnim(anim_index);
                skel->UpdateBones(&as->matr_palette_curr[0]);
            }
        }
    }
}

void GSUITest::Exit() { GSBaseState::Exit(); }

void GSUITest::DrawUI(Gui::Renderer *r, Gui::BaseElement *root) {
    using namespace GSUITestInternal;

    GSBaseState::DrawUI(r, root);

    dialog_font_->set_scale(std::max(root->size_px()[0] / 1024.0f, 1.0f));

    word_puzzle_->Draw(r);
}

bool GSUITest::HandleInput(const InputManager::Event &evt) {
    using namespace Ren;
    using namespace GSUITestInternal;

    // pt switch for touch controls
    if (evt.type == RawInputEvent::EvP1Down || evt.type == RawInputEvent::EvP2Down) {
        if (evt.point.x > float(ren_ctx_->w()) * 0.9f &&
            evt.point.y < float(ren_ctx_->h()) * 0.1f) {
            const uint64_t new_time = Sys::GetTimeMs();
            if (new_time - click_time_ < 400) {
                use_pt_ = !use_pt_;
                if (use_pt_) {
                    scene_manager_->InitScene_PT();
                    invalidate_view_ = true;
                }

                click_time_ = 0;
            } else {
                click_time_ = new_time;
            }
        }
    }

    bool input_processed = true;

    switch (evt.type) {
    case RawInputEvent::EvP1Down: {
        Ren::Vec2f p =
            Gui::MapPointToScreen(Ren::Vec2i{int(evt.point.x), int(evt.point.y)},
                                  Ren::Vec2i{ren_ctx_->w(), ren_ctx_->h()});
        word_puzzle_->Press(p, true);
    } break;
    case RawInputEvent::EvP2Down: {

    } break;
    case RawInputEvent::EvP1Up: {
        const Ren::Vec2f p =
            Gui::MapPointToScreen(Ren::Vec2i{int(evt.point.x), int(evt.point.y)},
                                  Ren::Vec2i{ren_ctx_->w(), ren_ctx_->h()});
        word_puzzle_->Press(p, false);
    } break;
    case RawInputEvent::EvP2Up: {

    } break;
    case RawInputEvent::EvP1Move: {
        Ren::Vec2f p =
            Gui::MapPointToScreen(Ren::Vec2i{int(evt.point.x), int(evt.point.y)},
                                  Ren::Vec2i{ren_ctx_->w(), ren_ctx_->h()});
        word_puzzle_->Hover(p);
    } break;
    case RawInputEvent::EvP2Move: {

    } break;
    case RawInputEvent::EvKeyDown: {
        input_processed = false;
    } break;
    case RawInputEvent::EvKeyUp: {
        if (evt.key_code == KeyUp || (evt.key_code == KeyW && !cmdline_enabled_)) {
            //word_puzzle_->restart();
        } else {
            input_processed = false;
        }
    } break;
    case RawInputEvent::EvResize:
        word_puzzle_->Resize(ui_root_.get());
        break;
    default:
        break;
    }

    if (!input_processed) {
        GSBaseState::HandleInput(evt);
    }

    return true;
}
