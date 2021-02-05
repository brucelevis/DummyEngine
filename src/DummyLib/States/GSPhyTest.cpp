#include "GSPhyTest.h"

#include <memory>

#include <Eng/Gui/Renderer.h>
#include <Eng/Renderer/Renderer.h>
#include <Eng/Scene/SceneManager.h>
#include <Eng/Utils/Cmdline.h>
#include <Eng/Utils/ShaderLoader.h>
#include <Ren/Context.h>
#include <Ren/GL.h>
#include <Ren/Utils.h>
#include <Sys/AssetFile.h>
#include <Sys/Json.h>
#include <Sys/MemBuf.h>
#include <Sys/Time_.h>

#include "../Gui/FontStorage.h"
#include "../Viewer.h"

namespace GSPhyTestInternal {
#if defined(__ANDROID__)
const char SCENE_NAME[] = "assets/scenes/"
#else
const char SCENE_NAME[] = "assets_pc/scenes/"
#endif
                          "physics_test.json";
} // namespace GSPhyTestInternal

GSPhyTest::GSPhyTest(GameBase *game) : GSBaseState(game) {}

void GSPhyTest::Enter() {
    using namespace GSPhyTestInternal;

    GSBaseState::Enter();

    log_->Info("GSPhyTest: Loading scene!");
    GSBaseState::LoadScene(SCENE_NAME);
}

void GSPhyTest::OnPreloadScene(JsObject &js_scene) {
    GSBaseState::OnPreloadScene(js_scene);

    JsArray &js_objects = js_scene.at("objects").as_arr();

    // Dynamic spheres
    for (int x = 0; x < 6; x++) {
        for (int z = 0; z < 6; z++) {
            const double Radius = 0.5;
            const double xx = double(x - 1) * Radius * 1.5;
            const double zz = double(z - 1) * Radius * 1.5;

            JsObject new_object;

            { // Add empty transform component
                JsObject js_transform;
                { // scale unit sphere to radius
                    JsArray js_scale;
                    js_scale.Push(JsNumber{Radius});
                    js_scale.Push(JsNumber{Radius});
                    js_scale.Push(JsNumber{Radius});
                    js_transform.Push("scale", std::move(js_scale));
                }
                new_object.Push(Transform::name(), std::move(js_transform));
            }

            { // Add physics component
                JsObject js_physics;

                { // position
                    JsArray js_pos;
                    js_pos.Push(JsNumber{xx});
                    js_pos.Push(JsNumber{10.0});
                    js_pos.Push(JsNumber{zz});
                    js_physics.Push("pos", std::move(js_pos));
                }

                js_physics.Push("inv_mass", JsNumber{1.0});
                js_physics.Push("elasticity", JsNumber{0.0});
                js_physics.Push("friction", JsNumber{0.5});

                { // shape
                    JsObject js_shape;
                    js_shape.Push("type", JsString{"sphere"});
                    js_shape.Push("radius", JsNumber{Radius});
                    js_physics.Push("shape", std::move(js_shape));
                }

                new_object.Push(Physics::name(), std::move(js_physics));
            }

            { // Add drawable component
                JsObject js_drawable;
                js_drawable.Push("mesh_file", JsString{"sphere_hi.mesh"});
                js_drawable.Push("visible_to_probes", JsLiteral{JsLiteralType::False});
                new_object.Push(Drawable::name(), std::move(js_drawable));
            }

            js_objects.Push(std::move(new_object));
        }
    }

    // Static spheres
    for (int x = 0; x < 3; x++) {
        for (int z = 0; z < 3; z++) {
            const double Radius = 80;
            const double xx = double(x - 1) * Radius * 0.25;
            const double zz = double(z - 1) * Radius * 0.25;

            JsObject new_object;

            { // Add empty transform component
                JsObject js_transform;
                { // scale unit sphere to radius
                    JsArray js_scale;
                    js_scale.Push(JsNumber{Radius});
                    js_scale.Push(JsNumber{Radius});
                    js_scale.Push(JsNumber{Radius});
                    js_transform.Push("scale", std::move(js_scale));
                }
                new_object.Push(Transform::name(), std::move(js_transform));
            }

            { // Add physics component
                JsObject js_physics;

                { // position
                    JsArray js_pos;
                    js_pos.Push(JsNumber{xx});
                    js_pos.Push(JsNumber{-Radius});
                    js_pos.Push(JsNumber{zz});
                    js_physics.Push("pos", std::move(js_pos));
                }

                js_physics.Push("inv_mass", JsNumber{0.0});
                js_physics.Push("elasticity", JsNumber{0.99});
                js_physics.Push("friction", JsNumber{0.5});

                { // shape
                    JsObject js_shape;
                    js_shape.Push("type", JsString{"sphere"});
                    js_shape.Push("radius", JsNumber{Radius});
                    js_physics.Push("shape", std::move(js_shape));
                }

                new_object.Push(Physics::name(), std::move(js_physics));
            }

            { // Add drawable component
                JsObject js_drawable;
                js_drawable.Push("mesh_file", JsString{"sphere_hi.mesh"});
                js_drawable.Push("visible_to_probes", JsLiteral{JsLiteralType::False});

                { // Set material to checkerboard
                    JsArray js_material_override;
                    js_material_override.Push(JsString{"checker.txt"});
                    js_drawable.Push("material_override",
                                     std::move(js_material_override));
                }

                new_object.Push(Drawable::name(), std::move(js_drawable));
            }

            js_objects.Push(std::move(new_object));
        }
    }
}

void GSPhyTest::OnPostloadScene(JsObject &js_scene) {
    GSBaseState::OnPostloadScene(js_scene);

    if (js_scene.Has("camera")) {
        const JsObject &js_cam = js_scene.at("camera").as_obj();
        if (js_cam.Has("view_origin")) {
            const JsArray &js_orig = js_cam.at("view_origin").as_arr();
            initial_view_pos_[0] = float(js_orig.at(0).as_num().val);
            initial_view_pos_[1] = float(js_orig.at(1).as_num().val);
            initial_view_pos_[2] = float(js_orig.at(2).as_num().val);
        }

        if (js_cam.Has("view_dir")) {
            const JsArray &js_dir = js_cam.at("view_dir").as_arr();
            initial_view_dir_[0] = float(js_dir.at(0).as_num().val);
            initial_view_dir_[1] = float(js_dir.at(1).as_num().val);
            initial_view_dir_[2] = float(js_dir.at(2).as_num().val);
        }

        if (js_cam.Has("fwd_speed")) {
            const JsNumber &js_fwd_speed = js_cam.at("fwd_speed").as_num();
            max_fwd_speed_ = float(js_fwd_speed.val);
        }

        if (js_cam.Has("fov")) {
            const JsNumber &js_fov = js_cam.at("fov").as_num();
            view_fov_ = float(js_fov.val);
        }

        if (js_cam.Has("max_exposure")) {
            const JsNumber &js_max_exposure = js_cam.at("max_exposure").as_num();
            max_exposure_ = float(js_max_exposure.val);
        }
    }

    view_origin_ = initial_view_pos_;
    view_dir_ = initial_view_dir_;
}

void GSPhyTest::Exit() { GSBaseState::Exit(); }

void GSPhyTest::Draw(const uint64_t dt_us) { GSBaseState::Draw(dt_us); }

void GSPhyTest::DrawUI(Gui::Renderer *r, Gui::BaseElement *root) {
    GSBaseState::DrawUI(r, root);
}

void GSPhyTest::Update(const uint64_t dt_us) {
    using namespace GSPhyTestInternal;

    GSBaseState::Update(dt_us);

    const Ren::Vec3f up = Ren::Vec3f{0, 1, 0}, side = Normalize(Cross(view_dir_, up));

    {
        const float fwd_speed = std::max(
                        std::min(fwd_press_speed_ + fwd_touch_speed_, max_fwd_speed_),
                        -max_fwd_speed_),
                    side_speed = std::max(
                        std::min(side_press_speed_ + side_touch_speed_, max_fwd_speed_),
                        -max_fwd_speed_);

        view_origin_ += view_dir_ * fwd_speed;
        view_origin_ += side * side_speed;

        if (std::abs(fwd_speed) > 0.0f || std::abs(side_speed) > 0.0f) {
            invalidate_view_ = true;
        }
    }
}

bool GSPhyTest::HandleInput(const InputManager::Event &evt) {
    using namespace Ren;
    using namespace GSPhyTestInternal;

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
    case RawInputEvent::EvP1Down:
        if (evt.point.x < (float(ren_ctx_->w()) / 3.0f) && move_pointer_ == 0) {
            move_pointer_ = 1;
        } else if (view_pointer_ == 0) {
            view_pointer_ = 1;
        }
        break;
    case RawInputEvent::EvP2Down:
        if (evt.point.x < (float(ren_ctx_->w()) / 3.0f) && move_pointer_ == 0) {
            move_pointer_ = 2;
        } else if (view_pointer_ == 0) {
            view_pointer_ = 2;
        }
        break;
    case RawInputEvent::EvP1Up:
        if (move_pointer_ == 1) {
            move_pointer_ = 0;
            fwd_touch_speed_ = 0;
            side_touch_speed_ = 0;
        } else if (view_pointer_ == 1) {
            view_pointer_ = 0;
        }
        break;
    case RawInputEvent::EvP2Up:
        if (move_pointer_ == 2) {
            move_pointer_ = 0;
            fwd_touch_speed_ = 0;
            side_touch_speed_ = 0;
        } else if (view_pointer_ == 2) {
            view_pointer_ = 0;
        }
        break;
    case RawInputEvent::EvP1Move:
        if (move_pointer_ == 1) {
            side_touch_speed_ += evt.move.dx * 0.002f;
            side_touch_speed_ =
                std::max(std::min(side_touch_speed_, max_fwd_speed_), -max_fwd_speed_);

            fwd_touch_speed_ -= evt.move.dy * 0.002f;
            fwd_touch_speed_ =
                std::max(std::min(fwd_touch_speed_, max_fwd_speed_), -max_fwd_speed_);
        } else if (view_pointer_ == 1) {
            auto up = Vec3f{0, 1, 0};
            Vec3f side = Normalize(Cross(view_dir_, up));
            up = Cross(side, view_dir_);

            Mat4f rot;
            rot = Rotate(rot, -0.005f * evt.move.dx, up);
            rot = Rotate(rot, -0.005f * evt.move.dy, side);

            auto rot_m3 = Mat3f(rot);
            view_dir_ = rot_m3 * view_dir_;

            invalidate_view_ = true;
        }
        break;
    case RawInputEvent::EvP2Move:
        if (move_pointer_ == 2) {
            side_touch_speed_ += evt.move.dx * 0.002f;
            side_touch_speed_ =
                std::max(std::min(side_touch_speed_, max_fwd_speed_), -max_fwd_speed_);

            fwd_touch_speed_ -= evt.move.dy * 0.002f;
            fwd_touch_speed_ =
                std::max(std::min(fwd_touch_speed_, max_fwd_speed_), -max_fwd_speed_);
        } else if (view_pointer_ == 2) {
            auto up = Vec3f{0, 1, 0};
            Vec3f side = Normalize(Cross(view_dir_, up));
            up = Cross(side, view_dir_);

            Mat4f rot;
            rot = Rotate(rot, 0.01f * evt.move.dx, up);
            rot = Rotate(rot, 0.01f * evt.move.dy, side);

            auto rot_m3 = Mat3f(rot);
            view_dir_ = rot_m3 * view_dir_;

            invalidate_view_ = true;
        }
        break;
    case RawInputEvent::EvKeyDown: {
        if (evt.key_code == KeyUp ||
            (evt.key_code == KeyW && (!cmdline_enabled_ || view_pointer_))) {
            fwd_press_speed_ = max_fwd_speed_;
        } else if (evt.key_code == KeyDown ||
                   (evt.key_code == KeyS && (!cmdline_enabled_ || view_pointer_))) {
            fwd_press_speed_ = -max_fwd_speed_;
        } else if (evt.key_code == KeyLeft ||
                   (evt.key_code == KeyA && (!cmdline_enabled_ || view_pointer_))) {
            side_press_speed_ = -max_fwd_speed_;
        } else if (evt.key_code == KeyRight ||
                   (evt.key_code == KeyD && (!cmdline_enabled_ || view_pointer_))) {
            side_press_speed_ = max_fwd_speed_;
        } else if (evt.key_code == KeySpace) {
        } else if (evt.key_code == KeyLeftShift || evt.key_code == KeyRightShift) {
            shift_down_ = true;
        } else {
            input_processed = false;
        }
    } break;
    case RawInputEvent::EvKeyUp: {
        if (!cmdline_enabled_ || view_pointer_) {
            if (evt.key_code == KeyUp || evt.key_code == KeyW ||
                evt.key_code == KeyDown || evt.key_code == KeyS) {
                fwd_press_speed_ = 0;
            } else if (evt.key_code == KeyLeft || evt.key_code == KeyA ||
                       evt.key_code == KeyRight || evt.key_code == KeyD) {
                side_press_speed_ = 0;
            } else {
                input_processed = false;
            }
        } else {
            input_processed = false;
        }
    } break;
    default:
        break;
    }

    if (!input_processed) {
        GSBaseState::HandleInput(evt);
    }

    return true;
}

void GSPhyTest::OnUpdateScene() {
    const float delta_time_s = fr_info_.delta_time_us * 0.000001f;

    // Update camera
    scene_manager_->SetupView(view_origin_, (view_origin_ + view_dir_),
                              Ren::Vec3f{0.0f, 1.0f, 0.0f}, view_fov_, max_exposure_);

    // log_->Info("%f %f %f | %f %f %f",
    //        view_origin_[0], view_origin_[1], view_origin_[2],
    //        view_dir_[0], view_dir_[1], view_dir_[2]);
}

void GSPhyTest::SaveScene(JsObject &js_scene) {
    GSBaseState::SaveScene(js_scene);

    { // write camera
        JsObject js_camera;

        { // write view origin
            JsArray js_view_origin;
            js_view_origin.Push(JsNumber{double(initial_view_pos_[0])});
            js_view_origin.Push(JsNumber{double(initial_view_pos_[1])});
            js_view_origin.Push(JsNumber{double(initial_view_pos_[2])});

            js_camera.Push("view_origin", std::move(js_view_origin));
        }

        { // write view direction
            JsArray js_view_dir;
            js_view_dir.Push(JsNumber{double(initial_view_dir_[0])});
            js_view_dir.Push(JsNumber{double(initial_view_dir_[1])});
            js_view_dir.Push(JsNumber{double(initial_view_dir_[2])});

            js_camera.Push("view_dir", std::move(js_view_dir));
        }

        { // write forward speed
            js_camera.Push("fwd_speed", JsNumber{double(max_fwd_speed_)});
        }

        { // write fov
            js_camera.Push("fov", JsNumber{double(view_fov_)});
        }

        { // write max exposure
            js_camera.Push("max_exposure", JsNumber{double(max_exposure_)});
        }

        js_scene.Push("camera", std::move(js_camera));
    }
}
