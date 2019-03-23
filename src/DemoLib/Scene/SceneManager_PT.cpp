#include "SceneManager.h"

#include <map>

#include <Ren/Context.h>
#include <Sys/AssetFile.h>
#include <Sys/Log.h>
#include <Sys/ThreadPool.h>

#include "../Renderer/Renderer.h"

namespace SceneManagerConstants {
extern const char *MODELS_PATH;
extern const char *TEXTURES_PATH;
extern const char *MATERIALS_PATH;
extern const char *SHADERS_PATH;
}

namespace SceneManagerInternal {
void Write_RGBM(const float *out_data, int w, int h, int channels, const char *name);
void Write_RGBE(const Ray::pixel_color_t *out_data, int w, int h, const char *name);

void LoadTGA(Sys::AssetFile &in_file, int w, int h, Ray::pixel_color8_t *out_data);

std::vector<Ray::pixel_color_t> FlushSeams(const Ray::pixel_color_t *pixels, int res, float invalid_threshold, int filter_size);

std::unique_ptr<Ray::pixel_color8_t[]> GetTextureData(const Ren::Texture2DRef &tex_ref);
}

void SceneManager::Draw_PT() {
    if (!ray_scene_) return;

    if (ray_reg_ctx_.empty()) {
        if (ray_renderer_.type() == Ray::RendererOCL) {
            ray_reg_ctx_.emplace_back(Ray::rect_t{ 0, 0, ctx_.w(), ctx_.h() });
        } else {
            const int TILE_SIZE = 64;

#if defined(__ANDROID__)
            int pt_res_w = 640, pt_res_h = 360;
#else
            int pt_res_w = ctx_.w(), pt_res_h = ctx_.h();
#endif

            for (int y = 0; y < pt_res_h + TILE_SIZE - 1; y += TILE_SIZE) {
                for (int x = 0; x < pt_res_w + TILE_SIZE - 1; x += TILE_SIZE) {
                    auto rect = Ray::rect_t{ x, y, std::min(TILE_SIZE, pt_res_w - x), std::min(TILE_SIZE, pt_res_h - y) };
                    if (rect.w > 0 && rect.h > 0) {
                        ray_reg_ctx_.emplace_back(rect);
                    }
                }
            }

            ray_renderer_.Resize(pt_res_w, pt_res_h);
        }
    }

    // main view camera
    ray_scene_->set_current_cam(0);

    if (ray_renderer_.type() == Ray::RendererOCL) {
        ray_renderer_.RenderScene(ray_scene_, ray_reg_ctx_[0]);
    } else {
        auto render_task = [this](int i) { ray_renderer_.RenderScene(ray_scene_, ray_reg_ctx_[i]); };
        std::vector<std::future<void>> ev(ray_reg_ctx_.size());
        for (int i = 0; i < (int)ray_reg_ctx_.size(); i++) {
            ev[i] = threads_.enqueue(render_task, i);
        }
        for (const auto &e : ev) {
            e.wait();
        }
    }

    const auto *pixels = ray_renderer_.get_pixels_ref();
    renderer_.BlitPixelsTonemap(pixels, ray_renderer_.size().first, ray_renderer_.size().second, Ren::RawRGBA32F);
}

void SceneManager::ResetLightmaps_PT() {
    if (!ray_scene_) return;

    Ray::camera_desc_t cam_desc;
    ray_scene_->GetCamera(1, cam_desc);

    for (size_t i = 0; i < objects_.size(); i++) {
        if (objects_[i].flags & HasLightmap) {
            cur_lm_obj_ = i;
            cam_desc.mi_index = objects_[i].pt_mi;
            break;
        }
    }

    cam_desc.skip_direct_lighting = false;
    cam_desc.skip_indirect_lighting = true;

    ray_scene_->SetCamera(1, cam_desc);

    cur_lm_indir_ = false;
}

bool SceneManager::PrepareLightmaps_PT() {
    using namespace SceneManagerConstants;

    if (!ray_scene_) return false;

    const int LM_SAMPLES_TOTAL =
#ifdef NDEBUG
        4096;
#else
        32;
#endif
    const int LM_SAMPLES_PER_PASS = 16;
    const int TILE_SIZE = 64;

    const int res = (int)objects_[cur_lm_obj_].lm->size[0];

    if (ray_reg_ctx_.empty()) {
        if (ray_renderer_.type() == Ray::RendererOCL) {
            ray_reg_ctx_.emplace_back(Ray::rect_t{ 0, 0, res, res });
            ray_renderer_.Resize(res, res);
        } else {
            for (int y = 0; y < res + TILE_SIZE - 1; y += TILE_SIZE) {
                for (int x = 0; x < res + TILE_SIZE - 1; x += TILE_SIZE) {
                    auto rect = Ray::rect_t{ x, y, std::min(TILE_SIZE, res - x), std::min(TILE_SIZE, res - y) };
                    if (rect.w > 0 && rect.h > 0) {
                        ray_reg_ctx_.emplace_back(rect);
                    }
                }
            }
        }
    }

    const auto &cur_size = ray_renderer_.size();

    if (cur_size.first != res || cur_size.second != res) {
        if (ray_renderer_.type() == Ray::RendererOCL) {
            ray_reg_ctx_[0] = Ray::RegionContext{ { 0, 0, res, res } };
        } else {
            ray_reg_ctx_.clear();

            for (int y = 0; y < res + TILE_SIZE - 1; y += TILE_SIZE) {
                for (int x = 0; x < res + TILE_SIZE - 1; x += TILE_SIZE) {
                    auto rect = Ray::rect_t{ x, y, std::min(TILE_SIZE, res - x), std::min(TILE_SIZE, res - y) };
                    if (rect.w > 0 && rect.h > 0) {
                        ray_reg_ctx_.emplace_back(rect);
                    }
                }
            }
        }
        ray_renderer_.Resize(res, res);
    }

    // special lightmap camera
    ray_scene_->set_current_cam(1);

    if (ray_reg_ctx_[0].iteration >= LM_SAMPLES_TOTAL) {
        {
            // Save lightmap to file
            const auto *pixels = ray_renderer_.get_pixels_ref();

            const float InvalidThreshold = 0.5f;
            const int FilterSize = 16;

            // apply dilation filter
            std::vector<Ray::pixel_color_t> out_pixels = SceneManagerInternal::FlushSeams(pixels, res, InvalidThreshold, FilterSize);

            std::string out_file_name = "./assets/textures/lightmaps/" + scene_name_;
            out_file_name += "_";
            out_file_name += std::to_string(cur_lm_obj_);
            if (!cur_lm_indir_) {
                out_file_name += "_lm_direct.png";
            } else {
                out_file_name += "_lm_indirect.png";
            }

            SceneManagerInternal::Write_RGBM(&out_pixels[0].r, res, res, 4, out_file_name.c_str());

            if (cur_lm_indir_) {
                std::vector<Ray::shl1_data_t> sh_data(ray_renderer_.get_sh_data_ref(), ray_renderer_.get_sh_data_ref() + res * res);
                std::vector<Ray::pixel_color_t> temp_pixels1(res * res, Ray::pixel_color_t{ 0.0f, 0.0f, 0.0f, 1.0f });

                const float SH_Y0 = 0.282094806f; // sqrt(1.0f / (4.0f * PI))
                const float SH_Y1 = 0.488602519f; // sqrt(3.0f / (4.0f * PI))

                const float SH_A0 = 0.886226952f; // PI / sqrt(4.0f * Pi)
                const float SH_A1 = 1.02332675f;  // sqrt(PI / 3.0f)

                const float SH_AY0 = 0.25f; // SH_A0 * SH_Y0
                const float SH_AY1 = 0.5f;  // SH_A1 * SH_Y1

                const float inv_pi = 1.0f / Ren::Pi<float>();
                const float mult[] = { SH_A0 * inv_pi, SH_A1 * inv_pi, SH_A1 * inv_pi, SH_A1 * inv_pi };

                for (int i = 0; i < res * res; i++) {
                    for (int sh_l = 0; sh_l < 4; sh_l++) {
                        sh_data[i].coeff_r[sh_l] *= mult[sh_l];
                        sh_data[i].coeff_g[sh_l] *= mult[sh_l];
                        sh_data[i].coeff_b[sh_l] *= mult[sh_l];
                    }
                }

                for (int i = 0; i < res * res; i++) {
                    const float coverage = pixels[i].a;

                    if (coverage < InvalidThreshold) continue;

                    sh_data[i].coeff_r[0] /= coverage;
                    sh_data[i].coeff_g[0] /= coverage;
                    sh_data[i].coeff_b[0] /= coverage;

                    for (int sh_l = 1; sh_l < 4; sh_l++) {
                        sh_data[i].coeff_r[sh_l] /= coverage;
                        sh_data[i].coeff_g[sh_l] /= coverage;
                        sh_data[i].coeff_b[sh_l] /= coverage;

                        if (sh_data[i].coeff_r[0] > FLT_EPSILON) {
                            sh_data[i].coeff_r[sh_l] /= sh_data[i].coeff_r[0];
                        }
                        if (sh_data[i].coeff_g[0] > FLT_EPSILON) {
                            sh_data[i].coeff_g[sh_l] /= sh_data[i].coeff_g[0];
                        }
                        if (sh_data[i].coeff_b[0] > FLT_EPSILON) {
                            sh_data[i].coeff_b[sh_l] /= sh_data[i].coeff_b[0];
                        }

                        sh_data[i].coeff_r[sh_l] = 0.25f * sh_data[i].coeff_r[sh_l] + 0.5f;
                        sh_data[i].coeff_g[sh_l] = 0.25f * sh_data[i].coeff_g[sh_l] + 0.5f;
                        sh_data[i].coeff_b[sh_l] = 0.25f * sh_data[i].coeff_b[sh_l] + 0.5f;

                        if (sh_data[i].coeff_r[sh_l] > 1.0f) sh_data[i].coeff_r[sh_l] = 1.0f;
                        else if (sh_data[i].coeff_r[sh_l] < 0.0f) sh_data[i].coeff_r[sh_l] = 0.0f;
                        if (sh_data[i].coeff_g[sh_l] > 1.0f) sh_data[i].coeff_g[sh_l] = 1.0f;
                        else if (sh_data[i].coeff_g[sh_l] < 0.0f) sh_data[i].coeff_g[sh_l] = 0.0f;
                        if (sh_data[i].coeff_b[sh_l] > 1.0f) sh_data[i].coeff_b[sh_l] = 1.0f;
                        else if (sh_data[i].coeff_b[sh_l] < 0.0f) sh_data[i].coeff_b[sh_l] = 0.0f;
                    }
                }

                for (int sh_l = 0; sh_l < 4; sh_l++) {
                    for (int i = 0; i < res * res; i++) {
                        temp_pixels1[i].r = sh_data[i].coeff_r[sh_l];
                        temp_pixels1[i].g = sh_data[i].coeff_g[sh_l];
                        temp_pixels1[i].b = sh_data[i].coeff_b[sh_l];

                        // coverage division is already applied in previous step
                        if (pixels[i].a > InvalidThreshold) {
                            temp_pixels1[i].a = 1.0f;
                        } else {
                            temp_pixels1[i].a = 0.0f;
                        }
                    }

                    const auto out_pixels = SceneManagerInternal::FlushSeams(&temp_pixels1[0], res, InvalidThreshold, FilterSize);

                    out_file_name = "./assets/textures/";
                    out_file_name += "/lightmaps/";
                    out_file_name += scene_name_;
                    out_file_name += "_";
                    out_file_name += std::to_string(cur_lm_obj_);
                    out_file_name += "_lm_sh_";
                    out_file_name += std::to_string(sh_l);
                    out_file_name += ".png";

                    if (sh_l == 0) {
                        // Write as HDR image
                        SceneManagerInternal::Write_RGBM(&out_pixels[0].r, res, res, 4, out_file_name.c_str());
                    } else {
                        // Write as LDR image
                        SceneManagerInternal::Write_RGB(&out_pixels[0], res, res, out_file_name.c_str());
                    }
                }
            }
        }

        Ray::camera_desc_t cam_desc;
        ray_scene_->GetCamera(1, cam_desc);

        if (!cur_lm_indir_) {
            cur_lm_indir_ = true;

            cam_desc.skip_direct_lighting = true;
            cam_desc.skip_indirect_lighting = false;
            cam_desc.output_sh = true;
        } else {
            bool found = false;

            for (size_t i = cur_lm_obj_ + 1; i < objects_.size(); i++) {
                if (objects_[i].flags & HasLightmap) {
                    cur_lm_obj_ = i;
                    cam_desc.mi_index = objects_[i].pt_mi;
                    found = true;
                    break;
                }
            }

            if (!found) {
                return false;
            }

            cur_lm_indir_ = false;
            cam_desc.skip_direct_lighting = false;
            cam_desc.skip_indirect_lighting = true;
            cam_desc.output_sh = false;
        }

        ray_scene_->SetCamera(1, cam_desc);

        ray_renderer_.Clear();
        for (auto &c : ray_reg_ctx_) {
            c.Clear();
        }
    }

    for (int i = 0; i < LM_SAMPLES_PER_PASS; i++) {
        if (ray_renderer_.type() == Ray::RendererOCL) {
            ray_renderer_.RenderScene(ray_scene_, ray_reg_ctx_[0]);
        } else {
            auto render_task = [this](int i) { ray_renderer_.RenderScene(ray_scene_, ray_reg_ctx_[i]); };
            std::vector<std::future<void>> ev(ray_reg_ctx_.size());
            for (int i = 0; i < (int)ray_reg_ctx_.size(); i++) {
                ev[i] = threads_.enqueue(render_task, i);
            }
            for (const auto &e : ev) {
                e.wait();
            }
        }
    }

    LOGI("Lightmap: %i %i/%i", int(cur_lm_obj_), ray_reg_ctx_[0].iteration, LM_SAMPLES_TOTAL);

    const auto *pixels = ray_renderer_.get_pixels_ref();
    renderer_.BlitPixels(pixels, res, res, Ren::RawRGBA32F);

    return true;
}

void SceneManager::InitScene_PT(bool _override) {
    using namespace SceneManagerConstants;

    if (ray_scene_) {
        if (_override) {
            ray_scene_ = nullptr;
        } else {
            return;
        }
    }

    ray_scene_ = ray_renderer_.CreateScene();

    ray_reg_ctx_.clear();

    // Setup environment
    {
        Ray::environment_desc_t env_desc;
        env_desc.env_col[0] = env_desc.env_col[1] = env_desc.env_col[2] = 1.0f;

        if (!env_map_pt_name_.empty()) {
            int w, h;
            auto tex_data = LoadHDR(("./assets/textures/" + env_map_pt_name_).c_str(), w, h);

            Ray::tex_desc_t tex_desc;
            tex_desc.data = (const Ray::pixel_color8_t *)&tex_data[0];
            tex_desc.w = w;
            tex_desc.h = h;
            tex_desc.generate_mipmaps = false;

            env_desc.env_map = ray_scene_->AddTexture(tex_desc);
            env_desc.env_clamp = 4.0f;
        }

        ray_scene_->SetEnvironment(env_desc);
    }

    // Add main camera
    {
        Ray::camera_desc_t cam_desc;
        cam_desc.type = Ray::Persp;
        cam_desc.filter = Ray::Tent;
        cam_desc.origin[0] = cam_desc.origin[1] = cam_desc.origin[2] = 0.0f;
        cam_desc.fwd[0] = cam_desc.fwd[1] = 0.0f;
        cam_desc.fwd[2] = -1.0f;
        cam_desc.fov = cam_.angle();
        cam_desc.gamma = 1.0f;
        cam_desc.focus_distance = 1.0f;
        cam_desc.focus_factor = 0.0f;

        ray_scene_->AddCamera(cam_desc);
    }

    // Add camera for lightmapping
    {
        Ray::camera_desc_t cam_desc;
        cam_desc.type = Ray::Geo;
        cam_desc.filter = Ray::Box;
        cam_desc.gamma = 1.0f;
        cam_desc.lighting_only = true;
        cam_desc.skip_direct_lighting = true;
        //cam_desc.skip_indirect_lighting = true;
        cam_desc.no_background = true;
        cam_desc.uv_index = 1;
        cam_desc.mi_index = 0;
        cam_desc.output_sh = true;
        cam_desc.use_coherent_sampling = true;

        ray_scene_->AddCamera(cam_desc);
    }

    // Add sun lamp
    if (Ren::Dot(env_.sun_dir, env_.sun_dir) > 0.00001f && Ren::Dot(env_.sun_col, env_.sun_col) > 0.00001f) {
        Ray::light_desc_t sun_desc;
        sun_desc.type = Ray::DirectionalLight;

        sun_desc.direction[0] = -env_.sun_dir[0];
        sun_desc.direction[1] = -env_.sun_dir[1];
        sun_desc.direction[2] = -env_.sun_dir[2];

        memcpy(&sun_desc.color[0], &env_.sun_col[0], 3 * sizeof(float));

        sun_desc.angle = env_.sun_softness;

        ray_scene_->AddLight(sun_desc);
    }

    std::map<std::string, uint32_t> loaded_materials, loaded_meshes, loaded_textures;

    uint32_t default_white_tex;

    {
        //  Add default white texture
        Ray::pixel_color8_t white = { 255, 255, 255, 255 };

        Ray::tex_desc_t tex_desc;
        tex_desc.data = &white;
        tex_desc.w = tex_desc.h = 1;
        tex_desc.generate_mipmaps = true;

        default_white_tex = ray_scene_->AddTexture(tex_desc);
    }
    
    (void)default_white_tex;

    /*uint32_t default_glow_mat;

    {
        Ray::mat_desc_t mat_desc;
        mat_desc.type = Ray::EmissiveMaterial;
        mat_desc.main_texture = default_white_tex;
        mat_desc.main_color[0] = 1.0f;
        mat_desc.main_color[1] = 0.0f;
        mat_desc.main_color[2] = 0.0f;

        default_glow_mat = ray_scene_->AddMaterial(mat_desc);
    }*/

    // Add objects
    for (auto &obj : objects_) {
        const uint32_t drawable_flags = HasMesh | HasTransform;
        if ((obj.flags & drawable_flags) == drawable_flags) {
            const auto *mesh = obj.mesh.get();
            const char *mesh_name = mesh->name();

            auto mesh_it = loaded_meshes.find(mesh_name);
            if (mesh_it == loaded_meshes.end()) {
                Ray::mesh_desc_t mesh_desc;
                mesh_desc.prim_type = Ray::TriangleList;
                mesh_desc.layout = Ray::PxyzNxyzBxyzTuvTuv;
                mesh_desc.vtx_attrs = (const float *)mesh->attribs();
                mesh_desc.vtx_attrs_count = (uint32_t)(mesh->attribs_size() / (13 * sizeof(float)));
                mesh_desc.vtx_indices = (const uint32_t *)mesh->indices();
                mesh_desc.vtx_indices_count = (uint32_t)(mesh->indices_size() / sizeof(uint32_t));
                mesh_desc.base_vertex = -int(mesh->attribs_offset() / (13 * sizeof(float)));

                const Ren::TriGroup *s = &mesh->group(0);
                while (s->offset != -1) {
                    const auto *mat = s->mat.get();
                    const char *mat_name = mat->name();

                    auto mat_it = loaded_materials.find(mat_name);
                    if (mat_it == loaded_materials.end()) {
                        Ray::mat_desc_t mat_desc;
                        mat_desc.main_color[0] = mat_desc.main_color[1] = mat_desc.main_color[2] = 1.0f;

                        Ren::Texture2DRef tex_ref;

                        //if (!mat->texture(2)) {
                            mat_desc.type = Ray::DiffuseMaterial;
                            tex_ref = mat->texture(0);
                        //} else {
                        //    mat_desc.type = Ray::EmissiveMaterial;
                        //    tex_ref = mat->texture(2);
                        //}

                        if (tex_ref) {
                            const char *tex_name = tex_ref->name();

                            auto tex_it = loaded_textures.find(tex_name);
                            if (tex_it == loaded_textures.end()) {
                                std::unique_ptr<Ray::pixel_color8_t[]> tex_data = SceneManagerInternal::GetTextureData(tex_ref);

                                auto params = tex_ref->params();

                                Ray::tex_desc_t tex_desc;
                                tex_desc.w = params.w;
                                tex_desc.h = params.h;
                                tex_desc.data = &tex_data[0];
                                tex_desc.generate_mipmaps = true;

                                uint32_t new_tex = ray_scene_->AddTexture(tex_desc);
                                tex_it = loaded_textures.emplace(tex_name, new_tex).first;
                            }

                            mat_desc.main_texture = tex_it->second;
                        }

                        uint32_t new_mat = ray_scene_->AddMaterial(mat_desc);
                        mat_it = loaded_materials.emplace(mat_name, new_mat).first;
                    }

                    mesh_desc.shapes.emplace_back(mat_it->second, 0xffffffff/*default_glow_mat*/, (size_t)(s->offset / sizeof(uint32_t)), (size_t)s->num_indices);
                    ++s;
                }

                uint32_t new_mesh = ray_scene_->AddMesh(mesh_desc);
                mesh_it = loaded_meshes.emplace(mesh_name, new_mesh).first;
            }

            const auto *tr = obj.tr.get();

            obj.pt_mi = ray_scene_->AddMeshInstance(mesh_it->second, Ren::ValuePtr(tr->mat));
        }
    }
}

void SceneManager::SetupView_PT(const Ren::Vec3f &origin, const Ren::Vec3f &target, const Ren::Vec3f &up) {
    if (!ray_scene_) return;

    Ray::camera_desc_t cam_desc;
    ray_scene_->GetCamera(0, cam_desc);

    auto fwd = Ren::Normalize(target - origin);

    memcpy(&cam_desc.origin[0], Ren::ValuePtr(origin), 3 * sizeof(float));
    memcpy(&cam_desc.fwd[0], Ren::ValuePtr(fwd), 3 * sizeof(float));

    ray_scene_->SetCamera(0, cam_desc);
}

void SceneManager::Clear_PT() {
    if (!ray_scene_) return;

    for (auto &c : ray_reg_ctx_) {
        c.Clear();
    }
    ray_renderer_.Clear();
}