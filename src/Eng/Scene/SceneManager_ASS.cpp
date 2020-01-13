#include "SceneManager.h"

#include <fstream>
#include <functional>
#include <iterator>
#include <numeric>

#include <dirent.h>

#ifdef __linux__
#include <sys/stat.h>
#endif

extern "C" {
#include <Ren/SOIL2/image_DXT.h>
}
#include <Ren/SOIL2/SOIL2.h>

#undef max
#undef min

#include <Eng/Gui/Renderer.h>
#include <Eng/Gui/Utils.h>
#include <Ray/internal/TextureSplitter.h>
#include <Ren/Utils.h>
#include <Sys/AssetFile.h>
#include <Sys/ThreadPool.h>

#define STB_TRUETYPE_IMPLEMENTATION
#define STBTT_STATIC
#include <stb/stb_truetype.h>

// TODO: pass defines as a parameter
#include "../Renderer/Renderer_GL_Defines.inl"

namespace SceneManagerInternal {
extern const char *MODELS_PATH;
extern const char *TEXTURES_PATH;
extern const char *MATERIALS_PATH;
extern const char *SHADERS_PATH;

void WriteImage(const uint8_t *out_data, int w, int h, int channels, const char *name);

void Write_RGBE(const Ray::pixel_color_t *out_data, int w, int h, const char *name) {
    std::unique_ptr<uint8_t[]> u8_data = Ren::ConvertRGB32F_to_RGBE(&out_data[0].r, w, h, 4);
    WriteImage(&u8_data[0], w, h, 4, name);
}

void Write_RGB(const Ray::pixel_color_t *out_data, int w, int h, const char *name) {
    std::vector<uint8_t> u8_data(w * h * 3);

    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            const Ray::pixel_color_t &p = out_data[y * w + x];

            u8_data[(y * w + x) * 3 + 0] = uint8_t(std::min(int(p.r * 255), 255));
            u8_data[(y * w + x) * 3 + 1] = uint8_t(std::min(int(p.g * 255), 255));
            u8_data[(y * w + x) * 3 + 2] = uint8_t(std::min(int(p.b * 255), 255));
        }
    }

    WriteImage(&u8_data[0], w, h, 3, name);
}

void Write_RGBM(const float *out_data, int w, int h, int channels, const char *name) {
    std::unique_ptr<uint8_t[]> u8_data = Ren::ConvertRGB32F_to_RGBM(out_data, w, h, channels);
    WriteImage(&u8_data[0], w, h, 4, name);
}

void Write_DDS_Mips(
        const uint8_t * const * mipmaps, const int *widths, const int *heights, const int mip_count,
        const int channels, const char *out_file) {
    //
    // Compress mip images
    //
    uint8_t *dxt_data[16] = {};
    int dxt_size[16] = {};
    int dxt_size_total = 0;

    for (int i = 0; i < mip_count; i++) {
        if (channels == 3) {
            dxt_data[i] = convert_image_to_DXT1(mipmaps[i], widths[i], heights[i], channels, &dxt_size[i]);
        } else if (channels == 4) {
            dxt_data[i] = convert_image_to_DXT5(mipmaps[i], widths[i], heights[i], channels, &dxt_size[i]);
        }
        dxt_size_total += dxt_size[i];
    }

    //
    // Write out file
    //
    DDS_header header = {};
    header.dwMagic = (unsigned('D') << 0u) | (unsigned('D') << 8u) | (unsigned('S') << 16u) | (unsigned(' ') << 24u);
    header.dwSize = 124;
    header.dwFlags =
            unsigned(DDSD_CAPS) | unsigned(DDSD_HEIGHT) | unsigned(DDSD_WIDTH) |
            unsigned(DDSD_PIXELFORMAT) | unsigned(DDSD_LINEARSIZE) | unsigned(DDSD_MIPMAPCOUNT);
    header.dwWidth = widths[0];
    header.dwHeight = heights[0];
    header.dwPitchOrLinearSize = dxt_size_total;
    header.dwMipMapCount = mip_count;
    header.sPixelFormat.dwSize = 32;
    header.sPixelFormat.dwFlags = DDPF_FOURCC;

    if (channels == 3) {
        header.sPixelFormat.dwFourCC =
                (unsigned('D') << 0u) | (unsigned('X') << 8u) | (unsigned('T') << 16u) | (unsigned('1') << 24u);
    } else {
        header.sPixelFormat.dwFourCC =
                (unsigned('D') << 0u) | (unsigned('X') << 8u) | (unsigned('T') << 16u) | (unsigned('5') << 24u);
    }

    header.sCaps.dwCaps1 = unsigned(DDSCAPS_TEXTURE) | unsigned(DDSCAPS_MIPMAP);

    std::ofstream out_stream(out_file, std::ios::binary);
    out_stream.write((char *)&header, sizeof(header));

    for (int i = 0; i < mip_count; i++) {
        out_stream.write((char *)dxt_data[i], dxt_size[i]);
        SOIL_free_image_data(dxt_data[i]);
        dxt_data[i] = nullptr;
    }
}

void Write_DDS(const uint8_t *image_data, const int w, const int h, const int channels, const bool flip_y, const char *out_file) {
    // Check if power of two
    const bool store_mipmaps = (unsigned(w) & unsigned(w - 1)) == 0 && (unsigned(h) & unsigned(h - 1)) == 0;

    std::unique_ptr<uint8_t[]> mipmaps[16] = {};
    int widths[16] = {},
        heights[16] = {};

    mipmaps[0].reset(new uint8_t[w * h * channels]);
    if (flip_y) {
        for (int j = 0; j < h; j++) {
            memcpy(&mipmaps[0][j * w * channels], &image_data[(h - j - 1) * w * channels], w * channels);
        }
    } else {
        memcpy(&mipmaps[0][0], &image_data[0], w * h * channels);
    }
    widths[0] = w;
    heights[0] = h;
    int mip_count;

    if (store_mipmaps) {
        mip_count = Ren::InitMipMaps(mipmaps, widths, heights, channels);
    } else {
        mip_count = 1;
    }

    uint8_t *_mipmaps[16];
    for (int i = 0; i < mip_count; i++) {
        _mipmaps[i] = mipmaps[i].get();
    }

    Write_DDS_Mips(_mipmaps, widths, heights, mip_count, channels, out_file);
}

void Write_KTX_DXT(const uint8_t *image_data, const int w, const int h, const int channels, const char *out_file) {
    // Check if power of two
    bool store_mipmaps = (w & (w - 1)) == 0 && (h & (h - 1)) == 0;

    std::unique_ptr<uint8_t[]> mipmaps[16] = {};
    int widths[16] = {},
        heights[16] = {};

    mipmaps[0].reset(new uint8_t[w * h * channels]);
    // mirror by y (????)
    for (int j = 0; j < h; j++) {
        memcpy(&mipmaps[0][j * w * channels], &image_data[(h - j - 1) * w * channels], w * channels);
    }
    widths[0] = w;
    heights[0] = h;
    int mip_count;

    if (store_mipmaps) {
        mip_count = Ren::InitMipMaps(mipmaps, widths, heights, channels);
    } else {
        mip_count = 1;
    }

    //
    // Compress mip images
    //
    uint8_t *dxt_data[16] = {};
    int dxt_size[16] = {};
    int dxt_size_total = 0;

    for (int i = 0; i < mip_count; i++) {
        if (channels == 3) {
            dxt_data[i] = convert_image_to_DXT1(mipmaps[i].get(), widths[i], heights[i], channels, &dxt_size[i]);
        } else if (channels == 4) {
            dxt_data[i] = convert_image_to_DXT5(mipmaps[i].get(), widths[i], heights[i], channels, &dxt_size[i]);
        }
        dxt_size_total += dxt_size[i];
    }

    //
    // Write out file
    //
    const uint32_t gl_rgb = 0x1907;
    const uint32_t gl_rgba = 0x1908;

    const uint32_t RGB_S3TC_DXT1 = 0x83F0;
    const uint32_t RGBA_S3TC_DXT5 = 0x83F3;

    Ren::KTXHeader header = {};
    header.gl_type = 0;
    header.gl_type_size = 1;
    header.gl_format = 0; // should be zero for compressed texture
    if (channels == 4) {
        header.gl_internal_format = RGBA_S3TC_DXT5;
        header.gl_base_internal_format = gl_rgba;
    } else {
        header.gl_internal_format = RGB_S3TC_DXT1;
        header.gl_base_internal_format = gl_rgb;
    }
    header.pixel_width = w;
    header.pixel_height = h;
    header.pixel_depth = 0;

    header.array_elements_count = 0;
    header.faces_count = 1;
    header.mipmap_levels_count = mip_count;
        
    header.key_value_data_size = 0;

    uint32_t file_offset = 0;
    std::ofstream out_stream(out_file, std::ios::binary);
    out_stream.write((char *)&header, sizeof(header));
    file_offset += sizeof(header);

    for (int i = 0; i < mip_count; i++) {
        assert((file_offset % 4) == 0);
        auto size = (uint32_t)dxt_size[i];
        out_stream.write((char *)&size, sizeof(uint32_t));
        file_offset += sizeof(uint32_t);
        out_stream.write((char *)dxt_data[i], size);
        file_offset += size;

        uint32_t pad = (file_offset % 4) ? (4 - (file_offset % 4)) : 0;
        while (pad) {
            const uint8_t zero_byte = 0;
            out_stream.write((char *)&zero_byte, 1);
            pad--;
        }

        SOIL_free_image_data(dxt_data[i]);
        dxt_data[i] = nullptr;
    }
}

int ConvertToASTC(const uint8_t *image_data, int width, int height, int channels, float bitrate, std::unique_ptr<uint8_t[]> &out_buf);
std::unique_ptr<uint8_t[]> DecodeASTC(const uint8_t *image_data, int data_size, int xdim, int ydim, int width, int height);
//std::unique_ptr<uint8_t[]> Decode_KTX_ASTC(const uint8_t *image_data, int data_size, int &width, int &height);

void Write_KTX_ASTC_Mips(
        const uint8_t * const * mipmaps, const int *widths, const int *heights, const int mip_count,
        const int channels, const char *out_file) {

    int quality = 0;
    if (strstr(out_file, "_norm")) {
        quality = 1;
    } else if (strstr(out_file, "lightmaps") || strstr(out_file, "probes_cache")) {
        quality = 2;
    }

    const float bits_per_pixel_sel[] = {
        2.0f, 3.56f, 8.0f
    };

    // Write file
    std::unique_ptr<uint8_t[]> astc_data[16];
    int astc_size[16] = {};
    int astc_size_total = 0;

    for (int i = 0; i < mip_count; i++) {
        astc_size[i] = ConvertToASTC(mipmaps[i], widths[i], heights[i], channels, bits_per_pixel_sel[quality], astc_data[i]);
        astc_size_total += astc_size[i];
    }

    const uint32_t gl_rgb = 0x1907;
    const uint32_t gl_rgba = 0x1908;

    const uint32_t gl_compressed_rgba_astc_4x4_khr = 0x93B0;
    const uint32_t gl_compressed_rgba_astc_6x6_khr = 0x93B4;
    const uint32_t gl_compressed_rgba_astc_8x8_khr = 0x93B7;

    const uint32_t gl_format_sel[] = {
        gl_compressed_rgba_astc_8x8_khr,
        gl_compressed_rgba_astc_6x6_khr,
        gl_compressed_rgba_astc_4x4_khr
    };

    Ren::KTXHeader header = {};
    header.gl_type = 0;
    header.gl_type_size = 1;
    header.gl_format = 0; // should be zero for compressed texture
    header.gl_internal_format = gl_format_sel[quality];

    if (channels == 4) {
        header.gl_base_internal_format = gl_rgba;
    } else {
        header.gl_base_internal_format = gl_rgb;
    }
    header.pixel_width = widths[0];
    header.pixel_height = heights[0];
    header.pixel_depth = 0;

    header.array_elements_count = 0;
    header.faces_count = 1;
    header.mipmap_levels_count = mip_count;

    header.key_value_data_size = 0;

    uint32_t file_offset = 0;
    std::ofstream out_stream(out_file, std::ios::binary);
    out_stream.write((char *)&header, sizeof(header));
    file_offset += sizeof(header);

    for (int i = 0; i < mip_count; i++) {
        assert((file_offset % 4) == 0);
        auto size = (uint32_t)astc_size[i];
        out_stream.write((char *)&size, sizeof(uint32_t));
        file_offset += sizeof(uint32_t);
        out_stream.write((char *)astc_data[i].get(), size);
        file_offset += size;

        uint32_t pad = (file_offset % 4) ? (4 - (file_offset % 4)) : 0;
        while (pad) {
            const uint8_t zero_byte = 0;
            out_stream.write((char *)&zero_byte, 1);
            pad--;
        }
    }
}

void Write_KTX_ASTC(const uint8_t *image_data, const int w, const int h, const int channels, const bool flip_y, const char *out_file) {
    // Check if power of two
    const bool store_mipmaps = (unsigned(w) & unsigned(w - 1)) == 0 && (unsigned(h) & unsigned(h - 1)) == 0;

    std::unique_ptr<uint8_t[]> mipmaps[16] = {};
    int widths[16] = {},
        heights[16] = {};

    mipmaps[0].reset(new uint8_t[w * h * channels]);
    if (flip_y) {
        for (int j = 0; j < h; j++) {
            memcpy(&mipmaps[0][j * w * channels], &image_data[(h - j - 1) * w * channels], w * channels);
        }
    } else {
        memcpy(&mipmaps[0][0], &image_data[0], w * h * channels);
    }
    widths[0] = w;
    heights[0] = h;
    int mip_count;

    if (store_mipmaps) {
        mip_count = Ren::InitMipMaps(mipmaps, widths, heights, channels);
    } else {
        mip_count = 1;
    }

    uint8_t *_mipmaps[16];
    for (int i = 0; i < mip_count; i++) {
        _mipmaps[i] = mipmaps[i].get();
    }

    Write_KTX_ASTC_Mips(_mipmaps, widths, heights, mip_count, channels, out_file);
}

void WriteImage(const uint8_t *out_data, int w, int h, int channels, const char *name) {
    int res = 0;
    if (strstr(name, ".tga")) {
        res = SOIL_save_image(name, SOIL_SAVE_TYPE_TGA, w, h, channels, out_data);
    } else if (strstr(name, ".png")) {
        res = SOIL_save_image(name, SOIL_SAVE_TYPE_PNG, w, h, channels, out_data);
    } else if (strstr(name, ".dds")) {
        res = 1;
        Write_DDS(out_data, w, h, channels, true /* flip_y */, name);
    } else if (strstr(name, ".ktx")) {
        res = 1;
        Write_KTX_ASTC(out_data, w, h, channels, true /* flip_y */, name);
    }

    if (!res) {
        LOGE("Failed to save image %s", name);
    }
}

void LoadTGA(Sys::AssetFile &in_file, int w, int h, Ray::pixel_color8_t *out_data) {
    auto in_file_size = (size_t)in_file.size();

    std::vector<char> in_file_data(in_file_size);
    in_file.Read(&in_file_data[0], in_file_size);

    Ren::eTexColorFormat format;
    int _w, _h;
    std::unique_ptr<uint8_t[]> pixels = Ren::ReadTGAFile(&in_file_data[0], _w, _h, format);

    if (_w != w || _h != h) return;

    if (format == Ren::RawRGB888) {
        int i = 0;
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                out_data[i++] = { pixels[3 * (y * w + x)], pixels[3 * (y * w + x) + 1], pixels[3 * (y * w + x) + 2], 255 };
            }
        }
    } else if (format == Ren::RawRGBA8888) {
        int i = 0;
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                out_data[i++] = { pixels[4 * (y * w + x)], pixels[4 * (y * w + x) + 1], pixels[4 * (y * w + x) + 2], pixels[4 * (y * w + x) + 3] };
            }
        }
    } else {
        assert(false);
    }
}

std::vector<Ray::pixel_color_t> FlushSeams(const Ray::pixel_color_t *pixels, int width, int height, float invalid_threshold, int filter_size) {
    std::vector<Ray::pixel_color_t> temp_pixels1{ pixels, pixels + width * height },
                                    temp_pixels2{ (size_t)width * height };

    // Avoid bound checks in debug
    Ray::pixel_color_t *_temp_pixels1 = temp_pixels1.data(),
                       *_temp_pixels2 = temp_pixels2.data();

    // apply dilation filter
    for (int i = 0; i < filter_size; i++) {
        bool has_invalid = false;

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                Ray::pixel_color_t in_p = _temp_pixels1[y * width + x];
                Ray::pixel_color_t &out_p = _temp_pixels2[y * width + x];

                float mul = 1.0f;
                if (in_p.a < invalid_threshold) {
                    has_invalid = true;

                    Ray::pixel_color_t new_p = { 0 };
                    int count = 0;

                    int _ys[] = { y - 1, y, y + 1 },
                        _xs[] = { x - 1, x, x + 1 };
                    for (int _y : _ys) {
                        if (_y < 0 || _y > height - 1) continue;

                        for (int _x : _xs) {
                            if (_x < 0 || _x > width - 1) continue;

                            const Ray::pixel_color_t &p = _temp_pixels1[_y * width + _x];
                            if (p.a >= invalid_threshold) {
                                new_p.r += p.r;
                                new_p.g += p.g;
                                new_p.b += p.b;
                                new_p.a += p.a;

                                count++;
                            }
                        }
                    }

                    if (count) {
                        const float inv_c = 1.0f / float(count);
                        new_p.r *= inv_c;
                        new_p.g *= inv_c;
                        new_p.b *= inv_c;
                        new_p.a *= inv_c;

                        in_p = new_p;
                    }
                } else {
                    mul = 1.0f / in_p.a;
                }

                out_p.r = in_p.r * mul;
                out_p.g = in_p.g * mul;
                out_p.b = in_p.b * mul;
                out_p.a = in_p.a * mul;
            }
        }

        std::swap(_temp_pixels1, _temp_pixels2);
        if (!has_invalid) break;
    }

    return temp_pixels1;
}

std::unique_ptr<Ray::pixel_color8_t[]> GetTextureData(const Ren::Texture2DRef &tex_ref) {
    const Ren::Texture2DParams &params = tex_ref->params();

    std::unique_ptr<Ray::pixel_color8_t[]> tex_data(new Ray::pixel_color8_t[params.w * params.h]);
#if defined(__ANDROID__)
    Sys::AssetFile in_file((std::string("assets/textures/") + tex_ref->name().c_str()).c_str());
    SceneManagerInternal::LoadTGA(in_file, params.w, params.h, &tex_data[0]);
#else
    tex_ref->ReadTextureData(Ren::RawRGBA8888, (void *)&tex_data[0]);
#endif

    return tex_data;
}

void ReadAllFiles_r(const char *in_folder, const std::function<void(const char *)> &callback) {
    DIR *in_dir = opendir(in_folder);
    if (!in_dir) {
        LOGE("Cannot open folder %s", in_folder);
        return;
    }

    struct dirent *in_ent = nullptr;
    while ((in_ent = readdir(in_dir))) {
        if (in_ent->d_type == DT_DIR) {
            if (strcmp(in_ent->d_name, ".") == 0 || strcmp(in_ent->d_name, "..") == 0) {
                continue;
            }
            std::string path = in_folder;
            path += '/';
            path += in_ent->d_name;

            ReadAllFiles_r(path.c_str(), callback);
        } else {
            std::string path = in_folder;
            path += '/';
            path += in_ent->d_name;

            callback(path.c_str());
        }
    }

    closedir(in_dir);
}

void ReadAllFiles_MT_r(const char *in_folder, const std::function<void(const char *)> &callback, Sys::ThreadPool &threads, std::vector<std::future<void>> &events) {
    DIR *in_dir = opendir(in_folder);
    if (!in_dir) {
        LOGE("Cannot open folder %s", in_folder);
        return;
    }

    struct dirent *in_ent = nullptr;
    while ((in_ent = readdir(in_dir))) {
        if (in_ent->d_type == DT_DIR) {
            if (strcmp(in_ent->d_name, ".") == 0 || strcmp(in_ent->d_name, "..") == 0) {
                continue;
            }
            std::string path = in_folder;
            path += '/';
            path += in_ent->d_name;

            ReadAllFiles_r(path.c_str(), callback);
        } else {
            std::string path = in_folder;
            path += '/';
            path += in_ent->d_name;

            events.push_back(threads.enqueue([path, &callback]() {
                callback(path.c_str());
            }));
        }
    }

    closedir(in_dir);
}

bool CheckCanSkipAsset(const char *in_file, const char *out_file) {
#if !defined(NDEBUG) && 0
    if (strstr(in_file, ".glsl")) return false;
#endif

#ifdef _WIN32
    HANDLE in_h = CreateFile(in_file, GENERIC_READ, 0, NULL, OPEN_EXISTING, NULL, NULL);
    if (in_h == INVALID_HANDLE_VALUE) {
        LOGI("[PrepareAssets] Failed to open file!");
        CloseHandle(in_h);
        return true;
    }
    HANDLE out_h = CreateFile(out_file, GENERIC_READ, 0, NULL, OPEN_EXISTING, NULL, NULL);
    LARGE_INTEGER out_size = {};
    if (out_h != INVALID_HANDLE_VALUE && GetFileSizeEx(out_h, &out_size) && out_size.QuadPart) {
        FILETIME in_t, out_t;
        GetFileTime(in_h, NULL, NULL, &in_t);
        GetFileTime(out_h, NULL, NULL, &out_t);

        if (CompareFileTime(&in_t, &out_t) == -1) {
            CloseHandle(in_h);
            CloseHandle(out_h);
            return true;
        }
    }

    CloseHandle(in_h);
    CloseHandle(out_h);
#else
    struct stat st1 = {}, st2 = {};
    if (stat(in_file, &st1) != -1 && stat(out_file, &st2) != -1) {
        struct tm tm1 = {}, tm2 = {};
        localtime_r(&st1.st_ctime, &tm1);
        localtime_r(&st2.st_ctime, &tm2);
        
        time_t t1 = mktime(&tm1), t2 = mktime(&tm2);

        double diff_s = difftime(t1, t2);
        if (diff_s < 0) {
            return true;
        }
    }
    
#endif
    return false;
}

bool CreateFolders(const char *out_file) {
    const char *end = strchr(out_file, '/');
    while (end) {
        char folder[256] = {};
        strncpy(folder, out_file, end - out_file + 1);
#ifdef _WIN32
        if (!CreateDirectory(folder, NULL)) {
            if (GetLastError() != ERROR_ALREADY_EXISTS) {
                LOGE("[PrepareAssets] Failed to create directory!");
                return false;
            }
        }
#else
        struct stat st = {};
        if (stat(folder, &st) == -1) {
            if (mkdir(folder, 0777) != 0) {
                LOGE("[PrepareAssets] Failed to create directory!");
                return false;
            }
        }
#endif
        end = strchr(end + 1, '/');
    }
    return true;
}

}

// these are from astc codec
#undef IGNORE
#include <astc/astc_codec_internals.h>
#include <Eng/Gui/BitmapFont.h>

#undef MAX

int astc_main(int argc, char **argv);

void test_inappropriate_extended_precision();
void prepare_angular_tables();
void build_quantization_mode_table();
void find_closest_blockdim_2d(float target_bitrate, int *x, int *y, int consider_illegal);

void encode_astc_image(const astc_codec_image *input_image,
                       astc_codec_image *output_image,
                       int xdim, int ydim, int zdim,
                       const error_weighting_params *ewp, astc_decode_mode decode_mode, swizzlepattern swz_encode, swizzlepattern swz_decode, uint8_t *buffer, int pack_and_unpack, int threadcount);

bool g_astc_initialized = false;

bool SceneManager::PrepareAssets(const char *in_folder, const char *out_folder, const char *platform, Sys::ThreadPool *p_threads) {
    using namespace SceneManagerInternal;

    // for astc codec
    if (!g_astc_initialized) {
        test_inappropriate_extended_precision();
        prepare_angular_tables();
        build_quantization_mode_table();
        g_astc_initialized = true;
    }

    auto replace_texture_extension = [platform](std::string &tex) {
        size_t n;
        if ((n = tex.find(".uncompressed")) == std::string::npos) {
            if ((n = tex.find(".tga")) != std::string::npos) {
                if (strcmp(platform, "pc") == 0) {
                    tex.replace(n + 1, 3, "dds");
                } else if (strcmp(platform, "android") == 0) {
                    tex.replace(n + 1, 3, "ktx");
                }
            } else if ((n = tex.find(".png")) != std::string::npos ||
                       (n = tex.find(".img")) != std::string::npos) {
                if (strcmp(platform, "pc") == 0) {
                    tex.replace(n + 1, 3, "dds");
                } else if (strcmp(platform, "android") == 0) {
                    tex.replace(n + 1, 3, "ktx");
                }
            }
        }
    };

    Ren::HashMap32<std::string, std::string> shader_constants;

    shader_constants.Insert("$ModifyWarning",   "/***********************************************/\r\n"
                                                "/* This file was autogenerated, do not modify! */\r\n"
                                                "/***********************************************/");

    shader_constants.Insert("$FltEps",          "0.0000001");

    shader_constants.Insert("$ItemGridResX",    AS_STR(REN_GRID_RES_X));
    shader_constants.Insert("$ItemGridResY",    AS_STR(REN_GRID_RES_Y));
    shader_constants.Insert("$ItemGridResZ",    AS_STR(REN_GRID_RES_Z));

    // Vertex attributes
    shader_constants.Insert("$VtxPosLoc",       AS_STR(REN_VTX_POS_LOC));
    shader_constants.Insert("$VtxNorLoc",       AS_STR(REN_VTX_NOR_LOC));
    shader_constants.Insert("$VtxTanLoc",       AS_STR(REN_VTX_TAN_LOC));
    shader_constants.Insert("$VtxUV1Loc",       AS_STR(REN_VTX_UV1_LOC));
    shader_constants.Insert("$VtxUV2Loc",       AS_STR(REN_VTX_UV2_LOC));

    // Texture slots
    shader_constants.Insert("$MatTex0Slot",     AS_STR(REN_MAT_TEX0_SLOT));
    shader_constants.Insert("$MatTex1Slot",     AS_STR(REN_MAT_TEX1_SLOT));
    shader_constants.Insert("$MatTex2Slot",     AS_STR(REN_MAT_TEX2_SLOT));
    shader_constants.Insert("$ShadTexSlot",     AS_STR(REN_SHAD_TEX_SLOT));
    //shader_constants.Insert("$LmapDirSlot",    AS_STR(REN_LMAP_DIR_SLOT));
    //shader_constants.Insert("$LmapIndirSlot",  AS_STR(REN_LMAP_INDIR_SLOT));
    shader_constants.Insert("$LmapSHSlot",      AS_STR(REN_LMAP_SH_SLOT));
    shader_constants.Insert("$DecalTexSlot",    AS_STR(REN_DECAL_TEX_SLOT));
    shader_constants.Insert("$SSAOTexSlot",     AS_STR(REN_SSAO_TEX_SLOT));
    shader_constants.Insert("$BRDFLutTexSlot",  AS_STR(REN_BRDF_TEX_SLOT));
    shader_constants.Insert("$LightBufSlot",    AS_STR(REN_LIGHT_BUF_SLOT));
    shader_constants.Insert("$DecalBufSlot",    AS_STR(REN_DECAL_BUF_SLOT));
    shader_constants.Insert("$CellsBufSlot",    AS_STR(REN_CELLS_BUF_SLOT));
    shader_constants.Insert("$ItemsBufSlot",    AS_STR(REN_ITEMS_BUF_SLOT));
    shader_constants.Insert("$InstanceBufSlot", AS_STR(REN_INST_BUF_SLOT));
    shader_constants.Insert("$EnvTexSlot",      AS_STR(REN_ENV_TEX_SLOT));
    shader_constants.Insert("$Moments0TexSlot", AS_STR(REN_MOMENTS0_TEX_SLOT));
    shader_constants.Insert("$Moments1TexSlot", AS_STR(REN_MOMENTS1_TEX_SLOT));
    shader_constants.Insert("$Moments2TexSlot", AS_STR(REN_MOMENTS2_TEX_SLOT));
    shader_constants.Insert("$Moments0MsTexSlot", AS_STR(REN_MOMENTS0_MS_TEX_SLOT));
    shader_constants.Insert("$Moments1MsTexSlot", AS_STR(REN_MOMENTS1_MS_TEX_SLOT));
    shader_constants.Insert("$Moments2MsTexSlot", AS_STR(REN_MOMENTS2_MS_TEX_SLOT));

    // Uniform locations
    shader_constants.Insert("$uMMatrixLoc",     AS_STR(REN_U_M_MATRIX_LOC));
    shader_constants.Insert("$uInstancesLoc",   AS_STR(REN_U_INSTANCES_LOC));

    // Uniform block locations
    shader_constants.Insert("$ubSharedDataLoc", AS_STR(REN_UB_SHARED_DATA_LOC));
    shader_constants.Insert("$ubBatchDataLoc",  AS_STR(REN_UB_BATCH_DATA_LOC));

    // Shader output channels
    shader_constants.Insert("$OutColorIndex",   AS_STR(REN_OUT_COLOR_INDEX));
    shader_constants.Insert("$OutNormIndex",    AS_STR(REN_OUT_NORM_INDEX));
    shader_constants.Insert("$OutSpecIndex",    AS_STR(REN_OUT_SPEC_INDEX));

    // Shadow properties
    if (strcmp(platform, "pc") == 0) {
        shader_constants.Insert("$ShadRes",     AS_STR(REN_SHAD_RES_PC));
    } else if (strcmp(platform, "android") == 0) {
        shader_constants.Insert("$ShadRes",     AS_STR(REN_SHAD_RES_ANDROID));
    } else {
        LOGE("Unknown platform %s", platform);
        return false;
    }

    shader_constants.Insert("$ShadCasc0Dist",   AS_STR(REN_SHAD_CASCADE0_DIST));
    shader_constants.Insert("$ShadCasc0Samp",   AS_STR(REN_SHAD_CASCADE0_SAMPLES));
    shader_constants.Insert("$ShadCasc1Dist",   AS_STR(REN_SHAD_CASCADE1_DIST));
    shader_constants.Insert("$ShadCasc1Samp",   AS_STR(REN_SHAD_CASCADE1_SAMPLES));
    shader_constants.Insert("$ShadCasc2Dist",   AS_STR(REN_SHAD_CASCADE2_DIST));
    shader_constants.Insert("$ShadCasc2Samp",   AS_STR(REN_SHAD_CASCADE2_SAMPLES));
    shader_constants.Insert("$ShadCasc3Dist",   AS_STR(REN_SHAD_CASCADE3_DIST));
    shader_constants.Insert("$ShadCasc3Samp",   AS_STR(REN_SHAD_CASCADE3_SAMPLES));
    shader_constants.Insert("$ShadCascSoft",    AS_STR(REN_SHAD_CASCADE_SOFT));

    shader_constants.Insert("$MaxShadowMaps",   AS_STR(REN_MAX_SHADOWMAPS_TOTAL));
    shader_constants.Insert("$MaxProbes",       AS_STR(REN_MAX_PROBES_TOTAL));

    shader_constants.Insert("$MaxBatchSize",    AS_STR(REN_MAX_BATCH_SIZE));

    auto inline_constants = [&shader_constants](std::string &line) {
        size_t n = 0;
        while ((n = line.find('$', n)) != std::string::npos) {
            size_t l = 1;

            const char punctuation_chars[] = ".,(); $*[]\r\n";
            while (std::find(std::begin(punctuation_chars), std::end(punctuation_chars), line[n + l]) == std::end(punctuation_chars)) {
                l++;
            }

            const std::string var = line.substr(n, l);

            const std::string *it = shader_constants.Find(var);
            if (it) {
                line.replace(n, l, *it);
            } else {
                LOGE("Unknown variable %s", var.c_str());
                throw std::runtime_error("Unknown variable!");
            }
        }
    };

    auto h_skip = [](const char *in_file, const char *out_file) {
        LOGI("[PrepareAssets] Skip %s", out_file);
    };

    auto h_copy = [](const char *in_file, const char *out_file) {
        LOGI("[PrepareAssets] Copy %s", out_file);

        std::ifstream src_stream(in_file, std::ios::binary);
        std::ofstream dst_stream(out_file, std::ios::binary);

        std::istreambuf_iterator<char> src_beg(src_stream);
        std::istreambuf_iterator<char> src_end;
        std::ostreambuf_iterator<char> dst_beg(dst_stream);
        std::copy(src_beg, src_end, dst_beg);
    };

    auto h_conv_to_dds = [](const char *in_file, const char *out_file) {
        LOGI("[PrepareAssets] Conv %s", out_file);

        std::ifstream src_stream(in_file, std::ios::binary | std::ios::ate);
        auto src_size = (size_t)src_stream.tellg();
        src_stream.seekg(0, std::ios::beg);

        std::unique_ptr<uint8_t[]> src_buf(new uint8_t[src_size]);
        src_stream.read((char *)&src_buf[0], src_size);

        int width, height, channels;
        unsigned char *image_data = SOIL_load_image_from_memory(&src_buf[0], (int)src_size, &width, &height, &channels, 0);

        if (strstr(in_file, "_norm")) {
            // this is normal map, store it in RxGB format
            std::unique_ptr<uint8_t[]> temp_data(new uint8_t[width * height * 4]);
            assert(channels == 3);

            for (int j = 0; j < height; j++) {
                for (int i = 0; i < width; i++) {
                    temp_data[4 * (j * width + i) + 0] = 0;
                    temp_data[4 * (j * width + i) + 1] = image_data[3 * (j * width + i) + 1];
                    temp_data[4 * (j * width + i) + 2] = image_data[3 * (j * width + i) + 2];
                    temp_data[4 * (j * width + i) + 3] = image_data[3 * (j * width + i) + 0];
                }
            }

            Write_DDS(temp_data.get(), width, height, 4, true /* flip_y */, out_file);
            SOIL_free_image_data(image_data);
        } else {
            Write_DDS(image_data, width, height, channels, true /* flip_y */, out_file);
            SOIL_free_image_data(image_data);
        }
    };

    auto h_conv_img_to_dds = [](const char *in_file, const char *out_file) {
        LOGI("[PrepareAssets] Conv %s", out_file);

        std::ifstream src_stream(in_file, std::ios::binary | std::ios::ate);
        auto src_size = (int)src_stream.tellg();
        src_stream.seekg(0, std::ios::beg);

        int res, mips_count;
        src_stream.read((char *)&res, sizeof(int));
        src_size -= sizeof(int);
        src_stream.read((char *)&mips_count, sizeof(int));
        src_size -= sizeof(int);

        std::unique_ptr<uint8_t[]> mipmaps[16];
        uint8_t *_mipmaps[16];
        int widths[16], heights[16];

        for (int i = 0; i < mips_count; i++) {
            const int mip_res = (res >> i);
            const int buf_size = mip_res * mip_res * 4;

            mipmaps[i].reset(new uint8_t[buf_size]);
            _mipmaps[i] = mipmaps[i].get();
            widths[i] = heights[i] = mip_res;

            src_stream.read((char *)&mipmaps[i][0], buf_size);
            src_size -= buf_size;
        }

        if (src_size != 0) {
            LOGE("Error reading file %s", in_file);
        }

        Write_DDS_Mips(_mipmaps, widths, heights, mips_count, 4, out_file);
    };

    auto h_conv_img_to_astc = [](const char *in_file, const char *out_file) {
        LOGI("[PrepareAssets] Conv %s", out_file);

        std::ifstream src_stream(in_file, std::ios::binary | std::ios::ate);
        auto src_size = (int)src_stream.tellg();
        src_stream.seekg(0, std::ios::beg);

        int res, mips_count;
        src_stream.read((char *)&res, sizeof(int));
        src_size -= sizeof(int);
        src_stream.read((char *)&mips_count, sizeof(int));
        src_size -= sizeof(int);

        std::unique_ptr<uint8_t[]> mipmaps[16];
        uint8_t *_mipmaps[16];
        int widths[16], heights[16];

        for (int i = 0; i < mips_count; i++) {
            const int mip_res = (res >> i);
            const int buf_size = mip_res * mip_res * 4;

            mipmaps[i].reset(new uint8_t[buf_size]);
            _mipmaps[i] = mipmaps[i].get();
            widths[i] = heights[i] = mip_res;

            src_stream.read((char *)&mipmaps[i][0], buf_size);
            src_size -= buf_size;
        }

        if (src_size != 0) {
            LOGE("Error reading file %s", in_file);
        }

        Write_KTX_ASTC_Mips(_mipmaps, widths, heights, mips_count, 4, out_file);
    };

    auto h_conv_to_astc = [](const char *in_file, const char *out_file) {
        LOGI("[PrepareAssets] Conv %s", out_file);

        std::ifstream src_stream(in_file, std::ios::binary | std::ios::ate);
        auto src_size = (size_t)src_stream.tellg();
        src_stream.seekg(0, std::ios::beg);

        std::unique_ptr<uint8_t[]> src_buf(new uint8_t[src_size]);
        src_stream.read((char *)&src_buf[0], src_size);

        int width, height, channels;
        unsigned char *image_data = SOIL_load_image_from_memory(&src_buf[0], (int)src_size, &width, &height, &channels, 0);


        if (strstr(in_file, "_norm")) {
            // this is normal map, store it in RxGB format
            std::unique_ptr<uint8_t[]> temp_data(new uint8_t[width * height * 4]);
            assert(channels == 3);

            for (int j = 0; j < height; j++) {
                for (int i = 0; i < width; i++) {
                    temp_data[4 * (j * width + i) + 0] = 0;
                    temp_data[4 * (j * width + i) + 1] = image_data[3 * (j * width + i) + 1];
                    temp_data[4 * (j * width + i) + 2] = image_data[3 * (j * width + i) + 2];
                    temp_data[4 * (j * width + i) + 3] = image_data[3 * (j * width + i) + 0];
                }
            }

            Write_KTX_ASTC(temp_data.get(), width, height, 4, true /* flip_y */, out_file);
            SOIL_free_image_data(image_data);
        } else {
            Write_KTX_ASTC(image_data, width, height, channels, true /* flip_y */, out_file);
            SOIL_free_image_data(image_data);
        }
    };

    auto h_conv_hdr_to_rgbm = [](const char *in_file, const char *out_file) {
        LOGI("[PrepareAssets] Conv %s", out_file);

        int width, height;
        const std::vector<uint8_t> image_rgbe = LoadHDR(in_file, width, height);
        std::unique_ptr<float[]> image_f32 = Ren::ConvertRGBE_to_RGB32F(&image_rgbe[0], width, height);
        
        std::unique_ptr<float[]> temp(new float[width * 3]);
        for (int j = 0; j < height / 2; j++) {
            int j1 = j, j2 = height - j - 1;
            memcpy(&temp[0], &image_f32[j1 * width * 3], width * 3 * sizeof(float));
            memcpy(&image_f32[j1 * width * 3], &image_f32[j2 * width * 3], width * 3 * sizeof(float));
            memcpy(&image_f32[j2 * width * 3], &temp[0], width * 3 * sizeof(float));
        }

        Write_RGBM(&image_f32[0], width, height, 3, out_file);
    };

    auto h_preprocess_material = [&replace_texture_extension](const char *in_file, const char *out_file) {
        LOGI("[PrepareAssets] Prep %s", out_file);

        std::ifstream src_stream(in_file, std::ios::binary);
        std::ofstream dst_stream(out_file, std::ios::binary);

        std::string line;
        while (std::getline(src_stream, line)) {
            replace_texture_extension(line);
            dst_stream << line << "\r\n";
        }
    };

    auto h_preprocess_json = [&replace_texture_extension](const char *in_file, const char *out_file) {
        LOGI("[PrepareAssets] Prep %s", out_file);

        std::ifstream src_stream(in_file, std::ios::binary);
        std::ofstream dst_stream(out_file, std::ios::binary);

        JsObject js_root;
        if (!js_root.Read(src_stream)) {
            throw std::runtime_error("Cannot load scene!");
        }

        if (js_root.Has("objects")) {
            auto &js_objects = (JsArray &)js_root.at("objects");
            for (JsElement &js_obj_el : js_objects.elements) {
                auto &js_obj = (JsObject &)js_obj_el;

                if (js_obj.Has("decal")) {
                    auto &js_decal = (JsObject &)js_obj.at("decal");
                    if (js_decal.Has("diff")) {
                        auto &js_diff_tex = (JsString &)js_decal.at("diff");
                        replace_texture_extension(js_diff_tex.val);
                    }
                    if (js_decal.Has("norm")) {
                        auto &js_norm_tex = (JsString &)js_decal.at("norm");
                        replace_texture_extension(js_norm_tex.val);
                    }
                    if (js_decal.Has("spec")) {
                        auto &js_spec_tex = (JsString &)js_decal.at("spec");
                        replace_texture_extension(js_spec_tex.val);
                    }
                }
            }
        }

        if (js_root.Has("probes")) {
            auto &js_probes = (JsArray &)js_root.at("probes");
            for (JsElement &js_probe_el : js_probes.elements) {
                auto &js_probe = (JsObject &)js_probe_el;

                if (js_probe.Has("faces")) {
                    JsArray &js_faces = (JsArray &)js_probe.at("faces");
                    for (JsElement &js_face_el : js_faces.elements) {
                        auto &js_face_str = (JsString &)js_face_el;
                        replace_texture_extension(js_face_str.val);
                    }
                }
            }
        }

        JsFlags flags;
        flags.use_spaces = 1;

        js_root.Write(dst_stream, flags);
    };

    auto h_preprocess_shader = [&inline_constants, platform](const char *in_file, const char *out_file) {
        LOGI("[PrepareAssets] Prep %s", out_file);

        {   // resolve includes, inline constants
            std::ifstream src_stream(in_file, std::ios::binary);
            std::ofstream dst_stream(out_file, std::ios::binary);
            std::string line;

            int line_counter = 0;

            while (std::getline(src_stream, line)) {
                if (!line.empty() && line.back() == '\r') {
                    line = line.substr(0, line.size() - 1);
                }

                if (line.rfind("#version ") == 0) {
                    if (strcmp(platform, "pc") == 0) {
                        line = "#version 430";
                    }
                    dst_stream << line << "\r\n";
                } else if (line.rfind("#include ") == 0) {
                    size_t n1 = line.find_first_of('\"');
                    size_t n2 = line.find_last_of('\"');

                    std::string file_name = line.substr(n1 + 1, n2 - n1 - 1);

                    auto slash_pos = (size_t)intptr_t(strrchr(in_file, '/') - in_file);
                    std::string full_path = std::string(in_file, slash_pos + 1) + file_name;

                    dst_stream << "#line 0\r\n";

                    std::ifstream incl_stream(full_path, std::ios::binary);
                    while (std::getline(incl_stream, line)) {
                        if (!line.empty() && line.back() == '\r') {
                            line = line.substr(0, line.size() - 1);
                        }

                        inline_constants(line);

                        dst_stream << line << "\r\n";
                    }

                    dst_stream << "\r\n#line " << line_counter << "\r\n";
                } else {
                    inline_constants(line);

                    dst_stream << line << "\r\n";
                }

                line_counter++;
            }
        }

        if (strcmp(platform, "pc") == 0) {
            std::string spv_file = out_file;

            size_t n;
            if ((n = spv_file.find(".glsl")) != std::string::npos) {
                spv_file.replace(n + 1, 4, "spv", 3);
            }

            std::string compile_cmd = "src/libs/spirv/glslangValidator -G ";
            compile_cmd += out_file;
            compile_cmd += " -o ";
            compile_cmd += spv_file;

#ifdef _WIN32
            std::replace(compile_cmd.begin(), compile_cmd.end(), '/', '\\');
#endif
            int res = system(compile_cmd.c_str());
            if (res != 0) {
                LOGE("[PrepareAssets] Failed to compile %s", spv_file.c_str());
            }

            std::string optimize_cmd = "src/libs/spirv/spirv-opt "
                "--eliminate-dead-branches "
                "--merge-return "
                "--inline-entry-points-exhaustive "
                "--loop-unswitch --loop-unroll "
                "--eliminate-dead-code-aggressive "
                "--private-to-local "
                "--eliminate-local-single-block "
                "--eliminate-local-single-store "
                "--eliminate-dead-code-aggressive "
                //"--scalar-replacement=100 "
                "--convert-local-access-chains "
                "--eliminate-local-single-block "
                "--eliminate-local-single-store "
                //"--eliminate-dead-code-aggressive "
                //"--eliminate-local-multi-store "
                //"--eliminate-dead-code-aggressive "
                "--ccp "
                //"--eliminate-dead-code-aggressive "
                "--redundancy-elimination "
                "--combine-access-chains "
                "--simplify-instructions "
                "--vector-dce "
                "--eliminate-dead-inserts "
                "--eliminate-dead-branches "
                "--simplify-instructions "
                "--if-conversion "
                "--copy-propagate-arrays "
                "--reduce-load-size "
                //"--eliminate-dead-code-aggressive "
                //"--merge-blocks "
                "--redundancy-elimination "
                "--eliminate-dead-branches "
                //"--merge-blocks "
                "--simplify-instructions "
                "--validate-after-all ";

            optimize_cmd += spv_file;
            optimize_cmd += " -o ";
            optimize_cmd += spv_file;

#ifdef _WIN32
            std::replace(optimize_cmd.begin(), optimize_cmd.end(), '/', '\\');
#endif
            res = system(optimize_cmd.c_str());
            if (res != 0) {
                LOGE("[PrepareAssets] Failed to optimize %s", spv_file.c_str());
            }

            std::string cross_cmd = "src/libs/spirv/spirv-cross ";
            if (strcmp(platform, "pc") == 0) {
                cross_cmd += "--version 430 ";
            } else if (strcmp(platform, "android") == 0) {
                cross_cmd += "--version 310 --es ";
                cross_cmd += "--extension GL_EXT_texture_buffer ";
            }
            cross_cmd += "--no-support-nonzero-baseinstance --glsl-emit-push-constant-as-ubo ";
            cross_cmd += spv_file;
            cross_cmd += " --output ";
            cross_cmd += out_file;

#ifdef _WIN32
            std::replace(cross_cmd.begin(), cross_cmd.end(), '/', '\\');
#endif
            res = system(cross_cmd.c_str());
            if (res != 0) {
                LOGE("[PrepareAssets] Failed to cross-compile %s", spv_file.c_str());
            }
        }
    };

    auto h_conv_to_font = [](const char *in_file, const char *out_file) {
        using namespace Ren;

        LOGI("[PrepareAssets] Conv %s", out_file);

        std::ifstream src_stream(in_file, std::ios::binary | std::ios::ate);
        auto src_size = (size_t)src_stream.tellg();
        src_stream.seekg(0, std::ios::beg);

        std::unique_ptr<uint8_t[]> src_buf(new uint8_t[src_size]);
        src_stream.read((char *)&src_buf[0], src_size);

        stbtt_fontinfo font;
        int res = stbtt_InitFont(&font, &src_buf[0], 0);
        if (!res) {
            LOGE("stbtt_InitFont failed (%s)", in_file);
            return;
        }

        const bool
            is_sdf_font = strstr(in_file, "_sdf") != nullptr,
            is_inv_wind = strstr(in_file, "_inv") != nullptr;
        float line_height = 48.0f;

        int temp_bitmap_res[2] = { 512, 256 };

        if (strstr(in_file, "_9px")) {
            line_height = 9.0f;
        } else if (strstr(in_file, "_12px")) {
            line_height = 12.0f;
            //temp_bitmap_res[0] = 256;
            //temp_bitmap_res[1] = 128;
        } else if (strstr(in_file, "_16px")) {
            line_height = 16.0f;
            //temp_bitmap_res[0] = 256;
            //temp_bitmap_res[1] = 128;
        } else if (strstr(in_file, "_24px")) {
            line_height = 24.0f;
        } else if (strstr(in_file, "_32px")) {
            line_height = 32.0f;
        } else if (strstr(in_file, "_48px")) {
            line_height = 48.0f;
        }

        const float scale = stbtt_ScaleForPixelHeight(&font, line_height);

        const int sdf_radius_px = is_sdf_font ? 1 : 0;
        const int padding = 1;

        const Gui::glyph_range_t glyph_ranges[] = {
            { Gui::g_unicode_latin_range.first, Gui::g_unicode_latin_range.second },
            { Gui::g_unicode_cyrilic_range_min.first, Gui::g_unicode_cyrilic_range_min.second },
            { Gui::g_unicode_umlauts[0], Gui::g_unicode_umlauts[0] + 1 },
            { Gui::g_unicode_umlauts[1], Gui::g_unicode_umlauts[1] + 1 },
            { Gui::g_unicode_umlauts[2], Gui::g_unicode_umlauts[2] + 1 },
            { Gui::g_unicode_umlauts[3], Gui::g_unicode_umlauts[3] + 1 },
            { Gui::g_unicode_heart,      Gui::g_unicode_heart + 1 }
        };
        const int glyph_range_count = sizeof(glyph_ranges) / sizeof(glyph_ranges[0]);

        const int total_glyph_count = std::accumulate(std::begin(glyph_ranges), std::end(glyph_ranges), 0,
            [](int sum, const Gui::glyph_range_t val) -> int {
                return sum + int(val.end - val.beg);
            });

        std::unique_ptr<Gui::glyph_info_t[]> out_glyphs(new Gui::glyph_info_t[total_glyph_count]);
        int out_glyph_count = 0;

        std::unique_ptr<uint8_t[]> temp_bitmap(new uint8_t[temp_bitmap_res[0] * temp_bitmap_res[1] * 4]);
        Ray::TextureSplitter temp_bitmap_splitter(temp_bitmap_res);

        std::fill(&temp_bitmap[0], &temp_bitmap[0] + 4 * temp_bitmap_res[0] * temp_bitmap_res[1], 0);

        for (const Gui::glyph_range_t &range : glyph_ranges) {
            LOGI("Processing glyph range (%i - %i)", range.beg, range.end);
            for (uint32_t i = range.beg; i < range.end; i++) {
                const int glyph_index = stbtt_FindGlyphIndex(&font, i);

                int x0, y0, x1, y1;
                bool is_drawable = stbtt_GetGlyphBox(&font, glyph_index, &x0, &y0, &x1, &y1) != 0;

                int advance_width, left_side_bearing;
                stbtt_GetGlyphHMetrics(&font, glyph_index, &advance_width, &left_side_bearing);

                int glyph_pos[2] = {}, glyph_res[2] = {}, glyph_res_act[2] = {};
                if (is_drawable) {
                    glyph_res_act[0] = (int)std::round(scale * float(x1 - x0 + 1)) + 2 * sdf_radius_px;
                    glyph_res_act[1] = (int)std::round(scale * float(y1 - y0 + 1)) + 2 * sdf_radius_px;

                    glyph_res[0] = glyph_res_act[0] + 2 * padding;
                    glyph_res[1] = glyph_res_act[1] + 2 * padding;

                    int node_index = temp_bitmap_splitter.Allocate(glyph_res, glyph_pos);
                    if (node_index == -1) {
                        throw std::runtime_error("Region allocation failed!");
                    }
                }

                Gui::glyph_info_t &out_glyph = out_glyphs[out_glyph_count++];

                out_glyph.pos[0] = glyph_pos[0] + padding;
                out_glyph.pos[1] = glyph_pos[1] + padding;
                out_glyph.res[0] = glyph_res_act[0];
                out_glyph.res[1] = glyph_res_act[1];
                out_glyph.off[0] = (int)std::round(scale * float(x0));
                out_glyph.off[1] = (int)std::round(scale * float(y0));
                out_glyph.adv[0] = (int)std::round(scale * float(advance_width));
                out_glyph.adv[1] = 0;

                if (!is_drawable) continue;

                using bezier_shape = std::vector<Gui::bezier_seg_t>;
                std::vector<bezier_shape> shapes;

                {   // Get glyph shapes
                    stbtt_vertex *vertices = nullptr;
                    const int vertex_count = stbtt_GetGlyphShape(&font, glyph_index, &vertices);

                    {   // transform input data
                        const auto pos_offset = Vec2d{ (double)(padding + sdf_radius_px), (double)(padding + sdf_radius_px) };

                        Vec2i cur_p;

                        for (int j = 0; j < vertex_count; j++) {
                            const stbtt_vertex &v = vertices[j];

                            const Vec2d
                                p0 = pos_offset + Vec2d{ cur_p - Vec2i{x0, y0} } * scale,
                                c0 = pos_offset + Vec2d{ double(v.cx - x0), double(v.cy - y0) } * scale,
                                c1 = pos_offset + Vec2d{ double(v.cx1 - x0), double(v.cy1 - y0) } * scale,
                                p1 = pos_offset + Vec2d{ double(v.x - x0), double(v.y - y0) } * scale;

                            if (v.type == STBTT_vmove) {
                                if (shapes.empty() || !shapes.back().empty()) {
                                    // start new shape
                                    shapes.emplace_back();
                                }
                            } else {
                                // 1 - line; 2,3 - bezier with 1 and 2 control points
                                const int order = (v.type == STBTT_vline) ? 1 : ((v.type == STBTT_vcurve) ? 2 : 3);

                                shapes.back().push_back({
                                    order, false /* is_closed */, false /* is_hard */,
                                    p0, p1,
                                    c0, c1
                                });
                            }

                            cur_p = Vec2i{ v.x, v.y };
                        }
                    }

                    stbtt_FreeShape(&font, vertices);
                }

                if (!is_sdf_font) {
                    //
                    // Simple rasterization
                    //
                    const int samples = 4;

                    // Loop through image pixels
                    for (int y = 0; y < glyph_res[1]; y++) {
                        for (int x = 0; x < glyph_res[0]; x++) {
                            uint32_t out_val = 0;

                            for (int dy = 0; dy < samples; dy++) {
                                for (int dx = 0; dx < samples; dx++) {
                                    const auto p = Vec2d{
                                        double(x) + (0.5 + double(dx)) / samples,
                                        double(y) + (0.5 + double(dy)) / samples
                                    };

                                    double
                                        min_sdist = std::numeric_limits<double>::max(),
                                        min_dot = std::numeric_limits<double>::lowest();

                                    for (const bezier_shape &sh : shapes) {
                                        for (const Gui::bezier_seg_t & seg : sh) {
                                            const Gui::dist_result_t result = Gui::BezierSegmentDistance(seg, p);

                                            if (std::abs(result.sdist) < std::abs(min_sdist) ||
                                                (std::abs(result.sdist) == std::abs(min_sdist) && result.dot < min_dot)) {
                                                min_sdist = result.sdist;
                                                min_dot = result.dot;
                                            }
                                        }
                                    }

                                    out_val += (min_sdist > 0.0) ? 255 : 0;
                                }
                            }

                            // Write output value
                            const int
                                out_x = glyph_pos[0] + x,
                                out_y = glyph_pos[1] + (glyph_res[1] - y - 1);
                            uint8_t *out_pixel = &temp_bitmap[4 * (out_y * temp_bitmap_res[0] + out_x)];

                            out_pixel[0] = out_pixel[1] = out_pixel[2] = 255;
                            out_pixel[3] = (out_val / (samples * samples));
                        }
                    }
                } else {
                    //
                    // Multi-channel SDF font
                    //

                    // find hard edges, mark if closed etc.
                    for (bezier_shape &sh : shapes) {
                        Gui::PreprocessBezierShape(sh.data(), (int)sh.size(), 30.0 * Ren::Pi<double>() / 180.0 /* angle threshold */);
                    }

                    // Loop through image pixels
                    for (int y = 0; y < glyph_res[1]; y++) {
                        for (int x = 0; x < glyph_res[0]; x++) {
                            const auto p = Vec2d{ double(x) + 0.5, double(y) + 0.5 };

                            // Per channel distances (used for multi-channel sdf)
                            Gui::dist_result_t min_result[3];
                            for (Gui::dist_result_t &r : min_result) {
                                r.sdist = std::numeric_limits<double>::max();
                                r.ortho = r.dot = std::numeric_limits<double>::lowest();
                            }

                            // Simple distances (used for normal sdf)
                            double
                                min_sdf_sdist = std::numeric_limits<double>::max(),
                                min_sdf_dot = std::numeric_limits<double>::lowest();

                            for (const bezier_shape &sh : shapes) {
                                int edge_color_index = 0;
                                static const Vec3i edge_colors[] = {
                                    Vec3i{ 255, 0, 255 }, Vec3i{ 255, 255, 0 }, Vec3i{ 0, 255, 255 }
                                };

                                for (int i = 0; i < (int)sh.size(); i++) {
                                    const Gui::bezier_seg_t &seg = sh[i];
                                    const Gui::dist_result_t result = Gui::BezierSegmentDistance(seg, p);

                                    if (i != 0 && seg.is_hard) {
                                        if ((i == sh.size() - 1) && sh[0].is_closed && !sh[0].is_hard) {
                                            edge_color_index = 0;
                                        } else {
                                            if (edge_color_index == 1) {
                                                edge_color_index = 2;
                                            } else {
                                                edge_color_index = 1;
                                            }
                                        }
                                    }
                                    const Vec3i &edge_color = edge_colors[edge_color_index];

                                    for (int j = 0; j < 3; j++) {
                                        if (edge_color[j]) {
                                            if (std::abs(result.sdist) < std::abs(min_result[j].sdist) ||
                                                (std::abs(result.sdist) == std::abs(min_result[j].sdist) && result.dot < min_result[j].dot)) {
                                                min_result[j] = result;
                                            }
                                        }
                                    }

                                    if (std::abs(result.sdist) < std::abs(min_sdf_sdist) ||
                                        (std::abs(result.sdist) == std::abs(min_sdf_sdist) && result.dot < min_sdf_dot)) {
                                        min_sdf_sdist = result.sdist;
                                        min_sdf_dot = result.dot;
                                    }
                                }
                            }

                            // Write distance to closest shape
                            const int
                                out_x = glyph_pos[0] + x,
                                out_y = glyph_pos[1] + (glyph_res[1] - y - 1);
                            uint8_t *out_pixel = &temp_bitmap[4 * (out_y * temp_bitmap_res[0] + out_x)];

                            for (int j = 0; j < 3; j++) {
                                uint8_t out_val = 0;
                                if (min_result[j].sdist != std::numeric_limits<double>::max()) {
                                    min_result[j].pseudodist = Clamp(0.5 + (min_result[j].pseudodist / (2 * sdf_radius_px)), 0.0, 1.0);
                                    out_val = (uint8_t)std::max(std::min(int(255 * min_result[j].pseudodist), 255), 0);
                                }
                                out_pixel[j] = out_val;
                            }

                            min_sdf_sdist = Clamp(0.5 + (min_sdf_sdist / (2 * sdf_radius_px)), 0.0, 1.0);
                            out_pixel[3] = (uint8_t)std::max(std::min(int(255 * min_sdf_sdist), 255), 0);
                        }
                    }
                }
            }
        }

        if (is_sdf_font) {
            // Fix collisions of uncorrelated areas
            Gui::FixSDFCollisions(temp_bitmap.get(), temp_bitmap_res[0], temp_bitmap_res[1], 4, 200 /* threshold */);
        }

        if (is_inv_wind) {
            // Flip colors (font has inversed winding order)
            for (int y = 0; y < temp_bitmap_res[1]; y++) {
                for (int x = 0; x < temp_bitmap_res[0]; x++) {
                    uint8_t *out_pixel = &temp_bitmap[4 * (y * temp_bitmap_res[0] + x)];

                    if (is_sdf_font) {
                        out_pixel[0] = 255 - out_pixel[0];
                        out_pixel[1] = 255 - out_pixel[1];
                        out_pixel[2] = 255 - out_pixel[2];
                    }
                    out_pixel[3] = 255 - out_pixel[3];
                }
            }
        }

        assert(out_glyph_count == total_glyph_count);

        /*if (strstr(in_file, "Roboto-Regular_12px")) {
            WriteImage(temp_bitmap.get(), temp_bitmap_res[0], temp_bitmap_res[1], 4, "test.png");
        }*/

        std::ofstream out_stream(out_file, std::ios::binary);
        const uint32_t header_size = 4 + sizeof(uint32_t) + int(Gui::FontChCount) * 3 * sizeof(uint32_t);
        uint32_t hdr_offset = 0, data_offset = header_size;

        {   // File format string
            const char signature[] = { 'F', 'O', 'N', 'T' };
            out_stream.write(signature, 4);
            hdr_offset += 4;
        }

        {   // Header size
            out_stream.write((const char *)&header_size, sizeof(uint32_t));
            hdr_offset += sizeof(uint32_t);
        }

        {   // Typograph data offsets
            const uint32_t
                typo_data_chunk_id = (uint32_t)Gui::FontChTypoData,
                typo_data_offset = data_offset,
                typo_data_size = sizeof(Gui::typgraph_info_t);
            out_stream.write((const char *)&typo_data_chunk_id, sizeof(uint32_t));
            out_stream.write((const char *)&typo_data_offset, sizeof(uint32_t));
            out_stream.write((const char *)&typo_data_size, sizeof(uint32_t));
            hdr_offset += 3 * sizeof(uint32_t);
            data_offset += typo_data_size;
        }

        {   // Image data offsets
            const uint32_t
                img_data_chunk_id = (uint32_t)Gui::FontChImageData,
                img_data_offset = data_offset,
                img_data_size = 2 * sizeof(uint16_t) + 2 * sizeof(uint16_t) + 4 * temp_bitmap_res[0] * temp_bitmap_res[1];
            out_stream.write((const char *)&img_data_chunk_id, sizeof(uint32_t));
            out_stream.write((const char *)&img_data_offset, sizeof(uint32_t));
            out_stream.write((const char *)&img_data_size, sizeof(uint32_t));
            hdr_offset += 3 * sizeof(uint32_t);
            data_offset += img_data_size;
        }

        {   // Glyph data offsets
            const uint32_t
                glyph_data_chunk_id = (uint32_t)Gui::FontChGlyphData,
                glyph_data_offset = data_offset,
                glyph_data_size = sizeof(uint32_t) + sizeof(glyph_ranges) + total_glyph_count * sizeof(Gui::glyph_info_t);
            out_stream.write((const char *)&glyph_data_chunk_id, sizeof(uint32_t));
            out_stream.write((const char *)&glyph_data_offset, sizeof(uint32_t));
            out_stream.write((const char *)&glyph_data_size, sizeof(uint32_t));
            hdr_offset += 3 * sizeof(uint32_t);
            data_offset += glyph_data_size;
        }

        assert(hdr_offset == header_size);

        {   // Typograph data
            Gui::typgraph_info_t info = {};
            info.line_height = (uint32_t)line_height;

            out_stream.write((const char *)&info, sizeof(Gui::typgraph_info_t));
        }

        {   // Image data
            const auto img_data_w = (uint16_t)temp_bitmap_res[0], img_data_h = (uint16_t)temp_bitmap_res[1];
            out_stream.write((const char *)&img_data_w, sizeof(uint16_t));
            out_stream.write((const char *)&img_data_h, sizeof(uint16_t));

            const uint16_t
                draw_mode = is_sdf_font ? Gui::DrDistanceField : Gui::DrPassthrough,
                blend_mode = Gui::BlAlpha;
            out_stream.write((const char *)&draw_mode, sizeof(uint16_t));
            out_stream.write((const char *)&blend_mode, sizeof(uint16_t));

            out_stream.write((const char *)temp_bitmap.get(), 4 * temp_bitmap_res[0] * temp_bitmap_res[1]);
        }

        {   // Glyph data
            uint32_t u32_glyph_range_count = glyph_range_count;
            out_stream.write((const char *)&u32_glyph_range_count, sizeof(uint32_t));
            out_stream.write((const char *)&glyph_ranges[0].beg, sizeof(glyph_ranges));
            out_stream.write((const char *)out_glyphs.get(), total_glyph_count * sizeof(Gui::glyph_info_t));
        }
    };

    struct Handler {
        const char *ext;
        std::function<void(const char *in_file, const char *out_file)> convert;
    };

    Ren::HashMap32<std::string, Handler> handlers;

    handlers["bff"]         = { "bff",          h_copy              };
    handlers["mesh"]        = { "mesh",         h_copy              };
    handlers["anim"]        = { "anim",         h_copy              };
    handlers["vert.glsl"]   = { "vert.glsl",    h_preprocess_shader };
    handlers["frag.glsl"]   = { "frag.glsl",    h_preprocess_shader };
    handlers["comp.glsl"]   = { "comp.glsl",    h_preprocess_shader };
    handlers["ttf"]         = { "font",         h_conv_to_font      };

    if (strcmp(platform, "pc") == 0) {
        handlers["json"]    = { "json",         h_preprocess_json   };
        handlers["txt"]     = { "txt",          h_preprocess_material };
        handlers["tga"]     = { "dds",          h_conv_to_dds       };
        handlers["hdr"]     = { "dds",          h_conv_hdr_to_rgbm  };
        handlers["png"]     = { "dds",          h_conv_to_dds       };
        handlers["img"]     = { "dds",          h_conv_img_to_dds   };
    } else if (strcmp(platform, "android") == 0) {
        handlers["json"]    = { "json",         h_preprocess_json   };
        handlers["txt"]     = { "txt",          h_preprocess_material };
        handlers["tga"]     = { "ktx",          h_conv_to_astc      };
        handlers["hdr"]     = { "ktx",          h_conv_hdr_to_rgbm  };
        handlers["png"]     = { "ktx",          h_conv_to_astc      };
        handlers["img"]     = { "ktx",          h_conv_img_to_astc  };
    }

    handlers["uncompressed.tga"] = { "uncompressed.tga",  h_copy };
    handlers["uncompressed.png"] = { "uncompressed.png",  h_copy };

    auto convert_file = [out_folder, &handlers](const char *in_file) {
        const char *base_path = strchr(in_file, '/');
        if (!base_path) return;
        const char *ext = strchr(in_file, '.');
        if (!ext) return;

        ext++;

        Handler *handler = handlers.Find(ext);
        if (!handler) {
            LOGI("[PrepareAssets] No handler found for %s", in_file);
            return;
        }

        const std::string out_file =
            out_folder +
            std::string(base_path, strlen(base_path) - strlen(ext)) +
            handler->ext;

        if (CheckCanSkipAsset(in_file, out_file.c_str())) {
            return;
        }

        if (!CreateFolders(out_file.c_str())) {
            LOGI("[PrepareAssets] Failed to create directories for %s", out_file.c_str());
            return;
        }

        const auto &conv_func = handler->convert;
        conv_func(in_file, out_file.c_str());
    };

#ifdef __linux__
    if (system("chmod +x src/libs/spirv/glslangValidator") ||
        system("chmod +x src/libs/spirv/spirv-opt") ||
        system("chmod +x src/libs/spirv/spirv-cross")) {
        LOGI("[PrepareAssets] Failed to chmod executables!");
    }
#endif

    if (p_threads) {
        std::vector<std::future<void>> events;
        ReadAllFiles_MT_r(in_folder, convert_file, *p_threads, events);

        for (std::future<void> &e : events) {
            e.wait();
        }
    } else {
        ReadAllFiles_r(in_folder, convert_file);
    }

    return true;
}

bool SceneManager::WriteProbeCache(const char *out_folder, const char *scene_name, const ProbeStorage &probes,
        const CompStorage *light_probe_storage, Ren::ILog *log) {
    using namespace SceneManagerInternal;

    const int res = probes.res();
    const int temp_buf_size = 4 * res * res;
    std::unique_ptr<uint8_t[]> temp_buf(new uint8_t[temp_buf_size]);

    std::string out_file_name_base;
    out_file_name_base += out_folder;
    if (out_file_name_base.back() != '/') {
        out_file_name_base += '/';
    }
    const size_t prelude_length = out_file_name_base.length();
    out_file_name_base += scene_name;

    if (!CreateFolders(out_file_name_base.c_str())) {
        LOGE("Failed to create folders!");
        return false;
    }

    // write probes
    uint32_t cur_index = light_probe_storage->First();
    while(cur_index != 0xffffffff) {
        const auto *lprobe = (const LightProbe *)light_probe_storage->Get(cur_index);
        assert(lprobe);

        if (lprobe->layer_index != -1) {
            JsArray js_probe_faces;

            std::string out_file_name;

            for (int j = 0; j < 6; j++) {
                const int mipmap_count = probes.max_level() + 1;

                out_file_name.clear();
                out_file_name += out_file_name_base;
                out_file_name += std::to_string(lprobe->layer_index);
                out_file_name += "_";
                out_file_name += std::to_string(j);
                out_file_name += ".img";

                std::ofstream out_file(out_file_name, std::ios::binary);

                out_file.write((char *)&res, 4);
                out_file.write((char *)&mipmap_count, 4);

                for (int k = 0; k < mipmap_count; k++) {
                    const int mip_res = int((unsigned)res >> (unsigned)k);
                    const int buf_size = mip_res * mip_res * 4;

                    if (!probes.GetPixelData(k, lprobe->layer_index, j, buf_size, &temp_buf[0], log)) {
                        LOGE("Failed to read cubemap level %i layer %i face %i", k, lprobe->layer_index, j);
                        return false;
                    }

                    out_file.write((char *)&temp_buf[0], buf_size);
                }
            }
        }

        cur_index = light_probe_storage->Next(cur_index);
    }

    return true;
}

int SceneManagerInternal::ConvertToASTC(const uint8_t *image_data, int width, int height, int channels, float bitrate, std::unique_ptr<uint8_t[]> &out_buf) {
    int padding = channels == 4 ? 1 : 0;
    
    astc_codec_image *src_image = allocate_image(8, width, height, 1, padding);

    if (channels == 4) {
        uint8_t *_img = &src_image->imagedata8[0][0][0];
        for (int j = 0; j < height; j++) {
            int y = j + padding;
            for (int i = 0; i < width; i++) {
                int x = i + padding;
                src_image->imagedata8[0][y][4 * x + 0] = image_data[4 * (j * width + i) + 0];
                src_image->imagedata8[0][y][4 * x + 1] = image_data[4 * (j * width + i) + 1];
                src_image->imagedata8[0][y][4 * x + 2] = image_data[4 * (j * width + i) + 2];
                src_image->imagedata8[0][y][4 * x + 3] = image_data[4 * (j * width + i) + 3];
            }
        }
    } else {
        uint8_t *_img = &src_image->imagedata8[0][0][0];
        for (int j = 0; j < height; j++) {
            int y = j + padding;
            for (int i = 0; i < width; i++) {
                int x = i + padding;
                _img[4 * (y * width + x) + 0] = image_data[3 * (j * width + i) + 0];
                _img[4 * (y * width + x) + 1] = image_data[3 * (j * width + i) + 1];
                _img[4 * (y * width + x) + 2] = image_data[3 * (j * width + i) + 2];
                _img[4 * (y * width + x) + 3] = 255;
            }
        }
    }

    int buf_size = 0;

    {
        const float target_bitrate = bitrate;
        int xdim, ydim;

        find_closest_blockdim_2d(target_bitrate, &xdim, &ydim, 0);

        float log10_texels_2d = (std::log((float)(xdim * ydim)) / std::log(10.0f));

        // 'medium' preset params
        int plimit_autoset = 25;
        float oplimit_autoset = 1.2f;
        float mincorrel_autoset = 0.75f;
        float dblimit_autoset_2d = std::max(95 - 35 * log10_texels_2d, 70 - 19 * log10_texels_2d);
        float bmc_autoset = 75;
        int maxiters_autoset = 2;

        int pcdiv;

        switch (ydim) {
        case 4:
            pcdiv = 25;
            break;
        case 5:
            pcdiv = 15;
            break;
        case 6:
            pcdiv = 15;
            break;
        case 8:
            pcdiv = 10;
            break;
        case 10:
            pcdiv = 8;
            break;
        case 12:
            pcdiv = 6;
            break;
        default:
            pcdiv = 6;
            break;
        };

        error_weighting_params ewp;

        ewp.rgb_power = 1.0f;
        ewp.alpha_power = 1.0f;
        ewp.rgb_base_weight = 1.0f;
        ewp.alpha_base_weight = 1.0f;
        ewp.rgb_mean_weight = 0.0f;
        ewp.rgb_stdev_weight = 0.0f;
        ewp.alpha_mean_weight = 0.0f;
        ewp.alpha_stdev_weight = 0.0f;

        ewp.rgb_mean_and_stdev_mixing = 0.0f;
        ewp.mean_stdev_radius = 0;
        ewp.enable_rgb_scale_with_alpha = 0;
        ewp.alpha_radius = 0;

        ewp.block_artifact_suppression = 0.0f;
        ewp.rgba_weights[0] = 1.0f;
        ewp.rgba_weights[1] = 1.0f;
        ewp.rgba_weights[2] = 1.0f;
        ewp.rgba_weights[3] = 1.0f;
        ewp.ra_normal_angular_scale = 0;

        int partitions_to_test = plimit_autoset;
        float dblimit_2d = dblimit_autoset_2d;
        float oplimit = oplimit_autoset;
        float mincorrel = mincorrel_autoset;

        int maxiters = maxiters_autoset;
        ewp.max_refinement_iters = maxiters;

        ewp.block_mode_cutoff = (bmc_autoset) / 100.0f;

        ewp.texel_avg_error_limit = 0.0f;

        ewp.partition_1_to_2_limit = oplimit;
        ewp.lowest_correlation_cutoff = mincorrel;

        if (partitions_to_test < 1) {
            partitions_to_test = 1;
        } else if (partitions_to_test > PARTITION_COUNT) {
            partitions_to_test = PARTITION_COUNT;
        }
        ewp.partition_search_limit = partitions_to_test;

        float max_color_component_weight = std::max(std::max(ewp.rgba_weights[0], ewp.rgba_weights[1]),
                                                    std::max(ewp.rgba_weights[2], ewp.rgba_weights[3]));
        ewp.rgba_weights[0] = std::max(ewp.rgba_weights[0], max_color_component_weight / 1000.0f);
        ewp.rgba_weights[1] = std::max(ewp.rgba_weights[1], max_color_component_weight / 1000.0f);
        ewp.rgba_weights[2] = std::max(ewp.rgba_weights[2], max_color_component_weight / 1000.0f);
        ewp.rgba_weights[3] = std::max(ewp.rgba_weights[3], max_color_component_weight / 1000.0f);

        if (channels == 4) {
            ewp.enable_rgb_scale_with_alpha = 1;
            ewp.alpha_radius = 1;
        }

        ewp.texel_avg_error_limit = (float)pow(0.1f, dblimit_2d * 0.1f) * 65535.0f * 65535.0f;

        expand_block_artifact_suppression(xdim, ydim, 1, &ewp);

        swizzlepattern swz_encode = { 0, 1, 2, 3 };

        //int padding = std::max(ewp.mean_stdev_radius, ewp.alpha_radius);

        if (channels == 4 /*ewp.rgb_mean_weight != 0.0f || ewp.rgb_stdev_weight != 0.0f || ewp.alpha_mean_weight != 0.0f || ewp.alpha_stdev_weight != 0.0f*/) {
            
            compute_averages_and_variances(src_image, ewp.rgb_power, ewp.alpha_power, ewp.mean_stdev_radius, ewp.alpha_radius, swz_encode);
        }

        int xsize = src_image->xsize;
        int ysize = src_image->ysize;

        int xblocks = (xsize + xdim - 1) / xdim;
        int yblocks = (ysize + ydim - 1) / ydim;
        int zblocks = 1;

        buf_size = xblocks * yblocks * zblocks * 16;
        out_buf.reset(new uint8_t[buf_size]);

        encode_astc_image(src_image, nullptr, xdim, ydim, 1, &ewp, DECODE_LDR, swz_encode, swz_encode, &out_buf[0], 0, 8);
    }

    destroy_image(src_image);

    return buf_size;
}

std::unique_ptr<uint8_t[]> SceneManagerInternal::DecodeASTC(const uint8_t *image_data, int data_size, int xdim, int ydim, int width, int height) {
    int xsize = width;
    int ysize = height;
    int zsize = 1;

    int xblocks = (xsize + xdim - 1) / xdim;
    int yblocks = (ysize + ydim - 1) / ydim;
    int zblocks = 1;
    
    if (!g_astc_initialized) {
        test_inappropriate_extended_precision();
        prepare_angular_tables();
        build_quantization_mode_table();
        g_astc_initialized = true;
    }

    astc_codec_image *img = allocate_image(8, xsize, ysize, 1, 0);
    initialize_image(img);

    swizzlepattern swz_decode = { 0, 1, 2, 3 };

    imageblock pb;
    for (int z = 0; z < zblocks; z++) {
        for (int y = 0; y < yblocks; y++) {
            for (int x = 0; x < xblocks; x++) {
                int offset = (((z * yblocks + y) * xblocks) + x) * 16;
                const uint8_t *bp = image_data + offset;

                physical_compressed_block pcb;
                memcpy(&pcb, bp, sizeof(physical_compressed_block));

                symbolic_compressed_block scb;
                physical_to_symbolic(xdim, ydim, 1, pcb, &scb);
                decompress_symbolic_block(DECODE_LDR, xdim, ydim, 1, x * xdim, y * ydim, z * 1, &scb, &pb);
                write_imageblock(img, &pb, xdim, ydim, 1, x * xdim, y * ydim, z * 1, swz_decode);
            }
        }
    }

    std::unique_ptr<uint8_t[]> ret_data;
    ret_data.reset(new uint8_t[xsize * ysize * 4]);

    memcpy(&ret_data[0], &img->imagedata8[0][0][0], xsize * ysize * 4);

    destroy_image(img);

    return ret_data;
}

std::unique_ptr<uint8_t[]> SceneManagerInternal::Decode_KTX_ASTC(const uint8_t *image_data, int data_size, int &width, int &height) {
    Ren::KTXHeader header;
    memcpy(&header, &image_data[0], sizeof(Ren::KTXHeader));

    width = (int)header.pixel_width;
    height = (int)header.pixel_height;

    int data_offset = sizeof(Ren::KTXHeader);

    {   // Decode first mip level
        uint32_t img_size;
        memcpy(&img_size, &image_data[data_offset], sizeof(uint32_t));
        data_offset += sizeof(uint32_t);

        const uint32_t gl_compressed_rgba_astc_4x4_khr = 0x93B0;
        const uint32_t gl_compressed_rgba_astc_6x6_khr = 0x93B4;
        const uint32_t gl_compressed_rgba_astc_8x8_khr = 0x93B7;

        int xdim, ydim;

        if (header.gl_internal_format == gl_compressed_rgba_astc_4x4_khr) {
            xdim = 4;
            ydim = 4;
        } else if (header.gl_internal_format == gl_compressed_rgba_astc_6x6_khr) {
            xdim = 6;
            ydim = 6;
        } else if (header.gl_internal_format == gl_compressed_rgba_astc_8x8_khr) {
            xdim = 8;
            ydim = 8;
        } else {
            throw std::runtime_error("Unsupported block size!");
        }

        return DecodeASTC(&image_data[data_offset], img_size, xdim, ydim, width, height);
    }
}