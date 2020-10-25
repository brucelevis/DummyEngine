#include "BitmapFont.h"

#include <cassert>
#include <cstring>

#include <Sys/AssetFile.h>

#include "BaseElement.h"
#include "Renderer.h"
#include "Utils.h"

namespace BitmapFontInternal {}

Gui::BitmapFont::BitmapFont(const char *name, Ren::Context *ctx)
    : info_{}, scale_(1.0f), tex_res_{} {
    if (name && ctx) {
        this->Load(name, *ctx);
    }
}

float Gui::BitmapFont::height(const BaseElement *parent) const {
    return 2.0f * scale_ * float(info_.line_height) / parent->size_px()[1];
}

bool Gui::BitmapFont::Load(const char *fname, Ren::Context &ctx) {
    Sys::AssetFile in_file(fname, Sys::eOpenMode::In);
    if (!in_file) {
        return false;
    }

    char sign[4];
    if (!in_file.Read(sign, 4) || sign[0] != 'F' || sign[1] != 'O' || sign[2] != 'N' ||
        sign[3] != 'T') {
        return false;
    }

    uint32_t header_size;
    if (!in_file.Read((char *)&header_size, sizeof(uint32_t))) {
        return false;
    }

    const uint32_t expected_chunks_size =
        uint32_t(Gui::eFontFileChunk::FontChCount) * 3 * sizeof(uint32_t);
    const uint32_t chunks_size = header_size - 4 - sizeof(uint32_t);
    if (chunks_size != expected_chunks_size) {
        return false;
    }

    for (uint32_t i = 0; i < chunks_size; i += 3 * sizeof(uint32_t)) {
        uint32_t chunk_id, chunk_off, chunk_size;
        if (!in_file.Read((char *)&chunk_id, sizeof(uint32_t)) ||
            !in_file.Read((char *)&chunk_off, sizeof(uint32_t)) ||
            !in_file.Read((char *)&chunk_size, sizeof(uint32_t))) {
            return false;
        }

        const size_t old_pos = in_file.pos();
        in_file.SeekAbsolute(chunk_off);

        if (chunk_id == uint32_t(Gui::eFontFileChunk::FontChTypoData)) {
            if (!in_file.Read((char *)&info_, sizeof(typgraph_info_t))) {
                return false;
            }
        } else if (chunk_id == uint32_t(Gui::eFontFileChunk::FontChImageData)) {
            uint16_t img_data_w, img_data_h;
            if (!in_file.Read((char *)&img_data_w, sizeof(uint16_t)) ||
                !in_file.Read((char *)&img_data_h, sizeof(uint16_t))) {
                return false;
            }

            tex_res_[0] = img_data_w;
            tex_res_[1] = img_data_h;

            uint16_t draw_mode, blend_mode;
            if (!in_file.Read((char *)&draw_mode, sizeof(uint16_t)) ||
                !in_file.Read((char *)&blend_mode, sizeof(uint16_t))) {
                return false;
            }

            draw_mode_ = (Gui::eDrawMode)draw_mode;
            blend_mode_ = (Gui::eBlendMode)blend_mode;

            const int img_data_size = 4 * img_data_w * img_data_h;
            std::unique_ptr<uint8_t[]> img_data(new uint8_t[img_data_size]);

            if (!in_file.Read((char *)img_data.get(), img_data_size)) {
                return false;
            }

            Ren::Texture2DParams p;
            p.w = img_data_w;
            p.h = img_data_h;
            p.filter = draw_mode_ == eDrawMode::DrPassthrough
                           ? Ren::eTexFilter::NoFilter
                           : Ren::eTexFilter::BilinearNoMipmap;
            p.repeat = Ren::eTexRepeat::ClampToBorder;
            p.format = Ren::eTexFormat::RawRGBA8888;

            tex_ =
                ctx.LoadTextureRegion(fname, img_data.get(), img_data_size, p, nullptr);
        } else if (chunk_id == uint32_t(Gui::eFontFileChunk::FontChGlyphData)) {
            if (!in_file.Read((char *)&glyph_range_count_, sizeof(uint32_t))) {
                return false;
            }

            glyph_ranges_.reset(new glyph_range_t[glyph_range_count_]);

            if (!in_file.Read((char *)glyph_ranges_.get(),
                              glyph_range_count_ * sizeof(glyph_range_t))) {
                return false;
            }

            glyphs_count_ = 0;
            for (uint32_t j = 0; j < glyph_range_count_; j++) {
                glyphs_count_ += (glyph_ranges_[j].end - glyph_ranges_[j].beg);
            }

            glyphs_.reset(new glyph_info_t[glyphs_count_]);

            if (!in_file.Read((char *)glyphs_.get(),
                              glyphs_count_ * sizeof(glyph_info_t))) {
                return false;
            }
        }

        in_file.SeekAbsolute(old_pos);
    }

    return true;
}

float Gui::BitmapFont::GetWidth(const char *text, int text_len,
                                const BaseElement *parent) const {
    int cur_x = 0;

    const glyph_range_t *glyph_ranges = glyph_ranges_.get();
    const glyph_info_t *glyphs = glyphs_.get();

    if (text_len == -1)
        text_len = std::numeric_limits<int>::max();

    int char_pos = 0;
    while (text[char_pos] && text_len--) {
        uint32_t unicode;
        char_pos += Gui::ConvChar_UTF8_to_Unicode(&text[char_pos], unicode);

        uint32_t glyph_index = 0;
        for (uint32_t i = 0; i < glyph_range_count_; i++) {
            const glyph_range_t &rng = glyph_ranges[i];

            if (unicode >= rng.beg && unicode < rng.end) {
                glyph_index += (unicode - rng.beg);
                break;
            } else {
                glyph_index += (rng.end - rng.beg);
            }
        }
        assert(glyph_index < glyphs_count_);

        const glyph_info_t &glyph = glyphs[glyph_index];
        cur_x += glyph.adv[0];
    }

    const float mul = scale_ * parent->size()[0] / (float)parent->size_px()[0];
    return float(cur_x) * mul;
}

float Gui::BitmapFont::DrawText(Renderer *r, const char *text, const Vec2f &pos,
                                const uint8_t col[4], const BaseElement *parent) const {
    using namespace BitmapFontInternal;

    const glyph_range_t *glyph_ranges = glyph_ranges_.get();
    const glyph_info_t *glyphs = glyphs_.get();

    const Vec2f p = parent->pos() + 0.5f * (pos + Vec2f(1, 1)) * parent->size(),
                m = scale_ * parent->size() / (Vec2f)parent->size_px();

    const uint16_t uvs_offset[2] = {(uint16_t)tex_->pos(0), (uint16_t)tex_->pos(1)},
                   tex_layer = f32_to_u16((1.0f / 16.0f) * float(tex_->pos(2)));

    uint16_t u16_draw_mode = 0;
    if (draw_mode_ == eDrawMode::DrDistanceField) {
        u16_draw_mode = 32727;
    } else if (draw_mode_ == eDrawMode::DrBlitDistanceField) {
        u16_draw_mode = 65535;
    }

    vertex_t *vtx_data;
    int vtx_avail = 0;
    uint16_t *ndx_data;
    int ndx_avail = 0;
    int ndx_offset = r->AcquireVertexData(&vtx_data, &vtx_avail, &ndx_data, &ndx_avail);

    vertex_t *cur_vtx = vtx_data;
    uint16_t *cur_ndx = ndx_data;

    int cur_x = 0;

    const Vec2f uvs_scale =
        1.0f / Vec2f{(float)Ren::TextureAtlasWidth, (float)Ren::TextureAtlasHeight};

    const Vec2f *clip = r->GetClipArea();

    int char_pos = 0;
    while (text[char_pos]) {
        uint32_t unicode;
        char_pos += Gui::ConvChar_UTF8_to_Unicode(&text[char_pos], unicode);

        uint32_t glyph_index = 0;
        for (uint32_t i = 0; i < glyph_range_count_; i++) {
            const glyph_range_t &rng = glyph_ranges[i];

            if (unicode >= rng.beg && unicode < rng.end) {
                glyph_index += (unicode - rng.beg);
                break;
            } else {
                glyph_index += (rng.end - rng.beg);
            }
        }
        assert(glyph_index < glyphs_count_);

        const glyph_info_t &glyph = glyphs[glyph_index];
        if (glyph.res[0]) {
            Vec4f pos_uvs[2] = {
                Vec4f{p[0] + float(cur_x + glyph.off[0] - 1) * m[0],
                      p[1] + float(glyph.off[1] - 1) * m[1],
                      uvs_scale[0] * float(uvs_offset[0] + glyph.pos[0] - 1),
                      uvs_scale[1] * float(uvs_offset[1] + glyph.pos[1] + glyph.res[1] + 1)},
                Vec4f{p[0] + float(cur_x + glyph.off[0] + glyph.res[0] + 1) * m[0],
                      p[1] + float(glyph.off[1] + glyph.res[1] + 1) * m[1],
                      uvs_scale[0] * float(uvs_offset[0] + glyph.pos[0] + glyph.res[0] + 1),
                      uvs_scale[1] * float(uvs_offset[1] + glyph.pos[1] - 1)}};
            if (clip && !ClipQuadToArea(pos_uvs, clip)) {
                cur_x += glyph.adv[0];
                continue;
            }

            vtx_avail -= 4;
            ndx_avail -= 6;
            assert(vtx_avail > 0 && ndx_avail > 0);

            cur_vtx->pos[0] = pos_uvs[0][0];
            cur_vtx->pos[1] = pos_uvs[0][1];
            cur_vtx->pos[2] = 0.0f;
            memcpy(cur_vtx->col, col, 4);
            cur_vtx->uvs[0] = f32_to_u16(pos_uvs[0][2]);
            cur_vtx->uvs[1] = f32_to_u16(pos_uvs[0][3]);
            cur_vtx->uvs[2] = tex_layer;
            cur_vtx->uvs[3] = u16_draw_mode;
            ++cur_vtx;

            cur_vtx->pos[0] = pos_uvs[1][0];
            cur_vtx->pos[1] = pos_uvs[0][1];
            cur_vtx->pos[2] = 0.0f;
            memcpy(cur_vtx->col, col, 4);
            cur_vtx->uvs[0] = f32_to_u16(pos_uvs[1][2]);
            cur_vtx->uvs[1] = f32_to_u16(pos_uvs[0][3]);
            cur_vtx->uvs[2] = tex_layer;
            cur_vtx->uvs[3] = u16_draw_mode;
            ++cur_vtx;

            cur_vtx->pos[0] = pos_uvs[1][0];
            cur_vtx->pos[1] = pos_uvs[1][1];
            cur_vtx->pos[2] = 0.0f;
            memcpy(cur_vtx->col, col, 4);
            cur_vtx->uvs[0] = f32_to_u16(pos_uvs[1][2]);
            cur_vtx->uvs[1] = f32_to_u16(pos_uvs[1][3]);
            cur_vtx->uvs[2] = tex_layer;
            cur_vtx->uvs[3] = u16_draw_mode;
            ++cur_vtx;

            cur_vtx->pos[0] = pos_uvs[0][0];
            cur_vtx->pos[1] = pos_uvs[1][1];
            cur_vtx->pos[2] = 0.0f;
            memcpy(cur_vtx->col, col, 4);
            cur_vtx->uvs[0] = f32_to_u16(pos_uvs[0][2]);
            cur_vtx->uvs[1] = f32_to_u16(pos_uvs[1][3]);
            cur_vtx->uvs[2] = tex_layer;
            cur_vtx->uvs[3] = u16_draw_mode;
            ++cur_vtx;

            (*cur_ndx++) = ndx_offset + 0;
            (*cur_ndx++) = ndx_offset + 1;
            (*cur_ndx++) = ndx_offset + 2;

            (*cur_ndx++) = ndx_offset + 0;
            (*cur_ndx++) = ndx_offset + 2;
            (*cur_ndx++) = ndx_offset + 3;

            ndx_offset += 4;
        }

        cur_x += glyph.adv[0];
    }

    r->SubmitVertexData(int(cur_vtx - vtx_data), int(cur_ndx - ndx_data));

    return float(cur_x) * m[0];
}

int Gui::BitmapFont::CheckText(const char *text, const Vec2f &pos, const Vec2f &press_pos,
                               float &out_char_offset, const BaseElement *parent) const {
    using namespace BitmapFontInternal;

    const glyph_range_t *glyph_ranges = glyph_ranges_.get();
    const glyph_info_t *glyphs = glyphs_.get();

    const Vec2f p = parent->pos() + 0.5f * (pos + Vec2f(1, 1)) * parent->size(),
                m = scale_ * parent->size() / (Vec2f)parent->size_px();

    int cur_x = 0;
    int char_index = 0;

    int char_pos = 0;
    while (text[char_pos]) {
        uint32_t unicode;
        char_pos += Gui::ConvChar_UTF8_to_Unicode(&text[char_pos], unicode);

        uint32_t glyph_index = 0;
        for (uint32_t i = 0; i < glyph_range_count_; i++) {
            const glyph_range_t &rng = glyph_ranges[i];

            if (unicode >= rng.beg && unicode < rng.end) {
                glyph_index += (unicode - rng.beg);
                break;
            } else {
                glyph_index += (rng.end - rng.beg);
            }
        }
        assert(glyph_index < glyphs_count_);

        const glyph_info_t &glyph = glyphs[glyph_index];
        if (glyph.res[0]) {
            const float corners[2][2]{
                {p[0] + float(cur_x + glyph.off[0] - 1) * m[0],
                 p[1] + float(glyph.off[1] - 1) * m[1]},
                {p[0] + float(cur_x + glyph.off[0] + glyph.res[0] + 1) * m[0],
                 p[1] + float(glyph.off[1] + glyph.res[1] + 1) * m[1]}};

            if (press_pos[0] >= corners[0][0] &&
                press_pos[0] <= corners[1][0] /*&&
                press_pos[1] >= corners[0][1] &&
                press_pos[1] <= corners[1][1]*/) {
                out_char_offset = float(cur_x) * m[0];
                return char_index;
            }
        }

        cur_x += glyph.adv[0];
        char_index++;
    }

    return -1;
}