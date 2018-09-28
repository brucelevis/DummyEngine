#pragma once

#include <Ren/Texture.h>
#include <Sys/Optional.h>

struct FrameBuf {
    Ren::eTexColorFormat col_format;
    int w = -1, h = -1, msaa = 0;

#if defined(USE_GL_RENDER)
    uint32_t fb;
    Sys::Optional<uint32_t> depth_rb, col_tex, depth_tex;
#endif
    FrameBuf() : col_format(Ren::Undefined), w(-1), h(-1) {}
    FrameBuf(int w, int h, Ren::eTexColorFormat col_format, Ren::eTexFilter filter,
                Ren::eTexRepeat repeat, bool with_depth = true, int msaa = 1);
    ~FrameBuf();

    FrameBuf(const FrameBuf &rhs) = delete;
    FrameBuf &operator=(const FrameBuf &rhs) = delete;
    FrameBuf(FrameBuf &&rhs);
    FrameBuf &operator=(FrameBuf &&rhs);
};
