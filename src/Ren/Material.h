#pragma once

#include <cstdint>

#include <functional>

#include "Fwd.h"
#include "Program.h"
#include "Storage.h"
#include "String.h"
#include "Texture.h"

#include "MVec.h"

namespace Ren {
class ILog;

enum class eMaterialFlags {
    AlphaTest = (1u << 0u),
    AlphaBlend = (1u << 1u),
    DepthWrite = (1u << 2u),
    TwoSided = (1u << 3u),
    TaaResponsive = (1u << 4u)
};

const int MaxMaterialProgramCount = 4;
const int MaxMaterialTextureCount = 8;
const int MaxMaterialParamCount = 8;

enum class eMatLoadStatus { Found, SetToDefault, CreatedFromData };

typedef std::function<Tex2DRef(const char *name, uint32_t flags)>
    texture_load_callback;
typedef std::function<ProgramRef(const char *name, const char *arg1, const char *arg2,
                                 const char *arg3, const char *arg4)>
    program_load_callback;

class Material : public RefCounter {
    uint32_t flags_ = 0;
    bool ready_ = false;
    String name_;

    void InitFromTXT(const char *mat_src, eMatLoadStatus *status,
                     const program_load_callback &on_prog_load,
                     const texture_load_callback &on_tex_load, ILog *log);

  public:
    ProgramRef programs[MaxMaterialProgramCount];
    Tex2DRef textures[MaxMaterialTextureCount];
    Vec4f params[MaxMaterialParamCount];
    int params_count = 0;

    Material() = default;
    Material(const char *name, const char *mat_src, eMatLoadStatus *status,
             const program_load_callback &on_prog_load,
             const texture_load_callback &on_tex_load, ILog *log);
    Material(const char *name, uint32_t flags, ProgramRef programs[],
             Tex2DRef textures[], const Vec4f params[], ILog *log);

    Material(const Mesh &rhs) = delete;
    Material(Material &&rhs) noexcept { *this = std::move(rhs); }

    Material &operator=(const Material &rhs) = delete;
    Material &operator=(Material &&rhs) noexcept;

    uint32_t flags() const { return flags_; }
    bool ready() const { return ready_; }
    const String &name() const { return name_; }

    void Init(uint32_t flags, ProgramRef _programs[], Tex2DRef _textures[],
              const Vec4f _params[], ILog *log);
    void Init(const char *mat_src, eMatLoadStatus *status,
              const program_load_callback &on_prog_load,
              const texture_load_callback &on_tex_load, ILog *log);
};

// typedef StrongRef<Material> MaterialRef;
typedef Storage<Material> MaterialStorage;
} // namespace Ren
