#include "DummyApp.h"

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#include <html5.h>
#endif

#if defined(USE_GL_RENDER)
#include <Ren/GL.h>
#elif defined(USE_SW_RENDER)

#endif

#if !defined(__ANDROID__)
#include <SDL2/SDL.h>
#include <SDL2/SDL_keyboard.h>
#include <SDL2/SDL_video.h>
#include <SDL2/SDL_events.h>
#endif

#include <Eng/GameBase.h>
#include <Eng/TimedInput.h>
#include <Sys/DynLib.h>
#include <Sys/Log.h>
#include <Sys/Time_.h>

#include "../DummyLib/Viewer.h"

namespace {
    DummyApp *g_app = nullptr;
}

extern "C" {
    // Enable High Performance Graphics while using Integrated Graphics
    DLL_EXPORT int32_t NvOptimusEnablement = 0x00000001;        // Nvidia
    DLL_EXPORT int AmdPowerXpressRequestHighPerformance = 1;    // AMD
}

DummyApp::DummyApp() : quit_(false) {
    g_app = this;
}

DummyApp::~DummyApp() {

}

int DummyApp::Init(int w, int h) {
#if !defined(__ANDROID__)
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        return -1;
    }

#if defined(USE_GL_RENDER)
    // This needs to be done before window creation
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);

    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 0);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
#endif

    uint32_t flags = fullscreen_ ? (SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_FULLSCREEN) : (SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE); 
    window_ = SDL_CreateWindow("View", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, w, h, flags);

#if defined(__EMSCRIPTEN__)
    emscripten_set_resize_callback(nullptr, nullptr, true,
    [](int event_type, const EmscriptenUiEvent *ui_event, void *user_data) -> int {
        if (event_type == EMSCRIPTEN_EVENT_RESIZE) {
            int new_w = ui_event->documentBodyClientWidth,
            new_h = (new_w / 16) * 9;
            SDL_SetWindowSize(g_app->window_, new_w, new_h);
        }
        return EMSCRIPTEN_RESULT_SUCCESS;
    });

    emscripten_set_fullscreenchange_callback(nullptr, nullptr, true,
    [](int event_type, const EmscriptenFullscreenChangeEvent *fullscreen_change_event, void *user_data) -> int {
        if (event_type == EMSCRIPTEN_EVENT_FULLSCREENCHANGE) {
            //int new_w = fullscreen_change_event->screenWidth,
            //    new_h = (new_w / 16) * 9;
            int new_h = fullscreen_change_event->screenHeight,
            new_w = (new_w / 9) * 16;
            SDL_SetWindowSize(g_app->window_, new_w, new_h);
        }

        return EMSCRIPTEN_RESULT_SUCCESS;
    });
#endif

#if defined(USE_GL_RENDER)
    gl_ctx_ = SDL_GL_CreateContext(window_);
#if !defined(__EMSCRIPTEN__)
    SDL_GL_SetSwapInterval(1);
#endif
#elif defined(USE_SW_RENDER)
    renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer_) {
        const char *s = SDL_GetError();
        printf("%s\n", s);
    }
    texture_ = SDL_CreateTexture(renderer_, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, w, h);
    if (!texture_) {
        const char *s = SDL_GetError();
        printf("%s\n", s);
    }
#endif
#endif
    try {
#if defined(WIN32)
        // TODO: make it work on linux
        Viewer::PrepareAssets("pc");
#endif
        viewer_.reset(new Viewer(w, h, nullptr));
    } catch (std::exception &e) {
        fprintf(stderr, "%s", e.what());
        return -1;
    }

    return 0;
}

void DummyApp::Destroy() {
    viewer_.reset();

#if !defined(__ANDROID__)
#if defined(USE_GL_RENDER)
    SDL_GL_DeleteContext(gl_ctx_);
#elif defined(USE_SW_RENDER)
    SDL_DestroyTexture(texture_);
    SDL_DestroyRenderer(renderer_);
#endif
    SDL_DestroyWindow(window_);
    SDL_Quit();
#endif
}

void DummyApp::Frame() {
    viewer_->Frame();
}

void DummyApp::Resize(int w, int h) {
    viewer_->Resize(w, h);
}

void DummyApp::AddEvent(int type, int key, int raw_key, float x, float y, float dx, float dy) {
    auto input_manager = viewer_->GetComponent<InputManager>(INPUT_MANAGER_KEY);
    if (!input_manager) return;

    InputManager::Event evt;
    evt.type = (InputManager::RawInputEvent)type;
    evt.key = (InputManager::RawInputButton)key;
    evt.raw_key = raw_key;
    evt.point.x = x;
    evt.point.y = y;
    evt.move.dx = dx;
    evt.move.dy = dy;
    evt.time_stamp = Sys::GetTimeUs();

    input_manager->AddRawInputEvent(evt);
}

#if !defined(__ANDROID__)
int DummyApp::Run(const std::vector<std::string> &args) {
    int w = 1024, h = 576;
    fullscreen_ = false;

    int args_count = (int)args.size();
    for (int i = 0; i < args_count; i++) {
        const std::string &arg = args[i];
        if (arg == "--prepare_assets") {
            Viewer::PrepareAssets(args[i + 1].c_str());
            i++;
        } else if (arg == "--norun") {
            return 0;
        } else if ((arg == "--width" || arg == "-w") && i < args_count) {
            w = std::atoi(args[++i].c_str());   
        } else if ((arg == "--height" || arg == "-h") && i < args_count) {
            h = std::atoi(args[++i].c_str());
        } else if ((arg == "--fullscreen") || (arg == "-fs")) {
            fullscreen_ = true;
        }
    }

    if (Init(w, h) < 0) {
        return -1;
    }

    std::vector<uint8_t> u8_pixel_data(w * h * 4);

#if defined(__EMSCRIPTEN__)
    emscripten_set_main_loop([]() {
        g_app->PollEvents();
        g_app->Frame();
    }, 0, 0);
#else
    while (!terminated()) {
        this->PollEvents();

        this->Frame();

#if defined(USE_GL_RENDER)
        uint64_t swap_start = Sys::GetTimeUs();
        SDL_GL_SwapWindow(window_);
        uint64_t swap_end = Sys::GetTimeUs();

        auto swap_interval = viewer_->GetComponent<TimeInterval>(SWAP_TIMER_KEY);
        if (swap_interval) {
            swap_interval->start_timepoint_us = swap_start;
            swap_interval->end_timepoint_us = swap_end;
        }
#elif defined(USE_SW_RENDER)
        SDL_UpdateTexture(texture_, NULL, p_get_renderer_pixels_(viewer_.get()), viewer_->width * sizeof(Uint32));

        //SDL_RenderClear(renderer_);
        SDL_RenderCopy(renderer_, texture_, NULL, NULL);
        SDL_RenderPresent(renderer_);
#endif
    }

    this->Destroy();
#endif
    return 0;
}

bool DummyApp::ConvertToRawButton(int &raw_key, InputManager::RawInputButton &button) {
    switch (raw_key) {
    case SDLK_UP:
        button = InputManager::RAW_INPUT_BUTTON_UP;
        break;
    case SDLK_DOWN:
        button = InputManager::RAW_INPUT_BUTTON_DOWN;
        break;
    case SDLK_LEFT:
        button = InputManager::RAW_INPUT_BUTTON_LEFT;
        break;
    case SDLK_RIGHT:
        button = InputManager::RAW_INPUT_BUTTON_RIGHT;
        break;
    case SDLK_ESCAPE:
        button = InputManager::RAW_INPUT_BUTTON_EXIT;
        break;
    case SDLK_TAB:
        button = InputManager::RAW_INPUT_BUTTON_TAB;
        break;
    case SDLK_BACKSPACE:
        button = InputManager::RAW_INPUT_BUTTON_BACKSPACE;
        break;
    case SDLK_LSHIFT:
    case SDLK_RSHIFT:
        button = InputManager::RAW_INPUT_BUTTON_SHIFT;
        break;
    case SDLK_DELETE:
        button = InputManager::RAW_INPUT_BUTTON_DELETE;
        break;
    case SDLK_SPACE:
        button = InputManager::RAW_INPUT_BUTTON_SPACE;
        break;
    case SDLK_RETURN:
        button = InputManager::RAW_INPUT_BUTTON_RETURN;
        break;
    default:
        button = InputManager::RAW_INPUT_BUTTON_OTHER;
        break;
    }
    return true;
}

void DummyApp::PollEvents() {
    auto input_manager = viewer_->GetComponent<InputManager>(INPUT_MANAGER_KEY);
    if (!input_manager) return;

    SDL_Event e;
    InputManager::RawInputButton button;
    InputManager::Event evt;
    while (SDL_PollEvent(&e)) {
        evt.type = InputManager::RAW_INPUT_NONE;
        switch (e.type) {
        case SDL_KEYDOWN: {
            if (e.key.keysym.sym == SDLK_ESCAPE) {
                quit_ = true;
                return;
            } /*else if (e.key.keysym.sym == SDLK_TAB) {
bool is_fullscreen = bool(SDL_GetWindowFlags(window_) & SDL_WINDOW_FULLSCREEN);
SDL_SetWindowFullscreen(window_, is_fullscreen ? 0 : SDL_WINDOW_FULLSCREEN);
return;
}*/ else if (ConvertToRawButton(e.key.keysym.sym, button)) {
                evt.type = InputManager::RAW_INPUT_KEY_DOWN;
                evt.key = button;
                evt.raw_key = e.key.keysym.sym;
            }
        }
        break;
        case SDL_KEYUP:
            if (ConvertToRawButton(e.key.keysym.sym, button)) {
                evt.type = InputManager::RAW_INPUT_KEY_UP;
                evt.key = button;
                evt.raw_key = e.key.keysym.sym;
            }
            break;
        case SDL_FINGERDOWN:
            evt.type = e.tfinger.fingerId == 0 ? InputManager::RAW_INPUT_P1_DOWN : InputManager::RAW_INPUT_P2_DOWN;
            evt.point.x = e.tfinger.x * viewer_->width;
            evt.point.y = e.tfinger.y * viewer_->height;
            break;
        case SDL_MOUSEBUTTONDOWN:
            evt.type = InputManager::RAW_INPUT_P1_DOWN;
            evt.point.x = (float) e.motion.x;
            evt.point.y = (float) e.motion.y;
            break;
        case SDL_FINGERUP:
            evt.type = e.tfinger.fingerId == 0 ? InputManager::RAW_INPUT_P1_UP : InputManager::RAW_INPUT_P2_UP;
            evt.point.x = e.tfinger.x * viewer_->width;
            evt.point.y = e.tfinger.y * viewer_->height;
            break;
        case SDL_MOUSEBUTTONUP:
            evt.type = InputManager::RAW_INPUT_P1_UP;
            evt.point.x = (float) e.motion.x;
            evt.point.y = (float) e.motion.y;
            break;
        case SDL_QUIT: {
            quit_ = true;
            return;
        }
        case SDL_FINGERMOTION:
            evt.type = e.tfinger.fingerId == 0 ? InputManager::RAW_INPUT_P1_MOVE : InputManager::RAW_INPUT_P2_MOVE;
            evt.point.x = e.tfinger.x * viewer_->width;
            evt.point.y = e.tfinger.y * viewer_->height;
            evt.move.dx = e.tfinger.dx * viewer_->width;
            evt.move.dy = e.tfinger.dy * viewer_->height;
            break;
        case SDL_MOUSEMOTION:
            evt.type = InputManager::RAW_INPUT_P1_MOVE;
            evt.point.x = (float) e.motion.x;
            evt.point.y = (float) e.motion.y;
            evt.move.dx = (float) e.motion.xrel;
            evt.move.dy = (float) e.motion.yrel;
            break;
        case SDL_WINDOWEVENT:
            if (e.window.event == SDL_WINDOWEVENT_RESIZED || e.window.event == SDL_WINDOWEVENT_SIZE_CHANGED) {
                evt.type = InputManager::RAW_INPUT_RESIZE;
                evt.point.x = (float)e.window.data1;
                evt.point.y = (float)e.window.data2;

                // TODO: ???
#if defined(__EMSCRIPTEN__)
                emscripten_set_canvas_size(e.window.data1, e.window.data2);
#endif
                Resize(e.window.data1, e.window.data2);
#if defined(USE_SW_RENDER)
                SDL_RenderPresent(renderer_);

                SDL_DestroyTexture(texture_);
                texture_ = SDL_CreateTexture(renderer_, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING,
                                             e.window.data1, e.window.data2);
#endif
            }
            break;
        default:
            return;
        }
        if (evt.type != InputManager::RAW_INPUT_NONE) {
            evt.time_stamp = Sys::GetTimeUs() - 1000 * (SDL_GetTicks() - e.common.timestamp);
            input_manager->AddRawInputEvent(evt);
        }
    }
}

#endif