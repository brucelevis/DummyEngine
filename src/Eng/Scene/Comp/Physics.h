#pragma once

#include <Phy/Body.h>

#include "Common.h"

struct Physics {
    Phy::Body body;

    static void Read(const JsObject &js_in, Physics &ph);
    static void Write(const Physics &ph, JsObject &js_out);

    static const char *name() { return "physics"; }
};