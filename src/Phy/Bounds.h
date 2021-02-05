#pragma once

#include "Core.h"

namespace Phy {
class Bounds {
  public:
    Bounds() { Clear(); }
    //Bounds(const Bounds &rhs) = default;

    void Clear();
    void Expand(const Vec3 pts[], int count);
    void Expand(const Vec3 &pt);
    void Expand(const Bounds &rhs);

    real width(const int i) const { return maxs[i] - mins[i]; }

    Vec3 mins;
    Vec3 maxs;
};

inline void Bounds::Clear() {
    mins = Vec3{ std::numeric_limits<real>::max() };
    maxs = Vec3{ std::numeric_limits<real>::lowest() };
}

inline void Bounds::Expand(const Vec3 pts[], const int count) {
    for (int i = 0; i < count; i++) {
        Expand(pts[i]);
    }
}

inline void Bounds::Expand(const Vec3& pt) {
    mins = Min(mins, pt);
    maxs = Max(maxs, pt);
}

inline void Bounds::Expand(const Bounds& rhs) {
    mins = Min(mins, rhs.mins);
    maxs = Max(maxs, rhs.maxs);
}

inline bool Intersect(const Bounds& b1, const Bounds& b2) {
    if (b1.maxs[0] < b2.mins[0] || b1.maxs[1] < b2.mins[1] || b1.maxs[2] < b2.mins[2]) {
        return false;
    }
    if (b2.maxs[0] < b1.mins[0] || b2.maxs[1] < b1.mins[1] || b2.maxs[2] < b1.mins[2]) {
        return false;
    }
    return true;
}
} // namespace Phy