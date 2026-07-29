#pragma once
#include <cmath>
#include <fstream>
#include "Point3d.h"

struct GeometryPoint {
    uint8_t id;
    float x, y, z;

    float angle_radians = 0.0f;   // bound direction (perpendicular to road heading)
    float lbound_offset = 1.75f;  // absolute distance from center to lBound
    float rbound_offset = 1.75f;  // absolute distance from center to rBound

    // Computed boundary points (derived from angle + offsets, not serialized)
    Point3d lBound, rBound;

    GeometryPoint() : x(0), y(0), z(0) {}
    GeometryPoint(float x, float y, float z) : x(x), y(y), z(z) {}

    void recomputeBounds() {
        float sa = std::sin(angle_radians);
        float ca = std::cos(angle_radians);
        lBound = { x - sa * lbound_offset, y + ca * lbound_offset, z };
        rBound = { x + sa * rbound_offset, y - ca * rbound_offset, z };
    }
};