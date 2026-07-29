#pragma once

#include "Point3d.h"
#include "AppConstants.h"
#include <cmath>
#include <cstdint>

struct RoadNode {
    uint32_t id                  = INVALID_ID;
    float    x                   = 0.0f;
    float    y                   = 0.0f;
    float    z                   = 0.0f;
    float    half_width_left_m   = 1.75f;
    float    half_width_right_m  = 1.75f;

    float    bound_angle_radians = 0.0f;
    uint32_t junction_id         = INVALID_ID;
    uint32_t segment_id          = INVALID_ID;
    uint32_t parking_spot_id     = INVALID_ID;
    uint8_t  flags               = 0;

    Point3d lBound, rBound;

    bool    isJunctionNode()      const { return (flags & 0x01) != 0; }
    bool    isParkingTerminal()   const { return (flags & 0x04) != 0; }
    void    setParkingTerminal(bool v)  { if (v) flags |= 0x04; else flags &= ~0x04; }
    Point3d center()              const { return Point3d(x, y, z); }

    void recomputeBounds() {
        float cx = std::cos(bound_angle_radians);
        float sx = std::sin(bound_angle_radians);
        lBound = Point3d(x - sx * half_width_left_m,  y + cx * half_width_left_m,  z);
        rBound = Point3d(x + sx * half_width_right_m, y - cx * half_width_right_m, z);
    }

    // Serialization
    struct SerialRecord {
        uint32_t id;
        float    x, y, z;
        float    half_width_left_m, half_width_right_m;
        float    bound_angle_radians;
        uint32_t junction_id;
        uint8_t  flags;
        uint8_t  pad[3];
        uint32_t segment_id;         // added in format v07
        uint32_t parking_spot_id;    // added in format v0A
    };

    SerialRecord toRecord() const {
        SerialRecord r{};
        r.id                  = id;
        r.x                   = x;
        r.y                   = y;
        r.z                   = z;
        r.half_width_left_m   = half_width_left_m;
        r.half_width_right_m  = half_width_right_m;
        r.bound_angle_radians = bound_angle_radians;
        r.junction_id         = junction_id;
        r.flags               = flags;
        r.segment_id          = segment_id;
        r.parking_spot_id     = parking_spot_id;
        return r;
    }

    static RoadNode fromRecord(const SerialRecord& r) {
        RoadNode n;
        n.id                  = r.id;
        n.x                   = r.x;
        n.y                   = r.y;
        n.z                   = r.z;
        n.half_width_left_m   = r.half_width_left_m;
        n.half_width_right_m  = r.half_width_right_m;
        n.bound_angle_radians = r.bound_angle_radians;
        n.junction_id         = r.junction_id;
        n.flags               = r.flags;
        n.segment_id          = r.segment_id;
        n.parking_spot_id     = r.parking_spot_id;
        n.recomputeBounds();
        return n;
    }
};
