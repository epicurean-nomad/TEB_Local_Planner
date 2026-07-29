#pragma once
#include "AppConstants.h"
#include "Point3d.h"
#include <cstdint>
#include <vector>

enum class ParkingSpotType : uint8_t {
    PERPENDICULAR = 0, PARALLEL = 1, ANGLED_45 = 2, ANGLED_60 = 3,
    DISABLED = 4, EV_CHARGING = 5, RESERVED = 6,
};

struct ParkingSpot {
    uint32_t id = INVALID_ID;

    // Physical footprint (area-local metres).
    // corners[0..1] = front edge (entry side; terminal node lives here)
    // corners[2..3] = back edge (far end of spot)
    std::vector<Vec2> corners = std::vector<Vec2>(4);
    float center[3]     = {};
    float heading_radians = 0.f;  // direction vehicle faces when correctly parked

    // [0] = front terminal, [1] = back terminal (INVALID_ID if absent)
    uint32_t terminal_node_ids[2] = {INVALID_ID, INVALID_ID};

    ParkingSpotType type = ParkingSpotType::PERPENDICULAR;

    float frontEdgeMidX() const { return (corners[0].X + corners[1].X) * 0.5f; }
    float frontEdgeMidY() const { return (corners[0].Y + corners[1].Y) * 0.5f; }
    float backEdgeMidX()  const { return (corners[2].X + corners[3].X) * 0.5f; }
    float backEdgeMidY()  const { return (corners[2].Y + corners[3].Y) * 0.5f; }

    struct SerialRecord {
        uint32_t id;
        float    corners[4][3];       // 48 bytes
        float    center[3];           // 12 bytes
        float    heading_radians;
        uint32_t terminal_node_ids[2];
        uint8_t  type;
        uint8_t  pad[3];
    };  // 80 bytes total, 4-byte aligned

    SerialRecord toRecord() const {
        SerialRecord rec{};
        rec.id = id;
        for (int i = 0; i < 4; ++i) {
            rec.corners[i][0] = corners[i].X;
            rec.corners[i][1] = corners[i].Y;
            rec.corners[i][2] = 0.f;
        }
        rec.center[0] = center[0];
        rec.center[1] = center[1];
        rec.center[2] = center[2];
        rec.heading_radians      = heading_radians;
        rec.terminal_node_ids[0] = terminal_node_ids[0];
        rec.terminal_node_ids[1] = terminal_node_ids[1];
        rec.type = static_cast<uint8_t>(type);
        return rec;
    }

    static ParkingSpot fromRecord(const SerialRecord& r) {
        ParkingSpot s;
        s.id = r.id;
        s.corners.resize(4);
        for (int i = 0; i < 4; ++i) {
            s.corners[i] = Vec2{r.corners[i][0], r.corners[i][1]};
        }
        s.center[0] = r.center[0];
        s.center[1] = r.center[1];
        s.center[2] = r.center[2];
        s.heading_radians      = r.heading_radians;
        s.terminal_node_ids[0] = r.terminal_node_ids[0];
        s.terminal_node_ids[1] = r.terminal_node_ids[1];
        s.type = static_cast<ParkingSpotType>(r.type);
        return s;
    }
};
