#pragma once

#include "ConnectionEdge.h"
#include "Point3d.h"
#include "AppConstants.h"
#include <vector>
#include <cstdint>

enum class JunctionType : uint8_t {
    FOUR_WAY   = 0,
    T_JUNCTION = 1,
    Y_JUNCTION = 2,
    ROUNDABOUT = 3,
    MERGE      = 4,
    UNKNOWN    = 255,
};

struct Junction {
    uint32_t     id   = INVALID_ID;
    JunctionType type = JunctionType::UNKNOWN;

    std::vector<uint32_t>       boundary_node_ids;
    std::vector<Vec2>           boundary_pts;
    std::vector<ConnectionEdge> connection_edges;

    mutable std::vector<Vec2> cached_polygon;
    mutable bool              polygon_dirty = true;

    bool containsPoint2D(const Point3d& p) const;
    const ConnectionEdge* findConnection(uint32_t from_node_id, uint32_t to_node_id) const;

    struct SerialHeader {
        uint32_t id;
        uint8_t  type;
        uint8_t  pad[3];
        uint32_t boundary_node_count;
        uint32_t connection_edge_count;
    };
};
