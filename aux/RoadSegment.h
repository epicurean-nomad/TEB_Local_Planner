#pragma once

#include "GeometryPoint.h"
#include "Point3d.h"
#include "AppConstants.h"
#include <vector>
#include <cstdint>

struct AABB;

enum class RoadClass : uint8_t {
    LOCAL     = 0,
    COLLECTOR = 1,
    ARTERIAL  = 2,
    HIGHWAY   = 3,
};

struct RoadSegment {
    uint32_t id        = INVALID_ID;
    uint32_t node_a_id = INVALID_ID;
    uint32_t node_b_id = INVALID_ID;

    // 2D endpoint positions in the graph's local coordinate frame.
    // Set when the segment is loaded/added to the graph (from RoadNode::x,y).
    // Required by PolyRTree for AABB computation and nearest-neighbour queries.
    Vec2 a{}, b{};
    mutable std::vector<Vec2>      cached_polygon;

    std::vector<GeometryPoint>     geometry_pts;

    float     arc_length_m  = 0.0f;
    uint8_t   lane_count    = 2;
    RoadClass road_class    = RoadClass::LOCAL;
    uint8_t   surface_type  = 0;

    uint32_t forward_path_id = INVALID_ID;
    uint32_t reverse_path_id = INVALID_ID;

    bool isBidirectional() const {
        return forward_path_id != INVALID_ID && reverse_path_id != INVALID_ID;
    }
    bool isOneWay() const {
        return (forward_path_id != INVALID_ID) ^ (reverse_path_id != INVALID_ID);
    }

    Point3d              sampleAt(float s_m, const Point3d& a_pos, const Point3d& b_pos) const;
    std::vector<Point3d> polyline(const Point3d& a_pos, const Point3d& b_pos) const;
};
