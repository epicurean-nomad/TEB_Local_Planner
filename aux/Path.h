#pragma once

#include "AppConstants.h"
#include <vector>
#include <cstdint>

struct Path {
    uint32_t id           = INVALID_ID;
    uint32_t segment_id   = INVALID_ID;
    uint32_t from_node_id = INVALID_ID;
    uint32_t to_node_id   = INVALID_ID;

    uint8_t speed_limit_kmh     = 40;
    bool    lane_change_allowed = true;
    uint8_t lane_count          = 2;

    struct SerialHeader {
        uint32_t id;
        uint32_t segment_id;
        uint32_t from_node_id;
        uint32_t to_node_id;
        uint8_t  lane_count;
        uint8_t  speed_limit_kmh;
        uint8_t  lane_change_allowed;
        uint8_t  pad;
        uint32_t dir_reg_count;
    };
};
