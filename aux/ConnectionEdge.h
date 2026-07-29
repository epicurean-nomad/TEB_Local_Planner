#pragma once
#include "AppConstants.h"
#include <cstdint>

struct ConnectionEdge {
    uint32_t id                    = INVALID_ID;
    uint32_t from_node_id          = INVALID_ID;  // node ending at a boundary node
    uint32_t to_node_id            = INVALID_ID;  // node starting at a boundary node
    int8_t   required_entry_lane   = 0;   // -1 = any lane
    int8_t   resulting_exit_lane   = 0;   // -1 = any lane
    float arc_length_m = 0.0f;
};
