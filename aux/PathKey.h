#pragma once

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "Point3d.h"

// I inside, O Outside (Always with reference to a segment)
// so a to b OI represents outside from a and inside to b (which is default maneuver)
enum class Dir : uint8_t {
    OI,
    IO,
    OO,
    II,
};

struct PathKey {
    uint32_t a;
    uint32_t b;
    Dir      dir;

    // Inter-segment
    PathKey (uint32_t from_node, uint32_t to_node) : a(from_node), b(to_node), dir(Dir::IO) {}

    // Between junction nodes
    PathKey (uint32_t from_node, uint32_t to_node, Dir dir) : a(from_node), b(to_node), dir(dir) {}

    bool operator==(const PathKey& other) const {
        return a == other.a && b == other.b && dir == other.dir;
    }
};

struct PathKeyHash {
    size_t operator()(const PathKey& k) const {
        uint64_t h = (static_cast<uint64_t>(k.a) << 32) ^ k.b;
        h ^= static_cast<uint64_t>(k.dir) * 0x9e3779b97f4a7c15ULL;
        return h;
    }
};

using SampledPathMap = std::unordered_map<PathKey, std::vector<Point3d>, PathKeyHash>;
