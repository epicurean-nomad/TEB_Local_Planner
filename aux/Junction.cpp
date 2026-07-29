#include "Junction.h"

bool Junction::containsPoint2D(const Point3d& p) const {
    bool   inside = false;
    size_t n      = cached_polygon.size();
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        float xi = cached_polygon[i].X, yi = cached_polygon[i].Y;
        float xj = cached_polygon[j].X, yj = cached_polygon[j].Y;
        if (((yi > p.y) != (yj > p.y)) &&
            (p.x < (xj - xi) * (p.y - yi) / (yj - yi) + xi))
            inside = !inside;
    }
    return inside;
}

const ConnectionEdge* Junction::findConnection(uint32_t from_node_id,
                                               uint32_t to_node_id) const {
    for (const auto& ce : connection_edges) {
        if (ce.from_node_id == from_node_id && ce.to_node_id == to_node_id)
            return &ce;
    }
    return nullptr;
}
