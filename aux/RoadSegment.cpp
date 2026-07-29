#include "RoadSegment.h"
#include <algorithm>

// By default z will be +1 for all the segments.
// Segments paths are already oriented, going from a_pos - b_pos we always travel in forward direction. 
// Similarly reverse edge is also stored, going from b_pos - a_pos also gets +1.
std::vector<Point3d> RoadSegment::polyline(const Point3d& a_pos, const Point3d& b_pos) const {
    std::vector<Point3d> pts;
    pts.reserve(geometry_pts.size() + 2);
    // pts.push_back(a_pos);
    // for (const auto& gp : geometry_pts)
    //     pts.emplace_back(gp.x, gp.y, gp.z);
    // pts.push_back(b_pos);

    pts.emplace_back(a_pos.x, a_pos.y, 1.0f);
    for (const auto& gp : geometry_pts)
        pts.emplace_back(gp.x, gp.y, 1.0f);
    pts.emplace_back(b_pos.x, b_pos.y, 1.0f);
    
    return pts;
}

Point3d RoadSegment::sampleAt(float s_m, const Point3d& a_pos, const Point3d& b_pos) const {
    s_m = std::clamp(s_m, 0.0f, arc_length_m);
    auto  pts = polyline(a_pos, b_pos);
    float acc = 0.0f;
    for (size_t i = 1; i < pts.size(); ++i) {
        float seg_len = (pts[i] - pts[i - 1]).length();
        if (acc + seg_len >= s_m) {
            float t = (seg_len > 0.0f) ? (s_m - acc) / seg_len : 0.0f;
            return pts[i - 1].mix(pts[i], t);
        }
        acc += seg_len;
    }
    return pts.back();
}