#include "AreaGraph.h"
#include <algorithm>
#include <cmath>
#include <cstring>

AreaGraph::AreaGraph() = default;

// ── Node operations ───────────────────────────────────────────────────────────

const RoadNode* AreaGraph::getNode(uint32_t id) const {
    auto it = node_index_.find(id);
    return (it != node_index_.end()) ? &nodes_[it->second] : nullptr;
}


// ── Segment operations ────────────────────────────────────────────────────────

const RoadSegment* AreaGraph::getSegment(uint32_t id) const {
    auto it = segment_index_.find(id);
    return (it != segment_index_.end()) ? &segments_[it->second] : nullptr;
}

// ── Path operations ───────────────────────────────────────────────────────────

const Path* AreaGraph::getPath(uint32_t id) const {
    auto it = paths_.find(id);
    return (it != paths_.end()) ? &it->second : nullptr;
}

// ── Junction operations ───────────────────────────────────────────────────────

const Junction* AreaGraph::getJunction(uint32_t id) const {
    auto it = junction_index_.find(id);
    return (it != junction_index_.end()) ? &junctions_[it->second] : nullptr;
}

// ── Parking spot operations ───────────────────────────────────────────────────

const ParkingSpot* AreaGraph::getParkingSpot(uint32_t id) const {
    auto it = parking_spot_index_.find(id);
    return (it != parking_spot_index_.end()) ? &parking_spots_[it->second] : nullptr;
}

// ── Adjacency queries ─────────────────────────────────────────────────────────

const EdgeRef* AreaGraph::edgeBetween(uint32_t from_node_id, uint32_t to_node_id) const {
    auto it = adjacency_.find(from_node_id);
    if (it == adjacency_.end()) return nullptr;
    auto jt = it->second.find(to_node_id);
    return (jt != it->second.end()) ? &jt->second : nullptr;
}

const std::unordered_map<uint32_t, EdgeRef>*
AreaGraph::outgoingEdges(uint32_t from_node_id) const {
    auto it = adjacency_.find(from_node_id);
    return (it != adjacency_.end()) ? &it->second : nullptr;
}

// ── Clear ─────────────────────────────────────────────────────────────────────

void AreaGraph::clear() {
    nodes_.clear();
    segments_.clear();
    junctions_.clear();
    paths_.clear();
    parking_spots_.clear();
    node_index_.clear();
    segment_index_.clear();
    junction_index_.clear();
    parking_spot_index_.clear();
    adjacency_.clear();
    sampled_paths_.clear();
    apsp_table_.clear();
    rtree_.clear();
    std::memset(area_name, 0, sizeof(area_name));
    std::memset(area_uuid, 0, sizeof(area_uuid));
    origin = GeoOrigin{};
}

// ── Junction polygons ─────────────────────────────────────────────────────────

void AreaGraph::recomputeDirtyJunctionPolygons() {
    for (auto& j : junctions_) {
        if (!j.polygon_dirty) continue;

        // Collect 2D boundary points from node lane bounds + explicit boundary_pts
        struct Pt2 { float x, y; };
        std::vector<Pt2> pts;
        pts.reserve(j.boundary_node_ids.size() * 2 + j.boundary_pts.size());
        for (uint32_t nid : j.boundary_node_ids) {
            const RoadNode* n = getNode(nid);
            if (!n) continue;
            pts.push_back({n->lBound.x, n->lBound.y});
            pts.push_back({n->rBound.x, n->rBound.y});
        }
        for (const auto& bp : j.boundary_pts)
            pts.push_back({bp.X, bp.Y});

        if (pts.size() < 3) continue;

        // Centroid of all boundary points
        float cx = 0.f, cy = 0.f;
        for (const auto& p : pts) { cx += p.x; cy += p.y; }
        cx /= static_cast<float>(pts.size());
        cy /= static_cast<float>(pts.size());

        // Sort by angle around centroid → convex-order simple polygon
        std::sort(pts.begin(), pts.end(), [cx, cy](const Pt2& a, const Pt2& b) {
            return std::atan2(a.y - cy, a.x - cx) < std::atan2(b.y - cy, b.x - cx);
        });

        j.cached_polygon.clear();
        j.cached_polygon.reserve(pts.size());
        for (const auto& p : pts)
            j.cached_polygon.emplace_back(p.x, p.y);

        j.polygon_dirty = false;
    }
}

// ── Derived-state rebuild ─────────────────────────────────────────────────────

void AreaGraph::rebuildDerivedState() {
    // Index maps
    node_index_.clear();
    for (uint32_t i = 0; i < static_cast<uint32_t>(nodes_.size()); ++i)
        node_index_[nodes_[i].id] = i;

    segment_index_.clear();
    for (uint32_t i = 0; i < static_cast<uint32_t>(segments_.size()); ++i)
        segment_index_[segments_[i].id] = i;

    junction_index_.clear();
    for (uint32_t i = 0; i < static_cast<uint32_t>(junctions_.size()); ++i)
        junction_index_[junctions_[i].id] = i;

    parking_spot_index_.clear();
    for (uint32_t i = 0; i < static_cast<uint32_t>(parking_spots_.size()); ++i)
        parking_spot_index_[parking_spots_[i].id] = i;

    // Segment forward/reverse path ids
    for (auto& seg : segments_) {
        seg.forward_path_id = INVALID_ID;
        seg.reverse_path_id = INVALID_ID;
    }
    for (const auto& [pid, p] : paths_) {
        auto it = segment_index_.find(p.segment_id);
        if (it == segment_index_.end()) continue;
        auto& seg = segments_[it->second];
        if      (p.from_node_id == seg.node_a_id) seg.forward_path_id = p.id;
        else if (p.from_node_id == seg.node_b_id) seg.reverse_path_id = p.id;
        seg.lane_count = p.lane_count;
    }

    // Populate endpoint positions on every segment (needed for AABB + distance).
    // Also cache the segment polygon, needed for quick point in poly tests
    for (auto& seg : segments_) {
        const RoadNode* na = getNode(seg.node_a_id);
        const RoadNode* nb = getNode(seg.node_b_id);
        if (!na || !nb) continue;

        seg.a = Vec2{na->x, na->y};
        seg.b = Vec2{nb->x, nb->y};

        // Centerline vertices in A->B order, each carrying its precomputed bound points.
        struct BoundVtx { float x, y; const Point3d* lb; const Point3d* rb; };
        std::vector<BoundVtx> verts;
        verts.reserve(seg.geometry_pts.size() + 2);
        verts.push_back({na->x, na->y, &na->lBound, &na->rBound});
        for (const auto& gp : seg.geometry_pts)
            verts.push_back({gp.x, gp.y, &gp.lBound, &gp.rBound});
        verts.push_back({nb->x, nb->y, &nb->lBound, &nb->rBound});

        // angle_radians (and hence lBound/rBound) is stored independent of travel
        // direction, so it can flip which physical side "lBound" lands on along the
        // segment. Resolve left/right per-vertex against the local A->B tangent so
        // the polygon walk (left side forward, right side back) stays a simple loop.
        auto tangentAt = [&](size_t i) -> Vec2 {
            float tx, ty;
            if (i + 1 < verts.size()) { tx = verts[i + 1].x - verts[i].x; ty = verts[i + 1].y - verts[i].y; }
            else                      { tx = verts[i].x - verts[i - 1].x; ty = verts[i].y - verts[i - 1].y; }
            const float len = std::sqrt(tx * tx + ty * ty);
            return (len < 1e-6f) ? Vec2{1.f, 0.f} : Vec2{tx / len, ty / len};
        };
        auto isLBoundOnLeft = [](const BoundVtx& v, const Vec2& t) {
            // Left side of tangent (tx,ty) is the half-plane toward (-ty, tx).
            return (v.lb->x - v.x) * -t.Y + (v.lb->y - v.y) * t.X >= 0.f;
        };

        auto& poly_pts = seg.cached_polygon;
        poly_pts.clear();
        poly_pts.reserve(verts.size() * 2);

        for (size_t i = 0; i < verts.size(); ++i) {
            const auto& v = verts[i];
            const Point3d* lp = isLBoundOnLeft(v, tangentAt(i)) ? v.lb : v.rb;
            poly_pts.push_back({lp->x, lp->y});
        }
        for (size_t i = verts.size(); i-- > 0; ) {
            const auto& v = verts[i];
            const Point3d* rp = isLBoundOnLeft(v, tangentAt(i)) ? v.rb : v.lb;
            poly_pts.push_back({rp->x, rp->y});
        }
    }

    // Junction cached_polygon must be ready before we can extract their vertices.
    recomputeDirtyJunctionPolygons();

    // Build unified SpatialPoly list: segments, junctions and parking spots, all closed polygons.
    std::vector<SpatialPoly> polys;
    polys.reserve(segments_.size() + junctions_.size() + parking_spots_.size());

    for (const auto& seg : segments_) {
        SpatialPoly p;
        p.id    = seg.id;
        p.type  = PolyType::SEGMENT;
        p.verts = seg.cached_polygon;
        polys.push_back(std::move(p));
    }

    for (const auto& junc : junctions_) {
        SpatialPoly p;
        p.id   = junc.id;
        p.type = PolyType::JUNCTION;
        p.verts.reserve(junc.cached_polygon.size());
        for (const auto& pt : junc.cached_polygon)
            p.verts.push_back(pt);
        polys.push_back(std::move(p));
    }

    for (const auto& spot : parking_spots_) {
        SpatialPoly p;
        p.id   = spot.id;
        p.type = PolyType::PARKING_SPOT;
        p.verts.reserve(4);
        for (int i = 0; i < 4; ++i)
            p.verts.push_back(spot.corners[i]);
        polys.push_back(std::move(p));
    }

    rtree_.build(std::move(polys));
}
