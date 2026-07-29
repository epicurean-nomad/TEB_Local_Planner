#include "RoutingLayer.h"
#include "Serializer.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>

// ── Timing helpers ────────────────────────────────────────────────────────────

long long RoutingLayer::nowNs() const {
    if (!logging_enabled_) return 0LL;
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::high_resolution_clock::now().time_since_epoch())
        .count();
}

double RoutingLayer::elapsedMs(long long start_ns) const {
    if (!logging_enabled_ || start_ns == 0LL) return 0.0;
    const long long end_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                 std::chrono::high_resolution_clock::now().time_since_epoch())
                                 .count();
    return static_cast<double>(end_ns - start_ns) / 1'000'000.0;
}

void RoutingLayer::log(const std::string& msg) const {
    if (logging_enabled_)
        std::cout << "[RoutingLayer] " << msg << "\n";
}

// ── Constructor ───────────────────────────────────────────────────────────────

RoutingLayer::RoutingLayer(bool enable_logging)
    : logging_enabled_(enable_logging)
{}

// ── Lifecycle ─────────────────────────────────────────────────────────────────

bool RoutingLayer::loadArea(const std::string& filepath, std::string& error) {
    const long long t_total = nowNs();

    log("Loading area file: " + filepath);

    graph_.clear();
    is_loaded_ = false;

    const long long t_io = nowNs();
    if (!Serializer::loadArea(graph_, filepath, error)) {
        log("Load failed: " + error);
        return false;
    }
    log("  File read + parsed in " + std::to_string(elapsedMs(t_io)) + " ms");

    if (logging_enabled_) {
        log("  Nodes:     " + std::to_string(graph_.nodeCount()));
        log("  Segments:  " + std::to_string(graph_.segmentCount()));
        log("  Paths:     " + std::to_string(graph_.pathCount()));
        log("  Junctions: " + std::to_string(graph_.junctionCount()));
    }

    const long long t_cache = nowNs();
    buildCaches();
    log("  Caches built in " + std::to_string(elapsedMs(t_cache)) + " ms");

    is_loaded_ = true;
    log("  Total load time: " + std::to_string(elapsedMs(t_total)) + " ms");
    return true;
}

bool RoutingLayer::saveRuntime(const std::string& filepath, std::string& error) const {
    if (!is_loaded_) { error = "No area loaded"; return false; }
    const long long t = nowNs();
    const bool ok = Serializer::saveAvRuntime(graph_, filepath, error);
    if (ok) log("Runtime saved in " + std::to_string(elapsedMs(t)) + " ms");
    return ok;
}

bool RoutingLayer::loadRuntime(const std::string& filepath, std::string& error) {
    if (!is_loaded_) { error = "Load area before loading runtime cache"; return false; }
    const long long t = nowNs();
    const bool ok = Serializer::loadAvRuntime(graph_, filepath, error);
    if (ok) log("Runtime loaded in " + std::to_string(elapsedMs(t)) + " ms");
    return ok;
}

void RoutingLayer::clear() {
    const long long t = nowNs();
    graph_.clear();
    graph_topo_.clear();
    graph_weighted_.clear();
    junction_adj_.clear();
    is_loaded_ = false;
    log("Cleared (took " + std::to_string(elapsedMs(t)) + " ms)");
}

// ── Cache builder ─────────────────────────────────────────────────────────────

void RoutingLayer::buildCaches() {
    const size_t n = graph_.nodeCount();

    // ── Node topology + weighted (two passes, both maps together) ─────────────
    graph_topo_.clear();
    graph_topo_.reserve(n);
    graph_weighted_.clear();
    graph_weighted_.reserve(n);

    for (const auto& node : graph_.nodes()) {
        graph_topo_.emplace(node.id, std::unordered_set<uint32_t>{});
        graph_weighted_.emplace(node.id, std::unordered_map<uint32_t, float>{});
    }

    for (const auto& node : graph_.nodes()) {
        const auto* edges = graph_.outgoingEdges(node.id);
        if (!edges) continue;
        auto& neighbors = graph_topo_[node.id];
        auto& cost_map  = graph_weighted_[node.id];
        neighbors.reserve(edges->size());
        cost_map.reserve(edges->size());

        for (const auto& [to_id, ref] : *edges) {
            neighbors.insert(to_id);

            float cost = 0.0f;
            if (ref.kind == EdgeKind::PATH) {
                const Path* p = graph_.getPath(ref.edge_id);
                if (p) {
                    const RoadSegment* seg = graph_.getSegment(p->segment_id);
                    if (seg) cost = seg->arc_length_m;
                }
            } else {
                const Junction* j = graph_.getJunction(ref.edge_id);
                if (j) {
                    const ConnectionEdge* ce = j->findConnection(node.id, to_id);
                    if (ce) cost = ce->arc_length_m;
                }
            }
            cost_map[to_id] = cost;
        }
    }

    // ── Junction adjacency ────────────────────────────────────────────────────
    // Seed every junction as a key so isolated junctions appear in the map.
    junction_adj_.clear();
    junction_adj_.reserve(graph_.junctionCount());
    for (const auto& j : graph_.junctions())
        junction_adj_.emplace(j.id, std::unordered_set<uint32_t>{});

    // A directed edge A → B exists when a segment goes from a boundary node of
    // junction A to a boundary node of junction B via a forward or reverse path.
    for (const auto& seg : graph_.segments()) {
        const RoadNode* na = graph_.getNode(seg.node_a_id);
        const RoadNode* nb = graph_.getNode(seg.node_b_id);
        if (!na || !nb) continue;

        const uint32_t ja = na->junction_id;
        const uint32_t jb = nb->junction_id;
        if (ja == INVALID_ID || jb == INVALID_ID || ja == jb) continue;

        if (seg.forward_path_id != INVALID_ID) junction_adj_[ja].insert(jb);
        if (seg.reverse_path_id != INVALID_ID) junction_adj_[jb].insert(ja);
    }
}

// ── Spatial queries ───────────────────────────────────────────────────────────

uint32_t RoutingLayer::nearestPoly(float x, float y,
                                    PolyType* out_type,
                                    float*    out_dist_sq) const {
    if (!is_loaded_) return INVALID_ID;
    return graph_.polyRTree().nearest(Vec2{x, y}, out_type, out_dist_sq);
}

// ── Control points ────────────────────────────────────────────────────────────

std::vector<Point3d> RoutingLayer::getControlPoints(uint32_t from_node_id,
                                                     uint32_t to_node_id,
                                                     float    spacing_m) const {
    if (!is_loaded_ || spacing_m <= 0.f) return {};

    // Junction edges have no centerline geometry to walk.
    const EdgeRef* ref = graph_.edgeBetween(from_node_id, to_node_id);
    if (!ref || ref->kind != EdgeKind::PATH) return {};

    const Path* path = graph_.getPath(ref->edge_id);
    if (!path) return {};

    const RoadSegment* seg = graph_.getSegment(path->segment_id);
    if (!seg || seg->arc_length_m <= 0.f) return {};

    const RoadNode* node_a = graph_.getNode(seg->node_a_id);
    const RoadNode* node_b = graph_.getNode(seg->node_b_id);
    if (!node_a || !node_b) return {};

    // Augmented polyline: position + bound angle and half-widths at each vertex.
    // Covers both node endpoints (bound_angle_radians) and intermediate geometry
    // points (angle_radians) — same lBound/rBound formula for both.
    struct BoundedPt {
        float x, y;
        float angle;       // bound direction angle stored at this vertex
        float lbound_off;  // distance from center to lBound
        float rbound_off;  // distance from center to rBound
    };

    std::vector<BoundedPt> bpts;
    bpts.reserve(seg->geometry_pts.size() + 2);
    bpts.push_back({node_a->x, node_a->y,
                    node_a->bound_angle_radians,
                    node_a->half_width_left_m,
                    node_a->half_width_right_m});
    for (const auto& gp : seg->geometry_pts)
        bpts.push_back({gp.x, gp.y, gp.angle_radians,
                        gp.lbound_offset, gp.rbound_offset});
    bpts.push_back({node_b->x, node_b->y,
                    node_b->bound_angle_radians,
                    node_b->half_width_left_m,
                    node_b->half_width_right_m});

    // Orient A→B to from→to.
    if (from_node_id != seg->node_a_id)
        std::reverse(bpts.begin(), bpts.end());

    // For 2-lane roads compute the lane-center position at each vertex.
    // Left-hand traffic: travel lane = left of the local travel direction.
    // At each vertex, the local tangent is the direction toward the next vertex
    // (or from the previous vertex at the tail). The dot product of the tangent
    // with (cos angle, sin angle) tells whether lBound is on the travel side:
    //   dot >= 0 → lBound is left of tangent → fwd lane uses lbound_off
    //   dot <  0 → lBound is right of tangent → fwd lane uses rbound_off
    // Lane center = left_perp(tangent) * half_bound_off.
    const bool two_lane = (seg->lane_count >= 2);

    std::vector<Point3d> lane_pts;
    lane_pts.reserve(bpts.size());

    for (size_t i = 0; i < bpts.size(); ++i) {
        const auto& p = bpts[i];

        if (!two_lane) {
            lane_pts.emplace_back(p.x, p.y, 1.f);
            continue;
        }

        // Tangent toward next vertex (or back-tangent at the last vertex).
        float tx, ty;
        if (i + 1 < bpts.size()) {
            tx = bpts[i + 1].x - p.x;
            ty = bpts[i + 1].y - p.y;
        } else {
            tx = p.x - bpts[i - 1].x;
            ty = p.y - bpts[i - 1].y;
        }
        const float len = std::sqrt(tx * tx + ty * ty);
        if (len < 1e-6f) { lane_pts.emplace_back(p.x, p.y, 1.f); continue; }
        tx /= len;
        ty /= len;

        const float dot = tx * std::cos(p.angle) + ty * std::sin(p.angle);
        const float hw  = (dot >= 0.f ? p.lbound_off : p.rbound_off) * 0.5f;

        // Left perpendicular of (tx, ty) = (-ty, tx).
        lane_pts.emplace_back(p.x - ty * hw, p.y + tx * hw, 1.f);
    }

    // Resample lane_pts at spacing_m intervals.
    std::vector<Point3d> result;
    result.reserve(static_cast<size_t>(seg->arc_length_m / spacing_m) + 2);
    result.push_back(lane_pts.front());

    float walked    = 0.f;
    float next_emit = spacing_m;

    for (size_t i = 1; i < lane_pts.size(); ++i) {
        const float span = (lane_pts[i] - lane_pts[i - 1]).length();
        if (span == 0.f) continue;

        while (walked + span >= next_emit) {
            const float t = (next_emit - walked) / span;
            result.push_back(lane_pts[i - 1].mix(lane_pts[i], t));
            next_emit += spacing_m;
        }
        walked += span;
    }

    if ((result.back() - lane_pts.back()).length() > 1e-4f)
        result.push_back(lane_pts.back());

    return result;
}


int8_t RoutingLayer::getLaneId(
    uint32_t prev_node,
    double x,
    double y,
    double theta
) const {
    if (!is_loaded_) return -1;

    const auto node = graph_.getNode(prev_node);
    if (!node) return -1;

    auto segment_id = node->segment_id;
    if (segment_id == INVALID_ID) return -1;

    const auto seg = graph_.getSegment(segment_id);
    if (!seg || seg->lane_count <= 1) return 0;

    const size_t N = 2 + seg->geometry_pts.size();

    auto vertexAt = [&](size_t idx) -> Vec2 {
        if (idx == 0)
            return seg->a;
        if (idx == N - 1)
            return seg->b ;
        const auto& gp = seg->geometry_pts[idx - 1];
        return {gp.x, gp.y};
    };

    const Vec2 veh = Vec2((float)x, (float)y);

    // Find the centerline edge the vehicle's projection falls between — the
    // point where the vector to each endpoint of the edge flips direction.
    // Carries the previous vertex forward so each one is computed only once.
    Vec2 a = vertexAt(0);
    Vec2 b = vertexAt(1);
    for (size_t i = 0; ; ++i) {
        const float d1x = a.X - veh.X, d1y = a.Y - veh.Y;
        const float d2x = b.X - veh.X, d2y = b.Y - veh.Y;
        if (d1x * d2x + d1y * d2y < 0.f) break;
        if (i + 2 >= N) break;
        a = b;
        b = vertexAt(i + 2);
    }

    Vec2 segVector = b - a;
    Vec2 vehPose = {std::cos(theta), std::sin(theta)};
    if (Vec2::DotProduct(segVector, vehPose) < 0) {
        std::swap(a, b);
        segVector = b - a;
    }

    Vec2 aToVeh = veh - a;

    uint8_t lane_id = 0;
    float crossProd = segVector.Cross(aToVeh);
    if (crossProd < 0) lane_id = 1;

    //std::cout << "Lane id: " << (int)lane_id << " (cross product: " << crossProd << ")\n";

    return lane_id;
}