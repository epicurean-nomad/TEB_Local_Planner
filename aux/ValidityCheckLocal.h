#ifndef VALIDITY_CHECK_LOCAL_H
#define VALIDITY_CHECK_LOCAL_H

// Experimental variant of ValidityCheck that scopes collision checks to a
// small "local zone" instead of scanning every polygon in the area.
//
// Rationale: junction_rsrrt_cache plans paths one junction-connection combo at
// a time — start/goal always sit on a single junction and (at most) one or two
// attached segments / a parking spot. There is no need to test candidate poses
// against polygons on the far side of the map. setLocalZone(...) resolves,
// from the combo's (junction id, from-node id, to-node id), exactly which
// polygons are relevant — using graph topology (junction boundary nodes →
// attached segments, parking-terminal flag → parking spot), not geometry/
// spatial queries — and isInsideZone then runs its cheap linear scan over that
// handful of polygons instead of every segment/junction/parking-spot.
//
// Resolution rules (per combo):
//   - normal node (not a parking terminal): take the junction polygon plus
//     every road segment attached to any of the junction's boundary nodes.
//   - parking-terminal node: take the junction polygon, every road segment
//     attached to the junction's *non*-parking boundary nodes (parking
//     terminals aren't segment endpoints, so they contribute no segments),
//     plus the single ParkingSpot polygon referenced by that terminal node.
//
// Kept as a separate header (rather than editing ValidityCheck.h) so the
// existing pipeline / junction_rsrrt_cache.cpp keeps using the proven
// global-scan implementation while this approach is evaluated.

#include "RoutingLayer.h"
#include "geometry_2D_utils.hpp"
#include <cmath>
#include <algorithm>
#include <unordered_set>
#include <fstream>
#include <iomanip>

struct VehicleFootprintLocal {
    float length = 2.2f;
    float width  = 1.3f;
};

class ValidityCheckLocal {
    public:
        ValidityCheckLocal() : graph_(nullptr) {}

        void init(const AreaGraph* g) {
            graph_ = g;

        }

        void setVehicle(float length, float width, float wheelbase = 0) {
            veh_.length = length;
            veh_.width  = width;
        }

        float vehicleLength() const { return veh_.length; }
        float vehicleWidth()  const { return veh_.width;  }

        // Resolve and cache the small set of polygons relevant to planning
        // between `from_node_id` and `to_node_id` across junction `jxn_id`.
        // Must be called before isInsideZone/isVehicleFeasible/isPathFeasible
        // for a given combo.
        void setLocalZone(uint32_t jxn_id, uint32_t from_node_id, uint32_t to_node_id) {
            local_polygons_.clear();
            if (!graph_) return;

            const Junction* jxn = graph_->getJunction(jxn_id);
            if (jxn && jxn->cached_polygon.size() >= 3)
                local_polygons_.push_back(&jxn->cached_polygon);

            const ParkingSpot* parking_a = parkingSpotForNode(from_node_id);
            const ParkingSpot* parking_b = parkingSpotForNode(to_node_id);

            std::unordered_set<uint32_t> seg_ids;
            if (jxn) {
                for (uint32_t nid : jxn->boundary_node_ids) {
                    const RoadNode* n = graph_->getNode(nid);
                    if (!n) continue;
                    // Parking-terminal nodes are not road-segment endpoints —
                    // skip them when collecting attached segments; their
                    // corresponding ParkingSpot polygon is added separately.
                    if (n->isParkingTerminal()) continue;
                    collectSegmentsAtNode(nid, seg_ids);
                }
            }

            for (uint32_t sid : seg_ids) {
                const RoadSegment* seg = graph_->getSegment(sid);
                if (!seg) continue;
                if (seg->cached_polygon.size() >= 3)
                    local_polygons_.push_back(&seg->cached_polygon);
            }

            if (parking_a && parking_a->corners.size() >= 3)
                local_polygons_.push_back(&parking_a->corners);
            if (parking_b && parking_b != parking_a && parking_b->corners.size() >= 3)
                local_polygons_.push_back(&parking_b->corners);

            printf("  [validity-local] jxn%u %u<->%u: %zu polygons\n",
                   jxn_id, from_node_id, to_node_id, local_polygons_.size());
        }

        // Build the local zone for an ENTIRE planned route. final_planner_with_parking
        // stitches a path that spans many junctions/segments, so the single-combo
        // setLocalZone() above is not enough on its own. This collects, as a generous
        // superset:
        //   - every segment and junction a route node belongs to,
        //   - the junctions at the endpoints of those route segments (so a maneuver
        //     that swings off a segment into an adjacent junction is still covered),
        //   - every segment attached to any of those junctions' boundary nodes,
        //   - the parking spot of any parking-terminal node encountered.
        //
        // The result is a strict superset of the polygons the stitched path can touch
        // and a strict subset of the whole map. Because it is a subset of all map
        // polygons, isInsideZone can never accept a point the global ValidityCheck
        // rejects; because it is a superset of the route corridor, it will not newly
        // reject a feasible path. Net effect: same feasibility verdict as the global
        // scan along the route, but far fewer polygons tested per point.
        void setRouteZone(const std::vector<uint32_t>& route_nodes) {
            local_polygons_.clear();
            if (!graph_) return;

            std::unordered_set<uint32_t> jxn_ids, seg_ids, parking_ids;

            auto addNode = [&](uint32_t nid) {
                const RoadNode* n = graph_->getNode(nid);
                if (!n) return;
                if (n->segment_id  != INVALID_ID) seg_ids.insert(n->segment_id);
                if (n->junction_id != INVALID_ID) jxn_ids.insert(n->junction_id);
                if (n->isParkingTerminal() && n->parking_spot_id != INVALID_ID)
                    parking_ids.insert(n->parking_spot_id);
            };

            for (uint32_t nid : route_nodes) addNode(nid);

            // Pull in the junctions sitting at the ends of every route segment.
            for (uint32_t sid : seg_ids) {
                const RoadSegment* seg = graph_->getSegment(sid);
                if (!seg) continue;
                const RoadNode* na = graph_->getNode(seg->node_a_id);
                const RoadNode* nb = graph_->getNode(seg->node_b_id);
                if (na && na->junction_id != INVALID_ID) jxn_ids.insert(na->junction_id);
                if (nb && nb->junction_id != INVALID_ID) jxn_ids.insert(nb->junction_id);
            }

            // For every junction in scope, add all segments attached to its boundary
            // nodes (and any parking spots those nodes terminate). Mirrors the
            // parking-terminal handling in setLocalZone().
            for (uint32_t jid : jxn_ids) {
                const Junction* jxn = graph_->getJunction(jid);
                if (!jxn) continue;
                for (uint32_t bnid : jxn->boundary_node_ids) {
                    const RoadNode* bn = graph_->getNode(bnid);
                    if (!bn) continue;
                    if (bn->isParkingTerminal()) {
                        if (bn->parking_spot_id != INVALID_ID)
                            parking_ids.insert(bn->parking_spot_id);
                        continue;
                    }
                    collectSegmentsAtNode(bnid, seg_ids);
                }
            }

            // Materialize the zone as pointers into the graph's polygons (no copies).
            for (uint32_t jid : jxn_ids) {
                const Junction* jxn = graph_->getJunction(jid);
                if (jxn && jxn->cached_polygon.size() >= 3)
                    local_polygons_.push_back(&jxn->cached_polygon);
            }
            for (uint32_t sid : seg_ids) {
                const RoadSegment* seg = graph_->getSegment(sid);
                if (seg && seg->cached_polygon.size() >= 3)
                    local_polygons_.push_back(&seg->cached_polygon);
            }
            for (uint32_t pid : parking_ids) {
                const ParkingSpot* spot = graph_->getParkingSpot(pid);
                if (spot && spot->corners.size() >= 3)
                    local_polygons_.push_back(&spot->corners);
            }

            printf("  [validity-local] route zone: %zu nodes -> %zu polygons "
                   "(%zu jxn, %zu seg, %zu park)\n",
                   route_nodes.size(), local_polygons_.size(),
                   jxn_ids.size(), seg_ids.size(), parking_ids.size());
        }

        bool isInsideZone(float px, float py) const {
            for (const auto* poly : local_polygons_) {
                if (pointInPolygon(px, py, *poly)) return true;   // ← ray cast
            }
            return false;
        }

        bool isVehicleFeasible(float px, float py, float theta) const {
            float L = veh_.length * 0.5f;
            float W = veh_.width  * 0.5f;
            float corners[][2] = {
                { L,  W},
                { L, -W},
                {-L, -W},
                {-L,  W},
            };
            float c = std::cos(theta), s = std::sin(theta);
            for (auto& cl : corners) {
                float wx = px + c * cl[0] - s * cl[1];
                float wy = py + s * cl[0] + c * cl[1];
                if (!isInsideZone(wx, wy)) return false;
            }
            return true;
        }

        bool isPathFeasible(const std::vector<utils_2d_geometry::PathPoint>& points) const {
            for (const auto& pt : points) {
                if (!isVehicleFeasible(pt.point.x, pt.point.y, pt.point.theta))
                    return false;
            }
            return true;
        }

        void dumpZone(const std::string& filename) const {
            std::ofstream f(filename);
            for (size_t i = 0; i < local_polygons_.size(); ++i) {
                const auto& poly = *local_polygons_[i];
                f << "# local " << i << "\n";
                for (const auto& pt : poly)
                    f << std::fixed << std::setprecision(4) << pt.X << "," << pt.Y << ",0\n";
                if (!poly.empty())
                    f << poly[0].X << "," << poly[0].Y << ",0\n";
                f << "\n";
            }
        }

        void dumpFootprint(float px, float py, float theta, const std::string& filename) const {
            std::ofstream f(filename);
            f << "# pose " << px << "," << py << "," << theta << "\n";
            const float L = veh_.length * 0.5f;
            const float W = veh_.width  * 0.5f;
            const float corners[5][2] = {
                { L,  W}, { L, -W}, {-L, -W}, {-L,  W}, { L,  W},
            };
            const float c = std::cos(theta), s = std::sin(theta);
            for (int i = 0; i < 5; ++i) {
                float wx = px + c * corners[i][0] - s * corners[i][1];
                float wy = py + s * corners[i][0] + c * corners[i][1];
                bool valid = isInsideZone(wx, wy);
                f << std::fixed << std::setprecision(4)
                  << wx << "," << wy << "," << (valid ? 1 : 0) << "\n";
            }
        }

private:
    const ParkingSpot* parkingSpotForNode(uint32_t node_id) const {
        const RoadNode* n = graph_->getNode(node_id);
        if (!n || !n->isParkingTerminal()) return nullptr;
        return graph_->getParkingSpot(n->parking_spot_id);
    }

    void collectSegmentsAtNode(uint32_t node_id, std::unordered_set<uint32_t>& out) const {
        for (const auto& seg : graph_->segments()) {
            if (seg.node_a_id == node_id || seg.node_b_id == node_id)
                out.insert(seg.id);
        }
    }

    // Same simple 4-vertex quad construction ValidityCheck uses — cheap to
    // test and adequate for collision purposes (see ValidityCheck::buildAll).
    // std::vector<Vec2> segmentQuad(const RoadSegment& seg) const {
    //     const RoadNode* na = graph_->getNode(seg.node_a_id);
    //     const RoadNode* nb = graph_->getNode(seg.node_b_id);
    //     if (!na || !nb) return {};
    //     return {
    //         Vec2{(float)na->lBound.x, (float)na->lBound.y},
    //         Vec2{(float)nb->lBound.x, (float)nb->lBound.y},
    //         Vec2{(float)nb->rBound.x, (float)nb->rBound.y},
    //         Vec2{(float)na->rBound.x, (float)na->rBound.y},
    //     };
    // }

    // General polygon — ray casting
    static bool pointInPolygon(float px, float py, const std::vector<Vec2>& poly) {
        bool inside = false;
        size_t n = poly.size();
        for (size_t i = 0, j = n - 1; i < n; j = i++) {
            float xi = poly[i].X, yi = poly[i].Y;
            float xj = poly[j].X, yj = poly[j].Y;
            if (((yi > py) != (yj > py)) &&
                (px < (xj - xi) * (py - yi) / (yj - yi) + xi))
                inside = !inside;
        }
        return inside;
    }

    const AreaGraph* graph_;

    VehicleFootprintLocal veh_;

    // Non-owning pointers into the graph's polygons (Junction::cached_polygon,
    // RoadSegment::cached_polygon, ParkingSpot::corners). These are built once at
    // area load and never mutated during planning, so a zone is just a list of
    // pointers — no per-call deep copies. Valid as long as graph_ is loaded.
    std::vector<const std::vector<Vec2>*> local_polygons_;
};

#endif
