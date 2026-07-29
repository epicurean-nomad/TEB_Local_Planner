#pragma once

#include "RoadNode.h"
#include "RoadSegment.h"
#include "Junction.h"
#include "Path.h"
#include "GeoOrigin.h"
#include "PathKey.h"
#include "ParkingSpot.h"
#include "RTree.h"
#include "APSP.h"
#include "AppConstants.h"
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <cstdint>

enum class EdgeKind : uint8_t { PATH, JUNCTION };

struct EdgeRef {
    uint32_t edge_id = INVALID_ID;
    EdgeKind kind    = EdgeKind::PATH;
};

class AreaGraph {
public:
    AreaGraph();

    // Area identity — filled by the serializer on load
    char      area_name[64] = {};
    char      area_uuid[37] = {};
    GeoOrigin origin;

    // ── Node operations ──────────────────────────────────────────────────────
    const RoadNode*              getNode(uint32_t id) const;


    const std::vector<RoadNode>& nodes()              const { return nodes_; }

    size_t                       nodeCount()          const { return nodes_.size(); }

    // ── Segment operations ───────────────────────────────────────────────────
    const RoadSegment*              getSegment(uint32_t id) const;
    const std::vector<RoadSegment>& segments()              const { return segments_; }
    size_t                          segmentCount()          const { return segments_.size(); }

    // ── Path operations ──────────────────────────────────────────────────────
    const Path* getPath(uint32_t id) const;
    size_t      pathCount()          const { return paths_.size(); }

    // ── Junction operations ──────────────────────────────────────────────────
    const Junction*              getJunction(uint32_t id) const;
    const std::vector<Junction>& junctions()              const { return junctions_; }
    size_t                       junctionCount()          const { return junctions_.size(); }

    // ── Parking spot operations ──────────────────────────────────────────────
    const ParkingSpot*              getParkingSpot(uint32_t id) const;
    const std::vector<ParkingSpot>& parkingSpots()              const { return parking_spots_; }
    size_t                          parkingSpotCount()          const { return parking_spots_.size(); }

    // ── Adjacency queries ────────────────────────────────────────────────────
    const EdgeRef* edgeBetween(uint32_t from_node_id, uint32_t to_node_id) const;
    const std::unordered_map<uint32_t, EdgeRef>* outgoingEdges(uint32_t from_node_id) const;

    // ── PolyRTree — segments + junctions (built eagerly in rebuildDerivedState) ─
    const PolyRTree& polyRTree() const { return rtree_; }

    // ── Sampled paths ─────────────────────────────────────────────────────────
    bool                  hasSampledPaths()  const { return !sampled_paths_.empty(); }
    const SampledPathMap& sampledPaths()     const { return sampled_paths_; }
    void addSampledPath(PathKey key, std::vector<Point3d> pts) {
        sampled_paths_[key] = std::move(pts);
    }
    void setSampledPaths(SampledPathMap paths) { sampled_paths_ = std::move(paths); }

    // ── APSP table ────────────────────────────────────────────────────────────
    bool             hasApspTable() const { return !apsp_table_.empty(); }
    const APSPTable& apspTable()    const { return apsp_table_; }
    void setApspTable(APSPTable table) { apsp_table_ = std::move(table); }

    // ── Reset ────────────────────────────────────────────────────────────────
    void clear();

    // ── Derived-state maintenance ────────────────────────────────────────────
    void recomputeDirtyJunctionPolygons();
    void rebuildDerivedState();

private:
    std::vector<RoadNode>    nodes_;
    std::vector<RoadSegment> segments_;
    std::vector<Junction>    junctions_;
    std::unordered_map<uint32_t, Path> paths_;
    std::vector<ParkingSpot> parking_spots_;

    std::unordered_map<uint32_t, uint32_t> node_index_;
    std::unordered_map<uint32_t, uint32_t> segment_index_;
    std::unordered_map<uint32_t, uint32_t> junction_index_;
    std::unordered_map<uint32_t, uint32_t> parking_spot_index_;

    std::unordered_map<uint32_t, std::unordered_map<uint32_t, EdgeRef>> adjacency_;

    SampledPathMap sampled_paths_;
    APSPTable      apsp_table_;

    PolyRTree rtree_;

    friend class Serializer;
};
