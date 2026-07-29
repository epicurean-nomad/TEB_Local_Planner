#pragma once

#include "AreaGraph.h"
#include "AppConstants.h"
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

/**
 * RoutingLayer
 *
 * Owns an AreaGraph loaded from a .area file and exposes graph-topology
 * queries, spatial lookups, and derived runtime data (sampled paths, APSP).
 *
 * Graph views (node topology, weighted, junction) are built once during
 * loadArea() and returned by const& — no per-call rebuild.
 *
 * Derived data (sampled paths, APSP table) live in the AreaGraph so they
 * can be persisted via saveRuntime() and restored via loadRuntime().
 *
 * Logging:
 *   Pass enable_logging = true (default) to print timestamped progress lines
 *   to stdout.  When disabled no clock calls are made.
 */
class RoutingLayer {
public:
    explicit RoutingLayer(bool enable_logging = true);

    // ── Lifecycle ─────────────────────────────────────────────────────────────

    /** Load a .area file, build the graph and all cached views. */
    bool loadArea(const std::string& filepath, std::string& error);

    /** Persist sampled paths and APSP table to a runtime cache file. */
    bool saveRuntime(const std::string& filepath, std::string& error) const;

    /** Restore sampled paths and APSP table from a previously saved cache. */
    bool loadRuntime(const std::string& filepath, std::string& error);

    /** Clear all loaded data, caches, and derived data. */
    void clear();

    // ── Graph views (built once at load, O(1) access) ─────────────────────────

    /** node_id → set of directly reachable node_ids. */
    const std::unordered_map<uint32_t, std::unordered_set<uint32_t>>&
    getGraph() const { return graph_topo_; }

    /** node_id → { neighbour_id → arc_length_m }. Ready for Dijkstra / A*. */
    const std::unordered_map<uint32_t, std::unordered_map<uint32_t, float>>&
    getGraphWithCost() const { return graph_weighted_; }

    /** junction_id → set of directly connected junction_ids. */
    const std::unordered_map<uint32_t, std::unordered_set<uint32_t>>&
    getJunctionGraph() const { return junction_adj_; }

    // ── Spatial query ─────────────────────────────────────────────────────────

    /** Nearest polygon (segment or junction) to (x, y).
     *  out_type receives PolyType::SEGMENT or PolyType::JUNCTION.
     *  out_dist_sq receives the squared distance (0 when inside a junction polygon).
     *  Returns INVALID_ID when no area is loaded. */
    uint32_t nearestPoly(float x, float y,
                         PolyType* out_type,
                         float*    out_dist_sq = nullptr) const;

    /**
     * Dense centerline waypoints between two directly connected nodes,
     * spaced spacing_m metres apart.  Returns empty for junction edges.
     */
    std::vector<Point3d> getControlPoints(uint32_t from_node_id,
                                          uint32_t to_node_id,
                                          float    spacing_m = 0.1f) const;

    int8_t getLaneId( uint32_t prev_node, double x, double y, double theta) const;

    // ── Derived data — sampled paths ──────────────────────────────────────────

    bool                  hasSampledPaths()  const { return graph_.hasSampledPaths(); }
    const SampledPathMap& getSampledPaths()  const { return graph_.sampledPaths(); }

    void addSampledPath(uint32_t from, uint32_t to, std::vector<Point3d> pts) {
        graph_.addSampledPath({from, to}, std::move(pts));
    }

    void addSampledPath(uint32_t from, uint32_t to, Dir dir, std::vector<Point3d> pts) {
        graph_.addSampledPath(PathKey(from, to, dir), std::move(pts));
    }
    void setSampledPaths(SampledPathMap paths) {
        graph_.setSampledPaths(std::move(paths));
    }

    // ── Derived data — APSP table ─────────────────────────────────────────────

    bool             hasApspTable() const { return graph_.hasApspTable(); }
    const APSPTable& getApspTable() const { return graph_.apspTable(); }

    void setApspTable(APSPTable table) { graph_.setApspTable(std::move(table)); }

    // ── Accessors ─────────────────────────────────────────────────────────────

    const AreaGraph& graph()    const { return graph_; }
    bool             isLoaded() const { return is_loaded_; }
    

    // ── Simple getters ─────────────────────────────────────────────────────────────
    bool isParkingNode(uint32_t id) {
        auto node = graph_.getNode(id);
        return node && node->isParkingTerminal();
    }

private:
    void      buildCaches();
    void      log(const std::string& msg) const;
    double    elapsedMs(long long start_ns) const;
    long long nowNs() const;

    AreaGraph graph_;
    bool      is_loaded_       = false;
    bool      logging_enabled_ = true;

    std::unordered_map<uint32_t, std::unordered_set<uint32_t>>        graph_topo_;
    std::unordered_map<uint32_t, std::unordered_map<uint32_t, float>> graph_weighted_;
    std::unordered_map<uint32_t, std::unordered_set<uint32_t>>        junction_adj_;
};
