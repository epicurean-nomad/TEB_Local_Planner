#pragma once
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>
#include <cstddef>

struct ShortestPath {
    float                 cost;   // total distance in metres
    std::vector<uint32_t> nodes;  // sequence from src to dst, inclusive
                                  // empty if unreachable
};

using PathMap   = std::unordered_map<uint32_t, ShortestPath>;
using APSPTable = std::unordered_map<uint32_t,PathMap>;

static constexpr float INF_COST = std::numeric_limits<float>::infinity();
// Per (source, dest) pair: a sorted list of all simple paths.
using APSPAllPathsTable = std::unordered_map<uint32_t,
    std::unordered_map<uint32_t, std::vector<ShortestPath>>>;

// Find all simple paths from source to sink, sorted by cost ascending.
std::vector<ShortestPath> allSimplePaths(
    uint32_t source, uint32_t sink,
    const std::unordered_map<uint32_t,
          std::unordered_map<uint32_t, float>>& graph);

// All-pairs version. WARNING: exponential cost.
APSPAllPathsTable computeAPSPAllPaths(
    const std::unordered_map<uint32_t,
          std::unordered_map<uint32_t, float>>& graph);

// Yen's k-shortest paths. `node_to_junction` (optional) collapses
// consecutive same-junction boundary nodes for output deduplication.
// Empty map = original node-sequence dedup behavior.
std::vector<ShortestPath> kShortestPaths(uint32_t source, uint32_t sink,
    const std::unordered_map<uint32_t,
          std::unordered_map<uint32_t, float>>& graph,
    size_t K,
    const std::unordered_map<uint32_t, uint32_t>& node_to_junction = {});

// Find up to K diverse alternates by iterated Dijkstra with cumulative
// removal of every internal node of each accepted path.
//
// Effect: alternate k+1 is guaranteed to share NO internal nodes with
// alternates 0..k (only source and sink may be shared). As a corollary
// this also makes alternates pairwise segment-disjoint, since two paths
// sharing zero internal nodes cannot share any edge.
//
// Returns up to K paths in cost order. Fewer if removals disconnect
// source from sink before K paths are found.
std::vector<ShortestPath> kAlternatesByNodeRemoval(
    uint32_t source, uint32_t sink,
    const std::unordered_map<uint32_t,
          std::unordered_map<uint32_t, float>>& graph,
    size_t K);

// Find up to K diverse alternates by iterated Dijkstra with cumulative
// removal of every edge of each accepted path (both directions).
//
// Effect: alternates pairwise share no edges. Nodes (including junction
// boundary nodes) may be shared, so paths can still traverse the same
// junction via different gates.
//
// Less aggressive than node removal — graph stays more connected, so
// you typically get more alternates, but they may share nodes.
std::vector<ShortestPath> kAlternatesByEdgeRemoval(
    uint32_t source, uint32_t sink,
    const std::unordered_map<uint32_t,
          std::unordered_map<uint32_t, float>>& graph,
    size_t K);

// Find up to K alternates by omitting each SEGMENT edge of the PRIMARY
// path in REVERSE order (last segment edge first), each on a fresh copy
// of the original graph.
//
// Junction-through edges (both endpoints in the same junction) are
// SKIPPED — omitting one of those just forces the path to enter the
// junction via a different gate, not a meaningful alternative. Only
// segment edges produce alternates.
//
// `node_to_junction` identifies junction-through edges: edge (u, v) is
// junction-through iff both u and v map to the same junction id. Nodes
// not in the map are treated as ordinary (non-junction) nodes, so any
// edge involving them is a segment edge.
//
// For primary p0 = [n0, n1, ..., n_{m-1}], iterates in reverse order:
//   - skip if (n_{i-1}, n_i) is a junction-through edge
//   - otherwise: copy graph (FRESH), remove edge (both directions),
//     Dijkstra. Add to result if non-empty and unique.
//
// Each iteration is INDEPENDENT — previous iteration's removal does
// not persist. Uses APSP_ReplanEdge as the inner Dijkstra call.
//
// Returns at most K paths (primary + up to K-1 alternates), in the
// order: [primary, detour-around-last-segment, detour-around-
// second-to-last-segment, ...].
std::vector<ShortestPath> kAlternatesByOmittingEachEdge(
    uint32_t source, uint32_t sink,
    const std::unordered_map<uint32_t,
          std::unordered_map<uint32_t, float>>& graph,
    size_t K,
    const std::unordered_map<uint32_t, uint32_t>& node_to_junction = {});

// Find up to K alternates by cumulatively removing edges of the PRIMARY
// path, one at a time, and re-planning after each removal.
//
// Algorithm (uses APSP_ReplanEdge for the inner Dijkstra call):
//   1. Find primary p0. Add to result.
//   2. For i = 0 .. len(p0)-2:
//        - Cumulatively remove edge (p0[i], p0[i+1]) from a working
//          graph copy. Both directions removed so two-way segments
//          come fully out.
//        - Run Dijkstra on the depleted graph.
//        - If a new unique path is found, add to result.
//        - Stop when K paths accepted or Dijkstra returns empty.
//
// Gives a gradient of alternates: alt 1 differs from p0 in one edge,
// alt 2 in two edges, ..., final alt is fully edge-disjoint from p0.
// Two alternates in the middle can still share edges with each other.
// Useful for diagnosing which specific edge of p0 is the bottleneck —
// the iteration that returns empty tells you which edge, when removed,
// disconnects source from sink.
std::vector<ShortestPath> kAlternatesByOriginalEdgeRemoval(
    uint32_t source, uint32_t sink,
    const std::unordered_map<uint32_t,
          std::unordered_map<uint32_t, float>>& graph,
    size_t K);

// Find up to K alternates that share NO segments with each other.
//
// Algorithm: iterated Dijkstra. After each accepted path, remove only its
// SEGMENT edges (preserving junction-through edges). Edge (u, v) is a
// segment edge iff u and v are not both boundary nodes of the same
// junction — determined via `node_to_junction`, which maps boundary node
// id → junction id (nodes absent from the map are treated as ordinary
// non-junction nodes, so any edge involving them is a segment).
//
// Effect: alternates are pairwise segment-disjoint. They may share
// junctions (entered/exited through different gates) and may share
// non-junction nodes, but no two alternates traverse the same segment.
//
// Returns up to K paths in cost order. Fewer if removing segments
// disconnects source from sink before K paths are found.
std::vector<ShortestPath> kAlternatesBySegmentDisjoint(
    uint32_t source, uint32_t sink,
    const std::unordered_map<uint32_t,
          std::unordered_map<uint32_t, float>>& graph,
    size_t K,
    const std::unordered_map<uint32_t, uint32_t>& node_to_junction);

// Find up to K diverse alternates by single-edge-at-a-time detour search.
//
// Algorithm:
//   1. Find the shortest path p0 via Dijkstra. Add to result.
//   2. For each edge (u,v) in p0, copy the graph, remove only that one
//      edge (both directions), run Dijkstra. Push non-empty unique
//      results onto a candidate min-heap keyed by cost.
//   3. Pop the cheapest candidate, add to result, then expand it the
//      same way (single-edge removals) to grow the candidate pool.
//   4. Repeat until K paths accepted or candidate heap exhausted.
//
// This is the most permissive of the three alternate-finders. Alternates
// can share most of their edges and nodes with the primary — they only
// have to detour around at least one specific edge. Use this when you
// want lots of alternatives on a small or tightly-coupled graph; use
// the "*Removal" functions when you want stronger pairwise diversity.
std::vector<ShortestPath> kAlternatesByDetour(
    uint32_t source, uint32_t sink,
    const std::unordered_map<uint32_t,
          std::unordered_map<uint32_t, float>>& graph,
    size_t K);

APSPTable computeAPSP(
    const std::unordered_map<uint32_t, std::unordered_map<uint32_t, float>>& graph
);
std::vector<ShortestPath> kAlternatesByGraphStack(
       uint32_t source, uint32_t sink,
       const std::unordered_map<uint32_t, std::unordered_map<uint32_t, float>>& graph,
       size_t K,
       const std::unordered_map<uint32_t, uint32_t>& node_to_junction);

ShortestPath APSP_Replan(uint32_t maskNodeID, uint32_t nodeID, uint32_t goalNodeID, std::unordered_map<uint32_t, std::unordered_map<uint32_t, float>> graph);
ShortestPath APSP_ReplanEdge(uint32_t fromNodeID, uint32_t toNodeID,
                              uint32_t nodeID, uint32_t goalNodeID,
                              std::unordered_map<uint32_t, std::unordered_map<uint32_t, float>> graph);