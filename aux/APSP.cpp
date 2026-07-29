#include "APSP.h"
#include <queue>
#include <vector>
#include <algorithm>
#include <unordered_set>
#include <string>
#include <set> 
#include "RoutingLayer.h"

static constexpr uint32_t NO_PRED = std::numeric_limits<uint32_t>::max();

static std::vector<uint32_t> reconstructPath(
    uint32_t src,
    uint32_t dst,
    const std::unordered_map<uint32_t, uint32_t>& pred
)
{
    std::vector<uint32_t> path;

    uint32_t curr = dst;
    while (curr != src){
        path.push_back(curr);
        auto it = pred.find(curr);
        if (it == pred.end() || it->second == NO_PRED)
            return {}; // unreachable
        curr = it->second;
    }
    path.push_back(src);
    std::reverse(path.begin(), path.end());

    return path;
}


static ShortestPath dijkstra(
    uint32_t source, uint32_t sink,
    const std::unordered_map<uint32_t,
          std::unordered_map<uint32_t, float>>& graph)
{
    using Entry = std::pair<float, uint32_t>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> pq;

    std::unordered_map<uint32_t, float>    dist;
    std::unordered_map<uint32_t, uint32_t> pred;
    dist.reserve(graph.size());
    pred.reserve(graph.size());

    for (const auto& [node, _] : graph) {
        dist[node] = INF_COST;
        pred[node] = NO_PRED;
    }

    // If source or sink isn't in the graph (e.g., removed by a previous
    // alternate iteration), there's nothing to do.
    if (dist.find(source) == dist.end() || dist.find(sink) == dist.end())
        return {INF_COST, {}};

    dist[source] = 0.f;
    pq.push({0.f, source});

    while (!pq.empty()) {
        auto [cost, u] = pq.top();
        pq.pop();

        if (u == sink) break;          // ← early exit, sink is finalized

        if (cost > dist[u]) continue;

        auto it = graph.find(u);
        if (it == graph.end()) continue;

        for (const auto& [v, w] : it->second) {
            float relaxed = dist[u] + w;
            if (relaxed < dist[v]) {
                dist[v] = relaxed;
                pred[v] = u;
                pq.push({dist[v], v});
            }
        }
    }

    // ← reconstruct and return
    if (dist[sink] == INF_COST)
        return {INF_COST, {}};         // unreachable

    return {dist[sink], reconstructPath(source, sink, pred)};
}





static PathMap dijkstra_all(
    uint32_t source,
    const std::unordered_map<uint32_t, std::unordered_map<uint32_t, float>>& graph
)
{
    using Entry = std::pair<float, uint32_t>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> pq;

    std::unordered_map<uint32_t, float> dist;
    std::unordered_map<uint32_t, uint32_t> pred;
    
    dist.reserve(graph.size());
    pred.reserve(graph.size());

    //Initialize all nodes to inf
    for(const auto& [node, _] : graph){
        dist[node] = INF_COST;
        pred[node] = NO_PRED;
    }

    dist[source] = 0.0f;
    pq.push({0.0f, source});

    while(!pq.empty())
    {
        auto [cost, u] = pq.top();
        pq.pop();

        if(cost > dist[u]) continue;

        auto it = graph.find(u);
        if(it == graph.end()) continue;

        for(const auto& [v,w] : it->second){
            float relaxed = dist[u] + w;
            

            if(relaxed < dist[v]){
                dist[v] = relaxed;
                pred[v] = u;
                pq.push({dist[v], v});

            }
        }
    }

    PathMap result;
    result.reserve(graph.size());

    for (const auto& [dst, cost] : dist) {
        if (dst == source) {
            result[dst] = {0.f, {source}};
        } else if (cost == INF_COST) {
            result[dst] = {INF_COST, {}};  // unreachable
        } else {
            result[dst] = {cost, reconstructPath(source, dst, pred)};
        }
    }
    return result;
}

APSPTable computeAPSP(const std::unordered_map<uint32_t, std::unordered_map<uint32_t, float>>& graph)
{
    APSPTable table;
    table.reserve(graph.size());

    for(const auto& [src, _] : graph){
        table[src] = dijkstra_all(src, graph);
    }

    return table;
}

ShortestPath APSP_ReplanEdge(uint32_t fromNodeID, uint32_t toNodeID,
                              uint32_t nodeID, uint32_t goalNodeID,
                              std::unordered_map<uint32_t, std::unordered_map<uint32_t, float>> graph)
{
    /*
    Erase the edge between fromNodeID and toNodeID (both directions)
    and return shortest path from nodeID to goalNodeID
    */
    auto it = graph.find(fromNodeID);
    if (it != graph.end()) it->second.erase(toNodeID);

    //it = graph.find(toNodeID);
    //if (it != graph.end()) it->second.erase(fromNodeID);

    return dijkstra(nodeID, goalNodeID, graph);
}
ShortestPath APSP_Replan(uint32_t maskNodeID, uint32_t nodeID, uint32_t goalNodeID, std::unordered_map<uint32_t, std::unordered_map<uint32_t, float>> graph)
{
    /*
    Erase maskNode from the graph and return shortest path from nodeID to goalNodeID
    */
    graph.erase(maskNodeID);

    for (auto& [node, neighbours] : graph)
        neighbours.erase(maskNodeID);

    return dijkstra(nodeID, goalNodeID, graph);
}


std::vector<ShortestPath> allSimplePaths(
    uint32_t source, uint32_t sink,
    const std::unordered_map<uint32_t,
          std::unordered_map<uint32_t, float>>& graph)
{
    std::vector<ShortestPath> all_paths;
 
    // Edge case: source == sink. Single trivial path with cost 0.
    if (source == sink) {
        all_paths.push_back({0.f, {source}});
        return all_paths;
    }
 
    // If source not in graph, no paths.
    if (graph.find(source) == graph.end()) return all_paths;
 
    // Each stack frame: current node, edge iterator (into graph[node]), cost so far.
    struct Frame {
        uint32_t node;
        std::unordered_map<uint32_t, float>::const_iterator it;
        std::unordered_map<uint32_t, float>::const_iterator end;
        float cost_to_node;
    };
 
    std::vector<Frame> stack;
    std::vector<uint32_t> path;          // current path under exploration
    std::unordered_set<uint32_t> in_path; // membership check for path
 
    // Push source.
    auto src_neighbors = graph.find(source);
    stack.push_back({source,
                     src_neighbors->second.begin(),
                     src_neighbors->second.end(),
                     0.f});
    path.push_back(source);
    in_path.insert(source);
 
    while (!stack.empty()) {
        Frame& top = stack.back();
 
        // Are there more neighbors to explore from this node?
        if (top.it == top.end) {
            // Done with this node — backtrack.
            in_path.erase(top.node);
            path.pop_back();
            stack.pop_back();
            continue;
        }
 
        uint32_t next_node = top.it->first;
        float    edge_w    = top.it->second;
        ++top.it;  // advance iterator so we don't revisit this neighbor
 
        // Skip if already in path (would create a cycle).
        if (in_path.count(next_node)) continue;
 
        float new_cost = top.cost_to_node + edge_w;
 
        if (next_node == sink) {
            // Found a complete path.
            path.push_back(sink);
            all_paths.push_back({new_cost, path});
            path.pop_back();
            // Don't descend further; continue exploring other neighbors of top.
            continue;
        }
 
        // Descend: push a new frame for next_node.
        auto next_neighbors = graph.find(next_node);
        if (next_neighbors == graph.end()) continue;  // dead end
 
        stack.push_back({next_node,
                         next_neighbors->second.begin(),
                         next_neighbors->second.end(),
                         new_cost});
        path.push_back(next_node);
        in_path.insert(next_node);
    }
 
    // Sort paths by cost ascending (shortest first).
    std::sort(all_paths.begin(), all_paths.end(),
              [](const ShortestPath& a, const ShortestPath& b) {
                  return a.cost < b.cost;
              });
 
    return all_paths;
}
 
 
// ════════════════════════════════════════════════════════════════════
//  ALL-PAIRS ALL-PATHS
// ════════════════════════════════════════════════════════════════════
//
// For every (source, sink) pair, enumerate all simple paths sorted by cost.
// WARNING: this is exponential in the worst case. Only safe for small
// graphs or graphs with low branching factor.
 
APSPAllPathsTable computeAPSPAllPaths(
    const std::unordered_map<uint32_t,
          std::unordered_map<uint32_t, float>>& graph)
{
    APSPAllPathsTable table;
    table.reserve(graph.size());
 
    for (const auto& [src, _] : graph) {
        std::unordered_map<uint32_t, std::vector<ShortestPath>>& row = table[src];
        row.reserve(graph.size());
        for (const auto& [dst, _2] : graph) {
            row[dst] = allSimplePaths(src, dst, graph);
        }
    }
 
    return table;
}

std::vector<ShortestPath> kShortestPaths(
    uint32_t source, uint32_t sink,
    const std::unordered_map<uint32_t,
          std::unordered_map<uint32_t, float>>& graph,
    size_t K,
    const std::unordered_map<uint32_t, uint32_t>& node_to_junction)
{
    std::vector<ShortestPath> result;
    if (K == 0) return result;
 
    // Edge case: source == sink.
    if (source == sink) {
        result.push_back({0.f, {source}});
        return result;
    }
 
    // 1. Shortest path via Dijkstra.
    ShortestPath p0 = dijkstra(source, sink, graph);
    if (p0.nodes.empty()) return result;  // unreachable
    result.push_back(p0);
 
    // Candidate priority queue, ordered by cost ascending.
    auto cmp = [](const ShortestPath& a, const ShortestPath& b) {
        return a.cost > b.cost;  // min-heap
    };
    std::priority_queue<ShortestPath, std::vector<ShortestPath>, decltype(cmp)>
        candidates(cmp);
 
    // pathKey: canonical dedup key. Empty node_to_junction → original
    // node-sequence dedup. Non-empty → consecutive boundary nodes of the
    // same junction collapse to one token (J<jid>); non-junction nodes
    // emit as N<id>. J/N prefixes prevent id-collision between node and
    // junction id namespaces.
    auto pathKey = [&](const std::vector<uint32_t>& nodes) {
        std::string s;
        s.reserve(nodes.size() * 8);
        uint32_t prev_junction_id = 0;
        bool prev_was_junction = false;
        for (auto n : nodes) {
            auto it = node_to_junction.find(n);
            const bool is_junction = (it != node_to_junction.end());
            if (is_junction) {
                const uint32_t jid = it->second;
                if (prev_was_junction && jid == prev_junction_id) continue;
                s += 'J';
                s += std::to_string(jid);
                s += ',';
                prev_was_junction = true;
                prev_junction_id  = jid;
            } else {
                s += 'N';
                s += std::to_string(n);
                s += ',';
                prev_was_junction = false;
            }
        }
        return s;
    };

    std::unordered_set<std::string> seen_paths;
    seen_paths.insert(pathKey(p0.nodes));
 
    while (result.size() < K) {
        const ShortestPath& prev = result.back();
 
        // For each node in prev (except the last), treat it as the "spur node".
        for (size_t i = 0; i + 1 < prev.nodes.size(); ++i) {
            uint32_t spur_node = prev.nodes[i];
            std::vector<uint32_t> root_path(prev.nodes.begin(),
                                            prev.nodes.begin() + i + 1);
 
            // Build a modified graph copy with selected edges/nodes removed.
            auto modified = graph;
 
            // Remove the edge spur_node → next_in_path for every accepted path
            // that shares the same root_path prefix.
            for (const auto& accepted : result) {
                if (accepted.nodes.size() <= i + 1) continue;
                bool same_root = std::equal(root_path.begin(), root_path.end(),
                                            accepted.nodes.begin());
                if (!same_root) continue;
                uint32_t next_node = accepted.nodes[i + 1];
                auto it = modified.find(spur_node);
                if (it != modified.end()) it->second.erase(next_node);
            }
 
            // Remove all nodes in root_path EXCEPT the spur_node itself.
            for (size_t j = 0; j < i; ++j) {
                uint32_t r = root_path[j];
                modified.erase(r);
                for (auto& [n, neighbors] : modified) {
                    neighbors.erase(r);
                }
            }
 
            // Compute spur path from spur_node to sink in the modified graph.
            ShortestPath spur = dijkstra(spur_node, sink, modified);
            if (spur.nodes.empty()) continue;  // no detour exists from here
 
            // Build total candidate: root_path (without its last = spur_node)
            // + spur.nodes (which starts at spur_node).
            ShortestPath candidate;
            candidate.nodes = root_path;
            candidate.nodes.pop_back();  // remove duplicate spur_node
            candidate.nodes.insert(candidate.nodes.end(),
                                   spur.nodes.begin(), spur.nodes.end());
 
            // Cost: sum edge weights along the candidate.
            float cost = 0.f;
            bool valid_cost = true;
            for (size_t k = 0; k + 1 < candidate.nodes.size(); ++k) {
                uint32_t u = candidate.nodes[k];
                uint32_t v = candidate.nodes[k + 1];
                auto uit = graph.find(u);
                if (uit == graph.end()) { valid_cost = false; break; }
                auto vit = uit->second.find(v);
                if (vit == uit->second.end()) { valid_cost = false; break; }
                cost += vit->second;
            }
            if (!valid_cost) continue;
            candidate.cost = cost;
 
            // Skip if we've already produced this candidate (dedupe).
            if (seen_paths.count(pathKey(candidate.nodes))) continue;
            seen_paths.insert(pathKey(candidate.nodes));
 
            candidates.push(candidate);
        }
 
        if (candidates.empty()) break;  // no more alternatives exist
 
        result.push_back(candidates.top());
        candidates.pop();
    }
 
    return result;
}

// ════════════════════════════════════════════════════════════════════
//  ALTERNATES BY ITERATED REMOVAL
// ════════════════════════════════════════════════════════════════════
//
// Two simple ways to get K diverse alternates without using Yen's:
//   (a) Find shortest path, ban its internal NODES, repeat.
//   (b) Find shortest path, ban its EDGES, repeat.
//
// Both walk a mutable copy of the graph that shrinks each iteration.
// Both return up to K paths in cost order (cost monotonically increases
// since the graph only loses edges/nodes). The loop terminates early if
// no path from source to sink remains.
//
// (a) is the stronger guarantee: alternates are pairwise node-disjoint
//     (apart from source/sink), which makes them edge-disjoint too. But
//     it removes more, so the graph disconnects sooner and you tend to
//     get fewer alternates back.
//
// (b) lets alternates share nodes (different boundary nodes of the same
//     junction count as "shared via the same junction" — handy when you
//     want junction diversity but not full route divergence). Tends to
//     return more alternates.

std::vector<ShortestPath> kAlternatesByNodeRemoval(
    uint32_t source, uint32_t sink,
    const std::unordered_map<uint32_t,
          std::unordered_map<uint32_t, float>>& graph,
    size_t K)
{
    std::vector<ShortestPath> result;
    if (K == 0) return result;
    if (source == sink) {
        result.push_back({0.f, {source}});
        return result;
    }

    // Mutable working copy of the graph. After each accepted path, all
    // of its INTERNAL nodes (indices 1..n-2) are erased — both as a
    // node entry and as a neighbor in every other node's adjacency map.
    auto remaining = graph;

    printf("[NodeRemoval] target K=%zu, src=%u sink=%u, initial graph: %zu nodes\n",
           K, source, sink, remaining.size());

    while (result.size() < K) {
        ShortestPath p = dijkstra(source, sink, remaining);
        if (p.nodes.empty()) {
            printf("[NodeRemoval] iter %zu: no path remains, stopping (%zu of %zu found)\n",
                   result.size(), result.size(), K);
            break;       // exhausted — no more reachable alternates
        }
        result.push_back(p);
        printf("[NodeRemoval] iter %zu: cost=%.2f, len=%zu, banning %zu internal nodes\n",
               result.size() - 1, p.cost, p.nodes.size(),
               p.nodes.size() >= 2 ? p.nodes.size() - 2 : 0);

        // Remove all internal nodes of this path. Source and sink stay.
        // After this, no future alternate can pass through any of these
        // banned nodes — which forces a structurally different route.
        for (size_t i = 1; i + 1 < p.nodes.size(); ++i) {
            uint32_t n = p.nodes[i];
            remaining.erase(n);                       // drop adjacency entry
            for (auto& [_, neighbors] : remaining) {  // drop incoming edges
                neighbors.erase(n);
            }
        }
    }
    return result;
}

std::vector<ShortestPath> kAlternatesByEdgeRemoval(
    uint32_t source, uint32_t sink,
    const std::unordered_map<uint32_t,
          std::unordered_map<uint32_t, float>>& graph,
    size_t K)
{
    std::vector<ShortestPath> result;
    if (K == 0) return result;
    if (source == sink) {
        result.push_back({0.f, {source}});
        return result;
    }

    auto remaining = graph;

    // Count edges (sum of adjacency map sizes) for diagnostics.
    auto countEdges = [&](const std::unordered_map<uint32_t,
                                std::unordered_map<uint32_t, float>>& g) {
        size_t n = 0;
        for (const auto& [_, neigh] : g) n += neigh.size();
        return n;
    };
    printf("[EdgeRemoval] target K=%zu, src=%u sink=%u, initial graph: %zu nodes, %zu directed edges\n",
           K, source, sink, remaining.size(), countEdges(remaining));

    while (result.size() < K) {
        ShortestPath p = dijkstra(source, sink, remaining);
        if (p.nodes.empty()) {
            printf("[EdgeRemoval] iter %zu: no path remains, stopping (%zu of %zu found, %zu directed edges left)\n",
                   result.size(), result.size(), K, countEdges(remaining));
            break;
        }
        result.push_back(p);

        // Remove every edge of this path. Both directions — the graph
        // representation here happens to be directed in form but the
        // road graph is logically undirected, so we erase u→v and v→u
        // both. (If your data is truly directed and you want directed
        // semantics, remove the second erase.)
        size_t removed = 0;
        for (size_t i = 0; i + 1 < p.nodes.size(); ++i) {
            uint32_t u = p.nodes[i];
            uint32_t v = p.nodes[i + 1];
            auto it = remaining.find(u);
            if (it != remaining.end() && it->second.erase(v) > 0) ++removed;
            it = remaining.find(v);
            if (it != remaining.end() && it->second.erase(u) > 0) ++removed;
        }
        printf("[EdgeRemoval] iter %zu: cost=%.2f, len=%zu, removed %zu directed edges (%zu remain)\n",
               result.size() - 1, p.cost, p.nodes.size(), removed, countEdges(remaining));
    }
    return result;
}

std::vector<ShortestPath> kAlternatesByOmittingEachEdge(
    uint32_t source, uint32_t sink,
    const std::unordered_map<uint32_t,
          std::unordered_map<uint32_t, float>>& graph,
            size_t K,
            const std::unordered_map<uint32_t, uint32_t>& node_to_junction)
{
    std::vector<ShortestPath> result;
    if (K == 0) return result;
    if (source == sink) {
        result.push_back({0.f, {source}});
        return result;
    }

    // Primary.
    ShortestPath p0 = dijkstra(source, sink, graph);
    if (p0.nodes.empty()) {
        printf("[OmitEachEdge] no primary path; aborting\n");
        return result;
    }
    result.push_back(p0);
    if (p0.nodes.size() < 2) return result;

    // Junction-through edge predicate: u and v both map to the same
    // junction id. Nodes absent from the map are not in any junction.
    auto isJunctionEdge = [&](uint32_t u, uint32_t v) -> bool {
        auto iu = node_to_junction.find(u);
        auto iv = node_to_junction.find(v);
        if (iu == node_to_junction.end()) return false;
        if (iv == node_to_junction.end()) return false;
        return iu->second == iv->second;
    };

    // Pre-count segment edges in p0 just for the header log.
    size_t seg_edges_in_p0 = 0;
    for (size_t i = 0; i + 1 < p0.nodes.size(); ++i) {
        if (!isJunctionEdge(p0.nodes[i], p0.nodes[i + 1])) ++seg_edges_in_p0;
    }
    printf("[OmitEachEdge] primary cost=%.2f len=%zu, %zu segment edges to omit "
           "(junction-through edges skipped), in reverse order\n",
           p0.cost, p0.nodes.size(), seg_edges_in_p0);

    auto pathKey = [](const std::vector<uint32_t>& nodes) {
        std::string s;
        s.reserve(nodes.size() * 6);
        for (auto n : nodes) { s += std::to_string(n); s += ','; }
        return s;
    };
    std::unordered_set<std::string> seen;
    seen.insert(pathKey(p0.nodes));

    // Walk edges of p0 from end to start.
    for (size_t edge_idx = p0.nodes.size() - 1;
         edge_idx > 0 && result.size() < K;
         --edge_idx)
    {
        const uint32_t u = p0.nodes[edge_idx - 1];
        const uint32_t v = p0.nodes[edge_idx];

        if (isJunctionEdge(u, v)) {
            printf("[OmitEachEdge] edge (%u-%u): junction-through, skipping\n", u, v);
            continue;
        }

        // FRESH copy of the ORIGINAL graph each iteration. Pre-remove v→u
        // here so the version handed to APSP_ReplanEdge already lacks the
        // reverse direction; APSP_ReplanEdge then removes u→v in its own
        // copy and runs Dijkstra. Both directions of the SEGMENT come out.
        auto modified = graph;
        {
            auto it = modified.find(v);
            if (it != modified.end()) it->second.erase(u);
        }
        ShortestPath candidate = APSP_ReplanEdge(u, v, source, sink, modified);

        if (candidate.nodes.empty()) {
            printf("[OmitEachEdge] omit segment (%u-%u): no path remains, skipping\n", u, v);
            continue;
        }

        const std::string k = pathKey(candidate.nodes);
        if (seen.count(k)) {
            printf("[OmitEachEdge] omit segment (%u-%u): same path as primary or prior, "
                   "skipping\n", u, v);
            continue;
        }
        seen.insert(k);
        result.push_back(candidate);
        printf("[OmitEachEdge] omit segment (%u-%u): cost=%.2f len=%zu accepted\n",
               u, v, candidate.cost, candidate.nodes.size());
    }

    if (result.size() < K) {
        printf("[OmitEachEdge] returning %zu paths (target was %zu) — out of segments "
               "to omit or all duplicates\n", result.size(), K);
    }
    return result;
}
std::vector<ShortestPath> kAlternatesByGraphStack(
    uint32_t source, uint32_t sink,
    const std::unordered_map<uint32_t,
          std::unordered_map<uint32_t, float>>& graph,
    size_t K,
    const std::unordered_map<uint32_t, uint32_t>& node_to_junction)
{
    std::vector<ShortestPath> result;
    if (K == 0) return result;
    if (source == sink) { result.push_back({0.f, {source}}); return result; }

    using Edge = std::pair<uint32_t, uint32_t>;

    // Don't cut junction-through edges — removing one just forces a different
    // gate of the SAME junction, not a meaningfully different route.
    auto isJunctionEdge = [&](uint32_t u, uint32_t v) -> bool {
        auto iu = node_to_junction.find(u);
        auto iv = node_to_junction.find(v);
        if (iu == node_to_junction.end() || iv == node_to_junction.end()) return false;
        return iu->second == iv->second;
    };

    auto pathKey = [](const std::vector<uint32_t>& nodes) {
        std::string s; s.reserve(nodes.size() * 6);
        for (auto n : nodes) { s += std::to_string(n); s += ','; }
        return s;
    };

    // Build (original graph − removed edges). `removed` holds directed edges;
    // we always insert both directions, so two-way segments come fully out.
    auto buildGraph = [&](const std::set<Edge>& removed) {
        auto g = graph;
        for (const auto& [u, v] : removed) {
            auto it = g.find(u);
            if (it != g.end()) it->second.erase(v);
        }
        return g;
    };

    // Worklist item: a path + the removed-edge set whose graph produced it.
    struct Item {
        ShortestPath   path;
        std::set<Edge> removed;
    };
    auto cmp = [](const Item& a, const Item& b) { return a.path.cost > b.path.cost; };
    std::priority_queue<Item, std::vector<Item>, decltype(cmp)> heap(cmp);  // min-heap by cost

    ShortestPath p0 = dijkstra(source, sink, graph);
    if (p0.nodes.empty()) { printf("[GraphStack] no primary path; aborting\n"); return result; }

    std::unordered_set<std::string> seen_paths;   // accepted/queued path node-sequences
    std::set<std::set<Edge>>        seen_graphs;   // removed-edge sets already expanded

    heap.push({p0, {}});
    printf("[GraphStack] target K=%zu, src=%u sink=%u, primary cost=%.2f len=%zu\n",
           K, source, sink, p0.cost, p0.nodes.size());

    while (!heap.empty() && result.size() < K) {
        Item cur = heap.top(); heap.pop();

        // Accept this path if it's new.
        if (!seen_paths.insert(pathKey(cur.path.nodes)).second) continue;
        result.push_back(cur.path);
        printf("[GraphStack] accepted %zu: cost=%.2f len=%zu (removed %zu edges)\n",
               result.size() - 1, cur.path.cost, cur.path.nodes.size(), cur.removed.size());
        if (result.size() >= K) break;

        // Expand: cut each SEGMENT edge of this path (on top of what's already
        // removed), rebuild, re-route, and push the result.
        for (size_t i = 0; i + 1 < cur.path.nodes.size(); ++i) {
            uint32_t u = cur.path.nodes[i];
            uint32_t v = cur.path.nodes[i + 1];
            if (isJunctionEdge(u, v)) continue;

            std::set<Edge> childRemoved = cur.removed;
            childRemoved.insert({u, v});
            childRemoved.insert({v, u});

            // Skip removed-sets we've already expanded.
            if (!seen_graphs.insert(childRemoved).second) continue;

            ShortestPath child = dijkstra(source, sink, buildGraph(childRemoved));
            if (child.nodes.empty()) continue;
            if (seen_paths.count(pathKey(child.nodes))) continue;
            heap.push({std::move(child), std::move(childRemoved)});
        }
    }

    if (result.size() < K)
        printf("[GraphStack] returning %zu paths (target %zu) — search exhausted\n",
               result.size(), K);
    return result;
}

std::vector<ShortestPath> kAlternatesByOriginalEdgeRemoval(
    uint32_t source, uint32_t sink,
    const std::unordered_map<uint32_t,
          std::unordered_map<uint32_t, float>>& graph,
    size_t K)
{
    std::vector<ShortestPath> result;
    if (K == 0) return result;
    if (source == sink) {
        result.push_back({0.f, {source}});
        return result;
    }

    // Find the primary first.
    ShortestPath p0 = dijkstra(source, sink, graph);
    if (p0.nodes.empty()) {
        printf("[OrigEdgeRemoval] no primary path; aborting\n");
        return result;
    }
    result.push_back(p0);
    printf("[OrigEdgeRemoval] primary cost=%.2f len=%zu (%zu edges to iterate)\n",
           p0.cost, p0.nodes.size(),
           p0.nodes.size() >= 1 ? p0.nodes.size() - 1 : 0);

    // Working copy that accumulates removals across iterations.
    auto current = graph;

    auto pathKey = [](const std::vector<uint32_t>& nodes) {
        std::string s;
        s.reserve(nodes.size() * 6);
        for (auto n : nodes) { s += std::to_string(n); s += ','; }
        return s;
    };
    std::unordered_set<std::string> seen;
    seen.insert(pathKey(p0.nodes));

    // Cumulatively remove edges of p0, one at a time, in p0's traversal order.
    // Each iteration: pre-remove the reverse direction in `current` so two-way
    // segments come fully out, then hand the modified graph to APSP_ReplanEdge
    // which copies, removes the forward direction in its copy, and runs Dijkstra.
    // We then also remove the forward direction in our own `current` so the
    // next iteration's view of `current` reflects this iteration's removal too.
    for (size_t i = 0; i + 1 < p0.nodes.size() && result.size() < K; ++i) {
        uint32_t u = p0.nodes[i];
        uint32_t v = p0.nodes[i + 1];

        // Pre-remove v→u in our cumulative current. (APSP_ReplanEdge will
        // remove u→v itself in its local copy.)
        {
            auto it = current.find(v);
            if (it != current.end()) it->second.erase(u);
        }

        // Run Dijkstra via APSP_ReplanEdge: copies current, removes u→v in
        // the copy, runs Dijkstra on the result. current itself isn't changed
        // by this call — we update it manually below.
        ShortestPath candidate = APSP_ReplanEdge(u, v, source, sink, current);

        // Also remove u→v in our cumulative current, so next iteration sees it gone.
        {
            auto it = current.find(u);
            if (it != current.end()) it->second.erase(v);
        }

        if (candidate.nodes.empty()) {
            printf("[OrigEdgeRemoval] iter %zu: removed edge (%u-%u) cumulatively → "
                   "no path remains, stopping\n",
                   i + 1, u, v);
            break;
        }

        std::string k = pathKey(candidate.nodes);
        if (seen.count(k)) {
            printf("[OrigEdgeRemoval] iter %zu: removed (%u-%u), candidate duplicates "
                   "an earlier result, skipping\n",
                   i + 1, u, v);
            continue;
        }
        seen.insert(k);
        result.push_back(candidate);
        printf("[OrigEdgeRemoval] iter %zu: removed (%u-%u) cumulatively → "
               "cost=%.2f len=%zu accepted\n",
               i + 1, u, v, candidate.cost, candidate.nodes.size());
    }

    if (result.size() < K) {
        printf("[OrigEdgeRemoval] stopped at %zu paths (target was %zu)\n",
               result.size(), K);
    }
    return result;
}

std::vector<ShortestPath> kAlternatesBySegmentDisjoint(
    uint32_t source, uint32_t sink,
    const std::unordered_map<uint32_t,
          std::unordered_map<uint32_t, float>>& graph,
    size_t K,
    const std::unordered_map<uint32_t, uint32_t>& node_to_junction)
{
    std::vector<ShortestPath> result;
    if (K == 0) return result;
    if (source == sink) {
        result.push_back({0.f, {source}});
        return result;
    }

    // Edge (u, v) is a SEGMENT edge unless u and v are both boundary nodes
    // of the SAME junction (in which case it's a junction-through edge).
    auto isSegmentEdge = [&](uint32_t u, uint32_t v) -> bool {
        auto iu = node_to_junction.find(u);
        auto iv = node_to_junction.find(v);
        if (iu == node_to_junction.end()) return true;
        if (iv == node_to_junction.end()) return true;
        return iu->second != iv->second;
    };

    auto countEdges = [&](const std::unordered_map<uint32_t,
                                std::unordered_map<uint32_t, float>>& g) {
        size_t n = 0;
        for (const auto& [_, neigh] : g) n += neigh.size();
        return n;
    };

    auto remaining = graph;
    printf("[SegDisjoint] target K=%zu, src=%u sink=%u, |node_to_junction|=%zu, "
           "initial graph: %zu nodes, %zu directed edges\n",
           K, source, sink, node_to_junction.size(),
           remaining.size(), countEdges(remaining));

    while (result.size() < K) {
        ShortestPath p = dijkstra(source, sink, remaining);
        if (p.nodes.empty()) {
            printf("[SegDisjoint] iter %zu: no path remains, stopping (%zu of %zu found, "
                   "%zu directed edges left)\n",
                   result.size(), result.size(), K, countEdges(remaining));
            break;
        }
        result.push_back(p);

        // Remove segment edges only. Junction-through edges stay so the
        // next iteration can re-enter the same junctions via other gates.
        size_t removed_seg = 0, kept_jxn = 0;
        for (size_t i = 0; i + 1 < p.nodes.size(); ++i) {
            uint32_t u = p.nodes[i];
            uint32_t v = p.nodes[i + 1];
            if (!isSegmentEdge(u, v)) { ++kept_jxn; continue; }
            auto it = remaining.find(u);
            if (it != remaining.end() && it->second.erase(v) > 0) ++removed_seg;
            it = remaining.find(v);
            if (it != remaining.end() && it->second.erase(u) > 0) ++removed_seg;
        }
        printf("[SegDisjoint] iter %zu: cost=%.2f len=%zu, removed %zu segment-edge directions, kept %zu junction edges (%zu remain)\n",
               result.size() - 1, p.cost, p.nodes.size(),
               removed_seg, kept_jxn, countEdges(remaining));
    }
    return result;
}

std::vector<ShortestPath> kAlternatesByDetour(
    uint32_t source, uint32_t sink,
    const std::unordered_map<uint32_t,
          std::unordered_map<uint32_t, float>>& graph,
    size_t K)
{
    std::vector<ShortestPath> result;
    if (K == 0) return result;
    if (source == sink) {
        result.push_back({0.f, {source}});
        return result;
    }

    // Primary path.
    ShortestPath p0 = dijkstra(source, sink, graph);
    if (p0.nodes.empty()) {
        printf("[Detour] no primary path; aborting\n");
        return result;
    }
    result.push_back(p0);

    // Dedup keyed on node sequence.
    auto pathKey = [](const std::vector<uint32_t>& nodes) {
        std::string s;
        s.reserve(nodes.size() * 6);
        for (auto n : nodes) { s += std::to_string(n); s += ','; }
        return s;
    };
    std::unordered_set<std::string> seen;
    seen.insert(pathKey(p0.nodes));

    // Min-heap of candidate alternates by cost.
    auto cmp = [](const ShortestPath& a, const ShortestPath& b) {
        return a.cost > b.cost;
    };
    std::priority_queue<ShortestPath, std::vector<ShortestPath>, decltype(cmp)>
        heap(cmp);

    printf("[Detour] target K=%zu, src=%u sink=%u, primary cost=%.2f len=%zu\n",
           K, source, sink, p0.cost, p0.nodes.size());

    // For each edge in `p`, remove it (only) and Dijkstra; collect non-empty
    // unique results onto the heap. `seen` is checked here for early skip,
    // but actually inserted only when the candidate is popped and accepted
    // — that way duplicate candidates produced from different expansions
    // don't get accidentally banned before we even pop them.
    auto generateCandidates = [&](const ShortestPath& p) {
        size_t pushed = 0, attempted = 0;
        for (size_t i = 0; i + 1 < p.nodes.size(); ++i) {
            ++attempted;
            uint32_t u = p.nodes[i];
            uint32_t v = p.nodes[i + 1];

            // Copy graph and remove only this one edge (both directions).
            auto modified = graph;
            auto it = modified.find(u);
            if (it != modified.end()) it->second.erase(v);
            it = modified.find(v);
            if (it != modified.end()) it->second.erase(u);

            ShortestPath cand = dijkstra(source, sink, modified);
            if (cand.nodes.empty()) continue;
            if (seen.count(pathKey(cand.nodes))) continue;

            heap.push(std::move(cand));
            ++pushed;
        }
        return std::make_pair(attempted, pushed);
    };

    auto [att0, push0] = generateCandidates(p0);
    printf("[Detour] expanded primary: %zu edges tried, %zu new candidates pushed\n",
           att0, push0);

    while (result.size() < K && !heap.empty()) {
        ShortestPath next = heap.top();
        heap.pop();

        // De-dup at pop time. Multiple expansions can produce the same
        // candidate path; the first one popped wins.
        std::string k = pathKey(next.nodes);
        if (seen.count(k)) continue;
        seen.insert(k);

        result.push_back(next);
        printf("[Detour] iter %zu: cost=%.2f len=%zu accepted\n",
               result.size() - 1, next.cost, next.nodes.size());

        // Expand the newly accepted path to grow the candidate pool.
        auto [att, push] = generateCandidates(next);
        printf("[Detour]   expanded: %zu edges tried, %zu new candidates pushed\n",
               att, push);
    }

    if (result.size() < K) {
        printf("[Detour] stopping: heap empty after %zu paths (target was %zu)\n",
               result.size(), K);
    }

    return result;
}