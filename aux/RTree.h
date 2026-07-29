// PolyRTree.h  —  static STR-packed R-tree over road segments and junction polygons
#pragma once
#include <vector>
#include <array>
#include <algorithm>
#include <cmath>
#include <limits>
#include <cstdint>
#include <numeric>
#include "RoadSegment.h"   // pulls in Point3d.h → Vec2{X,Y}, GeometryPoint

// ---------------------------------------------------------------
// Public types
// ---------------------------------------------------------------

enum class PolyType : uint8_t { SEGMENT, JUNCTION, PARKING_SPOT };

// Leaf stored in the tree.  Built once from graph data at load time.
// verts is always a closed polygon (SEGMENT: road-band outline from
// cached_polygon, JUNCTION: cached_polygon, PARKING_SPOT: 4 corners).
struct SpatialPoly {
    uint32_t          id   = INVALID_ID;
    PolyType          type = PolyType::SEGMENT;
    std::vector<Vec2> verts;
};

// Result element returned by rangeQuery.
struct PolyHit {
    uint32_t id;
    PolyType type;
};

// ---------------------------------------------------------------
// AABB helpers
// ---------------------------------------------------------------

struct AABB { float minX, minY, maxX, maxY; };

inline float aabbCenterX(const AABB& b) { return 0.5f * (b.minX + b.maxX); }
inline float aabbCenterY(const AABB& b) { return 0.5f * (b.minY + b.maxY); }

inline AABB mergeAABB(const AABB& a, const AABB& b) {
    return {
        std::min(a.minX, b.minX), std::min(a.minY, b.minY),
        std::max(a.maxX, b.maxX), std::max(a.maxY, b.maxY)
    };
}

// Tight AABB over all vertices of a SpatialPoly.
inline AABB polyAABB(const SpatialPoly& p) {
    if (p.verts.empty()) return {0.f, 0.f, 0.f, 0.f};
    AABB b { p.verts[0].X, p.verts[0].Y, p.verts[0].X, p.verts[0].Y };
    for (size_t i = 1; i < p.verts.size(); ++i) {
        b.minX = std::min(b.minX, p.verts[i].X); b.minY = std::min(b.minY, p.verts[i].Y);
        b.maxX = std::max(b.maxX, p.verts[i].X); b.maxY = std::max(b.maxY, p.verts[i].Y);
    }
    return b;
}

// ---------------------------------------------------------------
// Distance helpers
// ---------------------------------------------------------------

// Min squared distance from point to AABB (0 if inside).
inline float pointToAABBDist2(Vec2 p, const AABB& b) {
    float dx = std::max({b.minX - p.X, 0.0f, p.X - b.maxX});
    float dy = std::max({b.minY - p.Y, 0.0f, p.Y - b.maxY});
    return dx*dx + dy*dy;
}

// Squared distance from point P to segment AB.
inline float pointToSegmentDist2(Vec2 p, Vec2 a, Vec2 b) {
    float dx = b.X - a.X, dy = b.Y - a.Y;
    float t  = dx * dx + dy * dy;
    if (t < 1e-12f) {
        float ex = p.X - a.X, ey = p.Y - a.Y;
        return ex*ex + ey*ey;
    }
    float s = std::clamp(((p.X-a.X)*dx + (p.Y-a.Y)*dy) / t, 0.0f, 1.0f);
    float cx = a.X + s*dx - p.X;
    float cy = a.Y + s*dy - p.Y;
    return cx*cx + cy*cy;
}

// Ray-casting point-in-polygon test (2D, closed polygon in verts).
inline bool pointInPolygon2D(Vec2 p, const std::vector<Vec2>& verts) {
    bool inside = false;
    const size_t n = verts.size();
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        float xi = verts[i].X, yi = verts[i].Y;
        float xj = verts[j].X, yj = verts[j].Y;
        if (((yi > p.Y) != (yj > p.Y)) &&
            (p.X < (xj - xi) * (p.Y - yi) / (yj - yi) + xi))
            inside = !inside;
    }
    return inside;
}

// Min squared distance from point P to a SpatialPoly (always a closed polygon):
// 0 if P is inside, else min dist to each edge verts[i]→verts[(i+1)%n].
inline float pointToPolyDist2(Vec2 p, const SpatialPoly& poly) {
    const auto& v = poly.verts;
    const size_t n = v.size();
    if (n == 0) return std::numeric_limits<float>::max();
    if (n == 1) {
        float dx = p.X - v[0].X, dy = p.Y - v[0].Y;
        return dx*dx + dy*dy;
    }

    if (pointInPolygon2D(p, v))
        return 0.f;

    float minD2 = std::numeric_limits<float>::max();
    for (size_t i = 0; i < n; ++i)
        minD2 = std::min(minD2, pointToSegmentDist2(p, v[i], v[(i + 1) % n]));
    return minD2;
}

// ---------------------------------------------------------------
// PolyRTree — STR-packed static R-tree over SpatialPoly leaves
// ---------------------------------------------------------------
class PolyRTree {
public:
    static constexpr int FANOUT = 16;

    struct Node {
        AABB    bounds;
        int32_t childStart;  // index into nodes[] (internal) or leaves[] (leaf)
        int32_t childCount;  // >0: internal;  <0: leaf (count = -childCount)
    };

    void clear() {
        leaves_.clear();
        nodes_.clear();
        leafAABB_.clear();
        root_ = -1;
    }

    // Build from a mixed list of segment and junction polygons.
    // O(n log n), called once after graph load.
    void build(std::vector<SpatialPoly> polys) {
        leaves_ = std::move(polys);
        nodes_.clear();
        root_ = -1;
        if (leaves_.empty()) return;

        const int nLeaves = (int)leaves_.size();

        // Per-leaf AABBs
        leafAABB_.resize(nLeaves);
        for (int i = 0; i < nLeaves; ++i)
            leafAABB_[i] = polyAABB(leaves_[i]);

        // STR sort: tile by X centroid, sort by Y within each tile
        std::vector<uint32_t> idx(nLeaves);
        std::iota(idx.begin(), idx.end(), 0);

        const int    nTiles   = (int)std::ceil(std::sqrt((double)nLeaves / FANOUT));
        const size_t tileSize = (size_t)std::ceil((double)nLeaves / nTiles);

        std::sort(idx.begin(), idx.end(), [&](uint32_t a, uint32_t b){
            return aabbCenterX(leafAABB_[a]) < aabbCenterX(leafAABB_[b]);
        });
        for (int t = 0; t < nTiles; ++t) {
            size_t lo = (size_t)t * tileSize;
            size_t hi = std::min(lo + tileSize, (size_t)nLeaves);
            std::sort(idx.begin() + lo, idx.begin() + hi, [&](uint32_t a, uint32_t b){
                return aabbCenterY(leafAABB_[a]) < aabbCenterY(leafAABB_[b]);
            });
        }

        std::vector<SpatialPoly> sortedLeaves(nLeaves);
        std::vector<AABB>        sortedAABB(nLeaves);
        for (int i = 0; i < nLeaves; ++i) {
            sortedLeaves[i] = std::move(leaves_[idx[i]]);
            sortedAABB[i]   = leafAABB_[idx[i]];
        }
        leaves_   = std::move(sortedLeaves);
        leafAABB_ = std::move(sortedAABB);

        // Level-by-level build — guarantees children are contiguous in nodes_[],
        // so childStart + i is always a valid index at every tree depth.
        for (int i = 0; i < nLeaves; i += FANOUT) {
            int  count = std::min(FANOUT, nLeaves - i);
            AABB b     = leafAABB_[i];
            for (int j = i + 1; j < i + count; ++j)
                b = mergeAABB(b, leafAABB_[j]);
            nodes_.push_back({ b, i, -count });
        }

        int levelStart = 0;
        int levelSize  = (int)nodes_.size();
        while (levelSize > 1) {
            int nextStart = (int)nodes_.size();
            for (int i = 0; i < levelSize; i += FANOUT) {
                int  count      = std::min(FANOUT, levelSize - i);
                int  childStart = levelStart + i;
                AABB b          = nodes_[childStart].bounds;
                for (int j = 1; j < count; ++j)
                    b = mergeAABB(b, nodes_[childStart + j].bounds);
                nodes_.push_back({ b, childStart, count });
            }
            levelStart = nextStart;
            levelSize  = (int)nodes_.size() - nextStart;
        }

        root_ = (int)nodes_.size() - 1;
    }

    // Nearest polygon to P.
    // Returns its id; writes type and squared distance into the optional out-params.
    // Returns INVALID_ID when the tree is empty.
    uint32_t nearest(Vec2 p,
                     PolyType* out_type    = nullptr,
                     float*    out_dist_sq = nullptr) const {
        if (root_ < 0) return INVALID_ID;
        float    bestDist2 = std::numeric_limits<float>::max();
        uint32_t bestIdx   = 0;
        searchNN(root_, p, bestDist2, bestIdx);
        if (out_type)    *out_type    = leaves_[bestIdx].type;
        if (out_dist_sq) *out_dist_sq = bestDist2;
        return leaves_[bestIdx].id;
    }

    // All polygons whose AABB intersects the query box.
    std::vector<PolyHit> rangeQuery(const AABB& query) const {
        std::vector<PolyHit> result;
        if (root_ >= 0) searchRange(root_, query, result);
        return result;
    }

private:
    std::vector<Node>       nodes_;
    std::vector<SpatialPoly> leaves_;
    std::vector<AABB>       leafAABB_;
    int                     root_ = -1;

    void searchNN(int nodeIdx, Vec2 p,
                  float& bestDist2, uint32_t& bestLeafIdx) const
    {
        const Node& n = nodes_[nodeIdx];

        if (n.childCount < 0) {
            // Leaf node — exact distance to each polygon
            const int count = -n.childCount;
            for (int i = 0; i < count; ++i) {
                const float d2 = pointToPolyDist2(p, leaves_[n.childStart + i]);
                if (d2 < bestDist2) {
                    bestDist2   = d2;
                    bestLeafIdx = (uint32_t)(n.childStart + i);
                }
            }
            return;
        }

        // Internal node — visit children sorted by min AABB dist to prune early
        struct Entry { float minD2; int idx; };
        std::array<Entry, FANOUT> order;
        const int nc = n.childCount;

        for (int i = 0; i < nc; ++i)
            order[i] = { pointToAABBDist2(p, nodes_[n.childStart + i].bounds),
                         n.childStart + i };

        std::sort(order.begin(), order.begin() + nc,
                  [](const Entry& a, const Entry& b){ return a.minD2 < b.minD2; });

        for (int i = 0; i < nc; ++i) {
            if (order[i].minD2 >= bestDist2) break;
            searchNN(order[i].idx, p, bestDist2, bestLeafIdx);
        }
    }

    void searchRange(int nodeIdx, const AABB& q,
                     std::vector<PolyHit>& result) const
    {
        const Node& n = nodes_[nodeIdx];
        if (n.bounds.maxX < q.minX || n.bounds.minX > q.maxX ||
            n.bounds.maxY < q.minY || n.bounds.minY > q.maxY) return;

        if (n.childCount < 0) {
            for (int i = 0; i < -n.childCount; ++i)
                result.push_back({ leaves_[n.childStart + i].id,
                                   leaves_[n.childStart + i].type });
            return;
        }

        for (int i = 0; i < n.childCount; ++i)
            searchRange(n.childStart + i, q, result);
    }
};
