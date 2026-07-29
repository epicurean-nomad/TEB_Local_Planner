#include "Serializer.h"
#include "AreaGraph.h"
#include "ParkingSpot.h"
#include <fstream>
#include <vector>
#include <cstring>

// ── Format constants ──────────────────────────────────────────────────────────

static constexpr uint8_t  MAGIC[4]        = {'R', 'G', 'A', 'v'};
static constexpr uint8_t  VERSION         = 0x0A;
static constexpr uint8_t  VERSION_MIN_LOAD = 0x09;
static constexpr uint8_t  RT_MAGIC[4]    = {'R', 'T', 'v', '1'};

// ── Buffer writer ─────────────────────────────────────────────────────────────

struct BufWriter {
    std::vector<uint8_t>& buf;

    template<typename T>
    void w(const T& v) {
        const auto* p = reinterpret_cast<const uint8_t*>(&v);
        buf.insert(buf.end(), p, p + sizeof(T));
    }

    void wbytes(const void* data, size_t n) {
        const auto* p = reinterpret_cast<const uint8_t*>(data);
        buf.insert(buf.end(), p, p + n);
    }

    void wpad(size_t n) { buf.insert(buf.end(), n, uint8_t{0}); }
    size_t size() const { return buf.size(); }
};

// ── Buffer reader ─────────────────────────────────────────────────────────────

struct BufReader {
    const uint8_t* data;
    size_t         len;
    size_t         pos = 0;
    bool           ok  = true;

    template<typename T>
    bool r(T& v) {
        if (!ok || pos + sizeof(T) > len) { ok = false; return false; }
        std::memcpy(&v, data + pos, sizeof(T));
        pos += sizeof(T);
        return true;
    }

    bool rbytes(void* dst, size_t n) {
        if (!ok || pos + n > len) { ok = false; return false; }
        std::memcpy(dst, data + pos, n);
        pos += n;
        return true;
    }

    bool skip(size_t n) {
        if (!ok || pos + n > len) { ok = false; return false; }
        pos += n;
        return true;
    }
};

// ── CRC32 (IEEE 802.3 / zlib-compatible) ──────────────────────────────────────
// Must match serializer_util::crc32_compute used by the editor's AreaSerializer
// byte-for-byte, so files written here verify cleanly when opened in the editor
// (and vice versa).

static const uint32_t* crc32_table() {
    static uint32_t table[256];
    static bool ready = false;
    if (!ready) {
        for (uint32_t i = 0; i < 256; ++i) {
            uint32_t c = i;
            for (int j = 0; j < 8; ++j)
                c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
            table[i] = c;
        }
        ready = true;
    }
    return table;
}

static uint32_t crc32_compute(const uint8_t* data, size_t len) {
    const uint32_t* tbl = crc32_table();
    uint32_t crc = 0xFFFFFFFFu;
    while (len--) crc = (crc >> 8) ^ tbl[(crc ^ *data++) & 0xFFu];
    return crc ^ 0xFFFFFFFFu;
}

// ── Area-graph write (shared between save and saveAvRuntime) ──────────────────
//
// Writes header + identity block + all area sections + CRC32 footer into buf.
// Byte layout matches AreaSerializer::saveToBuffer exactly (magic, version,
// pad, 136-byte identity block, sections, CRC32 over [5 .. end-4)) so the
// resulting area-graph portion of the file loads cleanly in the editor.
// Must be called from a Serializer method so friend access to AreaGraph is active.

void Serializer::writeAreaGraph(BufWriter& w, const AreaGraph& g) {
    // ── Header ────────────────────────────────────────────────────────────────
    w.wbytes(MAGIC, 4);
    w.w(VERSION);
    w.wpad(3);

    // ── Identity ──────────────────────────────────────────────────────────────
    w.wbytes(g.area_name, 64);
    w.wbytes(g.area_uuid, 37);
    w.wpad(3);
    w.w(g.origin.lat);
    w.w(g.origin.lon);
    w.w(g.origin.alt);
    w.w(g.origin.cos_lat);

    // ── Nodes ─────────────────────────────────────────────────────────────────
    w.w(static_cast<uint32_t>(g.nodes_.size()));
    for (const auto& n : g.nodes_) {
        auto rec = n.toRecord();
        w.wbytes(&rec, sizeof(rec));
    }

    // ── Segments ──────────────────────────────────────────────────────────────
    // forward_path_id / reverse_path_id are NOT written — rebuilt by rebuildDerivedState().
    w.w(static_cast<uint32_t>(g.segments_.size()));
    for (const auto& seg : g.segments_) {
        w.w(seg.id);
        w.w(seg.node_a_id);
        w.w(seg.node_b_id);
        w.w(static_cast<uint32_t>(seg.geometry_pts.size()));
        for (const auto& gp : seg.geometry_pts) {
            w.w(gp.x); w.w(gp.y); w.w(gp.z);
            w.w(gp.angle_radians); w.w(gp.lbound_offset); w.w(gp.rbound_offset);
            w.w(gp.id); w.wpad(3);
        }
        w.w(seg.arc_length_m);
        w.w(seg.lane_count);
        w.w(seg.road_class);
        w.w(seg.surface_type);
        w.wpad(2);
    }

    // ── Paths ─────────────────────────────────────────────────────────────────
    w.w(static_cast<uint32_t>(g.paths_.size()));
    for (const auto& [pid, p] : g.paths_) {
        Path::SerialHeader hdr{};
        hdr.id                  = p.id;
        hdr.segment_id          = p.segment_id;
        hdr.from_node_id        = p.from_node_id;
        hdr.to_node_id          = p.to_node_id;
        const RoadSegment* seg  = g.getSegment(p.segment_id);
        hdr.lane_count          = seg ? seg->lane_count : 2;
        hdr.speed_limit_kmh     = p.speed_limit_kmh;
        hdr.lane_change_allowed = p.lane_change_allowed ? uint8_t{1} : uint8_t{0};
        hdr.pad                 = 0;
        hdr.dir_reg_count       = 0;
        w.wbytes(&hdr, sizeof(hdr));
    }

    // ── Junctions ─────────────────────────────────────────────────────────────
    // cached_polygon and polygon_dirty are NOT written — rebuilt on load.
    w.w(static_cast<uint32_t>(g.junctions_.size()));
    for (const auto& j : g.junctions_) {
        Junction::SerialHeader hdr{};
        hdr.id                    = j.id;
        hdr.type                  = static_cast<uint8_t>(j.type);
        hdr.pad[0] = hdr.pad[1] = hdr.pad[2] = 0;
        hdr.boundary_node_count   = static_cast<uint32_t>(j.boundary_node_ids.size());
        hdr.connection_edge_count = static_cast<uint32_t>(j.connection_edges.size());
        w.wbytes(&hdr, sizeof(hdr));
        for (uint32_t nid : j.boundary_node_ids) w.w(nid);
        for (const auto& ce : j.connection_edges) {
            w.w(ce.id);
            w.w(ce.from_node_id);
            w.w(ce.to_node_id);
            w.w(ce.required_entry_lane);
            w.w(ce.resulting_exit_lane);
            w.wpad(2);
            w.w(ce.arc_length_m);
        }
        w.w(static_cast<uint32_t>(j.boundary_pts.size()));
        for (const auto& pt : j.boundary_pts) { w.w(pt.X); w.w(pt.Y); }
    }

    // ── Adjacency ─────────────────────────────────────────────────────────────
    // Reverse adjacency is NOT written — derived by rebuildDerivedState().
    uint32_t adj_total = 0;
    for (const auto& [from, tos] : g.adjacency_)
        adj_total += static_cast<uint32_t>(tos.size());
    w.w(adj_total);
    for (const auto& [from, tos] : g.adjacency_) {
        for (const auto& [to, ref] : tos) {
            w.w(from);
            w.w(to);
            w.w(ref.edge_id);
            w.w((ref.kind == EdgeKind::PATH) ? uint8_t{0} : uint8_t{1});
            w.wpad(3);
        }
    }

    // ── Parking spots (v0x0A+) ────────────────────────────────────────────────
    w.w(static_cast<uint32_t>(g.parking_spots_.size()));
    for (const auto& spot : g.parking_spots_) {
        auto rec = spot.toRecord();
        w.wbytes(&rec, sizeof(rec));
    }

    // ── CRC32 footer ──────────────────────────────────────────────────────────
    // Covers bytes [5 .. end-1] (everything after magic+version, before this
    // CRC word) — matches AreaSerializer::saveToBuffer's footer exactly.
    const uint32_t checksum = crc32_compute(w.buf.data() + 5, w.buf.size() - 5);
    w.w(checksum);
}

// ── Runtime-section write helper ──────────────────────────────────────────────

void Serializer::writeRuntimeSections(BufWriter& w, const AreaGraph& g) {
    // ── RT_MAGIC ──────────────────────────────────────────────────────────────
    w.wbytes(RT_MAGIC, 4);

    // ── Sampled paths ─────────────────────────────────────────────────────────
    // Format: [count][from][to][dir u8][pt_count][x f][y f][z f]...
    const auto& paths = g.sampled_paths_;
    w.w(static_cast<uint32_t>(paths.size()));
    for (const auto& [key, pts] : paths) {
        w.w(key.a);
        w.w(key.b);
        w.w(static_cast<uint8_t>(key.dir));
        w.w(static_cast<uint32_t>(pts.size()));
        for (const auto& p : pts) {
            w.w(p.x); w.w(p.y); w.w(p.z);
        }
    }

    // ── APSP table ────────────────────────────────────────────────────────────
    // Format: [src_count][src][dst_count][dst][cost f][node_count][node_id]...
    const auto& apsp = g.apsp_table_;
    w.w(static_cast<uint32_t>(apsp.size()));
    for (const auto& [src, path_map] : apsp) {
        w.w(src);
        w.w(static_cast<uint32_t>(path_map.size()));
        for (const auto& [dst, sp] : path_map) {
            w.w(dst);
            w.w(sp.cost);
            w.w(static_cast<uint32_t>(sp.nodes.size()));
            for (uint32_t nid : sp.nodes) w.w(nid);
        }
    }
}

// ── Runtime-section read helper ───────────────────────────────────────────────

bool Serializer::readRuntimeSections(BufReader& r, AreaGraph& g, std::string& error) {
    // ── Sampled paths ─────────────────────────────────────────────────────────
    uint32_t path_count = 0;
    if (!r.r(path_count)) { error = "Truncated sampled_path count"; return false; }
    g.sampled_paths_.clear();
    g.sampled_paths_.reserve(path_count);
    for (uint32_t i = 0; i < path_count; ++i) {
        uint32_t a = 0, b = 0, pt_count = 0;
        uint8_t  dir_byte = 0;
        if (!r.r(a) || !r.r(b) || !r.r(dir_byte) || !r.r(pt_count)) {
            error = "Truncated sampled_path entry " + std::to_string(i); return false;
        }
        std::vector<Point3d> pts;
        pts.reserve(pt_count);
        for (uint32_t j = 0; j < pt_count; ++j) {
            float x = 0, y = 0, z = 0;
            if (!r.r(x) || !r.r(y) || !r.r(z)) {
                error = "Truncated sampled_path point " + std::to_string(j); return false;
            }
            pts.emplace_back(x, y, z);
        }
        g.sampled_paths_[{a, b, static_cast<Dir>(dir_byte)}] = std::move(pts);
    }

    // ── APSP table ────────────────────────────────────────────────────────────
    uint32_t src_count = 0;
    if (!r.r(src_count)) { error = "Truncated apsp src_count"; return false; }
    g.apsp_table_.clear();
    g.apsp_table_.reserve(src_count);
    for (uint32_t i = 0; i < src_count; ++i) {
        uint32_t src = 0, dst_count = 0;
        if (!r.r(src) || !r.r(dst_count)) {
            error = "Truncated apsp src entry " + std::to_string(i); return false;
        }
        PathMap& pm = g.apsp_table_[src];
        pm.reserve(dst_count);
        for (uint32_t j = 0; j < dst_count; ++j) {
            uint32_t dst = 0, node_count = 0;
            float    cost = 0;
            if (!r.r(dst) || !r.r(cost) || !r.r(node_count)) {
                error = "Truncated apsp dst entry " + std::to_string(j); return false;
            }
            ShortestPath& sp = pm[dst];
            sp.cost = cost;
            sp.nodes.reserve(node_count);
            for (uint32_t k = 0; k < node_count; ++k) {
                uint32_t nid = 0;
                if (!r.r(nid)) {
                    error = "Truncated apsp node_id"; return false;
                }
                sp.nodes.push_back(nid);
            }
        }
    }
    return true;
}

// ── Load ──────────────────────────────────────────────────────────────────────
//
// Auto-detects file format:
//   - "Minimal" (no identity block): node_count at offset 8
//   - "Editor"  (with identity block + CRC): 136-byte identity at offset 8,
//                                            node_count at offset 144,
//                                            CRC32 trailer (last 4 bytes)
//
// Also detects an optional runtime section at the end of the file
// (RT_MAGIC + sampled paths + APSP + sentinel) when present.

bool Serializer::loadArea(AreaGraph& g, const std::string& filepath, std::string& error) {
    std::ifstream is(filepath, std::ios::binary | std::ios::ate);
    if (!is) { error = "Cannot open: " + filepath; return false; }
    const size_t fsize = static_cast<size_t>(is.tellg());
    if (fsize < 12) { error = "File too small"; return false; }
    is.seekg(0);
    std::vector<uint8_t> buf(fsize);
    if (!is.read(reinterpret_cast<char*>(buf.data()),
                 static_cast<std::streamsize>(fsize))) {
        error = "Read failed: " + filepath; return false;
    }

    BufReader r{buf.data(), fsize, 0, true};

    // ── File header ──────────────────────────────────────────────────────────
    uint8_t magic[4] = {};
    if (!r.rbytes(magic, 4) || std::memcmp(magic, MAGIC, 4) != 0) {
        error = "Invalid magic — not an RGAv file"; return false;
    }
    uint8_t version = 0;
    if (!r.r(version) || version < VERSION_MIN_LOAD || version > VERSION) {
        error = "Unsupported version " + std::to_string(version) +
                " (supported range " + std::to_string(VERSION_MIN_LOAD) +
                "–" + std::to_string(VERSION) + ")";
        return false;
    }
    r.skip(3);  // padding

    // ── Detect optional runtime section ──────────────────────────────────────
    // Files with a runtime section end with:
    //   ... [area sections] [RT_MAGIC 4] [sampled paths] [apsp] [rt_offset u32]
    // Last 4 bytes store the file offset where RT_MAGIC begins.
    uint32_t rt_magic_pos = 0;
    if (fsize >= 9 + 4 + 4) {
        uint32_t sentinel = 0;
        std::memcpy(&sentinel, buf.data() + fsize - 4, 4);
        if (sentinel + 4 <= fsize - 4 &&
            std::memcmp(buf.data() + sentinel, RT_MAGIC, 4) == 0) {
            rt_magic_pos = sentinel;
        }
    }

    // ── Detect identity block (editor files) vs minimal layout ───────────────
    // Read what node_count would be at each candidate offset and pick the
    // plausible one.
    uint32_t count_at_8 = 0, count_at_144 = 0;
    std::memcpy(&count_at_8, buf.data() + 8, 4);
    if (fsize >= 148) std::memcpy(&count_at_144, buf.data() + 144, 4);

    const bool plausible_A = (count_at_8   < 1000000);  // minimal layout
    const bool plausible_B = (count_at_144 > 0 && count_at_144 < 1000000);

    bool has_identity_block;
    if (plausible_B && (count_at_8 == 0 || count_at_8 >= 1000000)) {
        // Offset 8 is zero/garbage, offset 144 looks sane → identity present.
        has_identity_block = true;
    } else if (plausible_A) {
        // Offset 8 looks sane → minimal layout.
        has_identity_block = false;
    } else {
        error = "Unrecognized format (counts at 8 and 144 both invalid: " +
                std::to_string(count_at_8) + ", " + std::to_string(count_at_144) + ")";
        return false;
    }

    if (has_identity_block) {
        // ── CRC32 verification (editor layout) ───────────────────────────────
        // The area-graph section ends at rt_magic_pos (if a runtime section
        // follows) or at fsize otherwise. Its last 4 bytes are the CRC32,
        // covering bytes [5 .. area_end-4) — matches AreaSerializer's footer.
        const size_t area_end = (rt_magic_pos > 0) ? rt_magic_pos : fsize;
        if (area_end < 9) { error = "Area section too small for CRC32 footer"; return false; }
        uint32_t stored_crc = 0;
        std::memcpy(&stored_crc, buf.data() + area_end - 4, 4);
        const uint32_t computed_crc = crc32_compute(buf.data() + 5, area_end - 4 - 5);
        if (computed_crc != stored_crc) {
            error = "CRC32 mismatch — file may be corrupt"; return false;
        }
    }

    g.clear();

    if (has_identity_block) {
        if (!r.rbytes(g.area_name, 64)) { error = "Truncated area_name"; return false; }
        if (!r.rbytes(g.area_uuid, 37)) { error = "Truncated area_uuid"; return false; }
        r.skip(3);
        double lat = 0, lon = 0, alt = 0, cos_lat = 1;
        if (!r.r(lat) || !r.r(lon) || !r.r(alt) || !r.r(cos_lat)) {
            error = "Truncated origin fields"; return false;
        }
        g.origin         = GeoOrigin(lat, lon, alt);
        g.origin.cos_lat = cos_lat;
    } else {
        std::memset(g.area_name, 0, sizeof(g.area_name));
        std::memset(g.area_uuid, 0, sizeof(g.area_uuid));
        g.origin = GeoOrigin();
    }

    // ── Nodes ────────────────────────────────────────────────────────────────
    // v09 record = 40 bytes (no parking_spot_id); v0A = 44 bytes.
    static constexpr size_t NODE_REC_V9 =
        offsetof(RoadNode::SerialRecord, parking_spot_id);

    uint32_t node_count = 0;
    if (!r.r(node_count)) { error = "Truncated node_count"; return false; }
    g.nodes_.reserve(node_count);
    for (uint32_t i = 0; i < node_count; ++i) {
        RoadNode::SerialRecord rec{};
        if (version >= 0x0A) {
            if (!r.rbytes(&rec, sizeof(rec))) {
                error = "Truncated node record " + std::to_string(i); return false;
            }
        } else {
            if (!r.rbytes(&rec, NODE_REC_V9)) {
                error = "Truncated node record " + std::to_string(i); return false;
            }
            rec.parking_spot_id = INVALID_ID;
        }
        g.nodes_.push_back(RoadNode::fromRecord(rec));
    }

    // ── Segments ─────────────────────────────────────────────────────────────
    uint32_t seg_count = 0;
    if (!r.r(seg_count)) { error = "Truncated segment_count"; return false; }
    g.segments_.reserve(seg_count);
    for (uint32_t i = 0; i < seg_count; ++i) {
        RoadSegment seg;
        uint32_t gp_count = 0;
        if (!r.r(seg.id) || !r.r(seg.node_a_id) || !r.r(seg.node_b_id) || !r.r(gp_count)) {
            error = "Truncated segment header " + std::to_string(i); return false;
        }
        seg.geometry_pts.reserve(gp_count);
        for (uint32_t j = 0; j < gp_count; ++j) {
            GeometryPoint gp;
            if (!r.r(gp.x) || !r.r(gp.y) || !r.r(gp.z) ||
                !r.r(gp.angle_radians) || !r.r(gp.lbound_offset) || !r.r(gp.rbound_offset) ||
                !r.r(gp.id) || !r.skip(3)) {
                error = "Truncated geometry pt, segment " + std::to_string(i); return false;
            }
            gp.recomputeBounds();
            seg.geometry_pts.push_back(gp);
        }
        if (!r.r(seg.arc_length_m) || !r.r(seg.lane_count) ||
            !r.r(seg.road_class) || !r.r(seg.surface_type)) {
            error = "Truncated segment fields " + std::to_string(i); return false;
        }
        r.skip(2);  // pad
        g.segments_.push_back(std::move(seg));
    }

    // ── Paths ────────────────────────────────────────────────────────────────
    uint32_t path_count = 0;
    if (!r.r(path_count)) { error = "Truncated path_count"; return false; }
    for (uint32_t i = 0; i < path_count; ++i) {
        Path::SerialHeader hdr{};
        if (!r.rbytes(&hdr, sizeof(hdr))) {
            error = "Truncated path header " + std::to_string(i); return false;
        }
        Path p;
        p.id                  = hdr.id;
        p.segment_id          = hdr.segment_id;
        p.from_node_id        = hdr.from_node_id;
        p.to_node_id          = hdr.to_node_id;
        p.speed_limit_kmh     = hdr.speed_limit_kmh;
        p.lane_change_allowed = (hdr.lane_change_allowed != 0);
        p.lane_count          = hdr.lane_count;
        g.paths_[p.id] = std::move(p);
    }

    // ── Junctions ────────────────────────────────────────────────────────────
    uint32_t junc_count = 0;
    if (!r.r(junc_count)) { error = "Truncated junction_count"; return false; }
    g.junctions_.reserve(junc_count);
    for (uint32_t i = 0; i < junc_count; ++i) {
        Junction::SerialHeader hdr{};
        if (!r.rbytes(&hdr, sizeof(hdr))) {
            error = "Truncated junction header " + std::to_string(i); return false;
        }
        Junction j;
        j.id            = hdr.id;
        j.type          = static_cast<JunctionType>(hdr.type);
        j.polygon_dirty = true;
        j.boundary_node_ids.reserve(hdr.boundary_node_count);
        for (uint32_t k = 0; k < hdr.boundary_node_count; ++k) {
            uint32_t nid = 0;
            if (!r.r(nid)) { error = "Truncated boundary node id"; return false; }
            j.boundary_node_ids.push_back(nid);
        }
        j.connection_edges.reserve(hdr.connection_edge_count);
        for (uint32_t k = 0; k < hdr.connection_edge_count; ++k) {
            ConnectionEdge ce;
            uint8_t ce_pad[2] = {};
            if (!r.r(ce.id) || !r.r(ce.from_node_id) || !r.r(ce.to_node_id) ||
                !r.r(ce.required_entry_lane) || !r.r(ce.resulting_exit_lane) ||
                !r.rbytes(ce_pad, 2) || !r.r(ce.arc_length_m)) {
                error = "Truncated connection edge " + std::to_string(k); return false;
            }
            j.connection_edges.push_back(std::move(ce));
        }
        if (version >= 0x09) {
            uint32_t bpts_count = 0;
            if (!r.r(bpts_count)) { error = "Truncated boundary_pts_count"; return false; }
            j.boundary_pts.reserve(bpts_count);
            for (uint32_t k = 0; k < bpts_count; ++k) {
                float x = 0, y = 0;
                if (!r.r(x) || !r.r(y)) { error = "Truncated boundary_pt " + std::to_string(k); return false; }
                j.boundary_pts.emplace_back(x, y);
            }
        }
        g.junctions_.push_back(std::move(j));
    }

    // ── Adjacency ────────────────────────────────────────────────────────────
    uint32_t adj_count = 0;
    if (!r.r(adj_count)) { error = "Truncated adjacency_count"; return false; }
    for (uint32_t i = 0; i < adj_count; ++i) {
        uint32_t from = 0, to = 0, eid = 0;
        uint8_t  kind_byte = 0, adj_pad[3] = {};
        if (!r.r(from) || !r.r(to) || !r.r(eid) ||
            !r.r(kind_byte) || !r.rbytes(adj_pad, 3)) {
            error = "Truncated adjacency entry " + std::to_string(i); return false;
        }
        EdgeRef ref;
        ref.edge_id = eid;
        ref.kind    = (kind_byte == 0) ? EdgeKind::PATH : EdgeKind::JUNCTION;
        g.adjacency_[from][to] = ref;
    }

    // ── Parking spots (v0x0A+) ───────────────────────────────────────────────
    if (version >= 0x0A) {
        uint32_t spot_count = 0;
        if (!r.r(spot_count)) { error = "Truncated parking_spot_count"; return false; }
        g.parking_spots_.reserve(spot_count);
        for (uint32_t i = 0; i < spot_count; ++i) {
            ParkingSpot::SerialRecord rec{};
            if (!r.rbytes(&rec, sizeof(rec))) {
                error = "Truncated parking_spot record " + std::to_string(i); return false;
            }
            g.parking_spots_.push_back(ParkingSpot::fromRecord(rec));
        }
    }

    g.rebuildDerivedState();

    // ── Runtime section (if present) ─────────────────────────────────────────
    if (rt_magic_pos > 0) {
        BufReader rr{buf.data(), fsize - 4, rt_magic_pos + 4, true};
        if (!readRuntimeSections(rr, g, error)) return false;
    }

    return true;
}


// bool Serializer::loadArea(AreaGraph& g, const std::string& filepath, std::string& error) {
//     std::ifstream is(filepath, std::ios::binary | std::ios::ate);
//     if (!is) { error = "Cannot open: " + filepath; return false; }
//     const size_t fsize = static_cast<size_t>(is.tellg());
//     if (fsize < 9) { error = "File too small"; return false; }
//     is.seekg(0);
//     std::vector<uint8_t> buf(fsize);
//     if (!is.read(reinterpret_cast<char*>(buf.data()),
//                  static_cast<std::streamsize>(fsize))) {
//         error = "Read failed: " + filepath; return false;
//     }

//     BufReader r{buf.data(), fsize, 0, true};

//     // ── Header ────────────────────────────────────────────────────────────────
//     uint8_t magic[4] = {};
//     if (!r.rbytes(magic, 4) || std::memcmp(magic, MAGIC, 4) != 0) {
//         error = "Invalid magic — not an RGAv file"; return false;
//     }
//     uint8_t version = 0;
//     if (!r.r(version) || version != VERSION) {
//         error = "Unsupported version " + std::to_string(version) +
//                 " (expected " + std::to_string(VERSION) + ")";
//         return false;
//     }
//     r.skip(3);  // padding

//     // ── Detect optional runtime section ───────────────────────────────────────
//     //
//     // Files with a runtime section end with:
//     //   ... [area sections] [RT_MAGIC 4] [sampled paths] [apsp] [rt_offset: uint32_t]
//     //
//     // The last 4 bytes store the file offset where RT_MAGIC starts.
//     //
//     uint32_t rt_magic_pos = 0;  // 0 = no runtime section

//     if (fsize >= 9 + 4 + 4) {  // header(9) + RT_MAGIC(4) + sentinel(4)
//         uint32_t sentinel = 0;
//         std::memcpy(&sentinel, buf.data() + fsize - 4, 4);
//         if (sentinel + 4 <= fsize - 4 &&
//             std::memcmp(buf.data() + sentinel, RT_MAGIC, 4) == 0) {
//             rt_magic_pos = sentinel;
//         }
//     }

//     g.clear();

//     // ── Nodes ─────────────────────────────────────────────────────────────────
//     uint32_t node_count = 0;
//     if (!r.r(node_count)) { error = "Truncated node_count"; return false; }
//     g.nodes_.reserve(node_count);
//     for (uint32_t i = 0; i < node_count; ++i) {
//         RoadNode::SerialRecord rec{};
//         if (!r.rbytes(&rec, sizeof(rec))) {
//             error = "Truncated node record " + std::to_string(i); return false;
//         }
//         g.nodes_.push_back(RoadNode::fromRecord(rec));
//     }

//     // ── Segments ──────────────────────────────────────────────────────────────
//     uint32_t seg_count = 0;
//     if (!r.r(seg_count)) { error = "Truncated segment_count"; return false; }
//     g.segments_.reserve(seg_count);
//     for (uint32_t i = 0; i < seg_count; ++i) {
//         RoadSegment seg;
//         uint32_t gp_count = 0;
//         if (!r.r(seg.id) || !r.r(seg.node_a_id) || !r.r(seg.node_b_id) || !r.r(gp_count)) {
//             error = "Truncated segment header " + std::to_string(i); return false;
//         }
//         seg.geometry_pts.reserve(gp_count);
//         for (uint32_t j = 0; j < gp_count; ++j) {
//             GeometryPoint gp;
//             if (!r.r(gp.x) || !r.r(gp.y) || !r.r(gp.z)) {
//                 error = "Truncated geometry pts, segment " + std::to_string(i); return false;
//             }
//             if (version >= 0x04) {
//                 if (!r.r(gp.angle_radians) || !r.r(gp.lbound_offset) || !r.r(gp.rbound_offset)) {
//                     error = "Truncated geometry pt fields, segment " + std::to_string(i); return false;
//                 }
//             }
//             if (version >= 0x05) {
//                 if (!r.r(gp.id) || !r.skip(3)) {
//                     error = "Truncated geometry pt id, segment " + std::to_string(i); return false;
//                 }
//             }
//             seg.geometry_pts.push_back(gp);
//         }
//         if (!r.r(seg.arc_length_m) || !r.r(seg.lane_count) || !r.r(seg.road_class) || !r.r(seg.surface_type)) {
//             error = "Truncated segment fields " + std::to_string(i); return false;
//         }
//         r.skip(2);  // pad
//         g.segments_.push_back(std::move(seg));
//     }

//     // ── Paths ─────────────────────────────────────────────────────────────────
//     uint32_t path_count = 0;
//     if (!r.r(path_count)) { error = "Truncated path_count"; return false; }
//     for (uint32_t i = 0; i < path_count; ++i) {
//         Path::SerialHeader hdr{};
//         if (!r.rbytes(&hdr, sizeof(hdr))) {
//             error = "Truncated path header " + std::to_string(i); return false;
//         }
//         Path p;
//         p.id                  = hdr.id;
//         p.segment_id          = hdr.segment_id;
//         p.from_node_id        = hdr.from_node_id;
//         p.to_node_id          = hdr.to_node_id;
//         p.speed_limit_kmh     = hdr.speed_limit_kmh;
//         p.lane_change_allowed = (hdr.lane_change_allowed != 0);
//         p.lane_count          = hdr.lane_count;
//         g.paths_[p.id] = std::move(p);
//     }

//     // ── Junctions ─────────────────────────────────────────────────────────────
//     uint32_t junc_count = 0;
//     if (!r.r(junc_count)) { error = "Truncated junction_count"; return false; }
//     g.junctions_.reserve(junc_count);
//     for (uint32_t i = 0; i < junc_count; ++i) {
//         Junction::SerialHeader hdr{};
//         if (!r.rbytes(&hdr, sizeof(hdr))) {
//             error = "Truncated junction header " + std::to_string(i); return false;
//         }
//         Junction j;
//         j.id            = hdr.id;
//         j.type          = static_cast<JunctionType>(hdr.type);
//         j.polygon_dirty = true;
//         j.boundary_node_ids.reserve(hdr.boundary_node_count);
//         for (uint32_t k = 0; k < hdr.boundary_node_count; ++k) {
//             uint32_t nid = 0;
//             if (!r.r(nid)) { error = "Truncated boundary node id"; return false; }
//             j.boundary_node_ids.push_back(nid);
//         }
//         j.connection_edges.reserve(hdr.connection_edge_count);
//         for (uint32_t k = 0; k < hdr.connection_edge_count; ++k) {
//             ConnectionEdge ce;
//             uint8_t ce_pad[2] = {};
//             if (!r.r(ce.id) || !r.r(ce.from_node_id) || !r.r(ce.to_node_id) ||
//                 !r.r(ce.required_entry_lane) || !r.r(ce.resulting_exit_lane) ||
//                 !r.rbytes(ce_pad, 2)) {
//                 error = "Truncated connection edge " + std::to_string(k); return false;
//             }
//             if (version >= 0x05) {
//                 if (!r.r(ce.arc_length_m)) {
//                     error = "Truncated CE arc_length_m " + std::to_string(k); return false;
//                 }
//             }
//             j.connection_edges.push_back(std::move(ce));
//         }
//         g.junctions_.push_back(std::move(j));
//     }

//     // ── Adjacency ─────────────────────────────────────────────────────────────
//     uint32_t adj_count = 0;
//     if (!r.r(adj_count)) { error = "Truncated adjacency_count"; return false; }
//     for (uint32_t i = 0; i < adj_count; ++i) {
//         uint32_t from = 0, to = 0, eid = 0;
//         uint8_t  kind_byte = 0, adj_pad[3] = {};
//         if (!r.r(from) || !r.r(to) || !r.r(eid) || !r.r(kind_byte) || !r.rbytes(adj_pad, 3)) {
//             error = "Truncated adjacency entry " + std::to_string(i); return false;
//         }
//         EdgeRef ref;
//         ref.edge_id = eid;
//         ref.kind    = (kind_byte == 0) ? EdgeKind::PATH : EdgeKind::JUNCTION;
//         g.adjacency_[from][to] = ref;
//     }

//     g.rebuildDerivedState();

//     // ── Runtime section (if present) ─────────────────────────────────────────
//     if (rt_magic_pos > 0) {
//         // Position reader past RT_MAGIC, stop before the 4-byte sentinel.
//         BufReader rr{buf.data(), fsize - 4, rt_magic_pos + 4, true};
//         if (!readRuntimeSections(rr, g, error)) return false;
//     }

//     return true;
// }

// ── Save with runtime ─────────────────────────────────────────────────────────
//
// File layout:
//   [area graph: header + sections]
//   [RT_MAGIC 4: 'RTv1']
//   [sampled paths section]
//   [APSP table section]
//   [rt_offset: uint32_t]  ← sentinel — offset of RT_MAGIC, lets loadArea
//                            locate the runtime section in O(1).

bool Serializer::saveAvRuntime(const AreaGraph& g,
                               const std::string& filepath,
                               std::string& error) {
    std::vector<uint8_t> buf;
    buf.reserve(256 * 1024);
    BufWriter w{buf};

    writeAreaGraph(w, g);

    // Record where RT_MAGIC will land (= current buf size = just after area CRC).
    const uint32_t rt_offset = static_cast<uint32_t>(buf.size());

    writeRuntimeSections(w, g);

    // Sentinel: file offset of RT_MAGIC, lets loadArea find the CRC in O(1).
    w.w(rt_offset);

    std::ofstream os(filepath, std::ios::binary | std::ios::trunc);
    if (!os) { error = "Cannot open for writing: " + filepath; return false; }
    os.write(reinterpret_cast<const char*>(buf.data()),
             static_cast<std::streamsize>(buf.size()));
    if (!os) { error = "Write failed: " + filepath; return false; }
    return true;
}

// ── Load runtime only (area section already loaded) ───────────────────────────
//
// Reads a file produced by saveAvRuntime and extracts only the runtime section
// (sampled paths + APSP table) into g.  Useful when the graph is already loaded
// and only the derived data needs to be refreshed.

bool Serializer::loadAvRuntime(AreaGraph& g,
                               const std::string& filepath,
                               std::string& error) {
    std::ifstream is(filepath, std::ios::binary | std::ios::ate);
    if (!is) { error = "Cannot open: " + filepath; return false; }
    const size_t fsize = static_cast<size_t>(is.tellg());
    if (fsize < 9 + 4 + 4 + 4) { error = "File too small to contain runtime section"; return false; }
    is.seekg(0);
    std::vector<uint8_t> buf(fsize);
    if (!is.read(reinterpret_cast<char*>(buf.data()),
                 static_cast<std::streamsize>(fsize))) {
        error = "Read failed: " + filepath; return false;
    }

    // Read the sentinel.
    uint32_t rt_offset = 0;
    std::memcpy(&rt_offset, buf.data() + fsize - 4, 4);
    if (rt_offset + 4 > fsize - 4 ||
        std::memcmp(buf.data() + rt_offset, RT_MAGIC, 4) != 0) {
        error = "No runtime section found — file was not saved by saveAvRuntime"; return false;
    }

    BufReader r{buf.data(), fsize - 4, rt_offset + 4, true};
    return readRuntimeSections(r, g, error);
}
