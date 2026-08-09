// test_local_planner_viz.cpp
//
// Standalone, real-time visual test harness for TebLocalPlanner -- bypasses
// LCM and PlannerNode entirely. Loads the same Area file the real planner
// would, reads a reference path CSV (as produced by the global planner /
// stitched_path.csv), simulates a vehicle moving along that path, and lets
// you click to drop obstacles in front of it to watch the local planner
// react in real time.
//
// Build (see the CMakeLists snippet in chat for the actual target):
//   requires OpenCV (find_package(OpenCV REQUIRED)) in addition to the
//   g2o/Eigen deps the local planner itself needs.
//
// Controls:
//   Left click   -- drop an obstacle at that point
//   Right click  -- remove the nearest obstacle
//   c            -- clear all obstacles
//   space        -- pause / resume vehicle motion
//   r            -- reset vehicle to the start of the path
//   v            -- toggle follow (zoomed, chases vehicle) / overview (whole map)
//   +/-          -- speed up / slow down vehicle
//   [ / ]        -- shrink / grow the follow-camera radius
//   q / ESC      -- quit
//
// Design notes:
//   - Drivable-region rendering draws Junction::cached_polygon and
//     RoadSegment::cached_polygon directly -- confirmed (via
//     ValidityCheckLocal.h) to be the exact same polygons
//     isInsideZone()/isVehicleFeasible() test against. An earlier version
//     of this file reconstructed an approximate boundary from
//     bound_angle_radians + half_width_left_m/right_m, which turned out to
//     be unrelated to actual collision geometry -- replaced entirely, no
//     more guessing.
//   - CRITICAL: ValidityCheckLocal's local_polygons_ starts empty and is
//     only populated by setLocalZone()/setRouteZone(). Without calling one
//     of these, isVehicleFeasible() silently rejects every pose,
//     unconditionally, regardless of obstacles -- this harness calls
//     setRouteZone() once at startup with every node in the graph (see
//     main()), since it bypasses the real pipeline's getKinodynamicPath()
//     (where PlannerNode normally calls this per-route).
//   - The vehicle advances along whichever plan is CURRENTLY live each
//     frame (the just-replanned path if the planner produced one, the
//     original route otherwise), re-anchored fresh every frame via a
//     nearest-point search -- not a full tracking controller (no
//     steering/lateral-error correction), but a receding-horizon
//     re-plan-and-advance loop: a successful detour actually gets driven,
//     and the next frame's plan is anchored wherever that left the
//     vehicle. Every frame's replan still starts from the pristine
//     original route (mirrors onObstacles()'s `pathCopy = globalPath_`
//     rather than compounding detours frame over frame) -- only the
//     vehicle's own position/heading carries over between frames.
//   - Vehicle icon size (width/length) is a cosmetic guess, not derived
//     from setVehicle()'s actual parameter semantics (I don't know their
//     exact meaning) -- purely visual, doesn't affect what the planner
//     actually checks via ValidityCheckLocal.

#include <opencv2/opencv.hpp>

#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>
#include <chrono>
#include <algorithm>
#include <unordered_map>
#include <atomic>
#include <thread>
#include <csignal>


#include "../aux/ValidityCheckLocal.h"
#include "../aux/RoutingLayer.h"
#include "../include/localPlannerTEBV2.hpp"
#include "../include/purePursuit.hpp"

using clk = std::chrono::steady_clock;


// ============================================================================
// CLI args
// ============================================================================
struct VizArgs {
    std::string AreaFile   = "../test.area";
    std::string RouteCSV   = "../stitched_path.csv";
    double      Speed      = 3.0;   // initial vehicle speed [m/s]
    int         WindowW    = 1280;
    int         WindowH    = 900;
    double      ObstacleHalfSize = 0.5; // [m] half-width/half-height of dropped obstacles
    double      FollowRadius = 20.0;    // [m] half-width of the follow-camera view
    double      wheelbase_m     = 2.0;
};

struct SharedPath
{
    mutable std::mutex m;
    std::vector<Vec3> pts;
    bool valid = false;
    uint64_t seq = 0;

    clk::time_point stamp = clk::now();

    void write(const std::vector<Vec3>& p, bool ok)
    {
        std::lock_guard<std::mutex> lk(m);
        pts = p;
        valid = ok;
        stamp = clk::now();
        ++seq;

    }

    bool read(std::vector<Vec3>& out, uint64_t& seq_out)
    {
        std::lock_guard<std::mutex> lk(m);
        out = pts;
        seq_out = seq;
        return valid;
    }

};

VizArgs parseArgs(int argc, char** argv) {
    VizArgs a;
    for (int i = 1; i < argc; ++i) {
        std::string s = argv[i];
        if      (s.rfind("--area=", 0) == 0)  a.AreaFile = s.substr(7);
        else if (s.rfind("--route=", 0) == 0) a.RouteCSV = s.substr(8);
        else if (s.rfind("--speed=", 0) == 0) a.Speed = std::stod(s.substr(8));
        else if (s.rfind("--window=", 0) == 0) {
            std::string wh = s.substr(9);
            size_t xpos = wh.find('x');
            if (xpos != std::string::npos) {
                a.WindowW = std::stoi(wh.substr(0, xpos));
                a.WindowH = std::stoi(wh.substr(xpos + 1));
            }
        }
        else if (s.rfind("--obst-size=", 0) == 0) a.ObstacleHalfSize = std::stod(s.substr(12));
        else if (s.rfind("--follow-radius=", 0) == 0) a.FollowRadius = std::stod(s.substr(16));
        else std::cout << "Unknown argument: " << s << "\n";
    }
    return a;
}

// ============================================================================
// Reference path CSV: X,Y,Dir,V,Curvature per line (matches the CSV dump in
// publishPath()/onObstacles() -- stitched_path.csv / local_path.csv format).
// ============================================================================
std::vector<Vec3> loadRouteCSV(const std::string& path) {
    std::vector<Vec3> route;
    std::ifstream f(path);
    if (!f.is_open()) {
        std::cerr << "Could not open route CSV: " << path << "\n";
        return route;
    }
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string tok;
        double vals[5] = {0, 0, 1, 0, 0};
        int idx = 0;
        while (std::getline(ss, tok, ',') && idx < 5) {
            try { vals[idx] = std::stod(tok); } catch (...) { }
            ++idx;
        }
        if (idx < 2) continue; // need at least X,Y
        Vec3 p{};
        p.X = vals[0]; p.Y = vals[1]; p.Dir = vals[2]; p.V = vals[3]; p.Curvature = vals[4];
        route.push_back(p);
    }
    std::cout << "Loaded " << route.size() << " route points from " << path << "\n";
    return route;
}

// ============================================================================
// World <-> pixel camera
// ============================================================================
struct Camera {
    double minX = 0, minY = 0, scale = 1.0;
    int width = 1280, height = 900;

    cv::Point2f toPx(double x, double y) const {
        return cv::Point2f(
            static_cast<float>((x - minX) * scale),
            static_cast<float>(height - (y - minY) * scale)); // flip Y: world-up = image-up
    }
    void toWorld(int px, int py, double& x, double& y) const {
        x = px / scale + minX;
        y = (height - py) / scale + minY;
    }
};

Camera computeCamera(const std::vector<Vec3>& route, int width, int height, double padFrac = 0.15) {
    Camera cam; cam.width = width; cam.height = height;
    if (route.empty()) { cam.minX = 0; cam.minY = 0; cam.scale = 1.0; return cam; }

    double minX = 1e18, maxX = -1e18, minY = 1e18, maxY = -1e18;
    for (const auto& p : route) {
        minX = std::min(minX, (double)p.X); maxX = std::max(maxX, (double)p.X);
        minY = std::min(minY, (double)p.Y); maxY = std::max(maxY, (double)p.Y);
    }
    double spanX = std::max(maxX - minX, 1.0);
    double spanY = std::max(maxY - minY, 1.0);
    double padX = spanX * padFrac, padY = spanY * padFrac;
    minX -= padX; maxX += padX; minY -= padY; maxY += padY;
    spanX = maxX - minX; spanY = maxY - minY;

    double scaleX = width / spanX, scaleY = height / spanY;
    cam.scale = std::min(scaleX, scaleY);
    // Center the content within the window on whichever axis has slack.
    double usedW = spanX * cam.scale, usedH = spanY * cam.scale;
    cam.minX = minX - (width - usedW) / 2.0 / cam.scale;
    cam.minY = minY - (height - usedH) / 2.0 / cam.scale;
    return cam;
}

// Fixed-zoom camera centered on a world point (the vehicle) -- same
// centering logic as computeCamera, but the "bounding box" is just a
// radius_m square around (cx, cy) instead of derived from the whole route.
// Recomputed every frame in follow mode, so the view stays tight around
// whatever the local planner is actually doing right now.
Camera computeFollowCamera(double cx, double cy, double radius_m, int width, int height) {
    Camera cam; cam.width = width; cam.height = height;
    double spanX = radius_m * 2.0, spanY = radius_m * 2.0;
    double scaleX = width / spanX, scaleY = height / spanY;
    cam.scale = std::min(scaleX, scaleY);
    double usedW = spanX * cam.scale, usedH = spanY * cam.scale;
    cam.minX = (cx - radius_m) - (width - usedW) / 2.0 / cam.scale;
    cam.minY = (cy - radius_m) - (height - usedH) / 2.0 / cam.scale;
    return cam;
}

// ============================================================================
// Drivable-region polygons -- drawn directly from Junction::cached_polygon
// and RoadSegment::cached_polygon, i.e. the EXACT same geometry
// ValidityCheckLocal::isInsideZone() tests against (see setLocalZone()/
// setRouteZone() in ValidityCheckLocal.h). This replaces an earlier version
// that reconstructed an approximate boundary from bound_angle_radians +
// half_width_left_m/half_width_right_m -- those fields turned out to be
// unrelated to the actual collision polygons, which is why that rendering
// looked wrong. No more guessing: what you see here is what the planner
// actually checks against.
// ============================================================================
struct PolygonWorld {
    std::vector<cv::Point2d> pts;
};

bool bboxOverlaps(double x, double y, double minX, double minY, double maxX, double maxY) {
    return x >= minX && x <= maxX && y >= minY && y <= maxY;
}

std::vector<PolygonWorld> buildDrivablePolygonsWorld(const AreaGraph& graph) {
    std::vector<PolygonWorld> out;
    auto addPoly = [&](const std::vector<Vec2>& poly) {
        if (poly.size() < 3) return;
        PolygonWorld pw;
        pw.pts.reserve(poly.size());
        for (const auto& p : poly) pw.pts.push_back(cv::Point2d(p.X, p.Y));
        out.push_back(std::move(pw));
    };
    for (const auto& seg : graph.segments()) addPoly(seg.cached_polygon);
    for (const auto& jxn : graph.junctions()) addPoly(jxn.cached_polygon);
    return out;
}

struct Polygon {
    std::vector<cv::Point> pts;
};

// Per-frame projection into whichever camera is currently active, with a
// cheap world-space bbox cull so a tight follow-camera doesn't pay for
// projecting geometry nowhere near the visible window.
std::vector<Polygon> projectPolygons(const std::vector<PolygonWorld>& polysWorld, const Camera& cam) {
    std::vector<Polygon> out;
    double minX, minY; cam.toWorld(0, cam.height, minX, minY);
    double maxX, maxY; cam.toWorld(cam.width, 0, maxX, maxY);
    double padX = (maxX - minX) * 0.1, padY = (maxY - minY) * 0.1;
    minX -= padX; maxX += padX; minY -= padY; maxY += padY;

    for (const auto& pw : polysWorld) {
        bool anyInside = false;
        for (const auto& p : pw.pts)
            if (bboxOverlaps(p.x, p.y, minX, minY, maxX, maxY)) { anyInside = true; break; }
        if (!anyInside) continue;

        Polygon poly;
        poly.pts.reserve(pw.pts.size());
        for (const auto& p : pw.pts) poly.pts.push_back(cam.toPx(p.x, p.y));
        out.push_back(std::move(poly));
    }
    return out;
}

// ============================================================================
// Obstacles: simple axis-aligned boxes, user-managed via mouse clicks.
// ============================================================================
struct SimObstacle {
    double cx, cy, halfW, halfH;
};

ADComms::lidarObstPolyMsg buildObstacleMsg(const std::vector<SimObstacle>& obstacles) {
    ADComms::lidarObstPolyMsg msg;
    msg.N = static_cast<int>(std::min<size_t>(obstacles.size(), 200));
    for (int i = 0; i < msg.N; ++i) {
        const auto& o = obstacles[i];
        // Corner order matches gatherObstacles()'s expected layout:
        // pts[i][0..3] = x0..x3, pts[i][4..7] = y0..y3.
        double xs[4] = { o.cx - o.halfW, o.cx + o.halfW, o.cx + o.halfW, o.cx - o.halfW };
        double ys[4] = { o.cy - o.halfH, o.cy - o.halfH, o.cy + o.halfH, o.cy + o.halfH };
        for (int k = 0; k < 4; ++k) { msg.pts[i][k] = xs[k]; msg.pts[i][4 + k] = ys[k]; }
    }
    return msg;
}

// ============================================================================
// Vehicle arc-length playback along the pristine global route (open-loop --
// see design note at top: this harness tests the PLANNER's output, it does
// not close the loop with a controller that would actually follow it).
// ============================================================================
struct RoutePlayback {
    std::vector<double> cumDist; // cumDist[i] = arc length from route[0] to route[i]

    void build(const std::vector<Vec3>& route) {
        cumDist.assign(route.size(), 0.0);
        for (size_t i = 1; i < route.size(); ++i) {
            double dx = route[i].X - route[i - 1].X;
            double dy = route[i].Y - route[i - 1].Y;
            cumDist[i] = cumDist[i - 1] + std::sqrt(dx * dx + dy * dy);
        }
    }
    double totalLength() const { return cumDist.empty() ? 0.0 : cumDist.back(); }

};

// ============================================================================
// Mouse interaction state
// ============================================================================
struct UIState {
    mutable std::mutex m;
    std::vector<SimObstacle> obstacles;
    Camera cam;
    double obstacleHalfSize = 0.3;
    bool haveObs = false;
};

void onMouse(int event, int px, int py, int, void* userdata) {
    auto* ui = static_cast<UIState*>(userdata);
    if (event == cv::EVENT_LBUTTONDOWN) {
        double wx, wy; ui->cam.toWorld(px, py, wx, wy);
        ui->obstacles.push_back({wx, wy, ui->obstacleHalfSize, ui->obstacleHalfSize});
        ui->haveObs  = true;
        std::cout << "Added obstacle at (" << wx << ", " << wy << ")\n";
    } else if (event == cv::EVENT_RBUTTONDOWN) {
        double wx, wy; ui->cam.toWorld(px, py, wx, wy);
        std::lock_guard<std::mutex> lk(ui->m);
        if (ui->obstacles.empty()) return;
        size_t best = 0; double bestD = 1e18;
        for (size_t i = 0; i < ui->obstacles.size(); ++i) {
            double dx = ui->obstacles[i].cx - wx, dy = ui->obstacles[i].cy - wy;
            double d = dx * dx + dy * dy;
            if (d < bestD) { bestD = d; best = i; }
        }

        ui->obstacles.erase(ui->obstacles.begin() + best);
        std::cout << "Removed nearest obstacle\n";
    }
}

// ============================================================================
// Drawing helpers
// ============================================================================
void drawDrivablePolygons(cv::Mat& img, const std::vector<Polygon>& polys) {
    for (const auto& poly : polys) {
        if (poly.pts.size() > 1)
            cv::polylines(img, poly.pts, true, cv::Scalar(140, 140, 140), 1, cv::LINE_AA);
    }
}

void drawPath(cv::Mat& img, const std::vector<Vec3>& route, const Camera& cam,
              const cv::Scalar& color, int thickness) {
    if (route.size() < 2) return;
    std::vector<cv::Point> pts;
    pts.reserve(route.size());
    for (const auto& p : route) pts.push_back(cam.toPx(p.X, p.Y));
    cv::polylines(img, pts, false, color, thickness, cv::LINE_AA);
}

void drawObstacles(cv::Mat& img, const std::vector<SimObstacle>& obstacles,
                    const Camera& cam, const ValidityCheckLocal* validity, float margin) {
    for (const auto& o : obstacles) {
        cv::Point2f c0 = cam.toPx(o.cx - o.halfW, o.cy - o.halfH);
        cv::Point2f c1 = cam.toPx(o.cx + o.halfW, o.cy + o.halfH);
        cv::rectangle(img, c0, c1, cv::Scalar(60, 60, 220), -1, cv::LINE_AA);

        // Effective clearance radius the planner actually keeps away from
        // (matches minClearance()'s formula: obstacle half-diagonal + margin
        // + vehicle width), drawn as a thin dashed-ish outline for intuition.
        double radius = std::hypot(o.halfW, o.halfH);
        double extra = validity ? validity->vehicleWidth() : 0.5;
        double clearance = radius + margin + extra;
        cv::Point2f centerPx = cam.toPx(o.cx, o.cy);
        int radiusPx = static_cast<int>(clearance * cam.scale);
        cv::circle(img, centerPx, radiusPx, cv::Scalar(120, 120, 255), 1, cv::LINE_AA);
    }
}

void drawVehicle(cv::Mat& img, const VehicleState& vs, const Camera& cam, bool halted) {
    // Cosmetic only -- see design note at top about setVehicle()'s real
    // parameter semantics being unconfirmed. Purely a visual marker.
    const double length = 2.2, width = 1.3;
    double c = std::cos(vs.theta), s = std::sin(vs.theta);
    auto corner = [&](double fx, double fy) {
        double wx = vs.x + fx * c - fy * s;
        double wy = vs.y + fx * s + fy * c;
        return cam.toPx(wx, wy);
    };
    std::vector<cv::Point> pts = {
        corner(length * 0.5, width * 0.5),
        corner(-length * 0.5, width * 0.5),
        corner(-length * 0.5, -width * 0.5),
        corner(length * 0.5, -width * 0.5)
    };
    cv::fillConvexPoly(img, pts,  halted ? cv::Scalar(40, 40, 200) : cv::Scalar(40, 200, 40), cv::LINE_AA);
}


void extractLocalWindow(const std::vector<Vec3>& route, std::vector<Vec3>& window, const VehicleState& vs, double horizonM) {
    if (route.size() < 2) return;

    // std::cout << "HERE\n";
    int i0 = 0;
    double bestD2 = 1e18;
    for (size_t i = 0; i < route.size(); ++i) {
        double dx = route[i].X - vs.x, dy = route[i].Y - vs.y;
        double d2 = dx * dx + dy * dy;
        if (d2 < bestD2) { bestD2 = d2; i0 = static_cast<int>(i); }
    }
    int i1 = i0;
    double acc = 0.0;
    for (size_t i = i0; i + 1 < route.size(); ++i) {
        if (route[i + 1].Dir != route[i0].Dir) break; // stop at a gear change -- can't represent a cusp in one window
        acc += std::hypot(route[i + 1].X - route[i].X, route[i + 1].Y - route[i].Y);
        i1 = static_cast<int>(i + 1);
        if (acc >= horizonM) break;
    }
    if (i1 <= i0) i1 = std::min(static_cast<int>(route.size()) - 1, i0 + 1);
    window = std::vector<Vec3>(route.begin() + i0, route.begin() + i1 + 1);
}


class PlanControlTestNode
{
    public: 
        PlanControlTestNode(VizArgs args_) : args(args_)
        {
            std::string err;
            if (!router.loadArea(args.AreaFile, err)) {
                std::cerr << "Failed to load area: " << err << "\n";
            }
            
            
            //Setup Path Validity Object

            validity_local_.init(&router.graph());
            validity_local_.setVehicle(2.2f, 1.3f);
            std::vector<uint32_t> allNodeIDs;
            allNodeIDs.reserve(router.graph().nodeCount());
            for(const auto& node : router.graph().nodes())
            {
                allNodeIDs.push_back(node.id);
            }
            validity_local_.setRouteZone(allNodeIDs);

            speedCap.store(args.Speed);


            localPlanner.init(&validity_local_, planHorizon_, margin_);

            globalRoute = loadRouteCSV(args.RouteCSV);
            if (globalRoute.size() < 2)
            {
                throw std::runtime_error("Route CSV had fewer than 2 points -- nothing to simulate.");
            }
            
            drivablePolygonsWorld_ = buildDrivablePolygonsWorld(router.graph());
            overviewCam_ = computeCamera(globalRoute, args.WindowW, args.WindowH);
            ui_.cam = overviewCam_;
            ui_.obstacleHalfSize = args.ObstacleHalfSize;

            resetVehicle();

        
        }

        ~PlanControlTestNode(){stop();}

        void spin()
        {
            localThread_ =   std::thread(&PlanControlTestNode::localLoop, this);
            controlThread_ = std::thread(&PlanControlTestNode::controlLoop, this);

            renderLoop();

            stop();
        }

        void stop()
        {
            if(stopped_.exchange(true)) return; // checks if stopped_ is already true

            shutdown_.store(true);

            if(localThread_.joinable()) localThread_.join();
            if(controlThread_.joinable()) controlThread_.join();

            std::cout << "[SIM] emergency stop (simulated)\n";
        }

        static void requestShutdown(int)
        {
            shutdown_.store(true);
        }
        


    
    private:

        void resetVehicle()
        {
            {
                std::lock_guard<std::mutex> lk(stateMtx_);
                vs_.x        = globalRoute[0].X;
                vs_.y        = globalRoute[0].Y;
                vs_.theta    = atan2(globalRoute[1].Y - globalRoute[0].Y, globalRoute[1].X - globalRoute[0].X);
                vs_.current_speed = 0.0;
            }

            {
                std::lock_guard<std::mutex> lk(histMtx_);
                trajectoryHistory.clear();
            }

            consecutiveOk_.store(0);
            halted_.store(false);
            localPath_.write({},false);
            goalAnnounced_ = false;
        }

        
        // =======================================================================
        //  Thread 1: Local planner -- 10 Hz
        // =======================================================================

        void localLoop()
        {
            
            const auto period = std::chrono::milliseconds(100);

            std::vector<Vec3> outpath;
            std::vector<Vec3> localWindow;
            
            while(!shutdown_.load())
            {
                const auto tick_start = clk::now();
                
                VehicleState vs;
                {
                    std::lock_guard<std::mutex> lk(stateMtx_);
                    vs = vs_;
                }

                // std::cout << "globalRoute : " << globalRoute.size() << std::endl;
                outpath = globalRoute; 
                // std::cout << "outpath : " << outpath.size() << std::endl;
                

                if(outpath.size() < 2)
                { 
                    std::this_thread::sleep_for(period); 
                    continue;
                }

                ADComms::lidarObstPolyMsg obs_copy;
                bool have_obs = false;
                
                {
                    std::lock_guard<std::mutex> lk(ui_.m);
                    if(ui_.haveObs)
                    {
                        obs_copy = buildObstacleMsg(ui_.obstacles);
                        have_obs = true;
                    }
                }

                bool ok = true;
                double planMs = 0.0;

                if(have_obs)
                {
                    const auto t0 = clk::now();
                    ok = localPlanner.localPlan(&obs_copy,vs, outpath);
                    // std::cout << "outpath after localPlan: " << outpath.size() << std::endl;
                    planMs = std::chrono::duration<double, std::milli>(clk::now() - t0).count();

                    

                }

                extractLocalWindow(outpath, localWindow, vs, planHorizon_);

                const double dist_to_end = std::hypot(outpath.back().X - vs.x, outpath.back().Y - vs.y);
                atGoal_.store(dist_to_end < 2.0);

                profileCurrPath(localWindow)

                // std::cout << "ok = " << ok << std::endl;
                // std::cout << "localWindow : " << localWindow.size() << std::endl;

                if(!ok)
                {

                    localPath_.write({}, /*valid = */false);
                    consecutiveOk_.store(0);
                    std::cerr << "[LOCAL] infeasible (" << " consecutive)"
                          << "  optimize=" << std::fixed << std::setprecision(1) << planMs << " ms\n";

                }
                else{
                    localPath_.write(localWindow, /*valid = */true);
                    consecutiveOk_.fetch_add(1);
                    if (obs_copy.N > 0)
                    std::cout << "[LOCAL] ok  optimize=" << std::fixed << std::setprecision(1) << planMs << " ms\n";

                }
                std::this_thread::sleep_until(tick_start + period);
                
            }
        }


        // =======================================================================
        //  Thread 2: Controller -- 50 Hz. Also the plant, in simulation.
        // =======================================================================

        void controlLoop()
        {
            const auto period = std::chrono::microseconds(20'000);
            auto lastTime = clk::now();
            
            while(!shutdown_.load())
            {
                const auto tick_start = clk::now();
                double dt = std::chrono::duration<double>(tick_start - lastTime).count();
                lastTime = tick_start;
                dt = std::min(dt, 0.1);   // clamp if a tick stalls (window drag etc.)

                std::vector<Vec3> localPath;
                uint64_t seq = 0;
                const bool pathValid = localPath_.read(localPath, seq);

                bool bad = !pathValid || localPath.size() < 2;

                // std::cout << "localPath: " << localPath.size() << std::endl;
                // std::cout << "bad: " << bad << std::endl;
                
                //HALTING LOGIC
                if(bad)
                {
                    if(!halted_.load())
                    {
                        halted_.store(true);
                        std::cerr << "[CTRL] HALT: valid=" << pathValid
                              << " pts=" << localPath.size()
                              << std::endl;

                    }
                    
                }
                else if(halted_.load() && (consecutiveOk_.load() > HALT_RELEASE_STREAK))
                {

                    std::cout << "[CTRL] releasing halt\n";
                    halted_.store(false);
                    
                }
                
                VehicleState vs;
                {
                    std::lock_guard<std::mutex> lk(stateMtx_);
                    vs = vs_;
                }

                if(halted_.load())
                {
                    vs.current_speed    = std::max(0.0, vs.current_speed - HALT_DECEL * dt);
                    vs.x               += vs.current_speed * std::cos(vs.theta) * dt;
                    vs.y               += vs.current_speed * std::sin(vs.theta) * dt; 
                }
                else
                {
                    // controller.setTrajectory(localPath);
                    controller.setPath(localPath,seq);
                    PurePursuitOutput out = controller.step(vs, vs.current_speed);

                    if (atGoal_.load()) {
                        vs.current_speed = 0.0;
                        if (!goalAnnounced_) { goalAnnounced_ = true; std::cout << "[CTRL] mission complete\n"; }
                    }else {
                        vs.current_speed = std::min(out.speed_mps, speedCap.load());
                        vs.x     += vs.current_speed * std::cos(vs.theta) * dt;
                        vs.y     += vs.current_speed * std::sin(vs.theta) * dt;
                        vs.theta += (vs.current_speed / args.wheelbase_m) * std::tan(out.steer_angle_rad) * dt;
                    }
                    

                }
                {
                    std::lock_guard<std::mutex> lk(stateMtx_);
                    vs_ = vs;
                }
                {
                    std::lock_guard<std::mutex> lk(histMtx_);
                    Vec3 h{};
                    h.X = vs.x; h.Y = vs.y; h.V = vs.current_speed;

                    trajectoryHistory.push_back(h);
                    if(trajectoryHistory.size() > MAX_HISTORY) trajectoryHistory.erase(trajectoryHistory.begin());
                    
                }

                std::this_thread::sleep_until(period + tick_start);
            }
        }
            // =======================================================================
        //  Main thread: render + input
        // =======================================================================
        void renderLoop()
        {
            const std::string winName = "TEB local planner -- threaded test";
            cv::namedWindow(winName, cv::WINDOW_AUTOSIZE);
            cv::setMouseCallback(winName, onMouse, &ui_);
        
            std::cout << "\nControls: left-click add obstacle, right-click remove nearest,\n"
                         "  c=clear, space=pause, r=reset, v=toggle follow/overview,\n"
                         "  [ / ] = shrink/grow follow radius, +/-=speed cap, q/ESC=quit\n\n";
        
            while (!shutdown_.load())
            {
                VehicleState vs;
                { std::lock_guard<std::mutex> lk(stateMtx_); vs = vs_; }
            
                std::vector<Vec3> livePath;
                uint64_t seq = 0;
                const bool path_valid = localPath_.read(livePath, seq);
            
                std::vector<SimObstacle> obsSnapshot;
                { std::lock_guard<std::mutex> lk(ui_.m); obsSnapshot = ui_.obstacles; }
            
                std::vector<Vec3> histSnapshot;
                { std::lock_guard<std::mutex> lk(histMtx_); histSnapshot = trajectoryHistory; }
            
                ui_.cam = followMode_ ? computeFollowCamera(vs.x, vs.y, followRadius_, args.WindowW, args.WindowH)
                                      : overviewCam_;
            
                std::vector<Polygon> drivablePolygons = projectPolygons(drivablePolygonsWorld_, ui_.cam);
            
                cv::Mat img(ui_.cam.height, ui_.cam.width, CV_8UC3, cv::Scalar(30, 30, 32));
                drawDrivablePolygons(img, drivablePolygons);
                drawPath(img, globalRoute, ui_.cam, cv::Scalar(180, 120, 40), 1);   // global route, thin
                if (path_valid)
                    drawPath(img, livePath, ui_.cam, cv::Scalar(40, 220, 220), 2);   // live local path
                drawPath(img, histSnapshot, ui_.cam, cv::Scalar(220, 0, 0), 2);      // driven trace
                drawObstacles(img, obsSnapshot, ui_.cam, &validity_local_, margin_);
                drawVehicle(img, vs, ui_.cam, halted_.load());
                
                std::ostringstream hud;
                hud << (paused_.load() ? "[PAUSED] " : "")
                    << (halted_.load() ? "[HALTED] " : "")
                    << (followMode_ ? "[FOLLOW r=" + std::to_string((int)followRadius_) + "m] " : "[OVERVIEW] ")
                    << "v=" << std::fixed << std::setprecision(1) << vs.current_speed
                    << "/" << speedCap.load() << " m/s"
                    << "  obs=" << obsSnapshot.size()
                    << "  path_age=" << std::setprecision(0)
                    << "  okStreak=" << consecutiveOk_.load()
                    << "  local=" << (path_valid ? "ok" : "FAILED");
                cv::putText(img, hud.str(), cv::Point(12, 24), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                            cv::Scalar(230, 230, 230), 1, cv::LINE_AA);
            
                cv::imshow(winName, img);
                int key = cv::waitKey(15) & 0xFF;     // ~60 Hz render; also pumps the mouse callback
                if (key == 'q' || key == 27) { shutdown_.store(true); break; }
                else if (key == ' ') paused_.store(!paused_.load());
                else if (key == 'c') {
                    std::lock_guard<std::mutex> lk(ui_.m);
                    ui_.obstacles.clear();
                }
                else if (key == 'r') resetVehicle();
                else if (key == 'v') followMode_ = !followMode_;
                else if (key == '[') followRadius_ = std::max(5.0, followRadius_ - 5.0);
                else if (key == ']') followRadius_ += 5.0;
                else if (key == '+' || key == '=') speedCap.store(speedCap.load() + 0.5);
                else if (key == '-' || key == '_') speedCap.store(std::max(0.0, speedCap.load() - 0.5));
            }
            cv::destroyWindow(winName);
        }


        void smoothAndResampleTrajectory(std::vector<Vec3>& currPath, double resolution_m = 0.1,
                         int    windowSize   = 15,
                         int    degree       = 3)
        {
            if (currPath.size() < 2) return;
        
            // std::cout << "PATH SIZE1: " << currPath.size() << std::endl;
            if (windowSize % 2 == 0) windowSize++;
            degree = std::min(degree, windowSize - 1);
            int half = windowSize / 2;
        
            auto polyEval = [](const std::vector<double>& c, double t) {
                double val = 0.0, tp = 1.0;
                for (double ci : c) { val += ci * tp; tp *= t; }
                return val;
            };
            // std::cout << "PATH SIZE2: " << currPath.size() << std::endl;
            auto localPolyFit = [&](const std::vector<double>& t,
                                    const std::vector<double>& b) -> std::vector<double>
            {
                int n    = static_cast<int>(t.size());
                int cols = degree + 1;
                std::vector<std::vector<double>> ATA(cols, std::vector<double>(cols, 0.0));
                std::vector<double>              ATb(cols, 0.0);
                for (int i = 0; i < n; ++i) {
                    double tp = 1.0;
                    std::vector<double> row(cols);
                    for (int j = 0; j < cols; ++j) { row[j] = tp; tp *= t[i]; }
                    for (int r = 0; r < cols; ++r) {
                        ATb[r] += row[r] * b[i];
                        for (int c = 0; c < cols; ++c)
                            ATA[r][c] += row[r] * row[c];
                    }
                }
                for (int col = 0; col < cols; ++col) {
                    int pivot = col;
                    for (int row = col + 1; row < cols; ++row)
                        if (std::abs(ATA[row][col]) > std::abs(ATA[pivot][col])) pivot = row;
                    std::swap(ATA[col], ATA[pivot]);
                    std::swap(ATb[col], ATb[pivot]);
                    if (std::abs(ATA[col][col]) < 1e-12) continue;
                    double inv = 1.0 / ATA[col][col];
                    for (int row = col + 1; row < cols; ++row) {
                        double f = ATA[row][col] * inv;
                        for (int k = col; k < cols; ++k) ATA[row][k] -= f * ATA[col][k];
                        ATb[row] -= f * ATb[col];
                    }
                }
                std::vector<double> coeff(cols, 0.0);
                for (int row = cols - 1; row >= 0; --row) {
                    if (std::abs(ATA[row][row]) < 1e-12) continue;
                    coeff[row] = ATb[row];
                    for (int k = row + 1; k < cols; ++k) coeff[row] -= ATA[row][k] * coeff[k];
                    coeff[row] /= ATA[row][row];
                }
                return coeff;
            };
            // std::cout << "PATH SIZE3: " << currPath.size() << std::endl;
            struct Segment { int start, end, direction; };
            std::vector<Segment> segments;
            int segStart = 0;
            for (int i = 1; i <= currPath.size(); ++i) {
                bool last    = (i == currPath.size());
                bool changed = !last && (currPath[i].Dir != currPath[i - 1].Dir);
                if (changed || last) {
                    segments.push_back({ segStart, i - 1,
                                          static_cast<int>(currPath[segStart].Dir) });
                    segStart = i;
                }
            }
            // std::cout << "PATH SIZE4: " << currPath.size() << std::endl;
            std::vector<std::array<double, 3>> allOutput;
        
            for (const auto& seg : segments) {
                int n = seg.end - seg.start + 1;
            
                std::vector<double> smoothX(n), smoothY(n);
                for (int i = 0; i < n; ++i) {
                    int wStart = std::max(0, i - half);
                    int wEnd   = std::min(n - 1, i + half);
                    int wn     = wEnd - wStart + 1;
                    std::vector<double> t(wn, 0.0);
                    for (int k = 1; k < wn; ++k) {
                        int gi = seg.start + wStart + k;
                        double dx = currPath[gi].X - currPath[gi - 1].X;
                        double dy = currPath[gi].Y - currPath[gi - 1].Y;
                        t[k] = t[k - 1] + std::sqrt(dx * dx + dy * dy);
                    }
                    double tCenter = t[i - wStart];
                    std::vector<double> wx(wn), wy(wn);
                    for (int k = 0; k < wn; ++k) {
                        wx[k] = currPath[seg.start + wStart + k].X;
                        wy[k] = currPath[seg.start + wStart + k].Y;
                    }
                    auto cx = localPolyFit(t, wx);
                    auto cy = localPolyFit(t, wy);
                    smoothX[i] = polyEval(cx, tCenter);
                    smoothY[i] = polyEval(cy, tCenter);
                }
            
                std::vector<double> arc(n, 0.0);
                for (int i = 1; i < n; ++i) {
                    double dx = smoothX[i] - smoothX[i - 1];
                    double dy = smoothY[i] - smoothY[i - 1];
                    arc[i] = arc[i - 1] + std::sqrt(dx * dx + dy * dy);
                }
                double totalLen = arc[n - 1];
            
                allOutput.push_back({ smoothX[0], smoothY[0], static_cast<double>(seg.direction) });
                double nextDist = resolution_m;
                int j = 0;
                while (nextDist < totalLen) {
                    while (j + 1 < n - 1 && arc[j + 1] <= nextDist) ++j;
                    double span  = arc[j + 1] - arc[j];
                    double alpha = (span > 1e-12) ? (nextDist - arc[j]) / span : 0.0;
                    double ix = smoothX[j] + alpha * (smoothX[j + 1] - smoothX[j]);
                    double iy = smoothY[j] + alpha * (smoothY[j + 1] - smoothY[j]);
                    allOutput.push_back({ ix, iy, static_cast<double>(seg.direction) });
                    nextDist += resolution_m;
                }
                allOutput.push_back({ smoothX[n - 1], smoothY[n - 1], static_cast<double>(seg.direction) });
            }
            // std::cout << "PATH SIZE5: " << currPath.size() << std::endl;
            // std::cout << "ALLOUTPUT SIZE5: " << allOutput.size() << std::endl;
            currPath.clear();
        
            for (int i = 0; i < allOutput.size(); ++i) {
                Vec3 temp = {allOutput[i][0],allOutput[i][1],allOutput[i][2], MAX_SPEED, 0.0f};
                currPath.push_back(temp);
            }
            std::cout << "[SMOOTH] " << currPath.size() << " points after smoothing + resampling\n";
        }


        /**
         * Compute curvature using spaced Menger curvature + moving-average smoothing.
         *
         * @param n            Number of trajectory points
         * @param step         Index stride for finite difference (e.g. 10 = 1 m at 10 pts/m)
         * @param smoothWindow Moving-average half-window applied after strided diff
         */
        void computeCurvature(std::vector<Vec3>& currPath, int n, int step = 10, int smoothWindow = 51) {

        
            // Step 1: Menger curvature with index stride
            std::vector<double> rawKappa(n, 0.0);
            for (int i = step; i < n - step; ++i) {
                const double x1 = currPath[i - step].X, y1 = currPath[i - step].Y;
                const double x2 = currPath[i      ].X, y2 = currPath[i      ].Y;
                const double x3 = currPath[i + step].X, y3 = currPath[i + step].Y;
            
                const double ax = x2 - x1, ay = y2 - y1;
                const double bx = x3 - x2, by = y3 - y2;
                const double cx = x1 - x3, cy = y1 - y3;
            
                const double la = std::hypot(ax, ay);
                const double lb = std::hypot(bx, by);
                const double lc = std::hypot(cx, cy);
            
                const double cross = std::abs(ax * by - ay * bx);
                const double denom = la * lb * lc;
            
                rawKappa[i] = (denom > 1e-9) ? (2.0 * cross / denom) : 0.0;
            }
        
            // Step 2: Moving-average smoothing
            int half = smoothWindow / 2;
            for (int i = 0; i < n; ++i) {
                int lo = std::max(0, i - half);
                int hi = std::min(n - 1, i + half);
                float sum = 0.0f;
                for (int k = lo; k <= hi; ++k) sum += rawKappa[k];
                currPath[i].Curvature = sum / (hi - lo + 1);
            }
        }
        



        void computeVelocityProfile(std::vector<Vec3>& currPath, double points_per_meter_val) {
            if (currPath.size() < 2 ) return;
            // targetVelocity.assign(numPoints, MAX_SPEED);

            const double ds = 1.0 / points_per_meter_val;

            // Segment bookkeeping
            struct Seg { int start, end; bool isReverse; };
            std::vector<Seg> segs;
            {
                int s0 = 0;
                for (int i = 1; i <= currPath.size(); ++i) {
                    bool last    = (i == currPath.size());
                    bool changed = !last && (currPath[i].Dir != currPath[i-1].Dir);
                    if (changed || last) {
                        segs.push_back({ s0, i-1, currPath[s0].Dir == 0 });
                        s0 = i;
                    }
                }
            }
        
            // Helper: per-segment constants
            auto segSpeedCap = [](const Seg& s) {
                return s.isReverse ? MAX_SPEED_REVERSE : MAX_SPEED;
            };
            auto segCrawlSpeed = [](const Seg& s) {
                return s.isReverse ? CRAWL_SPEED_REV : CRAWL_SPEED;
            };
            auto segEffMin = [](const Seg& s) {
                return s.isReverse ? EFFECTIVE_MIN_REV : EFFECTIVE_MIN;
            };
            auto segSlowStart = [](const Seg& s) {
                return s.isReverse ? SLOW_START_DIST_REV : SLOW_START_DIST;
            };
            auto segCrawlDist = [](const Seg& s) {
                return s.isReverse ? CRAWL_DIST_REV : CRAWL_DIST;
            };
            auto segStopDist = [](const Seg& s) {
                return s.isReverse ? STOP_DIST_REV : STOP_DIST;
            };
        
            // ── Pass 1: per-point ceiling ─────────────────────────────────────────
            for (const auto& seg : segs) {
                double spd_cap    = segSpeedCap(seg);
                double crawl_spd  = segCrawlSpeed(seg);
                double slow_start = segSlowStart(seg);
                double crawl_dist = segCrawlDist(seg);
                double stop_dist  = segStopDist(seg);
            
                for (int i = seg.start; i <= seg.end; ++i) {
                    int li = i - seg.start;
                
                    // a) Curvature limit
                    double kappa  = std::max(currPath[i].Curvature, 1e-6f);
                    double v_curv = std::clamp(std::sqrt(MAX_LAT_ACC / kappa),
                                               segEffMin(seg), spd_cap);
                
                    // b) Transition brake  [FIX-3: correct ordering slow_start > crawl_dist > stop_dist]
                    double dist_to_end = (seg.end - i) * ds;
                    double v_trans;
                    if      (dist_to_end > slow_start) {
                        v_trans = spd_cap;
                    } else if (dist_to_end > crawl_dist) {
                        double t = (dist_to_end - crawl_dist) / (slow_start - crawl_dist);
                        v_trans  = crawl_spd + t * (spd_cap - crawl_spd);
                    } else if (dist_to_end > stop_dist) {
                        double t = (dist_to_end - stop_dist) / (crawl_dist - stop_dist);
                        v_trans  = STOP_SPEED + t * (crawl_spd - STOP_SPEED);
                    } else {
                        v_trans = STOP_SPEED;
                    }
                
                    // c) Ramp-up from STOP_SPEED at segment start  [FIX-1 foundation]
                    double arc    = li * ds;
                    double v_ramp = (arc < RAMP_DIST)
                                  ? STOP_SPEED + (spd_cap - STOP_SPEED) * (arc / RAMP_DIST)
                                  : spd_cap;
                
                    currPath[i].V = std::min({ v_curv, v_trans, v_ramp });
                }
            }
        
            // ── Pass 2: forward accel ─────────────────────────────────────────────
            for (const auto& seg : segs)
                for (int i = seg.start+1; i <= seg.end; ++i)
                    currPath[i].V = std::min(currPath[i].V,
                        (float)std::sqrt(currPath[i-1].V*currPath[i-1].V + 2.0*MAX_ACC*ds));
                    
            // ── Pass 3: backward decel ────────────────────────────────────────────
            for (const auto& seg : segs)
                for (int i = seg.end-1; i >= seg.start; --i)
                    currPath[i].V = std::min(currPath[i].V,
                        (float)std::sqrt(currPath[i+1].V*currPath[i+1].V + 2.0*MAX_DEC*ds));
                    
            // ── Pass 4: hard floor (per-segment EFFECTIVE_MIN) ────────────────────
            for (const auto& seg : segs) {
                double eff_min = segEffMin(seg);
                for (int i = seg.start; i <= seg.end; ++i)
                    currPath[i].V = std::max(currPath[i].V, (float)eff_min);
            }
        
            // ── Pass 5: moving-average smoother (3 m window) ──────────────────────
            {
                const int half = std::max(3, static_cast<int>(points_per_meter_val * 3.0)) / 2;

                for (int i = 0; i < currPath.size(); ++i) {
                    int lo = std::max(0, i-half), hi = std::min(currPath.size()-1, (unsigned long)i+half);
                    double sum = 0.0;
                    for (int k = lo; k <= hi; ++k) sum += currPath[k].V;
                    currPath[i].V = sum / (hi-lo+1);
                }
            }
        
            // ── Pass 6: re-pin segment boundaries ────────────────────────────────
            for (const auto& seg : segs) {
                currPath[seg.start].V = STOP_SPEED;
                currPath[seg.end].V   = STOP_SPEED;
            }
        
            // ── Pass 7: per-segment kinematic re-sweep  [FIX-1] ──────────────────
            //   Backward decel, then forward accel WITH ramp ceiling enforced.
            for (const auto& seg : segs) {
                double spd_cap = segSpeedCap(seg);
            
                // backward decel
                for (int i = seg.end-1; i >= seg.start; --i)
                    currPath[i].V = std::min(currPath[i].V,
                        (float)std::sqrt(currPath[i+1].V*currPath[i+1].V + 2.0*MAX_DEC*ds));
                    
                // forward accel  — ramp ceiling applied at every step  [FIX-1]
                for (int i = seg.start+1; i <= seg.end; ++i) {
                    double arc    = (i - seg.start) * ds;
                    double v_ramp = (arc < RAMP_DIST)
                                  ? STOP_SPEED + (spd_cap - STOP_SPEED) * (arc / RAMP_DIST)
                                  : spd_cap;
                
                    double v_kin = std::sqrt(currPath[i-1].V*currPath[i-1].V
                                             + 2.0*MAX_ACC*ds);
                    currPath[i].V = std::min({currPath[i].V, (float)v_kin, (float)v_ramp});
                }
            }
        
            // Re-apply per-segment floor (skip boundary pins AND ramp zone)
            // [FIX-1] Do NOT apply EFFECTIVE_MIN while still inside the ramp zone —
            //         the ramp ceiling is below EFFECTIVE_MIN near the start, so the
            //         floor was overwriting it and causing the profile to launch high.
            for (const auto& seg : segs) {
                double spd_cap  = segSpeedCap(seg);
                double eff_min  = segEffMin(seg);
                for (int i = seg.start; i <= seg.end; ++i) {
                    if (i == seg.start || i == seg.end) continue;
                    double arc    = (i - seg.start) * ds;
                    if (arc < RAMP_DIST) continue;   // inside ramp zone: ramp is the ceiling, skip floor
                    currPath[i].V = std::max(currPath[i].V, (float)eff_min);
                }
            }
        
            std::cout << "[PROF]  Velocity profile computed for " << currPath.size() << " points.\n";
        }

        // Smooths/resamples currPath to a fixed point density, then derives
        // curvature and a kinodynamically-consistent velocity profile from that
        // geometry. Caller must already hold stateMtx -- this mutates currPath
        // in place.
        void profileCurrPath(std::vector<Vec3>& currPath)
        {
            smoothAndResampleTrajectory();
            computeCurvature(static_cast<int>(currPath.size()));
            computeVelocityProfile(10);
        }


        VizArgs args;
        UIState ui_;
        std::vector<PolygonWorld>   drivablePolygonsWorld_;
        Camera                      overviewCam_;

        ValidityCheckLocal validity_local_;
        RoutingLayer router;
        TebLocalPlanner localPlanner;
        PurePursuitController controller;
        VehicleState vs_;
        std::vector<Vec3> globalRoute;
        std::vector<Vec3> trajectoryHistory;
        std::thread localThread_, controlThread_;
        std::mutex stateMtx_, routeMtx_, histMtx_; 

        std::atomic<bool> stopped_{false};
        static std::atomic<bool> shutdown_;
        std::atomic<bool> paused_{false};
        std::atomic<bool> halted_{true};
        std::atomic<int> consecutiveOk_{0};
        std::atomic<bool> atGoal_{false};

        std::atomic<double> speedCap{0.0};

        SharedPath localPath_;

        bool   followMode_   = true;
        bool   goalAnnounced_ = false;
        double followRadius_ = 20.0;
        double planHorizon_ =  30;
        double margin_ = 0.7;
        
        const size_t MAX_HISTORY         = 500;
        const int HALT_RELEASE_STREAK = 3;
        const double HALT_DECEL = 2.0;


};
std::atomic<bool> PlanControlTestNode::shutdown_{false};

// ============================================================================
// main
// ============================================================================
int main(int argc, char** argv) {
    
    VizArgs args = parseArgs(argc, argv);
    
    std::signal(SIGINT,  PlanControlTestNode::requestShutdown);
    std::signal(SIGTERM, PlanControlTestNode::requestShutdown);

    try{
        PlanControlTestNode node(args);
        node.spin();
    }
    catch(const std::exception& e)
    {
        std::cerr << "FATAL: " << e.what() << std::endl;
    }

    std::cout << "SHUTDOWN CLEANLY\n";

    return 0;

}