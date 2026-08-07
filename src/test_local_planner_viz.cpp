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


#include "../aux/ValidityCheckLocal.h"
#include "../aux/RoutingLayer.h"
#include "../include/localPlannerTEB.hpp"
#include "../include/purePursuit.hpp"


// ============================================================================
// CLI args
// ============================================================================
struct VizArgs {
    std::string AreaFile   = "../test.area";
    std::string RouteCSV   = "../stitched_path.csv";
    double      Speed      = 3.0;   // initial vehicle speed [m/s]
    int         WindowW    = 1280;
    int         WindowH    = 900;
    double      ObstacleHalfSize = 0.3; // [m] half-width/half-height of dropped obstacles
    double      FollowRadius = 20.0;    // [m] half-width of the follow-camera view
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
    std::vector<SimObstacle> obstacles;
    Camera cam;
    double obstacleHalfSize = 4.5;
};

void onMouse(int event, int px, int py, int, void* userdata) {
    auto* ui = static_cast<UIState*>(userdata);
    if (event == cv::EVENT_LBUTTONDOWN) {
        double wx, wy; ui->cam.toWorld(px, py, wx, wy);
        ui->obstacles.push_back({wx, wy, ui->obstacleHalfSize, ui->obstacleHalfSize});
        std::cout << "Added obstacle at (" << wx << ", " << wy << ")\n";
    } else if (event == cv::EVENT_RBUTTONDOWN) {
        if (ui->obstacles.empty()) return;
        double wx, wy; ui->cam.toWorld(px, py, wx, wy);
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

void drawVehicle(cv::Mat& img, const VehicleState& vs, const Camera& cam) {
    // Cosmetic only -- see design note at top about setVehicle()'s real
    // parameter semantics being unconfirmed. Purely a visual marker.
    const double length = 3.0, width = 1.6;
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
    cv::fillConvexPoly(img, pts, cv::Scalar(40, 200, 40), cv::LINE_AA);
}


std::vector<Vec3> extractLocalWindow(const std::vector<Vec3>& route, const VehicleState& vs, double horizonM) {
    if (route.size() < 2) return route;
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
    return std::vector<Vec3>(route.begin() + i0, route.begin() + i1 + 1);
}


// ============================================================================
// main
// ============================================================================
int main(int argc, char** argv) {
    VizArgs args = parseArgs(argc, argv);

    RoutingLayer router;
    std::string err;
    if (!router.loadArea(args.AreaFile, err)) {
        std::cerr << "Failed to load area: " << err << "\n";
        return 1;
    }
    std::string rt_err;
    if (!router.loadRuntime("./tvs_campus_runtime_latest.area", rt_err))
        std::cerr << "Warning: no runtime cache: " << rt_err << "\n";

    ValidityCheckLocal validity_local_;
    validity_local_.init(&router.graph());
    validity_local_.setVehicle(2.2f, 1.3f);


    {
        std::vector<uint32_t> allNodeIds;
        allNodeIds.reserve(router.graph().nodeCount());
        for (const auto& n : router.graph().nodes()) allNodeIds.push_back(n.id);
        validity_local_.setRouteZone(allNodeIds);
    }

    const float margin = 0.1f, planHorizon =30.0f;
    TebLocalPlanner planner(margin, planHorizon);
    planner.init(&validity_local_);

    std::vector<Vec3> globalRoute = loadRouteCSV(args.RouteCSV);
    if (globalRoute.size() < 2) {
        std::cerr << "Route CSV had fewer than 2 points -- nothing to simulate.\n";
        return 1;
    }

    RoutePlayback playback;
    playback.build(globalRoute);

    UIState ui;
    Camera overviewCam = computeCamera(globalRoute, args.WindowW, args.WindowH);
    ui.cam = overviewCam;
    ui.obstacleHalfSize = args.ObstacleHalfSize;

    // Extracted once, in world space -- camera-independent. Re-projected to
    // pixels every frame below, for whichever camera (overview or follow)
    // is currently active.
    std::vector<PolygonWorld> drivablePolygonsWorld = buildDrivablePolygonsWorld(router.graph());

    const std::string winName = "TEB local planner -- live test";
    cv::namedWindow(winName, cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback(winName, onMouse, &ui);

    
    VehicleState vs{};
    vs.x = globalRoute[0].X; vs.y = globalRoute[0].Y;
    vs.theta = atan2(globalRoute[1].Y - globalRoute[0].Y, globalRoute[1].X - globalRoute[0].X);

    // Persistent, monotonically-increasing arc-length progress. Deliberately
    // NOT re-derived via a nearest-point search each frame: doing that
    // snaps to the nearest discrete route point and silently discards
    // whatever fractional progress was made within that point-to-point gap
    // -- if a frame's advancement (speed*dt) is smaller than the point
    // spacing, the vehicle never becomes closer to the next point than the
    // current one, and gets stuck re-deriving the exact same baseline
    // forever. Tracking it as a plain persistent scalar avoids that
    // entirely.
    // double sGlobal = 0.0;

    PurePursuitConfig ppCfg;
    ppCfg.steer_sign = 1.0;
    PurePursuitController pp(ppCfg);


    double speed = args.Speed;    // [m/s]
    bool paused = false;
    bool followMode = true;       // start zoomed on the vehicle -- that's almost always what you want when testing
    double followRadius = args.FollowRadius;
    auto lastTime = std::chrono::steady_clock::now();

    std::cout << "\nControls: left-click add obstacle, right-click remove nearest,\n"
                 "  c=clear, space=pause, r=reset, v=toggle follow/overview,\n"
                 "  [ / ] = shrink/grow follow radius, +/-=speed, q/ESC=quit\n\n";

    std::vector<Vec3> routeCopy = globalRoute;
    double currentSpeed = 0.0;

    std::vector<Vec3> trajectoryHistory;
    
    while (true) {
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - lastTime).count();
        dt = std::min(dt, 0.1); // clamp in case a frame stalls (window drag etc.)
        lastTime = now;

        // Fresh copy every frame, replanned from the pristine global route --
        // mirrors onObstacles()'s `pathCopy = globalPath_` rather than
        // compounding detours frame over frame. Anchored at the vehicle's
        // CURRENT pose (wherever the previous frame's advancement left it).
        
        ADComms::lidarObstPolyMsg msg = buildObstacleMsg(ui.obstacles);
        const auto planStart = std::chrono::steady_clock::now();
        bool localOk = planner.localPlan(&msg, vs, routeCopy);
        const auto planEnd = std::chrono::steady_clock::now();
        const double planMs = std::chrono::duration<double, std::milli>(planEnd - planStart).count();



        // Only prints when there's actually something to optimize (obstacles
        // present) -- otherwise localPlan() early-returns near-instantly
        // every single frame and this would just spam the console at
        // whatever the render loop's frame rate is.
        if (msg.N > 0) {
            std::cout << "[localPlan] ok=" << localOk << " optimize time=" << planMs << " ms\n";
        }

        if (!paused) {
            
            std::vector<Vec3> localWindow = extractLocalWindow(routeCopy, vs, planHorizon);
            pp.setTrajectory(localWindow);
            PurePursuitOutput out = pp.step(vs, currentSpeed);

            if (out.goal_reached) {
                currentSpeed = 0.0;
            } else {
                currentSpeed = out.speed_mps;
                // Standard bicycle model -- wheelbase_m matches what the
                // controller's OWN steering law just used, so there's no
                // mismatch between what steer_angle_rad was computed for
                // and what actually moves the vehicle.
                vs.x     += currentSpeed * std::cos(vs.theta) * dt;
                vs.y     += currentSpeed * std::sin(vs.theta) * dt;
                vs.theta += (currentSpeed / ppCfg.wheelbase_m) * std::tan(out.steer_angle_rad) * dt;
            }
 
            Vec3 histPoint{}; histPoint.X = static_cast<float>(vs.x); histPoint.Y = static_cast<float>(vs.y);
            histPoint.Dir = 1; histPoint.V = 0; histPoint.Curvature = 0;
            trajectoryHistory.push_back(histPoint);
            if (trajectoryHistory.size() > planHorizon) {
                trajectoryHistory.erase(trajectoryHistory.begin());
            }


        }

        // Follow camera recomputed every frame (it depends on vehicle
        // position); overview camera is static, computed once above.
        ui.cam = followMode ? computeFollowCamera(vs.x, vs.y, followRadius, args.WindowW, args.WindowH)
                             : overviewCam;

        std::vector<Polygon> drivablePolygons = projectPolygons(drivablePolygonsWorld, ui.cam);

        // ---- draw ----
        cv::Mat img(ui.cam.height, ui.cam.width, CV_8UC3, cv::Scalar(30, 30, 32));
        drawDrivablePolygons(img, drivablePolygons);
        drawPath(img, globalRoute, ui.cam, cv::Scalar(180, 120, 40), 1);      // global path, thin
        drawPath(img, routeCopy,   ui.cam, cv::Scalar(40, 220, 220), 2);     // current (possibly replanned) path
        drawPath(img, trajectoryHistory, ui.cam, cv::Scalar(220, 0,0), 2);
        drawObstacles(img, ui.obstacles, ui.cam, &validity_local_, margin);
        drawVehicle(img, vs, ui.cam);

        std::ostringstream hud;
        hud << (paused ? "[PAUSED] " : "") << (followMode ? "[FOLLOW r=" + std::to_string((int)followRadius) + "m] " : "[OVERVIEW] ")
            << "speed=" << std::fixed << std::setprecision(1) << currentSpeed
            << " m/s " << " m  obstacles=" << ui.obstacles.size()
            << "  localPlan=" << (localOk ? "ok" : "FAILED");
        cv::putText(img, hud.str(), cv::Point(12, 24), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                    cv::Scalar(230, 230, 230), 1, cv::LINE_AA);

        cv::imshow(winName, img);
        int key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 27) break;
        else if (key == ' ') paused = !paused;
        else if (key == 'c') ui.obstacles.clear();
        else if (key == 'r') {
            vs.x = globalRoute[0].X; vs.y = globalRoute[0].Y;
            vs.theta = atan2(globalRoute[1].Y - globalRoute[0].Y, globalRoute[1].X - globalRoute[0].X);
            currentSpeed = 0.0;
            trajectoryHistory.clear();

        }
        else if (key == 'v') followMode = !followMode;
        else if (key == '[') followRadius = std::max(5.0, followRadius - 5.0);
        else if (key == ']') followRadius += 5.0;
        else if (key == '+' || key == '=') speed += 0.5;
        else if (key == '-' || key == '_') speed = std::max(0.0, speed - 0.5);
    }
    return 0;
}