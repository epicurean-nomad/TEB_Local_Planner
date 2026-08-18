#include <unistd.h>


#include "../aux/ValidityCheckLocal.h"
#include "lidarObstPolyMsg.hpp"
#include "utils.hpp"
#include "../aux/Point3d.h"
#include "hyperedges.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam2d/vertex_se2.h>



struct TebPose{
    double x,y,theta;
    TebPose() : x(0.0),y(0.0),theta(0.0){}
    TebPose(double x_,double y_,double theta_) : x(x_),y(y_),theta(theta_){}
};

struct TebObstacle{
    Vec2 pos;
    float radius;
    TebObstacle(Vec2 p, float r) : pos(p), radius(r){};
};


struct TebConfig {
    // ---- vehicle limits ----
    // Sourced directly from the same constants your velocity profiler uses
    // (utils.hpp), so the two stay in agreement and tuning lives in one place.
    double max_vel          = MAX_SPEED;          // [m/s] forward
    double max_vel_reverse  = MAX_SPEED_REVERSE;   // [m/s] reverse
    double max_omega        = 1.0;                 // [rad/s] no direct analogue in utils.hpp; tune to your steering-rate limit
    double max_accel        = MAX_ACC;             // [m/s^2] speeding up
    double max_decel        = MAX_DEC;             // [m/s^2] slowing down (asymmetric from max_accel)
    double max_lat_acc      = MAX_LAT_ACC;         // [m/s^2] lateral-accel budget coupling curvature <-> speed
    double min_turn_radius  = WHEELBASE / std::tan(MAX_STEER_ANGLE_RAD);   // [m] steering-angle limit; matches the RS/RRT wheelbase-style radius used elsewhere
 
    // ---- optimization weights == information (1/variance) on each edge ----
    double w_obstacle    = 100.0;
    double w_velocity    = 8.0;
    double w_accel       = 4.0;
    double w_kinematics  = 500.0;    // non-holonomic (no-sideways-slip) term - should dominate
    double w_curvature   = 100.0;
    double w_viapoint    = 1.5;      // stay-near-reference pull, kept low
    double w_time        = 0.2;      // mild time-optimality pressure
    double w_forward_drive = 100.0;
    double w_corridor = 200;

    double sample_spacing = 0.4;
    double forward_drive_eps_frac = 0.5;
 

    double dt_min        = 0.05;
    double dt_max        = 1.0;
    size_t min_samples   = 6;
    size_t max_samples   = 80;
 
    // ---- solver ----
    // Empirically tuned: a representative single-obstacle detour over a
    // ~30-pose band needed noticeably more than a token number of LM
    // iterations to fully converge (each outer pass rebuilds a fresh g2o
    // graph -- see solveOnce() -- so Levenberg-Marquardt's damping factor
    // re-adapts from scratch every outer pass rather than carrying over;
    // that costs a few iterations each time). Cheap to afford: even the
    // full 6*30=180 LM iterations run well under a millisecond for band
    // sizes this small.
    int    outer_iterations = 5;     // autoResize + rebuild-graph-and-solve cycles
    int    inner_iterations = 6;    // LM iterations per outer cycle

    double side_hysteresis = 0.85;
 
    // How close to the target clearance counts as "feasible enough" after
    // optimization (which won't converge to the target exactly, since it's
    // a soft penalty, not a hard constraint). 1.0 = must fully meet target.

    double corridor_margin = 0.2;


};






// Clamp the speed by MAX and MIN values
inline double targetSpeedAt(const Vec3& p, bool isReverse, const TebConfig& cfg) {
    double floor_speed = isReverse ? EFFECTIVE_MIN_REV : EFFECTIVE_MIN;
    double vmax        = isReverse ? cfg.max_vel_reverse : cfg.max_vel;
    if (p.V > floor_speed) return std::min((double)p.V, vmax);
    return floor_speed;
}







inline double estimateHeadingFromPath(const std::vector<Vec3>& route, int i, int i0, int i1) {
    int a = i, b = i;
    if (i < i1)      b = i + 1;
    else if (i > i0) a = i - 1;
    else             return 0.0; // single-point window; caller should avoid this
 
    double dx = route[b].X - route[a].X;
    double dy = route[b].Y - route[a].Y;
    if (std::fabs(dx) < 1e-9 && std::fabs(dy) < 1e-9) return 0.0;
    return std::atan2(dy, dx);
}




inline double wrapToPi(double a) {
    while (a > M_PI)  a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}
 
// Shortest signed angular difference a - b, wrapped to (-pi, pi].
inline double angleDiff(double a, double b) {
    return wrapToPi(a - b);
}



class TimedElasticBand{
    
    public:
    
        std::vector<TebPose> pose; // size N
        std::vector<float> dt;  // size N-1
        std::vector<Vec2> refPts;
        std::vector<double> arclengths;
        std::vector<double> leftMargin;
        std::vector<double> rightMargin;
        std::vector<Vec2> normals;
        bool isReverse = false;

        void initFromRef(const std::vector<Vec3>& ref, VehicleState vs,
                     int i0, int i1, const TebConfig& cfg, const ValidityCheckLocal* validity_)
        {
            pose.clear(); dt.clear(); refPts.clear();
            isReverse = (ref[i0].Dir == 0);

            // --- 1. seed polyline: vehicle pose, then route[i0..i1], duplicates dropped
            std::vector<Vec2>   poly;
            std::vector<double> polyV;
            poly.push_back(Vec2(vs.x, vs.y));
            polyV.push_back(targetSpeedAt(ref[i0], isReverse, cfg));

            const double gear = isReverse ? -1.0 : 1.0;
            const double hx = gear * std::cos(vs.theta), hy = gear * std::sin(vs.theta);

            for (int i = i0; i <= i1; ++i) {
                Vec2 p(ref[i].X, ref[i].Y);
                if((p.X - vs.x)*hx + (p.Y - vs.y)*hy < 0.3) continue;
                if (Vec2::Dist(poly.back(), p) < 1e-3) continue;
                poly.push_back(p);
                polyV.push_back(targetSpeedAt(ref[i], isReverse, cfg));
            }
            if (poly.size() < 2) return;   // caller must treat empty pose[] as failure

            // --- 2. cumulative arclength
            std::vector<double> s(poly.size(), 0.0);
            for (size_t k = 1; k < poly.size(); ++k)
                s[k] = s[k-1] + Vec2::Dist(poly[k-1], poly[k]);
            const double L = s.back();
            if (L < 1e-3) return;

            // --- 3. fixed N for this solve
            const int N = std::clamp(static_cast<int>(std::llround(L / cfg.sample_spacing)) + 1,
                                     static_cast<int>(cfg.min_samples),
                                     static_cast<int>(cfg.max_samples));

            // --- 4. resample at uniform arclength
            pose.resize(N);
            refPts.resize(N);
            normals.resize(N);
            arclengths.resize(N);
            leftMargin.resize(N);
            rightMargin.resize(N);
            std::vector<double> seedV(N);
            size_t seg = 0;
            for (int k = 0; k < N; ++k) {
                const double target = (L * k) / (N - 1);
                while (seg + 2 < poly.size() && s[seg + 1] < target) ++seg;
                const double segLen = std::max(1e-9, s[seg + 1] - s[seg]);
                const double u = std::clamp((target - s[seg]) / segLen, 0.0, 1.0);
                const double x = poly[seg].X + u * (poly[seg + 1].X - poly[seg].X);
                const double y = poly[seg].Y + u * (poly[seg + 1].Y - poly[seg].Y);
                pose[k]   = TebPose(x, y, 0.0);
                refPts[k] = Vec2(x, y);
                seedV[k]  = polyV[seg] + u * (polyV[seg + 1] - polyV[seg]);
            }

            // --- 5. headings from the resampled geometry (central diff, one-sided at ends)
            for (int k = 0; k < N; ++k) {
                const int a = (k == 0)     ? 0     : k - 1;
                const int b = (k == N - 1) ? N - 1 : k + 1;
                const double dx = pose[b].x - pose[a].x;
                const double dy = pose[b].y - pose[a].y;
                if (dx*dx + dy*dy > 1e-12) pose[k].theta = std::atan2(dy, dx);
                // In reverse gear the body heading is opposite the direction of travel.
                if (isReverse) pose[k].theta = wrapToPi(pose[k].theta + M_PI);
            }
            pose[0].theta = vs.theta;   // pinned start uses the vehicle's real heading

            // --- 6. dt from seed speeds
            dt.resize(N - 1);
            for (int k = 0; k + 1 < N; ++k) {
                const double d = Vec2::Dist(Vec2(pose[k].x, pose[k].y),
                                            Vec2(pose[k+1].x, pose[k+1].y));
                const double v = std::max(0.05, 0.5 * (seedV[k] + seedV[k+1]));
                dt[k] = static_cast<float>(std::clamp(d / v, cfg.dt_min, cfg.dt_max));
            }


            //Calculate arclengths

            for(int k=0; k < N; k++){
                if(k>0){
                    arclengths[k] = arclengths[k-1] + Vec2::Dist(Vec2(pose[k].x, pose[k].y),
                                            Vec2(pose[k-1].x, pose[k-1].y));
                }
                normals[k] = Vec2(-std::sin(pose[k].theta), std::cos(pose[k].theta));
            }


            //Compute left and right margins

            for (int k = 0; k < N; ++k) {
                const bool centreOk = !validity_
                    || validity_->isVehicleFeasible(pose[k].x, pose[k].y, pose[k].theta);

                for (int sgn : {+1, -1}) {
                    double lo = 0.0, hi = 6.0;      // lo known feasible, hi to be tested
                    auto feasibleAt = [&](double d) {
                        if (!validity_) return true;
                        return validity_->isVehicleFeasible(pose[k].x + sgn * d * normals[k].X,
                                                            pose[k].y + sgn * d * normals[k].Y,
                                                            pose[k].theta);
                    };

                    if (feasibleAt(hi)) {
                        lo = hi;                    // corridor wider than the probe limit
                    } else {
                        for (int it = 0; it < 8; ++it) {   // 6.0 / 2^8 ~= 2 cm resolution
                            const double mid = 0.5 * (lo + hi);
                            if (feasibleAt(mid)) lo = mid; else hi = mid;
                        }
                    }

                    // If the seed pose is already outside the region, the corridor
                    // term must not pin the band there -- give it room to escape.
                    if (!centreOk) lo = 6.0;
                    else           lo = std::max(0.0, lo - cfg.corridor_margin);


                    if (sgn > 0) leftMargin[k]  = lo;   // normals[] is the LEFT normal
                    else         rightMargin[k] = lo;
                }
            }
        }
};



// =========================================================================
// TebLocalPlanner
// -------------------------------------------------------------------------
// Orchestrates one local-replanning cycle on top of a TimedElasticBand:
//
//   1. localPlan()        -- entry point called once per planning tick.
//        gatherObstacles() -> pathConflicts()? -> findHorizonEnd()
//        -> TimedElasticBand::initFromRef()   (seed the band, see hyperedges.hpp)
//        -> optimizeBand()                    (deform it against obstacles)
//        -> spliceIntoRoute()                 (write the result back)
//
//   2. optimizeBand() tries both lateral "sides" of a blocking obstacle
//      (side = -1 / +1) since the hinge-style obstacle cost alone has no
//      preference for which way to detour, and re-solves the g2o graph
//      (solveOnce(), defined in hyperedges.hpp/this file below) across
//      several outer iterations, re-densifying/resizing the band each pass.
//
// Only the region of the route between the vehicle's current index and the
// planning horizon is ever touched -- everything outside that window is
// left untouched by spliceIntoRoute().
// =========================================================================

class TebLocalPlanner{
    public:
        explicit TebLocalPlanner(TebConfig cfg = TebConfig())
         : cfg_(cfg){}

        void init(const ValidityCheckLocal* v, float plan_horizon, float margin){
            validity_ = v;
            plan_horizon_ = plan_horizon;
            margin_       = margin;
        }


        /* 
        Single entry point, called once per planning tick with the latest
        lidar obstacle message, vehicle state, and the (mutable) global
        route. Cheap early-outs short-circuit the expensive TEB machinery
        whenever a local replan clearly isn't needed:
          - no validity checker wired up, or route too short to plan over;
          - no obstacles within plan_horizon_ of the vehicle at all;
          - obstacles present, but none of them actually sit within
            minClearance of the existing route (pathConflicts);
          - a degenerate horizon (e.g. route ends right at the vehicle, or
            a gear-direction change sits at the very first index).
        Returns true when the route is left in a usable state (either
        untouched because no replan was needed, or successfully replaced
        in-place); returns false only when a replan was attempted and
        failed to find a feasible band -- callers should treat that as
        "do not trust this route segment".
    
        NOTE: desired_speed below is currently unused dead code left over
        from an earlier version -- targetSpeedAt()/the velocity profiler
        are what actually drive seeding speed now (see initFromRef()).
        */
        bool localPlan(const ADComms::lidarObstPolyMsg* msg, const VehicleState vs, std::vector<Vec3>& route, int* outI0 = nullptr, int* outI1 = nullptr){
            
            //If route size is too small or route is not valid, return
            if(!validity_ || route.size() < 2) return true;

            const std::vector<TebObstacle> obs_vec = gatherObstacles(msg, vs);

            //If there are no obstacles, return
            

            int vehicleIDx = findClosestIndex(route, vs);

            int i1 = findHorizonEnd(route, vehicleIDx, obs_vec);


            if (outI0) *outI0 = vehicleIDx;
            if (outI1) *outI1 = i1;




            
            if(obs_vec.empty()) return true;
            //If the obstacles are not in the path, return
            if(!pathConflicts(route, obs_vec, vehicleIDx, i1))return true;

            //If the horizon is degenerate, return
            if(i1 <= vehicleIDx) return true;

            TimedElasticBand teb;
            double desired_speed = 10.0;

            teb.initFromRef(route, vs, vehicleIDx, i1, cfg_, validity_);
            if (teb.pose.size() < 3) {
                std::cout << "[TEB] degenerate seed band -- skipping local re-plan\n";
                return false;
            }


            bool feasible = optimizeBand(teb, obs_vec);
            if(!feasible){
                std::cout << "[TEB] local re-plan failed to find a feasible band\n";
                return false;
            } 

            spliceIntoRoute(route, vehicleIDx, i1, teb);
            return true;


        }
    
    private:
        



        /* 
        Converts the raw lidar obstacle-polygon message into TebObstacles
        (circle approximations: centroid + bounding radius), keeping only
        obstacles within plan_horizon_ of the vehicle. Each polygon is
        assumed to be a 4-corner box, stored flattened as
        [x0,y0,x1,y1,x2,y2,x3,y3]; the radius is approximated as the
        corner-to-centroid distance (half the box diagonal), so it's a
        conservative (slightly oversized) circular bound on the true box.
        Hard-capped at 200 obstacles regardless of msg->N to bound the
        cost of both this loop and the O(poses x obstacles) edge-building
        in solveOnce().
        */
        std::vector<TebObstacle> gatherObstacles(const ADComms::lidarObstPolyMsg* msg, const VehicleState& vs)
        {
            std::vector<TebObstacle> obstacles;

            for (int i = 0; i < msg->N && i < 200; ++i) {

             
                 // Centroid from the 4 polygon corners stored as [x0,y0,x1,y1,...,x3,y3]
                 double ox = (msg->pts[i][0] + msg->pts[i][1] +
                              msg->pts[i][2] + msg->pts[i][3]) * 0.25;
                 double oy = (msg->pts[i][4] + msg->pts[i][5] +
                              msg->pts[i][6] + msg->pts[i][7]) * 0.25;
             
                 double diag_x = ox - msg->pts[i][0];
                 double diag_y = oy - msg->pts[i][4];
                 float r = std::sqrt(diag_x*diag_x + diag_y*diag_y);
             
                 // Skip if outside planning horizon
                 double dx = ox - vs.x, dy = oy - vs.y;
                 if (std::sqrt(dx*dx + dy*dy) > plan_horizon_ - (plan_horizon_/3.0 + margin_)) continue;

                 obstacles.push_back(TebObstacle(Vec2(ox,oy), r));
            }
            return obstacles;
        }

        int findClosestIndex(const std::vector<Vec3> route, VehicleState vs) const
        {
            int best = 0;
            double best_d = 1e9;

            for(int i=0; i<route.size(); i++){
                
                double dx = vs.x - route[i].X;
                double dy = vs.y - route[i].Y;
                double dist = dx*dx + dy*dy;
                if(dist < best_d){
                    best_d = dist;
                    best = i;
                }
            }
            return best;
        }

        bool pathConflicts(const std::vector<Vec3> route, std::vector<TebObstacle> obstacles, int vehicle_id, int i1) const
        {
            
            for(TebObstacle& o : obstacles){
                
                double min_d = safetyClearance(o);
                for(int i=vehicle_id; i <= i1 && i<route.size(); i++){
                    double dx = route[i].X - o.pos.X, dy = route[i].Y - o.pos.Y;
                    if (std::sqrt(dx * dx + dy * dy) < min_d) return true;

                }   
            }
            return false;
        }

        int findHorizonEnd(const std::vector<Vec3> route, int i0, const std::vector<TebObstacle>& obstacles)
        {
            
            int i1 = i0;
            double accum = 0.0;
            for(int i=i0; i+1<route.size(); ++i){
                
                if(route[i].Dir != route[i0].Dir) {i1 = i; break;}

                accum += Vec2::Dist(Vec2(route[i].X,   route[i].Y),
                                    Vec2(route[i+1].X, route[i+1].Y));
                if(accum >= plan_horizon_)  {return i1 = i+1;}
            }
            i1 = static_cast<int>(route.size()) - 1;

            const double R = std::max(cfg_.min_turn_radius, 0.1);
            bool extended = true;
            int guard = 0;
            while (extended && guard++ < 200) {
                extended = false;
                for (const TebObstacle& o : obstacles) {
                    const double A = minClearance(o);
                    const double Lret = 4.0 * std::sqrt(A * R) + 2.0 * o.radius;
                    // arclength from route[i1] back to the obstacle's closest route point
                    double d = Vec2::Dist(o.pos, Vec2(route[i1].X, route[i1].Y));
                    if (d < Lret) {
                        double accum = 0.0;
                        int j = i1;

                        int i1_cap = i1;
                        { 
                            double a = 0.0;
                            for (int i = i0; i + 1 < (int)route.size() && a < 2.0 * plan_horizon_; ++i) {
                                a += Vec2::Dist(Vec2(route[i].X, route[i].Y), Vec2(route[i+1].X, route[i+1].Y));
                                i1_cap = i + 1;
                            } 
                        }
                        while (j + 1 < (int)route.size() && j < i1_cap &&  accum < (Lret - d)) {
                            if (route[j].Dir != route[i0].Dir) break;   // never cross a gear change
                            accum += Vec2::Dist(Vec2(route[j].X, route[j].Y),
                                                Vec2(route[j+1].X, route[j+1].Y));
                            ++j;
                        }
                        if (j > i1) { i1 = j; extended = true; }
                    }
                }
            }
            return i1;


        }

        static double projectOntoSegment(const Vec2& p, const Vec2& a, const Vec2& b)
        {
            double dist = pow(b.X - a.X,2) + pow(b.Y - a.Y,2);
            double t = ((p.X - a.X)*(b.X - a.X) + (p.Y - a.Y)*(b.Y - a.Y))/dist;
            return  std::max(0.0, std::min(1.0, dist < 1e-9 ? 0.0 : t)); 
        }

        bool optimizeBand(TimedElasticBand& teb, const std::vector<TebObstacle>& obstacles)
        {
            const TimedElasticBand initial = teb;
            if (initial.pose.size() < 3) return false;

            TimedElasticBand best;
            double bestScore = std::numeric_limits<double>::max();
            int    bestSide  = 0;
            bool   found     = false;

            const int firstSide = (last_side_ != 0) ? last_side_ : -1;
            for (int side : { firstSide, -firstSide }) {
                TimedElasticBand attempt = initial;
                densifyNearObstacles(attempt, obstacles, side);

                const double chi2 = solveBand(attempt, obstacles);
                if (!std::isfinite(chi2)) continue;
                if (!checkFeasible(attempt, obstacles)) continue;

                const double score = chi2 * ((side == last_side_) ? cfg_.side_hysteresis : 1.0);
                if (score < bestScore) {
                    bestScore = score; best = attempt; bestSide = side; found = true;
                }
            }

            if (found) { teb = best; last_side_ = bestSide; }
            return found;
        }

        void retangent(TimedElasticBand& teb) const {
            const size_t N = teb.pose.size();
            if (N < 2) return;
            for (size_t k = 1; k + 1 < N; ++k) {          // leave pinned endpoints alone
                const double dx = teb.pose[k+1].x - teb.pose[k-1].x;
                const double dy = teb.pose[k+1].y - teb.pose[k-1].y;
                if (dx*dx + dy*dy > 1e-12) teb.pose[k].theta = std::atan2(dy, dx);
            }
        }

        /*
        Pre-optimization seeding step: wherever a segment of the straight-
        line band seed passes closer than minClearance to an obstacle
        (measured via projectOntoSegment's closest-approach point, not
        just the endpoints), insert an extra pose at that closest-approach
        point, nudged perpendicular to the local heading by `nudge` in the
        direction given by `side` (-1 or +1). `nudge` is at least 30% of
        the required clearance, or however much extra clearance is
        actually missing, whichever is larger -- so the inserted pose is a
        plausible starting point for a real detour rather than a token
        perturbation. The dt for the split segment is halved and
        duplicated for the new sub-segment so total time is preserved.
        Runs to a fixed point (guarded at 500 passes to avoid a runaway
        loop) since inserting a pose can reveal a new closest-approach
        violation on either of the two new sub-segments; stops early once
        max_samples is reached. This only shapes the INITIAL guess handed
        to solveOnce() -- EdgeObstacle's hinge cost is what actually
        enforces clearance during optimization.

        */
        void densifyNearObstacles(TimedElasticBand& teb,
                          const std::vector<TebObstacle>& obstacles, int side) const
        {
            const size_t N = teb.pose.size();
            if (N < 3) return;

        
            std::vector<double> off(N, 0.0);
            const double kappaMax = 1.0 / std::max(cfg_.min_turn_radius, 0.1);
        
            for (const TebObstacle& o : obstacles) {
                const double target = 1.15 * minClearance(o);
            
                
                //Find closest point to the obstacle
                size_t kc = 0; double best = 1e18;
                for (size_t k = 0; k < N; ++k) {
                    const double d = Vec2::Dist(o.pos, Vec2(teb.pose[k].x, teb.pose[k].y));
                    if (d < best) { best = d; kc = k; }
                }
                if (best >= target) continue;                    // if closes obstacle is not a threat
                if (kc == 0 || kc + 1 >= N) continue;
                

                // Signed lateral position of the obstacle w.r.t. the band.
                const double rx = o.pos.X - teb.pose[kc].x, ry = o.pos.Y - teb.pose[kc].y;  //vector from nearest path point to nearest obstacle
                const double lat   = rx * teb.normals[kc].X + ry * teb.normals[kc].Y;                             // dot product between left normal at kc and (rx,ry) 
                const double along = std::sqrt(std::max(0.0, best*best - lat*lat));         // 
                const double need  = std::sqrt(std::max(0.0, target*target - along*along));
                const double peak  = (side > 0) ? (lat + need) : (lat - need);
            
                // Window wide enough that the raised cosine stays inside the steering limit:
                // max curvature of  y(u) = A/2 (1+cos(pi u))  is  A pi^2 / (2 W^2).
                const double W = std::max({ 2.0 * target,
                                            std::sqrt(std::fabs(peak) * M_PI * M_PI / (2.0 * kappaMax)),
                                            3.0 });
                
                for (size_t k = 0; k < N; ++k) {
                    const double u = (teb.arclengths[k] - teb.arclengths[kc]) / W;
                    if (std::fabs(u) >= 1.0) continue;
                    const double c = peak * 0.5 * (1.0 + std::cos(M_PI * u));
                    if (std::fabs(c) > std::fabs(off[k])){
                        off[k] = c;
                    }    
                }
            }

            double peakAbs = 0.0;
            for (size_t k = 0; k < N; ++k) peakAbs = std::max(peakAbs, std::fabs(off[k]));
            const double L = std::max(1.0, M_PI * std::sqrt(peakAbs / (2.0 * kappaMax)));

            for (size_t k = 1; k + 1 < N; ++k) {
                if (off[k] == 0.0) continue;
                auto ramp = [&](double d) {
                    const double t = std::clamp(d / L, 0.0, 1.0);
                    return 0.5 * (1.0 - std::cos(M_PI * t));
                };
                off[k] = std::clamp(off[k], -teb.rightMargin[k], teb.leftMargin[k]);
                off[k] *= std::min(ramp(teb.arclengths[k]), ramp(teb.arclengths[N-1] - teb.arclengths[k]));
            }
        
            for (size_t k = 1; k + 1 < N; ++k) {                 // pinned endpoints untouched
                teb.pose[k].x += teb.normals[k].X * off[k];
                teb.pose[k].y += teb.normals[k].Y * off[k];
            }
            retangent(teb);
        }

        double clearanceTargetAt(const TebObstacle& o, double s_k) const {
            const double hard = safetyClearance(o);
            const double soft = minClearance(o);
            const double runup = M_PI * std::sqrt(soft / (2.0 / std::max(cfg_.min_turn_radius, 0.1)));
            const double t = std::clamp(s_k / std::max(runup, 1e-3), 0.0, 1.0);
            return hard + (soft - hard) * 0.5 * (1.0 - std::cos(M_PI * t));
        }


        // Final hard check after optimization. Checks both poses AND segment
        // closest-approach points (not just poses) against every obstacle,
        bool checkFeasible(const TimedElasticBand& teb, const std::vector<TebObstacle>& obstacles) const {
            
            //Check if every pose is inside drivable region and sufficiently away from obstacles
            for (int k=0; k+1 < teb.pose.size(); k++) {
                const TebPose& p = teb.pose[k];
                if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.theta)) return false;
                if (validity_ && !validity_->isVehicleFeasible(p.x, p.y, p.theta)){
                    std::cout << "[TEB checkFeasible] FAIL: pose " << k << " outside drivable region\n";
                    return false;
                } 
                for (const TebObstacle& o : obstacles) {
                    double dx = p.x - o.pos.X, dy = p.y - o.pos.Y;
                    double d = std::sqrt(dx * dx + dy * dy);

                    if (d < safetyClearance(o)){
                        std::cout << "[TEB checkFeasible] FAIL: pose " << k << " clearance "
                                  << d << " < " << safetyClearance(o) << "\n";
                        return false;
                    } 
                }
            }

            //Check if the edge between every two poses is outside obstacle
            for (size_t i = 0; i + 1 < teb.pose.size(); ++i) {
                Vec2 p0(teb.pose[i].x, teb.pose[i].y);
                Vec2 p1(teb.pose[i + 1].x, teb.pose[i + 1].y);
                for (const TebObstacle& o : obstacles) {
                    double t = projectOntoSegment(o.pos, p0, p1);
                    Vec2 proj(p0.X + t * (p1.X - p0.X), p0.Y + t * (p1.Y - p0.Y));
                    double d = Vec2::Dist(o.pos, proj);

                    if (d < safetyClearance(o)){
                        std::cout << "[TEB checkFeasible] FAIL: edge " << i << " clearance "
                                  << d << " < " << safetyClearance(o) << "\n";
                        return false;
                    }
                }
            }

            
            //Check for any direction changes in the route
            const double kMinStep = 0.05;
            int lastSign = 0;
            for (size_t i = 0; i + 1 < teb.pose.size(); ++i) {
                const double dx  = teb.pose[i+1].x - teb.pose[i].x;
                const double dy  = teb.pose[i+1].y - teb.pose[i].y;
                const double dth = angleDiff(teb.pose[i+1].theta, teb.pose[i].theta);
                const double th  = teb.pose[i].theta + 0.5 * dth;
                const double s   = dx * std::cos(th) + dy * std::sin(th);
                if (std::fabs(s) < kMinStep) continue;
                const int sign = (s > 0.0) ? 1 : -1;
                if (lastSign != 0 && sign != lastSign) {
                    std::cout << "[TEB checkFeasible] FAIL: direction change at pose " << i << "\n";
                    return false;
                }
                lastSign = sign;
            }


            
            const double kappaMax = 1.0 / std::max(cfg_.min_turn_radius, 0.1);
            const double kappaTol = 1.5 * kappaMax;   // allow modest hinge overshoot
            const double kMinArc  = 0.10;              // below this, Δθ/Δs is noise
            for (size_t i = 0; i + 1 < teb.pose.size(); ++i) {
                const double dx = teb.pose[i+1].x - teb.pose[i].x;
                const double dy = teb.pose[i+1].y - teb.pose[i].y;
                const double ds = std::sqrt(dx*dx + dy*dy);
                if (ds < kMinArc) continue;
                const double dth   = angleDiff(teb.pose[i+1].theta, teb.pose[i].theta);
                const double kappa = std::fabs(dth) / ds;
                if (kappa > kappaTol) {
                    std::cout << "[TEB checkFeasible] FAIL: curvature at pose " << i
                              << " kappa=" << std::setprecision(4) << kappa << " > " << std::setprecision(4) << kappaTol
                              << " (R=" << std::setprecision(4) << 1.0/std::max(kappa,1e-6) << "m)\n";
                    return false;
                }
            }
            

            return true;
        }


        // ---------------------------------------------------------------
        // g2o graph construction + solve for one outer-iteration pass.
        // Rebuilt from scratch every call (band sizes are tens of poses, so
        // this is cheap) -- much simpler to reason about than incrementally
        // patching a persistent graph as obstacles/geometry change call to
        // call, and it's how the reference teb_local_planner practically
        // operates too (its optimizeTEB() rebuilds each planning cycle).
        // ---------------------------------------------------------------
        double solveBand(TimedElasticBand& teb,
                       const std::vector<TebObstacle>& obstacles) const
        {
            size_t N = teb.pose.size();
            

            //Initialize the solver

            g2o::SparseOptimizer optimizer;
            optimizer.setVerbose(false);
            using LinearSolverT = g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>;
            std::unique_ptr<LinearSolverT> linearSolver = std::make_unique<LinearSolverT>();
            std::unique_ptr<g2o::BlockSolverX> blockSolver  = std::make_unique<g2o::BlockSolverX>(std::move(linearSolver));
            g2o::OptimizationAlgorithmLevenberg* algorithm   = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));
            optimizer.setAlgorithm(algorithm);
        
            
            
            //Initializing the set of optimization variables
            std::vector<g2o::VertexSE2*> poseVerts(N, nullptr);
            std::vector<VertexTimeDiff*> dtVerts(teb.dt.size(), nullptr);
        
            int id = 0;
            for (size_t k = 0; k < N; ++k) {
                g2o::VertexSE2* v = new g2o::VertexSE2();
                v->setId(id++);
                v->setEstimate(g2o::SE2(teb.pose[k].x, teb.pose[k].y, teb.pose[k].theta));
                if (k == 0 || k + 1 == N) v->setFixed(true); // pin both window endpoints
                optimizer.addVertex(v);
                poseVerts[k] = v;
            }

            for (size_t i = 0; i < teb.dt.size(); ++i) {
                VertexTimeDiff* v = new VertexTimeDiff();
                v->setId(id++);
                v->setEstimate(teb.dt[i]);
                optimizer.addVertex(v);
                dtVerts[i] = v;
            }
            

            


            // Add Obstacle Edges
            for (size_t k = 0; k < N; ++k) {
                for (const TebObstacle& o : obstacles) {
                    
                    double dx = teb.pose[k].x - o.pos.X, dy = teb.pose[k].y - o.pos.Y;
                    double d = dx*dx + dy*dy;
                    double gate = 2.5 * minClearance(o);
                    if(d < gate * gate){
                        
                        EdgeObstacle* e = new EdgeObstacle();
                        e->obstacleX = o.pos.X;
                        e->obstacleY = o.pos.Y;
                        e->minClearance = clearanceTargetAt(o, teb.arclengths[k]);
                        e->setVertex(0, poseVerts[k]);
                        e->setInformation(Eigen::Matrix<double,1,1>::Identity() * cfg_.w_obstacle);
                        optimizer.addEdge(e);
                    }
                    
                }
                // Via-point: pull toward the NEAREST point on the original
                // (pre-optimization, resize-invariant) reference band -- a
                // full nearest-neighbor scan, not an index-matched lookup,
                // since autoResize/densify change how many poses there are.

                if (k < teb.refPts.size()) {
                    EdgeViaPoint* e = new EdgeViaPoint();
                    e->setMeasurement(Eigen::Vector2d(teb.refPts[k].X, teb.refPts[k].Y));
                    e->setVertex(0, poseVerts[k]);
                    e->setInformation(Eigen::Matrix2d::Identity() * cfg_.w_viapoint);
                    optimizer.addEdge(e);

                    EdgeCorridor* ecorr = new EdgeCorridor();
                    ecorr->refX     = teb.refPts[k].X;
                    ecorr->refY     = teb.refPts[k].Y;
                    ecorr->nx       = teb.normals[k].X;
                    ecorr->ny       = teb.normals[k].Y;
                    ecorr->leftMax  = teb.leftMargin[k];
                    ecorr->rightMax = teb.rightMargin[k];
                    ecorr->setVertex(0, poseVerts[k]);
                    ecorr->setInformation(Eigen::Matrix<double,1,1>::Identity() * cfg_.w_corridor);
                    optimizer.addEdge(ecorr);
                }
            }
            

            //Add 
            for (size_t i = 0; i + 1 < N; ++i) {
                EdgeKinematics* ek = new EdgeKinematics();
                ek->gearSign = teb.isReverse ? -1.0 : 1.0;
                ek->setVertex(0, poseVerts[i]);
                ek->setVertex(1, poseVerts[i + 1]);
                Eigen::Matrix2d infoK = Eigen::Matrix2d::Zero();
                infoK(0,0) = cfg_.w_kinematics;      // 100  — lateral slip
                infoK(1,1) = cfg_.w_forward_drive;   // add to TebConfig, start at 30
                ek->eps = cfg_.forward_drive_eps_frac * cfg_.sample_spacing;
                ek->setInformation(infoK);
                optimizer.addEdge(ek);
            
                EdgeCurvature* ec = new EdgeCurvature();
                ec->kappaMax = 1.0 / std::max(cfg_.min_turn_radius, 0.1);
                ec->setVertex(0, poseVerts[i]);
                ec->setVertex(1, poseVerts[i + 1]);
                ec->setInformation(Eigen::Matrix<double,1,1>::Identity() * cfg_.w_curvature);
                optimizer.addEdge(ec);
            
                EdgeVelocity* ev = new EdgeVelocity();
                ev->vmax      = teb.isReverse ? cfg_.max_vel_reverse : cfg_.max_vel;
                ev->omegaMax  = cfg_.max_omega;
                ev->maxLatAcc = cfg_.max_lat_acc;
                ev->setVertex(0, poseVerts[i]);
                ev->setVertex(1, poseVerts[i + 1]);
                ev->setVertex(2, dtVerts[i]);
                Eigen::Matrix2d infoV = Eigen::Matrix2d::Identity() * cfg_.w_velocity;
                ev->setInformation(infoV);
                // ev->setRobustKernel(new g2o::RobustKernelHuber());
                optimizer.addEdge(ev);
            
                EdgeTimeOptimal* et = new EdgeTimeOptimal();
                et->setVertex(0, dtVerts[i]);
                et->setInformation(Eigen::Matrix<double,1,1>::Identity() * cfg_.w_time);
                optimizer.addEdge(et);
            }
        
            for (size_t i = 0; i + 2 < N; ++i) {
                EdgeAcceleration* ea = new EdgeAcceleration();
                ea->maxAccel = cfg_.max_accel;
                ea->maxDecel = cfg_.max_decel;
                ea->setVertex(0, poseVerts[i]);
                ea->setVertex(1, poseVerts[i + 1]);
                ea->setVertex(2, poseVerts[i + 2]);
                ea->setVertex(3, dtVerts[i]);
                ea->setVertex(4, dtVerts[i + 1]);
                ea->setInformation(Eigen::Matrix<double,1,1>::Identity() * cfg_.w_accel);
                // ea->setRobustKernel(new g2o::RobustKernelHuber());
                optimizer.addEdge(ea);
            }
        
            if (!optimizer.initializeOptimization()) {
                std::cout << "[TEB] g2o initializeOptimization() failed\n";
                return std::numeric_limits<double>::quiet_NaN();
            }
            optimizer.optimize(cfg_.outer_iterations*cfg_.inner_iterations);
            optimizer.computeActiveErrors();
            const double chi2 = optimizer.activeChi2();

            for (size_t k = 0; k < N; ++k) {
                const auto& est = poseVerts[k]->estimate();
                teb.pose[k].x     = est.translation().x();
                teb.pose[k].y     = est.translation().y();
                teb.pose[k].theta = est.rotation().angle();
            }
            for (size_t i = 0; i < teb.dt.size(); ++i)
                teb.dt[i] = static_cast<float>(
                    std::clamp<double>(dtVerts[i]->estimate(), cfg_.dt_min, cfg_.dt_max));

            return chi2;
            
        }



        void spliceIntoRoute(std::vector<Vec3>& route, int i0, int i1, TimedElasticBand& teb)
        {

            const int gearDir = teb.isReverse ? 0 : 1;
 
            std::vector<Vec3> replacement;
            replacement.reserve(teb.pose.size());
            for (size_t i = 0; i < teb.pose.size(); ++i) {
                Vec3 p{}; 
                p.X = teb.pose[i].x;
                p.Y = teb.pose[i].y;
                p.Dir = gearDir;
                // V is advisory only: publishPath() unconditionally calls
                // computeCurvature()/computeVelocityProfile() on the full path
                // right after this, which overwrites V (and Curvature) from
                // geometry -- so an approximate seed here is fine.
                double v = 0.0;
                if (i + 1 < teb.pose.size()) {
                    double dx = teb.pose[i + 1].x - teb.pose[i].x;
                    double dy = teb.pose[i + 1].y - teb.pose[i].y;
                    v = std::sqrt(dx * dx + dy * dy) / std::max(teb.dt[i], 1e-3f);
                }
                p.V = v;
                replacement.push_back(p);
            }
            route.erase(route.begin() + i0, route.begin() + i1 + 1);
            route.insert(route.begin() + i0, replacement.begin(), replacement.end());

        }

        double safetyClearance(const TebObstacle& o) const {
            const double halfW = validity_ ? validity_->vehicleWidth() * 0.5 : 0.5;
            return o.radius + halfW;
        }

        double minClearance(const TebObstacle& o) const {
            double extra = validity_ ? validity_->vehicleWidth()/3 : 0.5;
            return safetyClearance(o) + margin_ + extra;    
        }

    
        float margin_, plan_horizon_;
        TebConfig cfg_;
        const ValidityCheckLocal* validity_ = nullptr;
        int last_side_ = 0;


};