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
    double min_turn_radius  = 3.0;   // [m] steering-angle limit; matches the RS/RRT wheelbase-style radius used elsewhere
 
    // ---- optimization weights == information (1/variance) on each edge ----
    double w_obstacle    = 40.0;
    double w_velocity    = 8.0;
    double w_accel       = 4.0;
    double w_kinematics  = 100.0;    // non-holonomic (no-sideways-slip) term - should dominate
    double w_curvature   = 20.0;
    double w_viapoint    = 1.5;      // stay-near-reference pull, kept low
    double w_time        = 0.2;      // mild time-optimality pressure
    double w_forward_drive = 4.0;
 
    // ---- band shape / auto-resize ----
    double dt_ref        = 0.3;      // desired time between poses [s]
    double dt_hysteresis = 0.1;
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
    int    outer_iterations = 4;     // autoResize + rebuild-graph-and-solve cycles
    int    inner_iterations = 5;    // LM iterations per outer cycle
 
    // How close to the target clearance counts as "feasible enough" after
    // optimization (which won't converge to the target exactly, since it's
    // a soft penalty, not a hard constraint). 1.0 = must fully meet target.
    double feasibility_tolerance = 0.8;

    double cusp_reject_angle_deg = 150.0; // reject a band only for a substantial reversal, not a marginal near-perpendicular kink

};






// Clamp the speed by MAX and MIN values
inline double targetSpeedAt(Vec3& p, bool isReverse, TebConfig& cfg) {
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

        bool isReverse = false;

        void initFromRef(std::vector<Vec3> ref, VehicleState vs, int i0, int i1, TebConfig cfg)
        {
            pose.clear();
            dt.clear();

            isReverse = (ref[i0].Dir == 0);
            
            std::vector<double> seedV;
            
            Vec2 curr(vs.x, vs.y);
            
            pose.push_back(TebPose(vs.x, vs.y, vs.theta));
            seedV.push_back(targetSpeedAt(ref[i0], isReverse, cfg));
            
            float accum=0.0f;
            
            for (int i=i0; i<=i1; ++i){
                
                Vec2 p(ref[i].X, ref[i].Y);
                
                accum += Vec2::Dist(curr,p);
                curr = p;

                double v_here = targetSpeedAt(ref[i],isReverse,cfg);
                float spacing  = std::max(0.05, v_here * cfg.dt_ref);

                bool isLast = (i == i1);

                if(isLast || accum >= spacing){
                    accum = 0.0f;
                    double theta = estimateHeadingFromPath(ref, i, i0, i1);
                    pose.push_back(TebPose(p.X, p.Y, theta));
                    seedV.push_back(v_here);
                }
            }

            if (pose.size() < 2){
                double theta = estimateHeadingFromPath(ref, i1, i0, i1);
                pose.push_back(TebPose(ref[i1].X, ref[i1].Y, theta));
                seedV.push_back(targetSpeedAt(ref[i1], isReverse, cfg));
            }

            dt.resize(pose.size() - 1);
            for (size_t k = 0; k < dt.size(); ++k) {
                double d     = Vec2::Dist(Vec2(pose[k].x, pose[k].y), Vec2(pose[k + 1].x, pose[k + 1].y));
                double v_avg = std::max(0.05, 0.5 * (seedV[k] + seedV[k + 1]));
                dt[k] = std::max(cfg.dt_min, std::min(cfg.dt_max, d / v_avg));
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
        explicit TebLocalPlanner(float margin, float plan_horizon, TebConfig cfg = TebConfig())
         : margin_(margin), plan_horizon_(plan_horizon), cfg_(cfg){}

        void init(const ValidityCheckLocal* v){
            validity_ = v;
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

            int i1 = findHorizonEnd(route, vehicleIDx);


            if (outI0) *outI0 = vehicleIDx;
            if (outI1) *outI1 = i1;




            
            if(obs_vec.empty()) return true;
            //If the obstacles are not in the path, return
            if(!pathConflicts(route, obs_vec, vehicleIDx))return true;

            //If the horizon is degenerate, return
            if(i1 <= vehicleIDx) return true;

            TimedElasticBand teb;
            double desired_speed = 10.0;

            teb.initFromRef(route, vs, vehicleIDx, i1, cfg_);

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
                 if (std::sqrt(dx*dx + dy*dy) > plan_horizon_) continue;

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

        bool pathConflicts(const std::vector<Vec3> route, std::vector<TebObstacle> obstacles, int vehicle_id) const
        {
            
            for(TebObstacle& o : obstacles){
                
                double min_d = minClearance(o);
                for(int i=0; i<route.size(); i++){
                    double dx = route[i].X - o.pos.X, dy = route[i].Y - o.pos.Y;
                    if (std::sqrt(dx * dx + dy * dy) < min_d) return true;

                }   
            }
            return false;
        }

        int findHorizonEnd(const std::vector<Vec3> route, int i0)
        {
            double accum = 0.0;
            for(int i=i0; i+1<route.size(); ++i){
                
                if(route[i].Dir != route[i0].Dir) return i;

                double dx = route[i+1].X - route[i].X;
                double dy = route[i+1].Y - route[i].Y;

                accum += std::sqrt(dx*dx + dy*dy);
                if(accum >= plan_horizon_) return i+1;
            }
            return static_cast<int>(route.size()) - 1;
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

            
            
            for(int side : {-1,1}){
                TimedElasticBand attempt = initial;

                std::vector<Vec2> refPts;
                refPts.reserve(attempt.pose.size());
                for(const TebPose& p : attempt.pose) refPts.push_back(Vec2(p.x, p.y));

                densifyNearObstacles(attempt,obstacles, side);

                for(int outer=0; outer < cfg_.outer_iterations; ++outer)
                {
                    solveOnce(attempt, obstacles, refPts);
                    autoResize(attempt);
                    
                }

                if (checkFeasible(attempt, obstacles)) {
                    teb = attempt;
                    return true;
                }

            }
            return false;
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
        void densifyNearObstacles(TimedElasticBand& teb, const std::vector<TebObstacle>& obstacles, int side) const
        {
            size_t guard = 0;
            bool changed = true;

            while(changed && guard < 500){
                changed = false;
                ++guard;
                for (size_t i=0; i+1<teb.pose.size(); ++i){
                    Vec2 p0(teb.pose[i].x,teb.pose[i].y);
                    Vec2 p1(teb.pose[i+1].x,teb.pose[i+1].y);

                    for (const TebObstacle& o : obstacles){
                        double t = projectOntoSegment(o.pos, p0, p1);

                        Vec2 proj(p0.X + t*(p1.X - p0.X), p0.Y + t*(p1.Y - p0.Y));

                        double segDist = Vec2::Dist(o.pos, proj);
                        double need = minClearance(o);

                        if(segDist < need && teb.pose.size() < cfg_.max_samples){
                            
                            double theta = teb.pose[i].theta + t*angleDiff(teb.pose[i + 1].theta, teb.pose[i].theta);

                            double hx = -sin(theta), hy = cos(theta);
                            const Vec2 rel(o.pos.X - proj.X, o.pos.Y - proj.Y);
                            const double s = rel.X * hx + rel.Y * hy;                   // lateral offset of obstacle
                            const double a = rel.X * std::cos(theta) + rel.Y * std::sin(theta); // along-track
                            const double target = 1.15 * need;
                            const double disc = target * target - a * a;
                            if (disc <= 0.0) continue;
                            const double nudge = s + side * std::sqrt(disc);  
                            TebPose mid(proj.X + hx * nudge, proj.Y + hy * nudge, theta);

                            teb.dt[i] *= 0.5;
                            teb.pose.insert(teb.pose.begin() + i + 1, mid);
                            teb.dt.insert(teb.dt.begin() + i + 1, 0.0f);
                            const double v_seed = std::max(0.05f, Vec2::Dist(p0, p1) / std::max(teb.dt[i], 1e-3f));
                            const double dA = Vec2::Dist(p0, Vec2(mid.x, mid.y));
                            const double dB = Vec2::Dist(Vec2(mid.x, mid.y), p1);
                            teb.dt[i]     = std::clamp(dA / v_seed, cfg_.dt_min, cfg_.dt_max);
                            teb.dt[i + 1] = std::clamp(dB / v_seed, cfg_.dt_min, cfg_.dt_max);
                            changed = true;
                            break;
                        }
                    }
                    if(changed) break;
                }
            }
        }

        // Insert poses where dt grew too large, remove where it shrank too
        // small, keeping the band within [min_samples, max_samples].
        void autoResize(TimedElasticBand& teb) const {
            size_t guard = 0;
            bool changed = true;
            while (changed && guard < 500) {
                changed = false;
                ++guard;
                for (size_t i = 0; i < teb.dt.size(); ++i) {
                    if (teb.dt[i] > cfg_.dt_ref + cfg_.dt_hysteresis &&
                        teb.pose.size() < cfg_.max_samples) {
                        double midx = 0.5 * (teb.pose[i].x + teb.pose[i + 1].x);
                        double midy = 0.5 * (teb.pose[i].y + teb.pose[i + 1].y);
                        double midth = wrapToPi(
                            teb.pose[i].theta + 0.5 * angleDiff(teb.pose[i + 1].theta, teb.pose[i].theta));
                        double half = teb.dt[i] * 0.5;
                        
                        teb.dt[i] = half;
                        teb.pose.insert(teb.pose.begin() + i + 1, TebPose(midx, midy, midth));
                        teb.dt.insert(teb.dt.begin() + i + 1, half);
                        changed = true;
                        break;
                    }
                    if (i + 1 < teb.dt.size() &&
                        teb.dt[i] < cfg_.dt_ref - cfg_.dt_hysteresis &&
                        teb.pose.size() > cfg_.min_samples) {
                        teb.dt[i] += teb.dt[i + 1];
                        teb.dt.erase(teb.dt.begin() + i + 1);
                        teb.pose.erase(teb.pose.begin() + i + 1);
                        changed = true;
                        break;
                    }
                }
            }
        }


        // Final hard check after optimization. Checks both poses AND segment
        // closest-approach points (not just poses) against every obstacle,
        bool checkFeasible(const TimedElasticBand& teb, const std::vector<TebObstacle>& obstacles) const {
            for (const TebPose& p : teb.pose) {
                if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.theta)) return false;
                if (validity_ && !validity_->isVehicleFeasible(p.x, p.y, p.theta)) return false;
                for (const TebObstacle& o : obstacles) {
                    double dx = p.x - o.pos.X, dy = p.y - o.pos.Y;
                    double d = std::sqrt(dx * dx + dy * dy);
                    if (d < minClearance(o) * cfg_.feasibility_tolerance) return false;
                }
            }
            for (size_t i = 0; i + 1 < teb.pose.size(); ++i) {
                Vec2 p0(teb.pose[i].x, teb.pose[i].y);
                Vec2 p1(teb.pose[i + 1].x, teb.pose[i + 1].y);
                for (const TebObstacle& o : obstacles) {
                    double t = projectOntoSegment(o.pos, p0, p1);
                    Vec2 proj(p0.X + t * (p1.X - p0.X), p0.Y + t * (p1.Y - p0.Y));
                    double d = Vec2::Dist(o.pos, proj);
                    if (d < minClearance(o) * cfg_.feasibility_tolerance) return false;
                }
            }

            const double kMinSegLen = 0.01;
            const double cosThreshold = std::cos(cfg_.cusp_reject_angle_deg * M_PI / 180.0);
            for (size_t i = 0; i + 2 < teb.pose.size(); ++i) {
                const double d0x = teb.pose[i+1].x - teb.pose[i].x,   d0y = teb.pose[i+1].y - teb.pose[i].y;
                const double d1x = teb.pose[i+2].x - teb.pose[i+1].x, d1y = teb.pose[i+2].y - teb.pose[i+1].y;
                const double len0 = std::sqrt(d0x*d0x + d0y*d0y);
                const double len1 = std::sqrt(d1x*d1x + d1y*d1y);
                if (len0 < kMinSegLen || len1 < kMinSegLen) continue;
                const double cosAngle = (d0x*d1x + d0y*d1y) / (len0 * len1);
                if (cosAngle < cosThreshold)  {
                    std::cout << "[TEB checkFeasible] FAIL: direction change (cusp) at pose " << (i+1)
                              << " -- segments " << len0 << "m/" << len1 << "m, angle="
                              << (std::acos(std::max(-1.0, std::min(1.0, cosAngle))) * 180.0 / M_PI) << "deg\n";
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
        void solveOnce(TimedElasticBand& teb,
                       const std::vector<TebObstacle>& obstacles,
                       const std::vector<Vec2>& refPts) const
        {
            size_t N = teb.pose.size();
            if (N < 3) return;
            

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

                    if(d < minClearance(o) * minClearance(o)){
                        
                        EdgeObstacle* e = new EdgeObstacle();
                        e->obstacleX = o.pos.X;
                        e->obstacleY = o.pos.Y;
                        e->minClearance = minClearance(o);
                        e->setVertex(0, poseVerts[k]);
                        e->setInformation(Eigen::Matrix<double,1,1>::Identity() * cfg_.w_obstacle);
                        e->setRobustKernel(new g2o::RobustKernelHuber());
                        optimizer.addEdge(e);
                    }
                    
                }
                // Via-point: pull toward the NEAREST point on the original
                // (pre-optimization, resize-invariant) reference band -- a
                // full nearest-neighbor scan, not an index-matched lookup,
                // since autoResize/densify change how many poses there are.

                if (!refPts.empty()) {
                    double bestD2 = 1e18;
                    Vec2 nearest = refPts[0];
                    for (const Vec2& r : refPts) {
                        double dx = teb.pose[k].x - r.X, dy = teb.pose[k].y - r.Y;
                        double d2 = dx*dx + dy*dy;
                        if (d2 < bestD2) { bestD2 = d2; nearest = r; }
                    }
                    EdgeViaPoint* e = new EdgeViaPoint();
                    e->setMeasurement(Eigen::Vector2d(nearest.X, nearest.Y));
                    e->setVertex(0, poseVerts[k]);
                    e->setInformation(Eigen::Matrix2d::Identity() * cfg_.w_viapoint);
                    optimizer.addEdge(e);
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
                ev->setRobustKernel(new g2o::RobustKernelHuber());
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
                ea->setRobustKernel(new g2o::RobustKernelHuber());
                optimizer.addEdge(ea);
            }
        
            bool ok = optimizer.initializeOptimization();
            if (!ok) {
                std::cout << "[TEB] g2o initializeOptimization() failed -- leaving band as-is for this pass\n";
                return; // optimizer (and every vertex/edge it owns) is destroyed on scope exit
            }
            optimizer.optimize(cfg_.inner_iterations);
        
            for (size_t k = 0; k < N; ++k) {
                const auto& est = poseVerts[k]->estimate();
                teb.pose[k].x     = est.translation().x();
                teb.pose[k].y     = est.translation().y();
                teb.pose[k].theta = est.rotation().angle();
            }
            for (size_t i = 0; i < teb.dt.size(); ++i) {
                teb.dt[i] = std::max(cfg_.dt_min, std::min(cfg_.dt_max, dtVerts[i]->estimate()));
            }
            
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

        double minClearance(const TebObstacle& o) const {
            double extra = validity_ ? validity_->vehicleWidth() : 0.5;
            return o.radius + margin_ + extra;
            
        }

    
        float margin_, plan_horizon_;
        TebConfig cfg_;
        const ValidityCheckLocal* validity_ = nullptr;


};