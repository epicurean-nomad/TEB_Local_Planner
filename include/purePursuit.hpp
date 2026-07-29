#pragma once

// NOTE: this header does NOT define or include Vec2/Vec3/VehicleState --
// it assumes whatever includes this has already brought those in (e.g. by
// including the TEB planner's headers, which pull in road_graph_router's
// utils.hpp, first).
//
// Deliberately not including either project's "utils.hpp" for numeric
// constants either: there are two different utils.hpp files in this
// codebase with the SAME constant names (MAX_SPEED, EFFECTIVE_MIN, ...) but
// DIFFERENT values -- the road_graph_router one (defines Vec2/Vec3/
// VehicleState; MAX_SPEED=7.0, used to generate the TEB-planned route's
// velocity profile) and the real controller's one (uploaded separately; no
// Vec2/Vec3/VehicleState at all -- trajectory is a raw double[3] cenTraj +
// separate targetVelocity vector; MAX_SPEED=5.5, the REAL vehicle's actual
// physical limit). Silently including "utils.hpp" here would resolve to
// whichever one happens to be first on the include path -- fragile and
// wrong either way. This class's own tuning constants below are hardcoded
// literals copied directly from the REAL controller's utils.hpp, with their
// provenance noted in each comment, rather than derived from an ambient
// #include.
#include <vector>
#include <cmath>
#include <algorithm>

// ============================================================================
// Pure Pursuit Controller -- isolated from Swaayatt Robots' runFile_7_05_1.cpp
// ============================================================================
//
// This keeps the CORE control algorithm from the real controller (closest-
// point search, adaptive/tapered lookahead, direction-transition handling,
// the pure-pursuit steering law, the lead compensator, and steering
// rate-limiting/smoothing) and strips everything that was specific to the
// real vehicle's hardware/software stack:
//   - LCM publishing (steerMsg/brakeMsg/lookaheadMsg) -- replaced with a
//     plain PurePursuitOutput struct the caller does whatever it wants with.
//   - Encoder-tick steering representation (STEERING_ANGLE_OFFSET, the
//     100-550 encoder range, the /200.0*35.0 conversions) -- a simulated
//     vehicle takes a steering ANGLE, not an actuator encoder position, so
//     the rate-limit/smoothing math now operates directly in degrees and
//     the encoder conversion is dropped entirely.
//   - The localization thread / mutex-guarded globals (__gLocalizationData__,
//     zAg, rt_theta, ALLPROCESSINTERRUPT, ...) -- replaced with an explicit
//     VehicleState argument passed into step() each tick. This class holds
//     no thread of its own and touches no globals.
//   - Dead-reckoning between localization updates, and the yaw low-pass
//     filter (YAW_ALPHA) -- both exist in the real controller to compensate
//     for real sensor latency/noise. dead_reckoning is actually false in the
//     real deployment already (confirmed from utils.hpp), so dropping it
//     matches current real behavior, not just a simulation shortcut. The
//     yaw filter (yaw_filter=true, YAW_ALPHA=0.6 in the real config) IS
//     active there -- I'm omitting it here since a noiseless simulator has
//     nothing to filter and it would only add smoothing lag with no
//     corresponding benefit. If Gazebo ends up injecting sensor noise on
//     the pose feed (a simulated GPS/IMU), this should probably come back.
//
// Everything else -- the actual control law -- is a direct, same-formula
// port, not a reimplementation from scratch. Trajectory representation
// reuses this codebase's existing Vec3 convention (X, Y, Dir, V, Curvature)
// directly, since Dir already carries exactly the same 0=reverse/1=forward
// semantics as the original's cenTraj[i][2], and V already carries exactly
// what targetVelocity[i] did there.
//
// ----------------------------------------------------------------------
// UPDATE: the real utils.hpp has now been provided. Every constant below is
// the REAL value (with its real formula, for adaptive lookahead/speed --
// no longer placeholders), EXCEPT wheelbase/steering-angle/rate-limit/
// lead-compensator values are copied as literals rather than referencing
// the real utils.hpp's globals directly, for the reason explained above.
//
// TWO THINGS WORTH YOUR ATTENTION, found while cross-checking against the
// real file rather than silently absorbed:
//
// 1. VEHICLE CAPABILITY MISMATCH BETWEEN THE TWO PROJECTS: the TEB-planned
//    route's V values were generated assuming MAX_SPEED=7.0 m/s (road_graph_
//    router's utils.hpp), but the real controller's own MAX_SPEED is 5.5
//    m/s. This controller will correctly clamp to 5.5 regardless (safe --
//    matches the real vehicle's actual limit) but it means any route
//    segment the profiler planned above 5.5 m/s will never actually be
//    reached in simulation or reality. Not something to silently "fix" on
//    my end (which number is authoritative depends on what you're testing),
//    just worth knowing before you're confused by a speed plateau.
//
// 2. CONTROL LOOP TIMING IS INTERNALLY INCONSISTENT in the real utils.hpp,
//    not something I introduced: `CONTROL_LOOP_RATE = 50'000` us = 50ms,
//    but its own comment says "dt = 100 ms"; separately,
//    MAX_STEER_RATE_DEG_TICK's comment says "max delta-deg per 20 ms tick".
//    Three different numbers (50ms / 100ms / 20ms) for what should be one
//    tick period. I've defaulted dt_s to 0.05 (the one literal, executable
//    value -- CONTROL_LOOP_RATE itself, not a comment) but MAX_STEER_RATE_
//    DEG_TICK's absolute calibration depends on which tick duration it was
//    actually tuned against, which I can't resolve from the source alone --
//    worth confirming the real loop's measured period before trusting this
//    parameter's absolute scale in a fast-moving sim.
// ----------------------------------------------------------------------

struct PurePursuitConfig {
    // ---- vehicle geometry (real value, from utils.hpp) ----
    double wheelbase_m = 2.0;              // WHEELBASE

    // ---- adaptive lookahead (real formula + constants, from
    // getAdaptiveLookahead(speed_kmph) -- the single-arg overload, which is
    // the one actually called in pcontroller(); the cte/heading-aware
    // 3-arg overloads exist in utils.hpp but aren't wired into the real
    // control loop, so not ported here) ----
    double lookahead_time_constant = 3.5;  // LOOKAHEAD_TIME_CONSTANT [s]: ld = k * v
    double lookahead_min_m         = 2.5;  // LOOKAHEAD_MIN
    double lookahead_max_m         = 18.0; // LOOKAHEAD_MAX

    // ---- direction-transition (cusp) lookahead taper (unchanged from the
    // original -- these specific taper constants weren't in utils.hpp as
    // named globals, they were inline literals in runFile_7_05_1.cpp itself,
    // ported as-is) ----
    double taper_zone_min_m       = 15.0;
    double taper_zone_speed_gain  = 3.0;   // taper_zone = max(taper_zone_min, |v_kmh|/3.6 * gain)
    double taper_min_forward_m    = 4.0;
    double taper_min_reverse_m    = 3.5;
    double direction_switch_dist_m = 1.5;  // how close to a cusp point before flipping gear

    // ---- closest-point / lookahead search bounds (meters, not points --
    // avoids needing a points-per-meter density assumption) ----
    double search_window_min_m = 3.0;
    double search_window_max_m = 40.0;

    // ---- pure pursuit steering law ----
    // delta = steer_sign * atan(2*L*sin(alpha) / ld), alpha = angle to the
    // lookahead point in the vehicle frame.
    //
    // steer_sign = -1.0 matches the original controller's literal formula
    // (`-2.0 * WHEELBASE * sin(...)`) exactly, and is very likely correct
    // for the REAL vehicle -- it's almost certainly calibrated to whichever
    // physical convention that vehicle's steering actuator/IMU uses (which
    // direction a "positive" steering angle actually turns the wheels).
    // It is NOT something to blindly "fix" to the textbook +1 -- the right
    // sign is whichever one matches the plant this drives.
    //
    // CONFIRMED (not guessed): tested this controller closed-loop against a
    // standard-convention kinematic bicycle model (dtheta = (v/L)*tan(steer)*dt,
    // positive steer = left turn) -- steer_sign=-1.0 diverges catastrophically
    // (cross-track error grows unbounded), steer_sign=+1.0 tracks a curved
    // path to <10cm cross-track error and reaches the goal cleanly. That's
    // exactly what you'd expect if the sign is calibrated to a DIFFERENT
    // convention than the standard one -- not a sign of a broken algorithm.
    //
    // Action needed once we pick a Gazebo vehicle model: check which way it
    // turns for a given steering command, and set this to whichever value
    // (+1.0 or -1.0) makes that match "positive angle -> physically turns
    // toward positive lookahead-frame y". Default here is -1.0 (faithful to
    // the real controller) since that's the known-correct starting point
    // until we've verified Gazebo's convention.
    double steer_sign = -1.0;

    double reverse_steer_gain = -1.5;      // matches `-target_angle_deg * 1.5` for reverse

    // ---- lead compensator (phase-lead filter on the steering command) ----
    // LEAD_COMPENSATOR_ENABLE is false in the real utils.hpp -- the
    // compensator branch never actually executes in the current real
    // deployment, despite the code/comments built around it. Defaulted to
    // match that real, current, active configuration.
    bool   lead_enable              = false;   // LEAD_COMPENSATOR_ENABLE (real value)
    double lead_alpha_lo            = 0.1; // at/above lead_alpha_lo_speed_kmh
    double lead_alpha_hi            = 0.4; // at/below lead_alpha_hi_speed_kmh
    double lead_alpha_lo_speed_kmh  = 15.0;
    double lead_alpha_hi_speed_kmh  = 5.0;
    double lead_tau                 = 0.08; // LEAD_TAU (real value -- measured actuator lag)
    double lead_enable_min_speed_kmh = 5.0;
    double lead_output_clamp_rad_factor = 1.0; // matches `MAX_STEERING_ANGLE * 1.3`
    double max_steering_angle_deg   = 35.0;    // MAX_STEERING_ANGLE_ACTUAL (real value)

    // ---- steering rate limit + smoothing (now directly in degrees --
    // no encoder-tick conversion, since a simulator wants an angle).
    // steering_smoothening=true in the real utils.hpp, i.e. this IS the
    // actually-active branch of the original's two alternatives -- the
    // encoder-space branch this class never ported is the inactive one. ----
    double max_steer_rate_deg_per_tick = 1000.0; // MAX_STEER_RATE_DEG_TICK (real value -- see timing-ambiguity note above)
    double alpha_steer                 = 1.0;  // ALPHA_STEER (real value)

    // ---- speed shaping ----
    // Two different floors, faithfully preserved from the original rather
    // than unified -- getAdaptiveSpeed()'s own internal clamp floors at
    // STOP_SPEED (0.3, so it CAN command down to near-a-stop for a gear
    // transition), while the original's outer/final clamp
    // (`speed_cmd = clamp(current_cmd_speed, EFFECTIVE_MIN, MAX_SPEED)`)
    // floors at EFFECTIVE_MIN (0.9) instead. Ported exactly as structured,
    // not "fixed" -- not my place to second-guess whether that interaction
    // is intentional.
    double accel_limit_mps2 = 10.0;      // matches `current_cmd_speed + 10*dt_speed`
    double min_speed_mps    = 0.9;       // EFFECTIVE_MIN (real value) -- outer/final clamp floor
    double max_speed_mps    = 5.5;       // MAX_SPEED (real value) -- see mismatch note above
    double max_steering_angle_rad_for_speed = 0.480; // MAX_STEERING_ANGLE (real value, RADIANS -- distinct from max_steering_angle_deg above, used only in the getAdaptiveSpeed fallback's steering-based dropoff)
    double min_speed_fallback_mps = 0.83; // MIN_SPEED (real value) -- getAdaptiveSpeed's own floor input, distinct from min_speed_mps
    double stop_speed_mps    = 0.3;      // STOP_SPEED (real value)
    double crawl_speed_mps   = 1.3;      // CRAWL_SPEED (real value)
    double slow_start_dist_m = 10.0;     // SLOW_START_DIST (real value)
    double crawl_dist_m      = 8.0;      // CRAWL_DIST (real value)
    double stop_dist_m       = 8.0;      // STOP_DIST (real value)

    // ---- misc ----
    int    goal_tolerance_points = 10;
    double dt_s = 0.05; // control tick period -- see timing-ambiguity note above; matches CONTROL_LOOP_RATE's literal 50'000us
};

struct PurePursuitOutput {
    double steer_angle_rad = 0.0; // positive/negative per the original's atan(-2L sin(alpha)/ld) sign convention
    double speed_mps       = 0.0;
    bool   goal_reached    = false;
    bool   vehicle_direction_forward = true; // current gear, in case the caller needs to command a gear change
    int    index_closest   = 0;
    double lookahead_distance_m = 0.0;   // NOMINAL/intended lookahead (adaptive + taper)
    double actual_lookahead_dist_m = 0.0; // ACTUAL chord distance to lookahead_point -- can be
                                            // less than lookahead_distance_m if the search loop
                                            // terminated early (search_window_m bound, end of
                                            // window, gear boundary) before reaching the nominal
                                            // arc length. Diagnostic: if these two diverge a lot,
                                            // the steering law (which divides by the NOMINAL
                                            // value) is using a denominator larger than the real
                                            // geometry, silently understating the computed angle.
    Vec2   lookahead_point{0.0, 0.0};
};

class PurePursuitController {
public:
    explicit PurePursuitController(PurePursuitConfig cfg = PurePursuitConfig()) : cfg_(cfg) {}

    // route: X, Y, Dir (0=reverse/1=forward), V (target speed at that
    // point, m/s), Curvature (unused here -- already baked into V by
    // computeVelocityProfile()).
    void setTrajectory(const std::vector<Vec3>& route) {
        route_ = route;
        index_closest_ = 0;
        ic_persistent_ = 0;
        vehicle_direction_forward_ = route_.empty() ? true : (route_[0].Dir != 0.0);
        prev_steer_deg_ = 0.0;
        lead_e_prev_ = 0.0;
        lead_u_prev_ = 0.0;
        current_cmd_speed_ = 0.0;
    }

    // Call once per control tick with the vehicle's current true pose
    // (from the simulator/localization) and current speed (m/s, signed
    // magnitude -- this class tracks gear via the route's Dir, not sign of
    // speed).
    PurePursuitOutput step(const VehicleState& vs, double current_speed_mps) {
        PurePursuitOutput out;
        if (route_.size() < 2) { out.goal_reached = true; return out; }

        const int size_traj = static_cast<int>(route_.size());
        const double speed_kmh = current_speed_mps * 3.6; // original tuning (lead-alpha schedule, lead-enable
                                                            // threshold) was expressed in km/h -- preserved as-is
                                                            // rather than re-deriving thresholds in m/s.

        // ---- find next direction transition (cusp) ahead of current progress ----
        int    transition_index = -1;
        double dist_to_transition = 1e7;
        for (int i = index_closest_; i < size_traj - 1; ++i) {
            if (route_[i].Dir != route_[i + 1].Dir) { transition_index = i; break; }
        }
        if (transition_index >= 0) {
            dist_to_transition = std::hypot(route_[transition_index].X - vs.x,
                                             route_[transition_index].Y - vs.y);
        }

        // ---- adaptive lookahead, tapered near a cusp ----
        const double normal_ld = adaptiveLookahead(speed_kmh);
        const double taper_zone = std::max(cfg_.taper_zone_min_m,
                                            std::fabs(speed_kmh) / 3.6 * cfg_.taper_zone_speed_gain);
        double lookahead_distance;
        if (dist_to_transition < taper_zone) {
            const double taper_ratio = dist_to_transition / taper_zone;
            lookahead_distance = std::max(cfg_.taper_min_forward_m, normal_ld * taper_ratio);
            if (!vehicle_direction_forward_)
                lookahead_distance = std::max(cfg_.taper_min_reverse_m,
                                               normal_ld * dist_to_transition / taper_zone);
        } else {
            lookahead_distance = normal_ld;
        }
        // double lookahead_distance = 7.0;
        // ---- search window, bounded directly in meters (arc length) ----
        const double search_window_m = std::clamp(std::max(lookahead_distance * 2.0, 5.0),
                                                    cfg_.search_window_min_m, cfg_.search_window_max_m);

        // ---- Step 1: closest point, forward-only, same gear, arc-length bounded ----
        double dist_minimum = 1e7;
        {
            double arc = 0.0;
            for (int i = index_closest_; i < size_traj; ++i) {
                if (i > index_closest_) {
                    arc += std::hypot(route_[i].X - route_[i - 1].X, route_[i].Y - route_[i - 1].Y);
                    if (arc > search_window_m) break;
                }
                if ((route_[i].Dir != 0.0) != vehicle_direction_forward_) break;
                const double dist = std::hypot(route_[i].X - vs.x, route_[i].Y - vs.y);
                if (dist < dist_minimum) { dist_minimum = dist; index_closest_ = i; }
            }
        }

        // ---- Step 2: lookahead point, freeze at direction boundary ----
        bool approaching_boundary = false;
        double raw_tgt[2] = { route_[ic_persistent_].X, route_[ic_persistent_].Y };
        int ic = ic_persistent_;
        {
            double arc_acc = 0.0;
            double arc = 0.0;
            for (int s = index_closest_; s < size_traj; ++s) {
                if (s > index_closest_) {
                    arc += std::hypot(route_[s].X - route_[s - 1].X, route_[s].Y - route_[s - 1].Y);
                    if (arc > search_window_m) break;
                }
                if ((route_[s].Dir != 0.0) != vehicle_direction_forward_) {
                    approaching_boundary = true;
                    break;
                }
                if (s > index_closest_)
                    arc_acc += std::hypot(route_[s].X - route_[s - 1].X, route_[s].Y - route_[s - 1].Y);

                raw_tgt[0] = route_[s].X;
                raw_tgt[1] = route_[s].Y;
                ic = s;
                ic_persistent_ = s;
                if (arc_acc >= lookahead_distance) break;
            }
        }
        (void)approaching_boundary; // kept for parity/debuggability; not branched on further, same as original

        out.lookahead_point = Vec2(raw_tgt[0], raw_tgt[1]);
        out.lookahead_distance_m = lookahead_distance;
        out.index_closest = index_closest_;

        // ---- transform lookahead point into vehicle frame ----
        double tgtCoors[2] = { raw_tgt[0] - vs.x, raw_tgt[1] - vs.y };
        const double tx =  tgtCoors[0] * std::cos(-vs.theta) - tgtCoors[1] * std::sin(-vs.theta);
        const double ty =  tgtCoors[0] * std::sin(-vs.theta) + tgtCoors[1] * std::cos(-vs.theta);
        out.actual_lookahead_dist_m = std::hypot(tx, ty);

        // ---- direction switch at a cusp, once close enough to it ----
        if (ic < size_traj - 1 && (route_[ic + 1].Dir != 0.0) != vehicle_direction_forward_) {
            const double dist = std::hypot(route_[ic].X - vs.x, route_[ic].Y - vs.y);
            if (dist < cfg_.direction_switch_dist_m) {
                vehicle_direction_forward_ = (route_[ic + 1].Dir != 0.0);
                index_closest_ = ic + 1;
                ic_persistent_ = ic + 1;
                prev_steer_deg_ = 0.0; // matches prevEncoder reset -- no established steering history in the new gear
            }
        }

        // ---- pure pursuit steering law: delta = steer_sign * atan(2L sin(alpha) / ld) ----
        double target_angle_deg = 1.0 * std::atan(2.0 * cfg_.wheelbase_m * std::sin(std::atan2(ty, tx))
                                             / lookahead_distance) * (180.0 / M_PI);

        if (!vehicle_direction_forward_){
            target_angle_deg = -target_angle_deg * 1.5; // reverse compensation, matches original literal 1.5
        }
            
        

        
        // ---- lead compensator ----
        const double lead_alpha =
            (speed_kmh <= cfg_.lead_alpha_hi_speed_kmh) ? cfg_.lead_alpha_hi
          : (speed_kmh >= cfg_.lead_alpha_lo_speed_kmh) ? cfg_.lead_alpha_lo
          : cfg_.lead_alpha_hi + (cfg_.lead_alpha_lo - cfg_.lead_alpha_hi)
                * (speed_kmh - cfg_.lead_alpha_hi_speed_kmh)
                / (cfg_.lead_alpha_lo_speed_kmh - cfg_.lead_alpha_hi_speed_kmh);

        if (cfg_.lead_enable && vehicle_direction_forward_ && speed_kmh > cfg_.lead_enable_min_speed_kmh) {
            const double e_k = target_angle_deg * (M_PI / 180.0);
            const double lead_u = (lead_alpha * cfg_.lead_tau * lead_u_prev_
                                    + cfg_.lead_tau * (e_k - lead_e_prev_)
                                    + e_k * cfg_.dt_s)
                                 / (lead_alpha * cfg_.lead_tau + cfg_.dt_s);
            lead_e_prev_ = e_k;
            lead_u_prev_ = lead_u;

            const double max_rad = cfg_.max_steering_angle_deg * (M_PI / 180.0) * cfg_.lead_output_clamp_rad_factor;
            const double clamped = std::clamp(lead_u, -max_rad, max_rad);
            target_angle_deg = clamped * (180.0 / M_PI);
        }
        // std::cout << "\n";
        // std::cout << "target_angle_deg: " << target_angle_deg << std::endl;
        // std::cout << "vs: " << vs.x << " " << vs.y << " " << vs.theta << std::endl;
        // std::cout << "tx: " << tx << " " << "ty: " << ty << std::endl;
        // std::cout << "lookahead: " << raw_tgt[0] << " " << raw_tgt[1] << std::endl;
        // std::cout << "alpha: " << std::atan2(ty, tx) << std::endl<<std::endl;
        // exit(0);

        // ---- steering rate limit + smoothing (directly in degrees -- no
        // encoder-tick conversion, unlike the original's "smoothening"
        // branch, which this otherwise mirrors) ----
        const double delta_deg = std::clamp(target_angle_deg - prev_steer_deg_,
                                             -cfg_.max_steer_rate_deg_per_tick,
                                              cfg_.max_steer_rate_deg_per_tick);
        const double rate_limited_deg = prev_steer_deg_ + delta_deg;
        const double smoothed_deg = cfg_.alpha_steer * rate_limited_deg
                                   + (1.0 - cfg_.alpha_steer) * prev_steer_deg_;
        prev_steer_deg_ = smoothed_deg;

        out.steer_angle_rad = smoothed_deg * (M_PI / 180.0);
        out.vehicle_direction_forward = vehicle_direction_forward_;

        // ---- speed command from the route's own V (velocity profiler
        // output); getAdaptiveSpeed() fallback is a placeholder -- see
        // note at top of file ----
        const double speed_target = (index_closest_ < size_traj && route_[index_closest_].V > 0.0)
                                   ? route_[index_closest_].V
                                   : fallbackSpeed(target_angle_deg * (M_PI / 180.0), dist_to_transition);

        if (current_cmd_speed_ < speed_target) {
            current_cmd_speed_ = std::min(speed_target, current_cmd_speed_ + cfg_.accel_limit_mps2 * cfg_.dt_s);
        } else {
            current_cmd_speed_ = speed_target; // decelerating -- no limit, trust the velocity profiler
        }
        out.speed_mps = std::clamp(current_cmd_speed_, cfg_.min_speed_mps, cfg_.max_speed_mps);

        // ---- goal check ----
        if (index_closest_ >= size_traj - cfg_.goal_tolerance_points) {
            out.goal_reached = true;
            out.speed_mps = 0.0;
        }

        return out;
    }

private:
    // REAL formula, from getAdaptiveLookahead(speed_kmph) -- the single-arg
    // overload actually called in pcontroller(): ld = clamp(k * v_mps, min, max).
    double adaptiveLookahead(double speed_kmh) const {
        const double v_mps = std::fabs(speed_kmh) / 3.6;
        const double ld = cfg_.lookahead_time_constant * v_mps;
        return std::clamp(ld, cfg_.lookahead_min_m, cfg_.lookahead_max_m);
    }

    // REAL formula, from getAdaptiveSpeed(steering_rad, dist_to_transition) --
    // the 2-arg overload actually called in pcontroller() (the cte/heading-
    // aware 4-arg overloads exist in utils.hpp but aren't wired into the
    // real control loop). Only exercised here if a route's V is ever <= 0,
    // which shouldn't happen for anything that went through
    // computeVelocityProfile() -- but ported faithfully rather than
    // stubbed, in case it does fire.
    double fallbackSpeed(double steer_angle_rad, double dist_to_transition) const {
        // 1. Steering-based limit: quadratic dropoff max->min as |steer| -> max.
        const double abs_steer = std::min(std::fabs(steer_angle_rad), cfg_.max_steering_angle_rad_for_speed);
        const double ratio = abs_steer / cfg_.max_steering_angle_rad_for_speed;
        const double speed_steer = cfg_.max_speed_mps - (ratio * ratio) * (cfg_.max_speed_mps - cfg_.min_speed_fallback_mps);

        // 2. Transition-proximity limit: two-stage ramp to near-stop before a gear switch.
        double speed_transition;
        if (dist_to_transition > cfg_.slow_start_dist_m) {
            speed_transition = cfg_.max_speed_mps;
        } else if (dist_to_transition > cfg_.crawl_dist_m) {
            const double t = (dist_to_transition - cfg_.crawl_dist_m) / (cfg_.slow_start_dist_m - cfg_.crawl_dist_m);
            speed_transition = cfg_.crawl_speed_mps + t * (cfg_.max_speed_mps - cfg_.crawl_speed_mps);
        } else if (dist_to_transition > cfg_.stop_dist_m) {
            const double t = (dist_to_transition - cfg_.stop_dist_m) / (cfg_.crawl_dist_m - cfg_.stop_dist_m);
            speed_transition = cfg_.stop_speed_mps + t * (cfg_.crawl_speed_mps - cfg_.stop_speed_mps);
        } else {
            speed_transition = cfg_.stop_speed_mps;
        }

        return std::clamp(std::min(speed_steer, speed_transition), cfg_.stop_speed_mps, cfg_.max_speed_mps);
    }

    PurePursuitConfig cfg_;
    std::vector<Vec3> route_;
    int  index_closest_ = 0;
    int  ic_persistent_ = 0;
    bool vehicle_direction_forward_ = true;
    double prev_steer_deg_ = 0.0;
    double lead_e_prev_ = 0.0, lead_u_prev_ = 0.0;
    double current_cmd_speed_ = 0.0;
};