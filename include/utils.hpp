#pragma once

#include <string>
#include <cmath>

float MIN_LANE_WIDTH_FOR_CUSP = 4.45;
const double   MAX_SPEED                 = 7.0;          // [m/s]    //  1mps ---> 3.6 kmph
const double   MIN_SPEED                 = 0.83;         // [m/s]  
// Kinematics
const double MAX_ACC                        = 3.0;    // [m/s²]  max acceleration
const double MAX_DEC                        = 2.0;    // [m/s²]  max deceleration
const double MAX_LAT_ACC                    = 2.0;    // [m/s²]  lateral acceleration budget

const double EFFECTIVE_MIN                 = 1.39;  // [m/s]  ~5 km/h

// Transition braking profile
const double SLOW_START_DIST                = 8.0;    // [m]   begin braking before gear switch
const double CRAWL_DIST                     = 2.5;    // [m]   reach CRAWL_SPEED
const double STOP_DIST                      = 0.8;    // [m]   reach STOP_SPEED
const double CRAWL_SPEED                    = 1.5;    // [m/s]
const double STOP_SPEED                     = 0.3;    // [m/s]
const double RAMP_DIST                     = 5.0;   // [m]    ramp-up distance at segment start

// ── Reverse motion  [FIX-2] ───────────────────────────────────────────────
const double MAX_SPEED_REVERSE   = 1.389; // [m/s]   5  km/h  [CHANGED 3.0→1.389]
const double CRAWL_SPEED_REV     = 0.833; // [m/s]   3  km/h  [NEW]
const double EFFECTIVE_MIN_REV   = 0.556; // [m/s]   2  km/h  [NEW]

// Reverse transition braking distances  (shorter because speed is lower)
const double SLOW_START_DIST_REV = 15.0;  // [m]
const double CRAWL_DIST_REV      =  8.0;  // [m]
const double STOP_DIST_REV       =  5.0;  // [m]

const double WHEELBASE = 2.0;
const double MAX_STEER_ANGLE_RAD = 0.48;



// struct Vec3 {
//     float X, Y, Dir, V, Curvature;
// };

// struct Vec2 {
//     float X = 0, Y = 0;
//     Vec2() = default;
//     Vec2(float x, float y) : X(x), Y(y) {}
//     Vec2 operator+(const Vec2& O) const { return {X + O.X, Y + O.Y}; }
//     Vec2 operator-(const Vec2& O) const { return {X - O.X, Y - O.Y}; }
//     Vec2 operator*(float S) const { return {X * S, Y * S}; }
//     Vec2 operator/(float S) const { return {X / S, Y / S}; }
//     Vec2& operator+=(const Vec2& O) { X += O.X; Y += O.Y; return *this; }
//     Vec2& operator-=(const Vec2& O) { X -= O.X; Y -= O.Y; return *this; }
//     float Dot(const Vec2& O) const { return X * O.X + Y * O.Y; }
//     float Cross(const Vec2& O) const { return X * O.Y - Y * O.X; }
//     float Size() const { return std::sqrt(X * X + Y * Y); }
//     float SizeSq() const { return X * X + Y * Y; }
//     Vec2 GetNormalized() const { float s = Size(); return s > 0.001f ? Vec2{X / s, Y / s} : Vec2{1, 0}; }
//     Vec2 GetPerp() const { return Vec2{-Y , X}; }
//     static float Dist(const Vec2& A, const Vec2& B) { return (A - B).Size(); }
//     static float DistSq(const Vec2& A, const Vec2& B) { return (A - B).SizeSq(); }
//     static float DotProduct(const Vec2& A, const Vec2& B) { return A.Dot(B); }
//     static float Distance(const Vec2& A, const Vec2& B) { return Dist(A, B); }
//     static float DistSquared(const Vec2& A, const Vec2& B) { return DistSq(A, B); }
//     Vec2 Last() const { return *this; } // compatibility shim
// };

// inline Vec2 operator*(float S, const Vec2& V) { return {V.X * S, V.Y * S}; }

// inline float DegreesToRadians(float D) { return D * M_PI/ 180.f; }
// inline float RadiansToDegrees(float R) { return R * 180.f / M_PI; }



struct VehicleState {
    double x     = 0;
    double y     = 0;
    double theta = 0;
    double current_speed = 0;
};


