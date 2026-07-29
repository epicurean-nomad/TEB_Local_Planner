#pragma once
#include <cmath>

struct Point3d {
    float x = 0.f, y = 0.f, z = 0.f;

    Point3d() = default;
    Point3d(float x, float y) : x(x), y(y), z(0.f) {}
    Point3d(float x, float y, float z) : x(x), y(y), z(z) {}

    Point3d operator+(const Point3d& b) const { return {x + b.x, y + b.y, z + b.z}; }
    Point3d operator-(const Point3d& b) const { return {x - b.x, y - b.y, z - b.z}; }

    float length() const { return std::sqrt(x * x + y * y + z * z); }

    Point3d mix(const Point3d& b, float t) const {
        return {x + (b.x - x) * t, y + (b.y - y) * t, z + (b.z - z) * t};
    }
};

struct Vec3 {
    float X, Y, Dir, V, Curvature, LaneID;
};

struct Vec2 {
    float X = 0, Y = 0;
    Vec2() = default;
    Vec2(float x, float y) : X(x), Y(y) {}
    Vec2 operator+(const Vec2& O) const { return {X + O.X, Y + O.Y}; }
    Vec2 operator-(const Vec2& O) const { return {X - O.X, Y - O.Y}; }
    Vec2 operator*(float S) const { return {X * S, Y * S}; }
    Vec2 operator/(float S) const { return {X / S, Y / S}; }
    Vec2& operator+=(const Vec2& O) { X += O.X; Y += O.Y; return *this; }
    Vec2& operator-=(const Vec2& O) { X -= O.X; Y -= O.Y; return *this; }
    float Dot(const Vec2& O) const { return X * O.X + Y * O.Y; }
    float Cross(const Vec2& O) const { return X * O.Y - Y * O.X; }
    float Size() const { return std::sqrt(X * X + Y * Y); }
    float SizeSq() const { return X * X + Y * Y; }
    Vec2 GetNormalized() const { float s = Size(); return s > 0.001f ? Vec2{X / s, Y / s} : Vec2{1, 0}; }
    Vec2 GetPerp() const { return Vec2{-Y , X}; }
    static float Dist(const Vec2& A, const Vec2& B) { return (A - B).Size(); }
    static float DistSq(const Vec2& A, const Vec2& B) { return (A - B).SizeSq(); }
    static float DotProduct(const Vec2& A, const Vec2& B) { return A.Dot(B); }
    static float Distance(const Vec2& A, const Vec2& B) { return Dist(A, B); }
    static float DistSquared(const Vec2& A, const Vec2& B) { return DistSq(A, B); }
    Vec2 Last() const { return *this; } // compatibility shim
};

inline Vec2 operator*(float S, const Vec2& V) { return {V.X * S, V.Y * S}; }

inline float DegreesToRadians(float D) { return D * M_PI/ 180.f; }
inline float RadiansToDegrees(float R) { return R * 180.f / M_PI; }
