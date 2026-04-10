#pragma once

#include <string>
#include <vector>

// ─── data structures ──────────────────────────────────────────────────────────

struct Vec3 
{
    double x, y, z;
};

struct Triangle 
{
    Vec3 a, b, c;
};

struct Barycentric 
{
    double u, v, w;
};

// ─── PointInSTL ──────────────────────────────────────────────────────────────

class PointInSTL 
{
private:
    static constexpr double TOL = 1e-9;   // moved off global scope — only the class uses it

    std::vector<Triangle> mesh;

    static std::vector<Triangle> readSTL    (const std::string& path);
    static bool                  rayCasting (Vec3 point, const std::vector<Triangle>& mesh);

public:
    explicit PointInSTL(const std::string& path);

    static Vec3 sub(Vec3 a, Vec3 b);
    bool isInside      (Vec3 point) const;
    int  triangleCount () const;
    void boundingBox    (Vec3& lo, Vec3& hi) const;

    // static helpers — public so they can be reused and tested independently
    static Vec3   crossProduct (Vec3 u, Vec3 v);
    static double dotProduct   (Vec3 u, Vec3 v);
    static double magnitude    (Vec3 v);

    static bool        isPointOnSegment  (Vec3 a, Vec3 p, Vec3 b);
    static Vec3        computeNormal     (const Triangle& tri);
    static Barycentric barycentricCoords (Vec3 p, const Triangle& tri);

};
