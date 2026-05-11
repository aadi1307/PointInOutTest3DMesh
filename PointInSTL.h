#pragma once

#include <string>
#include <vector>

// ─── data structures ──────────────────────────────────────────────────────────

struct Point 
{
    double x, y, z;
};

struct Triangle 
{
    Point a, b, c;
};

struct Vec3        // direction / displacement — NOT a position
{
    double x, y, z;
};

struct Barycentric 
{
    double u, v, w;
};

/* ------------ PointInSTL ------------------------------- */

class PointInSTL 
{
private:
    static constexpr double TOL = 1e-9;  // tolerance for floating-point comparisons

    std::vector<Triangle> mesh; // all triangles from the STL file

    static std::vector<Triangle> readSTL    (const std::string& path); // reads STL file, returns list of triangles
    static bool                  rayCasting (Point point, const std::vector<Triangle>& mesh); // point is a POSITION

public:
    explicit PointInSTL(const std::string& path); // constructor reads STL file and stores triangles in mesh

    static Vec3  sub(Point a, Point b);         // two positions → direction vector
    
    bool isInside      (Point point) const;
    int  triangleCount () const;
    // void boundingBox    (Point& lo, Point& hi) const;

    /* only for boolean operations */
    const std::vector<Triangle>& getMesh() const { return mesh; }  // read-only access for boolean ops

    // static helpers, public so they can be reused and tested independently
    static Vec3   crossProduct (Vec3 u, Vec3 v);  // two vectors → perpendicular vector
    static double dotProduct   (Vec3 u, Vec3 v);  // two vectors → scalar
    //static double magnitude    (Point v);

    // static bool        isPointOnSegment  (Point a, Point p, Point b);
    // static Point        computeNormal     (const Triangle& tri);
    // static Barycentric barycentricCoords (Point p, const Triangle& tri);

};
