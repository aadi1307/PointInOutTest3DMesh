#include "PointInSTL.h"
#include <fstream>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <algorithm>

/* ---------------------------- constructor ---------------------------*/

PointInSTL::PointInSTL(const std::string& path) 
{
    mesh = readSTL(path);
}

bool PointInSTL::isInside(Vec3 point) const 
{
    return rayCasting(point, mesh);
}

int PointInSTL::triangleCount() const 
{
    return (int)mesh.size();
}

/* ------------------------------- math ------------------------------- */

void PointInSTL::boundingBox(Vec3& lo, Vec3& hi) const
{
    lo = hi = mesh[0].a;
    for (const auto& tri : mesh)
        for (const auto* v : {&tri.a, &tri.b, &tri.c}) 
        {
            lo = { std::min(lo.x, v->x), 
                   std::min(lo.y, v->y), 
                   std::min(lo.z, v->z) };
            hi = { std::max(hi.x, v->x), 
                   std::max(hi.y, v->y), 
                   std::max(hi.z, v->z) };
        }
}

Vec3 PointInSTL::sub(Vec3 a, Vec3 b) {
    return { b.x - a.x, b.y - a.y, b.z - a.z };
}

// Cross product of u x v  (2 direction vectors)
Vec3 PointInSTL::crossProduct(Vec3 u, Vec3 v) 
{
    return {
        u.y*v.z - u.z*v.y,
        u.z*v.x - u.x*v.z,
        u.x*v.y - u.y*v.x
    };
}

// Dot product of u . v  (2 direction vectors)
double PointInSTL::dotProduct(Vec3 u, Vec3 v) {
    return u.x*v.x + u.y*v.y + u.z*v.z;
}

double PointInSTL::magnitude(Vec3 v) {
    return std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
}

/* ---------------------------- geometry --------------------------- */

/*p lies on segment ab if:
  1. AB x AP = zero vector (p is collinear with a and b)
  2. p lies between a and b on every axis
Note: crossProduct(a, b, p) → AB x AP  because arg order is (origin, tip1, tip2)*/
bool PointInSTL::isPointOnSegment(Vec3 a, Vec3 p, Vec3 b) {
    Vec3 ap = sub(a, p);
    Vec3 ab = sub(a, b);

    Vec3 ap_cross_ab = crossProduct(ap, ab);     // AP x AB — zero if collinear

    if (magnitude(ap_cross_ab) > TOL)
    {
        return false;
    }

    return (std::min(a.x, b.x) - TOL <= p.x && p.x <= std::max(a.x, b.x) + TOL &&
            std::min(a.y, b.y) - TOL <= p.y && p.y <= std::max(a.y, b.y) + TOL &&
            std::min(a.z, b.z) - TOL <= p.z && p.z <= std::max(a.z, b.z) + TOL);
}
 
Vec3 PointInSTL::computeNormal(const Triangle& tri) 
{
    Vec3 ab = sub(tri.a, tri.b);
    Vec3 ac = sub(tri.a, tri.c);
    Vec3   n   = crossProduct(ab, ac);   // AB x AC
    double len = magnitude(n);

    if (len < TOL) 
    {
        return {0, 0, 0};
    }

    return { n.x/len, n.y/len, n.z/len };
}

/* Barycentric coordinates of point p relative to triangle */
Barycentric PointInSTL::barycentricCoords(Vec3 p, const Triangle& tri) 
{
    Vec3 ab = sub(tri.a, tri.b);
    Vec3 ac = sub(tri.a, tri.c);
    Vec3   n    = crossProduct(ab, ac);   // AB x AC
    double area = magnitude(n);

    if (area < TOL) 
    {
        return {0, 0, 0};
    }

    /*project p onto the triangle plane
    proj= p − dist * n/|n| where dist is the signed distance from p to the plane along the normal direction*/
    Vec3   unit_n = { n.x/area, n.y/area, n.z/area };
    double dist   = dotProduct(unit_n, sub(tri.a, p));

    Vec3 proj = { p.x - dist*unit_n.x,
                  p.y - dist*unit_n.y,
                  p.z - dist*unit_n.z };
 
    /*signed sub-triangle areas dot each sub-normal with the main normal
    this preserves sign so u/v go negative when proj is outside the triangle*/
    Vec3 pb = sub(proj,  tri.b);
    Vec3 pc = sub(proj,  tri.c);
    Vec3 ap = sub(tri.a, proj);

    double u = dotProduct(crossProduct(pb, pc), n) / (area * area);
    double v = dotProduct(crossProduct(ap, ac), n) / (area * area);
    double w = 1.0 - u - v;

    return { u, v, w };
}

/* ---------------------------- STL reader --------------------------- */

std::vector<Triangle> PointInSTL::readSTL(const std::string& path) 
{
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        std::cerr << "PointInSTL: cannot open: " << path << "\n";
        std::exit(1);
    }

    char header[80];
    f.read(header, 80);

    uint32_t count;
    f.read((char*)&count, 4);

    std::vector<Triangle> out(count);
    for (auto& tri : out) {
        float normal[3];
        f.read((char*)normal, 12);     

        float va[3]; f.read((char*)va, 12);
        float vb[3]; f.read((char*)vb, 12);
        float vc[3]; f.read((char*)vc, 12);

        // widen float -> double all arithmetic stays in double from here on
        tri.a = { va[0], va[1], va[2] };
        tri.b = { vb[0], vb[1], vb[2] };
        tri.c = { vc[0], vc[1], vc[2] };

        uint16_t attr;
        f.read((char*)&attr, 2);
    }
    return out;
}

/* ---------------------------- ray casting --------------------------- */

/*citing:- Möller–Trumbore ray-triangle intersection algorithm
Odd hits -> inside,  Even hits -> outside */
bool PointInSTL::rayCasting(Vec3 point, const std::vector<Triangle>& mesh) 
{
    Vec3 ray = {1.0, 1e-4, 1e-4};
    int  hits = 0;

    for (const auto& tri : mesh) 
    {
        Vec3 ab = sub(tri.a, tri.b);
        Vec3 ac = sub(tri.a, tri.c);

        Vec3   h   = crossProduct(ray, ac);     // ray x AC
        double det = dotProduct(ab, h);         // AB . h

        if (std::fabs(det) < TOL) continue;     // ray parallel to triangle plane

        double inv = 1.0 / det;
        Vec3   g   = sub(tri.a, point);

        double u = dotProduct(g, h) * inv;
        if (u < 0 || u > 1) continue;

        Vec3   q = crossProduct(g, ab);         // g x AB
        double v = dotProduct(ray, q) * inv;
        if (v < 0 || u + v > 1) continue;

        double t = dotProduct(ac, q) * inv;     // distance along ray
        if (t > TOL) hits++;
    }

    return hits % 2 == 1;
}
