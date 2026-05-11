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

bool PointInSTL::isInside(Point point) const 
{
    return rayCasting(point, mesh);
}

int PointInSTL::triangleCount() const 
{
    return (int)mesh.size();
}

/* ------------------------------- math ------------------------------- */

Vec3 PointInSTL::sub(Point a, Point b) {
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

// Dot product of u . v
double PointInSTL::dotProduct(Vec3 u, Vec3 v) {
    return u.x*v.x + u.y*v.y + u.z*v.z;
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
    for (int i = 0; i < (int)out.size(); i++) {
        Triangle& tri = out[i];
        
        float normal[3];
        f.read((char*)normal, 12);

        float va[3]; f.read((char*)va, 12);
        float vb[3]; f.read((char*)vb, 12);
        float vc[3]; f.read((char*)vc, 12);

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
bool PointInSTL::rayCasting(Point point, const std::vector<Triangle>& mesh) 
{
    Vec3 ray = {1.0, 1e-4, 1e-4};         // direction vector — not a position
    int  hits = 0;

    for (int i = 0; i < (int)mesh.size(); i++)
    {
        const Triangle& tri = mesh[i];
        Vec3 ab = sub(tri.a, tri.b);            // edge vector A→B
        Vec3 ac = sub(tri.a, tri.c);            // edge vector A→C

        Vec3   h   = crossProduct(ray, ac);     // ray x AC
        double det = dotProduct(ab, h);         // AB . h

        if (std::fabs(det) < TOL) continue;     // ray parallel to triangle plane

        double inv = 1.0 / det;
        Vec3   g   = sub(tri.a, point);         // vector from A to query point

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
