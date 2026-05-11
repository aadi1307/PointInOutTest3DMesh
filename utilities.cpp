
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