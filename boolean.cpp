#include "boolean.h"
#include <fstream>
#include <cmath>
#include <cstdint>
#include <iostream>

/* ═══════════════════════════════════════════════════════════════════════════
 *  BooleanSTL::subtract
 *  Main entry point.  Performs  A − B  and writes a binary STL.
 * ═══════════════════════════════════════════════════════════════════════════ */
bool BooleanSTL::subtract(const std::string& pathA,
                           const std::string& pathB,
                           const std::string& outputPath)
{
    // Load both meshes.  PointInSTL already reads the binary STL and builds
    // an internal triangle list + supports ray-casting point-in-mesh queries.
    PointInSTL solidA(pathA);
    PointInSTL solidB(pathB);

    std::cout << "Mesh A: " << solidA.triangleCount() << " triangles  (" << pathA << ")\n";
    std::cout << "Mesh B: " << solidB.triangleCount() << " triangles  (" << pathB << ")\n";

    // ── Step 0: Proximity guard ───────────────────────────────────────────
    // If the bounding boxes don't overlap there is no intersection region,
    // so boolean subtract is meaningless (B doesn't touch A at all).
    if (!aabbOverlap(solidA, solidB))
    {
        std::cout << "Meshes are not in vicinity – boolean subtract cannot be performed.\n";
        return false;
    }
    std::cout << "Bounding boxes overlap – proceeding with subtraction.\n";

    std::vector<Triangle> result;

    // ── Step 1: Faces of A that are OUTSIDE B ────────────────────────────
    //
    //  These triangles form the outer shell of the result.
    //  A triangle of A survives the cut when its centroid lies outside B.
    //  (Triangles fully inside B are carved away.)
    //
    //  Why centroid?
    //  It is a single representative point per triangle.  If the centroid is
    //  outside B the face is (mostly) part of the remaining solid.
    //
    int keptA = 0, droppedA = 0;
    for (const auto& tri : solidA.getMesh())
    {
        Vec3 c = centroid(tri);
        if (!solidB.isInside(c))
        {
            result.push_back(tri);  // keep: outside the cutter
            ++keptA;
        }
        else
        {
            ++droppedA;             // discard: inside the cutter (carved away)
        }
    }
    std::cout << "From A  – kept: " << keptA << "  dropped: " << droppedA << "\n";

    // ── Step 2: Faces of B that are INSIDE A  (flipped normals) ──────────
    //
    //  These triangles cap the cavity left by the subtraction.
    //  When B punches through A, B's surface becomes the inner wall of the
    //  hole.  For the mesh to be outward-normal-consistent the winding of
    //  those cap faces must be reversed (so normals point inward into A).
    //
    //  A triangle of B is included when its centroid is inside A.
    //
    int keptB = 0, droppedB = 0;
    for (const auto& tri : solidB.getMesh())
    {
        Vec3 c = centroid(tri);
        if (solidA.isInside(c))
        {
            result.push_back(flipWinding(tri));  // keep with flipped normal
            ++keptB;
        }
        else
        {
            ++droppedB;   // outside A – not part of the result
        }
    }
    std::cout << "From B  – kept (flipped): " << keptB << "  dropped: " << droppedB << "\n";

    // ── Step 3: Write the result ──────────────────────────────────────────
    writeSTL(result, outputPath);
    std::cout << "Result: " << result.size() << " triangles written to " << outputPath << "\n";

    return true;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  aabbOverlap
 *
 *  Axis-Aligned Bounding Box (AABB) overlap test.
 *
 *  Two boxes overlap on every axis simultaneously.
 *  The separating-axis theorem says: if there exists ANY axis along which
 *  the intervals [loA, hiA] and [loB, hiB] do not overlap, the boxes are
 *  separate.  We check all three axes (X, Y, Z).
 *
 *   Interval overlap on one axis:
 *       loA ──────── hiA
 *               loB ──────── hiB   → overlap (not separated)
 *
 *       loA ── hiA    loB ── hiB   → separated (hiA < loB)
 * ═══════════════════════════════════════════════════════════════════════════ */
bool BooleanSTL::aabbOverlap(const PointInSTL& a, const PointInSTL& b)
{
    Vec3 aLo, aHi, bLo, bHi;
    a.boundingBox(aLo, aHi);
    b.boundingBox(bLo, bHi);

    std::cout << "A BBox: [(" << aLo.x << ", " << aLo.y << ", " << aLo.z << ") – "
              <<              "(" << aHi.x << ", " << aHi.y << ", " << aHi.z << ")]\n";
    std::cout << "B BBox: [(" << bLo.x << ", " << bLo.y << ", " << bLo.z << ") – "
              <<              "(" << bHi.x << ", " << bHi.y << ", " << bHi.z << ")]\n";

    // Separated on X, or Y, or Z → no overlap
    if (aHi.x < bLo.x || bHi.x < aLo.x) return false;
    if (aHi.y < bLo.y || bHi.y < aLo.y) return false;
    if (aHi.z < bLo.z || bHi.z < aLo.z) return false;

    return true;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  centroid
 *
 *  Geometric center of a triangle = average of its three vertex positions.
 *
 *       C = (A + B + C) / 3
 *
 *  This gives a single representative point for classifying the triangle
 *  as inside or outside another mesh.
 * ═══════════════════════════════════════════════════════════════════════════ */
Vec3 BooleanSTL::centroid(const Triangle& tri)
{
    return {
        (tri.a.x + tri.b.x + tri.c.x) / 3.0,
        (tri.a.y + tri.b.y + tri.c.y) / 3.0,
        (tri.a.z + tri.b.z + tri.c.z) / 3.0
    };
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  flipWinding
 *
 *  Reverse the triangle's winding order by swapping vertices B and C.
 *
 *  By convention (right-hand rule) the outward normal of a triangle (A,B,C)
 *  is  N = AB × AC.  Swapping B↔C gives  N' = AC × AB = −N,  flipping the
 *  normal 180°.
 *
 *       Original winding:  A → B → C  (normal points outward from B)
 *       Flipped  winding:  A → C → B  (normal points inward)
 *
 *  This is needed for the cap faces contributed by mesh B so they are
 *  consistently oriented inside the result solid.
 * ═══════════════════════════════════════════════════════════════════════════ */
Triangle BooleanSTL::flipWinding(const Triangle& tri)
{
    return { tri.a, tri.c, tri.b };  // swap b ↔ c
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  writeSTL
 *
 *  Binary STL layout (little-endian):
 *
 *   Offset   Size   Field
 *   ──────   ────   ──────────────────────────────────────
 *      0      80    Header (arbitrary text, not null-terminated)
 *     80       4    uint32  triangle count
 *     84+     50    Per triangle:
 *               12    float[3]  face normal  (recomputed from vertices)
 *               12    float[3]  vertex 1
 *               12    float[3]  vertex 2
 *               12    float[3]  vertex 3
 *                2    uint16    attribute byte count (usually 0)
 * ═══════════════════════════════════════════════════════════════════════════ */
void BooleanSTL::writeSTL(const std::vector<Triangle>& tris, const std::string& path)
{
    std::ofstream f(path, std::ios::binary);
    if (!f)
    {
        std::cerr << "BooleanSTL: cannot write output file: " << path << "\n";
        return;
    }

    // 80-byte header
    char header[80] = {};
    const char* label = "BooleanSTL subtract result";
    for (int i = 0; label[i] && i < 79; ++i)
        header[i] = label[i];
    f.write(header, 80);

    // triangle count
    uint32_t count = static_cast<uint32_t>(tris.size());
    f.write(reinterpret_cast<const char*>(&count), 4);

    // write each triangle
    for (const auto& tri : tris)
    {
        // Recompute the face normal from vertices so it is always consistent
        // with the winding order we wrote (including any flipped triangles).
        Vec3 ab = PointInSTL::sub(tri.a, tri.b);
        Vec3 ac = PointInSTL::sub(tri.a, tri.c);
        Vec3 n  = PointInSTL::crossProduct(ab, ac);
        double len = PointInSTL::magnitude(n);
        if (len > 1e-12)
            n = { n.x/len, n.y/len, n.z/len };

        float normal[3] = { (float)n.x,     (float)n.y,     (float)n.z     };
        float va[3]     = { (float)tri.a.x,  (float)tri.a.y, (float)tri.a.z };
        float vb[3]     = { (float)tri.b.x,  (float)tri.b.y, (float)tri.b.z };
        float vc[3]     = { (float)tri.c.x,  (float)tri.c.y, (float)tri.c.z };
        uint16_t attr   = 0;

        f.write(reinterpret_cast<const char*>(normal), 12);
        f.write(reinterpret_cast<const char*>(va),     12);
        f.write(reinterpret_cast<const char*>(vb),     12);
        f.write(reinterpret_cast<const char*>(vc),     12);
        f.write(reinterpret_cast<const char*>(&attr),   2);
    }
}
