#pragma once

#include "PointInSTL.h"
#include <string>
#include <vector>

/*
 * BooleanSTL  –  CSG Subtract for binary STL meshes
 * ──────────────────────────────────────────────────
 *
 *  CONCEPT  (A subtract B  →  A − B)
 *  ───────────────────────────────────────────────────────────────────────
 *  The result solid contains every point that is INSIDE A  AND  OUTSIDE B.
 *
 *  Visually:
 *       ┌──────────────────┐          ┌──────────────────┐
 *       │        A         │          │   A minus B      │
 *       │    ┌──────┐      │          │   ┌──────┐       │
 *       │    │  B   │      │   →      │   │ gap  │       │
 *       │    └──────┘      │          │   └──────┘       │
 *       └──────────────────┘          └──────────────────┘
 *
 *
 *  ALGORITHM  (classification-based – good starting point for learning)
 *  ───────────────────────────────────────────────────────────────────────
 *  Step 0  Proximity guard
 *          Compute axis-aligned bounding boxes (AABBs) of both meshes.
 *          If they do not overlap there can be no intersection, so we
 *          return false immediately.
 *
 *  Step 1  Collect faces from A that lie OUTSIDE B
 *          For every triangle of A, test whether its centroid is inside B
 *          (using ray-casting point-in-mesh).
 *            centroid outside B  →  face is part of the result outer shell
 *            centroid inside  B  →  face is carved away, skip it
 *
 *  Step 2  Collect faces from B that lie INSIDE A  (with flipped winding)
 *          These triangles cap the cavity created by the subtraction.
 *          Their normals must point inward (toward A's interior), so we
 *          swap two vertices to reverse the winding order.
 *            centroid inside  A  →  flip and add to result
 *            centroid outside A  →  this face is irrelevant, skip it
 *
 *  Step 3  Write the collected triangles to a binary STL file.
 *
 *
 *  LIMITATION
 *  ───────────────────────────────────────────────────────────────────────
 *  Centroid-only classification is an approximation.  Triangles that
 *  straddle the A/B boundary are kept or dropped as a whole, which can
 *  leave small gaps or overlaps near the seam.  A production-grade CSG
 *  solver would intersect those triangles, split them, and re-triangulate.
 *  That is the natural next step after understanding this implementation.
 *
 *
 *  USAGE
 *  ───────────────────────────────────────────────────────────────────────
 *  bool ok = BooleanSTL::subtract("body.stl", "tool.stl", "result.stl");
 *  if (!ok)  // meshes are not in vicinity – no operation performed
 *
 *
 *  COMPILE
 *  ───────────────────────────────────────────────────────────────────────
 *  g++ -O2 -std=c++17 main.cpp PointInSTL.cpp boolean.cpp -o boolean_stl
 */

class BooleanSTL
{
public:
    // Subtract B from A and write the result to outputPath.
    // Returns false (no output written) when the two meshes do not overlap.
    static bool subtract(const std::string& pathA,
                         const std::string& pathB,
                         const std::string& outputPath);

private:
    // Returns true when the axis-aligned bounding boxes of a and b overlap.
    static bool    aabbOverlap (const PointInSTL& a, const PointInSTL& b);

    // Geometric centroid of a triangle  (average of its three vertices).
    static Vec3    centroid    (const Triangle& tri);

    // Reverse triangle winding (swap b ↔ c) so the face normal flips 180°.
    static Triangle flipWinding(const Triangle& tri);

    // Write a list of triangles to a binary STL file.
    static void    writeSTL   (const std::vector<Triangle>& tris,
                                const std::string& path);
};
