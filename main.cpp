#include "PointInSTL.h"
#include <iostream>
#include <algorithm>

// Compile: g++ -O2 -std=c++17 main.cpp PointInSTL.cpp -o point_in_stl
// Usage:   ./point_in_stl CircularPlateHole.STL [x y z]

int main(int argc, char* argv[]) 
{
    if (argc < 2) {
        std::cerr << "usage: point_in_stl file.stl [x y z]\n";
        return 1;
    }

    PointInSTL geom(argv[1]);
    std::cout << "Loaded " << geom.triangleCount() << " triangles\n";

    Vec3 lo, hi;
    geom.boundingBox(lo, hi);
    std::cout << "BBox min: (" << lo.x << ", " << lo.y << ", " << lo.z << ")\n";
    std::cout << "BBox max: (" << hi.x << ", " << hi.y << ", " << hi.z << ")\n";
    std::cout << "Center:   (" << (lo.x+hi.x)/2 << ", " << (lo.y+hi.y)/2
              << ", " << (lo.z+hi.z)/2 << ")\n";

    if (argc == 5) {
        try {
            Vec3 point = { std::stod(argv[2]),
                           std::stod(argv[3]),
                           std::stod(argv[4]) };

            std::cout << (geom.isInside(point) ? "INSIDE" : "OUTSIDE") << "\n";

        } catch (...) {
            std::cerr << "error: x y z must be valid numbers\n";
            return 1;
        }
    }

    return 0;
}
