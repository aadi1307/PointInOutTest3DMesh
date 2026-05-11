#include "PointInSTL.h"
#include <iostream>
#include <algorithm>

// Compile: g++ -O2 -std=c++17 main.cpp PointInSTL.cpp -o point_in_stl
// Usage:   ./point_in_stl CircularPlateHole.STL [0.5 0.5 0.2]

int main(int argc, char* argv[]) 
{
    if (argc < 2) {
        std::cerr << "usage: point_in_stl file.stl [x y z]\n";
        return 1;
    }

    PointInSTL geom(argv[1]);
    std::cout << "Loaded " << geom.triangleCount() << " triangles\n";

    if (argc == 5) {
        try {
            Point point = { std::stod(argv[2]),
                           std::stod(argv[3]),
                           std::stod(argv[4]) };  // position in 3D space

            std::cout << (geom.isInside(point) ? "INSIDE" : "OUTSIDE") << "\n";

        } catch (...) {
            std::cerr << "error: x y z must be valid numbers\n";
            return 1;
        }
    }

    return 0;
}
