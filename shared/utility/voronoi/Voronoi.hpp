
#ifndef UTILITY_VORONOI_VORONOI_H
#define UTILITY_VORONOI_VORONOI_H

#include <algorithm>
#include <cmath>
#include <queue>
#include <vector>

namespace utility::voronoi {

    struct Point {
        int a;
        int b;
        Point(int x, int y) : a(x), b(y) {}
    };

    struct Segment {
        Point p0;
        Point p1;
        Segment(int x1, int y1, int x2, int y2) : p0(x1, y1), p1(x2, y2) {}
    };

}  // namespace utility::voronoi

#endif  // UTILITY_VORONOI_VORONOI_H
