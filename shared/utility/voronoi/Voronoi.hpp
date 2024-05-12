
#ifndef UTILITY_VORONOI_VORONOI_H
#define UTILITY_VORONOI_VORONOI_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <queue>
#include <vector>

#include "boost/polygon/voronoi.hpp"

namespace utility::voronoi {

    using boost::polygon::voronoi_builder;
    using boost::polygon::voronoi_diagram;

    struct Point {
        double a;
        double b;
        Point(int x, int y) : a(x), b(y) {}
    };

    struct Segment {
        Point p0;
        Point p1;
        Segment(int x1, int y1, int x2, int y2) : p0(x1, y1), p1(x2, y2) {}
    };

    Point convert_to_Point(Eigen::Vector3f pt) {
        return Point(pt.x(), pt.y());
    }

    std::vector<Point> convert_to_Points(std::vector<Eigen::Vector3f> pts) {
        std::vector<Point> points;

        for (auto& p : pts) {
            points.push_back(Point(p.x(), p.y()));
        }

        return points;
    }

    voronoi_diagram<double> create_voronoi_model(std::vector<Eigen::Vector3f> positions) {
        std::vector<Point> points = convert_to_Points(positions);
        std::vector<Segment> segments;
        voronoi_diagram<double> vd;

        // Construct the Voronoi diagram from the set of points
        construct_voronoi(points.begin(), points.end(), segments.begin(), segments.end(), &vd);

        return voronoi_diagram<double>();
    }

    voronoi_diagram<double> update_voronoi(std::vector<Eigen::Vector3f> team_A, std::vector<Eigen::Vector3f> team_B) {
        // voronoi_diagram<float> vd;

        return voronoi_diagram<double>();
    }

}  // namespace utility::voronoi

#endif  // UTILITY_VORONOI_VORONOI_H
