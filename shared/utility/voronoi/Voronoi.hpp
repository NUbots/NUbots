
#ifndef UTILITY_VORONOI_VORONOI_H
#define UTILITY_VORONOI_VORONOI_H

#include <algorithm>
#include <cmath>
#include <queue>
#include <vector>

namespace utility::voronoi {

    // struct Site {
    //     double x, y;
    // };

    // struct CircleEvent {
    //     double x, y, r;
    //     bool valid;
    // };

    // struct Event {
    //     double x, y;
    //     Site* site;
    //     CircleEvent* circleEvent;

    //     bool operator<(const Event& other) const {
    //         if (y != other.y) {
    //             return y > other.y;
    //         }

    //         return x > other.x;
    //     }
    // };

    // struct Edge {
    //     Site *left, *right;
    //     Site *start, *end;
    //     double m, b;
    // };

    // class Voronoi {

    // private:
    //     void computeEdges(std::vector<Site*>& sites);
    //     bool done_all_edges = false;

    //     std::vector<Site*> sites;
    //     std::vector<Edge> edges;
    //     std::vector<CircleEvent*> circleEvents;

    // public:
    //     void computeVoronoi(std::vector<Site*>& sites);
    //     std::vector<Edge> get_edges();
    //     bool has_made_all_edges();
    // };
}  // namespace utility::voronoi

#endif  // UTILITY_VORONOI_VORONOI_H
