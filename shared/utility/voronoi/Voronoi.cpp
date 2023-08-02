#include "Voronoi.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <queue>
#include <vector>

namespace utility::voronoi {

    // void Voronoi::computeEdges(std::vector<Site*>& sites) {
    //     std::priority_queue<Event> pq;

    //     for (Site* site : sites) {
    //         pq.push({site->x, site->y, site, nullptr});
    //     }

    //     done_all_edges = false;
    //     while (!pq.empty()) {
    //         Event e = pq.top();
    //         pq.pop();

    //         if (e.circleEvent && !e.circleEvent->valid) {
    //             continue;
    //         }

    //         if (e.site) {
    //             Site* s = e.site;
    //             Edge edge;
    //             edge.left  = s;
    //             edge.right = nullptr;
    //             edges.push_back(edge);
    //         }
    //         else {
    //             CircleEvent* ce = e.circleEvent;

    //             for (Edge& edge : edges) {
    //                 if (edge.left->y > ce->y || (edge.right->x && edge.right->y > ce->y)) {
    //                     continue;
    //                 }

    //                 double dx = edge.left->x - ce->x;
    //                 double dy = edge.left->y - ce->y;
    //                 double dr = sqrt(dx * dx + dy * dy);

    //                 if (dr > ce->r) {
    //                     continue;
    //                 }

    //                 double midx = (edge.left->x + ce->x) / 2;
    //                 double midy = (edge.left->y + ce->y) / 2;
    //                 double dist = sqrt((edge.left->x - ce->x) * (edge.left->x - ce->x)
    //                                    + (edge.left->y - ce->y) * (edge.left->y - ce->y));

    //                 CircleEvent* newCe = new CircleEvent{midx, midy, dist, true};

    //                 pq.push({midx, midy + dist, nullptr, newCe});
    //                 // edge.right = ce->site;
    //                 edge.m = (edge.right->x - edge.left->x) / (edge.left->y - edge.right->y);
    //                 edge.b = midy - edge.m * midx;
    //             }
    //             ce->valid = false;
    //         }
    //     }
    //     done_all_edges = true;
    // }

    // void Voronoi::computeVoronoi(std::vector<Site*>& sites) {
    //     computeEdges(sites);
    //     for (Edge& edge : edges) {
    //         if (!edge.right) {
    //             edge.end = new Site{edge.left->x + 1000, edge.m * (edge.left->x + 1000) + edge.b};
    //         }
    //         else {
    //             edge.start = new Site{edge.left->x + 1000, edge.m * (edge.left->x + 1000) + edge.b};
    //         }
    //     }
    // }

    // std::vector<Edge> Voronoi::get_edges() {
    //     return edges;
    // }

    // bool Voronoi::has_made_all_edges() {
    //     return done_all_edges;
    // }

}  // namespace utility::voronoi
