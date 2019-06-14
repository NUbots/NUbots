#include "GreenHorizonDetector.h"
#include <set>
#include <vector>
#include "extension/Configuration.h"

#include "message/vision/GreenHorizon.h"

namespace module {
namespace vision {

    using extension::Configuration;

    using GreenHorizonMsg = message::vision::GreenHorizon;

    static constexpr int LINE_INDEX = 2;
    static constexpr int FIELD_INDEX = 3;

    GreenHorizonDetector::GreenHorizonDetector(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("GreenHorizonDetector.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file GreenHorizonDetector.yaml
            config.seed_confidence = config["seed_confidence"].as<float>;
            config.end_confidence = config["end_confidence"].as<float>;
            config.cluster_points = config["cluster_points"].as<int>;
        });

        on<Trigger<VisualMesh>>().then("Green Horizon", [this](const VisualMesh& mesh){

            // Convinence variables
            const auto& cls = mesh.classification;
            const auto& neighbours = mesh.classification;
            const auto& coords = mesh.coordinates;

            // List of clusters
            std::vector<std::vector<int>> clusters;

            std::set<int> visited;
            for (int i = 0; i < mesh.indices.size(); ++i) {
                if (!visited.count(i)) {

                    // Visiting this point
                    visited.insert(i);

                    // If this pont is confident enough to be a seed point breadth first search from here
                    if(cls(i,LINE_INDEX) + cls(i,FIELD_INDEX) > seedConfidence) {

                        // Start a new cluster
                        std::vector<int> cluster;
                        std::vector<int> stack;

                        // Start with this point
                        stack.insert(i);

                        while(!stack.empty()) {

                            // Grab the next point
                            idx = stack.back();
                            stack.pop_back();

                            // If it's good enough continue our search
                            if(cls(idx,LINE_INDEX) + cls(idx,FIELD_INDEX) > endConfidence) {
                                cluster.push_back(idx);

                                // Skip the first point as it's always just us
                                for(int j = 1; j< 7; ++j) {
                                    int idx = neighbours(i,j);
                                    if(!visited.count(idx)) {
                                        visited.insert(idx);
                                        stack.insert(idx)
                                    }
                                }
                            }
                        }

                        clusters.emplace_back(std::move(cluster));
                    }
                }
            }

            // TODO take the clusters and work out which ones we want to keep
            clusters.erase_if([](const std::vector<int>& cluster) {
                return cluster.size() < config.clusterPoints;
            });

            // Sort clusters by size
            std::sort(clusters.begin(), clusters.end(), [] (const auto& a, const auto& b) {
                return a.size() < b.size();
            });



            // Find overlapping elements
            for (auto it = clusters.begin(); it != clusters.end()) {

                // Get the largest and smallest X values
                auto minmax = std::minmax_element(it->begin(), it->end(),[] (const auto& a, const auto& b) {
                    return coords(a,0) < coords(b,0);
                });

                for (auto it2 = std::next(it); it != clusters.end()) {

                    // Get the largest and smallest X values
                    auto minmax = std::minmax_element(it->begin(), it->end(),[] (const auto& a, const auto& b) {
                        return coords(a,0) < coords(b,0);
                    });
                }
            }




            ///

            std::vector<int> cluster = clusters[0];
            for (auto& c : clusters) {
                if(c.size() > cluster.size()){
                    cluster = c;
                }
            }
            std::sort(cluster.begin(), cluster.end(), [&] (const int& a, const int& b) {
                return coords(a,0) < coords(b,0);
            });

            auto msg       = std::make_unique<GreenHorizonMsg>();
            msg->cluster_indices = cluster;

            // TODO do an upper convex hull on the clusters
            for (auto a = cluster.begin(); a + 2 < cluster.end();) {

                auto a_coords = std::pair<float, float>(coords(a, 0), coords(a, 1));
                auto b_coords = std::pair<float, float>(coords(a + 1, 0), coords(a + 1, 1));
                auto c_coords = std::pair<float, float>(coords(a + 2, 0), coords(a + 2, 1));

                // Get the Z component of a cross product to check if it is concave
                bool concave = 0 < (a_coords.first - b_coords.first) * (c_coords.second - b_coords.second)
                                       - (a_coords.second - b_coords.second) * (c_coords.first - b_coords.first);

                if (concave) {
                    cluster.erase(b);
                    a = a == cluster.begin() ? a : --a;
                }
                else {
                    ++a;
                }
            }

            msg->horizon_indices = cluster;

            emit(std::move(msg));

        });
    }
}
}
