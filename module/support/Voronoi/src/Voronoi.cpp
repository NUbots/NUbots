#include "Voronoi.hpp"

#include "extension/Configuration.hpp"

#include "message/platform/webots/messages.hpp"

#include "utility/voronoi/Voronoi.hpp"

namespace module::support {

    using extension::Configuration;
    using message::platform::webots::LocalisationGroundTruth;
    using message::platform::webots::SensorMeasurements;

    Voronoi::Voronoi(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("Voronoi.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Voronoi.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.robots_each_side = config["number_of_robots_per_team"].as<size_t>();
        });

        on<Trigger<LocalisationGroundTruth>>().then([this](const LocalisationGroundTruth lgt) {
            std::vector<Eigen::Vector3f> team_A{};
            std::vector<Eigen::Vector3f> team_B{};

            // for(auto& i : lgt.rRFf) {
            for (size_t i = 0; i < lgt.rRFf.size(); i++) {
                if (i < cfg.robots_each_side) {
                    // NUClear::log<NUClear::DEBUG>("Team A Robot ", i+1, " position ",
                    // Eigen::Vector3f(lgt.rRFf[i]).transpose());
                    team_A.emplace_back(Eigen::Vector3f(lgt.rRFf[i]));
                }
                else {
                    // NUClear::log<NUClear::DEBUG>("Team B Robot ", i+1, " position ",
                    // Eigen::Vector3f(lgt.rRFf[i]).transpose());
                    team_B.emplace_back(Eigen::Vector3f(lgt.rRFf[i]));
                }
            }

            // calculate voronoi from team_A and team_B

            // std::vector<utility::voronoi::Site*> sites;  // = {
            // for (int i = 0; i < team_A.size(); i++) {
            //     sites.push_back(new utility::voronoi::Site(team_A[i].x(), team_A[i].y()));
            //     NUClear::log<NUClear::DEBUG>("Robot", i, "Position", team_A[i].x(), ",", team_A[i].y());
            // }

            // sites.push_back(new utility::voronoi::Site{0, 0});
            // sites.push_back(new utility::voronoi::Site{1, 1});
            // sites.push_back(new utility::voronoi::Site{2, 2});
            // sites.push_back(new utility::voronoi::Site{3, 3});

            // on<Trigger<>, Single>().then([this] {
            // utility::voronoi::Voronoi vd;
            // vd.computeVoronoi(sites);
            // std::vector<utility::voronoi::Edge> edges = vd.get_edges();
            // NUClear::log<NUClear::DEBUG>(" Edges", edges.size());
            // int n = 0;
            // if (vd.has_made_all_edges() && !sites.empty()) {
            //     NUClear::log<NUClear::DEBUG>("Edge:", n);
            //     for (const utility::voronoi::Edge& edge : edges) {
            //         NUClear::log<NUClear::DEBUG>("Edge: ");
            //         NUClear::log<NUClear::DEBUG>("(", edge.start->x, ", ", edge.start->y, ")");
            //         NUClear::log<NUClear::DEBUG>(" to ");
            //         NUClear::log<NUClear::DEBUG>("(", edge.end->x, ", ", edge.end->y, ")");

            //         // delete edge.start;
            //         // delete edge.end;
            //         n++;
            //     }
            // }


            // Emit plotjuggler messages to visualise the voronoi diagram
            // emit(graph("x point", x));
        });

        // NUClear::log<NUClear::DEBUG>("Team A Robot 1 position ", team_A[0].transpose());
    }

}  // namespace module::support
