#include "Voronoi.hpp"

#include "message/platform/webots/messages.hpp"

#include "extension/Configuration.hpp"
#include "utility/voronoi/Voronoi.hpp"

namespace module::support {

using extension::Configuration;
//using utility::voronoi;
using message::platform::webots::SensorMeasurements;
using message::platform::webots::LocalisationGroundTruth;

    Voronoi::Voronoi(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("Voronoi.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Voronoi.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.robots_each_side = config["number_of_robots_per_team"].as<size_t>();
        });

        on<Trigger<LocalisationGroundTruth>>().then([this](const LocalisationGroundTruth lgt) {
            std::vector<Eigen::Vector3f> team_A{};
            std::vector<Eigen::Vector3f> team_B{};

            //for(auto& i : lgt.rRFf) {
            for(size_t i = 0; i < lgt.rRFf.size(); i++) {
                if(i < cfg.robots_each_side){
                    NUClear::log<NUClear::DEBUG>("Team A Robot ", i+1, " position ", Eigen::Vector3f(lgt.rRFf[i]).transpose());
                    team_A.emplace_back(Eigen::Vector3f(lgt.rRFf[i]));
                }
                else {
                    NUClear::log<NUClear::DEBUG>("Team B Robot ", i+1, " position ", Eigen::Vector3f(lgt.rRFf[i]).transpose());
                    team_B.emplace_back(Eigen::Vector3f(lgt.rRFf[i]));
                }
            }

            // calculate voronoi from team_A and team_B

            // std::vector<utility::voronoi::Site*> sites = {
            //     new utility::voronoi::Site(0, 0),
            //     new utility::voronoi::Site(1, 1),
            //     new utility::voronoi::Site(2, 2),
            //     new utility::voronoi::Site(3, 1),
            //     new utility::voronoi::Site(4, 0)
            // };

            // utility::voronoi::Voronoi voronoi(sites);
            // std::vector<utility::voronoi::Edge*> edges = voronoi.compute();

            // for (auto edge : edges) {
            //     log<NUClear::DEBUG>("(", edge->a->x, ", ", edge->a->y, ") - (", edge->b->x, ", ", edge->b->y, ")");
            // }


            // Emit plotjuggler messages to visualise the voronoi diagram
            // emit(graph("x point", x));

        });

        //NUClear::log<NUClear::DEBUG>("Team A Robot 1 position ", team_A[0].transpose());

    }

}  // namespace module::support
