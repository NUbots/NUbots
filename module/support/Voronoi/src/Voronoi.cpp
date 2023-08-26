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
        });

        // NUClear::log<NUClear::DEBUG>("Team A Robot 1 position ", team_A[0].transpose());
    }

}  // namespace module::support
