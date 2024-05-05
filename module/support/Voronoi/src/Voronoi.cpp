#include "Voronoi.hpp"

#include "boost/polygon/voronoi.hpp"

#include "extension/Configuration.hpp"

#include "message/platform/webots/messages.hpp"

#include "utility/voronoi/Voronoi.hpp"


namespace module::support {

    using extension::Configuration;
    using message::platform::webots::LocalisationGroundTruth;
    using message::platform::webots::OdometryGroundTruth;
    using message::platform::webots::SensorMeasurements;

    using boost::polygon::voronoi_builder;
    using boost::polygon::voronoi_diagram;

    Voronoi::Voronoi(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("Voronoi.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Voronoi.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.robots_each_side = config["number_of_robots_per_team"].as<size_t>();
            NUClear::log<NUClear::WARN>("Config Done");
        });

        on<Trigger<LocalisationGroundTruth>>().then([this](const LocalisationGroundTruth lgt) {
            std::vector<Eigen::Vector3f> team_A{};
            std::vector<Eigen::Vector3f> team_B{};

            // NUClear::log<NUClear::WARN>("Triggered");
            // NUClear::log<NUClear::DEBUG>("Number of Robots ", lgt.rRFf.size());

            // for(auto& i : lgt.rRFf) {
            for (size_t i = 0; i < lgt.rRFf.size(); i++) {
                if (i < cfg.robots_each_side) {
                    NUClear::log<NUClear::WARN>("Voronoi Team A Robot ",
                                                i + 1,
                                                " position ",
                                                Eigen::Vector3f(lgt.rRFf[i]).transpose());
                    team_A.emplace_back(Eigen::Vector3f(lgt.rRFf[i]));
                }
                else {
                    NUClear::log<NUClear::WARN>("Voronoi Team B Robot ",
                                                i + 1,
                                                " position ",
                                                Eigen::Vector3f(lgt.rRFf[i]).transpose());
                    team_B.emplace_back(Eigen::Vector3f(lgt.rRFf[i]));
                }
            }

            // std::vector<utility::voronoi::Point> points = {Point(0, 0), Point(1, 1), Point(2, 2), Point(3, 0)};

            // // Create an instance of the Voronoi diagram
            // voronoi_diagram<double> vd;

            // // Construct the Voronoi diagram from the set of points
            // construct_voronoi(points.begin(), points.end(), &vd);

            // // Print out the edges of the Voronoi diagram
            // for (auto& edge : vd.edges()) {
            //     std::cout << "Edge: " << edge.vertex0() << " -> " << edge.vertex1() << std::endl;
            // }
        });

        // on<Trigger<OdometryGroundTruth>>().then([this](const OdomteryGroundTruth ogt) {
        //     NUClear::log<NUClear::WARN>("Robot Positions ", ogt. [0].transpose());
        // });

    }  // namespace module::support
}  // namespace module::support
