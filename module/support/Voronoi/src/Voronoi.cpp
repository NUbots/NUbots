#include "Voronoi.hpp"

#include <CGAL/Simple_cartesian.h>

#include "extension/Configuration.hpp"

#include "message/platform/webots/messages.hpp"

#include "utility/voronoi/Voronoi.hpp"

// typedef CGAL::Simple_cartesian<double> Kernel;
// typedef Kernel::Point_2 Point_2;
// typedef Kernel::Segment_2 Segment_2;


namespace module::support {

    using extension::Configuration;
    using message::platform::webots::LocalisationGroundTruth;
    using message::platform::webots::OdometryGroundTruth;
    using message::platform::webots::SensorMeasurements;

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
            NUClear::log<NUClear::WARN>("Triggered");

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
                    NUClear::log<NUClear::DEBUG>("Voronoi Team B Robot ",
                                                 i + 1,
                                                 " position ",
                                                 Eigen::Vector3f(lgt.rRFf[i]).transpose());
                    team_B.emplace_back(Eigen::Vector3f(lgt.rRFf[i]));
                }
            }


            // Point_2 p(1, 1), q(10, 10);
            // std::cout << "p = " << p << std::endl;
            // std::cout << "q = " << q.x() << " " << q.y() << std::endl;
            // std::cout << "sqdist(p,q) = " << CGAL::squared_distance(p, q) << std::endl;
            // Segment_2 s(p, q);
            // Point_2 m(5, 9);
            // std::cout << "m = " << m << std::endl;
            // std::cout << "sqdist(Segment_2(p,q), m) = " << CGAL::squared_distance(s, m) << std::endl;
            // std::cout << "p, q, and m ";
            // switch (CGAL::orientation(p, q, m)) {
            //     case CGAL::COLLINEAR: std::cout << "are collinear\n"; break;
            //     case CGAL::LEFT_TURN: std::cout << "make a left turn\n"; break;
            //     case CGAL::RIGHT_TURN: std::cout << "make a right turn\n"; break;
            // }
            // std::cout << " midpoint(p,q) = " << CGAL::midpoint(p, q) << std::endl;
        });

        on<Trigger<LocalisationGroundTruth>>().then([this](const LocalisationGroundTruth lgt) {
            NUClear::log<NUClear::WARN>("Robot Positions ", lgt.rRFf[0].transpose());
        });

    }  // namespace module::support
}  // namespace module::support
