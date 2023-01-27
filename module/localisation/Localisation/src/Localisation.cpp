#include "Localisation.hpp"

#include "extension/Configuration.hpp"

namespace module::localisation {

    using extension::Configuration;

    using VisionGoal  = message::vision::Goal;
    using VisionGoals = message::vision::Goals;
    using message::support::FieldDescription;
    using VisionLines = message::vision::FieldLines;

    using utility::math::coordinates::reciprocalSphericalToCartesian;
    using utility::nusight::graph;


    Localisation::Localisation(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("Localisation.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Localisation.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Every<TIME_UPDATE_FREQUENCY, Per<std::chrono::seconds>>>().then("Time Update", [this]() {
            /* Perform time update */

            // TODO: One option here is to just use our walk command * dt as our motion model with a tunable parameter
        });

        on<Trigger<VisionLines>, With<FieldDescription>>().then(
            "Vision Lines",
            [this](const VisionLines& line_points, const FieldDescription& fd) {
                /* Perform measurement correction */

                // Identify the line points on the field
                for (auto point : line_points.points) {
                    Eigen::Isometry3d Hcw = Eigen::Isometry3d(line_points.Hcw.cast<double>());
                    auto rPCw             = ray2cartesian(point.cast<double>(), Hcw);
                    emit(graph("Field points:", rPCw.x(), rPCw.y(), rPCw.z()));
                }

                // TODO: Figure out how to incorporate these points into a measurement model.
                // Grid map?
                // Visual Mesh type grid map

                // Plug into particle filter

                // Profit.
            });
    }

    /// @brief Converts a ray from the camera to a point on the field
    /// @param uPCc ray from the camera to the field point
    /// @param Hcw the camera to world transform
    /// @return the field point in world coordinates
    Eigen::Vector3d Localisation::ray2cartesian(Eigen::Vector3d uPCc, Eigen::Isometry3d Hcw) {
        // Calculate the position of the field point relative to the robot
        auto Hwc  = Hcw.inverse();
        auto rWCw = -Hwc.translation();
        auto rPCw = uPCc * (rWCw.z() / uPCc.z());
        return rPCw;
    }

}  // namespace module::localisation
