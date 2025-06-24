#include "Support.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/localisation/Ball.hpp"
#include "message/purpose/Player.hpp"

namespace module::purpose {

    using extension::Configuration;

    using message::localisation::Ball;
    using message::purpose::Support;

    Support::Support(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Support.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Support.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Provide<Support>, With<Ball>, With<Field>, With<FieldDescription>>.then(
            [this](const Ball& ball, const Field& field, const FieldDescription& fd) {
                // Get ball in field coordinates
                Eigen::Vector3d rBFf = field.Hfw * ball.rBWw;

                // Keep in line with the ball on the x-axis, but stay on the other side of the field on the y-axis
                double y_position        = rBFf.y() < 0 ? rBFf.y() + cfg.y_offset : -fd.field_length / 2.0;
                Eigen::Vecotr3d position = Eigen::Vector3d(rBFf.x(), fd.field_length / 2.0, 0.0);
            });
    }

}  // namespace module::purpose
