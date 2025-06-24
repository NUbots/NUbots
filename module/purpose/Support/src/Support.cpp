#include "Support.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/purpose/Player.hpp"
#include "message/strategy/WalkToFieldPosition.hpp"
#include "message/support/FieldDescription.hpp"

#include "utility/math/euler.hpp"

namespace module::purpose {

    using extension::Configuration;

    using SupportMsg = message::purpose::Support;

    using message::localisation::Ball;
    using message::localisation::Field;
    using message::strategy::WalkToFieldPosition;
    using message::support::FieldDescription;

    using utility::math::euler::pos_rpy_to_transform;

    Support::Support(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Support.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Support.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Provide<SupportMsg>, With<Ball>, With<Field>, With<FieldDescription>>().then(
            [this](const Ball& ball, const Field& field, const FieldDescription& fd) {
                // Get ball in field coordinates
                Eigen::Vector3d rBFf = field.Hfw * ball.rBWw;

                // Keep in line with the ball on the x-axis, but stay on the other side of the field on the y-axis
                Eigen::Vector3d position = rBFf;
                position.y()             = rBFf.y() < 0 ? rBFf.y() + (fd.dimensions.field_width / 2)
                                                        : rBFf.y() - (fd.dimensions.field_width / 2);

                // Walk to the position
                emit<Task>(
                    std::make_unique<WalkToFieldPosition>(pos_rpy_to_transform(position, Eigen::Vector3d(0, 0, -M_PI)),
                                                          true));
            });
    }

}  // namespace module::purpose
