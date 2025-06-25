#include "Support.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/purpose/Player.hpp"
#include "message/strategy/WalkToFieldPosition.hpp"
#include "message/support/FieldDescription.hpp"

#include "utility/math/euler.hpp"

namespace module::purpose {

    using extension::Configuration;

    using SupportMsg = message::purpose::Support;

    using message::input::Sensors;
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

        on<Provide<SupportMsg>, With<Ball>, With<Sensors>, With<Field>, With<FieldDescription>>().then(
            [this](const Ball& ball, const Sensors& sensors, const Field& field, const FieldDescription& fd) {
                // Get ball in field coordinates
                Eigen::Vector3d rBFf = field.Hfw * ball.rBWw;
                Eigen::Vector3d rBRr = sensors.Hrw * ball.rBWw;

                // Keep in line with the ball on the x-axis, but stay on the other side of the field on the y-axis
                Eigen::Vector3d position = rBFf;
                position.y()             = rBRr.y() < 0 ? rBFf.y() + (fd.dimensions.field_width / 4)
                                                        : rBFf.y() - (fd.dimensions.field_width / 4);

                // If there isn't really space to walk to the desired side, walk to the other side
                if (position.y() > fd.dimensions.field_width / 2) {
                    position.y() = rBFf.y() - (fd.dimensions.field_width / 4);
                }
                else if (position.y() < -fd.dimensions.field_width / 2) {
                    position.y() = rBFf.y() + (fd.dimensions.field_width / 4);
                }

                // Bound x by the penalty areas
                double penalty_area = fd.dimensions.field_length / 2 - fd.dimensions.penalty_area_length;
                position.x()        = std::clamp(position.x(), -penalty_area, penalty_area);
                // Closest side, unless it wont fit
                // Walk to the position
                emit<Task>(
                    std::make_unique<WalkToFieldPosition>(pos_rpy_to_transform(position, Eigen::Vector3d(0, 0, M_PI)),
                                                          true));
            });
    }

}  // namespace module::purpose
