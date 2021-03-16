#include "OdometryLocalisation.hpp"

#include "extension/Configuration.hpp"

#include "message/behaviour/Nod.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/Field.hpp"
#include "message/platform/darwin/DarwinSensors.hpp"

#include "utility/math/matrix/Transform3D.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/eigen_armadillo.hpp"
#include "utility/support/yaml_armadillo.hpp"

namespace module {
namespace localisation {

    using extension::Configuration;
    using message::behaviour::Nod;
    using message::input::Sensors;
    using message::localisation::Field;
    using message::platform::darwin::ButtonLeftDown;
    using utility::localisation::projectTo2D;
    using utility::nusight::graph;
    using utility::support::Expression;

    OdometryLocalisation::OdometryLocalisation(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("OdometryLocalisation.yaml").then([this](const Configuration& config) {
            // Use configuration here from file OdometryLocalisation.yaml
            localisationOffset = config["localisationOffset"].as<Expression>();
        });

        on<Trigger<ButtonLeftDown>, Single, With<Sensors>, Sync<OdometryLocalisation>>().then(
            [this](const Sensors& sensors) {
                NUClear::log("Localisation Orientation reset. This direction is now forward.");
                emit(std::make_unique<Nod>(true));
                localisationOffset = projectTo2D(sensors.Htw);  // (= Hrw)
            });


        on<Trigger<Sensors>, Sync<OdometryLocalisation>, Single>().then("Odometry Loc", [this](const Sensors& sensors) {
            Eigen::Affine2d Hrw = projectTo2D(sensors.Htw);

            // Local to world transform
            Eigen::Affine2d state = Hrw.inverse() * localisationOffset;

            auto field        = std::make_unique<Field>();
            field->position   = state.matrix();
            field->covariance = Eigen::Matrix3d::Identity();

            emit(std::make_unique<std::vector<Field>>(1, *field));
            emit(field);
        });
    }
}  // namespace localisation
}  // namespace module
