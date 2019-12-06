#include "OdometryLocalisation.h"

#include "extension/Configuration.h"
#include "message/behaviour/Nod.h"
#include "message/input/Sensors.h"
#include "message/localisation/Field.h"
#include "message/platform/darwin/DarwinSensors.h"
#include "utility/nusight/NUhelpers.h"
#include "utility/support/yaml_armadillo.h"

namespace module {
namespace localisation {

    using extension::Configuration;
    using message::behaviour::Nod;
    using message::input::Sensors;
    using message::localisation::Field;
    using message::platform::darwin::ButtonLeftDown;
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
                Eigen::Affine2d Trw = sensors.Htw.projectTo2D();
                localisationOffset  = Trw;
            });


        on<Trigger<Sensors>, Sync<OdometryLocalisation>, Single>().then("Odometry Loc", [this](const Sensors& sensors) {
            Eigen::Affine2d Trw = sensors.Htw.projectTo2D();
            Eigen::Affine2d Twr = Trw.i();

            Eigen::Affine2d state = localisationOffset.localToWorld(Twr);

            auto field        = std::make_unique<Field>();
            field->position   = Eigen::Vector3d(state.x(), state.y(), state.angle());
            field->covariance = Eigen::Matrix3d::Identity();

            emit(std::make_unique<std::vector<Field>>(1, *field));
            emit(field);
        });
    }
}  // namespace localisation
}  // namespace module
