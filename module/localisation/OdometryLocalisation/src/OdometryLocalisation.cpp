#include "OdometryLocalisation.h"

#include "extension/Configuration.h"

#include "message/behaviour/Nod.h"
#include "message/input/Sensors.h"
#include "message/localisation/Field.h"
#include "message/platform/darwin/DarwinSensors.h"

#include "utility/math/matrix/Transform3D.h"
#include "utility/nubugger/NUhelpers.h"
#include "utility/support/eigen_armadillo.h"
#include "utility/support/yaml_armadillo.h"

namespace module {
namespace localisation {

    using extension::Configuration;
    using message::behaviour::Nod;
    using message::input::Sensors;
    using message::localisation::Field;
    using message::platform::darwin::ButtonLeftDown;
    using utility::math::matrix::Transform2D;
    using utility::math::matrix::Transform3D;
    using utility::nubugger::graph;

    OdometryLocalisation::OdometryLocalisation(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("OdometryLocalisation.yaml").then([this](const Configuration& config) {
            // Use configuration here from file OdometryLocalisation.yaml
            localisationOffset = config["localisationOffset"].as<arma::vec>();
        });

        on<Trigger<ButtonLeftDown>, Single, With<Sensors>, Sync<OdometryLocalisation>>().then(
            [this](const Sensors& sensors) {
                NUClear::log("Localisation Orientation reset. This direction is now forward.");
                emit(std::make_unique<Nod>(true));
                Transform2D Trw    = Transform3D(convert<double, 4, 4>(sensors.world)).projectTo2D();
                localisationOffset = Trw;
            });


        on<Trigger<Sensors>, Sync<OdometryLocalisation>, Single>().then("Odometry Loc", [this](const Sensors& sensors) {

            Transform2D Trw = Transform3D(convert<double, 4, 4>(sensors.world)).projectTo2D();
            Transform2D Twr = Trw.i();

            Transform2D state = localisationOffset.localToWorld(Twr);

            auto field        = std::make_unique<Field>();
            field->position   = Eigen::Vector3d(state.x(), state.y(), state.angle());
            field->covariance = Eigen::Matrix3d::Identity();

            emit(std::make_unique<std::vector<Field>>(1, *field));
            emit(field);
        });
    }
}  // namespace localisation
}  // namespace module
