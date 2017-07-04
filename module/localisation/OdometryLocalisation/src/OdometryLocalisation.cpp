#include "OdometryLocalisation.h"

#include "message/input/Sensors.h"
#include "extension/Configuration.h"
#include "message/localisation/FieldObject.h"

#include "message/platform/darwin/DarwinSensors.h"
#include "message/behaviour/Nod.h"
#include "utility/support/eigen_armadillo.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/support/yaml_armadillo.h"
#include "utility/nubugger/NUhelpers.h"

namespace module {
namespace localisation {

	using extension::Configuration;
    using message::input::Sensors;
    using message::localisation::Self;
    using utility::math::matrix::Transform2D;
    using utility::math::matrix::Transform3D;
    using message::platform::darwin::ButtonLeftDown;
    using message::behaviour::Nod;
    using utility::nubugger::graph;

    OdometryLocalisation::OdometryLocalisation(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("OdometryLocalisation.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file OdometryLocalisation.yaml
            localisationOffset = config["localisationOffset"].as<arma::vec>();
        });

        on<Trigger<ButtonLeftDown>, Single, With<Sensors>, Sync<OdometryLocalisation>>().then([this] (
                    const Sensors& sensors
                ) {
            NUClear::log("Localisation Orientation reset. This direction is now forward.");
            emit(std::make_unique<Nod>(true));
            Transform2D Trw = Transform3D(convert<double, 4, 4>(sensors.world)).projectTo2D();
            localisationOffset = Trw;
        });


        on<Trigger<Sensors>, Sync<OdometryLocalisation>, Single>().then("Odometry Loc", [this](const Sensors& sensors){

        	Transform2D Trw = Transform3D(convert<double, 4, 4>(sensors.world)).projectTo2D();
        	Transform2D Twr = Trw.i();

            Transform2D state = localisationOffset.localToWorld(Twr);

            auto selfs = std::make_unique<std::vector<Self>>();
            selfs->push_back(Self());
            selfs->back().locObject.position = convert<double, 2, 1>(state.xy());
            selfs->back().heading = Eigen::Vector2d(std::cos(state.angle()),std::sin(state.angle()));

        	emit(selfs);
        });
    }
}
}
