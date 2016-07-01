#include "OdometryLocalisation.h"

#include "message/input/Sensors.h"
#include "message/support/Configuration.h"
#include "message/localisation/FieldObject.h"

#include "message/platform/darwin/DarwinSensors.h"
#include "message/behaviour/Nod.h"

#include "utility/support/yaml_armadillo.h"
#include "utility/support/yaml_expression.h"

namespace module {
namespace localisation {

	using message::support::Configuration;
    using message::input::Sensors;
    using message::localisation::Self;
    using utility::math::matrix::Transform2D;
    using message::platform::darwin::ButtonLeftDown;
    using message::behaviour::Nod;

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
            Transform2D Trw = sensors.world.projectTo2D();
            localisationOffset = Trw.i();
        });


        on<Trigger<Sensors>, Sync<OdometryLocalisation>, Single>().then("Odometry Loc", [this](const Sensors& sensors){
        	
        	Transform2D Trw = sensors.world.projectTo2D();
        	Transform2D Twr = Trw.i();
        	
            if(arma::norm(Twr.localToWorld(Trw)) > 0.00001){
                log("arma::norm(Twr.localToWorld(Trw))",Trw.t(),Twr.t(),Twr.localToWorld(Trw).t());
            }
            if(arma::norm(Trw.localToWorld(Twr)) > 0.00001){
                log("arma::norm(Trw.localToWorld(Twr))",Trw.t(),Twr.t(),Trw.localToWorld(Twr).t());
            }

            Transform2D state = localisationOffset.localToWorld(Twr);

            auto selfs = std::make_unique<std::vector<Self>>();
            selfs->push_back(Self());
            selfs->back().position = state.xy();
            selfs->back().heading = arma::vec2({std::cos(state.angle()),std::sin(state.angle())});
            // log("sensors world",Twr.t());
        	emit(selfs);
        });
    }
}
}
