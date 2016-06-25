#include "RobotFieldLocalisation.h"

#include "message/input/Sensors.h"
#include "message/vision/VisionObjects.h"
#include "message/support/Configuration.h"

namespace module {
namespace localisation {

    using message::support::Configuration;
    using message::input::Sensors;
    using message::vision::Goal;

    RobotFieldLocalisation::RobotFieldLocalisation(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("RobotFieldLocalisation.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file RobotFieldLocalisation.yaml
        });


        on<Trigger<Sensors>>().then("Localisation Field Space" [this] (const Sensors& sensors) {

            // Use the current world to field state we are holding to modify sensors.world and emit that
        });

        on<Every<30, Per<std::chrono::seconds>, Sync<RobotFieldLocalisation>>().then("Robot Localisation Time Update", [this] {

            // Do a time update on the models
        });

        on<Trigger<std::vector<Goal>>, With<FieldDescription>, Sync<RobotFieldLocalisation>>().then("Localisation Goal Update", [this] (const std::vector<Goal>& goals, const FieldDescription& field) {

            // If we have two goals that are left/right
            if(goals.size() == 2) {

                // Split measurement into 2 possibilities and apply
            }

            // We have one ambigous goal
            else if(goals.size() == 1) {

                // Split measurement into 4 possibilities and apply
            }
        });
    }
}
}
