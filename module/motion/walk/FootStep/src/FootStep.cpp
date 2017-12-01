#include "FootStep.h"

#include "extension/Configuration.h"
#include "message/input/Sensors.h"
#include "message/motion/FootTarget.h"

namespace module {
namespace motion {
    namespace walk {

        using extension::Configuration;
        using message::input::Sensors;
        using message::motion::FootTarget;

        FootStep::FootStep(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

            on<Configuration>("FootStep.yaml").then([this](const Configuration& config) {
                // Use configuration here from file FootStep.yaml
            });


            on<Trigger<Sensors>, With<FootTarget>>().then([this](const FootTarget& target, const Sensors& sensors) {

                sensors.world;                                     // Htw
                sensors.forwardKinematics[ServoID::L_ANKLE_ROLL];  // Htf
                sensors.forwardKinematics[ServoID::R_ANKLE_ROLL];  // Htf

                Htf = supportFoot rFTf = swingFoot


                    float footHeight = 7;
                float stepSteep      = 10;
                float d              = 2;


                Hwf = Htw.yawless().i()* Htf  /// Hwt * Htf
                      Hfc
                      / Hcf = [normalise(otherfoot->target), z, cross(...)]


                    Htc

                // Get support foot coordinate system
                // Work out vector field
                // Apply using IK
            });


            on<Every<5, std::chrono::seconds>>().then([this] {
                //
                log("Making new foot target");

                emit(std::make_unique<FootTarget>(
                    NUClear::clock::now() + std::chrono::seconds(1), 0, Eigen::Vector2d(0.1, 0.1)));
            });


            emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(
                RegisterAction{id,
                               "FootStep",
                               {std::pair<float, std::set<LimbID>>(10, {LimbID::LEFT_LEG, LimbID::RIGHT_LEG})},
                               [this](const std::set<LimbID>&) {},
                               [this](const std::set<LimbID>&) {},
                               [this](const std::set<ServoID>& servoSet) {}}));
        }
    }
}
}
