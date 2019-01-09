#include "StaticWalk.h"

#include "extension/Configuration.h"
#include "message/motion/FootTarget.h"
#include "message/motion/TorsoTarget.h"

#include "utility/input/LimbID.h"
#include "utility/input/ServoID.h"

namespace module {
namespace motion {
    namespace walk {

        using extension::Configuration;

        using message::motion::FootTarget;
        using message::motion::TorsoTarget;
        using utility::behaviour::RegisterAction;
        using utility::input::LimbID;
        using utility::input::ServoID;

        StaticWalk::StaticWalk(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment)), subsumptionId(size_t(this) * size_t(this) - size_t(this)) {

            on<Configuration>("StaticWalk.yaml").then([this](const Configuration& config) {
                // Use configuration here from file StaticWalk.yaml
                phase_time = config["phase_time"].as<double>();
                start_time = NUClear::clock::now();
                auto swing = std::make_unique<FootTarget>();
                auto torso = std::make_unique<TorsoTarget>();
                emit()
            });

            on<ResetWalk>().then([this] {
                state      = LEFT_LEAN;
                start_time = NUClear::clock::now();
            });

            on<Trigger<Sensors>, With<WalkCommand>>().then(
                [this](const Sensors& sensors, const WalkCommand& walkcommand) {
                    // Timer is up, go to next phase
                    if (NUClear::clock::now() > start_phase + phase_time) {
                        start_phase = NUClear::clock::now();
                        switch (state) {
                            case LEFT_LEAN: state = RIGHT_STEP; break;
                            case RIGHT_STEP: state = RIGHT_LEAN; break;
                            case RIGHT_LEAN: state = LEFT_STEP; break;
                            case LEFT_STEP: state = LEFT_LEAN; break;
                        }
                    }

                    // Put our COM over the correct foot or move foot to target
                    switch (state) {
                        case LEFT_LEAN: {
                            // Support foot to torso transform
                            Eigen::Affine3d Htf = Eigen::Affine3d(sensors.forwardKinematics[ServoID::L_ANKLE_ROLL]);

                            // move COM to foot
                            // move torso to foot - COM
                            // distance from torso to COM
                            // (0,0,z) - rCTt

                            // Create foot to target vector
                            rAFf = Eigen::Vector3d(0, 0, height) - Htc.translation();

                            Haf.linear()      = Htf.linear();
                            Haf.translation() = -rAFf;

                            emit(std::make_unique<TorsoTarget>(start_phase + phase_time, false, Haf.matrix()));
                        } break;
                        case RIGHT_LEAN: {
                            // Support foot to torso transform
                            Eigen::Affine3d Htf = Eigen::Affine3d(sensors.forwardKinematics[ServoID::R_ANKLE_ROLL]);

                            // move COM to foot
                            // move torso to foot - COM
                            // distance from torso to COM
                            // (0,0,z) - rCTt

                            // Create foot to target vector
                            rAFf = Eigen::Vector3d(0, 0, height) - Htc.translation();

                            Haf.linear()      = Htf.linear();
                            Haf.translation() = -rAFf;

                            emit(std::make_unique<TorsoTarget>(start_phase + phase_time, false, Haf.matrix()));
                        } break;
                        case RIGHT_STEP: {
                            // walkcommand is (x,y,theta) where x,y is
                            // velocity in m/s and theta is angle in radians/seconds
                            Eigen::Affine3d Haf;
                            Haf.linear() = Eigen::Affine3d::Identity() * walkcommand.command.z() / phase_time;
                            Haf.translation() =
                                -(walkcommand.command.x() / phase_time, walkcommand.command.y() / phase_time, 0);

                            emit(std::make_unique<FootTarget>(start_phase + phase_time, true, Haf));
                        } break;
                        case LEFT_STEP: {
                            // walkcommand is (x,y,theta) where x,y is
                            // velocity in *metres/second* and theta is angle in
                            // *radians/seconds*
                            Eigen::Affine3d Haf;
                            Haf.linear() = Eigen::Affine3d::Identity() * walkcommand.command.z() / phase_time;
                            Haf.translation() =
                                -(walkcommand.command.x() / phase_time, walkcommand.command.y() / phase_time, 0);

                            emit(std::make_unique<FootTarget>(start_phase + phase_time, false, Haf));
                        } break;
                    }
                });

            emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(
                RegisterAction{subsumptionId,
                               "FootStep",
                               {std::pair<float, std::set<LimbID>>(10, {LimbID::LEFT_LEG, LimbID::RIGHT_LEG})},
                               [this](const std::set<LimbID>&) {},
                               [this](const std::set<LimbID>&) {},
                               [this](const std::set<ServoID>& servoSet) {}}));
        }
    }  // namespace walk
}  // namespace motion
}  // namespace module
