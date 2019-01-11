#include "StaticWalk.h"

#include "extension/Configuration.h"
#include "message/input/Sensors.h"
#include "message/motion/FootTarget.h"
#include "message/motion/TorsoTarget.h"
#include "message/motion/WalkCommand.h"
#include "utility/behaviour/Action.h"
#include "utility/input/LimbID.h"
#include "utility/input/ServoID.h"

namespace module {
namespace motion {
    namespace walk {

        using extension::Configuration;
        using message::input::Sensors;
        using message::motion::FootTarget;
        using message::motion::TorsoTarget;
        using message::motion::WalkCommand;
        using utility::behaviour::RegisterAction;
        using utility::input::LimbID;
        using utility::input::ServoID;

        StaticWalk::StaticWalk(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment)), subsumptionId(size_t(this) * size_t(this) - size_t(this)) {

            on<Configuration>("StaticWalk.yaml").then([this](const Configuration& config) {
                // Use configuration here from file StaticWalk.yaml
                double x_speed = config["x_speed"].as<double>();
                double y_speed = config["y_speed"].as<double>();
                double angle   = config["angle"].as<double>();
                torso_height   = config["torso_height"].as<double>();
                feet_distance  = config["feet_distance"].as<double>();
                phase_time     = std::chrono::seconds(config["phase_time"].as<int>());
                start_phase    = NUClear::clock::now();
                state          = INITIAL;
                emit(std::make_unique<WalkCommand>(subsumptionId, Eigen::Vector3d(x_speed, y_speed, angle)));
            });

            // on<ResetWalk>().then([this] {
            //     state       = INITIAL;
            //     start_phase = NUClear::clock::now();
            // });

            on<Trigger<Sensors>, With<WalkCommand>>().then(
                [this](const Sensors& sensors, const WalkCommand& walkcommand) {
                    if (state == INITIAL) {
                        Hff_w = (sensors.forwardKinematics[ServoID::R_ANKLE_ROLL]).inverse()
                                * (sensors.forwardKinematics[ServoID::L_ANKLE_ROLL]);
                        Hff_w.translation().z() = 0;
                        state                   = LEFT_LEAN;
                    }

                    // Timer is up, go to next phase
                    if (NUClear::clock::now() > start_phase + phase_time) {
                        start_phase = NUClear::clock::now();
                        switch (state) {
                            case LEFT_LEAN: state = RIGHT_STEP; break;
                            case RIGHT_STEP: {
                                // Store where support is relative to swing ignoring height
                                Hff_w = (sensors.forwardKinematics[ServoID::L_ANKLE_ROLL]).inverse()
                                        * (sensors.forwardKinematics[ServoID::R_ANKLE_ROLL]);
                                Hff_w.translation().z() = 0;
                                state                   = RIGHT_LEAN;
                            } break;
                            case RIGHT_LEAN: state = LEFT_STEP; break;
                            case LEFT_STEP: {
                                // Store where support is relative to swing ignoring height
                                Hff_w = (sensors.forwardKinematics[ServoID::R_ANKLE_ROLL]).inverse()
                                        * (sensors.forwardKinematics[ServoID::L_ANKLE_ROLL]);
                                Hff_w.translation().z() = 0;
                                state                   = LEFT_LEAN;
                            } break;
                        }
                    }

                    // Put our COM over the correct foot or move foot to target
                    switch (state) {
                        case LEFT_LEAN: {
                            // Support foot to torso transform
                            Eigen::Affine3d Htf(sensors.forwardKinematics[ServoID::L_ANKLE_ROLL]);

                            // move COM to foot
                            // move torso to foot - COM
                            // distance from torso to COM
                            // (0,0,z) - rCTt

                            // Create support foot to target vector
                            Eigen::Vector3d rAFf(0, 0, torso_height);

                            // Create matrix for TorsoTarget
                            Eigen::Affine3d Haf;
                            Haf.linear()      = Htf.linear();
                            Haf.translation() = -rAFf;

                            // COM over foot
                            emit(std::make_unique<TorsoTarget>(
                                start_phase + phase_time, false, Haf.matrix(), subsumptionId));

                            // // Maintain foot position
                            emit(std::make_unique<FootTarget>(
                                start_phase + phase_time, true, Hff_w.matrix(), false, subsumptionId));


                        } break;
                        case RIGHT_LEAN: {
                            // Support foot to torso transform
                            Eigen::Affine3d Htf(sensors.forwardKinematics[ServoID::R_ANKLE_ROLL]);

                            // move COM to foot
                            // move torso to foot - COM
                            // distance from torso to COM
                            // (0,0,z) - rCTt

                            // Create support foot to target vector
                            Eigen::Vector3d rAFf(0, 0, torso_height);  // - Htc.translation();

                            // Create matrix for TorsoTarget
                            Eigen::Affine3d Haf;

                            Haf.linear()      = Htf.linear();
                            Haf.translation() = -rAFf;

                            emit(std::make_unique<TorsoTarget>(
                                start_phase + phase_time, true, Haf.matrix(), subsumptionId));

                            // Maintain foot position
                            emit(std::make_unique<FootTarget>(
                                start_phase + phase_time, false, Hff_w.matrix(), false, subsumptionId));


                        } break;
                        case RIGHT_STEP: {
                            // walkcommand is (x,y,theta) where x,y is velocity in m/s and theta is angle in
                            // radians/seconds
                            Eigen::Affine3d Haf;
                            Haf.linear()      = Eigen::Matrix3d::Identity();  // * walkcommand.command.z() / phase_time;
                            Haf.translation() = -Eigen::Vector3d(
                                walkcommand.command.x() * 2 / (phase_time.count() / 1000000000),
                                (walkcommand.command.y() * 2 / (phase_time.count() / 1000000000)) - feet_distance,
                                0);
                            log(Haf.translation().x(), phase_time.count());

                            emit(std::make_unique<FootTarget>(
                                start_phase + phase_time, true, Haf.matrix(), true, subsumptionId));
                        } break;
                        case LEFT_STEP: {
                            // walkcommand is (x,y,theta) where x,y is velocity in *metres/second* and theta is angle in
                            // radians/seconds
                            Eigen::Affine3d Haf;
                            Haf.linear()      = Eigen::Matrix3d::Identity();  // * walkcommand.command.z() / phase_time;
                            Haf.translation() = -Eigen::Vector3d(
                                walkcommand.command.x() * 2 / (phase_time.count() / 1000000000),
                                (walkcommand.command.y() * 2 / (phase_time.count() / 1000000000)) + feet_distance,
                                0);

                            emit(std::make_unique<FootTarget>(
                                start_phase + phase_time, false, Haf.matrix(), true, subsumptionId));
                        } break;
                        case INITIAL: break;
                    }
                });

            emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(
                RegisterAction{subsumptionId,
                               "StaticWalk",
                               {std::pair<float, std::set<LimbID>>(10, {LimbID::LEFT_LEG, LimbID::RIGHT_LEG})},
                               [this](const std::set<LimbID>&) {},
                               [this](const std::set<LimbID>&) {},
                               [this](const std::set<ServoID>& servoSet) {}}));
        }
    }  // namespace walk
}  // namespace motion
}  // namespace module
