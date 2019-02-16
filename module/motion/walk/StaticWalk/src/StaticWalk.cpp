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
                start_phase = NUClear::clock::now();
                state       = INITIAL;

                // Use configuration here from file StaticWalk.yaml
                torso_height   = config["torso_height"].as<double>();
                stance_width   = config["stance_width"].as<double>();
                foot_offset    = config["foot_offset"].as<double>();
                phase_time     = std::chrono::milliseconds(config["phase_time"].as<int>());
                double x_speed = config["x_speed"].as<double>();
                double y_speed = config["y_speed"].as<double>();
                double angle   = config["angle"].as<double>();

                emit(std::make_unique<WalkCommand>(subsumptionId, Eigen::Vector3d(x_speed, y_speed, angle)));
            });

            on<Trigger<Sensors>, With<WalkCommand>>().then([this](const Sensors& sensors,
                                                                  const WalkCommand& walkcommand) {
                // INITIAL state occurs only as the first state in the walk to set the matrix Hff_s
                if (state == INITIAL) {
                    // Set the matrix Hff_s and change the state to a lean
                    Hff_s = (sensors.forwardKinematics[ServoID::L_ANKLE_ROLL]).inverse()
                            * (sensors.forwardKinematics[ServoID::R_ANKLE_ROLL]);
                    state = RIGHT_LEAN;
                }

                // When the time is over for this phase, begin the next phase
                if (NUClear::clock::now() > start_phase + phase_time) {
                    // Reset the start phase time for the new phase
                    start_phase = NUClear::clock::now();
                    // Change the state of the walk based on what the previous state was
                    switch (state) {
                        case LEFT_LEAN: state = RIGHT_STEP; break;
                        case RIGHT_STEP: {
                            // Store where support is relative to swing
                            Hff_s = (sensors.forwardKinematics[ServoID::L_ANKLE_ROLL]).inverse()
                                    * (sensors.forwardKinematics[ServoID::R_ANKLE_ROLL]);
                            state = RIGHT_LEAN;
                        } break;
                        case RIGHT_LEAN: state = LEFT_STEP; break;
                        case LEFT_STEP: {
                            // Store where support is relative to swing
                            Hff_s = (sensors.forwardKinematics[ServoID::R_ANKLE_ROLL]).inverse()
                                    * (sensors.forwardKinematics[ServoID::L_ANKLE_ROLL]);
                            state = LEFT_LEAN;
                        } break;
                        default: break;
                    }
                }

                // Put our COM over the correct foot or move foot to target, based on which state we are in
                switch (state) {
                    case LEFT_LEAN: {
                        // Create matrix for TorsoTarget
                        Eigen::Affine3d Haf;
                        Haf.linear()      = Eigen::Matrix3d::Identity();
                        Haf.translation() = -Eigen::Vector3d(foot_offset, 0, torso_height);

                        // Move the COM over the left foot
                        emit(std::make_unique<TorsoTarget>(
                            start_phase + phase_time, false, Haf.matrix(), subsumptionId));

                        // Maintain right foot position while the torso moves over the left foot
                        emit(std::make_unique<FootTarget>(
                            start_phase + phase_time, true, Hff_s.matrix(), false, subsumptionId));
                    } break;
                    case RIGHT_LEAN: {
                        // Create matrix for TorsoTarget
                        Eigen::Affine3d Haf;
                        Haf.linear()      = Eigen::Matrix3d::Identity();
                        Haf.translation() = -Eigen::Vector3d(foot_offset, 0, torso_height);

                        // Move the COM over the right foot
                        emit(
                            std::make_unique<TorsoTarget>(start_phase + phase_time, true, Haf.matrix(), subsumptionId));

                        // Maintain left foot position while the torso moves over the right foot
                        emit(std::make_unique<FootTarget>(
                            start_phase + phase_time, false, Hff_s.matrix(), false, subsumptionId));

                    } break;
                    case RIGHT_STEP: {
                        // walkcommand is (x,y,theta) where x,y is velocity in m/s and theta is angle in
                        // radians/seconds
                        Eigen::Affine3d Haf;

                        // If there is no rotation to be done, just set the translation to x and y, and set the rotation
                        // to identity
                        if (walkcommand.command.z() == 0) {
                            Haf.translation() = -Eigen::Vector3d(
                                walkcommand.command.x() * 2
                                    * std::chrono::duration_cast<std::chrono::duration<double>>(phase_time).count(),
                                walkcommand.command.y() * 2
                                        * std::chrono::duration_cast<std::chrono::duration<double>>(phase_time).count()
                                    - stance_width,
                                0);
                            Haf.linear() = Eigen::Matrix3d::Identity();
                        }

                        // If there is rotation, adjust the translation and rotation for this
                        else {
                            // Multiply by 2 so that the lean states are accounted for
                            //  Multiply by phase time so that we are moving in x metres/second and y metres/second
                            Haf.translation() = -Eigen::Vector3d(
                                (walkcommand.command.x() * 2)
                                    * std::chrono::duration_cast<std::chrono::duration<double>>(phase_time).count(),
                                (walkcommand.command.y() * 2)
                                    * std::chrono::duration_cast<std::chrono::duration<double>>(phase_time).count(),
                                0);

                            // Set rotation to given walkcommand
                            double rotation = walkcommand.command.z();
                            // Set origin of the circle
                            Eigen::Vector3d origin(Haf.translation().y(), Haf.translation().x(), 0);
                            origin /= rotation;
                            // Set the angle of the arc
                            double arcLength =
                                rotation
                                * std::chrono::duration_cast<std::chrono::duration<double>>(phase_time).count();
                            // Using the end points of the arc to create one side and radius lines to the center as two
                            // other sides, create a triangle. arcAngle is then the angle of either of the two other
                            // points of the triangle.
                            double arcAngle = (M_PI - arcLength) / 2;

                            // Create a rotation vector which is the vector from foot to origin then rotated about
                            // arcLength to get a vector pointing from the foot to the end of the arc (may not be the
                            // right length)
                            Eigen::Vector3d rAFf;
                            rAFf.x() = origin.x() * cos(arcAngle) - origin.y() * sin(arcAngle);
                            rAFf.y() = origin.x() * sin(arcAngle) + origin.y() * cos(arcAngle);
                            rAFf.z() = 0;

                            // Adjust the length of the vector so that it is the length to the end of the arc
                            rAFf = rAFf.normalized() * ((origin.norm() / sin(arcAngle)) * sin(arcLength));
                            // Adjust the y since this is taken in support foot space, and we want to move the swing
                            // foot
                            rAFf.y() -= stance_width;

                            // Create the matrix to send the foot to the correct target
                            Haf.linear()      = Eigen::Matrix3d::Identity();  // TODO: create the right rotation
                            Haf.translation() = -rAFf;
                        }

                        // Move the left foot to the location specified by the walkcommand
                        emit(std::make_unique<FootTarget>(
                            start_phase + phase_time, true, Haf.matrix(), true, subsumptionId));
                    } break;
                    case LEFT_STEP: {
                        // walkcommand is (x,y,theta) where x,y is velocity in m/s and theta is angle in
                        // radians/seconds
                        Eigen::Affine3d Haf;

                        // If there is no rotation to be done, just set the translation to x and y, and set the rotation
                        // to identity
                        if (walkcommand.command.z() == 0) {
                            // Multiply by 2 so that the lean states are accounted for
                            //  Multiply by phase time so that we are moving in x metres/second and y metres/second
                            Haf.translation() = -Eigen::Vector3d(
                                walkcommand.command.x() * 2
                                    * std::chrono::duration_cast<std::chrono::duration<double>>(phase_time).count(),
                                walkcommand.command.y() * 2
                                        * std::chrono::duration_cast<std::chrono::duration<double>>(phase_time).count()
                                    + stance_width,
                                0);
                            Haf.linear() = Eigen::Matrix3d::Identity();
                        }

                        // If there is rotation, adjust the translation and rotation for this
                        else {
                            Haf.translation() = -Eigen::Vector3d(
                                (walkcommand.command.x() * 2)
                                    * std::chrono::duration_cast<std::chrono::duration<double>>(phase_time).count(),
                                (walkcommand.command.y() * 2)
                                    * std::chrono::duration_cast<std::chrono::duration<double>>(phase_time).count(),
                                0);

                            // Set rotation to given walkcommand
                            double rotation = walkcommand.command.z() * 2;
                            // Set origin of the circle
                            Eigen::Vector3d origin(Haf.translation().y(), Haf.translation().x(), 0);
                            origin /= rotation;
                            // Set the angle of the arc
                            double arcLength =
                                rotation
                                * std::chrono::duration_cast<std::chrono::duration<double>>(phase_time).count();

                            // Using the end points of the arc to create one side and radius lines to the center as two
                            // other sides, create a triangle. arcAngle is then the angle of either of the two other
                            // points of the triangle.
                            double arcAngle = (M_PI - arcLength) / 2;

                            // Create a rotation vector which is the vector from foot to origin then rotated about
                            // arcLength to get a vector pointing from the foot to the end of the arc (may not be the
                            // right length)
                            Eigen::Vector3d rAFf;
                            rAFf.x() = origin.x() * cos(arcAngle) - origin.y() * sin(arcAngle);
                            rAFf.y() = origin.x() * sin(arcAngle) + origin.y() * cos(arcAngle);
                            rAFf.z() = 0;

                            // Adjust the length of the vector so that it is the length to the end of the arc
                            rAFf = rAFf.normalized() * ((origin.norm() / sin(arcAngle)) * sin(arcLength));
                            // Adjust the y since this is taken in support foot space, and we want to move the swing
                            // foot
                            rAFf.y() += stance_width;

                            // Create the matrix to send the foot to the correct target
                            Haf.linear()      = Eigen::Matrix3d::Identity();  // TODO: create the right rotation
                            Haf.translation() = -rAFf;
                        }

                        // Move the left foot to the location specified by the walkcommand
                        emit(std::make_unique<FootTarget>(
                            start_phase + phase_time, false, Haf.matrix(), true, subsumptionId));
                    } break;
                    default: break;
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
