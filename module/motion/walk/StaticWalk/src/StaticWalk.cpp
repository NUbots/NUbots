#include "StaticWalk.hpp"

#include <Eigen/Geometry>
#include <vector>

#include "extension/Configuration.hpp"

#include "message/behaviour/ServoCommand.hpp"
#include "message/input/Sensors.hpp"
#include "message/motion/FootTarget.hpp"
#include "message/motion/KinematicsModel.hpp"
#include "message/motion/TorsoTarget.hpp"
#include "message/motion/WalkCommand.hpp"

#include "utility/behaviour/Action.hpp"
#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/math/comparison.hpp"
#include "utility/motion/ForwardKinematics.hpp"
#include "utility/motion/InverseKinematics.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module {
    namespace motion {
        namespace walk {

            using extension::Configuration;
            using message::behaviour::ServoCommand;
            using message::input::Sensors;
            using message::motion::DisableWalkEngineCommand;
            using message::motion::EnableWalkEngineCommand;
            using message::motion::FootTarget;
            using message::motion::KinematicsModel;
            using message::motion::StopCommand;
            using message::motion::TorsoTarget;
            using message::motion::WalkCommand;
            using utility::behaviour::RegisterAction;
            using utility::input::LimbID;
            using utility::input::ServoID;
            using utility::motion::kinematics::calculateGroundSpace;
            using utility::motion::kinematics::calculateLegJoints;
            using utility::support::Expression;

            // Retrieves the target for the lean based on COM location
            Eigen::Affine3d StaticWalk::getLeanTarget(const Eigen::Vector3d& rCTt) {
                // This will be the returned matrix, ground to torso target
                Eigen::Affine3d Htg;

                // IF rCTt = 0, then CoM is on torso, move torso over the support foot (target to ground = 0)
                // IF rCTt > 0, then CoM is closer to support foot (if support is right), and target to ground should be
                // positive so torso does quite reach the support foot so the CoM lands on the support foot
                // IF rCTt < 0, then CoM is further away from support foot (support is right), and target the ground
                // should be negative so torso goes past support foot to land CoM on support foot
                Eigen::Vector3d rGTt(rCTt.x(), rCTt.y(), -torso_height);
                // Logic was based on if support was the right foot
                // Negate if it is left
                if (state == LEFT_LEAN) {
                    rGTt.x() = -rGTt.x();
                    rGTt.y() = -rGTt.y();
                }

                // Set the rotation relative to the ground as the identity matrix,
                // since the torso should rotate into ground space or stay in ground space
                Htg.linear()      = Eigen::Matrix3d::Identity();
                Htg.translation() = rGTt;
                return Htg;
            }

            // Find the foot target for the step
            // walkcommand is (x,y,theta) where x,y is velocity in m/s and theta is angle in
            // radians/seconds
            Eigen::Affine3d StaticWalk::getFootTarget() {

                // Clamp the rotation to rotation limit
                double rotation = std::min(walkCommand.z(), rotation_limit);

                // Set translation vector, z component is 0
                Eigen::Vector3d translation = Eigen::Vector3d(walkCommand.x(), walkCommand.y(), 0);

                // Foot to foot target matrix
                Eigen::Affine3d Htf;

                // TODO: write stop function and proper check
                // If there is no rotation to be done, just set the translation to x and y, and set the
                // rotation to identity
                // rotation check is to stop the robot from turning outside acceptable parameters
                if (rotation == 0) {
                    Eigen::Vector3d target = translation * time;
                    target.y()             = state == LEFT_STEP ? target.y() + stance_width : target.y() - stance_width;

                    Eigen::Affine3d Hft;
                    Hft.linear()      = Eigen::Matrix3d::Identity();
                    Hft.translation() = target;
                    Htf               = Hft.inverse();
                }

                // If there is rotation, adjust the translation and rotation for this
                else {
                    //  Multiply by phase time so that we are moving in x metres/second and y metres/second
                    Eigen::Vector3d origin = Eigen::Vector3d(-translation.y(), translation.x(), 0);
                    origin /= rotation;
                    Eigen::Vector3d end_point =
                        rotation > 0 ? (translation / std::abs(rotation)) * std::sin(std::abs(rotation) * time)
                                           + (origin * (1 - std::cos(std::abs(rotation) * time)))
                                     : (translation / std::abs(rotation)) * std::sin(std::abs(rotation) * time)
                                           + (origin * (1 - std::cos(std::abs(rotation) * time)));
                    Eigen::Vector3d target = end_point;
                    target.y() += (state == LEFT_STEP ? 1 : -1) * stance_width;

                    const Eigen::Matrix3d Rtf(Eigen::AngleAxisd(-rotation, Eigen::Vector3d::UnitZ()));

                    Htf.linear()      = Rtf;
                    Htf.translation() = -Rtf * target;
                }

                return Htf;
            }

            StaticWalk::StaticWalk(std::unique_ptr<NUClear::Environment> environment)
                : Reactor(std::move(environment)), subsumptionId(size_t(this) * size_t(this) - size_t(this)) {
                on<Startup, Trigger<KinematicsModel>>().then(
                    [this](const KinematicsModel& kinematicsModel) { model = kinematicsModel; });

                on<Configuration>("StaticWalk.yaml").then([this](const Configuration& config) {
                    // Set initial conditions
                    start_phase = NUClear::clock::now();
                    state       = INITIAL;
                    start_right_lean =
                        config["start_right_lean"].as<bool>();  // TODO: dynamically determine which foot to start on

                    // Use configuration here from file StaticWalk.yaml
                    torso_height   = config["torso_height"].as<double>();
                    stance_width   = config["stance_width"].as<double>();
                    phase_time     = std::chrono::milliseconds(config["phase_time"].as<int>());
                    y_offset       = config["y_offset"].as<double>();
                    x_offset       = config["x_offset"].as<double>();
                    rotation_limit = config["rotation_limit"].as<Expression>();

                    // Multiply by 2 so that the lean states are accounted for
                    time = std::chrono::duration_cast<std::chrono::duration<double>>(phase_time).count() * 2;
                });

                updateHandle = on<Trigger<Sensors>>().then([this](const Sensors& sensors) {
                    if (walkCommand == Eigen::Vector3d::Zero()) {
                        state = STOP;
                    }

                    // INITIAL state occurs only as the first state in the walk to set the matrix Hff_s
                    if (state == INITIAL) {
                        start_phase = NUClear::clock::now();
                        // Set the state based on the config
                        state = start_right_lean ? RIGHT_LEAN : LEFT_LEAN;

                        // Get swing foot in ground space
                        Eigen::Affine3d Hts = state == LEFT_LEAN ? Eigen::Affine3d(sensors.Htx[ServoID::L_ANKLE_ROLL])
                                                                 : Eigen::Affine3d(sensors.Htx[ServoID::R_ANKLE_ROLL]);

                        Eigen::Affine3d Htg(calculateGroundSpace(Hts, Eigen::Affine3d(sensors.Htw).inverse()));

                        Hwg = state == LEFT_LEAN ? Eigen::Affine3d(sensors.Htx[ServoID::R_ANKLE_ROLL]).inverse() * Htg
                                                 : Eigen::Affine3d(sensors.Htx[ServoID::L_ANKLE_ROLL]).inverse() * Htg;

                        // Force the foot to be in ground space. TODO: check this
                        Hwg.linear() = Eigen::Matrix3d::Identity();
                    }

                    // When the time is over for this phase, begin the next phase
                    if (NUClear::clock::now() > start_phase + phase_time) {
                        // Reset the start phase time for the new phase
                        start_phase = NUClear::clock::now();

                        // Change the state of the walk based on what the previous state was
                        switch (state) {
                            case LEFT_LEAN: state = RIGHT_STEP; break;
                            case RIGHT_STEP: {
                                // Store where support is relative to swing in ground space
                                Eigen::Affine3d Htg(
                                    calculateGroundSpace(Eigen::Affine3d(sensors.Htx[ServoID::R_ANKLE_ROLL]),
                                                         Eigen::Affine3d(sensors.Htw).inverse()));
                                Hwg = Eigen::Affine3d(sensors.Htx[ServoID::L_ANKLE_ROLL]).inverse() * Htg;
                                Hwg.translation().z() = 0;
                                Hwg.translation().y() = -stance_width;
                                // Force the foot into ground space. TODO: check this
                                Hwg.linear() = Eigen::Matrix3d::Identity();

                                state = RIGHT_LEAN;
                            } break;
                            case RIGHT_LEAN: state = LEFT_STEP; break;
                            case LEFT_STEP: {
                                // Store where support is relative to swing in ground space
                                Eigen::Affine3d Htg(
                                    calculateGroundSpace(Eigen::Affine3d(sensors.Htx[ServoID::L_ANKLE_ROLL]),
                                                         Eigen::Affine3d(sensors.Htw).inverse()));
                                Hwg = Eigen::Affine3d(sensors.Htx[ServoID::R_ANKLE_ROLL]).inverse() * Htg;
                                Hwg.translation().z() = 0;
                                Hwg.translation().y() = stance_width;
                                // Force the foot into ground space. TODO: check this
                                Hwg.linear() = Eigen::Matrix3d::Identity();

                                state = LEFT_LEAN;
                            } break;
                            default: break;
                        }
                    }

                    // Get centre of mass
                    Eigen::Vector3d centre_of_mass(sensors.rMTt.x(), sensors.rMTt.y(), 0);

                    // Put our COM over the correct foot or move foot to target, based on which state we are in
                    switch (state) {
                        case LEFT_LEAN: {
                            // Get lean target
                            Eigen::Affine3d lean_target = getLeanTarget(centre_of_mass);

                            // Move the torso over the left foot
                            emit(std::make_unique<TorsoTarget>(start_phase + phase_time,
                                                               TorsoTarget::SupportFoot::LEFT,
                                                               lean_target.matrix(),
                                                               subsumptionId));

                            // Keep the swing foot in place relative to support, with ground rotation
                            emit(std::make_unique<FootTarget>(start_phase + phase_time / 2,
                                                              FootTarget::SwingFoot::RIGHT,
                                                              Hwg.matrix(),
                                                              FootTarget::Mode::NO_LIFT,
                                                              subsumptionId));
                        } break;
                        case RIGHT_LEAN: {
                            // Get lean target
                            Eigen::Affine3d lean_target = getLeanTarget(centre_of_mass);

                            // Move the torso over the left foot
                            emit(std::make_unique<TorsoTarget>(start_phase + phase_time,
                                                               TorsoTarget::SupportFoot::RIGHT,
                                                               lean_target.matrix(),
                                                               subsumptionId));

                            // Maintain left foot position while the torso moves over the right foot
                            emit(std::make_unique<FootTarget>(start_phase + phase_time / 2,
                                                              FootTarget::SwingFoot::LEFT,
                                                              Hwg.matrix(),
                                                              FootTarget::Mode::NO_LIFT,
                                                              subsumptionId));
                        } break;


                        case RIGHT_STEP: {
                            // Move the right foot to the location specified by the walkcommand
                            emit(std::make_unique<FootTarget>(start_phase + phase_time,
                                                              FootTarget::SwingFoot::RIGHT,
                                                              getFootTarget().matrix(),
                                                              FootTarget::Mode::STEP,
                                                              subsumptionId));
                        } break;

                        case LEFT_STEP: {
                            // Move the left foot to the location specified by the walkcommand
                            emit(std::make_unique<FootTarget>(start_phase + phase_time,
                                                              FootTarget::SwingFoot::LEFT,
                                                              getFootTarget().matrix(),
                                                              FootTarget::Mode::STEP,
                                                              subsumptionId));
                        } break;
                        case STOP: {
                            Eigen::Affine3d Htg;
                            Htg.linear() = Eigen::Matrix3d::Identity();
                            Htg.translation() =
                                Eigen::Vector3d(model.leg.HIP_OFFSET_X, stance_width / 2, -torso_height);

                            // Move the torso between the feet
                            emit(std::make_unique<TorsoTarget>(start_phase + phase_time,
                                                               TorsoTarget::SupportFoot::LEFT,
                                                               Htg.matrix(),
                                                               subsumptionId));

                            // We want to move the feet to be in line with each other
                            Hwg                   = Eigen::Affine3d(sensors.Htx[ServoID::R_ANKLE_ROLL]).inverse() * Htg;
                            Hwg.translation().x() = 0;
                            Hwg.translation().y() = stance_width;
                            Hwg.translation().z() = 0;
                            // Force the foot into ground space. TODO: check this
                            Hwg.linear() = Eigen::Matrix3d::Identity();

                            // Keep the swing foot in place relative to support, with ground rotation
                            emit(std::make_unique<FootTarget>(start_phase + phase_time,
                                                              FootTarget::SwingFoot::RIGHT,
                                                              Hwg.matrix(),
                                                              FootTarget::Mode::NO_LIFT,
                                                              subsumptionId));
                        } break;
                        default: break;
                    }
                });

                on<Trigger<WalkCommand>>().then([this](const WalkCommand& command) {
                    subsumptionId = command.subsumption_id;
                    walkCommand   = Eigen::Vector3d(command.command.x(), command.command.y(), command.command.z());

                    // If we were told to stop, and now we've got a command, move to the initial state to start walking
                    if (state == STOP) {
                        state = INITIAL;
                    }
                });

                on<Trigger<EnableWalkEngineCommand>>().then([this](const EnableWalkEngineCommand& command) {
                    subsumptionId = command.subsumption_id;
                    state         = INITIAL;
                    updateHandle.enable();
                });

                // Nobody needs the walk engine, so stop updating it.
                on<Trigger<DisableWalkEngineCommand>>().then([this](const DisableWalkEngineCommand& command) {
                    subsumptionId = command.subsumption_id;
                    updateHandle.disable();
                });

                on<Trigger<StopCommand>>().then([this](const StopCommand& command) {
                    subsumptionId = command.subsumption_id;
                    walkCommand   = Eigen::Vector3d::Zero();
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
    }      // namespace motion
}  // namespace module
