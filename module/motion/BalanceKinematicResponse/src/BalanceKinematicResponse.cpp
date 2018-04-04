/*
 * This file is part of NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2016 NUbots <nubots@nubots.net>
 */
/*===========================================================================================================*/
/*----------------------------------------CONSTANTS AND DEFINITIONS------------------------------------------*/
/*===========================================================================================================*/
//      INCLUDE(S)
/*===========================================================================================================*/
#include "BalanceKinematicResponse.h"

#include "utility/support/eigen_armadillo.h"

/*===========================================================================================================*/
//      NAMESPACE(S)
/*===========================================================================================================*/
namespace module {
namespace motion {
    /*=======================================================================================================*/
    //      UTILIZATION REFERENCE(S)
    /*=======================================================================================================*/
    using message::input::FallingDetected;
    using message::input::Sensors;

    using message::motion::BalanceBodyUpdate;
    using message::motion::DisableBalanceResponse;
    using message::motion::EnableBalanceResponse;
    using message::motion::FootMotionUpdate;
    using message::motion::HeadMotionUpdate;
    using message::motion::KinematicsModel;
    using message::motion::TorsoMotionUpdate;

    using extension::Configuration;
    using utility::support::Expression;

    using LimbID  = utility::input::LimbID;
    using ServoID = utility::input::ServoID;

    using utility::math::angle::normalizeAngle;
    using utility::math::matrix::Transform2D;
    using utility::math::matrix::Transform3D;

    using utility::nubugger::graph;
    /*=======================================================================================================*/
    //      NUCLEAR METHOD: BalanceKinematicResponse
    /*=======================================================================================================*/
    BalanceKinematicResponse::BalanceKinematicResponse(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , DEBUG(false)
        , DEBUG_ITER(0)
        , initialStep(0)
        , balanceEnabled(false)
        , hipRollCompensationEnabled(false)
        , ankleTorqueCompensationEnabled(false)
        , armRollCompensationEnabled()
        , toeTipCompensationEnabled(false)
        , supportCompensationEnabled(false)
        , balanceOptimiserEnabled(false)
        , pushRecoveryEnabled(false)
        , emitFootPosition(false)
        , armMotionEnabled(false)
        , updateHandle()
        , updateOptimiser()
        , generateStandScriptReaction()
        , torsoPositionsTransform()
        , leftFootPosition2D()
        , rightFootPosition2D()
        , leftFootPositionTransform()
        , rightFootPositionTransform()
        , uSupportMass()
        , activeForwardLimb()
        , activeLimbInitial(LimbID::LEFT_LEG)
        , bodyTilt(0.0)
        , bodyHeight(0.0)
        , supportFront(0.0)
        , supportFront2(0.0)
        , supportBack(0.0)
        , supportSideX(0.0)
        , supportSideY(0.0)
        , supportTurn(0.0)
        , stanceLimitY2(0.0)
        , stepTime(0.0)
        , stepHeight(0.0)
        , step_height_slow_fraction(0.0f)
        , step_height_fast_fraction(0.0f)
        , stepLimits(arma::fill::zeros)
        , footOffsetCoefficient(arma::fill::zeros)
        , uLRFootOffset()
        , armLPostureTransform()
        , armLPostureSource()
        , armLPostureDestination()
        , armRPostureTransform()
        , armRPostureSource()
        , armRPostureDestination()
        , ankleImuParamX()
        , ankleImuParamY()
        , kneeImuParamX()
        , hipImuParamY()
        , armImuParamX()
        , armImuParamY()
        , beginStepTime(0.0)
        , footMotionPhase(0.0)
        , STAND_SCRIPT_DURATION(0.0)
        , lastVeloctiyUpdateTime()
        , velocityHigh(0.0)
        , accelerationTurningFactor(0.0)
        , velocityLimits(arma::fill::zeros)
        , accelerationLimits(arma::fill::zeros)
        , accelerationLimitsHigh(arma::fill::zeros)
        , velocityCurrent()
        , velocityCommand()
        , velFastForward(0.0)
        , velFastTurn(0.0)
        , phase1Single(0.0)
        , phase2Single(0.0)
        , rollParameter(0.0)
        , pitchParameter(0.0)
        , yawParameter(0.0)
        , toeTipParameter(0.0)
        , hipRollParameter(0.0)
        , armRollParameter(0.0)
        , hipCompensationScale(1.0)
        , toeCompensationScale(1.0)
        , ankleCompensationScale(1.0)
        , armCompensationScale(1.0)
        , supportCompensationScale(1.0)
        , hipCompensationMax(0.0)
        , toeCompensationMax(0.0)
        , ankleCompensationMax(0.0)
        , armCompensationMax(0.0)
        , supportCompensationMax(0.0)
        , balancer()
        , kinematicsModel()
        , balanceAmplitude(0.0)
        , balanceWeight(0.0)
        , balanceOffset(0.0)
        , balancePGain(0.0)
        , balanceIGain(0.0)
        , balanceDGain(0.0)
        , lastFootGoalRotation()
        , footGoalErrorSum() {

        // Configure balance kinematic response...
        on<Configuration>("WalkEngine.yaml")
            .then("Balance Response Planner - Configure",
                  [this](const Configuration& config) { configure(config.config); });

        // Define kinematics model for physical calculations...
        on<Startup, Trigger<KinematicsModel>>().then("WalkEngine - Update Kinematics Model",
                                                     [this](const KinematicsModel& model) { kinematicsModel = model; });

        // TODO: Optimise balance configuration using feedback from environmental noise...
        updateOptimiser = on<Every<10, Per<std::chrono::milliseconds>>, With<Configuration>>()
                              .then([this](const Configuration& /*config*/) {
                                  // [this](const BalanceOptimiserCommand& command)
                                  // {
                                  //     if ((NUClear::clock::now() - pushTime) >
                                  //     std::chrono::milliseconds(config["walk_cycle"]["balance"]["balance_time"].as<int>))
                                  //     {
                                  //         balancer.configure(config["walk_cycle"]["balance"]);
                                  //     }
                                  // }
                              })
                              .disable();

        // Aim to avoid dependancy on target position to enhance statelessness and adaptive balance compensation...
        // Single, Priority::HIGH //
        updateHandle =
            on<Trigger<TorsoMotionUpdate>, With<FootMotionUpdate>, With<Sensors>>()
                .then("Balance Response Planner - Received Update (Torso & Foot Position) Info",
                      [this](const TorsoMotionUpdate& tmu, const FootMotionUpdate& fmu, const Sensors& sensors) {
                          // Torso Position is a queued evaluation...
                          if (DEBUG) {
                              log<NUClear::TRACE>(
                                  "Messaging: Balance Kinematic Response - Received Update (Active Torso Position) "
                                  "Info(0)");
                          }
                          setTorsoPositionLegs(convert<double, 3>(tmu.frameArms));
                          setTorsoPositionArms(convert<double, 3>(tmu.frameLegs));
                          setTorsoPosition3D(convert<double, 4, 4>(tmu.frame3D));
                          if (DEBUG) {
                              log<NUClear::TRACE>(
                                  "Messaging: Balance Kinematic Response - Received Update (Active Torso Position) "
                                  "Info(1)");
                          }

                          // Foot Position is a queued, continuous evaluation...
                          if (DEBUG) {
                              log<NUClear::TRACE>(
                                  "Messaging: Balance Kinematic Response - Received Update (Active Foot Position) "
                                  "Info(0)");
                          }
                          setMotionPhase(fmu.phase);
                          setActiveForwardLimb(fmu.activeForwardLimb);
                          setLeftFootPosition2D(convert<double, 3>(fmu.leftFoot2D));
                          setRightFootPosition2D(convert<double, 3>(fmu.rightFoot2D));
                          // Transform feet positions to be relative to the robot torso...
                          setLeftFootPosition(
                              Transform3D(convert<double, 4, 4>(fmu.leftFoot3D)).worldToLocal(getTorsoPosition3D()));
                          setRightFootPosition(
                              Transform3D(convert<double, 4, 4>(fmu.rightFoot3D)).worldToLocal(getTorsoPosition3D()));
                          if (DEBUG) {
                              log<NUClear::TRACE>(
                                  "Messaging: Balance Kinematic Response - Received Update (Active Foot Position) "
                                  "Info(1)");
                          }

                          // With a set of valid anthopomorphic data, balance posture and update WalkEngine...
                          if (DEBUG) {
                              log<NUClear::TRACE>("Messaging: Balance Kinematic Response - Update Robot Posture(0)");
                          }
                          updateBody(sensors);
                          if (DEBUG) {
                              log<NUClear::TRACE>("Messaging: Balance Kinematic Response - Update Robot Posture(1)");
                          }

                          // DEBUG: Printout of motion phase function...
                          emit(graph("BKR Synchronising Motion Phase", fmu.phase));
                      })
                .disable();

        // Aim to avoid dependancy on target position to enhance statelessness and adaptive balance compensation...
        on<Trigger<HeadMotionUpdate>>().then(
            "Balance Response Planner - Received Update (Active Head Position) Info", [this] {
                if (DEBUG) {
                    log<NUClear::TRACE>(
                        "Messaging: Balance Kinematic Response - Received Update (Active Head Position) Info(0)");
                }

                if (DEBUG) {
                    log<NUClear::TRACE>(
                        "Messaging: Balance Kinematic Response - Received Update (Active Head Position) Info(1)");
                }
            });

        // If there is some impulse relating to the robots orientation, then capture values for processing...
        on<Trigger<FallingDetected>>().then("Balance Response Planner - Received Update (Falling) Info",
                                            [this](const FallingDetected& info) {
                                                // Capture normalised angular acceleration experienced...
                                                setRollParameter(info.x);
                                                setPitchParameter(info.y);
                                                setYawParameter(info.z);
                                            });

        // If balance response is required, enable updating...
        on<Trigger<EnableBalanceResponse>>().then([this] {
            postureInitialize();  // Reset stance as we don't know where our limbs are
            updateHandle.enable();
            if (balanceOptimiserEnabled) {
                updateOptimiser.enable();
            }
        });

        // If balance response no longer requested, cease updating...
        on<Trigger<DisableBalanceResponse>>().then([this] {
            updateHandle.disable();
            updateOptimiser.disable();
        });
    }
    /*=======================================================================================================*/
    //      METHOD: armRollCompensation
    /*=======================================================================================================*/
    void BalanceKinematicResponse::armRollCompensation(const Sensors& /*sensors*/) {
        // If feature enabled, apply balance compensation through support actuator...
        if (armRollCompensationEnabled) {
            double shiftShoulder = ((getRollParameter() * getArmCompensationScale()) * armRollParameter);
            setLArmPosition(arma::vec3({getLArmPosition()[0],
                                        getLArmPosition()[1] + (shiftShoulder > 0 ? shiftShoulder : 0),
                                        getLArmPosition()[2]}));
            setRArmPosition(arma::vec3({getRArmPosition()[0],
                                        getRArmPosition()[1] - (shiftShoulder < 0 ? -shiftShoulder : 0),
                                        getRArmPosition()[2]}));
        }
        // std::min(/*Maxium Roll*/0.0, std::max(/*Minimum Roll*/1.0,/*Calculated Roll Offset * 45° Base roll? */0.5));
    }
    /*=======================================================================================================*/
    //      METHOD: ankleTorqueCompensation
    /*=======================================================================================================*/
    void BalanceKinematicResponse::ankleTorqueCompensation(/*const Sensors& sensors*/) {
        // If feature enabled, apply balance compensation through support actuator...
        if (ankleTorqueCompensationEnabled) {
            if (getActiveForwardLimb() == LimbID::LEFT_LEG) {
                setRightFootPosition(getRightFootPosition().rotateX(
                    std::min(getAnkleCompensationMax(),
                             getAnkleCompensationMax() * getRollParameter() * getAnkleCompensationScale())));
            }
            else {
                setLeftFootPosition(getLeftFootPosition().rotateX(
                    std::min(getAnkleCompensationMax(),
                             getAnkleCompensationMax() * getRollParameter() * getAnkleCompensationScale())));
            }
        }
    }
    /*=======================================================================================================*/
    //      METHOD: toeTipCompensation
    /*=======================================================================================================*/
    void BalanceKinematicResponse::toeTipCompensation(/*const Sensors& sensors*/) {
        // If feature enabled, apply balance compensation through support actuator...
        if (toeTipCompensationEnabled) {
            if (getActiveForwardLimb() == LimbID::LEFT_LEG) {
                setRightFootPosition(getRightFootPosition().rotateY(
                    std::max(getToeCompensationMax(),
                             getToeCompensationMax() * getPitchParameter() * getAnkleCompensationScale())));
            }
            else {
                setLeftFootPosition(getLeftFootPosition().rotateY(
                    std::max(getToeCompensationMax(),
                             getToeCompensationMax() * getPitchParameter() * getAnkleCompensationScale())));
            }
        }
    }
    /*=======================================================================================================*/
    //      METHOD: hipCompensation
    /*=======================================================================================================*/
    void BalanceKinematicResponse::hipRollCompensation(const Sensors& sensors) {
        // If feature enabled, apply balance compensation through support actuator...
        if (hipRollCompensationEnabled) {
            // Instantiate unitless phases for x(=0), y(=1) and z(=2) foot motion...
            arma::vec3 getFootPhases = getFootPhase(getMotionPhase(), phase1Single, phase2Single);

            // Evaluate scaled minimum distance of y(=1) phase position to the range [0,1] for hip roll parameter
            // compensation...
            double yBoundedMinimumPhase = std::min({1.0, getFootPhases[1] / 0.1, (1 - getFootPhases[1]) / 0.1});

            // Rotate foot around hip by the given hip roll compensation...
            if (getActiveForwardLimb() == LimbID::LEFT_LEG) {
                arma::mat44 rHipRoll = convert<double, 4, 4>(sensors.forwardKinematics[ServoID::R_HIP_ROLL]);
                setRightFootPosition(
                    getRightFootPosition().rotateZLocal(-hipRollParameter * yBoundedMinimumPhase, rHipRoll));
            }
            else {
                arma::mat44 lHipRoll = convert<double, 4, 4>(sensors.forwardKinematics[ServoID::L_HIP_ROLL]);
                setLeftFootPosition(
                    getLeftFootPosition().rotateZLocal(hipRollParameter * yBoundedMinimumPhase, lHipRoll));
            }
        }
    }
    /*=======================================================================================================*/
    //      METHOD: supportMassCompensation
    /*=======================================================================================================*/
    void BalanceKinematicResponse::supportMassCompensation(const Sensors& sensors) {
        // If feature enabled, apply balance compensation through support actuator...
        if (supportCompensationEnabled) {
            // Create local duplicates of the left and right foot for modification...
            Transform3D leftFoot  = getLeftFootPosition();
            Transform3D rightFoot = getRightFootPosition();

            // Balance the support foot, upon stopping this will be applied alternatively to both feet...
            balancer.balance(kinematicsModel,
                             (getActiveForwardLimb() == LimbID::LEFT_LEG) ? rightFoot : leftFoot,
                             (getActiveForwardLimb() == LimbID::LEFT_LEG) ? LimbID::RIGHT_LEG : LimbID::LEFT_LEG,
                             sensors);

            // Apply changes from balancer to respective left and right foot positions...
            setLeftFootPosition(leftFoot);
            setRightFootPosition(rightFoot);
        }
    }
    /*=======================================================================================================*/
    //      NAME: updateBody
    /*=======================================================================================================*/
    void BalanceKinematicResponse::updateBodyPushRecovery() {
        if (pushRecoveryEnabled) {
            // balanceAmplitude = balance["amplitude"].as<Expression>();
            // balanceWeight = balance["weight"].as<Expression>();
            // balanceOffset = balance["offset"].as<Expression>();
            // balancer.configure(config["walk_cycle"]["balance"]["push_recovery"]);
            // pushTime = NUClear::clock::now();
            // configure(config.config);
        }
    }
    /*=======================================================================================================*/
    //      NAME: updateBody
    /*=======================================================================================================*/
    void BalanceKinematicResponse::updateBody(const Sensors& sensors) {
        // Apply balance and compensation functions to robot posture...
        if (balanceEnabled) {
            updateLowerBody(sensors);
            updateUpperBody(sensors);
        }

        // DEBUGGING: Emit relative feet position with respect to robot torso model...
        if (emitFootPosition) {
            emit(graph("Right foot position", getRightFootPosition2D()));
            emit(graph("Left  foot position", getLeftFootPosition2D()));
        }

        // Emit new robot posture once there has been valid data set in all relevant variables...
        emit(std::make_unique<BalanceBodyUpdate>(getMotionPhase(),
                                                 convert<double, 4, 4>(getLeftFootPosition()),
                                                 convert<double, 4, 4>(getRightFootPosition()),
                                                 convert<double, 3>(getLArmPosition()),
                                                 convert<double, 3>(getRArmPosition())));
    }
    /*=======================================================================================================*/
    //      METHOD: updateLowerBody
    /*=======================================================================================================*/
    void BalanceKinematicResponse::updateLowerBody(const Sensors& sensors) {
        hipRollCompensation(sensors);
        ankleTorqueCompensation();
        toeTipCompensation();
        supportMassCompensation(sensors);
        // etc.
    }
    /*=======================================================================================================*/
    //      NAME: updateUpperBody
    /*=======================================================================================================*/
    void BalanceKinematicResponse::updateUpperBody(const Sensors& sensors) {
        // Move arms so as to reduce ground reaction vector and converse momentum...
        if (armMotionEnabled) {
            // Converts the phase into a sine wave that oscillates between 0 and 1 with a period of 2 phases...
            double easing =
                (1.0 - abs(getRollParameter())) * (std::sin(M_PI * getMotionPhase() - M_PI / 2.0) / 2.0 + 0.5);
            if (getActiveForwardLimb() == LimbID::RIGHT_LEG) {
                // Gets the 2nd half of the sine wave...
                easing = -easing + ((1.0 - abs(getRollParameter())) * 1.0);
            }

            // Linearly interpolate between the start and end positions using the easing parameter
            setLArmPosition(easing * getLArmSource() + (1.0 - easing) * getLArmDestination());
            setRArmPosition((1.0 - easing) * getRArmSource() + easing * getRArmDestination());
        }
        else {
            setLArmPosition(getLArmSource());
            setRArmPosition(getRArmSource());
        }

        // Compensation for balance reactions with arm dynamic roll...
        armRollCompensation(sensors);

        // Start arm/leg collision/prevention
        double rotLeftA           = normalizeAngle(getLeftFootPosition2D().angle() - getTorsoPositionArms().angle());
        double rotRightA          = normalizeAngle(getTorsoPositionArms().angle() - getRightFootPosition2D().angle());
        Transform2D leftLegTorso  = getTorsoPositionArms().worldToLocal(getLeftFootPosition2D());
        Transform2D rightLegTorso = getTorsoPositionArms().worldToLocal(getRightFootPosition2D());
        double leftMinValue       = 5 * M_PI / 180 + std::max(0.0, rotLeftA) / 2
                              + std::max(0.0, leftLegTorso.y() - 0.04) / 0.02 * (6 * M_PI / 180);
        double rightMinValue = -5 * M_PI / 180 - std::max(0.0, rotRightA) / 2
                               - std::max(0.0, -rightLegTorso.y() - 0.04) / 0.02 * (6 * M_PI / 180);

        // Update shoulder pitch to move arm away from body
        // TODO min of max of values... for arm compensation...
        setLArmPosition(
            arma::vec3({getLArmPosition()[0], std::max(leftMinValue, getLArmPosition()[1]), getLArmPosition()[2]}));
        setRArmPosition(
            arma::vec3({getRArmPosition()[0], std::min(rightMinValue, getRArmPosition()[1]), getRArmPosition()[2]}));
    }
    /*=======================================================================================================*/
    //      METHOD: getFootPhase
    /*=======================================================================================================*/
    arma::vec3 BalanceKinematicResponse::getFootPhase(double phase, double phase1Single, double phase2Single) {
        // Computes relative x,z motion of foot during single support phase
        // phSingle = 0: x=0, z=0, phSingle = 1: x=1,z=0
        double phaseSingle     = std::min(std::max(phase - phase1Single, 0.0) / (phase2Single - phase1Single), 1.0);
        double phaseSingleSkew = std::pow(phaseSingle, 0.8) - 0.17 * phaseSingle * (1 - phaseSingle);
        double xf              = 0.5 * (1 - std::cos(M_PI * phaseSingleSkew));
        double zf              = 0.5 * (1 - std::cos(2 * M_PI * phaseSingleSkew));
        return {xf, phaseSingle, zf};
    }
    /*=======================================================================================================*/
    //      METHOD: Reset The Stance of the Humanoid to Initial Valid Stance
    /*=======================================================================================================*/
    void BalanceKinematicResponse::postureInitialize() {
        // Default Initial Torso Position...
        Transform2D uTorso = Transform2D({-getFootOffsetCoefficient(0), 0, 0});

        // Default Initial Left  Foot Position...
        setLeftFootPosition2D(uTorso.localToWorld(
            {getFootOffsetCoefficient(0), kinematicsModel.leg.HIP_OFFSET_Y - getFootOffsetCoefficient(1), 0}));

        // Default Initial Right Foot Position...
        setRightFootPosition2D(uTorso.localToWorld(
            {getFootOffsetCoefficient(0), -kinematicsModel.leg.HIP_OFFSET_Y + getFootOffsetCoefficient(1), 0}));

        Transform3D leftFootLocal  = getLeftFootPosition2D();
        Transform3D rightFootLocal = getRightFootPosition2D();

        // Default Initial Left  Foot Position 3D...
        setLeftFootPosition(leftFootLocal.worldToLocal(uTorso));

        // Default Initial Right Foot Position 3D...
        setRightFootPosition(rightFootLocal.worldToLocal(uTorso));

        // Default Active Forward Limb...
        setActiveForwardLimb(activeLimbInitial);
    }
    /*=======================================================================================================*/
    //      ENCAPSULATION METHOD: Foot Offset Coefficient
    /*=======================================================================================================*/
    double BalanceKinematicResponse::getFootOffsetCoefficient(int index) {
        return (footOffsetCoefficient[index]);
    }
    void BalanceKinematicResponse::setFootOffsetCoefficient(const arma::vec2& inFootOffsetCoefficient) {
        footOffsetCoefficient = inFootOffsetCoefficient;
    }
    void BalanceKinematicResponse::setFootOffsetCoefficient(int index, double inValue) {
        footOffsetCoefficient[index] = inValue;
    }
    /*=======================================================================================================*/
    //      ENCAPSULATION METHOD: Time
    /*=======================================================================================================*/
    double BalanceKinematicResponse::getTime() {
        if (DEBUG) {
            log<NUClear::TRACE>(
                "System Time:%f\n\r",
                double(NUClear::clock::now().time_since_epoch().count()) * (1.0 / double(NUClear::clock::period::den)));
        }
        return (double(NUClear::clock::now().time_since_epoch().count()) * (1.0 / double(NUClear::clock::period::den)));
    }
    /*=======================================================================================================*/
    //      ENCAPSULATION METHOD: Roll Parameter
    /*=======================================================================================================*/
    double BalanceKinematicResponse::getRollParameter() {
        return (rollParameter);
    }
    void BalanceKinematicResponse::setRollParameter(double inRollParameter) {
        rollParameter = inRollParameter;
    }
    /*=======================================================================================================*/
    //      ENCAPSULATION METHOD: Pitch Parameter
    /*=======================================================================================================*/
    double BalanceKinematicResponse::getPitchParameter() {
        return (pitchParameter);
    }
    void BalanceKinematicResponse::setPitchParameter(double inPitchParameter) {
        pitchParameter = inPitchParameter;
    }
    /*=======================================================================================================*/
    //      ENCAPSULATION METHOD: Yaw Parameter
    /*=======================================================================================================*/
    double BalanceKinematicResponse::getYawParameter() {
        return (yawParameter);
    }
    void BalanceKinematicResponse::setYawParameter(double inYawParameter) {
        yawParameter = inYawParameter;
    }
    /*=======================================================================================================*/
    //      ENCAPSULATION METHOD: Motion Phase
    /*=======================================================================================================*/
    double BalanceKinematicResponse::getMotionPhase() {
        return (footMotionPhase);
    }
    void BalanceKinematicResponse::setMotionPhase(double inMotionPhase) {
        footMotionPhase = inMotionPhase;
    }
    /*=======================================================================================================*/
    //      ENCAPSULATION METHOD: Left Arm Position
    /*=======================================================================================================*/
    arma::vec3 BalanceKinematicResponse::getLArmPosition() {
        return (armLPostureTransform);
    }
    void BalanceKinematicResponse::setLArmPosition(arma::vec3 inLArm) {
        armLPostureTransform = inLArm;
    }
    /*=======================================================================================================*/
    //      ENCAPSULATION METHOD: Left Arm Source
    /*=======================================================================================================*/
    arma::vec3 BalanceKinematicResponse::getLArmSource() {
        return (armLPostureSource);
    }
    void BalanceKinematicResponse::setLArmSource(arma::vec3 inLArm) {
        armLPostureSource = inLArm;
    }
    /*=======================================================================================================*/
    //      ENCAPSULATION METHOD: Left Arm Destination
    /*=======================================================================================================*/
    arma::vec3 BalanceKinematicResponse::getLArmDestination() {
        return (armLPostureDestination);
    }
    void BalanceKinematicResponse::setLArmDestination(arma::vec3 inLArm) {
        armLPostureDestination = inLArm;
    }
    /*=======================================================================================================*/
    //      ENCAPSULATION METHOD: Right Arm Position
    /*=======================================================================================================*/
    arma::vec3 BalanceKinematicResponse::getRArmPosition() {
        return (armRPostureTransform);
    }
    void BalanceKinematicResponse::setRArmPosition(arma::vec3 inRArm) {
        armRPostureTransform = inRArm;
    }
    /*=======================================================================================================*/
    //      ENCAPSULATION METHOD: Right Arm Source
    /*=======================================================================================================*/
    arma::vec3 BalanceKinematicResponse::getRArmSource() {
        return (armRPostureSource);
    }
    void BalanceKinematicResponse::setRArmSource(arma::vec3 inRArm) {
        armRPostureSource = inRArm;
    }
    /*=======================================================================================================*/
    //      ENCAPSULATION METHOD: Right Arm Destination
    /*=======================================================================================================*/
    arma::vec3 BalanceKinematicResponse::getRArmDestination() {
        return (armRPostureDestination);
    }
    void BalanceKinematicResponse::setRArmDestination(arma::vec3 inRArm) {
        armRPostureDestination = inRArm;
    }
    /*=======================================================================================================*/
    //      ENCAPSULATION METHOD: Torso Position
    /*=======================================================================================================*/
    Transform2D BalanceKinematicResponse::getTorsoPositionArms() {
        return (torsoPositionsTransform.FrameArms);
    }
    void BalanceKinematicResponse::setTorsoPositionArms(const Transform2D& inTorsoPosition) {
        torsoPositionsTransform.FrameArms = inTorsoPosition;
    }
    Transform2D BalanceKinematicResponse::getTorsoPositionLegs() {
        return (torsoPositionsTransform.FrameLegs);
    }
    void BalanceKinematicResponse::setTorsoPositionLegs(const Transform2D& inTorsoPosition) {
        torsoPositionsTransform.FrameLegs = inTorsoPosition;
    }
    Transform3D BalanceKinematicResponse::getTorsoPosition3D() {
        return (torsoPositionsTransform.Frame3D);
    }
    void BalanceKinematicResponse::setTorsoPosition3D(const Transform3D& inTorsoPosition) {
        torsoPositionsTransform.Frame3D = inTorsoPosition;
    }
    /*=======================================================================================================*/
    //      ENCAPSULATION METHOD: Active Forward Limb
    /*=======================================================================================================*/
    LimbID BalanceKinematicResponse::getActiveForwardLimb() {
        return (activeForwardLimb);
    }
    void BalanceKinematicResponse::setActiveForwardLimb(const LimbID& inActiveForwardLimb) {
        activeForwardLimb = inActiveForwardLimb;
    }
    /*=======================================================================================================*/
    //      ENCAPSULATION METHOD: Support Mass
    /*=======================================================================================================*/
    Transform2D BalanceKinematicResponse::getSupportMass() {
        return (uSupportMass);
    }
    void BalanceKinematicResponse::setSupportMass(const Transform2D& inSupportMass) {
        uSupportMass = inSupportMass;
    }
    /*=======================================================================================================*/
    //      ENCAPSULATION METHOD: Left Foot Position
    /*=======================================================================================================*/
    Transform2D BalanceKinematicResponse::getLeftFootPosition2D() {
        return (leftFootPosition2D);
    }
    void BalanceKinematicResponse::setLeftFootPosition2D(const Transform2D& inLeftFootPosition) {
        leftFootPosition2D = inLeftFootPosition;
    }
    Transform3D BalanceKinematicResponse::getLeftFootPosition() {
        return (leftFootPositionTransform);
    }
    void BalanceKinematicResponse::setLeftFootPosition(const Transform3D& inLeftFootPosition) {
        leftFootPositionTransform = inLeftFootPosition;
    }
    /*=======================================================================================================*/
    //      ENCAPSULATION METHOD: Right Foot Position
    /*=======================================================================================================*/
    Transform2D BalanceKinematicResponse::getRightFootPosition2D() {
        return (rightFootPosition2D);
    }
    void BalanceKinematicResponse::setRightFootPosition2D(const Transform2D& inRightFootPosition) {
        rightFootPosition2D = inRightFootPosition;
    }
    Transform3D BalanceKinematicResponse::getRightFootPosition() {
        return (rightFootPositionTransform);
    }
    void BalanceKinematicResponse::setRightFootPosition(const Transform3D& inRightFootPosition) {
        rightFootPositionTransform = inRightFootPosition;
    }

    double BalanceKinematicResponse::getHipCompensationScale() {
        return hipCompensationScale;
    }
    double BalanceKinematicResponse::getAnkleCompensationScale() {
        return ankleCompensationScale;
    }
    double BalanceKinematicResponse::getToeCompensationScale() {
        return toeCompensationScale;
    }
    double BalanceKinematicResponse::getArmCompensationScale() {
        return armCompensationScale;
    }
    double BalanceKinematicResponse::getSupportCompensationScale() {
        return supportCompensationScale;
    }

    double BalanceKinematicResponse::getHipCompensationMax() {
        return hipCompensationMax;
    }
    double BalanceKinematicResponse::getAnkleCompensationMax() {
        return ankleCompensationMax;
    }
    double BalanceKinematicResponse::getToeCompensationMax() {
        return toeCompensationMax;
    }
    double BalanceKinematicResponse::getArmCompensationMax() {
        return armCompensationMax;
    }
    double BalanceKinematicResponse::getSupportCompensationMax() {
        return supportCompensationMax;
    }

    /*=======================================================================================================*/
    //      METHOD: Configuration
    /*=======================================================================================================*/
    void BalanceKinematicResponse::configure(const YAML::Node& config) {
        if (DEBUG) {
            log<NUClear::TRACE>("Configure BalanceKinematicResponse - Start");
        }
        auto& wlk = config["walk_engine"];
        auto& bkr = config["balance_kinematic_response"];

        auto& debug = bkr["debugging"];
        DEBUG       = debug["enabled"].as<bool>();

        auto& sensors      = wlk["sensors"];
        auto& sensors_gyro = sensors["gyro"];

        auto& sensors_imu = sensors["imu"];
        ankleImuParamX    = sensors_imu["ankleImuParamX"].as<arma::vec>();
        ankleImuParamY    = sensors_imu["ankleImuParamY"].as<arma::vec>();
        kneeImuParamX     = sensors_imu["kneeImuParamX"].as<arma::vec>();
        hipImuParamY      = sensors_imu["hipImuParamY"].as<arma::vec>();
        armImuParamX      = sensors_imu["armImuParamX"].as<arma::vec>();
        armImuParamY      = sensors_imu["armImuParamY"].as<arma::vec>();

        auto& bkr_stance = bkr["stance"];
        auto& arms       = bkr_stance["arms"];
        setLArmSource(arms["left"]["start"].as<arma::vec>());
        setLArmDestination(arms["left"]["end"].as<arma::vec>());
        setRArmSource(arms["right"]["start"].as<arma::vec>());
        setRArmDestination(arms["right"]["end"].as<arma::vec>());
        armMotionEnabled = bkr_stance["moving_enabled"].as<bool>();


        auto& wlk_stance = wlk["stance"];
        auto& body       = wlk_stance["body"];
        bodyHeight       = body["height"].as<Expression>();
        bodyTilt         = body["tilt"].as<Expression>();
        setFootOffsetCoefficient(wlk_stance["foot_offset"].as<arma::vec>());
        stanceLimitY2         = kinematicsModel.leg.LENGTH_BETWEEN_LEGS - wlk_stance["limit_margin_y"].as<Expression>();
        STAND_SCRIPT_DURATION = wlk_stance["STAND_SCRIPT_DURATION"].as<Expression>();

        auto& walkCycle = wlk["walk_cycle"];
        stepTime        = walkCycle["step_time"].as<Expression>();
        stepHeight      = walkCycle["step"]["height"].as<Expression>();
        stepLimits      = walkCycle["step"]["limits"].as<arma::mat::fixed<3, 2>>();

        step_height_slow_fraction = walkCycle["step"]["height_slow_fraction"].as<float>();
        step_height_fast_fraction = walkCycle["step"]["height_fast_fraction"].as<float>();

        auto& velocity = walkCycle["velocity"];
        velocityLimits = velocity["limits"].as<arma::mat::fixed<3, 2>>();
        velocityHigh   = velocity["high_speed"].as<Expression>();

        auto& acceleration        = walkCycle["acceleration"];
        accelerationLimits        = acceleration["limits"].as<arma::vec>();
        accelerationLimitsHigh    = acceleration["limits_high"].as<arma::vec>();
        accelerationTurningFactor = acceleration["turning_factor"].as<Expression>();

        phase1Single = walkCycle["single_support_phase"]["start"].as<Expression>();
        phase2Single = walkCycle["single_support_phase"]["end"].as<Expression>();

        auto& balance                  = bkr["balance"];
        balanceEnabled                 = balance["enabled"].as<bool>();
        balanceOptimiserEnabled        = balance["optimiser_enabled"].as<bool>();
        hipRollCompensationEnabled     = balance["hip_compensation"].as<bool>();
        toeTipCompensationEnabled      = balance["toe_compensation"].as<bool>();
        ankleTorqueCompensationEnabled = balance["ankle_compensation"].as<bool>();
        armRollCompensationEnabled     = balance["arm_compensation"].as<bool>();
        supportCompensationEnabled     = balance["support_compensation"].as<bool>();

        hipCompensationScale     = balance["hip_compensation_scale"].as<double>();
        toeCompensationScale     = balance["toe_compensation_scale"].as<double>();
        ankleCompensationScale   = balance["ankle_compensation_scale"].as<double>();
        armCompensationScale     = balance["arm_compensation_scale"].as<double>();
        supportCompensationScale = balance["support_compensation_scale"].as<double>();

        hipCompensationMax     = balance["hip_compensation_max"].as<Expression>();
        toeCompensationMax     = balance["toe_compensation_max"].as<Expression>();
        ankleCompensationMax   = balance["ankle_compensation_max"].as<Expression>();
        armCompensationMax     = balance["arm_compensation_max"].as<Expression>();
        supportCompensationMax = balance["support_compensation_max"].as<Expression>();

        balanceAmplitude = balance["amplitude"].as<Expression>();
        balanceWeight    = balance["weight"].as<Expression>();
        balanceOffset    = balance["offset"].as<Expression>();

        auto& pushRecovery  = bkr["push_recovery"];
        pushRecoveryEnabled = pushRecovery["enabled"].as<bool>();

        auto& bias       = bkr["support_bias"];
        velFastForward   = bias["velFastForward"].as<Expression>();
        velFastTurn      = bias["velFastTurn"].as<Expression>();
        supportFront     = bias["supportFront"].as<Expression>();
        supportFront2    = bias["supportFront2"].as<Expression>();
        supportBack      = bias["supportBack"].as<Expression>();
        supportSideX     = bias["supportSideX"].as<Expression>();
        supportSideY     = bias["supportSideY"].as<Expression>();
        toeTipParameter  = bias["toe_tip_compensation"].as<Expression>();
        hipRollParameter = bias["hip_roll_compensation"].as<Expression>();
        armRollParameter = bias["arm_roll_compensation"].as<Expression>();
        // ankleMod            = {-toeTipCompensation, 0};

        balancer.configure(balance);
        if (DEBUG) {
            log<NUClear::TRACE>("Configure BalanceKinematicResponse - Finish");
        }
    }
}  // namespace motion
}  // namespace module
