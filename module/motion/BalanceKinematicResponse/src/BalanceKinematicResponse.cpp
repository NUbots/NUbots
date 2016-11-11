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
/*===========================================================================================================*/
//      NAMESPACE(S)
/*===========================================================================================================*/
namespace module 
{
namespace motion 
{
/*=======================================================================================================*/
//      UTILIZATION REFERENCE(S)
/*=======================================================================================================*/
    using message::input::LimbID;
    using message::input::Sensors;
    using message::input::PushDetection;
    using message::input::FallingDetected;

    using message::motion::FootMotionUpdate;
    using message::motion::HeadMotionUpdate;
    using message::motion::TorsoMotionUpdate;
    using message::motion::BalanceBodyUpdate;
    using message::motion::EnableBalanceResponse;
    using message::motion::DisableBalanceResponse;
    using message::motion::kinematics::KinematicsModel;

    using message::support::Configuration;
    using utility::support::Expression;

    using utility::math::matrix::Transform2D;
    using utility::math::matrix::Transform3D;
    using utility::math::angle::normalizeAngle;

    using utility::nubugger::graph;
/*=======================================================================================================*/
//      NUCLEAR METHOD: BalanceKinematicResponse
/*=======================================================================================================*/
    BalanceKinematicResponse::BalanceKinematicResponse(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) 
        , DEBUG(false), DEBUG_ITER(0), initialStep(0)
        , balanceEnabled(false)
        , hipRollCompensationEnabled(false), ankleTorqueCompensationEnabled(false)
        , armRollCompensationEnabled(), toeTipCompensationEnabled(false)
        , supportCompensationEnabled(false)
        , balanceOptimiserEnabled(false), pushRecoveryEnabled(false)
        , emitLocalisation(false), emitFootPosition(false)
        , updateHandle(), updateOptimiser(), generateStandScriptReaction()
        , torsoPositionsTransform()
        , leftFootPosition2D(), rightFootPosition2D()
        , leftFootPositionTransform(), rightFootPositionTransform()
        , uSupportMass()
        , activeForwardLimb(), activeLimbInitial(LimbID::LEFT_LEG)
        , bodyTilt(0.0), bodyHeight(0.0), stanceLimitY2(0.0), stepTime(0.0), stepHeight(0.0)
        , step_height_slow_fraction(0.0f), step_height_fast_fraction(0.0f)
        , stepLimits(arma::fill::zeros), footOffsetCoefficient(arma::fill::zeros), uLRFootOffset()
        , armLPostureTransform(), armLPostureSource(), armLPostureDestination()
        , armRPostureTransform(), armRPostureSource(), armRPostureDestination()
        , beginStepTime(0.0), footMotionPhase()
        , STAND_SCRIPT_DURATION(0.0), pushTime(), lastVeloctiyUpdateTime()
        , velocityHigh(0.0), accelerationTurningFactor(0.0), velocityLimits(arma::fill::zeros)
        , accelerationLimits(arma::fill::zeros), accelerationLimitsHigh(arma::fill::zeros)
        , velocityCurrent(), velocityCommand()
        , phase1Single(0.0), phase2Single(0.0)
        , toeTipParameter(0.0), hipRollParameter(0.0), shoulderRollParameter(0.0)
        , balancer(), kinematicsModel()
        , balanceAmplitude(0.0), balanceWeight(0.0), balanceOffset(0.0)
        , balancePGain(0.0), balanceIGain(0.0), balanceDGain(0.0)
        , lastFootGoalRotation(), footGoalErrorSum()       
    {

        //Configure balance kinematic response...
        on<Configuration>("BalanceKinematicResponse.yaml").then("Balance Response Planner - Configure", [this] (const Configuration& config) 
        {
            configure(config.config);
        });

        //Define kinematics model for physical calculations...
        on<Trigger<KinematicsModel>>().then("WalkEngine - Update Kinematics Model", [this](const KinematicsModel& model)
        {
            kinematicsModel = model;
        });

        // Tune requested robot posture such that balance is maintained and motion is differential... 
        updateHandle = on<Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>, With<Sensors>, Single, Priority::HIGH>()
        .then("Balance Response Planner - Update Robot Posture", [this] (const Sensors& sensors)
        {
            if(DEBUG) { log<NUClear::TRACE>("Messaging: Balance Kinematic Response - Update Robot Posture(0)"); }
                updateBody(sensors);
            if(DEBUG) { log<NUClear::TRACE>("Messaging: Balance Kinematic Response - Update Robot Posture(1)"); }
        }).disable();

        // TODO: Optimise balance configuration using feedback from environmental noise...
        updateOptimiser = on<Every<10, Per<std::chrono::milliseconds>>, With<Configuration>>().then([this](const Configuration& config) 
        {
            // [this](const BalanceOptimiserCommand& command) 
            // {
            //     if ((NUClear::clock::now() - pushTime) > std::chrono::milliseconds(config["walk_cycle"]["balance"]["balance_time"].as<int>)) 
            //     {
            //         balancer.configure(config["walk_cycle"]["balance"]);
            //     }
            // }
        }).disable(); 

        // Aim to avoid dependancy on target position to enhance statelessness and adaptive balance compensation...
        on<Trigger<FootMotionUpdate>>().then("Balance Response Planner - Received Update (Active Foot Position) Info", [this] (const FootMotionUpdate& info)
        {
            if(DEBUG) { log<NUClear::TRACE>("Messaging: Balance Kinematic Response - Received Update (Active Foot Position) Info(0)"); }
                setMotionPhase(info.phase);
                setActiveForwardLimb(info.activeForwardLimb);          
                setLeftFootPosition2D(info.leftFoot2D);       
                setRightFootPosition2D(info.rightFoot2D);           
                // Transform feet positions to be relative to the robot torso...            
                setLeftFootPosition(info.leftFoot3D.worldToLocal(getTorsoPosition3D()));         
                setRightFootPosition(info.rightFoot3D.worldToLocal(getTorsoPosition3D()));                    
            if(DEBUG) { log<NUClear::TRACE>("Messaging: Balance Kinematic Response - Received Update (Active Foot Position) Info(1)"); }
        });

        // Aim to avoid dependancy on target position to enhance statelessness and adaptive balance compensation...
        on<Trigger<TorsoMotionUpdate>>().then("Balance Response Planner - Received Update (Active Torso Position) Info", [this] (const TorsoMotionUpdate& info)
        {
            if(DEBUG) { log<NUClear::TRACE>("Messaging: Balance Kinematic Response - Received Update (Active Torso Position) Info(0)"); }
                setTorsoPositionLegs(info.frameArms);       
                setTorsoPositionArms(info.frameLegs);             
                setTorsoPosition3D(info.frame3D);            
            if(DEBUG) { log<NUClear::TRACE>("Messaging: Balance Kinematic Response - Received Update (Active Torso Position) Info(1)"); }
        });

        // Aim to avoid dependancy on target position to enhance statelessness and adaptive balance compensation...
        on<Trigger<HeadMotionUpdate>>().then("Balance Response Planner - Received Update (Active Head Position) Info", [this] 
        {
            if(DEBUG) { log<NUClear::TRACE>("Messaging: Balance Kinematic Response - Received Update (Active Head Position) Info(0)"); }

            if(DEBUG) { log<NUClear::TRACE>("Messaging: Balance Kinematic Response - Received Update (Active Head Position) Info(1)"); }
        });

        // If significant environmental noise is present, attempt to recover stability...
        pushTime = NUClear::clock::now();
        on<Trigger<PushDetection>, With<Configuration>>().then([this](const PushDetection& pd, const Configuration& config) 
        {
            if(DEBUG) { log<NUClear::TRACE>("Messaging: Balance Kinematic Response - Received Update (Push Detected) Info(0)"); }               
                updateBodyPushRecovery();
            if(DEBUG) { log<NUClear::TRACE>("Messaging: Balance Kinematic Response - Received Update (Push Detected) Info(1)"); }
        });        

        // If there is some impulse relating to the robots orientation, then capture values for processing...
        on<Trigger<FallingDetected>>().then("Balance Response Planner - Received Update (Falling) Info", [this](const FallingDetected& info) 
        {
            // Capture normalised angular acceleration experienced...
            setRollParameter(info.y);
            setPitchParameter(info.x);
	    setYawParameter(info.z)
        });

        // If balance response is required, enable updating...
        on<Trigger<EnableBalanceResponse>>().then([this] 
        {          
            postureInitialize(); // Reset stance as we don't know where our limbs are
            updateHandle.enable();
            if(balanceOptimiserEnabled)
            {
                updateOptimiser.enable();
            }
        });

        // If balance response no longer requested, cease updating...
        on<Trigger<DisableBalanceResponse>>().then([this] 
        {
            updateHandle.disable(); 
            updateOptimiser.disable();
        });
    }
/*=======================================================================================================*/
//      METHOD: armRollCompensation
/*=======================================================================================================*/
    void BalanceKinematicResponse::armRollCompensation(const Sensors& sensors) 
    {
        //If feature enabled, apply balance compensation through support actuator...
        if (armRollCompensationEnabled) 
        {
            if(getShoulderRollParameter() > 0)
            {
std::cout << "Gyro roll:\n\r" << getRollParameter();                
            }
            if(getShoulderPitchParameter() > 0)
            {
std::cout << "Gyro pitch:\n\r" << getPitchParameter();                
            }

        }
    }      
/*=======================================================================================================*/
//      METHOD: ankleTorqueCompensation
/*=======================================================================================================*/
    void BalanceKinematicResponse::ankleTorqueCompensation(/*const Sensors& sensors*/) 
    {
        //If feature enabled, apply balance compensation through support actuator...
        if (ankleTorqueCompensationEnabled) 
        {
            if (getActiveForwardLimb() == LimbID::LEFT_LEG)
            {
                setRightFootPosition(getRightFootPosition().rotateZ(0)); //insert factor based on orientation
            }
            else 
            {
                setLeftFootPosition(getLeftFootPosition().rotateZ(0));
            }
        }
    }      
/*=======================================================================================================*/
//      METHOD: toeTipCompensation
/*=======================================================================================================*/
    void BalanceKinematicResponse::toeTipCompensation(/*const Sensors& sensors*/) 
    {
        //If feature enabled, apply balance compensation through support actuator...
        if (toeTipCompensationEnabled) 
        {
            //
        }
    }    
/*=======================================================================================================*/
//      METHOD: hipCompensation
/*=======================================================================================================*/
	void BalanceKinematicResponse::hipRollCompensation(const Sensors& sensors) 
    {
        //If feature enabled, apply balance compensation through support actuator...
        if (hipRollCompensationEnabled) 
        {
            //Instantiate unitless phases for x(=0), y(=1) and z(=2) foot motion...
            arma::vec3 getFootPhases = getFootPhase(getMotionPhase(), phase1Single, phase2Single);
            
            //Evaluate scaled minimum distance of y(=1) phase position to the range [0,1] for hip roll parameter compensation... 
            double yBoundedMinimumPhase = std::min({1.0, getFootPhases[1] / 0.1, (1 - getFootPhases[1]) / 0.1});

            //Rotate foot around hip by the given hip roll compensation...
            if (getActiveForwardLimb() == LimbID::LEFT_LEG)
            {
                setRightFootPosition(getRightFootPosition().rotateZLocal(-hipRollParameter * yBoundedMinimumPhase, sensors.forwardKinematics.find(ServoID::R_HIP_ROLL)->second));
            }
            else 
            {
                setLeftFootPosition(getLeftFootPosition().rotateZLocal( hipRollParameter  * yBoundedMinimumPhase, sensors.forwardKinematics.find(ServoID::L_HIP_ROLL)->second));
            }
        }
    }
/*=======================================================================================================*/
//      METHOD: supportMassCompensation
/*=======================================================================================================*/
    void BalanceKinematicResponse::supportMassCompensation(const Sensors& sensors) 
    {
        //If feature enabled, apply balance compensation through support actuator...
        if (supportCompensationEnabled) 
        {
            // Create local duplicates of the left and right foot for modification...
            Transform3D leftFoot  = getLeftFootPosition();
            Transform3D rightFoot = getRightFootPosition();

            // Balance the support foot, upon stopping this will be applied alternatively to both feet...
            balancer.balance(kinematicsModel, (getActiveForwardLimb() == LimbID::LEFT_LEG) ? rightFoot : leftFoot 
                                            , (getActiveForwardLimb() == LimbID::LEFT_LEG) ? LimbID::RIGHT_LEG : LimbID::LEFT_LEG, sensors);
            
            // Apply changes from balancer to respective left and right foot positions...
            setLeftFootPosition(leftFoot);
            setRightFootPosition(rightFoot);
        }
    }  
/*=======================================================================================================*/
//      NAME: updateBody
/*=======================================================================================================*/
    void BalanceKinematicResponse::updateBodyPushRecovery()
    {
        if(pushRecoveryEnabled)
        {
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
    void BalanceKinematicResponse::updateBody(const Sensors& sensors)
    {
        // Apply balance and compensation functions to robot posture...
        //if(balanceEnabled)
        //{
            updateLowerBody(sensors);
            updateUpperBody(sensors);
        //}

        //DEBUGGING: Emit relative torso position with respect to world model... 
        if (emitLocalisation) 
        {
            //localise(getTorsoPositionArms().localToWorld({-kinematicsModel.Leg.HIP_OFFSET_X, 0, 0}));
        }

        //DEBUGGING: Emit relative feet position with respect to robot torso model... 
        if (emitFootPosition)
        {
            //emit(graph("Right foot position", rightFootTorso.translation()));
            //emit(graph("Left  foot position",  leftFootTorso.translation()));
        }  

        emit(std::make_unique<BalanceBodyUpdate>(getMotionPhase(), getLeftFootPosition(), getRightFootPosition(), getTorsoPositionArms(), getTorsoPositionLegs(), getTorsoPosition3D(), getLArmPosition(), getRArmPosition()));
    }      
/*=======================================================================================================*/
//      METHOD: updateLowerBody
/*=======================================================================================================*/
    void BalanceKinematicResponse::updateLowerBody(const Sensors& sensors) 
    {
        hipRollCompensation(sensors);
        ankleTorqueCompensation();
        toeTipCompensation();
        supportMassCompensation(sensors);
        //etc.
    }
/*=======================================================================================================*/
//      NAME: updateUpperBody
/*=======================================================================================================*/
    void BalanceKinematicResponse::updateUpperBody(const Sensors& sensors) 
    {
        // Converts the phase into a sine wave that oscillates between 0 and 1 with a period of 2 phases
        double easing = std::sin(M_PI * getMotionPhase() - M_PI / 2.0) / 2.0 + 0.5;
        if (getActiveForwardLimb() == LimbID::LEFT_LEG) 
        {
            easing = -easing + 1.0; // Gets the 2nd half of the sine wave
        }

        // Linearly interpolate between the start and end positions using the easing parameter
        setLArmPosition(easing * getLArmSource() + (1.0 - easing) * getLArmDestination());
        setRArmPosition((1.0 - easing) * getRArmSource() + easing * getRArmDestination());

        // Compensation for balance reactions with arm dynamic roll...
        armRollCompensation(sensors);

        // Start arm/leg collision/prevention
        double rotLeftA = normalizeAngle(getLeftFootPosition2D().angle() - getTorsoPositionArms().angle());
        double rotRightA = normalizeAngle(getTorsoPositionArms().angle() - getRightFootPosition2D().angle());
        Transform2D leftLegTorso = getTorsoPositionArms().worldToLocal(getLeftFootPosition2D());
        Transform2D rightLegTorso = getTorsoPositionArms().worldToLocal(getRightFootPosition2D());
        double leftMinValue = 5 * M_PI / 180 + std::max(0.0, rotLeftA) / 2 + std::max(0.0, leftLegTorso.y() - 0.04) / 0.02 * (6 * M_PI / 180);
        double rightMinValue = -5 * M_PI / 180 - std::max(0.0, rotRightA) / 2 - std::max(0.0, -rightLegTorso.y() - 0.04) / 0.02 * (6 * M_PI / 180);
        
        // update shoulder pitch to move arm away from body
        setLArmPosition(arma::vec3({getLArmPosition()[0], std::max(leftMinValue,  getLArmPosition()[1]), getLArmPosition()[2]}));
        setRArmPosition(arma::vec3({getRArmPosition()[0], std::min(rightMinValue, getRArmPosition()[1]), getRArmPosition()[2]}));
    }   
/*=======================================================================================================*/
//      NAME: localise
/*=======================================================================================================*/
    void BalanceKinematicResponse::localise(Transform2D position)
    {
        // emit position as a fake localisation
        auto localisation = std::make_unique<std::vector<message::localisation::Self>>();
        message::localisation::Self self;
        self.position = {position.x(), position.y()};
        self.position_cov = arma::eye(2,2) * 0.1; // made up
        self.heading = {std::cos(position.angle()), std::sin(position.angle())}; // convert to cartesian coordinates
        self.velocity = arma::zeros(2); // not used
        self.robot_to_world_rotation = arma::zeros(2,2); // not used
        localisation->push_back(self);
        emit(std::move(localisation));
    }    
/*=======================================================================================================*/
//      METHOD: getFootPhase
/*=======================================================================================================*/
    arma::vec3 BalanceKinematicResponse::getFootPhase(double phase, double phase1Single, double phase2Single) 
    {
        // Computes relative x,z motion of foot during single support phase
        // phSingle = 0: x=0, z=0, phSingle = 1: x=1,z=0
        double phaseSingle = std::min(std::max(phase - phase1Single, 0.0) / (phase2Single - phase1Single), 1.0);
        double phaseSingleSkew = std::pow(phaseSingle, 0.8) - 0.17 * phaseSingle * (1 - phaseSingle);
        double xf = 0.5 * (1 - std::cos(M_PI * phaseSingleSkew));
        double zf = 0.5 * (1 - std::cos(2 * M_PI * phaseSingleSkew));
        return {xf, phaseSingle, zf};
    }    
/*=======================================================================================================*/
//      METHOD: Reset The Stance of the Humanoid to Initial Valid Stance
/*=======================================================================================================*/
    void BalanceKinematicResponse::postureInitialize() 
    {      
         // Default Initial Torso Position...
        Transform2D uTorso = Transform2D({-getFootOffsetCoefficient(0), 0, 0});
        
        // Default Initial Left  Foot Position...
        setLeftFootPosition2D(uTorso.localToWorld({getFootOffsetCoefficient(0), kinematicsModel.Leg.HIP_OFFSET_Y - getFootOffsetCoefficient(1), 0}));        
        
        // Default Initial Right Foot Position...
        setRightFootPosition2D(uTorso.localToWorld({getFootOffsetCoefficient(0), -kinematicsModel.Leg.HIP_OFFSET_Y + getFootOffsetCoefficient(1), 0}));
        
        Transform3D leftFootLocal  =  getLeftFootPosition2D();
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
    double BalanceKinematicResponse::getFootOffsetCoefficient(int index)
    {
        return (footOffsetCoefficient[index]);
    }
    void BalanceKinematicResponse::setFootOffsetCoefficient(const arma::vec2& inFootOffsetCoefficient)
    {
        footOffsetCoefficient = inFootOffsetCoefficient;
    }
    void BalanceKinematicResponse::setFootOffsetCoefficient(int index, double inValue)
    {
        footOffsetCoefficient[index] = inValue;
    }    
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Time
/*=======================================================================================================*/
    double BalanceKinematicResponse::getTime() 
    {
        if(DEBUG) { printf("System Time:%f\n\r", double(NUClear::clock::now().time_since_epoch().count()) * (1.0 / double(NUClear::clock::period::den))); }
        return (double(NUClear::clock::now().time_since_epoch().count()) * (1.0 / double(NUClear::clock::period::den)));
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Roll Parameter
/*=======================================================================================================*/    
    double BalanceKinematicResponse::getRollParameter()
    {
        return (shoulderRollParameter);
    }
    void BalanceKinematicResponse::setRollParameter(double inShoulderRollParameter)
    {
        shoulderRollParameter = inShoulderRollParameter;
    }    
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Pitch Parameter
/*=======================================================================================================*/    
    double BalanceKinematicResponse::getPitchParameter()
    {
        return (shoulderPitchParameter);
    }
    void BalanceKinematicResponse::setPitchParameter(double inShoulderPitchParameter)
    {
        shoulderPitchParameter = inShoulderPitchParameter;
    }        
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Yaw Parameter
/*=======================================================================================================*/    
    double BalanceKinematicResponse::getYawParameter()
    {
        return (yawPitchParameter);
    }
    void BalanceKinematicResponse::setYawParameter(double inYawPitchParameter)
    {
        YawParameter = inYawParameter;
    }        
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Motion Phase
/*=======================================================================================================*/    
    double BalanceKinematicResponse::getMotionPhase()
    {
        return (footMotionPhase);
    }
    void BalanceKinematicResponse::setMotionPhase(double inMotionPhase)  
    {
        footMotionPhase = inMotionPhase;
    }       
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Left Arm Position
/*=======================================================================================================*/    
    arma::vec3 BalanceKinematicResponse::getLArmPosition()
    {
        return (armLPostureTransform);
    }    
    void BalanceKinematicResponse::setLArmPosition(arma::vec3 inLArm)
    {
        armLPostureTransform = inLArm;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Left Arm Source
/*=======================================================================================================*/     
    arma::vec3 BalanceKinematicResponse::getLArmSource()
    {
        return (armLPostureSource);
    }    
    void BalanceKinematicResponse::setLArmSource(arma::vec3 inLArm)
    {
        armLPostureSource = inLArm;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Left Arm Destination
/*=======================================================================================================*/     
    arma::vec3 BalanceKinematicResponse::getLArmDestination()
    {
        return (armLPostureDestination);
    }    
    void BalanceKinematicResponse::setLArmDestination(arma::vec3 inLArm)
    {
        armLPostureDestination = inLArm;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Right Arm Position
/*=======================================================================================================*/ 
    arma::vec3 BalanceKinematicResponse::getRArmPosition()
    {
        return (armRPostureTransform);
    }   
    void BalanceKinematicResponse::setRArmPosition(arma::vec3 inRArm)
    {
        armRPostureTransform = inRArm;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Right Arm Source
/*=======================================================================================================*/     
    arma::vec3 BalanceKinematicResponse::getRArmSource()
    {
        return (armRPostureSource);
    }   
    void BalanceKinematicResponse::setRArmSource(arma::vec3 inRArm)
    {
        armRPostureSource = inRArm;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Right Arm Destination
/*=======================================================================================================*/     
    arma::vec3 BalanceKinematicResponse::getRArmDestination()
    {
        return (armRPostureDestination);
    }    
    void BalanceKinematicResponse::setRArmDestination(arma::vec3 inRArm)
    {
        armRPostureDestination = inRArm;
    }      
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Torso Position
/*=======================================================================================================*/
    Transform2D BalanceKinematicResponse::getTorsoPositionArms()
    {
        return (torsoPositionsTransform.FrameArms);
    }
    void BalanceKinematicResponse::setTorsoPositionArms(const Transform2D& inTorsoPosition)
    {
        torsoPositionsTransform.FrameArms = inTorsoPosition;
    }
    Transform2D BalanceKinematicResponse::getTorsoPositionLegs()
    {
        return (torsoPositionsTransform.FrameLegs);
    }    
    void BalanceKinematicResponse::setTorsoPositionLegs(const Transform2D& inTorsoPosition)
    {
        torsoPositionsTransform.FrameLegs = inTorsoPosition;
    }
    Transform3D BalanceKinematicResponse::getTorsoPosition3D()
    {
        return (torsoPositionsTransform.Frame3D);
    }            
    void BalanceKinematicResponse::setTorsoPosition3D(const Transform3D& inTorsoPosition)
    {
        torsoPositionsTransform.Frame3D = inTorsoPosition;
    }    
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Active Forward Limb
/*=======================================================================================================*/
    LimbID BalanceKinematicResponse::getActiveForwardLimb()
    {
        return (activeForwardLimb);
    }
    void BalanceKinematicResponse::setActiveForwardLimb(const LimbID& inActiveForwardLimb)
    {
        activeForwardLimb = inActiveForwardLimb;
    }      
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Support Mass
/*=======================================================================================================*/
    Transform2D BalanceKinematicResponse::getSupportMass()
    {
        return (uSupportMass);
    }
    void BalanceKinematicResponse::setSupportMass(const Transform2D& inSupportMass)
    {
        uSupportMass = inSupportMass;
    }    
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Left Foot Position
/*=======================================================================================================*/
    Transform2D BalanceKinematicResponse::getLeftFootPosition2D()
    {
        return (leftFootPosition2D);
    }
    void BalanceKinematicResponse::setLeftFootPosition2D(const Transform2D& inLeftFootPosition)
    {
        leftFootPosition2D = inLeftFootPosition;
    }
    Transform3D BalanceKinematicResponse::getLeftFootPosition()
    {
        return (leftFootPositionTransform);
    }
    void BalanceKinematicResponse::setLeftFootPosition(const Transform3D& inLeftFootPosition)
    {
        leftFootPositionTransform = inLeftFootPosition;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Right Foot Position
/*=======================================================================================================*/
    Transform2D BalanceKinematicResponse::getRightFootPosition2D()
    {
        return (rightFootPosition2D);
    }
    void BalanceKinematicResponse::setRightFootPosition2D(const Transform2D& inRightFootPosition)
    {
        rightFootPosition2D = inRightFootPosition;
    }
    Transform3D BalanceKinematicResponse::getRightFootPosition()
    {
        return (rightFootPositionTransform);
    }
    void BalanceKinematicResponse::setRightFootPosition(const Transform3D& inRightFootPosition)
    {
        rightFootPositionTransform = inRightFootPosition;
    }
/*=======================================================================================================*/
//      METHOD: Configuration
/*=======================================================================================================*/
    void BalanceKinematicResponse::configure(const YAML::Node& config)
    {
        auto& debug = config["debugging"];
        DEBUG = debug["enabled"].as<bool>();
        
        emitLocalisation = config["emit_localisation"].as<bool>();

        auto& stance = config["stance"];
        bodyHeight = stance["body_height"].as<Expression>();
        bodyTilt = stance["body_tilt"].as<Expression>();
        setLArmSource(stance["arms"]["left"]["start"].as<arma::vec>());
        setLArmDestination(stance["arms"]["left"]["end"].as<arma::vec>());
        setRArmSource(stance["arms"]["right"]["start"].as<arma::vec>());
        setRArmDestination(stance["arms"]["right"]["end"].as<arma::vec>());
        setFootOffsetCoefficient(stance["foot_offset"].as<arma::vec>());
        // gToe/heel overlap checking values
        stanceLimitY2 = kinematicsModel.Leg.LENGTH_BETWEEN_LEGS() - stance["limit_margin_y"].as<Expression>();

        auto& walkCycle = config["walk_cycle"];
        stepTime = walkCycle["step_time"].as<Expression>();
        hipRollParameter = walkCycle["hip_roll_compensation"].as<Expression>();
        stepHeight = walkCycle["step"]["height"].as<Expression>();
        stepLimits = walkCycle["step"]["limits"].as<arma::mat::fixed<3,2>>();

        step_height_slow_fraction = walkCycle["step"]["height_slow_fraction"].as<float>();
        step_height_fast_fraction = walkCycle["step"]["height_fast_fraction"].as<float>();

        auto& velocity = walkCycle["velocity"];
        velocityLimits = velocity["limits"].as<arma::mat::fixed<3,2>>();
        velocityHigh = velocity["high_speed"].as<Expression>();

        auto& acceleration = walkCycle["acceleration"];
        accelerationLimits = acceleration["limits"].as<arma::vec>();
        accelerationLimitsHigh = acceleration["limits_high"].as<arma::vec>();
        accelerationTurningFactor = acceleration["turning_factor"].as<Expression>();

        phase1Single = walkCycle["single_support_phase"]["start"].as<Expression>();
        phase2Single = walkCycle["single_support_phase"]["end"].as<Expression>();

        auto& balance = walkCycle["balance"];
        balanceEnabled = balance["enabled"].as<bool>();
        balanceOptimiserEnabled = balance["optimiser_enabled"].as<bool>();
        hipRollCompensationEnabled = balance["hip_compensation"].as<bool>();
        toeTipCompensationEnabled = balance["toe_compensation"].as<bool>();
        ankleTorqueCompensationEnabled = balance["ankle_compensation"].as<bool>();
        armRollCompensationEnabled = balance["arm_compensation"].as<bool>();
        supportCompensationEnabled = balance["support_compensation"].as<bool>();
        //balanceAmplitude = balance["amplitude"].as<Expression>();
        //balanceWeight = balance["weight"].as<Expression>();
        //balanceOffset = balance["offset"].as<Expression>();

        auto& pushRecovery = balance["push_recovery"];
        pushRecoveryEnabled = pushRecovery["enabled"].as<bool>();

        balancer.configure(balance);

        
        /* TODO
        // gCompensation parameters
        toeTipParameter = config["toeTipCompensation"].as<Expression>();
        ankleMod = {-toeTipCompensation, 0};

        // gGyro stabilization parameters
        ankleImuParamX = config["ankleImuParamX"].as<arma::vec>();
        ankleImuParamY = config["ankleImuParamY"].as<arma::vec>();
        kneeImuParamX = config["kneeImuParamX"].as<arma::vec>();
        hipImuParamY = config["hipImuParamY"].as<arma::vec>();
        armImuParamX = config["armImuParamX"].as<arma::vec>();
        armImuParamY = config["armImuParamY"].as<arma::vec>();

        // gSupport bias parameters to reduce backlash-based instability
        velFastForward = config["velFastForward"].as<Expression>();
        velFastTurn = config["velFastTurn"].as<Expression>();
        supportFront = config["supportFront"].as<Expression>();
        supportFront2 = config["supportFront2"].as<Expression>();
        supportBack = config["supportBack"].as<Expression>();
        supportSideX = config["supportSideX"].as<Expression>();
        supportSideY = config["supportSideY"].as<Expression>();
        supportTurn = config["supportTurn"].as<Expression>();
        */
    }          
}  // motion
}  // modules   
