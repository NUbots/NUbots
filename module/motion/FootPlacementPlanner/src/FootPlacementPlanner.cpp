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
#include "FootPlacementPlanner.h"
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
    using message::motion::NewStepTargetInfo;
    using message::motion::NewFootTargetInfo;
    using message::motion::TorsoMotionUpdate;
    using message::motion::EnableFootPlacement;
    using message::motion::DisableFootPlacement;
    using message::motion::FootStepRequested;
    using message::motion::NewWalkCommand;
    using message::motion::FootPlacementStopped;
    using message::support::Configuration;

    using utility::support::Expression;
    using message::motion::kinematics::KinematicsModel;
    using utility::math::matrix::Transform2D;
    using utility::math::matrix::Transform3D;
    using utility::nubugger::graph;
/*=======================================================================================================*/
//      NUCLEAR METHOD: FootPlacementPlanner
/*=======================================================================================================*/
    FootPlacementPlanner::FootPlacementPlanner(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) 
        , DEBUG(false), DEBUG_ITER(0), EPSILON(1e-6)
        , updateHandle(), generateStandScriptReaction()
        , startFromStep(false)
        , torsoPositionTransform(), torsoPositionSource(), torsoPositionDestination()
        , leftFootPositionTransform(), leftFootSource(), rightFootPositionTransform()
        , rightFootSource(), leftFootDestination(), rightFootDestination(), uSupportMass()
        , activeForwardLimb(), activeLimbInitial(LimbID::LEFT_LEG)
        , bodyTilt(0.0), bodyHeight(0.0), stanceLimitY2(0.0), stepTime(0.0), stepHeight(0.0)
        , step_height_slow_fraction(0.0f), step_height_fast_fraction(0.0f)
        , stepLimits(arma::fill::zeros), footOffsetCoefficient(arma::fill::zeros), uLRFootOffset()
        , beginStepTime(0.0), STAND_SCRIPT_DURATION(0.0), lastVeloctiyUpdateTime()
        , velocityHigh(0.0), accelerationTurningFactor(0.0), velocityLimits(arma::fill::zeros)
        , accelerationLimits(arma::fill::zeros), accelerationLimitsHigh(arma::fill::zeros)
        , velocityCurrent(), velocityCommand()
        , zmpCoefficients(arma::fill::zeros), zmpParameters(arma::fill::zeros)
        , zmpTime(0.0), phase1Single(0.0), phase2Single(0.0)
        , kinematicsModel()
        , lastFootGoalRotation(), footGoalErrorSum()
    {
        //Configure foot motion planner...
        on<Configuration>("WalkEngine.yaml").then("Foot Placement Planner - Configure", [this] (const Configuration& config) 
        {
            configure(config.config);
        });

        //Define kinematics model for physical calculations...
        on<Trigger<KinematicsModel>>().then("WalkEngine - Update Kinematics Model", [this](const KinematicsModel& model)
        {
            kinematicsModel = model;
        });

        //When new torso destination is computed, inform FPP in preparation for next footstep...
        on<Trigger<TorsoMotionUpdate>>().then("Foot Placement Planner - Received Torso Destination Update", [this] (const TorsoMotionUpdate& info) 
        {                     
            if(DEBUG) { log<NUClear::TRACE>("Messaging: Foot Placement Planner - Received Torso Destination Update(0)"); }    
                setTorsoPosition(info.frameArms);  
                setTorsoDestination(info.frameDestination);     
            if(DEBUG) { log<NUClear::TRACE>("Messaging: Foot Placement Planner - Received Torso Destination Update(1)"); }
        });

        updateHandle = on<Trigger<FootStepRequested>>().then("Foot Placement Planner - Calculate Target Foot Position", [this]
        {          
            if(DEBUG) { log<NUClear::TRACE>("Messaging: Foot Placement Planner - Calculate Target Foot Position(0)"); }             
                calculateNewStep(getVelocityCurrent(), getTorsoDestination(), getTorsoPosition());
            if(DEBUG) { log<NUClear::TRACE>("Messaging: Foot Placement Planner - Calculate Target Foot Position(1)"); }
        }).disable();

        on<Trigger<NewWalkCommand>>().then("Foot Placement Planner - Update Foot Target", [this] (const NewWalkCommand& command) 
        {                            
            if(DEBUG) { log<NUClear::TRACE>("Messaging: Foot Placement Planner - On New Walk Command(0)"); }          
                setVelocityCommand(command.velocityTarget);
                calculateNewStep(getVelocityCurrent(), getTorsoDestination(), getTorsoPosition());          
            if(DEBUG) { log<NUClear::TRACE>("Messaging: Foot Placement Planner - On New Walk Command(1)"); }
        }).enable();

        on<Trigger<EnableFootPlacement>>().then([this]
        {         
            postureInitialize(); // Reset stance as we don't know where our limbs are.
            updateHandle.enable();
        });

        //If foot motion no longer requested, cease updating...
        on<Trigger<DisableFootPlacement>>().then([this] 
        {
            updateHandle.disable();
        });
    }
/*=======================================================================================================*/
//      NAME: calculateNewStep
/*=======================================================================================================*/
    void FootPlacementPlanner::calculateNewStep(const Transform2D& inVelocityCurrent, const Transform2D& inTorsoDestination, const Transform2D& inTorsoPosition) 
    {      
        // Update current velocity, bounded by acceleration and physical limitations...
        updateVelocity();

        // Alternate active forward limb between left and right legs...
        setActiveForwardLimb((getActiveForwardLimb() == LimbID::LEFT_LEG) ? LimbID::RIGHT_LEG : LimbID::LEFT_LEG);

        // Update foot and torso source positions to reflect the stance achieved by the previous footstep...
        setLeftFootSource(getLeftFootDestination());
        setRightFootSource(getRightFootDestination());
        setTorsoSource(inTorsoDestination);

        arma::vec2 supportMod = arma::zeros(2); // support point modulation for wallkick

        if(isZeroVelocityRequest()) 
        {         
            setVelocityCurrent(arma::zeros(3));
            // Stop with feet together by targetting swing leg next to support leg...
            if (getActiveForwardLimb() == LimbID::RIGHT_LEG) 
            {
                setRightFootDestination(getLeftFootSource().localToWorld(-2 * uLRFootOffset));
            }
            else 
            {
                setLeftFootDestination(getRightFootSource().localToWorld( 2 * uLRFootOffset));
            }
        }
        else 
        {           
            // Calculate projected foot position to achieve input velocity...
            if (getActiveForwardLimb() == LimbID::RIGHT_LEG) 
            {
                setRightFootDestination(getNewFootTarget(inVelocityCurrent, getActiveForwardLimb()));
            }
            else 
            {
                setLeftFootDestination(getNewFootTarget(inVelocityCurrent,  getActiveForwardLimb()));
            }
        }       
        // apply velocity-based support point modulation for SupportMass
        if (getActiveForwardLimb() == LimbID::RIGHT_LEG) 
        {
            Transform2D uLeftFootTorso = getTorsoSource().worldToLocal(getLeftFootSource());            
            Transform2D uTorsoModded = inTorsoPosition.localToWorld({supportMod[0], supportMod[1], 0});             
            Transform2D uLeftFootModded = uTorsoModded.localToWorld(uLeftFootTorso);
            setSupportMass(uLeftFootModded.localToWorld({-getFootOffsetCoefficient(0), -getFootOffsetCoefficient(1), 0})); 
        }
        else 
        {
            Transform2D uRightFootTorso = getTorsoSource().worldToLocal(getRightFootSource());          
            Transform2D uTorsoModded = inTorsoPosition.localToWorld({supportMod[0], supportMod[1], 0});                 
            Transform2D uRightFootModded = uTorsoModded.localToWorld(uRightFootTorso);
            setSupportMass(uRightFootModded.localToWorld({-getFootOffsetCoefficient(0), getFootOffsetCoefficient(1), 0}));  
        }                   
        emit(std::make_unique<NewStepTargetInfo>(stepTime, inVelocityCurrent, getSupportMass())); //New Step Target Information
        emit(std::make_unique<NewFootTargetInfo>(getLeftFootSource(), getRightFootSource(), getActiveForwardLimb(), getLeftFootDestination(), getRightFootDestination()));  //New Foot Target Information
    }
/*=======================================================================================================*/
//      METHOD: getNewFootTarget
/*=======================================================================================================*/
    Transform2D FootPlacementPlanner::getNewFootTarget(const Transform2D& inVelocity, const LimbID& inActiveForwardLimb) 
    {   
        // Negative if right leg to account for the mirroring of the foot target
        int8_t sign = (inActiveForwardLimb == LimbID::LEFT_LEG) ? 1 : -1;

        // Get midpoint between the two feet
        Transform2D midPoint = getLeftFootSource().interpolate(0.5, getRightFootSource());  

        // Get midpoint 1.5 steps in future
        // Note: The reason for 1.5 rather than 1 is because it takes an extra 0.5 steps
        // for the torso to reach a given position when you want both feet together   
        Transform2D forwardPoint = midPoint.localToWorld(1.5 * inVelocity);     

        // Offset to towards the foot in use to get the target location          
        Transform2D footTarget = forwardPoint.localToWorld(sign * uLRFootOffset);       

        // Start applying step limits:
        // Get the vector between the feet and clamp the components between the min and max step limits
        setSupportMass((inActiveForwardLimb == LimbID::LEFT_LEG) ? getRightFootSource() : getLeftFootSource());       
        Transform2D feetDifference = getSupportMass().worldToLocal(footTarget);
        feetDifference.x()     = std::min(std::max(feetDifference.x(),            stepLimits(0,0)), stepLimits(0,1));
        feetDifference.y()     = std::min(std::max(feetDifference.y()     * sign, stepLimits(1,0)), stepLimits(1,1)) * sign;
        feetDifference.angle() = std::min(std::max(feetDifference.angle() * sign, stepLimits(2,0)), stepLimits(2,1)) * sign;
        // end applying step limits

        // Start feet collision detection:
        // Uses a rough measure to detect collision and move feet apart if too close
        double overlap = kinematicsModel.Leg.FOOT_LENGTH / 2.0 * std::abs(feetDifference.angle());
        feetDifference.y() = std::max(feetDifference.y() * sign, stanceLimitY2 + overlap) * sign;
        // End feet collision detection


        // Update foot target to be 'feetDistance' away from the support foot
        footTarget = getSupportMass().localToWorld(feetDifference);        

        // TODO: Improve feedback logic to provide 'smart' feet coordination...

        return footTarget;
    }
/*=======================================================================================================*/
//      METHOD: updateVelocity
/*=======================================================================================================*/
    void FootPlacementPlanner::updateVelocity() 
    { 
        // slow accelerations at high speed
        //TODO: Add acceleration to velocity (Replace initialStep)
        auto now = NUClear::clock::now();
        double deltaT = std::chrono::duration_cast<std::chrono::microseconds>(now - lastVeloctiyUpdateTime).count() * 1e-6;
        lastVeloctiyUpdateTime = now;

        auto& limit = (getVelocityCurrent().x() > velocityHigh ? accelerationLimitsHigh : accelerationLimits) * deltaT; // TODO: use a function instead

        Transform2D velocityDifference = arma::zeros(3); // Current velocity differential
        velocityDifference.x()     = std::min(std::max(getVelocityCommand().x()     - getVelocityCurrent().x(),     -limit[0]), limit[0]);
        velocityDifference.y()     = std::min(std::max(getVelocityCommand().y()     - getVelocityCurrent().y(),     -limit[1]), limit[1]);
        velocityDifference.angle() = std::min(std::max(getVelocityCommand().angle() - getVelocityCurrent().angle(), -limit[2]), limit[2]);

        setVelocityCurrent(Transform2D({getVelocityCurrent().x() + velocityDifference.x(), getVelocityCurrent().y() + velocityDifference.y(), getVelocityCurrent().angle() + velocityDifference.angle()}));
    }
/*=======================================================================================================*/
//      METHOD: Reset The Stance of the Humanoid to Initial Valid Stance
/*=======================================================================================================*/
    void FootPlacementPlanner::postureInitialize() 
    {        
        // Default Initial Torso Position...
        setTorsoPosition({-getFootOffsetCoefficient(0), 0, 0});
   
        // Default Initial Left  Foot Position...
        setLeftFootPosition(getTorsoPosition().localToWorld({getFootOffsetCoefficient(0), kinematicsModel.Leg.HIP_OFFSET_Y - getFootOffsetCoefficient(1), 0}));
     
        // Default Initial Right Foot Position...
        setRightFootPosition(getTorsoPosition().localToWorld({getFootOffsetCoefficient(0), -kinematicsModel.Leg.HIP_OFFSET_Y + getFootOffsetCoefficient(1), 0}));

        // Default Active Forward Limb...
        setActiveForwardLimb(activeLimbInitial);
      
        // Default Left  Foot Targets...
        setLeftFootSource(getLeftFootPosition());
        setLeftFootDestination(getLeftFootPosition());

        // Default Right Foot Targets...
        setRightFootSource(getRightFootPosition());
        setRightFootDestination(getRightFootPosition());

        // Default Support Mass Reference...
        setSupportMass(getTorsoPosition());

        // Default Inner Foot Offset...
        uLRFootOffset = {0, kinematicsModel.Leg.HIP_OFFSET_Y - getFootOffsetCoefficient(1), 0};    
    }
/*=======================================================================================================*/
//      METHOD: reset
/*=======================================================================================================*/
    void FootPlacementPlanner::reset() 
    {
        setTorsoPosition({-getFootOffsetCoefficient(0), 0, 0});
        setLeftFootPosition({0, kinematicsModel.Leg.HIP_OFFSET_Y, 0});
        setRightFootPosition({0, -kinematicsModel.Leg.HIP_OFFSET_Y, 0});

        setTorsoSource(arma::zeros(3));
        setTorsoDestination(arma::zeros(3));
        setLeftFootSource(arma::zeros(3));
        setLeftFootDestination(arma::zeros(3));
        setRightFootSource(arma::zeros(3));
        setRightFootDestination(arma::zeros(3));

        setVelocityCurrent(arma::zeros(3));
        setVelocityCommand(arma::zeros(3));

        // gGyro stabilization variables
        setActiveForwardLimb(activeLimbInitial);

        // gStandard offset
        uLRFootOffset = {0, kinematicsModel.Leg.HIP_OFFSET_Y - getFootOffsetCoefficient(1), 0};

        // gWalking/Stepping transition variables
        startFromStep = false;

        // interrupted = false;
    }
/*=======================================================================================================*/
//      METHOD: Time
/*=======================================================================================================*/
    double FootPlacementPlanner::getTime() 
    {
        if(DEBUG) { log<NUClear::TRACE>("System Time:%f\n\r", double(NUClear::clock::now().time_since_epoch().count()) * (1.0 / double(NUClear::clock::period::den))); }
        return (double(NUClear::clock::now().time_since_epoch().count()) * (1.0 / double(NUClear::clock::period::den)));
    }  
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Velocity
/*=======================================================================================================*/
    Transform2D FootPlacementPlanner::getVelocityCurrent() 
    {
        return (velocityCurrent);
    }
    void FootPlacementPlanner::setVelocityCurrent(Transform2D inVelocityCurrent) 
    {
        velocityCurrent = inVelocityCurrent;
    }   
    Transform2D FootPlacementPlanner::getVelocityCommand() 
    {
        return (velocityCommand);
    }
    void FootPlacementPlanner::setVelocityCommand(Transform2D inVelocityCommand) 
    {
        velocityCommand = inVelocityCommand;
    }      
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: isZeroVelocityRequest
/*=======================================================================================================*/
    bool FootPlacementPlanner::isZeroVelocityRequest()
    {
        return  (
                    abs(getVelocityCommand().x())        < EPSILON   &&
                    abs(getVelocityCommand().y())        < EPSILON   &&
                    abs(getVelocityCommand().angle())    < EPSILON
                );
    }    
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Torso Position
/*=======================================================================================================*/
    Transform2D FootPlacementPlanner::getTorsoPosition()
    {
        return (torsoPositionTransform);
    }
    void FootPlacementPlanner::setTorsoPosition(const Transform2D& inTorsoPosition)
    {
        torsoPositionTransform = inTorsoPosition;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Torso Source
/*=======================================================================================================*/
    Transform2D FootPlacementPlanner::getTorsoSource()
    {
        return (torsoPositionSource);
    }
    void FootPlacementPlanner::setTorsoSource(const Transform2D& inTorsoSource)
    {
        torsoPositionSource = inTorsoSource;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Torso Destination
/*=======================================================================================================*/
    Transform2D FootPlacementPlanner::getTorsoDestination()
    {
        return (torsoPositionDestination);
    }
    void FootPlacementPlanner::setTorsoDestination(const Transform2D& inTorsoDestination)
    {
        torsoPositionDestination = inTorsoDestination;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Active Forward Limb
/*=======================================================================================================*/
    LimbID FootPlacementPlanner::getActiveForwardLimb()
    {
        return (activeForwardLimb);
    }
    void FootPlacementPlanner::setActiveForwardLimb(const LimbID& inActiveForwardLimb)
    {
        activeForwardLimb = inActiveForwardLimb;
    }          
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Support Mass
/*=======================================================================================================*/
    Transform2D FootPlacementPlanner::getSupportMass()
    {
        return (uSupportMass);
    }
    void FootPlacementPlanner::setSupportMass(const Transform2D& inSupportMass)
    {
        uSupportMass = inSupportMass;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Foot Offset Coefficient
/*=======================================================================================================*/
    double FootPlacementPlanner::getFootOffsetCoefficient(int index)
    {
        return (footOffsetCoefficient[index]);
    }
    void FootPlacementPlanner::setFootOffsetCoefficient(const arma::vec2& inFootOffsetCoefficient)
    {
        footOffsetCoefficient = inFootOffsetCoefficient;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: setFootOffsetCoefficient
/*=======================================================================================================*/
    void FootPlacementPlanner::setFootOffsetCoefficient(int index, double inValue)
    {
        footOffsetCoefficient[index] = inValue;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Left Foot Position
/*=======================================================================================================*/
    Transform2D FootPlacementPlanner::getLeftFootPosition()
    {
        return (leftFootPositionTransform);
    }
    void FootPlacementPlanner::setLeftFootPosition(const Transform2D& inLeftFootPosition)
    {
        leftFootPositionTransform = inLeftFootPosition;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Right Foot Position
/*=======================================================================================================*/
    Transform2D FootPlacementPlanner::getRightFootPosition()
    {
        return (rightFootPositionTransform);
    }
    void FootPlacementPlanner::setRightFootPosition(const Transform2D& inRightFootPosition)
    {
        rightFootPositionTransform = inRightFootPosition;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Left Foot Source
/*=======================================================================================================*/
    Transform2D FootPlacementPlanner::getLeftFootSource()
    {
        return (leftFootSource);
    }
    void FootPlacementPlanner::setLeftFootSource(const Transform2D& inLeftFootSource)
    {
        leftFootSource = inLeftFootSource;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Right Foot Source
/*=======================================================================================================*/
    Transform2D FootPlacementPlanner::getRightFootSource()
    {
        return (rightFootSource);
    }
    void FootPlacementPlanner::setRightFootSource(const Transform2D& inRightFootSource)
    {
        rightFootSource = inRightFootSource;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Left Foot Destination
/*=======================================================================================================*/
    Transform2D FootPlacementPlanner::getLeftFootDestination()
    {
        return (leftFootDestination);   
    }
    void FootPlacementPlanner::setLeftFootDestination(const Transform2D& inLeftFootDestination)
    {
        leftFootDestination = inLeftFootDestination;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Right Foot Destination
/*=======================================================================================================*/
    Transform2D FootPlacementPlanner::getRightFootDestination()
    {
        return (rightFootDestination);
    }
    void FootPlacementPlanner::setRightFootDestination(const Transform2D& inRightFootDestination)
    {
        rightFootDestination = inRightFootDestination;
    }
/*=======================================================================================================*/
//      METHOD: configure
/*=======================================================================================================*/
    void FootPlacementPlanner::configure(const YAML::Node& config)
    {
std::cout << "Test Config(S)\n\r";           
        auto& wlk = config["walk_engine"];
        auto& fpp = config["foot_placement_planner"];

        auto& debug = fpp["debugging"];
        DEBUG = debug["enabled"].as<bool>();
        
        auto& stance = wlk["stance"];
        auto& body   = stance["body"];
        bodyHeight   = body["height"].as<Expression>();
        bodyTilt     = body["tilt"].as<Expression>();
        setFootOffsetCoefficient(stance["foot_offset"].as<arma::vec>());
        stanceLimitY2 = kinematicsModel.Leg.LENGTH_BETWEEN_LEGS() - stance["limit_margin_y"].as<Expression>(); 
        STAND_SCRIPT_DURATION = stance["STAND_SCRIPT_DURATION"].as<Expression>();   

        auto& walkCycle = wlk["walk_cycle"];
        stepTime    = walkCycle["step_time"].as<Expression>();
        stepHeight  = walkCycle["step"]["height"].as<Expression>();
        stepLimits  = walkCycle["step"]["limits"].as<arma::mat::fixed<3,2>>();

        step_height_slow_fraction = walkCycle["step"]["height_slow_fraction"].as<float>();
        step_height_fast_fraction = walkCycle["step"]["height_fast_fraction"].as<float>();

        auto& velocity = walkCycle["velocity"];
        velocityLimits = velocity["limits"].as<arma::mat::fixed<3,2>>();
        velocityHigh   = velocity["high_speed"].as<Expression>();

        auto& acceleration = walkCycle["acceleration"];
        accelerationLimits          = acceleration["limits"].as<arma::vec>();
        accelerationLimitsHigh      = acceleration["limits_high"].as<arma::vec>();
        accelerationTurningFactor   = acceleration["turning_factor"].as<Expression>();

        phase1Single = walkCycle["single_support_phase"]["start"].as<Expression>();
        phase2Single = walkCycle["single_support_phase"]["end"].as<Expression>();         
std::cout << "Test Config(E)\n\r";      
    }
}  // motion    
}  // modules