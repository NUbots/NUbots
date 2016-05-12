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
    using message::motion::FootMotionUpdate;
    using message::motion::TorsoMotionUpdate;
    using message::motion::EnableBalanceResponse;
    using message::motion::DisableBalanceResponse;
    using message::support::Configuration;

    using utility::support::Expression;
    using utility::motion::kinematics::DarwinModel;
    using utility::math::matrix::Transform2D;
    using utility::math::matrix::Transform3D;
    using utility::nubugger::graph;
<<<<<<< 12f7f13e6289b1c41bcd0957996a042cd2b4e247
<<<<<<< 183df72fb88459adef7436f0515b768fde100df7:module/motion/ModularWalkEngine/src/BalanceKinematicResponse.cpp
<<<<<<< 96dd3deaa26010585080bb841a4e4f1773925448
<<<<<<< d07c1d138e6fd9fbe83afad4f4e877e4b814ea95
<<<<<<< 3b87f3cda74870e5b6ca278b79d41f45e3932336
    
=======
=======
<<<<<<< HEAD
>>>>>>> Merging Changes
    /*=======================================================================================================*/
    //      NAME: torsoZMP
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
    void BalanceKinematicResponse::torsoZMP() //originally part of CalculateNewStep
    {
        uTorsoDestination = stepTorso(uLeftFootDestination, uRightFootDestination, 0.5);
=======
>>>>>>> Adding emits and triggers for flow of data
=======
    using extension::Configuration;
<<<<<<< 0b2306cf95d79477cc349f41ab6ab0c2d542fd0d
>>>>>>> Further Modularization in development hierarchy, reorganised directory structure:module/motion/BalanceKinematicResponse/src/BalanceKinematicResponse.cpp

=======
=======
>>>>>>> Remodelling #1
/*=======================================================================================================*/
/*      NUCLEAR METHOD: FootPlacementPlanner
/*=======================================================================================================*/
>>>>>>> Balance Remodelling
    BalanceKinematicResponse::BalanceKinematicResponse(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        //Configure foot motion planner...
        on<Configuration>("BalanceKinematicResponse.yaml").then("Balance Response Planner - Configure", [this] (const Configuration& config) 
        {
            configure(config.config);
        });

        updateHandle = on<Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>, With<Sensors>, Single, Priority::HIGH>()
        .then("Balance Response Planner - Update Waypoints", [this](const Sensors& sensors) 
        {
            //hipCompensation();
            //supportMassCompensation();
        }).disable();

        //Aim to avoid dependancy on target position to enhance statelessness and adaptive balance compensation...
        on<Trigger<FootMotionUpdate>>().then("Balance Response Planner - Received Update (Active Foot Position) Info", [this] 
        {
            
        });

        //Aim to avoid dependancy on target position to enhance statelessness and adaptive balance compensation...
        on<Trigger<TorsoMotionUpdate>>().then("Balance Response Planner - Received Update (Active Torso Position) Info", [this] 
        {
            
        });

        /*Aim to avoid dependancy on target position to enhance statelessness and adaptive balance compensation...
        on<Trigger<HeadMotionUpdate>>().then("Balance Response Planner - Received Update (Active Head Position) Info", [this] 
        {
            
        });*/

        on<Trigger<EnableBalanceResponse>>().then([this] (const EnableBalanceResponse& command) 
        {
            subsumptionId = command.subsumptionId;
            updateHandle.enable();
        });

        //If balance response no longer requested, cease updating...
        on<Trigger<DisableBalanceResponse>>().then([this] 
        {
            updateHandle.disable(); 
        });
    }
<<<<<<< 0b2306cf95d79477cc349f41ab6ab0c2d542fd0d
<<<<<<< 96dd3deaa26010585080bb841a4e4f1773925448
<<<<<<< d07c1d138e6fd9fbe83afad4f4e877e4b814ea95
>>>>>>> Remodelling...
=======
=======
    
>>>>>>> 38f588c4f50f3dc4b0637d4cf061a37535e6567d
>>>>>>> Merging Changes
=======
>>>>>>> Adding emits and triggers for flow of data
    /*=======================================================================================================*/
    //      NAME: hipRollCompensation
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
=======
/*=======================================================================================================*/
/*      METHOD: hipCompensation
/*=======================================================================================================*/
<<<<<<< 32586f734a07de99763562175d89ba4f5e291750
>>>>>>> Balance Remodelling
	void BalanceKinematicResponse::hipRollCompensation(arma::vec3 footPhases, LimbID swingLeg, Transform3D rightFootT, Transform3D leftFootT) 
=======
	void BalanceKinematicResponse::hipCompensation(const Sensors& sensors, arma::vec3 footPhases, LimbID swingLeg, Transform3D rightFootT, Transform3D leftFootT) 
>>>>>>> Remodelling balance #2
    {
        //If feature enabled, apply balance compensation through support actuator...
        if (balanceEnabled) 
        {
            //Evaluate scaled minimum distance of y(=1) phase position to the range [0,1] for hip roll parameter compensation... 
            double yBoundedMinimumPhase = std::min({1.0, footPhases[1] / 0.1, (1 - footPhases[1]) / 0.1});

            //Rotate foot around hip by the given hip roll compensation...
            if (swingLeg == LimbID::LEFT_LEG) 
            {
                rightFootT = rightFootT.rotateZLocal(-hipRollCompensation * yBoundedMinimumPhase, sensors.forwardKinematics.find(ServoID::R_HIP_ROLL)->second);
            }
            else 
            {
                leftFootT  = leftFootT.rotateZLocal( hipRollCompensation  * yBoundedMinimumPhase, sensors.forwardKinematics.find(ServoID::L_HIP_ROLL)->second);
            }
        }
    }
/*=======================================================================================================*/
/*      METHOD: supportMassCompensation
/*=======================================================================================================*/
    void BalanceKinematicResponse::supportMassCompensation(const Sensors& sensors, LimbID swingLeg, Transform3D rightFootT, Transform3D leftFootT) 
    {
        //If feature enabled, apply balance compensation through support actuator...
        if (balanceEnabled) 
        {
        	//Apply balance transformation to stipulated support actuator...
            balancer.balance(swingLeg == LimbID::LEFT_LEG ? rightFootT : leftFootT
                           , swingLeg == LimbID::LEFT_LEG ? LimbID::RIGHT_LEG : LimbID::LEFT_LEG
                           , sensors);
        }
    }  
/*=======================================================================================================*/
/*      METHOD: configure
/*=======================================================================================================*/
    void BalanceKinematicResponse::configure(const YAML::Node& config)
    {
        emitLocalisation = config["emit_localisation"].as<bool>();

        auto& stance = config["stance"];
        bodyHeight = stance["body_height"].as<Expression>();
        bodyTilt = stance["body_tilt"].as<Expression>();
        qLArmStart = stance["arms"]["left"]["start"].as<arma::vec>();
        qLArmEnd = stance["arms"]["left"]["end"].as<arma::vec>();
        qRArmStart = stance["arms"]["right"]["start"].as<arma::vec>();
        qRArmEnd = stance["arms"]["right"]["end"].as<arma::vec>();
        //setFootOffsetCoefficient(stance["foot_offset"].as<arma::vec>());
        // gToe/heel overlap checking values
        stanceLimitY2 = DarwinModel::Leg::LENGTH_BETWEEN_LEGS - stance["limit_margin_y"].as<Expression>();

        auto& gains = stance["gains"];
        gainArms = gains["arms"].as<Expression>();
        gainLegs = gains["legs"].as<Expression>();

        for(ServoID i = ServoID(0); i < ServoID::NUMBER_OF_SERVOS; i = ServoID(int(i)+1))
        {
            if(int(i) < 6)
            {
                jointGains[i] = gainArms;
            } 
            else 
            {
                jointGains[i] = gainLegs;
            }
        }

        auto& walkCycle = config["walk_cycle"];
        stepTime = walkCycle["step_time"].as<Expression>();
        zmpTime = walkCycle["zmp_time"].as<Expression>();
        hipRollCompensation = walkCycle["hip_roll_compensation"].as<Expression>();
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
        // balanceAmplitude = balance["amplitude"].as<Expression>();
        // balanceWeight = balance["weight"].as<Expression>();
        // balanceOffset = balance["offset"].as<Expression>();

        balancer.configure(balance);

        for(auto& gain : balance["servo_gains"])
        {
            float p = gain["p"].as<Expression>();
            ServoID sr = message::input::idFromPartialString(gain["id"].as<std::string>(),message::input::ServoSide::RIGHT);
            ServoID sl = message::input::idFromPartialString(gain["id"].as<std::string>(),message::input::ServoSide::LEFT);
            servoControlPGains[sr] = p;
            servoControlPGains[sl] = p;
        }
        /* TODO
        // gCompensation parameters
        toeTipCompensation = config["toeTipCompensation"].as<Expression>();
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
        STAND_SCRIPT_DURATION = config["STAND_SCRIPT_DURATION"].as<Expression>();
    }          
}  // motion
}  // modules   
