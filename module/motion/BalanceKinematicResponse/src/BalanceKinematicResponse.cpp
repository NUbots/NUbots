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
    using utility::motion::kinematics::DarwinModel;
    using utility::math::matrix::Transform2D;
    using utility::nubugger::graph;
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
/*=======================================================================================================*/
/*      NUCLEAR METHOD: FootPlacementPlanner
/*=======================================================================================================*/
>>>>>>> Balance Remodelling
    BalanceKinematicResponse::BalanceKinematicResponse(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        //Configure foot motion planner...
        on<Configuration>(CONFIGURATION_PATH).then([this] (const Configuration& config) 
        {
            configure(config.config);
        });

        on<Trigger<FootMotionUpdate>>().then([this] {
            hipRollCompensation();
            supportFootCompensation();
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
/*      METHOD: supportFootCompensation
/*=======================================================================================================*/
>>>>>>> Balance Remodelling
	void BalanceKinematicResponse::hipRollCompensation(arma::vec3 footPhases, LimbID swingLeg, Transform3D rightFootT, Transform3D leftFootT) 
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
/*      METHOD: supportFootCompensation
/*=======================================================================================================*/
     void BalanceKinematicResponse::supportFootCompensation(LimbID swingLeg, Transform3D rightFootT, Transform3D leftFootT) 
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
}  // motion
}  // modules   
