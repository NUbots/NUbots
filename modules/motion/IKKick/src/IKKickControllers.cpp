/*
 * This file is part of the NUbots Codebase.
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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "IKKickControllers.h"

using messages::input::Sensors;
using messages::input::LimbID;
using messages::input::ServoID;
using utility::math::matrix::Transform3D;


namespace modules{
namespace motion{

	Transform3D KickBalancer::getFootPose(const Sensors& sensors, float deltaT){
		    // Get our foot positions
			float standHeight = 0.18;
			float torsoShiftVelocity = 0.01;
            Transform3D leftFoot = sensors.forwardKinematics.find(ServoID::L_ANKLE_ROLL)->second;
            Transform3D rightFoot = sensors.forwardKinematics.find(ServoID::R_ANKLE_ROLL)->second;

//            Transform3D IKKick::balance(Transform3D leftFoot, Transform3D rightFoot) {
            // Moving the torso to balance on support foot before kick

            // Obtain the position of the torso and the direction in which the torso needs to move
            
                // The position that the torso needs to move to in support foot coordinates
            auto torsoTarget = arma::vec({0, 0, standHeight}); 

                // Find position vector from support foot to torso in support foot coordinates.
            auto torsoPosition = leftFoot.i().translation();
           
                // Find the direction in which we want to shift the torso in support foot coordinates
            auto torsoDirection = torsoTarget - torsoPosition;
           
                // Normalise the direction
            auto normalTorsoDirection = arma::normalise(torsoDirection);

/*
            // Finds out when the torso is within tolerance of the target
            //TODO define displacementTolerance
            
            if (arma::abs(torsoTarget - torsoPosition) <= std::abs(torsoShiftVelocity/UPDATE_FREQUENCY)) {    
                if (arma::abs(torsoTarget - torsoPosition <= diplacementTolerance) {
                    //TODO Run Kick!
                    state = State::BALANCE
                } else {
                    auto torsoDisplacement = ((torsoShiftVelocity/UPDATE_FREQUENCY)/2)*normalTorsoDirection;
                }
            } else {
                auto torsoDisplacement = (torsoShiftVelocity/UPDATE_FREQUENCY)*normalTorsoDirection;
            }
*/

            // torsoShiftVelocity [m/s] is the configurable velocity that the torso should move at
            // Net displacement of torso each 1/UPDATE_FREQUENCY seconds
            // ((P1-P0))*velocity*(1/UPDATE_FREQUENCY)
            auto torsoDisplacement = (torsoShiftVelocity * deltaT) * normalTorsoDirection;

            // New position to give to inverse kinematics in support foot coordinates
            auto torsoNewPosition = torsoPosition + torsoDisplacement;

            Transform3D supportFootNewPose = leftFoot;            
            // Proof of Line Below
            // auto supportFootNewPose = leftFoot - arma::join_cols(arma::zeros(4,3), arma::join_cols(-1*torsoDisplacement, arma::vec({0})).t());        
            // = {deltaRotation, deltaTranslation; 0, 1}
            // = {0, torsoPosition + torsoDisplacement; 0, 1}

            // Put new position of the left foot to the torso in left foot coordinates into the transform matrix
            supportFootNewPose.col(3) = arma::join_cols(torsoNewPosition, arma::vec({1}));  
         
            // TODO Need a way around this.
            Transform3D leftFootNewPose = supportFootNewPose;
            Transform3D rightFootNewPose = rightFoot;

/*
// TODO CHECK THIS!!!!!! Don't need to convert DELETE THIS
            // Convert the new torso position into torso coordinates
            auto torsoNewPositionTorso = leftFoot*(arma::join_cols(torsoNewPosition, arma::vec({0})).t());
            // Find support foot position relative to the torso
            auto supportFootPosition = leftFoot.translation();
            // Moving torso is equivalent to moving foot in the opposite direction
            // New support foot position
            auto supportFootNewPosition = supportFootPosition - torsoNewPositionTorso;
            //HAVE TO CONVERT BACK TO SUPPORT FOOT COORDINATES TO PUT IN MATRIX
            // TODO give position to inverse kinematics
            // Puts together matrix to give to inverse kinematics
            auto supportFootNewPose = leftFoot;
            auto supportFootNewPose.col(3)= (arma::join_cols(supportFootNewPosition, arma::vec({1}))).t();
            auto kickFootNewPose = rightFoot;
*/
            //}


            // Lifted from WalkEngine::motionLegs()
            // Move torso to target
            //std::unique_ptr<std::vector<ServoCommand>> IKKick::motionLegs(Transform3D leftFootNewPose, Transform3D rightFootNewPose) {
          
            // Lifted from WalkEngine::updateStep()
            // Calculate leg joints

		return leftFootNewPose;
	}

	Transform3D FootLifter::getFootPose(const Sensors& sensors, float deltaT){
		return Transform3D();
	}

	Transform3D Kicker::getFootPose(const Sensors& sensors, float deltaT){
		return Transform3D();
	}
}
}