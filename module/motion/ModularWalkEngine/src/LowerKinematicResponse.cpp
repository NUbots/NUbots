/*----------------------------------------------DOCUMENT HEADER----------------------------------------------*/
/*===========================================================================================================*/
/*
 * This file is part of ModularWalkEngine.
 *
 * ModularWalkEngine is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ModularWalkEngine is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ModularWalkEngine.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */
/*===========================================================================================================*/
/*----------------------------------------CONSTANTS AND DEFINITIONS------------------------------------------*/
/*===========================================================================================================*/
//      INCLUDE(S)
/*===========================================================================================================*/
#include "ModularWalkEngine.h"

#include "utility/motion/RobotModels.h"
#include "utility/nubugger/NUhelpers.h"
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
    /*=======================================================================================================*/
    //      NAME: updateLowerBody
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
    std::unique_ptr<std::vector<ServoCommand>> ModularWalkEngine::updateLowerBody(double phase, const Sensors& sensors) 
    {
        //Interpret robot's zero point reference from torso for positional transformation into relative space...
        uTorsoLocal = zmpTorsoCompensation(phase, zmpCoefficients, zmpParams, stepTime, zmpTime, phase1Single, phase2Single, uSupport, uLeftFootDestination, uLeftFootSource, uRightFootDestination, uRightFootSource);

        //Interpret robot's world position from torso as local positional reference...
        Transform2D uTorsoWorld = uTorsoLocal.localToWorld({-DarwinModel::Leg::HIP_OFFSET_X, 0, 0});

        //Collect attributed metrics that describe the robot's spatial orientation in environmental space...
        Transform3D torsoWorldMetrics = arma::vec6({uTorsoWorld.x(), uTorsoWorld.y(), bodyHeight, 0, bodyTilt, uTorsoWorld.angle()});

        //DEBUGGING: Emit relative torsoWorldMetrics position with respect to world model... 
        if (emitLocalisation) 
        {
            localise(uTorsoWorld);
        }

        // Transform feet targets to be relative to the robot torso...
        Transform3D leftFootTorso = leftFoot.worldToLocal(torsoWorldMetrics);
        Transform3D rightFootTorso = rightFoot.worldToLocal(torsoWorldMetrics);

        //Compute compensation moment to apply hip roll and support foot balance...
        hipCompensation(footPhases, swingLeg, rightFootTorso, leftFootTorso);

        //DEBUGGING: Emit relative feet position with respect to robot torso model... 
        if (emitFootPosition)
        {
            emit(graph("Foot phase motion", phase));
            emit(graph("Right foot position", rightFootTorso.translation()));
            emit(graph("Left  foot position",  leftFootTorso.translation()));
        }

        auto joints = calculateLegJointsTeamDarwin<DarwinModel>(leftFootTorso, rightFootTorso);

        return (motionLegs(joints)); 
    }
    /*=======================================================================================================*/
    //      NAME: motionLegs
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
    std::unique_ptr<std::vector<ServoCommand>> ModularWalkEngine::motionLegs(std::vector<std::pair<ServoID, float>> joints) 
    {
        auto waypoints = std::make_unique<std::vector<ServoCommand>>();
        waypoints->reserve(16);

        NUClear::clock::time_point time = NUClear::clock::now() + std::chrono::nanoseconds(std::nano::den / UPDATE_FREQUENCY);

        for (auto& joint : joints) 
        {
            waypoints->push_back({ subsumptionId, time, joint.first, joint.second, jointGains[joint.first], 100 }); // TODO: support separate gains for each leg
        }

        return std::move(waypoints);
    }

    void ModularWalkEngine::hipCompensation(arma::vec3 footPhases, LimbID swingLeg, Transform3D rightFootT, Transform3D leftFootT) 
    {
        //If feature enabled, apply balance compensation through robot hip alignment and support foot...
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
        
            balancer.balance(swingLeg == LimbID::LEFT_LEG ? rightFootT : leftFootT
                           , swingLeg == LimbID::LEFT_LEG ? LimbID::RIGHT_LEG : LimbID::LEFT_LEG
                           , sensors);
        }
    }
}  // motion
}  // modules