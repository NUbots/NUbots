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
/*#include "ModularWalkEngine.h"

#include "utility/motion/RobotModels.h"
#include "utility/nubugger/NUhelpers.h"
/*===========================================================================================================*/
//      NAMESPACE(S)
/*===========================================================================================================*/
/*namespace module 
{
namespace motion 
{
    /*=======================================================================================================*/
    //      UTILIZATION REFERENCE(S)
    /*=======================================================================================================*/
    /*using message::input::LimbID;
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
    /*std::pair<Transform3D, Transform3D> LowerKinematicResponse::updateLowerBody(double phase, auto torsoWorld, auto feetLocal) 
    {
        // Transform feet targets to be relative to the robot torso...
        Transform3D leftFootTorso  =  leftFootLocal.worldToLocal(torsoWorld);
        Transform3D rightFootTorso = rightFootLocal.worldToLocal(torsoWorld);

        //DEBUGGING: Emit relative feet position with respect to robot torso model... 
        if (emitFootPosition)
        {
            emit(graph("Right foot position", rightFootTorso.translation()));
            emit(graph("Left  foot position",  leftFootTorso.translation()));
        }

        return {leftFootTorso, rightFootTorso};
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
    /*std::unique_ptr<std::vector<ServoCommand>> LowerKinematicResponse::motionLegs(std::vector<std::pair<ServoID, float>> joints) 
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
}  // motion
}  // modules