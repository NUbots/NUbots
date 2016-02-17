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
#include "FootMotionPlanner.h"

#include "utility/motion/RobotModels.h"
#include "utility/nubugger/NUhelpers.h"
#include "extension/Configuration.h"
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
    using extension::Configuration;
    /*=======================================================================================================*/
    //      NAME: FootMotionPlanner
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
    FootMotionPlanner::FootMotionPlanner(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) 
    {
    	//Configure foot motion planner...
        on<Configuration>(CONFIGURATION_PATH).then([this] (const Configuration& config) 
        {
            configure(config.config);
        });

        //Transform analytical foot positions in accordance with the stipulated targets...
        updateHandle = on<Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>, With<Sensors>, Single, Priority::HIGH>()
        .then([this](const Sensors& sensors) {
            updateFootPosition(getMotionPhase(), getLeftFootDestination(), getRightFootDestination());
        }).disable();

        //In the event of a new foot step target specified by the foot placement planning module...
        on<Trigger<FootStepTarget>>().then([this] 
        {
            if(supportFoot)
            {
                setLeftFootDestination(targetDestination);
            }
            else
            {
                setRightFootDestination(targetDestination);
            }
            setDestinationTime(targetTime);    
        });

        //If foot motion no longer requested, cease updating...
        on<Trigger<DisableWalkEngineCommand>>().then([this] 
        {
            updateHandle.disable(); 
        });
    }
    /*=======================================================================================================*/
    //      NAME: getTime
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
    double FootMotionPlanner::getTime() 
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(NUClear::clock::now().time_since_epoch()).count() * 1E-6;
    }
    /*=======================================================================================================*/
    //      NAME: getDestinationTime
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
    double FootMotionPlanner::getDestinationTime()
    {
        return (destinationTime);
    }
    /*=======================================================================================================*/
    //      NAME: setDestinationTime
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
    void FootMotionPlanner::setDestinationTime(double inDestinationTime)
    {
        destinationTime = inDestinationTime;
    }
    /*=======================================================================================================*/
    //      NAME: getLeftFootDestination
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
    double FootMotionPlanner::getLeftFootDestination()
    {
        setNewStepReceived(false);
        return (leftFootDestination.front());
    }
    /*=======================================================================================================*/
    //      NAME: setLeftFootDestination
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
    void FootMotionPlanner::setLeftFootDestination(double inLeftFootDestination)
    {
        setNewStepReceived(true);
        leftFootDestination.push(inLeftFootDestination);
    }
    /*=======================================================================================================*/
    //      NAME: getRightFootDestination
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
    double FootMotionPlanner::getRightFootDestination()
    {
        setNewStepReceived(false);
        return (rightFootDestination.front());
    }
    /*=======================================================================================================*/
    //      NAME: setRightFootDestination
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
    void FootMotionPlanner::setRightFootDestination(double inRightFootDestination)
    {
        setNewStepReceived(true);
        rightFootDestination.push(inRightFootDestination);
    }
    /*=======================================================================================================*/
    //      NAME: isNewStepReceived
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
    bool FootMotionPlanner::getNewStepReceived()
    {
        return (updateStepInstruction);
    }
    /*=======================================================================================================*/
    //      NAME: setNewStepReceived
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
    void FootMotionPlanner::setNewStepReceived(bool inUpdateStepInstruction)
    {
        updateStepInstruction = inUpdateStepInstruction;
    }
    /*=======================================================================================================*/
    //      NAME: getMotionPhase
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
    double FootMotionPlanner::getMotionPhase()
    {
        // Obtain current system time...
        double currentTime = getTime();
        // The percentage completed of the current step, range: [0,1]...
        double motionPhase = 1 - ((getDestinationTime() - currentTime) / stepTime);
        // Bind phase value to range [0,1], emit status if step completed...
        if (motionPhase > 1)
        {
            resetMotionPhase();
        }
        return (motionPhase);
    }
    /*=======================================================================================================*/
    //      NAME: resetMotionPhase
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
    void FootMotionPlanner::resetMotionPhase()
    {
        motionPhase = std::fmod(motionPhase, 1);
        if(getNewStepReceived())
        {
            emit(std::make_unique<StepCompleted>();
        }
    }
    /*=======================================================================================================*/
    //      NAME: footPhase
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
    arma::vec3 FootMotionPlanner::footPhase(double phase, double phase1Single, double phase2Single) 
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
    //      NAME: updateFootPosition
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
    void FootMotionPlanner::updateFootPosition(double phase, auto leftFootDestination, auto rightFootDestination) 
    {
        //Instantiate unitless phases for x(=0), y(=1) and z(=2) foot motion...
        arma::vec3 footPhases = footPhase(phase, phase1Single, phase2Single);

        //Lift foot by amount depending on walk speed
        auto& limit = (velocityCurrent.x() > velocityHigh ? accelerationLimitsHigh : accelerationLimits); // TODO: use a function instead
        float speed = std::min(1.0, std::max(std::abs(velocityCurrent.x() / limit[0]), std::abs(velocityCurrent.y() / limit[1])));
        float scale = (step_height_fast_fraction - step_height_slow_fraction) * speed + step_height_slow_fraction;
        footPhases[2] *= scale;

        //Interpolate Transform2D from start to destination - deals with flat resolved movement in (x,y) coordinates
        if (swingLeg == LimbID::RIGHT_LEG) 
        {
            //Vector field function??
            uRightFoot = uRightFootSource.interpolate(footPhases[0], rightFootDestination());
        }
        else
        {
            //Vector field function??
            uLeftFoot  = uLeftFootSource.interpolate(footPhases[0], leftFootDestination();
        }
        
        //Translates foot motion into z dimension for stepping in three-dimensional space...
        Transform3D leftFootLocal  = uLeftFoot;
        Transform3D rightFootLocal = uRightFoot;

        //Lift swing leg - manipulate(update) z component of foot position to action movement with a varying altitude locus...
        if (swingLeg == LimbID::RIGHT_LEG) 
        {
            rightFootLocal = rightFootLocal.translateZ(stepHeight * footPhases[2]);
        }
        else
        {
            leftFootLocal  = leftFootLocal.translateZ(stepHeight * footPhases[2]);
        }      

        //DEBUGGING: Emit relative feet position phase with respect to robot state... 
        if (emitFootPosition)
        {
            emit(graph("Foot phase motion", phase));
        }

        //Broadcast struct of updated foot motion data at corresponding phase identity...
        emit(std:make_unique<FootMotionUpdate>(phase, leftFootLocal, rightFootLocal));
    }
}  // motion
}  // modules
