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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "IKKick.h"

#include "messages/support/Configuration.h"
#include "messages/motion/KickCommand.h"
#include "messages/input/Sensors.h"
#include "messages/input/ServoID.h"
#include "messages/input/LimbID.h"
#include "messages/behaviour/ServoCommand.h"
#include "messages/behaviour/Action.h"
#include "messages/support/FieldDescription.h"


#include "utility/math/matrix/Transform3D.h"
#include "utility/motion/InverseKinematics.h"
#include "utility/motion/RobotModels.h"
#include "utility/support/yaml_armadillo.h"


namespace modules {
namespace motion {

    using messages::support::Configuration;
    using messages::motion::KickCommand;
    using messages::motion::KickFinished;
    using messages::input::Sensors;
    using messages::input::ServoID;
    using messages::input::LimbID;
    using messages::behaviour::ServoCommand;
    using messages::behaviour::RegisterAction;
    using messages::behaviour::ActionPriorites;
    using messages::support::FieldDescription;

    using utility::motion::kinematics::calculateLegJoints;
    using utility::math::matrix::Transform3D;
    using utility::motion::kinematics::calculateLegJoints;
    using utility::motion::kinematics::calculateLegJointsTeamDarwin;
    using utility::motion::kinematics::DarwinModel;

    struct ExecuteKick{};
    struct FinishKick{};

    bool doThings = false;

    IKKick::IKKick(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , id(size_t(this) * size_t(this) - size_t(this)) {

        on<Trigger<Configuration<IKKickConfig>>>([this] (const Configuration<IKKickConfig>& config){
            balancer.configure(config);
            lifter.configure(config);
            kicker.configure(config);

            KICK_PRIORITY = config["kick_priority"].as<float>();
            EXECUTION_PRIORITY = config["execution_priority"].as<float>();

            emit(std::make_unique<KickCommand>(
                config["target"].as<arma::vec3>(),
                config["direction"].as<arma::vec3>()
            ));
        });

        on<Trigger<KickCommand>>([this] (const KickCommand&) {
            // We want to kick!
            log("KickCommand");
            log("Priority: Kick Priority");

            if(!doThings) {
                doThings = true;
            }
            else {
                updatePriority(KICK_PRIORITY);
            }

        });

        on<Trigger<ExecuteKick>, With<KickCommand>, With<Sensors>>([this] (const ExecuteKick&, const KickCommand& command, const Sensors& sensors) {

            // TODO Work out which of our feet are going to be the support foot
            // TODO store the support foot
            // Assume leftFoot is support
            //leftFootIsSupport = true;

            // 4x4 homogeneous transform matrices for left foot and right foot relative to torso
            Transform3D leftFoot = sensors.forwardKinematics.find(ServoID::L_ANKLE_ROLL)->second;
            Transform3D rightFoot = sensors.forwardKinematics.find(ServoID::R_ANKLE_ROLL)->second;

            //Transform3D supportFoot

            //if(leftFootIsSupport == true) {

            //}


            // Convert the direction vector and position of the ball into left foot coordinates by multiplying the inverse of the
            // homogeneous transforms with the coordinates in torso space. 1 for a point and 0 for a vector.

            //TODO: talk to jake about why this is wrong
            ballPosition = leftFoot.i() * arma::join_cols(command.target, arma::vec({1}));
            goalPosition = leftFoot.i() * arma::join_cols(command.direction, arma::vec({0}));


            log("Got a new kick!");
            // Should this be kickcommand.target
            log("Target:", "x:", command.target[0], "y:", command.target[1], "z:", command.target[2]);
            log("Direction:", "x:", command.direction[0], "y:", command.direction[1], "z:", command.direction[2]);
            log("Ball Position in support foot coordinates:", "x:", ballPosition[0], "y:", ballPosition[1], "z:", ballPosition[2]);
            log("Goal Direction in support foot coordinates:", "x:", goalPosition[0], "y:", goalPosition[1], "z:", goalPosition[2]);

            // Enable our kick pather
            updater.enable();

            updatePriority(EXECUTION_PRIORITY);

            balancer.setKickParameters(supportFoot, ballPosition, goalPosition);
            lifter.setKickParameters(supportFoot, ballPosition, goalPosition);
            kicker.setKickParameters(supportFoot, ballPosition, goalPosition);
            
            balancer.start();
        });

        updater = on<Trigger<Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>>, With<Sensors>, Options<Single>>([this](const time_t&, const Sensors& sensors) {

            //Setup kick variables
            float deltaT = 1 / float(UPDATE_FREQUENCY);
            LimbID kickLegID = LimbID::RIGHT_LEG;
            LimbID supportLegID;
            if(kickLegID == LimbID::RIGHT_LEG){
                supportLegID = LimbID::LEFT_LEG;  
            } else {
                supportLegID = LimbID::RIGHT_LEG;
            }

            float footSeparation = 0.1;

            int negativeIfKickRight = kickLegID == LimbID::RIGHT_LEG ? -1 : 1;

            //State checker
            if(balancer.isStable()){
                lifter.start();
            }

            if(lifter.isStable()){
                kicker.start();
            }

            if(kicker.isFinished()){
                lifter.stop();
            }

            if(lifter.isFinished()){
                balancer.stop();
            }
            
            //Do things based on current state

            Transform3D kickFootGoal;
            Transform3D supportFootGoal;
            
            if(balancer.isRunning()){
                kickFootGoal =  balancer.getFootPose(sensors, deltaT).translate(arma::vec3({0, negativeIfKickRight * footSeparation, 0}));
                supportFootGoal = balancer.getFootPose(sensors, deltaT);
            }
            if(lifter.isRunning()){
                //TODO: CHECK ORDER
                kickFootGoal *= lifter.getFootPose(sensors, deltaT);
            }
            if(kicker.isRunning()){
                //TODO: CHECK ORDER
                kickFootGoal *= kicker.getFootPose(sensors, deltaT);
            }

            //Calculate IK and send waypoints

            float gainLegs = 30;
            float torque = 100;
            
            std::vector<std::pair<messages::input::ServoID, float>> joints;
            auto kickJoints = calculateLegJoints<DarwinModel>(kickFootGoal, kickLegID);
            auto supportJoints = calculateLegJoints<DarwinModel>(supportFootGoal, supportLegID);
            joints.insert(joints.end(),kickJoints.begin(),kickJoints.end());
            joints.insert(joints.end(),supportJoints.begin(),supportJoints.end());

            auto waypoints = std::make_unique<std::vector<ServoCommand>>();
            waypoints->reserve(16);

            time_t time = NUClear::clock::now() + std::chrono::nanoseconds(std::nano::den / UPDATE_FREQUENCY);

            for (auto& joint : joints) {
                waypoints->push_back({ id, time, joint.first, joint.second, gainLegs, torque});
            }

            emit(std::move(waypoints));


//END BALANCER
/*
//START FOOTLIFTER w.r.t Torso, Should be support foot coordinates????
            // 4x4 homogeneous transform matrices for left foot and right foot relative to torso
            Transform3D leftFoot = sensors.forwardKinematics.find(ServoID::L_ANKLE_ROLL)->second;
            Transform3D rightFoot = sensors.forwardKinematics.find(ServoID::R_ANKLE_ROLL)->second;
            
            // Finds the current position of the kick foot in support foot coordinates
            auto liftFootPosition  = leftFoot.i()*rightFoot.translation();

            // Finds the target position of the kick foot to lift foot in support foot coordinates
            auto liftFootTarget = leftFoot.i()*rightFoot.translation();
            
            // Raises the foot
            auto liftFootTarget.col(2) = liftFootTarget.col(2) + liftFootHeight;
            
            // Moves the heel backwards
            // Negative taken into account
            auto liftFootTarget.col(0) = liftFootTarget.col(0) - liftFootBack;

            // Direction in which the foot needs to be lifted
            auto liftFootDirection = liftFootTarget - liftFootPosition;
            // Normalise Direction
            auto normalLiftFootDirection = arma::normalise(liftFoot);

            // Net displacement to move kick foot
            auto liftFootDisplacement = (torsoShiftVelocity/UPDATE_FREQUENCY)*normalLiftFootDirection;
            
            // Convert displacement from support foot coordinates to kick foot coordinates
            auto liftFootDisplacmentKick = rightFoot.i()*leftFoot*arma::join_cols(liftFootDisplacement, arma::vec({0}));
            // Find the new position vector from the right foot to the torso
            auto newLiftFootPositionKick = rightFoot.i().translation() - liftFootDisplacementKick;
            
            // New transform matrix to give to inverse kinematics           
            auto newLiftFootPose = rightFoot;
            auto newLiftFootPose.col(3) = (arma::join_cols(newLiftFootPositionKick, arma::vec({1}))).t();

//END FOOTLIFTER

//START KICK Assume we want to kick the ball straight ahead, and that the foot will move in a straight line
            
            // Homogeneous transform matrices
            Transform3D leftFoot = sensors.forwardKinematics.find(ServoID::L_ANKLE_ROLL)->second;
            Transform3D rightFoot = sensors.forwardKinematics.find(ServoID::R_ANKLE_ROLL)->second;

            // Find position of the right foot w.r.t the torso
            auto kickFootPosition = rightFoot.translation();
            // Convert this position to support foot coordinates
            auto kickFootPosition = leftFoot.i()*kickFootPosition;
            // The direction we want the foot to move
            auto kickFootDirection = ballPosition - kickFootPosition;
            auto normalKickFootDirection = arma::normalise(kickFootDirection);
            // Net displacement we want the foot to move per cycle 
            auto kickFootDisplacement = (kickVelocity/UPDATE_FREQUENCY)*normalKickFootDirection;
            // Convert to kick foot coordinates
            auto kickFootDisplacementKick = rightFoot.i()*leftFoot*arma::join_cols(kickFootDisplacement, arma::vec({0}))
            // New position of the foot we want to move to in support foot coordinates
            auto newKickFootPositionKick = rightFoot.i().translation() - kickFootDisplacementKick;

            // New transform matrix to give to inverse kinematics
            auto newKickFootPose = rightFoot;
            auto newKickFootPose.col(3) = (arma::join_cols(newKickFootPositionKick, arma::vec({1}))).t()
//END KICK
*/
            // TODO We're always finished kicking because we never start :(
            // updatePriority(0);
        });

        on<Trigger<FinishKick>>([this] (const FinishKick&) {
            emit(std::move(std::make_unique<KickFinished>()));
            updater.disable();
            updatePriority(0);
        });

        emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction {
            id,
            "IK Kick",
            { std::pair<float, std::set<LimbID>>(0, { LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM }) },
            [this] (const std::set<LimbID>&) {
                emit(std::make_unique<ExecuteKick>());
                log("ExecuteKick");
            },
            [this] (const std::set<LimbID>&) {
                emit(std::make_unique<FinishKick>());
            },
            [this] (const std::set<ServoID>&) {
            }
        }));

            // Listen for kickcommands

            // When we get one ask for prioirty to kick

            // When we get the priority to kick

            // Run a loop that updates our foot position and body position


    }

    void IKKick::updatePriority(const float& priority) {
        emit(std::make_unique<ActionPriorites>(ActionPriorites { id, { priority }}));
    }





} // motion
} // modules

