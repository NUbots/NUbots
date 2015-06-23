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

    struct KickVector{
        //
        LimbID supportFoot;
        // NEED the vector from the point on the surface of the ball where we want to kick to the front of the kick foot which is rightFootFront
        // KickPlanner has to add the radius of the all to get the location of the centre of the ball
        // point position of ball
        arma::vec3 ballPosition;
        // direction we want to kick the ball
        arma::vec3 goalDirection;
    };


    IKKick::IKKick(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , id(size_t(this) * size_t(this) - size_t(this)) {

        on<Trigger<Configuration<IKKick>>>([this] (const Configuration<IKKick>& config){
            KICK_PRIORITY = config["kick_priority"].as<float>();
            EXECUTION_PRIORITY = config["execution_priority"].as<float>();
            torsoShiftVelocity = config["torsoShiftVelocity"].as<float>();
            kickVelocity = config["kickVelocity"].as<float>();
            standHeight = config["standHeight"].as<float>();
            liftFootHeight = config["liftFootHeight"].as<float>();
            liftFootBack = config["liftFootBack"].as<float>();

            emit(std::make_unique<KickCommand>(KickCommand{
                config["direction"].as<arma::vec3>()
            }));
        });


        on<Trigger<KickCommand>>([this] (const KickCommand& kickCommand) {

            // We want to kick!
            updatePriority(KICK_PRIORITY);
        });

        on<Trigger<ExecuteKick>, With<KickCommand>, With<Sensors>>([this] (const ExecuteKick&, const KickCommand& command, const Sensors& sensors) {

            // TODO Work out which of our feet are going to be the support foot
            // TODO store the support foot
            // Assume leftFoot is support

            // 4x4 homogeneous transform matrices for left foot and right foot relative to torso
            Transform3D leftFoot = sensors.forwardKinematics.find(ServoID::L_ANKLE_ROLL)->second;
            Transform3D rightFoot = sensors.forwardKinematics.find(ServoID::R_ANKLE_ROLL)->second;

            // Convert the direction vector and position of the ball into left foot coordinates by multiplying the inverse of the
            // homogeneous transforms with the coordinates in torso space. 1 for a point and 0 for a vector.
            arma::vec4 ballposition = leftFoot.i() * arma::join_cols(command.target, arma::vec({1}));
            arma::vec4 goaldirection = leftFoot.i() * arma::join_cols(command.direction, arma::vec({0}));

            emit(std::make_unique<KickVector>(KickVector{
                LimbID::LEFT_LEG,
                ballPosition.rows(0,2),
                goalDirection.rows(0,2),
            }));

            log("Got a new kick!");
            log("Target:", "x:", command.target[0], "y:", command.target[1], "z:", command.target[2]);
            log("Direction:", "x:", command.direction[0], "y:", command.direction[1], "z:", command.direction[2]);
            log("position in support foot:", "x:", position[0], "y:", position[1], "z:", position[2]);
            log("Direction in support foot:", "x:", direction[0], "y:", direction[1], "z:", direction[2]);

            // Enable our kick pather
            updater.enable();

            updatePriority(EXECUTION_PRIORITY);
        });

        updater = on<Trigger<Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>>, With<Sensors>, With<KickVector>, Options<Single>>([this](const time_t&, const Sensors& sensors, const KickVector& kickVector) {
            //PSEUODCODE
            //State checker
/*
            if(balancer.isEnabled()){
                
                if(balancer.isBalanced() && !footLifter.isEnabled() && kicker.hasKicked()){
                    
                    footLifter.start();

                } else if(footLifter.isLifted() && !kicker.isEnabled()){
                    kicker.start();
                }

                if(kicker.hasKicked()){
                    footLifter.stop();
                }

                if(footLifter.done()){
                    balancer.stop();
                }

            }else{
                balancer.start()
            }

            //Do things based on current state
            
            if(balancer.isEnabled()){
                support.kickfoot = balancer.getFootPose();
            }
            if(kickLifter.isEnabled()){
                kickfoot += kickLifter.getFootPose();
            }
            if(kicker.isEnabled()){
                kickfoot += kicker.getFootPose();
            }

            emit(ServoWaypoints(InverseKinematics(support,kickfoot))); //look up in walk engine
*/
            // TODO use states

            // Get our foot positions
            Transform3D leftFoot = sensors.forwardKinematics.find(ServoID::L_ANKLE_ROLL)->second;
            Transform3D rightFoot = sensors.forwardKinematics.find(ServoID::R_ANKLE_ROLL)->second;

//START BALANCER
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
            auto torsoDisplacement = (torsoShiftVelocity/UPDATE_FREQUENCY)*normalTorsoDirection;

            // New position to give to inverse kinematics in support foot coordinates
            auto torsoNewPosition = torsoPosition + torsoDisplacement; 
/*
// TODO CHECK THIS!!!!!! Don't need to convert
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
/* ALTERNATIVE
            auto supportFootNewPose = leftFoot - arma::join_cols(arma::zeros(4,3), arma::join_cols(-1*torsoDisplacement, arma::vec({0})).t());        
            //auto supportFootNewPose = leftFoot;
            //auto supportFootNewPose.col(3)= (arma::join_cols(torsoNewPosition, arma::vec({1}))).t(); 
*/
            }


            // Lifted from WalkEngine::motionLegs()
            // Move torso to target
            //std::unique_ptr<std::vector<ServoCommand>> IKKick::motionLegs(Transform3D leftFootNewPose, Transform3D rightFootNewPose) {
            
            // Lifted from WalkEngine::updateStep()
            // Calculate leg joints
            float gainLegs = 80;
            float torque = 100;
            auto joints = calculateLegJoints<DarwinModel>(leftFootNewPose, rightFootNewPose);
            auto waypoints = std::make_unique<std::vector<ServoCommand>>();
            waypoints->reserve(16);

            time_t time = NUClear::clock::now() + std::chrono::nanoseconds(std::nano::den / UPDATE_FREQUENCY);

            for (auto& joint : joints) {
                waypoints->push_back({ id, time, joint.first, joint.second, gainLegs, torque}); // TODO: change 100 to torque, support separate gains for each leg
            }

            //return std::move(waypoints);
            //}

            emit(std::move(waypoints));

//END BALANCER



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
            // Net Displacement to move kick foot
            auto liftFootDisplacement = (torsoShiftVelocity/UPDATE_FREQUENCY)*normalLiftFootDirection;
            // Convert new position from support foot coordinates to kick foot coordinates
            auto newLiftFootDisplacmentKick = rightFoot.i()*leftFoot*arma::join_cols(newLiftDisplacement, arma::vec({0}));
            auto newLiftFootPositionKick = rightFoot.i().translation() - newLiftFootPositionKick;
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
            // New position of the foot we want to move to in support foot coordinates
            auto newKickFootPosition = kickFootPosition + kickFootDisplacement;
            // TODO CHECK THIS !!!!!
            // New transform matrix to give to inverse kinematics
            auto newKickFootPose = rightFoot;
            auto newKickFootPose.col(3) = (arma::join_cols(newKickFootPosition, arma::vec({1}))).t()
*/

//END KICK
            // TODO We're always finished kicking because we never start :(
            updatePriority(0);
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

