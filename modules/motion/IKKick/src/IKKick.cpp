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
        arma::vec3 position;
        // direction we want to kick the ball
        arma::vec3 direction;
    };

    IKKick::IKKick(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , id(size_t(this) * size_t(this) - size_t(this)) {

        on<Trigger<Configuration<IKKick>>>([this] (const Configuration<IKKick>& config){
            KICK_PRIORITY = config["kick_priority"].as<float>();
            EXECUTION_PRIORITY = config["execution_priority"].as<float>();
            torsoShiftVelocity = config["torsoShiftVelocity"].as<float>();
            standHeight = config["standHeight"].as<float>();

            emit(std::make_unique<KickCommand>(KickCommand{
                config["target"].as<arma::vec3>(),
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
            arma::vec4 position = leftFoot.i() * arma::join_cols(command.target, arma::vec({1}));
            arma::vec4 direction = leftFoot.i() * arma::join_cols(command.direction, arma::vec({0}));

            emit(std::make_unique<KickVector>(KickVector{
                LimbID::LEFT_LEG,
                position.rows(0,2),
                direction.rows(0,2),
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

            emit(InverseKinematics(support,kickfoot));


            // TODO use states

            float gain = 80;
            float torque = 100;

            // Get our foot positions
            Transform3D leftFootTorso = sensors.forwardKinematics.find(ServoID::L_ANKLE_ROLL)->second;
            Transform3D rightFootTorso = sensors.forwardKinematics.find(ServoID::R_ANKLE_ROLL)->second;

            // Moving the torso to balance on support foot before kick

            // Obtain the position of the torso and the direction in which the torso needs to move
            
                // The position that the torso needs to move to in support foot coordinates
            auto torsoTarget = arma::vec({0, 0, standHeight}); 
            
                // Find position vector from support foot to torso in leftFoot coordinates.
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
                    //TODO Stop!!
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

            // New position to give to inverse kinematics
            auto torsoNewPosition = torsoPosition + torsoDisplacement; 

            // Convert the new torso position into torso coordinates
            auto torsoNewPositionTorso = leftFoot*arma::join_cols(torsoNewPosition, arma::vec({0}));
            // Find support foot position relative to the torso
            auto supportFootPosition = leftFoot.translation();
            // Moving torso is equivalent to moving foot in the opposite direction
            // New support foot position
            auto supportFootNewPosition = supportFootPosition - torsoNewPositionTorso;

            // TODO give position to inverse kinematics            


            // TODO We're always finished kicking because we never start :(
            updatePriority(0);

            // // Add the length from the centre of the foot to the front to get starting position of curve as front of the foot.
            // auto rightFootFront = leftFoot.i() * arma::join_cols((rightfoot*Transform3D::translation(arma::vec({TOE_LENGTH,0,0})))translation(), arma::vec({1}));

            //// If our feet are at the target then stop
            //// if position is off this won't work
            // if(position - rightFootFront <= ball_radius) {
                // emit(std::make_unique<FinishKick>());
            // }
            //else {
                //// Do a series of transforms and whatnot to put leftFootTorso and RightFootTorso where you want them to be in 1/90th of a second!
                //// TODO Move everything towards where it needs to be
                //
                //
                //
            // }

            // Move our feet towards our target
            // auto joints = calculateLegJointsTeamDarwin<DarwinModel>(leftFootTorso, rightFootTorso);
            // auto waypoints = std::make_unique<std::vector<ServoCommand>>();

            // waypoints->reserve(16);
            // time_t time = NUClear::clock::now() + std::chrono::nanoseconds(std::nano::den / UPDATE_FREQUENCY);

            // for (auto& joint : joints) {
            //     waypoints->push_back({ id, time, joint.first, joint.second, gain, torque });
            // }

            // emit(std::move(waypoints));
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


}
}

