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
#include "utility/nubugger/NUhelpers.h"


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
    using utility::nubugger::graph;

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

            foot_separation = config["balancer"]["foot_separation"].as<float>();
            
            gain_legs =config["servo"]["gain"].as<float>();
            torque = config["servo"]["torque"].as<float>();

        });

        on<Trigger<KickCommand>>([this] (const KickCommand&) {
            // We want to kick!
            log("IKKICK: KickCommand received");

            updatePriority(KICK_PRIORITY);

        });

        on<Trigger<ExecuteKick>, With<KickCommand>, With<Sensors>>([this] (const ExecuteKick&, const KickCommand& command, const Sensors& sensors) {

            log("Got a new kick!");
            log("Target:", "x:", command.target[0], "y:", command.target[1], "z:", command.target[2]);
            log("Direction:", "x:", command.direction[0], "y:", command.direction[1], "z:", command.direction[2]);
            log("Ball Position in support foot coordinates:", "x:", ballPosition[0], "y:", ballPosition[1], "z:", ballPosition[2]);
            log("Goal Direction in support foot coordinates:", "x:", goalPosition[0], "y:", goalPosition[1], "z:", goalPosition[2]);

            // Enable our kick pather
            updater.enable();
            updatePriority(EXECUTION_PRIORITY);

            // 4x4 homogeneous transform matrices for left foot and right foot relative to torso
            Transform3D leftFoot = sensors.forwardKinematics.find(ServoID::L_ANKLE_ROLL)->second;
            Transform3D rightFoot = sensors.forwardKinematics.find(ServoID::R_ANKLE_ROLL)->second;

            // Work out which of our feet are going to be the support foot
            // Store the support foot and kick foot
            //TODO: fixe theses after debugging
            if(ballPosition[1] < - foot_separation / 2){
                supportFoot = LimbID::LEFT_LEG;
            }else{
                supportFoot = LimbID::RIGHT_LEG;
            }

            Transform3D torsoPose = (supportFoot == messages::input::LimbID::LEFT_LEG) ? leftFoot.i() : rightFoot.i();

            // Put the ball position from vision into torso coordinates
            arma::vec3 targetTorso = sensors.orientationBodyToGround.i().transformPoint(command.target);
            // Put the ball position into support foot coordinates
            arma::vec3 targetSupportFoot = torsoPose.transformPoint(targetTorso);

            // Put the goal from vision into torso coordinates
            arma::vec3 directionTorso = sensors.orientationBodyToGround.i().transformVector(command.direction);
            // Put the goal into support foot coordinates
            arma::vec3 directionSupportFoot = torsoPose.transformVector(directionTorso);
/*
            float hackVal = (supportFoot == messages::input::LimbID::LEFT_LEG) ? 0 : foot_separation;
            arma::vec3 hackDebuggingOffset = arma::vec3({0, hackVal, 0});
*/
            arma::vec3 ballPosition = targetSupportFoot;
            arma::vec3 goalPosition = directionSupportFoot;

            balancer.setKickParameters(supportFoot, ballPosition, goalPosition);
            lifter.setKickParameters(supportFoot, ballPosition, goalPosition);
            kicker.setKickParameters(supportFoot, ballPosition, goalPosition);
            
            balancer.start(sensors);
        });

        updater = on<Trigger<Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>>, With<Sensors>, Options<Single>>([this](const time_t&, const Sensors& sensors) {

            //Setup kick variables
            LimbID kickFoot;
            if(supportFoot == LimbID::RIGHT_LEG){
                kickFoot = LimbID::LEFT_LEG;  
            } else {
                kickFoot = LimbID::RIGHT_LEG;
            }

            int negativeIfKickRight = kickFoot == LimbID::RIGHT_LEG ? -1 : 1;

            //State checker
            if(balancer.isStable()){
                // std::cout << "balancer.isStable" << std::endl;
                lifter.start(sensors);
            }

            if(lifter.isStable()){
                // std::cout << "lifter.isStable" << std::endl;
                kicker.start(sensors);
            }

            if(kicker.isStable()){
                // std::cout << "kicker.isStable" << std::endl;
                kicker.stop();
            }

            if(kicker.isFinished()){
                // std::cout << "kicker.isFinished" << std::endl;
                lifter.stop();
            }

            if(lifter.isFinished()){
                // std::cout << "lifter.isFinished" << std::endl;
                balancer.stop();
            }

            if(balancer.isFinished()){
                // std::cout << "balancer.isFinished" << std::endl;
                emit(std::move(std::make_unique<FinishKick>()));
            }
            
            //Do things based on current state

            Transform3D kickFootGoal;
            Transform3D supportFootGoal;
            
            if(balancer.isRunning()){
                // std::cout << "balancer is running" << std::endl;
                Transform3D supportFootPose = balancer.getFootPose(sensors);
                supportFootGoal = supportFootPose;
                kickFootGoal = supportFootPose.translate(arma::vec3({0, negativeIfKickRight * foot_separation, 0}));
            }

            if(lifter.isRunning()){
                // std::cout << "lifter is running" << std::endl;
                //TODO: CHECK ORDER
                kickFootGoal *= lifter.getFootPose(sensors);
            }
            if(kicker.isRunning()){
                // std::cout << "kicker is running" << std::endl;
                //TODO: CHECK ORDER
                kickFootGoal *= kicker.getFootPose(sensors);
            }

            //Calculate IK and send waypoints

            std::vector<std::pair<messages::input::ServoID, float>> joints;

            // std::cout << "kickFootGoal\n" << kickFootGoal << std::endl;
            auto kickJoints = calculateLegJoints<DarwinModel>(kickFootGoal, kickFoot);
            // std::cout << "supportFootGoal\n" << supportFootGoal << std::endl;
            auto supportJoints = calculateLegJoints<DarwinModel>(supportFootGoal, supportFoot);
            joints.insert(joints.end(),kickJoints.begin(),kickJoints.end());
            joints.insert(joints.end(),supportJoints.begin(),supportJoints.end());

            auto waypoints = std::make_unique<std::vector<ServoCommand>>();
            waypoints->reserve(16);

            time_t time = NUClear::clock::now() + std::chrono::nanoseconds(std::nano::den / UPDATE_FREQUENCY);

            for (auto& joint : joints) {
                waypoints->push_back({ id, time, joint.first, joint.second, gain_legs, torque});
            }

            emit(std::move(waypoints));

        });

        updater.disable();

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

    }

    void IKKick::updatePriority(const float& priority) {
        emit(std::make_unique<ActionPriorites>(ActionPriorites { id, { priority }}));
    }

} // motion
} // modules

