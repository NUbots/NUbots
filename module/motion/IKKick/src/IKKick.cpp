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

#include "message/support/Configuration.h"
#include "message/motion/KickCommand.h"
#include "message/input/Sensors.h"
#include "message/input/ServoID.h"
#include "message/input/LimbID.h"
#include "message/behaviour/ServoCommand.h"
#include "message/behaviour/Action.h"
#include "message/support/FieldDescription.h"
#include "message/motion/WalkCommand.h"
#include "message/behaviour/KickPlan.h"


#include "utility/math/matrix/Transform3D.h"
#include "utility/motion/InverseKinematics.h"
#include "utility/motion/RobotModels.h"
#include "utility/support/yaml_armadillo.h"
#include "utility/nubugger/NUhelpers.h"



namespace module {
namespace motion {

    using message::support::Configuration;
    using message::motion::WalkStopCommand;
    using message::motion::KickCommand;
    using message::motion::IKKickParams;
    using message::motion::KickFinished;
    using message::input::Sensors;
    using message::input::ServoID;
    using message::input::LimbID;
    using message::behaviour::ServoCommand;
    using message::behaviour::RegisterAction;
    using message::behaviour::ActionPriorites;
    using message::behaviour::KickPlan;
    using message::behaviour::KickType;
    using message::support::FieldDescription;

    using utility::motion::kinematics::calculateLegJoints;
    using utility::math::matrix::Transform3D;
    using utility::motion::kinematics::calculateLegJoints;
    using utility::motion::kinematics::DarwinModel;
    using utility::nubugger::graph;

    struct ExecuteKick{};
    struct FinishKick{};

    IKKick::IKKick(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , subsumptionId(size_t(this) * size_t(this) - size_t(this)) {

        on<Configuration>("IKKick.yaml").then([this] (const Configuration& config) {
            balancer.configure(config);
            kicker.configure(config);

            KICK_PRIORITY = config["kick_priority"].as<float>();
            EXECUTION_PRIORITY = config["execution_priority"].as<float>();

            foot_separation = config["balancer"]["foot_separation"].as<float>();

            gain_legs =config["servo"]["gain"].as<float>();
            torque = config["servo"]["torque"].as<float>();

            auto& balanceConfig = config["active_balance"];
            feedback_active = balanceConfig["enabled"].as<bool>();
            feedbackBalancer.configure(balanceConfig);

            //Emit useful info to KickPlanner
            emit(std::make_unique<IKKickParams>(IKKickParams{config["balancer"]["stand_height"].as<float>()}));
            emit(std::make_unique<KickPlan>(KickPlan{{4.5,0},KickType::IK_KICK}));

        });

        on<Startup>().then("IKKick Startup", [this] {
            //Default kick plan at enemy goals
            emit(std::make_unique<KickPlan>(KickPlan{{4.5,0},KickType::IK_KICK}));
        });

        on<Trigger<KickCommand>>().then([this] {
            // We want to kick!

            emit(std::make_unique<WalkStopCommand>(subsumptionId)); // Stop the walk

            updatePriority(KICK_PRIORITY);
        });

        on<Trigger<ExecuteKick>, With<KickCommand>, With<Sensors>>().then([this] (const KickCommand& command, const Sensors& sensors) {

            // Enable our kick pather
            updater.enable();
            updatePriority(EXECUTION_PRIORITY);


            // 4x4 homogeneous transform matrices for left foot and right foot relative to torso
            Transform3D leftFoot = sensors.forwardKinematics.find(ServoID::L_ANKLE_ROLL)->second;
            Transform3D rightFoot = sensors.forwardKinematics.find(ServoID::R_ANKLE_ROLL)->second;

            // Work out which of our feet are going to be the support foot
            // Store the support foot and kick foot
            if(command.target[1] < 0){
                supportFoot = LimbID::LEFT_LEG;
            }else{
                supportFoot = LimbID::RIGHT_LEG;
            }

            Transform3D torsoPose = (supportFoot == message::input::LimbID::LEFT_LEG) ? leftFoot.i() : rightFoot.i();

            // Put the ball position from vision into torso coordinates
            arma::vec3 targetTorso = sensors.kinematicsBodyToGround.i().transformPoint(command.target);
            // Put the ball position into support foot coordinates
            arma::vec3 targetSupportFoot = torsoPose.transformPoint(targetTorso);

            // Put the goal from vision into torso coordinates
            arma::vec3 directionTorso = sensors.kinematicsBodyToGround.i().transformVector(command.direction);
            // Put the goal into support foot coordinates
            arma::vec3 directionSupportFoot = torsoPose.transformVector(directionTorso);

            arma::vec3 ballPosition = targetSupportFoot;
            ballPosition[2] = 0.05; //TODO: figure out why ball height is unreliable
            arma::vec3 goalPosition = directionSupportFoot;
            goalPosition[2] = 0.0; //TODO: figure out why ball height is unreliable

            balancer.setKickParameters(supportFoot, ballPosition, goalPosition);
            kicker.setKickParameters(supportFoot, ballPosition, goalPosition);

            balancer.start(sensors);
        });

        updater = on<Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>, With<Sensors>, Single>().then([this] (const Sensors& sensors) {

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
                kicker.start(sensors);
            }

            if(kicker.isStable()){
                kicker.stop(sensors);
                balancer.stop(sensors);
            }

            if(balancer.isFinished()){
                emit(std::move(std::make_unique<FinishKick>()));
            }

            //Do things based on current state

            Transform3D kickFootGoal;
            Transform3D supportFootGoal;

            //Move torso over support foot
            if(balancer.isRunning()){
                Transform3D supportFootPose = balancer.getFootPose(sensors);
                supportFootGoal = supportFootPose;
                kickFootGoal = supportFootPose.translate(arma::vec3({0, negativeIfKickRight * foot_separation, 0}));
            }

            //Move foot to ball to kick
            if(kicker.isRunning()){
                kickFootGoal *= kicker.getFootPose(sensors);
            }

            //Balance based on the IMU

            if(feedback_active){
                feedbackBalancer.balance(supportFootGoal,supportFoot,sensors);
            }

            //Calculate IK and send waypoints
            std::vector<std::pair<message::input::ServoID, float>> joints;

            //IK
            auto kickJoints = calculateLegJoints<DarwinModel>(kickFootGoal, kickFoot);
            auto supportJoints = calculateLegJoints<DarwinModel>(supportFootGoal, supportFoot);

            //Combine left and right legs
            joints.insert(joints.end(),kickJoints.begin(),kickJoints.end());
            joints.insert(joints.end(),supportJoints.begin(),supportJoints.end());

            //Create message to send to servos
            auto waypoints = std::make_unique<std::vector<ServoCommand>>();
            waypoints->reserve(16);

            //Goal time is by next frame
            NUClear::clock::time_point time = NUClear::clock::now();

            //Push back each servo command
            for (auto& joint : joints) {
                waypoints->push_back({ subsumptionId, time, joint.first, joint.second, gain_legs, torque});
            }

            //Send message
            emit(std::move(waypoints));

        });

        updater.disable();

        on<Trigger<FinishKick>>().then([this] {
            emit(std::move(std::make_unique<KickFinished>()));
            updater.disable();
            updatePriority(0);
        });

        emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction {
            subsumptionId,
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

    }

    void IKKick::updatePriority(const float& priority) {
        emit(std::make_unique<ActionPriorites>(ActionPriorites { subsumptionId, { priority }}));
    }


} // motion
} // modules

