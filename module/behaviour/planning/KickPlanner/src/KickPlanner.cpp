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

#include "KickPlanner.h"

#include "extension/Configuration.h"

#include "message/behaviour/KickPlan.h"
#include "message/behaviour/ServoCommand.h"
#include "message/localisation/FieldObject.h"
#include "message/motion/KinematicsModels.h"
#include "message/motion/WalkCommand.h"
#include "message/support/FieldDescription.h"
#include "message/vision/VisionObjects.h"

#include "utility/behaviour/Action.h"
#include "utility/input/LimbID.h"
#include "utility/localisation/transform.h"
#include "utility/math/coordinates.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/motion/InverseKinematics.h"
#include "utility/nubugger/NUhelpers.h"
#include "utility/support/eigen_armadillo.h"
#include "utility/support/yaml_armadillo.h"

namespace module {
namespace behaviour {
namespace planning {

    using extension::Configuration;

    using message::behaviour::KickPlan;
    using KickType = message::behaviour::KickPlan::KickType;
    using message::behaviour::WantsToKick;
    using message::localisation::Ball;
    using message::localisation::Self;
    using message::input::Sensors;
    using message::motion::IKKickParams;
    using message::motion::KickCommand;
    using KickCommandType = message::motion::KickCommandType;
    using message::motion::KickScriptCommand;
    using message::motion::KickPlannerConfig;
    using message::motion::KinematicsModel;
    using message::support::FieldDescription;

    using LimbID = utility::input::LimbID;
    using utility::localisation::transform::RobotToWorldTransform;
    using utility::localisation::transform::WorldToRobotTransform;
    using utility::math::matrix::Transform3D;
    using utility::math::coordinates::sphericalToCartesian;
    using utility::motion::kinematics::legPoseValid;
    using utility::nubugger::graph;

    KickPlanner::KickPlanner(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), cfg(), ball_last_measurement_time(NUClear::clock::now()),lastTimeValid(NUClear::clock::now()) {


        on<Configuration>("KickPlanner.yaml").then([this](const Configuration& config) {
            cfg.max_ball_distance = config["max_ball_distance"].as<float>();
            cfg.kick_corridor_width = config["kick_corridor_width"].as<float>();
            cfg.seconds_not_seen_limit = config["seconds_not_seen_limit"].as<float>();
            cfg.kick_forward_angle_limit = config["kick_forward_angle_limit"].as<float>();
            emit(std::make_unique<KickPlannerConfig>(cfg));
            emit(std::make_unique<WantsToKick>(false));
        });

        on<Trigger<Ball>>().then([this](const Ball&){  log("Ball>>");   });
        on<Trigger<std::vector<Self>>>().then([this](const std::vector<Self>&){ log("std::");   });
        on<Trigger<FieldDescription>>().then([this](const FieldDescription&){  log("FieldDescription>>");   });
        on<Trigger<KickPlan>>().then([this](const KickPlan&){  log("KickPlan>>");   });
        on<Trigger<Sensors>>().then([this](const Sensors&){   log("Sensors>>");     });

        on<Trigger<Ball>,
            With<std::vector<Self>>,
            With<FieldDescription>,
            With<KickPlan>,
            With<Sensors>>().then([this] (
            const Ball& ball,
            const std::vector<Self>& selfs,
            const FieldDescription& fd,
            const KickPlan& kickPlan,
            const Sensors& sensors) {

            //Get time since last seen ball
            auto now = NUClear::clock::now();
            double secondsSinceLastSeen = std::chrono::duration_cast<std::chrono::microseconds>(now - ball_last_measurement_time).count() * 1e-6;

            //Compute target in robot coords
            auto self = selfs[0];
            // arma::vec2 kickTarget = {1,0,0}; //Kick forwards
            arma::vec2 kickTarget = WorldToRobotTransform(convert<double,2>(self.locObject.position), convert<double,2>(self.heading), convert<double,2>(kickPlan.target));

            Transform3D Htw = convert<double, 4, 4>(sensors.world);
            arma::vec3 ballPosition = Htw.transformPoint({ball.locObject.position[0], ball.locObject.position[1], fd.ball_radius});
            ball_last_measurement_time = ball.locObject.last_measurement_time;

            log("ballPos Torso", ballPosition);

            float KickAngle = std::fabs(std::atan2(kickTarget[1], kickTarget[0]));

            //Check whether to kick
            // log("kickTarget",kickTarget.t());
            // log("KickAngle",KickAngle);
            // log("ballPosition",ballPosition);
            // log("secondsSinceLastSeen",secondsSinceLastSeen);
            bool kickIsValid = kickValid(ballPosition);
            if(kickIsValid){
                lastTimeValid = now;
            }
            float timeSinceValid = (now - lastTimeValid).count() * (1 / double(NUClear::clock::period::den));

            // log("kick checks",secondsSinceLastSeen < cfg.seconds_not_seen_limit
            //     , kickIsValid
            //     , KickAngle < cfg.kick_forward_angle_limit);
            if(secondsSinceLastSeen < cfg.seconds_not_seen_limit
                && kickIsValid
                && KickAngle < cfg.kick_forward_angle_limit) {

                switch (kickPlan.kickType.value) {
                    case KickType::IK_KICK:
                        // NUClear::log("ik_kick");
                        if(ballPosition[1] > 0){
                            emit(std::make_unique<KickCommand>(KickCommand(Eigen::Vector3d(0.1, 0.04, 0), Eigen::Vector3d(1.0, 0.0, 0.0), KickCommandType::NORMAL)));
                            emit(std::make_unique<WantsToKick>(true));
                        } else {
                            emit(std::make_unique<KickCommand>(KickCommand(Eigen::Vector3d(0.1, -0.04, 0), Eigen::Vector3d(1.0, 0.0, 0.0), KickCommandType::NORMAL)));
                            emit(std::make_unique<WantsToKick>(true));
                        }
                        break;
                    case KickType::SCRIPTED:
                        // NUClear::log("scripted");
                        if(ballPosition[1] > 0){
                            emit(std::make_unique<KickScriptCommand>(KickScriptCommand(Eigen::Vector3d(1.0, 0.0, 0.0), LimbID::LEFT_LEG)));
                            emit(std::make_unique<WantsToKick>(true));;
                        } else {
                            emit(std::make_unique<KickScriptCommand>(KickScriptCommand(Eigen::Vector3d(1.0, 0.0, 0.0), LimbID::RIGHT_LEG)));
                            emit(std::make_unique<WantsToKick>(true));;
                        }
                        break;
                    default: throw new std::runtime_error("KickPlanner: Invalid KickType");
                }
            } else if(secondsSinceLastSeen > cfg.seconds_not_seen_limit || timeSinceValid > cfg.seconds_not_seen_limit){
                emit(std::make_unique<WantsToKick>(WantsToKick(false)));
            }

        });
    }


    bool KickPlanner::kickValid(const arma::vec3& ballPos){
        return (ballPos[0] > 0) && (ballPos[0] < cfg.max_ball_distance) && (std::fabs(ballPos[1]) < cfg.kick_corridor_width / 2.0);
    }


}
}
}

