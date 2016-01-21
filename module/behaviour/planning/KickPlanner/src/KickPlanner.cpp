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

#include "message/motion/WalkCommand.h"
#include "message/localisation/FieldObject.h"
#include "message/support/Configuration.h"
#include "message/behaviour/Action.h"
#include "message/behaviour/ServoCommand.h"
#include "message/behaviour/KickPlan.h"
#include "message/vision/VisionObjects.h"
#include "message/support/FieldDescription.h"
#include "message/input/LimbID.h"

#include "utility/support/yaml_armadillo.h"
#include "utility/math/coordinates.h"
#include "utility/localisation/transform.h"
#include "utility/nubugger/NUhelpers.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/motion/RobotModels.h"
#include "utility/motion/InverseKinematics.h"


using message::input::Sensors;
using message::input::LimbID;
using message::localisation::Ball;
using message::localisation::Self;
using message::motion::IKKickParams;
using message::motion::KickCommand;
using message::motion::KickCommandType;
using message::motion::KickScriptCommand;
using message::motion::KickPlannerConfig;
using message::support::Configuration;
using message::motion::WalkStopCommand;
using message::input::LimbID;
using message::behaviour::KickPlan;
using message::behaviour::KickType;
using message::support::FieldDescription;

using utility::math::matrix::Transform3D;
using utility::motion::kinematics::legPoseValid;
using utility::math::coordinates::sphericalToCartesian;
using utility::localisation::transform::WorldToRobotTransform;
using utility::localisation::transform::RobotToWorldTransform;
using utility::nubugger::graph;
using utility::motion::kinematics::DarwinModel;

namespace module {
namespace behaviour {
namespace planning {

    KickPlanner::KickPlanner(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {


        on<Configuration>("KickPlanner.yaml").then([this](const Configuration& config) {
            cfg.max_ball_distance = config["max_ball_distance"].as<float>();
            cfg.kick_corridor_width = config["kick_corridor_width"].as<float>();
            cfg.seconds_not_seen_limit = config["seconds_not_seen_limit"].as<float>();
            cfg.kick_forward_angle_limit = config["kick_forward_angle_limit"].as<float>();
            emit(std::make_unique<KickPlannerConfig>(cfg));
        });


        on<Trigger<Ball>,
            With<std::vector<Self>>,
            With<FieldDescription>,
            With<KickPlan>,
            With<Sensors>,
            With<IKKickParams>>().then([this] (
            const Ball& ball,
            const std::vector<Self>& selfs,
            const FieldDescription& fd,
            const KickPlan& kickPlan,
            const Sensors& sensors,
            const IKKickParams& params) {

            //Get time since last seen ball
            auto now = NUClear::clock::now();
            double secondsSinceLastSeen = std::chrono::duration_cast<std::chrono::microseconds>(now - ball.last_measurement_time).count() * 1e-6;

            //Compute target in robot coords
            auto self = selfs[0];
            arma::vec2 kickTarget = WorldToRobotTransform(self.position, self.heading, kickPlan.target);
            arma::vec3 ballPosition = {ball.position[0], ball.position[1], fd.ball_radius};

            float KickAngle = std::fabs(std::atan2(kickTarget[1], kickTarget[0]));

            //Check whether to kick
            if(secondsSinceLastSeen < cfg.seconds_not_seen_limit
                && kickValid(ballPosition, params.stand_height, sensors)
                && KickAngle < cfg.kick_forward_angle_limit) {

                switch (kickPlan.kickType) {
                    case KickType::IK_KICK:
                        NUClear::log("ik_kick");
                        if(ballPosition[1] > 0){
                            emit(std::make_unique<KickCommand>(KickCommand({0.1,0.04,0}, {1, 0, 0}, KickCommandType::NORMAL )));
                        } else {
                            emit(std::make_unique<KickCommand>(KickCommand({0.1,-0.04,0}, {1, 0, 0}, KickCommandType::NORMAL )));
                        }
                        break;
                    case KickType::SCRIPTED:
                        // NUClear::log("scripted");
                        if(ballPosition[1] > 0){
                            emit(std::make_unique<KickScriptCommand>(KickScriptCommand({{1, 0, 0}, LimbID::LEFT_LEG})));
                        } else {
                            emit(std::make_unique<KickScriptCommand>(KickScriptCommand({{1, 0, 0}, LimbID::RIGHT_LEG})));
                        }
                        break;
                    default: throw new std::runtime_error("KickPlanner: Invalid KickType");
                }
            }

        });
    }


    bool KickPlanner::kickValid(const arma::vec3& ballPos, float /*standHeight*/, const Sensors& /*sensors*/){
        // IK check seems broken
        // Transform3D ballPose;
        // Transform3D torsoToGround = sensors.orientationBodyToGround;
        // torsoToGround.translation()[2] = standHeight;
        // ballPose.translation() = torsoToGround.i().transformPoint(ballPos);
        // ballPose.translate(arma::vec3({-DarwinModel::Leg::FOOT_LENGTH / 2,0,0}));
        // return (legPoseValid<DarwinModel>(ballPose, LimbID::RIGHT_LEG) || legPoseValid<DarwinModel>(ballPose, LimbID::LEFT_LEG));
        return (ballPos[0] > 0) && (ballPos[0] < cfg.max_ball_distance) && (std::fabs(ballPos[1]) < cfg.kick_corridor_width / 2.0);
    }


}
}
}

