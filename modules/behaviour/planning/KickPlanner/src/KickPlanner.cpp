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

#include "messages/motion/WalkCommand.h"
#include "messages/localisation/FieldObject.h"
#include "messages/support/Configuration.h"
#include "messages/behaviour/Action.h"
#include "messages/behaviour/ServoCommand.h"
#include "messages/behaviour/KickPlan.h"
#include "messages/vision/VisionObjects.h"
#include "messages/support/FieldDescription.h"
#include "messages/input/LimbID.h"

#include "utility/support/yaml_armadillo.h"
#include "utility/math/coordinates.h"
#include "utility/localisation/transform.h"
#include "utility/nubugger/NUhelpers.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/motion/RobotModels.h"
#include "utility/motion/InverseKinematics.h"



using messages::input::Sensors;
using messages::input::LimbID;
using messages::localisation::Ball;
using messages::localisation::Self;
using messages::motion::IKKickParams;
using messages::motion::KickCommand;
using messages::motion::KickPlannerConfig;
using messages::support::Configuration;
using messages::motion::WalkStopCommand;
using messages::input::LimbID;
using messages::behaviour::KickPlan;
using messages::support::FieldDescription;

using utility::math::matrix::Transform3D;
using utility::motion::kinematics::legPoseValid;
using utility::math::coordinates::sphericalToCartesian;
using utility::localisation::transform::WorldToRobotTransform;
using utility::localisation::transform::RobotToWorldTransform;
using utility::nubugger::graph;
using utility::motion::kinematics::DarwinModel;

namespace modules {
namespace behaviour {
namespace planning {

    KickPlanner::KickPlanner(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {


        on<Trigger<Configuration<KickPlanner> > >([this](const Configuration<KickPlanner>& config) {
            cfg.max_ball_distance = config["max_ball_distance"].as<float>();
            cfg.kick_corridor_width = config["kick_corridor_width"].as<float>();
            cfg.seconds_not_seen_limit = config["seconds_not_seen_limit"].as<float>();
            emit(std::make_unique<KickPlannerConfig>(cfg));
        });


        on< Trigger<Ball>, 
            With<std::vector<Self>>,
            With<FieldDescription>,
            With<KickPlan>,
            With<Sensors>,
            With<IKKickParams>>([this] (
            const Ball& ball,
            const std::vector<Self>& selfs,
            const FieldDescription& fd,
            const KickPlan& kickPlan,
            const Sensors& sensors,
            const IKKickParams& params) {

            //length to the closest end of the field abs(robot.x)
            

            // Defines the box within in which the kick target is changed from the centre 
            // of the oppposition goal to the perpendicular distance from the robot to the goal

            float maxKickRange = 0.60; //TODO: make configurable, only want to change at the last kick to avoid smart goalies
            float xTakeOverBox = maxKickRange;
            float error = 0.05;
            float buffer = error + 2*fd.ball_radius;            
            float yTakeOverBox = (fd.dimensions.field_width - fd.dimensions.goal_width)/2 + buffer;
            float xRobot = selfs.front().position[0];
            float yRobot = selfs.front().position[1];

            if(xRobot < xTakeOverBox && yRobot < yTakeOverBox) {
                newTarget = kickPlan.target;
                newTarget[1] = yRobot;
            } else {
                newTarget = kickPlan.target;
            }

            //Get time since last seen ball
            auto now = NUClear::clock::now();
            double secondsSinceLastSeen = std::chrono::duration_cast<std::chrono::microseconds>(now - ball.last_measurement_time).count() * 1e-6;
            
            //Compute target in robot coords
            auto self = selfs[0];
            arma::vec2 kickTarget = WorldToRobotTransform(self.position, self.heading, newTarget);
            arma::vec3 ballPosition = {ball.position[0],ball.position[1],fd.ball_radius}; 
            
            float KickAngle = std::fabs(std::atan2(kickTarget[1], kickTarget[0]));
            float kickAngleThreshold = M_PI_4;

            //Check whether to kick
            if(secondsSinceLastSeen < cfg.seconds_not_seen_limit
                && kickValid(ballPosition, params.stand_height, sensors)
                    && KickAngle < kickAngleThreshold){
                    emit(std::make_unique<KickCommand>(KickCommand{ballPosition, {kickTarget[0],kickTarget[1],0} }));
            }

        });
    }


    bool KickPlanner::kickValid(const arma::vec3& ballPos, float standHeight, const Sensors& sensors){
        Transform3D ballPose;
        //TODO: make this take into account the correct kick stand height
        Transform3D torsoToGround = sensors.orientationBodyToGround;
        torsoToGround.translation()[2] = standHeight;
        ballPose.translation() = torsoToGround.i().transformPoint(ballPos);
        ballPose.translate(arma::vec3({-DarwinModel::Leg::FOOT_LENGTH / 2,0,0}));
        return (legPoseValid<DarwinModel>(ballPose, LimbID::RIGHT_LEG) || legPoseValid<DarwinModel>(ballPose, LimbID::LEFT_LEG));
    }


}
}
}

