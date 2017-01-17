/*
 * This file is part of the NUbots Codebase.
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
 * Copyright 2015 NUBots <nubots@nubots.net>
 */

#include "NUbugger.h"

#include "message/behaviour/proto/WalkPath.h"
#include "message/behaviour/proto/KickPlan.h"
#include "message/input/proto/Image.h"
#include "message/input/proto/Sensors.h"
#include "message/vision/VisionObjects.h"
#include "message/motion/proto/WalkCommand.h"

#include "utility/time/time.h"
#include "utility/localisation/transform.h"
#include "utility/support/eigen_armadillo.h"

/**
 * @author Monica Olejniczak
 */
namespace module {
namespace support {

    using NUClear::message::CommandLineArguments;
    using message::behaviour::proto::Behaviour;
    using message::behaviour::proto::WalkPath;
    using message::behaviour::proto::KickPlan;
    using message::input::proto::Image;
    using message::input::proto::Sensors;
    using message::input::proto::GameState;
    using message::localisation::proto::Self;
    using LocalisationBall = message::localisation::proto::Ball;
    using VisionGoal = message::vision::Goal;
    using VisionBall = message::vision::Ball;
    using message::motion::proto::WalkCommand;

    using utility::localisation::transform::RobotToWorldTransform;

    /**
     * @brief Provides triggers to send overview information over the network using the overview
     * instance variable.
     */
    void NUbugger::provideOverview() {

        handles["overview"].push_back(on<Every<1, std::chrono::seconds>, Single, Priority::LOW>().then([this] {
            // Send the overview packet
            send(overview, 0, false, NUClear::clock::now());
        }));

        handles["overview"].push_back(on<Trigger<CommandLineArguments>, Single, Priority::LOW>().then([this] (const CommandLineArguments& arguments) {

            std::string role_name = arguments.at(0);
            auto index = role_name.rfind('/');
            if (index != std::string::npos) {
                role_name = role_name.substr(index + 1);
            }
            overview.role_name = role_name;

        }));

        handles["overview"].push_back(on<Trigger<Behaviour::State>, Single, Priority::LOW>().then([this] (const Behaviour::State& state) {

            overview.behaviour_state = state;

        }));

        handles["overview"].push_back(on<Trigger<KickPlan>, Single, Priority::LOW>().then([this] (const KickPlan& /*kickPlan*/) {

            // TODO fix runtime error:
            // *overview.kick_target = convert<double, 2>(kickPlan.target);

        }));

        handles["overview"].push_back(on<Trigger<Sensors>, Single, Priority::LOW>().then([this] (const Sensors& sensors) {

            overview.voltage = sensors.voltage;
            overview.battery = sensors.battery;

        }));

        handles["overview"].push_back(on<Trigger<std::vector<Self>>, Single, Priority::LOW>().then([this](const std::vector<Self>& selfs) {

            // Retrieve the first self in the vector.
            Self self = selfs.front();

            // Set robot position.
            overview.robot_position = self.locObject.position;

            // Set robot position covariance.
            overview.robot_position_covariance = self.locObject.position_cov;

            // Set robot heading.
            overview.robot_heading = self.heading;
        }));

        handles["overview"].push_back(on<Trigger<std::vector<LocalisationBall>>, With<std::vector<Self>>, Single, Priority::LOW>()
            .then([this](const std::vector<LocalisationBall>& balls, const std::vector<Self>& selfs) {

            // Retrieve the first ball and self in the vector.
            LocalisationBall ball = balls.front();
            Self self = selfs.front();

            // Set local ball position.
            overview.ball_position = ball.locObject.position;

            // Set world ball position.
            overview.ball_world_position = convert<double, 2>(RobotToWorldTransform(convert<double, 2>(self.locObject.position), 
                                                                                    convert<double, 2>(self.heading), 
                                                                                    convert<double, 2>(ball.locObject.position)));
        }));

        handles["overview"].push_back(on<Trigger<Image>, Single, Priority::LOW>().then([this] {

            overview.last_camera_image = NUClear::clock::now();
        }));

        handles["overview"].push_back(on<Trigger<std::vector<VisionBall>>, Single, Priority::LOW>().then([this] (const std::vector<VisionBall>& balls) {

            if (!balls.empty()) {
                overview.last_seen_ball = NUClear::clock::now();
            }
        }));

        handles["overview"].push_back(on<Trigger<std::vector<VisionGoal>>, Single, Priority::LOW>().then([this] (const std::vector<VisionGoal>& goals) {

            if (!goals.empty()) {
                overview.last_seen_goal = NUClear::clock::now();
            }
        }));

        handles["overview"].push_back(on<Trigger<GameState>, Single, Priority::LOW>().then([this] (const GameState& gameState) {

            overview.game_mode      = gameState.data.mode;
            overview.game_phase     = gameState.data.phase;
            overview.penalty_reason = gameState.data.self.penalty_reason;
        }));

        handles["overview"].push_back(on<Trigger<WalkPath>, Single, Priority::LOW>().then([this] (const WalkPath& walkPath) {

            overview.path_plan.clear();

            for (auto state : walkPath.states) {
                overview.path_plan.push_back(Eigen::Vector2d(state.x(), state.y()));
            }
        }));

        handles["overview"].push_back(on<Trigger<WalkCommand>, Single, Priority::LOW>().then([this] (const WalkCommand& walkCommand) {

            overview.walk_command = walkCommand.command;
        }));

    }
}
}
