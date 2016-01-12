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

#include "message/behaviour/WalkPath.h"
#include "message/behaviour/KickPlan.h"
#include "message/input/gameevents/GameEvents.h"
#include "message/input/Image.h"
#include "message/vision/VisionObjects.h"
#include "message/motion/WalkCommand.h"

#include "utility/time/time.h"
#include "utility/localisation/transform.h"
#include "utility/support/proto_armadillo.h"

/**
 * @author Monica Olejniczak
 */
namespace module {
namespace support {

    using NUClear::message::CommandLineArguments;
    using message::behaviour::proto::Behaviour;
    using message::behaviour::WalkPath;
    using message::behaviour::KickPlan;
    using message::input::gameevents::GameState;
    using message::input::Image;
    using message::input::Sensors;
    using message::localisation::Self;
    using LocalisationBall = message::localisation::Ball;
    using VisionGoal = message::vision::Goal;
    using VisionBall = message::vision::Ball;
    using message::motion::WalkCommand;

    using utility::time::getUtcTimestamp;
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
            overview.set_role_name(role_name);

        }));

        handles["overview"].push_back(on<Trigger<Behaviour::State>, Single, Priority::LOW>().then([this] (const Behaviour::State& state) {

            overview.set_behaviour_state(state);

        }));

        handles["overview"].push_back(on<Trigger<KickPlan>, Single, Priority::LOW>().then([this] (const KickPlan& kickPlan) {

            // TODO fix runtime error:
            // *overview.mutable_kick_target() << kickTarget;

        }));

        handles["overview"].push_back(on<Trigger<Sensors>, Single, Priority::LOW>().then([this] (const Sensors& sensors) {

            overview.set_voltage(sensors.voltage);
            overview.set_battery(sensors.battery);

        }));

        handles["overview"].push_back(on<Trigger<std::vector<Self>>, Single, Priority::LOW>().then([this](const std::vector<Self>& selfs) {

            // Retrieve the first self in the vector.
            Self self = selfs.front();

            // Set robot position.
            *overview.mutable_robot_position() << self.position;

            // Set robot position covariance.
            *overview.mutable_robot_position_covariance() << self.position_cov;

            // Set robot heading.
            *overview.mutable_robot_heading() << self.heading;
        }));

        handles["overview"].push_back(on<Trigger<std::vector<LocalisationBall>>, With<std::vector<Self>>, Single, Priority::LOW>()
            .then([this](const std::vector<LocalisationBall>& balls, const std::vector<Self>& selfs) {

            // Retrieve the first ball and self in the vector.
            LocalisationBall ball = balls.front();
            Self self = selfs.front();

            // Set local ball position.
            *overview.mutable_ball_position() << ball.position;

            // Set world ball position.
            *overview.mutable_ball_world_position() << RobotToWorldTransform(self.position, self.heading, ball.position);
        }));

        handles["overview"].push_back(on<Trigger<Image>, Single, Priority::LOW>().then([this] {

            overview.set_last_camera_image(getUtcTimestamp());

        }));

        handles["overview"].push_back(on<Trigger<std::vector<VisionBall>>, Single, Priority::LOW>().then([this] (const std::vector<VisionBall>& balls) {

            if (!balls.empty()) {
                overview.set_last_seen_ball(getUtcTimestamp());
            }

        }));

        handles["overview"].push_back(on<Trigger<std::vector<VisionGoal>>, Single, Priority::LOW>().then([this] (const std::vector<VisionGoal>& goals) {

            if (!goals.empty()) {
                overview.set_last_seen_goal(getUtcTimestamp());
            }

        }));

        handles["overview"].push_back(on<Trigger<GameState>, Single, Priority::LOW>().then([this] (const GameState& gameState) {

            overview.set_game_mode(getMode(gameState.mode));
            overview.set_game_phase(getPhase(gameState.phase));
            overview.set_penalty_reason(getPenaltyReason(gameState.self.penaltyReason));

        }));

        handles["overview"].push_back(on<Trigger<WalkPath>, Single, Priority::LOW>().then([this] (const WalkPath& walkPath) {

            overview.clear_path_plan();

            for (auto state : walkPath.states) {
                *overview.add_path_plan() << arma::vec2(state.xy());
            }

        }));

        handles["overview"].push_back(on<Trigger<WalkCommand>, Single, Priority::LOW>().then([this] (const WalkCommand& walkCommand) {

            *overview.mutable_walk_command() << walkCommand.command;

        }));

    }
}
}
