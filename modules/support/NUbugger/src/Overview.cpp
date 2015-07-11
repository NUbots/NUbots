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

#include "messages/behaviour/WalkPath.h"
#include "messages/behaviour/KickPlan.h"
#include "messages/input/gameevents/GameEvents.h"
#include "messages/input/Image.h"
#include "messages/support/nubugger/proto/Message.pb.h"
#include "messages/vision/VisionObjects.h"

#include "utility/time/time.h"
#include "utility/localisation/transform.h"
#include "utility/support/proto_armadillo.h"

/**
 * @author Monica Olejniczak
 */
namespace modules {
namespace support {

    using messages::behaviour::proto::Behaviour;
    using messages::behaviour::WalkPath;
    using messages::behaviour::KickPlan;
    using messages::input::gameevents::GameState;
    using messages::input::Image;
    using messages::input::proto::Sensors;
    using messages::localisation::Self;
    using messages::support::nubugger::proto::Message;
    using LocalisationBall = messages::localisation::Ball;
    using VisionGoal = messages::vision::Goal;
    using VisionBall = messages::vision::Ball;

    using utility::time::getUtcTimestamp;
    using utility::localisation::transform::RobotToWorldTransform;

    /**
     * @brief Provides triggers to send overview information over the network using the overview
     * instance variable.
     */
    void NUbugger::provideOverview() {

        handles[Message::OVERVIEW].push_back(on<Trigger<Every<1, std::chrono::seconds>>, Options<Single, Priority<NUClear::LOW>>>([this](const time_t&) {
            Message message;
            message.set_type(Message::OVERVIEW);
            message.set_filter_id(0);
            message.set_utc_timestamp(getUtcTimestamp());

            *message.mutable_overview() = overview;

            send(message);
        }));

        handles[Message::OVERVIEW].push_back(on<Trigger<CommandLineArguments>, Options<Single, Priority<NUClear::LOW>>>([this](const std::vector<std::string>& arguments) {

            std::string role_name = arguments.at(0);
            auto index = role_name.rfind('/');
            if (index != std::string::npos) {
                role_name = role_name.substr(index + 1);
            }
            overview.set_role_name(role_name);

        }));

        handles[Message::OVERVIEW].push_back(on<Trigger<Behaviour::State>, Options<Single, Priority<NUClear::LOW>>>([this](const Behaviour::State& state) {

            overview.set_behaviour_state(state);

        }));

        handles[Message::OVERVIEW].push_back(on<Trigger<Sensors>, Options<Single, Priority<NUClear::LOW>>>([this](const Sensors& sensors) {

            overview.set_voltage(sensors.voltage());
            overview.set_battery(sensors.battery());

        }));

        handles[Message::OVERVIEW].push_back(on<Trigger<std::vector<Self>>, Options<Single, Priority<NUClear::LOW>>>([this](const std::vector<Self>& selfs) {

            // Retrieve the first self in the vector.
            Self self = selfs.front();

            // Set robot position.
            *overview.mutable_robot_position() << self.position;

            // Set robot position covariance.
            *overview.mutable_robot_position_covariance() << self.position_cov;

            // Set robot heading.
            *overview.mutable_robot_heading() << self.heading;
        }));

        handles[Message::OVERVIEW].push_back(on<Trigger<std::vector<LocalisationBall>>, With<std::vector<Self>>, Options<Single,
            Priority<NUClear::LOW>>>([this](const std::vector<LocalisationBall>& balls, const std::vector<Self>& selfs) {

            // Retrieve the first ball and self in the vector.
            LocalisationBall ball = balls.front();
            Self self = selfs.front();

            // Set local ball position.
            *overview.mutable_ball_position() << ball.position;

            // Set world ball position.
            *overview.mutable_ball_world_position() << RobotToWorldTransform(self.position, self.heading, ball.position);
        }));

        handles[Message::OVERVIEW].push_back(on<Trigger<Image>, Options<Single, Priority<NUClear::LOW>>>([this](const Image&/* image*/) {

            overview.set_last_camera_image(getUtcTimestamp());

        }));

        handles[Message::OVERVIEW].push_back(on<Trigger<std::vector<VisionBall>>, Options<Single, Priority<NUClear::LOW>>>([this] (const std::vector<VisionBall>&/* balls*/) {

            overview.set_last_seen_ball(getUtcTimestamp());

        }));

        handles[Message::OVERVIEW].push_back(on<Trigger<std::vector<VisionGoal>>, Options<Single, Priority<NUClear::LOW>>>([this] (const std::vector<VisionGoal>&/* goals*/) {

            overview.set_last_seen_goal(getUtcTimestamp());

        }));

        handles[Message::OVERVIEW].push_back(on<Trigger<GameState>, Options<Single, Priority<NUClear::LOW>>>([this] (const GameState& gamestate) {

            overview.set_game_mode(getMode(gamestate.mode));
            overview.set_game_phase(getPhase(gamestate.phase));
            overview.set_penalty_reason(getPenaltyReason(gamestate.self.penaltyReason));

        }));

        handles[Message::OVERVIEW].push_back(on<Trigger<WalkPath>, Options<Single, Priority<NUClear::LOW>>>([this] (const WalkPath& walkPath) {

            overview.clear_path_plan();

            for (auto state : walkPath.states) {
                *overview.add_path_plan() << arma::vec2(state.xy());
            }

        }));

        handles[Message::OVERVIEW].push_back(on<Trigger<KickPlan>, Options<Single, Priority<NUClear::LOW>>>([this] (const KickPlan& KickPlan) {

            *overview.mutable_kick_target() << KickPlan.target;

        }));

    }
}
}
