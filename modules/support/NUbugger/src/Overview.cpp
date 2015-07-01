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

#include "messages/input/gameevents/GameEvents.h"
#include "messages/input/Image.h"
#include "messages/support/nubugger/proto/Message.pb.h"
#include "messages/vision/VisionObjects.h"

#include "utility/time/time.h"

/**
 * @author Monica Olejniczak
 */
namespace modules {
namespace support {

    using messages::behaviour::proto::Behaviour;
    using messages::input::gameevents::GameState;
    using messages::input::Image;
    using messages::input::proto::Sensors;
    using messages::localisation::Self;
    using messages::support::nubugger::proto::Message;
    using LocalisationBall = messages::localisation::Ball;
    using VisionGoal = messages::vision::Goal;
    using VisionBall = messages::vision::Ball;

    using utility::time::getUtcTimestamp;

    /**
     * @brief Provides triggers to send overview information over the network using the overview
     * instance variable.
     */
    void NUbugger::provideOverview() {

        handles["overview"].push_back(on<Trigger<Every<1, std::chrono::seconds>>, Options<Single, Priority<NUClear::LOW>>>([this](const time_t&) {
            Message message;
            message.set_type(Message::OVERVIEW);
            message.set_filter_id(0);
            message.set_utc_timestamp(getUtcTimestamp());

            *message.mutable_overview() = overview;

            send(message);
        }));

        handles["overview"].push_back(on<Trigger<Behaviour::State>, Options<Single, Priority<NUClear::LOW>>>([this](const Behaviour::State& state) {

            overview.set_behaviour_state(state);

        }));

        handles["overview"].push_back(on<Trigger<Sensors>, Options<Single, Priority<NUClear::LOW>>>([this](const Sensors& sensors) {

            overview.set_voltage(sensors.voltage());
            overview.set_battery(sensors.battery());

        }));

        handles["overview"].push_back(on<Trigger<std::vector<Self>>, Options<Single, Priority<NUClear::LOW>>>([this](const std::vector<Self>& selfs) {

            // Retrieve the first self in the vector.
            Self self = selfs.front();

            // Set robot position.
            auto* robotPosition = overview.mutable_robot_position();
            arma::vec2 position = self.position;
            robotPosition->set_x(position[0]);
            robotPosition->set_y(position[1]);

            // Set robot position covariance.
            auto* robotPositionCovariance = overview.mutable_robot_position_covariance();
            arma::mat22 covariance = self.position_cov;
            auto* xAxis = robotPositionCovariance->mutable_x();
            xAxis->set_x(covariance[0]);
            xAxis->set_y(covariance[1]);
            auto* yAxis = robotPositionCovariance->mutable_y();
            yAxis->set_x(covariance[2]);
            yAxis->set_y(covariance[3]);

            // Set robot heading.
            auto* robotHeading = overview.mutable_robot_heading();
            arma::vec2 heading = self.heading;
            robotHeading->set_x(heading[0]);
            robotHeading->set_y(heading[1]);

        }));

        handles["overview"].push_back(on<Trigger<std::vector<LocalisationBall>>, Options<Single, Priority<NUClear::LOW>>>([this](const std::vector<LocalisationBall>& balls) {

            // Retrieve the first ball in the vector.
            LocalisationBall ball = balls.front();

            // Set ball position.
            auto* ballPosition = overview.mutable_ball_position();
            arma::vec2 position = ball.position;
            ballPosition->set_x(position[0]);
            ballPosition->set_y(position[1]);          

        }));

        handles["overview"].push_back(on<Trigger<Image>, Options<Single, Priority<NUClear::LOW>>>([this](const Image&/* image*/) {

            overview.set_last_camera_image(getUtcTimestamp());

        }));

        handles["overview"].push_back(on<Trigger<std::vector<VisionBall>>, Options<Single, Priority<NUClear::LOW>>>([this] (const std::vector<VisionBall>&/* balls*/) {

            overview.set_last_seen_ball(getUtcTimestamp());

        }));

        handles["overview"].push_back(on<Trigger<std::vector<VisionGoal>>, Options<Single, Priority<NUClear::LOW>>>([this] (const std::vector<VisionGoal>&/* goals*/) {

            overview.set_last_seen_goal(getUtcTimestamp());

        }));

        handles["overview"].push_back(on<Trigger<GameState>, Options<Single, Priority<NUClear::LOW>>>([this] (const GameState& gamestate) {

            overview.set_game_mode(getMode(gamestate.mode));
            overview.set_game_phase(getPhase(gamestate.phase));
            overview.set_penalty_reason(getPenaltyReason(gamestate.self.penaltyReason));

        }));

    }
}
}
