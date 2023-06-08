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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef MODULES_BEHAVIOUR_STRATEGY_SOCCERSTRATEGGY_HPP
#define MODULES_BEHAVIOUR_STRATEGY_SOCCERSTRATEGGY_HPP

#include <Eigen/Core>
#include <nuclear>

#include "message/behaviour/Behaviour.hpp"
#include "message/behaviour/KickPlan.hpp"
#include "message/input/GameEvents.hpp"
#include "message/input/GameState.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/localisation/FilteredBall.hpp"
#include "message/support/FieldDescription.hpp"

namespace module::behaviour::strategy {

    using FilteredBall = message::localisation::FilteredBall;

    class SoccerStrategy : public NUClear::Reactor {
    private:
        struct Config {
            Config() = default;
            NUClear::clock::duration ball_last_seen_max_time{};
            NUClear::clock::duration goal_last_seen_max_time{};
            double ball_search_walk_start_speed = 0.0;
            double ball_search_walk_stop_speed  = 0.0;
            double ball_search_walk_slow_time   = 0.0;
            Eigen::Vector2d start_position_offensive{Eigen::Vector2d::Zero()};
            Eigen::Vector2d start_position_defensive{Eigen::Vector2d::Zero()};
            bool is_goalie                          = false;
            double goalie_max_ball_distance         = 0.0;
            double goalie_command_timeout           = 0.0;
            double goalie_rotation_speed_factor     = 0.0;
            double goalie_max_rotation_speed        = 0.0;
            double goalie_translation_speed_factor  = 0.0;
            double goalie_max_translation_speed     = 0.0;
            double goalie_side_walk_angle_threshold = 0.0;
            NUClear::clock::duration localisation_interval{};
            NUClear::clock::duration localisation_duration{};
            bool force_playing                = false;
            bool force_penalty_shootout       = false;
            int walk_to_ready_time            = 0;
            double kicking_distance_threshold = 0.0;
            double kicking_angle_threshold    = 0.0;
        } cfg;

        /// @brief Flag to determine whether the ball is on the left or right side of the robot
        double ball_lost_clockwise = true;

        /// @brief Bool to indicate  if the robot is currently getting up
        bool is_getting_up = false;

        /// @brief Bool to indicate if we are currently penalised
        bool self_penalised = false;

        /// @brief Bool to indicate if we want to reset localisation in initial phase
        bool reset_in_initial = true;

        /// @brief Used to indicate which team is kicking off
        message::input::GameEvents::Context team_kicking_off = message::input::GameEvents::Context::UNKNOWN;

        message::behaviour::KickPlan::KickType kick_type{};

        /// @brief Stores which behaviour state the robot is in
        message::behaviour::Behaviour::State current_state = message::behaviour::Behaviour::State::INIT;

        /// @brief Stores which behaviour state the robot was previosuly in
        message::behaviour::Behaviour::State previous_state = message::behaviour::Behaviour::State::INIT;

        /// @brief Stores the time stamp of when the robot starts walking to the ready position
        NUClear::clock::time_point started_walking_to_ready_at;

        /// @brief Bool to indicate that the robot is currently walking to the ready position
        bool started_walking_to_ready = false;

        /// @brief Bool to indicate that the half has reset
        bool is_reset_half = false;

        /// @brief The time since the last ball was measured
        // TODO(BehaviourTeam): Currently the time is initialised long enough in the past to ensure that the robot
        // starts in a state where it hasn't seen ball
        NUClear::clock::time_point ball_last_measured = NUClear::clock::now() - std::chrono::seconds(6000);

        /// @brief The time since the last goal was measured
        NUClear::clock::time_point goal_last_measured;

        /// @brief Resets ball localisation for use in initial phase of normal mode
        void initial_localisation_reset();

        /// @brief Resets robot and ball localisation for use in initial phase of penalty mode
        // TODO(BehaviourTeam): This method needs a rewrite
        void penalty_shootout_localisation_reset();

        /// @brief Resets ball localisation for use after we are unpenalised
        void unpenalised_localisation_reset();

        /// @brief Makes the robot stand still
        void stand_still();

        /// @brief Playing behaviour when ball is visible, currently just walks to the ball
        void play(const std::shared_ptr<const FilteredBall>& ball);

        /// @brief Playing behaviour when ball is lost, currently just rotate on spot
        void find(const std::shared_ptr<const FilteredBall>& ball);

        /// @brief Determines if robot is currently picked up based on foot down sensors
        bool picked_up(const message::input::Sensors& sensors) const;

        /// @brief Determines if robot is currently penalised
        bool penalised() const;

        /// @brief Penalty mode state machine, used to decide what phase behaviour to use.
        void penalty_shootout(const message::input::GameState::Data::Phase& phase,
                              const std::shared_ptr<const FilteredBall>& ball);

        /// @brief Normal mode state machine, used to decide what phase behaviour to use.
        void normal(const message::input::GameState& game_state,
                    const message::input::GameState::Data::Phase& phase,
                    const std::shared_ptr<const FilteredBall>& ball);

        /// @brief Penalty mode, initial phase behaviour/strategy
        void penalty_shootout_initial();

        /// @brief Penalty mode, ready phase behaviour/strategy
        void penalty_shootout_ready();

        /// @brief Penalty mode, set phase behaviour/strategy
        void penalty_shootout_set();

        /// @brief Penalty mode, playing phase behaviour/strategy
        void penalty_shootout_playing(const std::shared_ptr<const FilteredBall>& ball);

        /// @brief Penalty mode, timeout phase behaviour/strategy
        void penalty_shootout_timeout();

        /// @brief Penalty mode, finished phase behaviour/strategy
        void penalty_shootout_finished();

        /// @brief Normal mode, initial phase behaviour/strategy
        void normal_initial();

        /// @brief Normal mode, ready phase behaviour/strategy
        void normal_ready(const message::input::GameState& game_state);

        /// @brief Normal mode, set phase behaviour/strategy
        void normal_set();

        /// @brief Normal mode, playing phase behaviour/strategy
        void normal_playing(const std::shared_ptr<const FilteredBall>& ball);

        /// @brief Normal mode, finished phase behaviour/strategy
        void normal_finished();

        /// @brief Normal mode, time phase behaviour/strategy
        void normal_timeout();

    public:
        explicit SoccerStrategy(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace module::behaviour::strategy

#endif  // MODULES_BEHAVIOUR_STRATEGY_SOCCERSTRATEGGY_HPP
