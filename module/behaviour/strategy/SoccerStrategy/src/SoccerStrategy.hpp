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
#include "message/behaviour/FieldTarget.hpp"
#include "message/behaviour/KickPlan.hpp"
#include "message/input/GameEvents.hpp"
#include "message/input/GameState.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/support/FieldDescription.hpp"

namespace module::behaviour::strategy {

    class SoccerStrategy : public NUClear::Reactor {
    private:
        struct Config {
            Config() = default;
            NUClear::clock::duration ball_last_seen_max_time{};
            NUClear::clock::duration goal_last_seen_max_time{};
            float ball_search_walk_start_speed = 0.0f;
            float ball_search_walk_stop_speed  = 0.0f;
            float ball_search_walk_slow_time   = 0.0f;
            Eigen::Vector2d start_position_offensive{Eigen::Vector2d::Zero()};
            Eigen::Vector2d start_position_defensive{Eigen::Vector2d::Zero()};
            bool is_goalie                         = false;
            float goalie_command_timeout           = 0.0f;
            float goalie_rotation_speed_factor     = 0.0f;
            float goalie_max_rotation_speed        = 0.0f;
            float goalie_translation_speed_factor  = 0.0f;
            float goalie_max_translation_speed     = 0.0f;
            float goalie_side_walk_angle_threshold = 0.0f;
            NUClear::clock::duration localisation_interval{};
            NUClear::clock::duration localisation_duration{};
            bool alwaysPowerKick        = false;
            bool force_playing          = false;
            bool force_penalty_shootout = false;
            int walk_to_ready_time      = 0;
        } cfg;

        message::behaviour::FieldTarget walkTarget{};

        std::vector<message::behaviour::FieldTarget> lookTarget{};

        bool is_getting_up            = false;
        bool has_kicked               = false;
        bool self_penalised           = false;
        bool reset_in_initial         = true;
        bool started_walking_to_ready = false;
        bool is_reset_half            = false;


        NUClear::clock::time_point started_walking_to_ready_at;
        message::input::GameEvents::Context team_kicking_off = message::input::GameEvents::Context::UNKNOWN;
        message::behaviour::KickPlan::KickType kick_type{};
        message::behaviour::Behaviour::State currentState = message::behaviour::Behaviour::State::INIT;

        NUClear::clock::time_point ball_last_measured =
            NUClear::clock::now() - std::chrono::seconds(6000);  // TODO(BehaviourTeam): unhack
        NUClear::clock::time_point goal_last_measured;
        void initial_localisation_reset();
        void penalty_shootout_localisation_reset(const message::support::FieldDescription& field_description);
        void unpenalised_localisation_reset();

        void stand_still();
        void walk_to(const message::support::FieldDescription& field_description,
                     const message::behaviour::FieldTarget::Target& target);
        void walk_to(const message::support::FieldDescription& field_description, const Eigen::Vector2d& position);
        void find(const std::vector<message::behaviour::FieldTarget>& objects);
        bool picked_up(const message::input::Sensors& sensors) const;
        bool penalised() const;
        static bool ball_distance(const message::localisation::Ball& ball);
        void goalie_walk(const message::localisation::Field& field, const message::localisation::Ball& ball);
        static Eigen::Vector2d get_kick_plan(const message::localisation::Field& field,
                                             const message::support::FieldDescription& field_description);
        void play(const message::localisation::Field& field,
                  const message::localisation::Ball& ball,
                  const message::support::FieldDescription& field_description,
                  const message::input::GameState::Data::Mode& mode);

        void penalty_shootout(const message::input::GameState::Data::Phase& phase,
                              const message::support::FieldDescription& field_description,
                              const message::localisation::Field& field,
                              const message::localisation::Ball& ball);

        void normal(const message::input::GameState& game_state,
                    const message::input::GameState::Data::Phase& phase,
                    const message::support::FieldDescription& field_description,
                    const message::localisation::Field& field,
                    const message::localisation::Ball& ball);

        // PENALTY mode functions
        void penalty_shootout_initial();
        void penalty_shootout_ready();
        void penalty_shootout_set(const message::support::FieldDescription& field_description);
        void penalty_shootout_playing(const message::localisation::Field& field,
                                      const message::localisation::Ball& ball);
        void penalty_shootout_timeout();
        void penalty_shootout_finished();

        // NORMAL mode functions
        void normal_initial();
        void normal_ready(const message::input::GameState& game_state,
                          const message::support::FieldDescription& field_description);
        void normal_set();
        void normal_playing(const message::localisation::Field& field,
                            const message::localisation::Ball& ball,
                            const message::support::FieldDescription& field_description);
        void normal_finished();
        void normal_timeout();

    public:
        explicit SoccerStrategy(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace module::behaviour::strategy

#endif  // MODULES_BEHAVIOUR_STRATEGY_SOCCERSTRATEGGY_HPP
