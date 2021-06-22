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

            Eigen::Vector2d start_position_offensive = Eigen::Vector2d::Zero();
            Eigen::Vector2d start_position_defensive = Eigen::Vector2d::Zero();
            bool is_goalie                           = false;

            float goalie_command_timeout           = 0.0f;
            float goalie_rotation_speed_factor     = 0.0f;
            float goalie_max_rotation_speed        = 0.0f;
            float goalie_translation_speed_factor  = 0.0f;
            float goalie_max_translation_speed     = 0.0f;
            float goalie_side_walk_angle_threshold = 0.0f;
            NUClear::clock::duration localisation_interval{};
            NUClear::clock::duration localisation_duration{};
            bool alwaysPowerKick      = false;
            bool forcePlaying         = false;
            bool forcePenaltyShootout = false;
        } cfg_{};

        enum class FieldTarget { BALL, GOAL };

        FieldTarget walkTarget = FieldTarget::BALL;

        // TODO: remove horrible
        bool isGettingUp                                  = false;
        bool selfPenalised                                = false;
        bool manualOrientationReset                       = false;
        double manualOrientation                          = 0.0;
        message::behaviour::KickPlan::KickType kickType   = message::behaviour::KickPlan::KickType::SCRIPTED;
        message::behaviour::Behaviour::State currentState = message::behaviour::Behaviour::State::INIT;

        NUClear::clock::time_point lastLocalised = NUClear::clock::now();

        NUClear::clock::time_point ballLastMeasured =
            NUClear::clock::now() - std::chrono::seconds(600);  // TODO: unhack
        NUClear::clock::time_point ballSearchStartTime = NUClear::clock::now();
        NUClear::clock::time_point goalLastMeasured    = NUClear::clock::now();
        void initialLocalisationReset(const message::support::FieldDescription& fieldDescription) noexcept;
        void penaltyShootoutLocalisationReset() noexcept;
        void unpenalisedLocalisationReset(const message::support::FieldDescription& fieldDescription) noexcept;

        void standStill() noexcept;
        void walkTo(const message::support::FieldDescription& fieldDescription, const FieldTarget& object);
        void walkTo(const message::support::FieldDescription& fieldDescription, const Eigen::Vector2d& position);
        void find(const FieldTarget& objects) noexcept;
        [[nodiscard]] bool pickedUp(const message::input::Sensors& sensors) noexcept;
        [[nodiscard]] bool penalised() noexcept;
        [[nodiscard]] static double ballDistance(const message::localisation::Ball& ball) noexcept;
        void goalieWalk(const message::localisation::Field& field, const message::localisation::Ball& ball) noexcept;
        [[nodiscard]] static Eigen::Vector2d getKickPlan(
            const message::localisation::Field& field,
            const message::support::FieldDescription& fieldDescription) noexcept;
        void play(const message::localisation::Field& field,
                  const message::localisation::Ball& ball,
                  const message::support::FieldDescription& fieldDescription,
                  const message::input::GameState::Data::Mode& mode) noexcept;

    public:
        explicit SoccerStrategy(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace module::behaviour::strategy

#endif  // MODULES_BEHAVIOUR_STRATEGY_SOCCERSTRATEGGY_HPP
