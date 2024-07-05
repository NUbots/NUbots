/*
 * MIT License
 *
 * Copyright (c) 2023 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "Soccer.hpp"

#include <string>
#include <vector>

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/behaviour/state/Stability.hpp"
#include "message/behaviour/state/WalkState.hpp"
#include "message/input/GameEvents.hpp"
#include "message/input/Purposes.hpp"
#include "message/input/RoboCup.hpp"
#include "message/localisation/Field.hpp"
#include "message/platform/RawSensors.hpp"
#include "message/purpose/Defender.hpp"
#include "message/purpose/FindPurpose.hpp"
#include "message/purpose/Goalie.hpp"
#include "message/purpose/Striker.hpp"
#include "message/skill/Look.hpp"
#include "message/strategy/FallRecovery.hpp"
#include "message/strategy/StandStill.hpp"
#include "message/strategy/StartSafely.hpp"
#include "message/support/GlobalConfig.hpp"
#include "message/support/nusight/Purposes.hpp"

namespace module::purpose {

    using extension::Configuration;
    using Penalisation   = message::input::GameEvents::Penalisation;
    using Unpenalisation = message::input::GameEvents::Unpenalisation;
    using message::behaviour::state::Stability;
    using message::behaviour::state::WalkState;
    using message::input::GameEvents;
    using message::input::Purposes;
    using message::input::RoboCup;
    using message::input::SoccerPosition;
    using message::input::State;
    using message::localisation::ResetFieldLocalisation;
    using message::platform::ButtonMiddleDown;
    using message::platform::ResetWebotsServos;
    using message::purpose::Defender;
    using message::purpose::FindPurpose;
    using message::purpose::Goalie;
    using message::purpose::Striker;
    using message::skill::Look;
    using message::strategy::FallRecovery;
    using message::strategy::StandStill;
    using message::strategy::StartSafely;
    using message::support::GlobalConfig;
    using NusightPurposes = message::support::nusight::Purposes;

    Soccer::Soccer(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration, Trigger<GlobalConfig>>("Soccer.yaml")
            .then([this](const Configuration& config, const GlobalConfig& global_config) {
                // Use configuration here from file Soccer.yaml
                this->log_level   = config["log_level"].as<NUClear::LogLevel>();
                cfg.force_playing = config["force_playing"].as<bool>();

                // Get the soccer position, if not valid option then default to striker
                cfg.position = Position(config["position"].as<std::string>());

                // Get the number of seconds until we assume a teammate is inactive
                cfg.timeout = config["timeout"].as<int>();

                cfg.max_robots = config["max_robots"].as<uint>();
                // Resize the vector based on cfg.max_robots
                robots.resize(cfg.max_robots);

                // Decide which player we are
                player_id = global_config.player_id;
                log<NUClear::DEBUG>("Configure Player ID ", int(player_id));
            });

        // Start the Director graph for the soccer scenario!
        on<Startup>().then([this] {
            // At the start of the program, we should be standing
            // Without these emits, modules that need a Stability and WalkState messages may not run
            emit(std::make_unique<Stability>(Stability::UNKNOWN));
            emit(std::make_unique<WalkState>(WalkState::State::STOPPED));
            // Idle stand if not doing anything
            emit<Task>(std::make_unique<StandStill>());
            // Idle look forward if the head isn't doing anything else
            emit<Task>(std::make_unique<Look>(Eigen::Vector3d::UnitX(), true));
            // Create robocup message with necessary info
            find_purpose();
            // When starting, we want to safely move to the stand position
            emit<Task>(std::make_unique<StartSafely>(), 2);
            // The robot should always try to recover from falling, if applicable, regardless of purpose
            emit<Task>(std::make_unique<FallRecovery>(), 3);
        });

        on<Provide<FindPurpose>, Every<BEHAVIOUR_UPDATE_RATE, Per<std::chrono::seconds>>>().then([this]() {
            // Make task based on configured purpose/soccer position
            switch (cfg.position) {
                case Position::STRIKER:
                    emit<Task>(std::make_unique<Striker>(cfg.force_playing));
                    soccer_position = Position("STRIKER");
                    break;
                case Position::GOALIE:
                    emit<Task>(std::make_unique<Goalie>(cfg.force_playing));
                    soccer_position = Position("GOALIE");
                    break;
                case Position::DEFENDER:
                    emit<Task>(std::make_unique<Defender>(cfg.force_playing));
                    soccer_position = Position("DEFENDER");
                    break;
                case Position::DYNAMIC:
                    give_directions();
                    if (soccer_position == Position::STRIKER) {
                        emit<Task>(std::make_unique<Striker>(cfg.force_playing));
                    }
                    else if (soccer_position == Position::DEFENDER) {
                        emit<Task>(std::make_unique<Defender>(cfg.force_playing));
                    }
                    else {
                        log<NUClear::ERROR>("Invalid robot position");
                    }
                    break;
                default: log<NUClear::ERROR>("Invalid robot position");
            }
        });

        on<Trigger<Penalisation>>().then([this](const Penalisation& self_penalisation) {
            // Set penalised robot to inactive
            robots[self_penalisation.robot_id - 1].is_active = false;

            // If the robot is penalised, its purpose doesn't matter anymore, it must stand still
            if (!cfg.force_playing && self_penalisation.context == GameEvents::Context::SELF) {
                robots[player_id - 1].is_active = false;
                emit(std::make_unique<ResetWebotsServos>());
                emit(std::make_unique<Stability>(Stability::UNKNOWN));
                emit(std::make_unique<ResetFieldLocalisation>());
                emit<Task>(std::unique_ptr<FindPurpose>(nullptr));
            }
        });

        on<Trigger<Unpenalisation>>().then([this](const Unpenalisation& self_unpenalisation) {
            // If the robot is unpenalised, stop standing still and find its purpose
            if (!cfg.force_playing && self_unpenalisation.context == GameEvents::Context::SELF) {
                find_purpose();
            }
        });

        on<Trigger<ButtonMiddleDown>, Single>().then([this] {
            // Middle button forces playing
            log<NUClear::INFO>("Middle button pressed!");
            if (!cfg.force_playing) {
                log<NUClear::INFO>("Force playing started.");
                cfg.force_playing = true;
                find_purpose();
            }
        });

        // Heartbeat
        on<Every<2, Per<std::chrono::seconds>>>().then([this] {
            // Check if we have heard from robots periodically, and update their status if not
            auto now = NUClear::clock::now();
            for (std::size_t i = 0; i < robots.size(); ++i) {
                if ((i + 1) != player_id && now - robots[i].last_update_time > cfg.timeout) {
                    log<NUClear::DEBUG>("Update player to inactive: ", int(i + 1));
                    robots[i].is_active = false;
                }
            }

            if (is_active) {
                log<NUClear::DEBUG>("Is active true");
            }
            if (!is_active) {
                log<NUClear::DEBUG>("Is active false");
            }
        });

        // Emit our own Purpose for NUsight debugging
        on<Every<2, Per<std::chrono::seconds>>>().then([this]() {
            log<NUClear::DEBUG>("Current soccer position ", soccer_position);
            auto purposes_msg             = std::make_unique<NusightPurposes>();
            purposes_msg->purpose.purpose = soccer_position.to_soccer_position();
            purposes_msg->startup_time    = robots[player_id - 1].startup_time;
            emit(std::move(purposes_msg));
        });

        on<Trigger<RoboCup>>().then([this](const RoboCup& robocup) {
            // Save info from incoming robocup message
            uint8_t incoming_robot_id                      = robocup.current_pose.player_id;
            robots[incoming_robot_id - 1].last_update_time = now;
            robots[incoming_robot_id - 1].is_active        = robocup.purpose_commands.is_active;
            robots[incoming_robot_id - 1].startup_time     = robocup.purpose_commands.startup_time;

            // If this robot we just heard from is the leader, listen to them
            bool is_leader = find_leader() == robocup.current_pose.player_id;
            if (is_leader) {
                follow_directions(robocup);
            }

            // All robots can give directions to speed up switch over time
            give_directions();
        });
    }

    void Soccer::find_purpose() {
        robots[player_id - 1].is_active = true;

        if (cfg.position == Position::DYNAMIC) {
            robots[player_id - 1].startup_time = NUClear::clock::now();
        }

        emit<Task>(std::make_unique<FindPurpose>(), 1);
    }

    // Find which robot should be our leader
    uint8_t Soccer::find_leader() {
        uint8_t leader_idx = 0;

        for (uint8_t i = 0; i < robots.size(); ++i) {
            if (robots[i].is_active) {
                // If the current prospective leader is not active, take the first active robot
                if (!robots[leader_idx].is_active) {
                    log<NUClear::WARN>("Leader is now inactive", int(leader_idx + 1));
                    leader_idx = i;
                }
                // The leader should be the robot alive the longest
                else if (robots[i] < robots[leader_idx]) {
                    log<NUClear::WARN>("Leader", int(leader_idx + 1), "is not as cool as us", int(i + 1));
                    leader_idx = i;
                }
            }
        }

        log<NUClear::DEBUG>("Leader is ", int(leader_idx + 1));
        return leader_idx + 1;
    }

    // Find which robot should be our striker
    uint8_t Soccer::find_striker() {
        uint8_t leader_idx = find_leader();
        // Always make oldest robot striker for now
        return leader_idx;

        // TODO: ideas- furthest ahead on field could be striker, or closest to ball
    }

    // Give directions if I am dynamic
    void Soccer::give_directions() {
        bool self_is_leader = find_leader() == player_id;
        auto purposes_msg   = std::make_unique<Purposes>();
        uint8_t striker_idx = find_striker();

        // Leader assigns own position
        if (self_is_leader) {
            soccer_position = Position("STRIKER");
            log<NUClear::INFO>("Leader made striker");
        }

        // Decide soccer positions
        for (int i = 0; i < int(robots.size()); ++i) {
            purposes_msg->purpose_commands.push_back(
                {i == striker_idx ? SoccerPosition::STRIKER : SoccerPosition::DEFENDER});
        }

        // Emit the startup time of the module to claim leadership
        purposes_msg->startup_time = robots[player_id - 1].startup_time;
        purposes_msg->is_active    = robots[player_id - 1].is_active;

        emit(purposes_msg);
    };

    // Listen to directions if they are the leader
    void Soccer::follow_directions(const RoboCup& robocup) {
        // Find the striker by id
        uint8_t striker_id = 0;
        for (const auto& purpose : robocup.purpose_commands.purpose_commands) {
            if (purpose.purpose == SoccerPosition::STRIKER) {
                striker_id = robocup.current_pose.player_id;
                break;
            }
        }

        // If we are now the striker and we don't know it, make us a striker
        if (striker_id == player_id && soccer_position != Position::STRIKER) {
            soccer_position = Position("STRIKER");
            log<NUClear::INFO>("Robot made striker");
        }
        // If we are not now the striker and we don't know it, make us a defender
        else if (striker_id != player_id && soccer_position != Position::DEFENDER) {
            soccer_position = Position("DEFENDER");
            log<NUClear::INFO>("Robot made defender");
        }
    }


}  // namespace module::purpose
