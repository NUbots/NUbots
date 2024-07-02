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

    Soccer::Soccer(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration, Trigger<GlobalConfig>>("Soccer.yaml")
            .then([this](const Configuration& config, const GlobalConfig& global_config) {
                // Use configuration here from file Soccer.yaml
                this->log_level   = config["log_level"].as<NUClear::LogLevel>();
                cfg.force_playing = config["force_playing"].as<bool>();

                // Get the soccer position, if not valid option then default to striker
                cfg.position = Position(config["position"].as<std::string>());

                // Get the number of seconds until we assume a teammate is inactive
                cfg.timeout = config["timeout"].as<uint>();

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
            // This emit starts the tree to play soccer
            emit<Task>(std::make_unique<FindPurpose>(), 1);
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
                    follow_directions();
                    break;
                default: log<NUClear::ERROR>("Invalid robot position");
            }
        });

        on<Trigger<Penalisation>>().then([this](const Penalisation& self_penalisation) {
            // Need to remove penalised robot from active_robots
            for (auto it = active_robots.begin(); it != active_robots.end(); ++it) {
                if (it->robot_id == self_penalisation.robot_id) {
                    active_robots.erase(it);
                    break;
                }
            }

            // If the robot is penalised, its purpose doesn't matter anymore, it must stand still
            if (!cfg.force_playing && self_penalisation.context == GameEvents::Context::SELF) {
                emit(std::make_unique<ResetWebotsServos>());
                emit(std::make_unique<Stability>(Stability::UNKNOWN));
                emit(std::make_unique<ResetFieldLocalisation>());
                emit<Task>(std::unique_ptr<FindPurpose>(nullptr));
            } else {
                give_directions();
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

        // Check if we have heard from robots periodically, and remove them if not
        on<Every<2, Per<std::chrono::seconds>>>().then([this] {
            auto now     = NUClear::clock::now();
            auto timeout = std::chrono::seconds(int(cfg.timeout));

            // Remove inactive robots
            for (auto it = active_robots.begin(); it != active_robots.end();) {
                if (it->robot_id != player_id && now - it->last_update_time > timeout) {
                    log<NUClear::DEBUG>("remove player ", int(it->robot_id));
                    it = active_robots.erase(it);
                    give_directions();
                } else {
                    ++it;
                }
            }

            log<NUClear::DEBUG>("active robots length ", int(active_robots.size()));
        });

        // Emit our own Purpose for NUsight debugging
        on<Every<2, Per<std::chrono::seconds>>, With<Purposes>>().then([this](const Purposes& purposes) {
            log<NUClear::DEBUG>("Current soccer position ", soccer_position);
            auto purposes_msg = std::make_unique<Purposes>(purposes);
            purposes_msg->purpose.purpose = soccer_position.to_soccer_position();
            emit(std::move(purposes_msg));
        });

        // Capture leader's last message in case of bad timing
        on<Trigger<RoboCup>>().then([this](const RoboCup& robocup) {
            manage_active_robots(robocup);

            // A robot may emit leader messages mistakenly. Only listen to real leader
            bool other_is_leader = !active_robots.empty() && active_robots.front().robot_id == robocup.current_pose.player_id;

            if (other_is_leader && !robocup.purpose_commands.purpose_commands.empty()) {
                leader_message = std::make_unique<RoboCup>(robocup);
            }
        });
    }

    void Soccer::find_purpose() {
        emit<Task>(std::make_unique<FindPurpose>(), 1);

        if (cfg.position == Position::DYNAMIC) {
            // Trigger FindPurpose by emitting Robocup message
            auto msg = std::make_unique<RoboCup>();
            // We need this info to perform checks
            msg->current_pose.player_id = player_id;
            // Reset the startup time, to redetermine leader
            auto now                   = NUClear::clock::now();
            msg->purpose_commands.startup_time = now;
            startup_time               = now;
            emit(msg);
        }
    }

    void Soccer::add_robot(RobotInfo new_robot) {
        // Find the correct insertion point by robot startup time
        auto insertPos = std::lower_bound(active_robots.begin(), active_robots.end(), new_robot);

        // Insert the new robot at the found position to maintain order
        active_robots.insert(insertPos, new_robot);
    }

    void Soccer::manage_active_robots(const RoboCup& robocup) {
        //TODO: segafault around here somewhere on startup I think
        // Do not manage penalised robots, or robots that are not dynamic
        if (robocup.state == State::PENALISED || (robocup.current_pose.player_id != player_id && robocup.purpose_commands.purpose_commands.empty())) {
            return;
        }

        uint8_t incoming_robot_id = robocup.current_pose.player_id;
        auto now = NUClear::clock::now();

        std::vector<RobotInfo>::iterator it =
            std::find_if(active_robots.begin(), active_robots.end(), [incoming_robot_id](const RobotInfo& info) {
                return info.robot_id == incoming_robot_id;
            });

        if (it != active_robots.end()) {
            // If the robot is in active_robots, update timestamp
            it->last_update_time = now;
            // Add more data here
        } else {
            // Add new robot's info, and re-assign positions
            RobotInfo new_robot{incoming_robot_id, robocup.purpose_commands.startup_time, now};
            add_robot(new_robot);
            give_directions();
        }
    };

    // Find which robot should be our striker
    uint8_t Soccer::find_striker() {
        // Otherwise make first robot striker
        return 0;

        // TODO: ideas- furthest ahead on field could be striker, or closest to ball
    }

    // Give directions if I am the leader
    void Soccer::give_directions() {
        bool self_is_leader = !active_robots.empty() && active_robots.front().robot_id == player_id;
        if (cfg.position == Position::DYNAMIC && self_is_leader) {
            if (self_is_leader) {
                auto purposes_msg   = std::make_unique<Purposes>();
                uint8_t striker_idx = find_striker();

                // Leader assigns own position
                emit<Task>(std::make_unique<Striker>(cfg.force_playing));
                soccer_position = Position("STRIKER");
                log<NUClear::INFO>("Leader made striker");

                // Decide soccer positions
                for (int i = 0; i < int(active_robots.size()); ++i) {
                    if (i == striker_idx) {
                        active_robots[i].position = Position("STRIKER");
                        purposes_msg->purpose_commands.push_back({active_robots[i].robot_id, SoccerPosition::STRIKER});
                    } else {
                        active_robots[i].position = Position("DEFENDER");
                        purposes_msg->purpose_commands.push_back({active_robots[i].robot_id, SoccerPosition::DEFENDER});
                    }
                }

                // Emit the startup time of the module to claim leadership
                purposes_msg->startup_time = startup_time;

                // Emit info for NUsight
                purposes_msg->purpose.purpose = soccer_position.to_soccer_position();
                purposes_msg->purpose.player_id = player_id;

                emit(purposes_msg);
            }
        }
    };

    // Listen to directions if they are the leader
    void Soccer::follow_directions() {
        if (active_robots.empty() || active_robots.front().robot_id == player_id) { return; }
        uint8_t striker_id   = 0;

        // Find the striker by id
        for (const auto& purpose : leader_message->purpose_commands.purpose_commands) {
            if (purpose.purpose == SoccerPosition::STRIKER) {
                striker_id = purpose.player_id;
                break;
            }
        }

        // Start our soccer position's task
        if (striker_id == player_id) {
            if (soccer_position != Position::STRIKER) {
                emit<Task>(std::make_unique<Striker>(cfg.force_playing));
                soccer_position = Position("STRIKER");
                log<NUClear::INFO>("Robot made striker");
            }
        } else {
            if (soccer_position != Position::DEFENDER) {
                emit<Task>(std::make_unique<Defender>(cfg.force_playing));
                soccer_position = Position("DEFENDER");
                log<NUClear::INFO>("Robot made defender");
            }
        }
    }


}  // namespace module::purpose
