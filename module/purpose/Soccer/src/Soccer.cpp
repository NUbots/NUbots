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
#include "message/input/RoboCup.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/platform/RawSensors.hpp"
#include "message/purpose/Defender.hpp"
#include "message/purpose/FindPurpose.hpp"
#include "message/purpose/Goalie.hpp"
#include "message/purpose/Striker.hpp"
#include "message/strategy/FallRecovery.hpp"
#include "message/strategy/StandStill.hpp"
#include "message/support/GlobalConfig.hpp"

namespace module::purpose {

    using extension::Configuration;
    using Penalisation   = message::input::GameEvents::Penalisation;
    using Unpenalisation = message::input::GameEvents::Unpenalisation;
    using message::behaviour::state::Stability;
    using message::behaviour::state::WalkState;
    using message::input::GameEvents;
    using message::input::RoboCup;
    using message::localisation::ResetFieldLocalisation;
    using message::localisation::Field;
    using message::localisation::Ball;
    using message::platform::ButtonMiddleDown;
    using message::platform::ResetWebotsServos;
    using message::purpose::Defender;
    using message::purpose::FindPurpose;
    using message::purpose::Goalie;
    using message::purpose::Striker;
    using message::strategy::FallRecovery;
    using message::strategy::StandStill;
    using message::support::GlobalConfig;
    using message::input::State;

    Soccer::Soccer(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration, Trigger<GlobalConfig>>("Soccer.yaml")
            .then([this](const Configuration& config, const GlobalConfig& global_config) {
            // Use configuration here from file Soccer.yaml
            this->log_level   = config["log_level"].as<NUClear::LogLevel>();
            cfg.force_playing = config["force_playing"].as<bool>();

            // Get the soccer position, if not valid option then default to striker
            cfg.position = Position(config["position"].as<std::string>());

            // initialise robot storage
            active_robots = std::vector<RobotInfo>();

            // decide which player we are
            PLAYER_ID = global_config.player_id;
            log<NUClear::DEBUG>("Configure Player ID ", PLAYER_ID);
        });

        // Start the Director graph for the soccer scenario!
        on<Startup>().then([this] {
            // At the start of the program, we should be standing
            // Without these emis, modules that need a Stability and WalkState messages may not run
            emit(std::make_unique<Stability>(Stability::UNKNOWN));
            emit(std::make_unique<WalkState>(WalkState::State::STOPPED));
            // Idle stand if not doing anything
            emit<Task>(std::make_unique<StandStill>());
            // This emit starts the tree to play soccer
            emit<Task>(std::make_unique<FindPurpose>(), 1);
            // The robot should always try to recover from falling, if applicable, regardless of purpose
            emit<Task>(std::make_unique<FallRecovery>(), 2);
            // Trigger Find purpose by emitting robocup
            auto msg = std::make_unique<RoboCup>();
            msg->current_pose.player_id = PLAYER_ID;
            emit(msg);
        });

        on<Provide<FindPurpose>(), Trigger<RoboCup>>().then([this] (const RoboCup robocup) {
            // Make task based on configured purpose/soccer position
            switch (cfg.position) {
                case Position::STRIKER: emit<Task>(std::make_unique<Striker>(cfg.force_playing)); break;
                case Position::GOALIE: emit<Task>(std::make_unique<Goalie>(cfg.force_playing)); break;
                case Position::DEFENDER: emit<Task>(std::make_unique<Defender>(cfg.force_playing)); break;
                case Position::DYNAMIC:
                    // default to defender?
                    // if (cfg.position == Position::DYNAMIC && robocup.state == State::UNPENALISED) {
                    manage_active_robots(robocup.current_pose.player_id);
                    find_soccer_position(robocup);
                    break;
                default: log<NUClear::ERROR>("Invalid robot position");
            }
        });

        on<Provide<FindPurpose>, Every<2, Per<std::chrono::seconds>>, With<Ball>, With<Field>>().then([this](const Ball& ball, const Field& field) {
            log<NUClear::DEBUG>("every 3 seconds");
            if (active_robots.size() == 1) {
                // Get the current position of the ball on the field
                Eigen::Isometry3d Hfw = field.Hfw;
                Eigen::Vector3d rBFf  = Hfw * ball.rBWw;

                log<NUClear::DEBUG>("rBFf", rBFf.x());
                // leeway to prevent kickoff confusion
                // TODO: store this value, test variation instead?
                if (rBFf.x() >= 0.2) {
                    // Ball is own half, so become Defender.
                    if (active_robots.front().position.value != Position::DEFENDER) {
                        emit<Task>(std::make_unique<Defender>(cfg.force_playing));
                        active_robots.front().position = Position("DEFENDER");
                        log<NUClear::DEBUG>("last robot made defender");
                    }
                }
                else {
                    if (active_robots.front().position.value != Position::STRIKER) {
                        // Ball is in opponent's half, become Striker
                        emit<Task>(std::make_unique<Striker>(cfg.force_playing));
                        active_robots.front().position = Position("STRIKER");
                        log<NUClear::DEBUG>("last robot made striker");
                    }
                }
            }
        });

        on<Trigger<Penalisation>>().then([this](const Penalisation& self_penalisation) {
            // SPC: Need to remove robot from active_robots
            // TODO: keeping robot around for now, might want to keep info, revisit
            for (auto it = active_robots.begin(); it != active_robots.end(); ++it) {
                if (it->robot_id == self_penalisation.robot_id) {
                    penalised_robots.push_back(*it);
                    active_robots.erase(it);
                    break;
                }
            }

            // If the robot is penalised, its purpose doesn't matter anymore, it must stand still
            if (self_penalisation.context == GameEvents::Context::SELF) {
                emit(std::make_unique<ResetWebotsServos>());
                emit(std::make_unique<Stability>(Stability::UNKNOWN));
                emit(std::make_unique<ResetFieldLocalisation>());
                emit<Task>(std::unique_ptr<FindPurpose>(nullptr));
            }
        });

        on<Trigger<Unpenalisation>>().then([this](const Unpenalisation& self_unpenalisation) {
            // TODO: maybe take robot from penalised vector and add robot to active_robots
            // If the robot is unpenalised, stop standing still and find its purpose
            if (self_unpenalisation.context == GameEvents::Context::SELF) {
                emit<Task>(std::make_unique<FindPurpose>(), 1);
                auto msg = std::make_unique<RoboCup>();
                msg->current_pose.player_id = PLAYER_ID;
                emit(msg);
            }
        });

        on<Trigger<ButtonMiddleDown>, Single>().then([this] {
            // Middle button forces playing
            log<NUClear::INFO>("Middle button pressed!");
            if (!cfg.force_playing) {
                log<NUClear::INFO>("Force playing started.");
                cfg.force_playing = true;
                emit<Task>(std::make_unique<FindPurpose>(), 1);
                auto msg = std::make_unique<RoboCup>();
                msg->current_pose.player_id = PLAYER_ID;
                emit(msg);
            }
        });

        on<Every<2, Per<std::chrono::seconds>>>().then([this] {
            auto now = std::chrono::steady_clock::now();
            constexpr auto timeout = std::chrono::seconds(10); //TODO: test with faster computer

            // remove inactive robots
            auto it = active_robots.begin();
            while (it != active_robots.end()) {
                if (it->robot_id != PLAYER_ID && now - it->last_heard_from > timeout) {
                    log<NUClear::DEBUG>("remove player ", int(it->robot_id));
                    it = active_robots.erase(it);
                } else {
                    ++it;
                }
            }

            //TODO: DELETE ME
            // Loop through each robot and print its info
            log<NUClear::DEBUG>("-- PRINT ROBOT INFO --");
            for (const auto& robot : active_robots) {
                log<NUClear::DEBUG>("Robot ID ", int(robot.robot_id));
                if (robot.robot_id != PLAYER_ID) {
                    log<NUClear::DEBUG>("last heard from ", std::chrono::duration_cast<std::chrono::seconds>(now - robot.last_heard_from).count());
                }
                log<NUClear::DEBUG>("Position: ", robot.position.toString());
            }
            log<NUClear::DEBUG>("-- END --");
            // DELETE ME END
        });
    }

    void Soccer::add_robot(RobotInfo new_robot) {
        // Find the correct insertion point using std::lower_bound
        auto insertPos = std::lower_bound(active_robots.begin(), active_robots.end(), new_robot,
            [](const RobotInfo& a, const RobotInfo& b) {
                return a.robot_id < b.robot_id;
            });

        // Insert the new robot at the found position to maintain order
        active_robots.insert(insertPos, new_robot);
    }

    bool Soccer::manage_active_robots(const uint8_t robot_id) {
        auto now = std::chrono::steady_clock::now();

        std::vector<RobotInfo>::iterator it = std::find_if(active_robots.begin(), active_robots.end(), [robot_id](const RobotInfo& info) {
            return info.robot_id == robot_id;
        });

        if (it != active_robots.end()) {
            // Update existing robot's last_heard_from
            it->last_heard_from = now;
        } else {
            // Add new robot info
            RobotInfo new_robot{robot_id, now};
            add_robot(new_robot);

            return true;
        }
        return false;
    };

    uint8_t Soccer::count_defenders() {
        uint8_t res = std::count_if(active_robots.begin(), active_robots.end(),
            [](const RobotInfo& robot) { return robot.position.value == Position::DEFENDER; });
        return res;
    };

    void Soccer::find_soccer_position(const RoboCup& robocup) {
        // If I am not the leader, follow leader's message, and store positions

        // If I am the leader, then decide positions and emit this
        bool leader = active_robots.front().robot_id == PLAYER_ID;
        if (leader) {
            log<NUClear::DEBUG>("I am ze leader");
        }

        uint8_t num_robots = active_robots.size();
        uint8_t num_defenders = std::ceil(num_robots / 2.0);
        uint8_t defenderCount = count_defenders();

        // test needed for decisions later
        if (num_defenders != defenderCount) {
            // Assign roles (temporary logic for simplicity)
            // robocup + position tracker probs needed here later
            for (size_t i = 0; i < active_robots.size(); ++i) {
                active_robots[i].position = (i < num_defenders) ? Position("DEFENDER") : Position("STRIKER");

                if (PLAYER_ID == active_robots[i].robot_id) {
                    switch (active_robots[i].position.value) {
                        case Position::STRIKER: emit<Task>(std::make_unique<Striker>(cfg.force_playing)); break;
                        case Position::DEFENDER: emit<Task>(std::make_unique<Defender>(cfg.force_playing)); break;
                        default: log<NUClear::ERROR>("Invalid robot position");
                    }
                }
            }
        }
    };


}  // namespace module::purpose
