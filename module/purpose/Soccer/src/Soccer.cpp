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
#include "message/input/Purposes.hpp"
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
    using message::input::Purposes;
    using message::input::SoccerPosition;
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
            log<NUClear::DEBUG>("Configure Player ID ", int(PLAYER_ID));
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
                    manage_active_robots(robocup);
                    break;
                default: log<NUClear::ERROR>("Invalid robot position");
            }
        });

        on<Trigger<Penalisation>>().then([this](const Penalisation& self_penalisation) {
            // SPC: Need to remove robot from active_robots
            for (auto it = active_robots.begin(); it != active_robots.end(); ++it) {
                if (it->robot_id == self_penalisation.robot_id) {
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

            decide_purposes();
        });

        on<Trigger<Unpenalisation>>().then([this](const Unpenalisation& self_unpenalisation) {
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
            for (auto it = active_robots.begin(); it != active_robots.end(); ) {
                if (it->robot_id != PLAYER_ID && now - it->last_heard_from > timeout) {
                    log<NUClear::DEBUG>("remove player ", int(it->robot_id));
                    it = active_robots.erase(it);
                    decide_purposes();
                } else {
                    ++it;
                }
            }

            log<NUClear::DEBUG>("active robots length ", int(active_robots.size()));
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

    void Soccer::manage_active_robots(const RoboCup& robocup) {
        // do not manage penalised robots
        if (robocup.state == State::PENALISED) { return; }

        uint8_t incoming_robot_id = robocup.current_pose.player_id;
        auto now = std::chrono::steady_clock::now();

        std::vector<RobotInfo>::iterator it = std::find_if(active_robots.begin(), active_robots.end(), [incoming_robot_id](const RobotInfo& info) {
            return info.robot_id == incoming_robot_id;
        });

        if (it != active_robots.end()) {
            // Update existing robot's last_heard_from
            it->last_heard_from = now;
            // TODO: may need more data
            // Update existing robot's x position
        } else {
            // Add their robot info
            RobotInfo new_robot{incoming_robot_id, now};
            add_robot(new_robot);


            bool self_is_leader = active_robots.front().robot_id == PLAYER_ID;
            // A robot may emit leader messages mistakenly. Only listen to real leader.
            bool other_is_leader = active_robots.front().robot_id == incoming_robot_id;

            if (self_is_leader) {
                decide_purposes();
            } else if (other_is_leader) {
                learn_purpose(robocup);
            }
        }
    };

    // find which robot should be our striker
    uint8_t Soccer::find_striker() {
        // temp logic
        return active_robots.size() - 1;

        // TODO: player furthest ahead on field should be striker, or maybe based on closest to ball
        //         // Get the current position of the ball on the field
        //         Eigen::Isometry3d Hfw = field.Hfw;
        //         Eigen::Vector3d rBFf  = Hfw * ball.rBWw;

        //         log<NUClear::DEBUG>("rBFf", rBFf.x());
        //         // leeway to prevent kickoff confusion
        //         // TODO: store this value, test variation instead?
        //         if (rBFf.x() >= 0.2) {
        //             // Ball is own half, so become Defender.

    }

    // decide everyone's soccer positions if I am the leader
    void Soccer::decide_purposes() {
        // TODO: robocup + position tracker probs needed here later
        auto purposes_msg = std::make_unique<Purposes>();
        uint8_t striker_idx = find_striker();

        for (int i = 0; i < int(active_robots.size()); ++i) {
            if (i == striker_idx) {
                purposes_msg->purposes.push_back({active_robots[i].robot_id, SoccerPosition::STRIKER});
            } else {
                purposes_msg->purposes.push_back({active_robots[i].robot_id, SoccerPosition::DEFENDER});
            }
        }

        emit(purposes_msg);
    };

    // listen to soccer positions if they are the leader
    void Soccer::learn_purpose(const RoboCup& robocup) {
        for (const auto& purpose : robocup.purposes.purposes) {
            if (purpose.purpose == SoccerPosition::DEFENDER) {
                emit<Task>(std::make_unique<Defender>(cfg.force_playing));
                log<NUClear::DEBUG>("made defender");
            }
            else if (purpose.purpose == SoccerPosition::STRIKER) {
                emit<Task>(std::make_unique<Striker>(cfg.force_playing));
                log<NUClear::DEBUG>("made striker");
            }
        }
    }


}  // namespace module::purpose
