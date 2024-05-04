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
            activeRobots = std::vector<RobotInfo>();

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
        });

        on<Provide<FindPurpose>>().then([this] () {
            // Make task based on configured purpose/soccer position
            switch (cfg.position) {
                case Position::STRIKER: emit<Task>(std::make_unique<Striker>(cfg.force_playing)); break;
                case Position::GOALIE: emit<Task>(std::make_unique<Goalie>(cfg.force_playing)); break;
                case Position::DEFENDER: emit<Task>(std::make_unique<Defender>(cfg.force_playing)); break;
                case Position::DYNAMIC:
                    // default to defender?
                    manageActiveRobots(PLAYER_ID);
                    emit<Task>(std::make_unique<Defender>(cfg.force_playing)); break;
                default: log<NUClear::ERROR>("Invalid robot position");
            }
        });

        on<Provide<FindPurpose>(), Trigger<RoboCup>>().then([this] (const RoboCup robocup) {
            // Make task based on configured purpose/soccer position
            // TODO: will still need penalised robots info if leader elec
            if (cfg.position == Position::DYNAMIC && robocup.state == State::UNPENALISED) {
                manageActiveRobots(robocup.current_pose.player_id);
                findSoccerPosition(robocup);
            }
        });

        on<Trigger<Penalisation>>().then([this](const Penalisation& self_penalisation) {
            // SPC: Need to remove robot from activeRobots
            // TODO: keeping robot around for now, might want to keep info, revisit
            for (auto it = activeRobots.begin(); it != activeRobots.end(); ++it) {
                if (it->robotId == self_penalisation.robot_id) {
                    penalisedRobots.push_back(*it);
                    activeRobots.erase(it);
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
            // TODO: maybe take robot from penalised vector and add robot to activeRobots
            // If the robot is unpenalised, stop standing still and find its purpose
            if (self_unpenalisation.context == GameEvents::Context::SELF) {
                emit<Task>(std::make_unique<FindPurpose>(), 1);
            }
        });

        on<Trigger<ButtonMiddleDown>, Single>().then([this] {
            // Middle button forces playing
            log<NUClear::INFO>("Middle button pressed!");
            if (!cfg.force_playing) {
                log<NUClear::INFO>("Force playing started.");
                cfg.force_playing = true;
                emit<Task>(std::make_unique<FindPurpose>(), 1);
            }
        });

        on<Every<2, Per<std::chrono::seconds>>>().then([this] {
            auto now = std::chrono::steady_clock::now();
            constexpr auto timeout = std::chrono::seconds(10); //TODO: test with faster computer

            // remove inactive robots
            auto it = activeRobots.begin();
            while (it != activeRobots.end()) {
                if (it->robotId != PLAYER_ID && now - it->lastHeardFrom > timeout) {
                    log<NUClear::DEBUG>("remove player ", int(it->robotId));
                    it = activeRobots.erase(it);
                } else {
                    ++it;
                }
            }

            //TODO: DELETE ME
            // Loop through each robot and print its info
            log<NUClear::DEBUG>("-- PRINT ROBOT INFO --");
            for (const auto& robot : activeRobots) {
                log<NUClear::DEBUG>("Robot ID ", int(robot.robotId));
                if (robot.robotId != PLAYER_ID) {
                    log<NUClear::DEBUG>("last heard from ", std::chrono::duration_cast<std::chrono::seconds>(now - robot.lastHeardFrom).count());
                }
                log<NUClear::DEBUG>("Position: ", robot.position.toString());
            }
            log<NUClear::DEBUG>("-- END --");
            // DELETE ME END
        });
    }

    void Soccer::addRobot(RobotInfo newRobot) {
        // Find the correct insertion point using std::lower_bound
        auto insertPos = std::lower_bound(activeRobots.begin(), activeRobots.end(), newRobot,
            [](const RobotInfo& a, const RobotInfo& b) {
                return a.robotId < b.robotId;
            });

        // Insert the new robot at the found position to maintain order
        activeRobots.insert(insertPos, newRobot);
    }

    bool Soccer::manageActiveRobots(const uint8_t robotId) {
        auto now = std::chrono::steady_clock::now();

        std::vector<RobotInfo>::iterator it = std::find_if(activeRobots.begin(), activeRobots.end(), [robotId](const RobotInfo& info) {
            return info.robotId == robotId;
        });

        if (it != activeRobots.end()) {
            // Update existing robot's lastHeardFrom
            it->lastHeardFrom = now;
        } else {
            // Add new robot info
            RobotInfo newRobot{robotId, now};
            addRobot(newRobot);

            return true;
        }
        return false;
    };

    uint8_t Soccer::countDefenders() {
        uint8_t res = std::count_if(activeRobots.begin(), activeRobots.end(),
            [](const RobotInfo& robot) { return robot.position.value == Position::DEFENDER; });
        return res;
    }

    void Soccer::findSoccerPosition(const RoboCup& robocup) {
        // If I am not the leader, follow leader's message, and store positions
        // check that the leader is actually the expected leader?

        // If I am the leader, then decide positions and emit this
        bool leader = activeRobots.front().robotId == PLAYER_ID;
        if (leader) {
            log<NUClear::DEBUG>("I am ze leader");
        }


        uint8_t numRobots = activeRobots.size();
        uint8_t numDefenders = std::ceil(numRobots / 2.0);
        uint8_t defenderCount = countDefenders();

        // test needed for decisions later
        if (numDefenders != defenderCount) {
            // Assign roles (temporary logic for simplicity)
            // robocup + position tracker probs needed here later
            for (size_t i = 0; i < activeRobots.size(); ++i) {
                activeRobots[i].position = (i < numDefenders) ? Position("DEFENDER") : Position("STRIKER");

                if (PLAYER_ID == activeRobots[i].robotId) {
                    switch (activeRobots[i].position.value) {
                        case Position::STRIKER: emit<Task>(std::make_unique<Striker>(cfg.force_playing)); break;
                        case Position::DEFENDER: emit<Task>(std::make_unique<Defender>(cfg.force_playing)); break;
                        default: log<NUClear::ERROR>("Invalid robot position");
                    }
                }
            }
        }
    };


}  // namespace module::purpose
