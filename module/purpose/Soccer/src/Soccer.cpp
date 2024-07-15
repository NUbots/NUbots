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

#include <fmt/format.h>
#include <string>
#include <vector>

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/behaviour/state/Stability.hpp"
#include "message/behaviour/state/WalkState.hpp"
#include "message/input/Buttons.hpp"
#include "message/input/GameEvents.hpp"
#include "message/input/RoboCup.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/output/Buzzer.hpp"
#include "message/platform/RawSensors.hpp"
#include "message/purpose/AllRounder.hpp"
#include "message/purpose/Defender.hpp"
#include "message/purpose/FindPurpose.hpp"
#include "message/purpose/Goalie.hpp"
#include "message/purpose/Purpose.hpp"
#include "message/purpose/Striker.hpp"
#include "message/purpose/UpdateBoundingBox.hpp"
#include "message/skill/Look.hpp"
#include "message/skill/Walk.hpp"
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
    using message::input::ButtonLeftDown;
    using message::input::ButtonLeftUp;
    using message::input::ButtonMiddleDown;
    using message::input::ButtonMiddleUp;
    using message::input::GameEvents;
    using message::input::RoboCup;
    using message::input::Sensors;
    using message::localisation::Ball;
    using message::localisation::ResetFieldLocalisation;
    using message::output::Buzzer;
    using message::platform::ResetWebotsServos;
    using message::purpose::AllRounder;
    using message::purpose::Defender;
    using message::purpose::FindPurpose;
    using message::purpose::Goalie;
    using message::purpose::Purpose;
    using message::purpose::SoccerPosition;
    using message::purpose::Striker;
    using message::purpose::UpdateBoundingBox;
    using message::skill::Look;
    using message::skill::Walk;
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

                cfg.disable_idle_delay = config["disable_idle_delay"].as<int>();

                // Get the number of seconds until we assume a teammate is inactive
                cfg.timeout = config["timeout"].as<int>();

                cfg.max_robots = config["max_robots"].as<uint>();
                // Resize the vector based on cfg.max_robots
                robots.resize(cfg.max_robots);

                // Decide which player we are
                player_id = global_config.player_id;
                log<NUClear::DEBUG>("Configure Player ID ", int(player_id));

                if (cfg.position == Position::DYNAMIC) {
                    robots[player_id - 1].dynamic = true;
                }

                // Load bounding box configs
                cfg.goalie_bounding_box.x_min = config["goalie"]["x_min"].as<double>();
                cfg.goalie_bounding_box.x_max = config["goalie"]["x_max"].as<double>();
                cfg.goalie_bounding_box.y_min = config["goalie"]["y_min"].as<double>();
                cfg.goalie_bounding_box.y_max = config["goalie"]["y_max"].as<double>();

                cfg.defender_bounding_box.x_min = config["defender"]["x_min"].as<double>();
                cfg.defender_bounding_box.x_max = config["defender"]["x_max"].as<double>();
                cfg.defender_bounding_box.y_min = config["defender"]["y_min"].as<double>();
                cfg.defender_bounding_box.y_max = config["defender"]["y_max"].as<double>();

                cfg.striker_bounding_box.x_min = config["striker"]["x_min"].as<double>();
                cfg.striker_bounding_box.x_max = config["striker"]["x_max"].as<double>();
                cfg.striker_bounding_box.y_min = config["striker"]["y_min"].as<double>();
                cfg.striker_bounding_box.y_max = config["striker"]["y_max"].as<double>();
            });

        // Start the Director graph for the soccer scenario!
        on<Startup>().then([this] {
            // At the start of the program, we should be standing
            // Without these emits, modules that need a Stability and WalkState messages may not run
            emit(std::make_unique<Stability>(Stability::UNKNOWN));
            emit(std::make_unique<WalkState>(WalkState::State::STOPPED));
            // Stand idle
            emit<Task>(std::make_unique<Walk>(Eigen::Vector3d::Zero()), 0);
            // Idle look forward if the head isn't doing anything else
            emit<Task>(std::make_unique<Look>(Eigen::Vector3d::UnitX(), true), 0);
            // This emit starts the tree to play soccer
            emit<Task>(std::make_unique<FindPurpose>(), 1);
            // The robot should always try to recover from falling, if applicable, regardless of purpose
            emit<Task>(std::make_unique<FallRecovery>(), 2);
        });

        on<Provide<FindPurpose>, Every<BEHAVIOUR_UPDATE_RATE, Per<std::chrono::seconds>>>().then([this]() {
            // We are alive!
            robots[player_id - 1].active     = true;
            robots[player_id - 1].last_heard = NUClear::clock::now();

            // Make task based on configured purpose/soccer position
            switch (cfg.position) {
                case Position::ALL_ROUNDER:
                    emit<Task>(std::make_unique<AllRounder>(cfg.force_playing));
                    robots[player_id - 1].position = Position("ALL_ROUNDER");
                    break;
                case Position::STRIKER:
                    emit<Task>(std::make_unique<Striker>(cfg.force_playing));
                    robots[player_id - 1].position = Position("STRIKER");
                    break;
                case Position::GOALIE:
                    emit<Task>(std::make_unique<Goalie>(cfg.force_playing));
                    robots[player_id - 1].position = Position("GOALIE");
                    break;
                case Position::DEFENDER:
                    emit<Task>(std::make_unique<Defender>(cfg.force_playing));
                    robots[player_id - 1].position = Position("DEFENDER");
                    break;
                case Position::DYNAMIC: determine_purpose(); break;
                default: log<NUClear::ERROR>("Invalid robot position");
            }
        });

        on<Every<5, Per<std::chrono::seconds>>>().then([this] {
            // Emit the purpose
            emit(std::make_unique<Purpose>(player_id,
                                           SoccerPosition(int(robots[player_id - 1].position)),
                                           robots[player_id - 1].dynamic,
                                           robots[player_id - 1].active));
        });

        on<Trigger<Penalisation>>().then([this](const Penalisation& self_penalisation) {
            // If the robot is penalised, its purpose doesn't matter anymore, it must stand still
            if (!cfg.force_playing && self_penalisation.context == GameEvents::Context::SELF) {
                emit(std::make_unique<ResetWebotsServos>());
                emit(std::make_unique<Stability>(Stability::UNKNOWN));
                emit(std::make_unique<ResetFieldLocalisation>());
                emit<Task>(std::unique_ptr<FindPurpose>(nullptr));
                emit<Task>(std::make_unique<Walk>(Eigen::Vector3d::Zero()), 3);
            }

            // Reset dynamic robot to no position
            if (robots[self_penalisation.robot_id - 1].dynamic) {
                robots[self_penalisation.robot_id - 1].position = Position::DYNAMIC;
            }
            // Set penalised robot to inactive
            robots[self_penalisation.robot_id - 1].active = false;
        });

        on<Trigger<Unpenalisation>>().then([this](const Unpenalisation& self_unpenalisation) {
            // If the robot is unpenalised, stop standing still and find its purpose
            if (!cfg.force_playing && self_unpenalisation.context == GameEvents::Context::SELF) {
                emit<Task>(std::make_unique<FindPurpose>(), 1);
                emit<Task>(std::make_unique<Walk>(Eigen::Vector3d::Zero()), 0);
            }
        });

        // Left button pauses the soccer scenario
        on<Trigger<ButtonLeftDown>>().then([this] {
            emit<Scope::DIRECT>(std::make_unique<ResetFieldLocalisation>());
            emit<Scope::DIRECT>(std::make_unique<EnableIdle>());
            emit<Scope::DIRECT>(std::make_unique<Buzzer>(1000));
            idle = true;
        });

        on<Trigger<ButtonLeftUp>>().then([this] { emit<Scope::DIRECT>(std::make_unique<Buzzer>(0)); });

        on<Trigger<EnableIdle>>().then([this] {
            // Stop all tasks and stand still
            emit<Task>(std::unique_ptr<FindPurpose>(nullptr));
            emit<Task>(std::unique_ptr<FallRecovery>(nullptr));
            emit(std::make_unique<Stability>(Stability::UNKNOWN));
            log<NUClear::INFO>("Idle mode enabled");
        });

        // Middle button resumes the soccer scenario
        on<Trigger<ButtonMiddleDown>>().then([this] {
            emit<Scope::DIRECT>(std::make_unique<ResetFieldLocalisation>());
            // Restart the Director graph for the soccer scenario after a delay
            emit<Scope::DELAY>(std::make_unique<DisableIdle>(), std::chrono::seconds(cfg.disable_idle_delay));
            emit<Scope::DIRECT>(std::make_unique<Buzzer>(1000));
            idle = false;
        });

        on<Trigger<ButtonMiddleUp>>().then([this] { emit<Scope::DIRECT>(std::make_unique<Buzzer>(0)); });

        on<Trigger<DisableIdle>>().then([this] {
            // If the robot is not idle, restart the Director graph for the soccer scenario!
            if (!idle) {
                emit<Task>(std::make_unique<FindPurpose>(), 1);
                emit<Task>(std::make_unique<FallRecovery>(), 2);
                log<NUClear::INFO>("Idle mode disabled");
            }
        });

        on<Trigger<RoboCup>>().then([this](const RoboCup& robocup) {
            // Save info from this robot
            robots[robocup.current_pose.player_id - 1].position   = Position(robocup.purpose.purpose);
            robots[robocup.current_pose.player_id - 1].active     = robocup.purpose.active;
            robots[robocup.current_pose.player_id - 1].last_heard = NUClear::clock::now();
            robots[robocup.current_pose.player_id - 1].dynamic    = robocup.purpose.dynamic;

            // Determine distance to ball
            Eigen::Vector2d rBFf(robocup.ball.position.x(), robocup.ball.position.y());
            Eigen::Vector2d rRFf(robocup.current_pose.position.x(), robocup.current_pose.position.y());
            robots[robocup.current_pose.player_id - 1].distance_to_ball = (rRFf - rBFf).norm();
        });

        on<Trigger<Ball>, With<Sensors>>().then([this](const Ball& ball, const Sensors& sensors) {
            // Update our distance to the ball
            robots[player_id - 1].distance_to_ball = (sensors.Hrw * ball.rBWw).norm();
        });
    }

    void Soccer::determine_purpose() {
        // Print all robots if in debug
        if (log_level <= NUClear::DEBUG) {
            log<NUClear::DEBUG>("\nRobot positions:");
            for (size_t i = 0; i < robots.size(); i++) {
                log<NUClear::DEBUG>(fmt::format("Robot {}\tPosition: {}\tActive: {}\tDynamic: {}",
                                                i + 1,
                                                std::string(robots[i].position),
                                                robots[i].active ? "Yes" : "No",
                                                robots[i].dynamic ? "Yes" : "No"));
            }
        }

        // Update active robots
        for (auto& robot : robots) {
            if (std::chrono::duration_cast<std::chrono::seconds>(NUClear::clock::now() - robot.last_heard).count()
                > cfg.timeout) {
                robot.active = false;
            }
        }

        // If we have no purpose (dynamic) be a defender
        robots[player_id - 1].position =
            robots[player_id - 1].position == Position::DYNAMIC ? Position::DEFENDER : robots[player_id - 1].position;

        // Check if there are any strikers
        int number_strikers = false;
        for (auto& robot : robots) {
            if (robot.active && robot.position == Position::STRIKER) {
                number_strikers++;
            }
        }

        // If there are no strikers, become a striker
        robots[player_id - 1].position = number_strikers == 0 ? Position::STRIKER : robots[player_id - 1].position;

        // If there are too many strikers, and we are one of them, see if we should be a defender
        if (number_strikers > 1 && robots[player_id - 1].position == Position::STRIKER) {
            // Battle it out for striker position
            // If we are the closest to the ball, we will be the striker
            bool striker = true;
            for (auto& robot : robots) {
                // Ignore inactive robots and robots that are not strikers
                if (!robot.active || robot.position != Position::STRIKER) {
                    continue;
                }

                // Non-dynamic robots automatically win, otherwise check distance to the ball
                if (!robot.dynamic || robot.distance_to_ball < robots[player_id - 1].distance_to_ball) {
                    striker = false;
                    break;
                }
            }
            // We lost, be a defender
            robots[player_id - 1].position = striker ? Position::STRIKER : Position::DEFENDER;
        }

        // Emit the Task for our position
        if (robots[player_id - 1].position == Position::STRIKER) {
            emit<Task>(std::make_unique<Striker>(cfg.force_playing));
        }
        else {
            emit<Task>(std::make_unique<Defender>(cfg.force_playing));
        }

        // Update bounding box based on the position of the robot and existence of other robots
        bool defender_exist = std::count_if(robots.begin(),
                                            robots.end(),
                                            [](const RobotInfo& robot) { return robot.position == Position::DEFENDER; })
                              > 0;
        bool goalie_exist = std::count_if(robots.begin(),
                                          robots.end(),
                                          [](const RobotInfo& robot) { return robot.position == Position::GOALIE; })
                            > 0;

        // If you are striker or defender and no one else exists
        if (std::count_if(robots.begin(), robots.end(), [](const RobotInfo& robot) { return robot.active; }) == 1) {
            // Update bounding box to full box
            emit(std::make_unique<UpdateBoundingBox>(cfg.striker_bounding_box.x_min,
                                                     cfg.goalie_bounding_box.x_max,
                                                     cfg.striker_bounding_box.y_min,
                                                     cfg.striker_bounding_box.y_max));
            log<NUClear::DEBUG>("Full field player");
            return;
        }


        // If you are striker and defender exists
        if (robots[player_id - 1].position == Position::STRIKER && defender_exist) {
            // Update bounding box to default box
            emit(std::make_unique<UpdateBoundingBox>(cfg.striker_bounding_box.x_min,
                                                     cfg.striker_bounding_box.x_max,
                                                     cfg.striker_bounding_box.y_min,
                                                     cfg.striker_bounding_box.y_max));
            log<NUClear::DEBUG>("Default striker");
            return;
        }

        // If you are striker, but there isn't any defender
        if (robots[player_id - 1].position == Position::STRIKER && !defender_exist) {
            // Increase to defender + striker bounding box
            emit(std::make_unique<UpdateBoundingBox>(cfg.striker_bounding_box.x_min,
                                                     cfg.defender_bounding_box.x_max,
                                                     cfg.striker_bounding_box.y_min,
                                                     cfg.striker_bounding_box.y_max));
            log<NUClear::DEBUG>("Extended striker");
            return;
        }

        // If you are defender and a goalie exists
        if (robots[player_id - 1].position == Position::DEFENDER && goalie_exist) {
            // Update bounding box to default box
            emit(std::make_unique<UpdateBoundingBox>(cfg.defender_bounding_box.x_min,
                                                     cfg.defender_bounding_box.x_max,
                                                     cfg.defender_bounding_box.y_min,
                                                     cfg.defender_bounding_box.y_max));
            log<NUClear::DEBUG>("Default defender");
            return;
        }

        // If you are defender, but there isn't a goalie
        if (robots[player_id - 1].position == Position::DEFENDER && !goalie_exist) {
            // Increase to defender + goalie bounding box
            emit(std::make_unique<UpdateBoundingBox>(cfg.defender_bounding_box.x_min,
                                                     cfg.goalie_bounding_box.x_max,
                                                     cfg.defender_bounding_box.y_min,
                                                     cfg.defender_bounding_box.y_max));
            log<NUClear::DEBUG>("Extended defender");
            return;
        }
    }

}  // namespace module::purpose
