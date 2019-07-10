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

#include "SoccerSimulator.h"

#include <nuclear>
#include <sstream>

#include "extension/Configuration.h"

#include "message/input/GameEvents.h"
#include "message/input/GameState.h"
#include "message/input/Image.h"
#include "message/input/Sensors.h"
#include "message/localisation/Ball.h"
#include "message/localisation/Field.h"
#include "message/motion/WalkCommand.h"
#include "message/vision/Ball.h"
#include "message/vision/Goal.h"

#include "utility/math/angle.h"
#include "utility/math/coordinates.h"
#include "utility/motion/ForwardKinematics.h"
#include "utility/nusight/NUhelpers.h"
#include "utility/support/yaml_armadillo.h"


namespace module {
namespace support {

    using extension::Configuration;

    using message::input::GameEvents;
    using message::input::GameState;
    using message::input::Image;
    using message::input::Sensors;
    using message::motion::KickCommand;
    using message::motion::KickFinished;
    using message::motion::KickPlannerConfig;
    using message::motion::KickScriptCommand;
    using message::motion::StopCommand;
    using message::motion::WalkCommand;
    using message::platform::darwin::ButtonMiddleDown;
    using message::platform::darwin::DarwinSensors;
    using message::support::FieldDescription;
    using message::support::GlobalConfig;
    using message::vision::Balls;
    using message::vision::Goal;
    using message::vision::Goals;

    using utility::math::angle::bearingToUnitVector;
    using utility::math::angle::normalizeAngle;
    using utility::math::angle::vectorToBearing;
    using utility::math::coordinates::cartesianToSpherical;
    using utility::math::matrix::Transform2D;
    using utility::motion::kinematics::calculateRobotToIMU;
    using utility::nusight::drawArrow;
    using utility::nusight::drawSphere;
    using utility::nusight::graph;
    using utility::support::Expression;

    double triangle_wave(double t, double period) {
        auto a = period;  // / 2.0;
        auto k = t / a;
        return 2.0 * std::abs(2.0 * (k - std::floor(k + 0.5))) - 1.0;
    }
    double sawtooth_wave(double t, double period) {
        return 2.0 * std::fmod(t / period, 1.0) - 1.0;
    }
    double square_wave(double t, double period) {
        return std::copysign(1.0, sawtooth_wave(t, period));
    }
    double sine_wave(double t, double period) {
        return std::sin((2.0 * M_PI * t) / period);
    }
    double SoccerSimulator::absolute_time() {
        auto now          = NUClear::clock::now();
        auto msSinceStart = std::chrono::duration_cast<std::chrono::microseconds>(now - moduleStartupTime).count();
        double ms         = static_cast<double>(msSinceStart);
        double t          = ms * 1e-6;
        return t;
    }

    void SoccerSimulator::updateConfiguration(const Configuration& config, const GlobalConfig& globalConfig) {

        moduleStartupTime = NUClear::clock::now();

        cfg_.simulate_goal_observations         = config["vision"]["goal_observations"].as<bool>();
        cfg_.simulate_ball_observations         = config["vision"]["ball_observations"].as<bool>();
        cfg_.distinguish_own_and_opponent_goals = config["vision"]["distinguish_own_and_opponent_goals"].as<bool>();
        cfg_.distinguish_left_and_right_goals   = config["vision"]["distinguish_own_and_opponent_goals"].as<bool>();

        cfg_.robot.motion_type = motionTypeFromString(config["robot"]["motion_type"].as<std::string>());
        cfg_.robot.path.period = config["robot"]["path"]["period"].as<Expression>();
        cfg_.robot.path.x_amp  = config["robot"]["path"]["x_amp"].as<Expression>();
        cfg_.robot.path.y_amp  = config["robot"]["path"]["y_amp"].as<Expression>();
        cfg_.robot.path.type   = pathTypeFromString(config["robot"]["path"]["type"].as<std::string>());

        cfg_.ball.motion_type = motionTypeFromString(config["ball"]["motion_type"].as<std::string>());
        cfg_.ball.path.period = config["ball"]["path"]["period"].as<Expression>();
        cfg_.ball.path.x_amp  = config["ball"]["path"]["x_amp"].as<Expression>();
        cfg_.ball.path.y_amp  = config["ball"]["path"]["y_amp"].as<Expression>();
        cfg_.ball.path.type   = pathTypeFromString(config["ball"]["path"]["type"].as<std::string>());

        world.robotPose = config["initial"]["robot_pose"].as<arma::vec3>();
        world.ball      = VirtualBall(config["initial"]["ball"]["position"].as<arma::vec2>(),
                                 config["initial"]["ball"]["diameter"].as<Expression>());

        cfg_.blind_robot = config["blind_robot"].as<bool>();

        cfg_.vision_error(0) = config["vision"]["variance"]["r"]["proportional_factor"].as<Expression>();
        cfg_.vision_error(1) = config["vision"]["variance"]["r"]["min_error"].as<Expression>();
        cfg_.vision_error(2) = config["vision"]["variance"]["theta"].as<Expression>();
        cfg_.vision_error(3) = config["vision"]["variance"]["phi"].as<Expression>();

        lastNow = NUClear::clock::now();

        kicking   = false;
        PLAYER_ID = globalConfig.playerId;

        cfg_.auto_start_behaviour = config["auto_start_behaviour"].as<bool>();
    }

    SoccerSimulator::SoccerSimulator(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , moduleStartupTime()
        , kick_cfg()
        , cfg_()
        , goalPosts()
        , world()
        , kickQueue()
        , oldRobotPose()
        , oldBallPose()
        , PLAYER_ID(0)
        , lastNow() {


        on<Trigger<FieldDescription>>().then("FieldDescription Update", [this](const FieldDescription& desc) {
            auto fdptr = std::make_shared<const FieldDescription>(desc);
            loadFieldDescription(fdptr);
        });

        on<Configuration, Trigger<GlobalConfig>>("SoccerSimulator.yaml")
            .then("Soccer Simulator Configuration",
                  [this](const Configuration& config, const GlobalConfig& globalConfig) {
                      updateConfiguration(config, globalConfig);
                  });

        on<Trigger<KickPlannerConfig>>().then("Get Kick Planner Config",
                                              [this](const KickPlannerConfig& cfg) { kick_cfg = cfg; });

        on<Trigger<KickCommand>>().then("Simulator Queue KickCommand", [this](const KickCommand& k) {
            kickQueue.push(k);
            kicking = true;
        });

        on<Trigger<KickScriptCommand>>().then("Simulator Queue KickCommand", [this](const KickScriptCommand& k) {
            kickQueue.push(KickCommand({1, 0, 0}, k.direction));
            kicking = true;
        });

        on<Trigger<KickFinished>>().then("Simulator Kick Finished", [this] { kicking = false; });

        on<Trigger<WalkCommand>>().then("Sim walk start", [this] { walking = true; });

        on<Trigger<StopCommand>>().then("Sim walk start", [this] { walking = false; });

        on<Every<SIMULATION_UPDATE_FREQUENCY, Per<std::chrono::seconds>>, With<Sensors>, Optional<With<WalkCommand>>>()
            .then(
                "Robot motion simulation",
                [this](const Sensors& /*sensors*/, std::shared_ptr<const WalkCommand> walkCommand) {
                    NUClear::clock::time_point now = NUClear::clock::now();
                    double deltaT = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(now - lastNow).count();
                    Transform2D diff;

                    switch (cfg_.robot.motion_type) {
                        case MotionType::NONE: world.robotVelocity = Transform2D({0, 0, 0}); break;

                        case MotionType::PATH:

                            world.robotPose.xy() = getPath(cfg_.robot.path);

                            diff = world.robotPose - oldRobotPose;
                            // Face along direction of movement
                            world.robotPose.angle() = vectorToBearing(diff.xy());

                            world.robotVelocity = Transform2D({arma::norm(diff) / deltaT, 0, 0});  // Robot
                                                                                                   // coordinates
                            break;

                        case MotionType::MOTION:

                            // Update based on walk engine
                            if (walking && walkCommand && !kicking) {
                                world.robotVelocity.xy() = Transform2D(convert(walkCommand->command)).xy() * 0.15;
                                // world.robotVelocity.xy() = sensors.odometry;
                                // angle from command:
                                world.robotVelocity.angle() = Transform2D(convert(walkCommand->command)).angle() * 1.0;
                            }
                            else {
                                world.robotVelocity = utility::math::matrix::Transform2D({0, 0, 0});
                            }
                            world.robotVelocity.xy() = world.robotPose.rotation() * world.robotVelocity.xy();
                            world.robotPose += world.robotVelocity * deltaT;
                            break;
                    }
                    // Update ball position
                    switch (cfg_.ball.motion_type) {
                        case MotionType::NONE: world.ball.velocity = {0, 0, 0}; break;

                        case MotionType::PATH:

                            world.ball.position.rows(0, 1) = getPath(cfg_.ball.path);

                            world.ball.velocity = (world.ball.position - oldBallPose) / deltaT;  // world coordinates
                            break;

                        case MotionType::MOTION:
                            // log("check kick:", !kickQueue.empty(), !kicking, lastKicking);
                            if (!kickQueue.empty() && !kicking && lastKicking) {
                                // Get last queue
                                KickCommand lastKickCommand = kickQueue.back();
                                // Empty queue
                                std::queue<KickCommand>().swap(kickQueue);
                                // Check if kick worked:
                                Transform2D relativeBallPose = world.robotPose.worldToLocal(world.ball.position);

                                world.ball.position.rows(0, 1) +=
                                    world.robotPose.rotation()
                                    * convert(lastKickCommand.direction.head<2>().normalized());
                            }
                            break;
                    }


                    // Emit the change in orientation as a DarwinSensors::Gyroscope,
                    // to be handled by HardwareSimulator.
                    emit(computeGyro(world.robotPose.angle(), oldRobotPose.angle()));

                    oldRobotPose = world.robotPose;
                    oldBallPose  = world.ball.position;
                    lastNow      = now;
                    lastKicking  = kicking;
                });

        // Simulate Vision
        // VirtualCamera is emitting images with lens parameters at 30 fps
        on<Trigger<Image>, With<Sensors>, Optional<With<FieldDescription>>, Single>().then(
            "Vision Simulation",
            [this](const Image& image, const Sensors& sensors, const std::shared_ptr<const FieldDescription> fd) {
                if (!fd) {
                    NUClear::log<NUClear::ERROR>(
                        __FILE__, __LINE__, "Field Description must be available for vision simulation!");
                    powerplant.shutdown();
                    return;
                }

                if (goalPosts.size() == 0) {
                    loadFieldDescription(fd);
                }

                if (cfg_.simulate_goal_observations) {
                    auto goals = std::make_unique<Goals>();
                    if (cfg_.blind_robot) {
                        emit(std::move(goals));
                        return;
                    }


                    for (auto& g : goalPosts) {

                        // Detect the goal:
                        auto m = g.detect(image, world.robotPose, sensors, cfg_.vision_error, *fd);

                        // Copy across the important bits
                        goals->camera_id = m.camera_id;
                        goals->timestamp = m.timestamp;
                        goals->Hcw       = m.Hcw;

                        if (!m.goals.at(0).measurements.empty()) {
                            if (!cfg_.distinguish_own_and_opponent_goals) {
                                m.goals.at(0).team = message::vision::Goal::Team::UNKNOWN_TEAM;
                            }
                            goals->goals.push_back(m.goals.at(0));
                        }
                    }

                    if (!cfg_.distinguish_left_and_right_goals) {
                        setGoalLeftRightKnowledge(*goals);
                    }

                    emit(std::move(goals));
                }
                else {
                    // Emit current field exactly
                    auto r = std::make_unique<std::vector<message::localisation::Field>>();
                    r->push_back(message::localisation::Field());
                    r->back().position =
                        Eigen::Vector3d(world.robotPose.x(), world.robotPose.y(), world.robotPose.angle());
                    r->back().covariance = Eigen::Matrix3d::Identity() * 0.00001;
                    emit(std::move(r));
                }


                if (cfg_.simulate_ball_observations) {
                    // auto ball_vec = std::make_unique<Balls>();
                    if (cfg_.blind_robot) {
                        emit(std::make_unique<Balls>());
                        return;
                    }

                    emit(
                        std::make_unique<Balls>(world.ball.detect(image, world.robotPose, sensors, cfg_.vision_error)));
                }
                else {
                    // Emit current ball exactly
                    auto b        = std::make_unique<message::localisation::Ball>();
                    b->position   = convert(arma::vec2(world.robotPose.worldToLocal(world.ball.position).xy()));
                    b->covariance = Eigen::Matrix2d::Identity() * 0.00001;

                    emit(std::make_unique<std::vector<message::localisation::Ball>>(1, *b));
                    emit(std::move(b));
                }
            });


        // Emit exact position to NUsight
        on<Every<100, std::chrono::milliseconds>>().then("Emit True Robot Position", [this] {
            arma::vec2 bearingVector      = world.robotPose.rotation() * arma::vec2({1, 0});
            arma::vec3 robotHeadingVector = {bearingVector[0], bearingVector[1], 0};
            emit(drawArrow("robot", {world.robotPose.x(), world.robotPose.y(), 0}, 1, robotHeadingVector, 0));

            emit(drawSphere("ball", {world.ball.position(0), world.ball.position(1), 0}, 0.1, 0));
        });

        on<Startup>().then("SoccerSimulator Startup", [this] {
            if (cfg_.auto_start_behaviour) {
                auto time = NUClear::clock::now();
                emit(std::make_unique<GameEvents::Unpenalisation>(
                    GameEvents::Unpenalisation(GameEvents::Context::SELF, PLAYER_ID)));
                emit(std::make_unique<GameEvents::GamePhase>(
                    GameEvents::GamePhase(GameState::Data::Phase::PLAYING, time, time)));
                emit(std::make_unique<GameState::Data::Phase>(GameState::Data::Phase::PLAYING));
            }
        });
    }

    std::unique_ptr<DarwinSensors::Gyroscope> SoccerSimulator::computeGyro(float heading, float oldHeading) {
        // float dHeading = utility::math::angle::difference(heading, oldHeading);
        float dHeading = heading - oldHeading;

        auto g = std::make_unique<DarwinSensors::Gyroscope>();
        g->x   = 0;
        g->y   = 0;
        g->z   = dHeading;
        return std::move(g);
    }

    arma::vec2 SoccerSimulator::getPath(SoccerSimulator::Config::Motion::Path p) {
        auto t = absolute_time();
        float wave1, wave2;
        switch (p.type) {
            case PathType::SIN:
                wave1 = p.x_amp * sine_wave(t, p.period);
                wave2 = p.y_amp * sine_wave(t + (p.period / 4.0), p.period);
                break;
            case PathType::TRIANGLE:
                wave1 = p.x_amp * triangle_wave(t, p.period);
                wave2 = p.y_amp * triangle_wave(t + (p.period / 4.0), p.period);
                break;
            default:
                std::stringstream str;
                str << __FILE__ << ", " << __LINE__ << ": " << __func__ << ": unknown p.type.";
                throw std::runtime_error(str.str());
        }
        return arma::vec2({wave1, wave2});
    }

    void SoccerSimulator::setGoalLeftRightKnowledge(Goals& goals) {
        // for (auto& g : goalPosts) {
        int leftGoals    = 0;
        int rightGoals   = 0;
        int unknownGoals = 0;
        for (auto& g : goals.goals) {
            // Count sides
            if (g.side == Goal::Side::LEFT) {
                leftGoals++;
            }
            else if (g.side == Goal::Side::RIGHT) {
                rightGoals++;
            }
            else {
                unknownGoals++;
            }
        }

        int totalGoals = leftGoals + rightGoals + unknownGoals;

        // we need to check if more or less than two goals are visible, or if the two visible goals are not a left-right
        // pair,
        // and remove left-right labels if so
        if (totalGoals != 2 || leftGoals != 1 || rightGoals != 1) {
            for (auto& g : goals.goals) {
                g.side = Goal::Side::UNKNOWN_SIDE;
            }
        }
    }

    void SoccerSimulator::loadFieldDescription(const std::shared_ptr<const FieldDescription> fd) {
        // Load goal posts
        goalPosts.clear();

        arma::vec3 goal_opp_r = {fd->goalpost_opp_r[0], fd->goalpost_opp_r[1], 0};
        goalPosts.push_back(VirtualGoalPost(goal_opp_r, 1.1, Goal::Side::RIGHT, Goal::Team::OPPONENT));

        arma::vec3 goal_opp_l = {fd->goalpost_opp_l[0], fd->goalpost_opp_l[1], 0};
        goalPosts.push_back(VirtualGoalPost(goal_opp_l, 1.1, Goal::Side::LEFT, Goal::Team::OPPONENT));

        arma::vec3 goal_own_r = {fd->goalpost_own_r[0], fd->goalpost_own_r[1], 0};
        goalPosts.push_back(VirtualGoalPost(goal_own_r, 1.1, Goal::Side::RIGHT, Goal::Team::OWN));

        arma::vec3 goal_own_l = {fd->goalpost_own_l[0], fd->goalpost_own_l[1], 0};
        goalPosts.push_back(VirtualGoalPost(goal_own_l, 1.1, Goal::Side::LEFT, Goal::Team::OWN));
    }
}  // namespace support
}  // namespace module
