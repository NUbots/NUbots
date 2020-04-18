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
#include "utility/support/yaml_expression.h"


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
    using utility::math::coordinates::cartesianToSpherical;
    using utility::motion::kinematics::calculateRobotToIMU;
    using utility::nusight::graph;
    using utility::support::Expression;

    double triangle_wave(double t, double period) {
        double a = period;  // / 2.0;
        double k = t / a;
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

        Eigen::Vector3d initial_pose  = config["initial"]["robot_pose"].as<Expression>();
        world.robotPose.linear()      = Eigen::Rotation2Dd(initial_pose.z()).toRotationMatrix();
        world.robotPose.translation() = initial_pose.head<2>();
        world.ball                    = VirtualBall(config["initial"]["ball"]["position"].as<Expression>(),
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
            loadFieldDescription(std::make_shared<const FieldDescription>(desc));
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
            kickQueue.push(KickCommand(Eigen::Vector3d::UnitX(), k.direction));
            kicking = true;
        });

        on<Trigger<KickFinished>>().then("Simulator Kick Finished", [this] { kicking = false; });

        on<Trigger<WalkCommand>>().then("Sim walk start", [this] { walking = true; });

        on<Trigger<StopCommand>>().then("Sim walk start", [this] { walking = false; });

        on<Every<SIMULATION_UPDATE_FREQUENCY, Per<std::chrono::seconds>>, With<Sensors>, Optional<With<WalkCommand>>>()
            .then("Robot motion simulation",
                  [this](const Sensors& /*sensors*/, std::shared_ptr<const WalkCommand> walkCommand) {
                      NUClear::clock::time_point now = NUClear::clock::now();
                      double deltaT =
                          1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(now - lastNow).count();
                      Eigen::Affine2d diff;
                      Eigen::Affine2d ball_pose;
                      ball_pose.linear()      = Eigen::Rotation2Dd(world.ball.position.z()).toRotationMatrix();
                      ball_pose.translation() = world.ball.position.head<2>();

                      switch (cfg_.robot.motion_type) {
                          case MotionType::NONE: world.robotVelocity = Eigen::Affine2d::Identity(); break;

                          case MotionType::PATH:

                              world.robotPose.translation() = getPath(cfg_.robot.path);

                              diff.linear() = Eigen::Rotation2Dd(Eigen::Rotation2Dd(world.robotPose.linear()).angle()
                                                                 - Eigen::Rotation2Dd(oldRobotPose.linear()).angle())
                                                  .toRotationMatrix();
                              diff.translation() = world.robotPose.translation() - oldRobotPose.translation();

                              // Face along direction of movement
                              world.robotPose.linear() =
                                  Eigen::Rotation2Dd(std::atan2(diff.translation().y(), diff.translation().x()))
                                      .toRotationMatrix();

                              // Robot coordinates
                              world.robotVelocity.linear() = Eigen::Matrix2d::Zero();
                              world.robotVelocity.translation() =
                                  Eigen::Vector2d(diff.translation().norm() / deltaT, 0.0);

                              break;

                          case MotionType::MOTION:

                              // Update based on walk engine
                              if (walking && walkCommand && !kicking) {
                                  world.robotVelocity.translation() = walkCommand->command.head<2>() * 0.15;
                                  // world.robotVelocity.xy() = sensors.odometry;
                                  // angle from command:
                                  world.robotVelocity.linear() =
                                      Eigen::Rotation2Dd(walkCommand->command.z()).toRotationMatrix();
                              }
                              else {
                                  world.robotVelocity = Eigen::Affine2d::Identity();
                              }
                              world.robotVelocity.translation() =
                                  world.robotPose.rotation() * world.robotVelocity.translation();
                              world.robotPose.translation() += world.robotVelocity.translation() * deltaT;
                              break;
                      }
                      // Update ball position
                      switch (cfg_.ball.motion_type) {
                          case MotionType::NONE: world.ball.velocity = Eigen::Vector3d::Zero(); break;

                          case MotionType::PATH:

                              world.ball.position.head<2>() = getPath(cfg_.ball.path);

                              world.ball.velocity =
                                  (world.ball.position
                                   - Eigen::Vector3d(oldBallPose.translation().x(),
                                                     oldBallPose.translation().y(),
                                                     Eigen::Rotation2Dd(oldBallPose.rotation()).angle()))
                                  / deltaT;  // world coordinates
                              break;

                          case MotionType::MOTION:
                              // log("check kick:", !kickQueue.empty(), !kicking, lastKicking);
                              if (!kickQueue.empty() && !kicking && lastKicking) {
                                  // Get last queue
                                  KickCommand lastKickCommand = kickQueue.back();
                                  // Empty queue
                                  std::queue<KickCommand>().swap(kickQueue);
                                  // Check if kick worked:
                                  Eigen::Affine2d relativeBallPose = world.robotPose.inverse() * ball_pose;

                                  world.ball.position.head<2>() +=
                                      world.robotPose.rotation() * lastKickCommand.direction.head<2>().normalized();
                              }
                              break;
                      }


                      // Emit the change in orientation as a DarwinSensors::Gyroscope,
                      // to be handled by HardwareSimulator.
                      emit(computeGyro(Eigen::Rotation2Dd(world.robotPose.linear()).angle(),
                                       Eigen::Rotation2Dd(oldRobotPose.linear()).angle()));

                      oldRobotPose = world.robotPose;
                      oldBallPose  = ball_pose;
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
                    r->back().position   = Eigen::Vector3d(world.robotPose.translation().x(),
                                                         world.robotPose.translation().y(),
                                                         Eigen::Rotation2Dd(world.robotPose.linear()).angle());
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
                    auto b = std::make_unique<message::localisation::Ball>();
                    Eigen::Affine2d ball_pose;
                    ball_pose.linear()      = Eigen::Rotation2Dd(world.ball.position.z()).toRotationMatrix();
                    ball_pose.translation() = world.ball.position.head<2>();
                    b->position             = (world.robotPose.inverse() * ball_pose).translation();
                    b->covariance           = Eigen::Matrix2d::Identity() * 0.00001;

                    emit(std::make_unique<std::vector<message::localisation::Ball>>(1, *b));
                    emit(std::move(b));
                }
            });


        // Emit exact position to NUsight
        on<Every<100, std::chrono::milliseconds>>().then("Emit True Robot Position", [this] {
            Eigen::Vector2d bearingVector = world.robotPose.rotation() * Eigen::Vector2d::UnitX();
            Eigen::Vector3d robotHeadingVector(bearingVector.x(), bearingVector.x(), 0.0);
            // TODO: Fix this.
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
        return g;
    }

    Eigen::Vector2d SoccerSimulator::getPath(SoccerSimulator::Config::Motion::Path p) {
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
        return Eigen::Vector2d(wave1, wave2);
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

        Eigen::Vector3d goal_opp_r(fd->goalpost_opp_r.x(), fd->goalpost_opp_r.y(), 0.0);
        goalPosts.push_back(VirtualGoalPost(goal_opp_r, 1.1, Goal::Side::RIGHT, Goal::Team::OPPONENT));

        Eigen::Vector3d goal_opp_l(fd->goalpost_opp_l.x(), fd->goalpost_opp_l.y(), 0.0);
        goalPosts.push_back(VirtualGoalPost(goal_opp_l, 1.1, Goal::Side::LEFT, Goal::Team::OPPONENT));

        Eigen::Vector3d goal_own_r(fd->goalpost_own_r.x(), fd->goalpost_own_r.y(), 0.0);
        goalPosts.push_back(VirtualGoalPost(goal_own_r, 1.1, Goal::Side::RIGHT, Goal::Team::OWN));

        Eigen::Vector3d goal_own_l(fd->goalpost_own_l.x(), fd->goalpost_own_l.y(), 0.0);
        goalPosts.push_back(VirtualGoalPost(goal_own_l, 1.1, Goal::Side::LEFT, Goal::Team::OWN));
    }
}  // namespace support
}  // namespace module
