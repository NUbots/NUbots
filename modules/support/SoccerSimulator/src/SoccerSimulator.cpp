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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "SoccerSimulator.h"
#include <nuclear>
#include "utility/math/angle.h"
#include "utility/math/coordinates.h"
#include "utility/nubugger/NUhelpers.h"
#include "utility/localisation/transform.h"
#include "utility/motion/ForwardKinematics.h"
#include "messages/vision/VisionObjects.h"
#include "messages/support/Configuration.h"
#include "messages/localisation/FieldObject.h"
#include "messages/input/Sensors.h"
#include "messages/input/ServoID.h"
#include "messages/platform/darwin/DarwinSensors.h"
#include "messages/motion/WalkCommand.h"

namespace modules {
namespace support {

    using messages::input::Sensors;
    using messages::input::ServoID;
    using utility::math::angle::normalizeAngle;
    using utility::math::angle::vectorToBearing;
    using utility::math::angle::bearingToUnitVector;
    using utility::math::coordinates::cartesianToSpherical;
    using utility::motion::kinematics::calculateRobotToIMU;
    using utility::localisation::transform::SphericalRobotObservation;
    using utility::localisation::transform::WorldToRobotTransform;
    using utility::localisation::transform::RobotToWorldTransform;
    using utility::nubugger::graph;
    using messages::support::Configuration;
    using messages::support::FieldDescription;
    using messages::localisation::Mock;
    using messages::motion::WalkCommand;
    using messages::motion::KickCommand;
    using messages::motion::KickPlannerConfig;
    using messages::platform::darwin::DarwinSensors;
    using utility::math::matrix::Transform2D;

    

    double triangle_wave(double t, double period) {
        auto a = period; // / 2.0;
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
    double absolute_time() {
        auto now = NUClear::clock::now();
        auto ms_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
        double ms = static_cast<double>(ms_since_epoch - 1393322147502L);
        double t = ms / 1000.0;
        return t;
    }

    void SoccerSimulator::UpdateConfiguration(
        const messages::support::Configuration<SoccerSimulatorConfig>& config) {

        cfg_.localisation.simulate_ball = config["localisation"]["simulate_ball"].as<bool>();
        cfg_.localisation.simulate_self = config["localisation"]["simulate_self"].as<bool>();

        cfg_.simulate_goal_observations = config["vision"]["goal_observations"].as<bool>();
        cfg_.simulate_ball_observations = config["vision"]["ball_observations"].as<bool>();
        cfg_.observe_left_goal = config["vision"]["observe"]["left_goal"].as<bool>();
        cfg_.observe_right_goal = config["vision"]["observe"]["right_goal"].as<bool>();
        cfg_.distinguish_left_and_right_goals = config["vision"]["distinguish_left_and_right_goals"].as<bool>();

        cfg_.robot.motion_type = motionTypeFromString(config["robot"]["motion_type"].as<std::string>());
        cfg_.robot.path.period = config["robot"]["path"]["period"].as<float>();
        cfg_.robot.path.period = config["robot"]["path"]["x_amp"].as<float>();
        cfg_.robot.path.period = config["robot"]["path"]["y_amp"].as<float>();
        cfg_.robot.path.type = pathTypeFromString(config["robot"]["path"]["type"].as<std::string>());
      
        cfg_.ball.motion_type = motionTypeFromString(config["ball"]["motion_type"].as<std::string>());
        cfg_.ball.path.period = config["ball"]["path"]["period"].as<float>();
        cfg_.ball.path.period = config["ball"]["path"]["x_amp"].as<float>();
        cfg_.ball.path.period = config["ball"]["path"]["y_amp"].as<float>();
        cfg_.ball.path.type = pathTypeFromString(config["ball"]["path"]["type"].as<std::string>());
        
        cfg_.emit_robot_fieldobjects = config["nusight"]["emit_self"].as<bool>();
        cfg_.emit_ball_fieldobjects = config["nusight"]["emit_ball"].as<bool>();

    }

    SoccerSimulator::SoccerSimulator(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        world.robotPose = Transform2D({0,0,0});
        world.ballPose = Transform2D({0,0,0});

        on<Trigger<FieldDescription>>("FieldDescription Update", [this](const FieldDescription& desc) {
            field_description_ = std::make_shared<FieldDescription>(desc);
        });

        on<Trigger<Configuration<SoccerSimulatorConfig>>>(
            "SoccerSimulatorConfig Update",
            [this](const Configuration<SoccerSimulatorConfig>& config) {
            UpdateConfiguration(config);
        });

        on<Trigger<KickPlannerConfig>>("Get Kick Planner Config", [this](const KickPlannerConfig& cfg_){
            kick_cfg = cfg_;
        });

        on<Trigger<KickCommand>>("Queue KickCommand",[this](const KickCommand& k){
            kickQueue.push(k);
        });

        on<
            Trigger<Every<SIMULATION_UPDATE_FREQUENCY, Per<std::chrono::seconds>>>,
            With<Optional<WalkCommand>>
        >("Robot motion", [this](const time_t&,
                                 const std::shared_ptr<const WalkCommand>& walkCommand) {

            
            Transform2D oldRobotPose = world.robotPose;
            Transform2D oldBallPose = world.ballPose;
            Transform2D diff;
            
            switch (cfg_.robot.motion_type){
                case MotionType::NONE: 
                    world.robotVelocity = Transform2D({ 0, 0 ,0 });
                    break;

                case MotionType::PATH: 
                    
                    world.robotPose.xy() = getPath(cfg_.robot.path);

                    diff = world.robotPose - oldRobotPose;
                    //Face along direction of movement
                    world.robotPose.angle() = vectorToBearing(diff.xy());
                    world.robotVelocity = Transform2D({arma::norm(diff) * SIMULATION_UPDATE_FREQUENCY, 0, 0}); //Robot coordinates
                    break;

                case MotionType::MOTION:
                //Update based on walk engine
                    if(walkCommand){
                        world.robotVelocity = walkCommand->command; 
                    } else {
                        world.robotVelocity = {0,0,0};
                    }
                    world.robotPose += world.robotVelocity / SIMULATION_UPDATE_FREQUENCY;
                    break;
            }
            // Update ball position
            switch (cfg_.robot.motion_type){
                case MotionType::NONE: 
                    world.ballVelocity = { 0, 0 , 0};
                    break;

                case MotionType::PATH:
                    
                    world.ballPose.xy() = getPath(cfg_.ball.path);

                    diff = world.ballPose - oldBallPose;

                    world.ballVelocity = Transform2D({arma::norm(diff) * SIMULATION_UPDATE_FREQUENCY, 0, 0}); //Robot coordinates
                    break;

                case MotionType::MOTION:
                    if(!kickQueue.empty()){
                        //Get last queue
                        KickCommand lastKickCommand = kickQueue.back();
                        //Empty queue
                        std::queue<KickCommand>().swap(kickQueue);
                        //Check if kick worked:
                        arma::vec2 ballPosition = world.robotPose.worldToLocal(world.ballPose.xy());

                        if( ballPosition[0] < kick_cfg.MAX_BALL_DISTANCE &&
                            std::fabs(ballPosition[1]) < kick_cfg.KICK_CORRIDOR_WIDTH / 2){
                            world.ballPose.xy() = world.robotPose.localToWorld(lastKickCommand.direction.rows(0,1));
                        }
                    }
                    break;
            }
            emit(computeGyro(world.robotPose.angle() - oldRobotPose.angle()));
        });

        // Simulate Vision
        on<Trigger<Every<30, Per<std::chrono::seconds>>>,
            With<Raw<Sensors>>,
            Options<Sync<SoccerSimulator>>>("Vision Simulation", [this](const time_t&, const std::shared_ptr<Sensors>& sensors) {

            if (field_description_ == nullptr) {
                NUClear::log(__FILE__, __LINE__, ": field_description_ == nullptr");
                return;
            }

            if (cfg_.simulate_goal_observations) {
                auto goals = std::make_unique<std::vector<messages::vision::Goal>>();

                // Only observe goals that are in front of the robot
                arma::vec3 goal_l_pos = {0, 0, 0};
                arma::vec3 goal_r_pos = {0, 0, 0};
                goal_l_pos.rows(0, 1) = field_description_->goalpost_yl;
                goal_r_pos.rows(0, 1) = field_description_->goalpost_yr;
                if (world.robotPose.angle() < -M_PI * 0.5 || world.robotPose.angle() > M_PI * 0.5) {
                    goal_l_pos.rows(0, 1) = field_description_->goalpost_bl;
                    goal_r_pos.rows(0, 1) = field_description_->goalpost_br;
                }

                if (cfg_.observe_left_goal) {
                    messages::vision::Goal goal1;
                    messages::vision::VisionObject::Measurement g1_m;
                    g1_m.position = SphericalRobotObservation(world.robotPose.xy(), world.robotPose.angle(), goal_r_pos);
                    g1_m.error = arma::eye(3, 3) * 0.1;
                    goal1.measurements.push_back(g1_m);
                    goal1.measurements.push_back(g1_m);
                    goal1.side = messages::vision::Goal::Side::RIGHT;
                    if (cfg_.distinguish_left_and_right_goals) {
                        goal1.side = messages::vision::Goal::Side::RIGHT;
                    } else {
                        goal1.side = messages::vision::Goal::Side::UNKNOWN;
                    }
                    goal1.sensors = sensors;
                    goals->push_back(goal1);
                }

                if (cfg_.observe_right_goal) {
                    messages::vision::Goal goal2;
                    messages::vision::VisionObject::Measurement g2_m;
                    g2_m.position = SphericalRobotObservation(world.robotPose.xy(), world.robotPose.angle(), goal_l_pos);
                    g2_m.error = arma::eye(3, 3) * 0.1;
                    goal2.measurements.push_back(g2_m);
                    goal2.measurements.push_back(g2_m);
                    if (cfg_.distinguish_left_and_right_goals) {
                        goal2.side = messages::vision::Goal::Side::LEFT;
                    } else {
                        goal2.side = messages::vision::Goal::Side::UNKNOWN;
                    }
                    goal2.sensors = sensors;
                    goals->push_back(goal2);
                }

                if (goals->size() > 0)
                    emit(std::move(goals));

            }else if(cfg_.localisation.simulate_self){
                //Emit current self exactly
                auto r = std::make_unique<std::vector<messages::localisation::Self>>();
                r->push_back(messages::localisation::Self());
                r->back().position = world.robotPose.xy();
                r->back().heading = bearingToUnitVector(world.robotPose.angle());
                r->back().velocity = world.robotVelocity.rows(0,1);
                r->back().position_cov = 0.00001 * arma::eye(2,2);
                emit(std::move(r));
            }
        

            if (cfg_.simulate_ball_observations) {
                auto ball_vec = std::make_unique<std::vector<messages::vision::Ball>>();

                messages::vision::Ball ball;
                messages::vision::VisionObject::Measurement b_m;
                arma::vec3 ball_pos_3d = {0, 0, 0};
                ball_pos_3d.rows(0, 1) = world.ballPose.xy();
                b_m.position = SphericalRobotObservation(world.robotPose.xy(), world.robotPose.angle(), ball_pos_3d);
                b_m.error = arma::eye(3, 3) * 0.1;
                ball.measurements.push_back(b_m);
                ball.sensors = sensors;
                ball_vec->push_back(ball);

                emit(std::move(ball_vec));

            } else if(cfg_.localisation.simulate_ball){
                //Emit current ball exactly                
                auto b = std::make_unique<std::vector<messages::localisation::Ball>>();
                b->push_back(messages::localisation::Ball());
                b->back().position = world.ballPose.xy();
                b->back().velocity = world.ballVelocity.rows(0,1);
                b->back().position_cov = 0.00001 * arma::eye(2,2);
                emit(std::move(b));

            }

        });

        // Emit robot to NUbugger
        on<Trigger<Every<100, std::chrono::milliseconds>>,
           With<Mock<std::vector<messages::localisation::Self>>>,
           Options<Sync<SoccerSimulator>>>("NUbugger Output Self",
            [this](const time_t&,
                   const Mock<std::vector<messages::localisation::Self>>& mock_robots) {

            auto& robots = mock_robots.data;

            emit(graph("Actual robot pose", world.robotPose[0], world.robotPose[1], world.robotPose.angle()));
            // emit(graph("Actual robot heading", world.robotPose.angle()[0], world.robotPose.angle()[1]));
            emit(graph("Actual robot velocity", world.robotVelocity[0], world.robotVelocity[1], world.robotVelocity[2]));

            if (robots.size() >= 1) {
                emit(graph("Estimated robot position", robots[0].position[0], robots[0].position[1]));
                emit(graph("Estimated robot heading", robots[0].heading[0], robots[0].heading[1]));
            }

            // Robot message
            if (!cfg_.emit_robot_fieldobjects)
                return;

            auto robots_msg = std::make_unique<std::vector<messages::localisation::Self>>();
            for (auto& model : robots) {
                robots_msg->push_back(model);
            }

            messages::localisation::Self self_marker;
            self_marker.position = world.robotPose.xy();
            self_marker.heading = bearingToUnitVector(world.robotPose.angle());
            self_marker.position_cov = arma::eye(2,2) * 0.1;
            robots_msg->push_back(self_marker);

            emit(std::move(robots_msg));
        });

        // Emit ball to Nubugger
        on<Trigger<Every<100, std::chrono::milliseconds>>,
           With<Mock<messages::localisation::Ball>>,
           With<Mock<std::vector<messages::localisation::Self>>>,
           Options<Sync<SoccerSimulator>>>("NUbugger Output Ball",
            [this](const time_t&,
                   const Mock<messages::localisation::Ball>& mock_ball,
                   const Mock<std::vector<messages::localisation::Self>>& mock_robots) {
            auto& ball = mock_ball.data;
            auto& robots = mock_robots.data;

            if (robots.empty())
                return;

            arma::vec2 robot_ball_pos = RobotToWorldTransform(
                robots[0].position, robots[0].heading, ball.position);
            arma::vec2 ball_pos = RobotToWorldTransform(
                world.robotPose, world.robotPose.angle(), ball.position);
            emit(graph("Estimated ball position", ball_pos[0], ball_pos[1]));
            // emit(graph("Estimated ball velocity", state[2], state[3]));
            emit(graph("Actual ball position", world.ballPose[0], world.ballPose[1], world.ballPose[2]));
            emit(graph("Actual ball velocity", world.ballVelocity[0], world.ballVelocity[1], world.ballPose[2]));

            // Ball message
            if (!cfg_.emit_ball_fieldobjects)
                return;

            auto balls_msg = std::make_unique<std::vector<messages::localisation::Ball>>();

            // True ball position:
            messages::localisation::Ball ball_marker;
            ball_marker.position = world.ballPose.xy();
            ball_marker.velocity = world.ballVelocity.rows(0,1);
            ball_marker.position_cov = arma::eye(2,2) * 0.1;
            ball_marker.world_space = true;
            balls_msg->push_back(ball_marker);

            messages::localisation::Ball ball_model;
            ball_model.position = ball.position;
            ball_model.velocity = ball.velocity;
            ball_model.position_cov = ball.position_cov;
            ball_model.world_space = true;
            balls_msg->push_back(ball_model);

            // messages::localisation::Ball robot_ball;
            // robot_ball.position = robot_ball_pos;
            // robot_ball.velocity = ballVelocity;
            // robot_ball.position_cov = arma::eye(2,2) * 0.1;
            // robot_ball.world_space = true;
            // balls_msg->push_back(robot_ball);

            emit(std::move(balls_msg));
        });
    }

    std::unique_ptr<DarwinSensors::Gyroscope> SoccerSimulator::computeGyro(float dHeading){
        auto g = std::make_unique<DarwinSensors::Gyroscope>();
        g->x = 0;
        g->y = 0;
        g->z = dHeading;
        return std::move(g);
    }

    arma::vec2 SoccerSimulator::getPath(SoccerSimulator::Config::Motion::Path p){
        auto t = absolute_time();
        float wave1,wave2;
        switch(p.type){
            case PathType::SIN:
                wave1 = p.x_amp * sine_wave(t, p.period);
                wave2 = p.y_amp * sine_wave(t + (p.period / 4.0), p.period);                      
            case PathType::TRIANGLE:
                wave1 = p.x_amp * triangle_wave(t, p.period);
                wave2 = p.y_amp * triangle_wave(t + (p.period / 4.0), p.period);
        }
        return arma::vec2({wave1,wave2});
    }

}
}

