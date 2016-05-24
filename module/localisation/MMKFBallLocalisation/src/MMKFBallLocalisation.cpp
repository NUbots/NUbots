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

#include "MMKFBallLocalisation.h"

#include <nuclear>

#include "utility/math/angle.h"
#include "utility/math/coordinates.h"
#include "utility/nubugger/NUhelpers.h"
#include "message/input/Sensors.h"
#include "message/vision/VisionObjects.h"
#include "message/support/Configuration.h"
#include "message/localisation/FieldObject.h"
#include "BallModel.h"
#include "utility/localisation/transform.h"
#include "utility/nubugger/NUhelpers.h"

using message::localisation::Self;
using utility::nubugger::graph;
using message::input::Sensors;
using message::support::Configuration;
// using message::localisation::FakeOdometry;
using message::localisation::Ball;
using utility::nubugger::drawArrow;


namespace module {
namespace localisation {

    double time_seconds() {
            auto now = NUClear::clock::now();
            auto ms_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
            double ms = static_cast<double>(ms_since_epoch - 1393322147502L);
            return ms / 1000.0;
    }

    MMKFBallLocalisation::MMKFBallLocalisation(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment)) {
        
        //INITIALISE VARIABLES
        ballFilters.push_back(utility::math::filter::UKF<mmball::BallModel>(
                                {3, 2, 0, 0}, // mean
                                arma::eye(mmball::BallModel::size, 
                                          mmball::BallModel::size) * 2., // cov
                                0.1 // alpha
                              )); 
        lastUpdateTime = time_seconds();
        
        //init the odometry variables
        odometryUpdate = arma::zeros(2);
        odometryNoiseMatrix = arma::zeros(mmball::BallModel::size,mmball::BallModel::size);
        
        
        
        //CONFIGURATION
        on<Configuration>("MMKFBallLocalisationEngine.yaml").then([this](const Configuration& config) {
            //check whether to use NUbugger
            if (config["EmitBallFieldobjects"].as<bool>()) {
                emit_data_handle.enable();
            } else {
                emit_data_handle.disable();
            }
            
            //apply the ball drag coefficient to all models
            for (size_t i = 0; i < ballFilters.size(); ++i) {
                ballFilters[i].model.ballDragCoefficient = config["BallDragCoefficient"].as<double>();
            }
            
            sphericalMinDist = config["MinimumSphericalDistanceMeasurement"].as<double>();
            numFilters = config["NumberOfHypotheses"].as<size_t>();
            
            //load our various process noise factors
            processNoiseModels = config["ProcessNoise"].as<std::vector<arma::mat>>();     
        });



        //NUBUGGER/FINAL BALL EMIT OUTPUT
       emit_data_handle = on<Every<30, Per<std::chrono::seconds>,
           With<Sensors>,
           With<std::vector<Self>>,
           Sync<MMKFBallLocalisation>>().then("NUbugger Output", [this](const Sensors& sensors, const std::vector<Self>& robots) {

            arma::vec model_state = ballFilters[0].get();
            arma::mat model_cov = ballFilters[0].getCovariance();

            arma::mat22 imu_to_robot = sensors.robotToIMU.t();
            arma::vec2 robot_space_ball_pos = imu_to_robot * model_state.rows(0, 1);
            arma::vec2 robot_space_ball_vel = imu_to_robot * model_state.rows(2, 3);

            message::localisation::Ball ball;
            ball.position = robot_space_ball_pos;
            ball.velocity = robot_space_ball_vel + robots[0].velocity;
            ball.position_cov = model_cov.submat(0,0,1,1);
            ball.world_space = false;

            ball.last_measurement_time = lastUpdateTime;

            auto ball_msg = std::make_unique<Ball>(ball);
            auto ball_vec_msg = std::make_unique<std::vector<Ball>>();
            ball_vec_msg->push_back(ball);
            emit(std::move(ball_msg));
            emit(std::move(ball_vec_msg));

            arma::vec3 worldSpaceBallPos = arma::zeros(3);
            worldSpaceBallPos.rows(0,1) = utility::localisation::transform::RobotToWorldTransform(robots[0].position, robots[0].heading, robot_space_ball_pos);
            arma::vec3 worldSpaceBallVel = arma::zeros(3);
            worldSpaceBallVel.rows(0,1) = utility::localisation::transform::RobotToWorldTransform(arma::zeros(2), robots[0].heading, robot_space_ball_vel);

            emit(drawArrow("ballvel", worldSpaceBallPos, worldSpaceBallVel, arma::norm(worldSpaceBallVel)));

            emit(graph("Localisation Ball", model_state(0), model_state(1)));
            emit(graph("Localisation Ball Velocity", model_state(2), model_state(3)));
        });
        //Disable until first data
        emit_data_handle.disable();
        
        
        
        //TIME UPDATE / BALL OUTPUT
        on<Every<30, Per<std::chrono::seconds>>, Sync<MMKFBallLocalisation>>().then("MMKFBallLocalisation Time", [this] {
            double seconds = time_seconds();
            
            //make copies of the existing filters to apply different noise models
            ballFilters.resize(ballFilters.size()*processNoiseModels.size());
            for (size_t i = 0; i < ballFilters.size()/processNoiseModels.size(); ++i) {
                for (size_t j = 0; j < processNoiseModels.size(); ++j) {
                    size_t tmpInd = i+j*ballFilters.size()/processNoiseModels.size()
                    ballFilters[tmpInd] = ballFilters[i];
                    ballFilters[tmpInd].model.processNoiseMatrix = processNoiseModels[j] + 
                                                                   odometryNoiseMatrix;
                }
            }
            
            //update the filters according to different models
            for (size_t i = 0; i < ballFilters.size(); ++i) {
                ballFilters[i].timeUpdate(seconds-lastUpdateTime,odometryUpdate);
            }
            lastUpdateTime= seconds;
            odometryUpdate.fill(0);
            odometryNoiseMatrix.fill(0);
        });
        
        
        
        //ODOMETRY UPDATE
        
        
        
        
        //OBSERVATION UPDATE
        on<Trigger<std::vector<message::vision::Ball>>,
             Sync<MMKFBallLocalisation>
             >().then("MMKFBallLocalisation Obs",
                [this](const std::vector<message::vision::Ball>& balls) {
            
            //apply all KFs
            std::vector<std::pair<double,size_t>> qualities(ballFilters[i].size());
            for (size_t i = 0; i < ballFilters[i].size(); ++i) {
                double quality = 1.0;
                
                //apply all measurements to each KF
                for (auto& measurement : observed_object.measurements) {
                    // Spherical from ground:
                    auto currentState = ball_filter_.get();

                    double ballAngle = 0;
                    if (0 != currentState(1) || 0 != currentState(0)) {
                        ballAngle = std::atan2(currentState(1), currentState(0));
                    }

                    arma::vec3 measuredPosCartesian = sphericalToCartesian(measurement.position);
                    arma::vec2 cartesianImuObservation2d = observed_object.sensors->robotToIMU * measuredPosCartesian.rows(0,1);
                    
                    if (measurement.error(0,0) > sphericalMinDist) {
                        double ballAngle = 0;
                        if (0 != currentState(1) || 0 != currentState(0)) {
                            ballAngle = std::atan2(cartesianImuObservation2d(1), cartesianImuObservation2d(0));
                        }
                        sphericalImuObservation(1) = ballAngle;
                        //we need to decide between cartesian and spherical updates
                        quality *= ballFilters[i].measurementUpdate(
                                        sphericalImuObservation, 
                                        measurement.error
                                        );
                    } else {
                        //XXX: this conversion of uncertainties is a massive hack
                        double cba = math::cos(ballAngle);
                        double sba = math::sin(ballAngle);
                        quality *= ballFilters[i].measurementUpdate(
                                        cartesianImuObservation2d, 
                                        arma::eye(
                                            arma::vec({cba*measurement.error(0,0) +
                                                       sba*measurement.error(1,1),
                                                       sba*measurement.error(0,0) +
                                                       cba*measurement.error(1,1)})
                                            )
                                        );
                    }
                    
                }
                
                qualities[i] = std::make_pair<double,size_t>(quality,i);
            }  
            
            //XXX: re-order the KFs according to goodness
            if (ballFilters.size() > 1) {
                ;
            }
        });
}
