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

#include "KinematicsDebug.h"

#include "messages/support/Configuration.h"
#include "messages/input/ServoID.h"
#include "messages/input/Sensors.h"
#include "messages/motion/ServoTarget.h"
#include "utility/motion/InverseKinematics.h"
#include "utility/motion/ForwardKinematics.h"
#include "utility/math/matrix.h"
#include "utility/motion/RobotModels.h"
#include <cstdlib>


namespace modules {
    namespace debug {
            using messages::support::Configuration;
            using messages::motion::ServoTarget;
            using messages::input::ServoID;
            using messages::input::Sensors;
            using utility::motion::kinematics::calculateLegJoints;
            using utility::motion::kinematics::calculatePosition;
            using utility::motion::kinematics::Side;
            using utility::math::matrix::xRotationMatrix;
            using utility::math::matrix::yRotationMatrix;
            using utility::math::matrix::zRotationMatrix;
            using utility::motion::kinematics::DarwinModel;
            using utility::motion::kinematics::calculateHeadJoints;

            KinematicsDebug::KinematicsDebug(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
                    on< Trigger<Configuration<InverseKinematicsRequest>> >([this](const Configuration<InverseKinematicsRequest>& request) {
                        return;
                        arma::mat44 target = yRotationMatrix(request.config["yAngle"].as<double>(), 4);
                        target *= xRotationMatrix(request.config["xAngle"].as<double>(), 4);
                        target *= zRotationMatrix(request.config["zAngle"].as<double>(), 4);

                        // translation
                        target(0,3) = request.config["x"].as<double>(); // down/up
                        target(1,3) = request.config["y"].as<double>(); // left/right
                        target(2,3) = request.config["z"].as<double>(); // front/back

                        bool left = request.config["left"].as<bool>();
                        bool right = request.config["right"].as<bool>();

                        auto waypoints = std::make_unique<std::vector<ServoTarget> >();

                        if (left) {
                            std::vector<std::pair<ServoID, float> > legJoints = calculateLegJoints<DarwinModel>(target, Side::LEFT);
                            for (auto& legJoint : legJoints) {
                                ServoTarget waypoint;

                                ServoID servoID;
                                float position;

                                std::tie(servoID, position) = legJoint;

                                waypoint.time = NUClear::clock::now() + std::chrono::seconds(2);
                                waypoint.id = servoID;
                                waypoint.position = position;
                                waypoint.gain = 20;

                                waypoints->push_back(waypoint);
                            }
                        }

                        if (right) {
                            std::vector<std::pair<ServoID, float> > legJoints = calculateLegJoints<DarwinModel>(target, Side::RIGHT);
                            for (auto& legJoint : legJoints) {
                                ServoTarget waypoint;

                                ServoID servoID;
                                float position;

                                std::tie(servoID, position) = legJoint;

                                waypoint.time = NUClear::clock::now() + std::chrono::seconds(2);
                                waypoint.id = servoID;
                                waypoint.position = position;
                                waypoint.gain = 20;

                                waypoints->push_back(waypoint);
                            }
                        }

                        emit(std::move(waypoints));
                    });

                    on< Trigger<Configuration<LegKinematicsNULLTest>> >([this](const Configuration<LegKinematicsNULLTest>& request) {
                        int iterations = 1;
                        int numberOfFails = 0;
                        float ERROR_THRESHOLD = request.config["ERROR_THRESHOLD"].as<float>();

                        if(request.config["RANDOMIZE"].as<bool>()){
                            iterations = request.config["RANDOM_ITERATIONS"].as<int>();
                        }

                        for(int i = 0; i<iterations; i++){
                            arma::mat44 ikRequest = yRotationMatrix(request.config["yAngle"].as<double>(), 4);
                            ikRequest *= xRotationMatrix(request.config["xAngle"].as<double>(), 4);
                            ikRequest *= zRotationMatrix(request.config["zAngle"].as<double>(), 4);

                            // translation
                            ikRequest(0,3) = request.config["x"].as<double>();
                            ikRequest(1,3) = request.config["y"].as<double>();
                            ikRequest(2,3) = request.config["z"].as<double>();

                            if(request.config["RANDOMIZE"].as<bool>()){
                                ikRequest = yRotationMatrix(2*M_PI*rand()/static_cast<double>(RAND_MAX), 4);
                                ikRequest *= xRotationMatrix(2*M_PI*rand()/static_cast<double>(RAND_MAX), 4);
                                ikRequest *= zRotationMatrix(2*M_PI*rand()/static_cast<double>(RAND_MAX), 4);
                                ikRequest(0,3) = 0.1 * rand()/static_cast<double>(RAND_MAX);
                                ikRequest(1,3) = 0.1 * rand()/static_cast<double>(RAND_MAX);
                                ikRequest(2,3) = 0.1 * rand()/static_cast<double>(RAND_MAX);
                            }

                            bool left = request.config["left"].as<bool>();
                            bool right = request.config["right"].as<bool>();

                            std::unique_ptr<Sensors> sensors = std::make_unique<Sensors>();
                            sensors->servos = std::vector<Sensors::Servo>(20);

                            if (left) {
                                std::vector<std::pair<ServoID, float> > legJoints = calculateLegJoints<DarwinModel>(ikRequest, Side::LEFT);
                                for (auto& legJoint : legJoints) {
                                    ServoID servoID;
                                    float position;

                                    std::tie(servoID, position) = legJoint;

                                    sensors->servos[static_cast<int>(servoID)].presentPosition = position;
                                }
                            }

                            if (right) {
                                std::vector<std::pair<ServoID, float> > legJoints = calculateLegJoints<DarwinModel>(ikRequest, Side::RIGHT);
                                for (auto& legJoint : legJoints) {
                                    ServoID servoID;
                                    float position;

                                    std::tie(servoID, position) = legJoint;

                                    sensors->servos[static_cast<int>(servoID)].presentPosition = position;
                                }
                            }
                            std::cout<< "KinematicsNULLTest -calculating forward kinematics." <<std::endl;
                            arma::mat44 lFootPosition = calculatePosition<DarwinModel>(*sensors, ServoID::L_ANKLE_ROLL)[ServoID::L_ANKLE_ROLL];
                            arma::mat44 rFootPosition = calculatePosition<DarwinModel>(*sensors, ServoID::R_ANKLE_ROLL)[ServoID::R_ANKLE_ROLL];
                            NUClear::log<NUClear::DEBUG>("Forward Kinematics predicts left foot: \n",lFootPosition);
                            NUClear::log<NUClear::DEBUG>("Forward Kinematics predicts right foot: \n",rFootPosition);
                            std::cout << "Compared to request: \n" << ikRequest << std::endl;

                            float lmax_error = 0;
                            float rmax_error = 0;
                            for(int i = 0; i <16 ; i++){
                                float lerror = std::abs(lFootPosition(i%4, i/4) - ikRequest(i%4, i/4));
                                float rerror = std::abs(rFootPosition(i%4, i/4) - ikRequest(i%4, i/4));
                                if (lerror>lmax_error) {
                                    lmax_error = lerror;
                                }
                                if (rerror>rmax_error) {
                                    rmax_error = rerror;
                                }
                            }
                            std::cout<< "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" <<std::endl;
                            std::cout<< (lmax_error < ERROR_THRESHOLD ? "LEFT IK TEST PASSED" : "\n\n\n!!!!!!!!!! LEFT IK TEST FAILED !!!!!!!!!!\n\n\n" ) << "     (max_error = " << lmax_error << ")"<<std::endl;
                            std::cout<< "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" <<std::endl;

                            std::cout<< "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" <<std::endl;
                            std::cout<< (rmax_error < ERROR_THRESHOLD ? "RIGHT IK TEST PASSED" : "\n\n\n!!!!!!!!!! RIGHT IK TEST FAILED !!!!!!!!!!\n\n\n" ) << "     (max_error = " << rmax_error << ")"<<std::endl;
                            std::cout<< "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" <<std::endl;

                            if(lmax_error >= ERROR_THRESHOLD or rmax_error >= ERROR_THRESHOLD){
                                numberOfFails++;
                            }
                            sensors->orientation = arma::eye(3,3);
                            emit(std::move(sensors));
                        }
                        std::cout<< "IK Leg NULL Test : "<< numberOfFails << " Total Failures " <<std::endl;

                    });

                    on< Trigger<Configuration<HeadKinematicsNULLTest>> >([this](const Configuration<HeadKinematicsNULLTest>& request) {
                        int iterations = 1;
                        int numberOfFails = 0;
                        float ERROR_THRESHOLD = request.config["ERROR_THRESHOLD"].as<float>();
                        float yaw = request.config["yaw"].as<float>();
                        float pitch = request.config["pitch"].as<float>();
                        bool RANDOMIZE = request.config["RANDOMIZE"].as<bool>();

                        arma::vec3 cameraVec = {cos(yaw)*cos(pitch), sin(yaw)*cos(pitch), -sin(pitch)};
                        if(RANDOMIZE){
                            iterations = request.config["RANDOM_ITERATIONS"].as<int>();
                        }

                        for(int i = 0; i<iterations; i++){
                            if(RANDOMIZE){
                                cameraVec[0] = rand()/static_cast<double>(RAND_MAX);
                                cameraVec[1] = rand()/static_cast<double>(RAND_MAX);
                                cameraVec[2] = rand()/static_cast<double>(RAND_MAX);
                                cameraVec *= 1/arma::norm(cameraVec,2);
                            }

                            std::vector< std::pair<messages::input::ServoID, float> > angles = calculateHeadJoints<DarwinModel>(cameraVec);
                            Sensors sensors;
                            sensors.servos = std::vector<Sensors::Servo>(20);

                            for (auto& angle : angles) {
                                    ServoID servoID;
                                    float position;

                                    std::tie(servoID, position) = angle;

                                    sensors.servos[static_cast<int>(servoID)].presentPosition = position;
                            }

                            arma::mat44 fKin = calculatePosition<DarwinModel>(sensors, ServoID::HEAD_PITCH)[ServoID::HEAD_PITCH];

                            float max_error = 0;
                            for(int i = 0; i < 3 ; i++){
                                float error = std::abs(fKin(i, 0) - cameraVec[i]);
                                if (error>max_error) {
                                    max_error = error;
                                }
                            }
                            if(max_error >= ERROR_THRESHOLD){
                                    numberOfFails++;
                            }

                            NUClear::log<NUClear::DEBUG>("++++++++++++++++++++++++++++++++++++++++++++++++++");
                            NUClear::log<NUClear::DEBUG>("Request = \n", cameraVec);
                            NUClear::log<NUClear::DEBUG>("Angles = \n", angles[0].second, angles[1].second);
                            NUClear::log<NUClear::DEBUG>("Final FKin = \n", fKin);
                            NUClear::log<NUClear::DEBUG>(max_error >= ERROR_THRESHOLD ?  "FAIL" : "PASS", "Max Error =",max_error, ", ERROR_THRESHOLD", ERROR_THRESHOLD);
                            NUClear::log<NUClear::DEBUG>("++++++++++++++++++++++++++++++++++++++++++++++++++");

                        }
                        std::cout<< "IK Head NULL Test : "<< numberOfFails << " Total Failures out of " << iterations <<std::endl;


                    });
            }
    } // debug
} // modules
