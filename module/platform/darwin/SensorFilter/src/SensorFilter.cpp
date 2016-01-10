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

#include "SensorFilter.h"

#include "message/platform/darwin/DarwinSensors.h"
#include "message/input/CameraParameters.h"
#include "message/support/Configuration.h"

#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/geometry/UnitQuaternion.h"
#include "utility/nubugger/NUhelpers.h"
#include "utility/motion/ForwardKinematics.h"

namespace module {
    namespace platform {
        namespace darwin {

            using message::support::Configuration;
            using utility::nubugger::drawArrow;
            using utility::nubugger::drawSphere;
            using message::platform::darwin::DarwinSensors;
            using message::platform::darwin::ButtonLeftDown;
            using message::platform::darwin::ButtonLeftUp;
            using message::platform::darwin::ButtonMiddleDown;
            using message::platform::darwin::ButtonMiddleUp;
            using message::input::Sensors;
            using message::input::CameraParameters;
            using message::input::ServoID;
            using utility::nubugger::graph;
            using utility::motion::kinematics::calculateAllPositions;
            using utility::motion::kinematics::DarwinModel;
            using utility::motion::kinematics::calculateCentreOfMass;
            using utility::motion::kinematics::Side;
            using utility::motion::kinematics::calculateRobotToIMU;
            using utility::math::matrix::Transform3D;
            using utility::math::matrix::Rotation3D;
            using utility::math::geometry::UnitQuaternion;

            std::string makeErrorString(const std::string& src, uint errorCode) {
                std::stringstream s;

                s << "Error on ";
                s << src;
                s << ":";

                if(errorCode & DarwinSensors::Error::INPUT_VOLTAGE) {
                    s << " Input Voltage ";
                }
                if(errorCode & DarwinSensors::Error::ANGLE_LIMIT) {
                    s << " Angle Limit ";
                }
                if(errorCode & DarwinSensors::Error::OVERHEATING) {
                    s << " Overheating ";
                }
                if(errorCode & DarwinSensors::Error::OVERLOAD) {
                    s << " Overloaded ";
                }
                if(errorCode & DarwinSensors::Error::INSTRUCTION) {
                    s << " Bad Instruction ";
                }
                if(errorCode & DarwinSensors::Error::CORRUPT_DATA) {
                    s << " Corrupt Data ";
                }
                if(errorCode & DarwinSensors::Error::TIMEOUT) {
                    s << " Timeout ";
                }

                return s.str();
            }

            SensorFilter::SensorFilter(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment))
            // intialize orientation filter to measured values when standing
            , orientationFilter(arma::vec({0, 0, 0, -9.6525e-01, -2.4957e-02, 1.8088e-01, 1.8696e-01})) {

                on<Configuration>("DarwinSensorFilter.yaml").then([this](const Configuration& config){
                    DEFAULT_NOISE_GAIN = config["default_noise_gain"].as<double>();
                    HIGH_NOISE_THRESHOLD = config["high_noise_threshold"].as<double>();
                    HIGH_NOISE_GAIN = config["high_noise_gain"].as<double>();
                    LOW_NOISE_THRESHOLD = config["low_noise_threshold"].as<double>();
                    DEBOUNCE_THRESHOLD = config["debounce_threshold"].as<int>();

                    SUPPORT_FOOT_FSR_THRESHOLD = config["support_foot_fsr_threshold"].as<double>();
                    REQUIRED_NUMBER_OF_FSRS = config["required_number_of_fsrs"].as<int>();

                    orientationFilter.model.processNoiseDiagonal = arma::ones(orientationFilter.model.size);
                    orientationFilter.model.processNoiseDiagonal.rows(orientationFilter.model.QW,orientationFilter.model.QZ) *= config["imu_position_process_noise"].as<double>();
                    orientationFilter.model.processNoiseDiagonal.rows(orientationFilter.model.VX,orientationFilter.model.VZ) *= config["imu_velocity_process_noise"].as<double>();
                    // NUClear::log("ProcessNoise Set: \n", orientationFilter.model.processNoiseDiagonal.t());

                    MEASUREMENT_NOISE_ACCELEROMETER = arma::eye(3,3) * config["measurement_noise_accelerometer"].as<double>();
                    MEASUREMENT_NOISE_GYROSCOPE = arma::eye(3,3) * config["measurement_noise_gyroscope"].as<double>();
                    MEASUREMENT_NOISE_FOOT_UP = arma::eye(3,3) * config["measurement_noise_foot_up"].as<double>();
                    FOOT_UP_SAFE_ZONE = config["foot_up_safe_zone"].as<double>();

                    odometry_covariance_factor = config["odometry_covariance_factor"].as<double>();
                });

                on<Last<20, Trigger<DarwinSensors>>>().then([this](const std::list<std::shared_ptr<const DarwinSensors>>& sensors) {

                    int leftCount = 0;
                    int middleCount = 0;

                    // If we have any downs in the last 20 frames then we are button pushed
                    for (const auto& s : sensors) {
                        if(s->buttons.left && !s->cm730ErrorFlags) {
                            ++leftCount;
                        }
                        if(s->buttons.middle && !s->cm730ErrorFlags) {
                            ++middleCount;
                        }
                    }

                    bool newLeftDown = leftCount > DEBOUNCE_THRESHOLD;
                    bool newMiddleDown = middleCount > DEBOUNCE_THRESHOLD;

                    if(newLeftDown != leftDown) {

                        leftDown = newLeftDown;

                        if(newLeftDown) {
                            std::cout << "Left Button Down" << std::endl;
                            emit(std::make_unique<ButtonLeftDown>());
                        }
                        else {
                            std::cout << "Left Button Up" << std::endl;
                            emit(std::make_unique<ButtonLeftUp>());
                        }
                    }
                    if(newMiddleDown != middleDown) {

                        middleDown = newMiddleDown;

                        if(newMiddleDown) {
                            std::cout << "Middle Button Down" << std::endl;
                            emit(std::make_unique<ButtonMiddleDown>());
                        }
                        else {
                            std::cout << "Middle Button Up" << std::endl;
                            emit(std::make_unique<ButtonMiddleUp>());
                        }
                    }
                });

                on< Trigger<DarwinSensors>
                  , Optional<With<Sensors>>
                  , Single
                  , Priority::HIGH>().then(
                            "Main Sensors Loop",
                            [this](const DarwinSensors& input,
                                   std::shared_ptr<const Sensors> previousSensors) {

                    auto sensors = std::make_unique<Sensors>();

                    // Set our timestamp to when the data was read
                    sensors->timestamp = input.timestamp;
                    // Set the min and max voltage
                    float flatVoltage = 10.7;
                    float chargedVoltage = 12.9;
                    // Set our voltage and battery
                    sensors->voltage = input.voltage;
                    sensors->battery = std::max(0.0f, (input.voltage - flatVoltage) / (chargedVoltage - flatVoltage));
                    // This checks for an error on the CM730 and reports it
                    if (input.cm730ErrorFlags != DarwinSensors::Error::OK) {
                        NUClear::log<NUClear::WARN>(makeErrorString("CM730", input.cm730ErrorFlags));
                    }

                    // Output errors on the FSRs
                    if (input.fsr.left.errorFlags != DarwinSensors::Error::OK) {
                        NUClear::log<NUClear::WARN>(makeErrorString("Left FSR", input.fsr.left.errorFlags));
                    }

                    if (input.fsr.right.errorFlags != DarwinSensors::Error::OK) {
                        NUClear::log<NUClear::WARN>(makeErrorString("Right FSR", input.fsr.right.errorFlags));
                    }

                    // Read through all of our sensors
                    for(uint i = 0; i < 20; ++i) {
                        auto& original = input.servo[i];
                        auto& error = original.errorFlags;

                        // Check for an error on the servo and report it
                        while(error != DarwinSensors::Error::OK) {
                            std::stringstream s;
                            s << "Error on Servo " << (i + 1) << " (" << message::input::stringFromId(ServoID(i)) << "):";

                            if(error & DarwinSensors::Error::INPUT_VOLTAGE) {
                                s << " Input Voltage - " << original.voltage;
                            }
                            if(error & DarwinSensors::Error::ANGLE_LIMIT) {
                                s << " Angle Limit - " << original.presentPosition;
                            }
                            if(error & DarwinSensors::Error::OVERHEATING) {
                                s << " Overheating - " << original.temperature;
                            }
                            if(error & DarwinSensors::Error::OVERLOAD) {
                                s << " Overloaded - " << original.load;
                            }
                            if(error & DarwinSensors::Error::INSTRUCTION) {
                                s << " Bad Instruction ";
                            }
                            if(error & DarwinSensors::Error::CORRUPT_DATA) {
                                s << " Corrupt Data ";
                                break;
                            }
                            if(error & DarwinSensors::Error::TIMEOUT) {
                                s << " Timeout ";
                            }

                            NUClear::log<NUClear::WARN>(s.str());
                            break;
                        }

                        if(previousSensors && error != DarwinSensors::Error::OK) {
                            // Add the sensor values to the system properly
                            sensors->servos.push_back({
                                original.errorFlags,
                                static_cast<ServoID>(i),
                                original.torqueEnabled,
                                original.pGain,
                                original.iGain,
                                original.dGain,
                                original.goalPosition,
                                original.movingSpeed,
                                previousSensors->servos[i].presentPosition,
                                previousSensors->servos[i].presentVelocity,
                                previousSensors->servos[i].load,
                                previousSensors->servos[i].voltage,
                                previousSensors->servos[i].temperature
                            });
                        }
                        else {
                            // Add the sensor values to the system properly
                            sensors->servos.push_back({
                                original.errorFlags,
                                static_cast<ServoID>(i),
                                original.torqueEnabled,
                                original.pGain,
                                original.iGain,
                                original.dGain,
                                original.goalPosition,
                                original.movingSpeed,
                                original.presentPosition,
                                original.presentSpeed,
                                original.load,
                                original.voltage,
                                float(original.temperature)
                            });
                        }

                    }

                    // If we have a previous sensors and our cm730 has errors then reuse our last sensor value
                    if(previousSensors && (input.cm730ErrorFlags)) {
                        sensors->accelerometer = previousSensors->accelerometer;
                    }
                    else {
                        sensors->accelerometer = {-input.accelerometer.y, input.accelerometer.x, -input.accelerometer.z};
                    }

                    // If we have a previous sensors and our cm730 has errors then reuse our last sensor value
                    if(previousSensors && (input.cm730ErrorFlags || arma::norm(arma::vec({input.gyroscope.x, input.gyroscope.y, input.gyroscope.z}), 2) > 4 * M_PI)) {
                        NUClear::log("Bad gyroscope value", arma::norm(arma::vec({input.gyroscope.x, input.gyroscope.y, input.gyroscope.z}), 2));
                        sensors->gyroscope = previousSensors->gyroscope;
                    }
                    else {
                        sensors->gyroscope = {-input.gyroscope.x, -input.gyroscope.y, input.gyroscope.z};
                    }
                    /************************************************
                     *               Buttons and LEDs               *
                     ************************************************/
                    sensors->buttons.push_back({ 0, input.buttons.left });
                    sensors->buttons.push_back({ 1, input.buttons.middle });
                    sensors->leds.push_back({ 0, uint32_t(input.ledPanel.led2 ? 0xFF0000 : 0) });
                    sensors->leds.push_back({ 1, uint32_t(input.ledPanel.led3 ? 0xFF0000 : 0) });
                    sensors->leds.push_back({ 2, uint32_t(input.ledPanel.led4 ? 0xFF0000 : 0) });
                    sensors->leds.push_back({ 3, uint32_t((input.headLED.r << 16) | (input.headLED.g << 8) | (input.headLED.b)) }); // Head
                    sensors->leds.push_back({ 4, uint32_t((input.eyeLED.r  << 16) | (input.eyeLED.g  << 8) | (input.eyeLED.b))  }); // Eye

                    /************************************************
                     *                  Kinematics                  *
                     ************************************************/
                    sensors->forwardKinematics = calculateAllPositions<DarwinModel>(*sensors);

                    /************************************************
                     *                 Orientation                  *
                     ************************************************/

                     /*! The orientation matrix is the map from world to robot coordinates, measured by the gyro.
                      It is the world coordinates in columns relative to the robot.
                     */

                    // Calculate our time offset from the last read
                    double deltaT = (input.timestamp - (previousSensors ? previousSensors->timestamp : input.timestamp)).count() / double(NUClear::clock::period::den);
                    orientationFilter.timeUpdate(deltaT);

                    orientationFilter.measurementUpdate(sensors->accelerometer, MEASUREMENT_NOISE_ACCELEROMETER, IMUModel::MeasurementType::ACCELEROMETER());
                    emit(graph("accelerometer", sensors->accelerometer[0], sensors->accelerometer[1], sensors->accelerometer[2]));
                    orientationFilter.measurementUpdate(sensors->gyroscope,     MEASUREMENT_NOISE_GYROSCOPE, IMUModel::MeasurementType::GYROSCOPE());
                    emit(graph("gyroscope", sensors->gyroscope[0], sensors->gyroscope[1], sensors->gyroscope[2]));

                    // If we assume the feet are flat on the ground, we can use forward kinematics to feed a measurement update to the orientation filter.
                    // if (std::abs(input.fsr.left.centreX) < FOOT_UP_SAFE_ZONE && std::abs(input.fsr.left.centreY) < FOOT_UP_SAFE_ZONE) {
                    //     auto footUp = sensors->forwardKinematics.find(ServoID::L_ANKLE_ROLL)->second.rotation().col(2);
                    //     orientationFilter.measurementUpdate(footUp, MEASUREMENT_NOISE_FOOT_UP, IMUModel::MeasurementType::UP());
                    // }
                    // if (std::abs(input.fsr.right.centreX) < FOOT_UP_SAFE_ZONE && std::abs(input.fsr.right.centreY) < FOOT_UP_SAFE_ZONE) {
                    //     auto footUp = sensors->forwardKinematics.find(ServoID::R_ANKLE_ROLL)->second.rotation().col(2);
                    //     orientationFilter.measurementUpdate(footUp, MEASUREMENT_NOISE_FOOT_UP, IMUModel::MeasurementType::UP());
                    // }

                    // Gives us the quaternion representation
                    arma::vec o = orientationFilter.get();
                    emit(graph("orientation quat", o[0], o[1], o[2], o[3]));
                    //Map from robot to world coordinates
                    sensors->orientation = Rotation3D(UnitQuaternion(o.rows(orientationFilter.model.QW, orientationFilter.model.QZ)));

                    // sensors->orientation.col(2) = -orientation.rows(0,2);
                    // sensors->orientation.col(0) = orientation.rows(3,5);
                    // sensors->orientation.col(1) = arma::cross(sensors->orientation.col(2), sensors->orientation.col(0));

                    sensors->robotToIMU = calculateRobotToIMU(sensors->orientation);

                    /************************************************
                     *                   Odometry                   *
                     ************************************************/

                    emit(graph("FSR Left Raw", input.fsr.left.fsr1, input.fsr.left.fsr2, input.fsr.left.fsr3, input.fsr.left.fsr4));
                    emit(graph("FSR Right Raw", input.fsr.right.fsr1, input.fsr.right.fsr2, input.fsr.right.fsr3, input.fsr.right.fsr4));
                    //Check support foot:
                    sensors->leftFootDown = false;
                    sensors->rightFootDown = false;

                    // int zeroSensorsLeft = (input.fsr.left.fsr1 == 0) + (input.fsr.left.fsr2 == 0) + (input.fsr.left.fsr3 == 0) + (input.fsr.left.fsr4 == 0);
                    // int zeroSensorsRight = (input.fsr.right.fsr1 == 0) + (input.fsr.right.fsr2 == 0) + (input.fsr.right.fsr3 == 0) + (input.fsr.right.fsr4 == 0);

                    // if(input.fsr.left.fsr1 + input.fsr.left.fsr2 + input.fsr.left.fsr3 + input.fsr.left.fsr4 > SUPPORT_FOOT_FSR_THRESHOLD && zeroSensorsLeft <= 4 - REQUIRED_NUMBER_OF_FSRS){
                    //     sensors->leftFootDown = true;
                    // }
                    // if(input.fsr.right.fsr1 + input.fsr.right.fsr2 + input.fsr.right.fsr3 + input.fsr.right.fsr4 > SUPPORT_FOOT_FSR_THRESHOLD && zeroSensorsRight <= 4 - REQUIRED_NUMBER_OF_FSRS){
                    //     sensors->rightFootDown = true;
                    // }

                    // if(!std::isnan(input.fsr.left.centreX) && !std::isnan(input.fsr.left.centreY)) {
                    //     // Left foot is on the ground?
                    //     sensors->leftFootDown = true;
                    //     sensors->leftFSRCenter = {input.fsr.left.centreX, input.fsr.left.centreY};
                    //     // log("bodyCentre", bodyCentre.t());
                    //     // log("bodyCentre", sensors->leftFSRCenter);
                    // }
                    // if(!std::isnan(input.fsr.right.centreX) && !std::isnan(input.fsr.right.centreY)) {
                    //     // Right foot is on the ground?
                    //     sensors->rightFootDown = true;
                    //     sensors->rightFSRCenter = {input.fsr.right.centreX, input.fsr.right.centreY};
                    // }


                    auto rightFootPose = sensors->forwardKinematics.find(ServoID::R_ANKLE_ROLL)->second;
                    auto leftFootPose = sensors->forwardKinematics.find(ServoID::L_ANKLE_ROLL)->second;
                    arma::vec3 torsoFromRightFoot = -rightFootPose.rotation().i() * rightFootPose.translation();
                    arma::vec3 torsoFromLeftFoot = -leftFootPose.rotation().i() * leftFootPose.translation();
                    // emit(graph("torsoFromRightFoot", torsoFromRightFoot));
                    // emit(graph("torsoFromLeftFoot", torsoFromLeftFoot));
                    if(torsoFromRightFoot(2) > torsoFromLeftFoot(2)){
                        sensors->rightFootDown = true;
                    } else if(torsoFromRightFoot(2) < torsoFromLeftFoot(2)){
                        sensors->leftFootDown = true;
                    } else {
                        sensors->leftFootDown = false;
                        sensors->rightFootDown = false;
                    }


                    // log("left", sensors->leftFSRCenter.t(), "right", sensors->rightFSRCenter.t());

                    // if(previousSensors && (!sensors->leftFootDown && !sensors->rightFootDown )) {
                    //     //std::cout << "No feet down!" << std::endl;
                    //     sensors->leftFootDown = previousSensors->leftFootDown;
                    //     sensors->rightFootDown = previousSensors->rightFootDown;
                    // }

                    // // Kinematics odometry
                    // Transform3D odometryRightFoot = arma::eye(4,4);
                    // Transform3D odometryLeftFoot = arma::eye(4,4);
                    // if(previousSensors){
                    //     //NOTE: calculateOdometryMatrix requires sensors->forwardKinematics to be calculated before calling
                    //     odometryLeftFoot = calculateOdometryMatrix(*sensors, *previousSensors, Side::LEFT);
                    //     odometryRightFoot = calculateOdometryMatrix(*sensors, *previousSensors, Side::RIGHT);
                    // }


                    // if(sensors->leftFootDown || sensors->rightFootDown){
                    //     sensors->odometry.submat(0,3,2,3) = (odometryLeftFoot.submat(0,3,2,3) * sensors->leftFootDown + odometryLeftFoot.submat(0,3,2,3) * sensors->rightFootDown)
                    //                                         / (sensors->leftFootDown + sensors->rightFootDown);
                    //     if(sensors->leftFootDown && sensors->rightFootDown){
                    //         sensors->odometry.submat(0,0,2,2) = odometryLeftFoot.submat(0,0,2,2);
                    //     } else {
                    //         sensors->odometry.submat(0,0,2,2) = odometryLeftFoot.submat(0,0,2,2) * sensors->leftFootDown + odometryRightFoot.submat(0,0,2,2) * sensors->rightFootDown;
                    //     }
                    // }

                    if(previousSensors){
                        if(sensors->leftFootDown || sensors->rightFootDown){
                            arma::vec3 measuredTorsoFromLeftFoot = -sensors->forwardKinematics.at(ServoID::L_ANKLE_ROLL).submat(0,0,2,2).t() * sensors->forwardKinematics.at(ServoID::L_ANKLE_ROLL).col(3).rows(0,2);
                            arma::vec3 measuredTorsoFromRightFoot = -sensors->forwardKinematics.at(ServoID::R_ANKLE_ROLL).submat(0,0,2,2).t() * sensors->forwardKinematics.at(ServoID::R_ANKLE_ROLL).col(3).rows(0,2);

                            arma::vec3 previousMeasuredTorsoFromLeftFoot = -previousSensors->forwardKinematics.at(ServoID::L_ANKLE_ROLL).submat(0,0,2,2).t() * previousSensors->forwardKinematics.at(ServoID::L_ANKLE_ROLL).col(3).rows(0,2);
                            arma::vec3 previousMeasuredTorsoFromRightFoot = -previousSensors->forwardKinematics.at(ServoID::R_ANKLE_ROLL).submat(0,0,2,2).t() * previousSensors->forwardKinematics.at(ServoID::R_ANKLE_ROLL).col(3).rows(0,2);

                            arma::vec3 torsoVelFromLeftFoot =  (measuredTorsoFromLeftFoot - previousMeasuredTorsoFromLeftFoot);
                            arma::vec3 torsoVelFromRightFoot =  (measuredTorsoFromRightFoot - previousMeasuredTorsoFromRightFoot);

                            arma::vec3 averageVelocity = (torsoVelFromLeftFoot * static_cast<int>(sensors->leftFootDown) + torsoVelFromRightFoot * static_cast<int>(sensors->rightFootDown))/(static_cast<int>(sensors->rightFootDown) + static_cast<int>(sensors->leftFootDown));
                            if(deltaT > 0){
                                sensors->odometry = averageVelocity.rows(0,1) / deltaT;
                            } else {
                                sensors->odometry = {0,0};
                            }
                        }

                        // Gyro based odometry for orientation
                    } else {
                        sensors->odometry.zeros();
                    }
                    sensors->odometryCovariance = arma::eye(2,2) * odometry_covariance_factor;

                    if(sensors->leftFootDown){
                        sensors->bodyCentreHeight = -sensors->forwardKinematics[ServoID::L_ANKLE_ROLL](2,3);
                    } else if(sensors->rightFootDown){
                        sensors->bodyCentreHeight = -sensors->forwardKinematics[ServoID::R_ANKLE_ROLL](2,3);
                    } else {
                        sensors->bodyCentreHeight = 0;
                    }


                    /************************************************
                     *                  Mass Model                  *
                     ************************************************/
                    //LOOKOUT!!!! ARRAYOPS_MEAT
                    arma::vec4 COM = calculateCentreOfMass<DarwinModel>(sensors->forwardKinematics, true);
                    sensors->centreOfMass = {COM[0],COM[1], COM[2], COM[3]};
                    //emit(drawSphere("COM",sensors->centreOfMass.rows(0,2) + arma::vec3({0,0,2 * 0.093 + 0.0335 + 0.034}),0.1)); //Correcting for robot height in nubugger
                    //END MASS MODEL


                    /************************************************
                     *                  Kinematics Horizon          *
                     ************************************************/
                    sensors->orientationBodyToGround = utility::motion::kinematics::calculateBodyToGround(sensors->orientation.submat(0,2,2,2), sensors->bodyCentreHeight);
                    sensors->orientationCamToGround = sensors->orientationBodyToGround * sensors->forwardKinematics[ServoID::HEAD_PITCH];
                    if(sensors->leftFootDown) {
                        sensors->kinematicsBodyToGround = utility::motion::kinematics::calculateBodyToGround(sensors->forwardKinematics[ServoID::L_ANKLE_ROLL].submat(0,2,2,2),sensors->bodyCentreHeight);
                    } else if (sensors->rightFootDown) {
                        sensors->kinematicsBodyToGround = utility::motion::kinematics::calculateBodyToGround(sensors->forwardKinematics[ServoID::R_ANKLE_ROLL].submat(0,2,2,2),sensors->bodyCentreHeight);
                    }
                    else {
                        sensors->kinematicsBodyToGround = sensors->orientationCamToGround;
                    }
                    sensors->kinematicsCamToGround = sensors->orientationBodyToGround * sensors->forwardKinematics[ServoID::HEAD_PITCH];


                    /************************************************
                     *                  CENTRE OF PRESSURE          *
                     ************************************************/

                    sensors->centreOfPressure = utility::motion::kinematics::calculateCentreOfPressure<DarwinModel>(*sensors);
                    // emit(graph("sensors->centreOfPressure", sensors->centreOfPressure));
                    // emit(graph("groundCoM", arma::vec3(sensors->centreOfMass.rows(0,2))));
                    // log("sensors->centreOfPressure", sensors->centreOfPressure.t());

                    // std::cout << "sensors->kinematicsCamToGround\n" << sensors->kinematicsCamToGround << std::endl;
                    // std::cout << "sensors->orientationCamToGround\n" << sensors->orientationCamToGround << std::endl;
                    // std::cout << "sensors->bodyCentreHeight\n" << sensors->bodyCentreHeight << std::endl;

                    /*emit(graph("Filtered Gravity Vector",
                            float(orientation[0]*9.807),
                            float(orientation[1]*9.807),
                            float(orientation[2]*9.807)
                        ));
                     emit(graph("Filtered Forward Vector",
                            float(orientation[3]),
                            float(orientation[4]),
                            float(orientation[5])
                        ));
                    emit(graph("Orientation Quality", quality
                        ));
                    emit(graph("Difference from gravity", normAcc
                        ));
                    emit(graph("Gyro Filtered", sensors->gyroscope[0],sensors->gyroscope[1], sensors->gyroscope[2]
                        ));*/

                    // TODO: crashes binary
                        integratedOdometry += sensors->odometry * deltaT;

                    // emit(graph("LFoot Down", sensors->leftFootDown
                    //     ));
                    // emit(graph("RFoot Down", sensors->rightFootDown
                    //     ));
                    // emit(graph("Torso Velocity (vx,vy,vz)", sensors->odometry(0,3), sensors->odometry(1,3), sensors->odometry(2,3)
                    //     ));
                    emit(graph("Integrated Odometry", integratedOdometry[0], integratedOdometry[1]
                       ));
                    // emit(graph("COM", sensors->centreOfMass[0], sensors->centreOfMass[1], sensors->centreOfMass[2], sensors->centreOfMass[3]
                    //     ));

                    emit(std::move(sensors));
                });
            }

            Transform3D SensorFilter::calculateOdometryMatrix(
                const message::input::Sensors& sensors,
                const message::input::Sensors& previousSensors,
                utility::motion::kinematics::Side side) {
                    Transform3D bodyFromAnkleInitialInverse, bodyFromAnkleFinal;
                    if(side == Side::LEFT){
                        bodyFromAnkleInitialInverse = previousSensors.forwardKinematics.at(ServoID::L_ANKLE_ROLL);   //Double Inverse
                        bodyFromAnkleFinal = sensors.forwardKinematics.at(ServoID::L_ANKLE_ROLL).i();
                    } else {
                        bodyFromAnkleInitialInverse = previousSensors.forwardKinematics.at(ServoID::R_ANKLE_ROLL);   //Double Inverse
                        bodyFromAnkleFinal = sensors.forwardKinematics.at(ServoID::R_ANKLE_ROLL).i();
                    }
                    return  bodyFromAnkleInitialInverse * bodyFromAnkleFinal;
            }


        }  // darwin
    }  // platform
}  // modules
