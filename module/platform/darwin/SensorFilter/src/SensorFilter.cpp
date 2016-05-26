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
#include "message/input/LimbID.h"
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
            using message::input::LimbID;
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
            : Reactor(std::move(environment)) {

                on<Configuration>("DarwinSensorFilter.yaml").then([this] (const Configuration& config) {

                    // Button config
                    this->config.buttons.debounceThreshold = config["buttons"]["debounce_threshold"].as<int>();

                    // Battery config
                    this->config.battery.chargedVoltage = config["battery"]["charged_voltage"].as<float>();
                    this->config.battery.flatVoltage = config["battery"]["flat_voltage"].as<float>();

                    // Foot down config
                    this->config.foot.fsr.footDownWeight = config["foot"]["fsr"]["foot_down_weight"].as<double>();

                    // Motion filter config
                    this->config.motionFilter.noise.measurement.accelerometer = arma::eye(3,3) * config["motion_filter"]["noise"]["measurement"]["accelerometer"].as<double>();
                    this->config.motionFilter.noise.measurement.gyroscope = arma::eye(3,3) * config["motion_filter"]["noise"]["measurement"]["gyroscope"].as<double>();
                    this->config.motionFilter.noise.measurement.flatFootOdometry = arma::eye(6,6) * config["motion_filter"]["noise"]["measurement"]["gyroscope"].as<double>();

                    // Set our process noise model parameters
                    // Q_input_diag;

                    // orientationFilter.model.processNoiseDiagonal = arma::ones(orientationFilter.model.size);
                    // orientationFilter.model.processNoiseDiagonal.rows(orientationFilter.model.QW,orientationFilter.model.QZ) *= config["imu_position_process_noise"].as<double>();
                    // orientationFilter.model.processNoiseDiagonal.rows(orientationFilter.model.VX,orientationFilter.model.VZ) *= config["imu_velocity_process_noise"].as<double>();
                    // NUClear::log("ProcessNoise Set: \n", orientationFilter.model.processNoiseDiagonal.t());
                });

                on<Last<20, Trigger<DarwinSensors>>, Single>().then([this](const std::list<std::shared_ptr<const DarwinSensors>>& sensors) {

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

                    bool newLeftDown = leftCount > config.buttons.debounceThreshold;
                    bool newMiddleDown = middleCount > config.buttons.debounceThreshold;

                    if(newLeftDown != leftDown) {

                        leftDown = newLeftDown;

                        if(newLeftDown) {
                            log("Left Button Down");
                            emit(std::make_unique<ButtonLeftDown>());
                        }
                        else {
                            log("Left Button Up");
                            emit(std::make_unique<ButtonLeftUp>());
                        }
                    }
                    if(newMiddleDown != middleDown) {

                        middleDown = newMiddleDown;

                        if(newMiddleDown) {
                            log("Middle Button Down");
                            emit(std::make_unique<ButtonMiddleDown>());
                        }
                        else {
                            log("Middle Button Up");
                            emit(std::make_unique<ButtonMiddleUp>());
                        }
                    }
                });

                on<Trigger<DarwinSensors>, Optional<With<Sensors>>, Single, Priority::HIGH>()
                .then("Main Sensors Loop", [this](const DarwinSensors& input, std::shared_ptr<const Sensors> previousSensors) {

                    auto sensors = std::make_unique<Sensors>();

                    /************************************************
                     *                 Raw Sensors                  *
                     ************************************************/

                    // Set our timestamp to when the data was read
                    sensors->timestamp = input.timestamp;

                    // Set our voltage and battery
                    sensors->voltage = input.voltage;

                    // Work out a battery charged percentage
                    sensors->battery = std::max(0.0f, (input.voltage - config.battery.chargedVoltage) / (config.battery.chargedVoltage - config.battery.flatVoltage));

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

                        // If we have previous sensors and our current sensors have an error
                        // we then use our previous sensor values with some updates
                        if(previousSensors && error != DarwinSensors::Error::OK) {
                            // Add the sensor values to the system properly
                            sensors->servos.push_back({
                                error,
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
                        // Otherwise we can just use the new values as is
                        else {
                            // Add the sensor values to the system properly
                            sensors->servos.push_back({
                                error,
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
                        NUClear::log<NUClear::WARN>("Bad gyroscope value", arma::norm(arma::vec({input.gyroscope.x, input.gyroscope.y, input.gyroscope.z}), 2));
                        sensors->gyroscope = previousSensors->gyroscope;
                    }
                    else {
                        sensors->gyroscope = {-input.gyroscope.x, -input.gyroscope.y, input.gyroscope.z};
                    }

                    // Put in our FSR information
                    sensors->fsrs.emplace_back();
                    sensors->fsrs.emplace_back();

                    sensors->fsrs[int(LimbID::LEFT_LEG)].centre = { input.fsr.left.centreX, input.fsr.left.centreY };
                    sensors->fsrs[int(LimbID::LEFT_LEG)].values.push_back(input.fsr.left.fsr1);
                    sensors->fsrs[int(LimbID::LEFT_LEG)].values.push_back(input.fsr.left.fsr2);
                    sensors->fsrs[int(LimbID::LEFT_LEG)].values.push_back(input.fsr.left.fsr3);
                    sensors->fsrs[int(LimbID::LEFT_LEG)].values.push_back(input.fsr.left.fsr4);

                    sensors->fsrs[int(LimbID::RIGHT_LEG)].centre = { input.fsr.right.centreX, input.fsr.right.centreY };
                    sensors->fsrs[int(LimbID::RIGHT_LEG)].values.push_back(input.fsr.right.fsr1);
                    sensors->fsrs[int(LimbID::RIGHT_LEG)].values.push_back(input.fsr.right.fsr2);
                    sensors->fsrs[int(LimbID::RIGHT_LEG)].values.push_back(input.fsr.right.fsr3);
                    sensors->fsrs[int(LimbID::RIGHT_LEG)].values.push_back(input.fsr.right.fsr4);

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
                     *            Foot down information             *
                     ************************************************/
                    // Count the number of FSRs that have more than 1/16th of the robots mass
                    sensors->leftFootDown = (input.fsr.left.fsr1 > config.foot.fsr.footDownWeight ? 0.25 : 0.0)
                                          + (input.fsr.left.fsr2 > config.foot.fsr.footDownWeight ? 0.25 : 0.0)
                                          + (input.fsr.left.fsr3 > config.foot.fsr.footDownWeight ? 0.25 : 0.0)
                                          + (input.fsr.left.fsr4 > config.foot.fsr.footDownWeight ? 0.25 : 0.0);

                    // Count the number of FSRs that have more than 1/16th of the robots mass
                    sensors->rightFootDown = (input.fsr.left.fsr1 > config.foot.fsr.footDownWeight ? 0.25 : 0.0)
                                           + (input.fsr.left.fsr2 > config.foot.fsr.footDownWeight ? 0.25 : 0.0)
                                           + (input.fsr.left.fsr3 > config.foot.fsr.footDownWeight ? 0.25 : 0.0)
                                           + (input.fsr.left.fsr4 > config.foot.fsr.footDownWeight ? 0.25 : 0.0);

                    /************************************************
                     *             Motion (IMU+Odometry)            *
                     ************************************************/

                    // Calculate our time offset from the last read
                    double deltaT = (input.timestamp - (previousSensors ? previousSensors->timestamp : input.timestamp)).count() / double(NUClear::clock::period::den);

                    // Time update
                    motionFilter.timeUpdate(deltaT);

                    // Accelerometer measurment update
                    motionFilter.measurementUpdate(sensors->accelerometer, config.motionFilter.noise.measurement.accelerometer, MotionModel::MeasurementType::ACCELEROMETER());

                    // Gyroscope measurement update
                    motionFilter.measurementUpdate(sensors->gyroscope, config.motionFilter.noise.measurement.gyroscope, MotionModel::MeasurementType::GYROSCOPE());

                    // 3 points on the ground mean that we can assume this foot is flat
                    // We also have to ensure that the previous foot was also down for this to be valid

                    // Check if our foot is flat on the ground
                    if (sensors->leftFootDown >= 0.75) {

                        // Get the torso in foot space
                        auto footToTorso = sensors->forwardKinematics[ServoID::L_ANKLE_ROLL].i();

                        // Construct our measurement vector from the up vector in torso space and the z height from the foot
                        arma::vec4 footUpWithZ;
                        // This is an up world vector in torso space
                        footUpWithZ.rows(0,2) = sensors->forwardKinematics[ServoID::L_ANKLE_ROLL].col(2);
                        // This is the z height of the torso above the ground
                        footUpWithZ[3] = footToTorso.translation()[2];
                        motionFilter.measurementUpdate(footUpWithZ, config.motionFilter.noise.measurement.footUpWithZ, MotionModel::MeasurementType::FOOT_UP_WITH_Z());

                        // If we don't have previous sensors, or the previous sensors had the foot up
                        if (!previousSensors || previousSensors->leftFootDown < 0.75) {

                            // Get the torso's x,y position in left foot space and from the current estimation
                            // We use this coordinates as the origins for our odometry position delta updates
                            leftFootLanding = footToTorso.translation().rows(0,1);
                            leftFootLandingWorld = motionFilter.get().rows(MotionModel::PX, MotionModel::PY);
                        }
                        else {
                            // Get how much our torso has moved from our foot landing in foot coordinates
                            arma::vec2 footTorsoDelta = footToTorso.translation().rows(0,1) - leftFootLanding;

                            // TODO Josiah need to rotate footTorsoDelta by the yaw between world space and this foot from the state
                            // rotate footTorsoDelta by yaw between global and foot space to put the delta in global space

                            // Do our measurement update and pass in the original state x,y we measured when the foot landed.
                            motionFilter.measurementUpdate(footTorsoDelta, config.motionFilter.noise.measurement.flatFootOdometry, leftFootLandingWorld, MotionModel::MeasurementType::FLAT_FOOT_ODOMETRY());
                        }
                    }

                    // Gives us the quaternion representation
                    const auto& o = motionFilter.get();

                    // Map from robot to world coordinates
                    sensors->orientation = Rotation3D(UnitQuaternion(o.rows(MotionModel::QW, MotionModel::QZ)));

                    sensors->robotToIMU = calculateRobotToIMU(sensors->orientation);

                    /************************************************
                     *                  Mass Model                  *
                     ************************************************/
                    sensors->centreOfMass = calculateCentreOfMass<DarwinModel>(sensors->forwardKinematics, true);

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


                    emit(std::move(sensors));
                });
            }

        }  // darwin
    }  // platform
}  // modules
