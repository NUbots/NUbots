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

#include "SensorFilter.h"

#include "extension/Configuration.h"
#include "message/input/Sensors.h"
#include "message/platform/darwin/DarwinSensors.h"
#include "utility/input/LimbID.h"
#include "utility/input/ServoID.h"
#include "utility/math/matrix/matrix.h"
#include "utility/motion/ForwardKinematics.h"
#include "utility/nusight/NUhelpers.h"
#include "utility/platform/darwin/DarwinSensors.h"
#include "utility/support/yaml_expression.h"

namespace module {
namespace platform {
    namespace darwin {

        using extension::Configuration;

        using message::input::Sensors;
        using message::motion::BodySide;
        using message::motion::KinematicsModel;
        using message::platform::darwin::ButtonLeftDown;
        using message::platform::darwin::ButtonLeftUp;
        using message::platform::darwin::ButtonMiddleDown;
        using message::platform::darwin::ButtonMiddleUp;
        using message::platform::darwin::DarwinSensors;

        using utility::input::LimbID;
        using utility::input::ServoID;
        using utility::input::ServoSide;
        using utility::motion::kinematics::calculateAllPositions;
        using utility::motion::kinematics::calculateCentreOfMass;
        using utility::motion::kinematics::calculateInertialTensor;
        using utility::motion::kinematics::calculateRobotToIMU;
        using utility::nusight::graph;
        using utility::support::Expression;

        std::string makeErrorString(const std::string& src, uint errorCode) {
            std::stringstream s;

            s << "Error on ";
            s << src;
            s << ":";


            if (errorCode & DarwinSensors::Error::INPUT_VOLTAGE) {
                s << " Input Voltage ";
            }
            if (errorCode & DarwinSensors::Error::ANGLE_LIMIT) {
                s << " Angle Limit ";
            }
            if (errorCode & DarwinSensors::Error::OVERHEATING) {
                s << " Overheating ";
            }
            if (errorCode & DarwinSensors::Error::OVERLOAD) {
                s << " Overloaded ";
            }
            if (errorCode & DarwinSensors::Error::INSTRUCTION) {
                s << " Bad Instruction ";
            }
            if (errorCode & DarwinSensors::Error::CORRUPT_DATA) {
                s << " Corrupt Data ";
            }
            if (errorCode & DarwinSensors::Error::TIMEOUT) {
                s << " Timeout ";
            }

            return s.str();
        }

        SensorFilter::SensorFilter(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment)), theta(Eigen::Vector3d::Zero()) {

            on<Configuration>("SensorFilter.yaml").then([this](const Configuration& config) {
                this->config.debug = config["debug"].as<bool>();
                // Button config
                this->config.buttons.debounceThreshold = config["buttons"]["debounce_threshold"].as<int>();

                // Foot down config
                this->config.footDown.fromLoad           = config["foot_down"]["from_load"].as<bool>();
                this->config.footDown.certaintyThreshold = config["foot_down"]["certainty_threshold"].as<float>();

                // Motion filter config
                // Update our velocity timestep dekay
                this->config.motionFilter.velocityDecay =
                    config["motion_filter"]["update"]["velocity_decay"].as<Expression>();
                motionFilter.model.timeUpdateVelocityDecay = this->config.motionFilter.velocityDecay;

                // Update our measurement noises
                this->config.motionFilter.noise.measurement.accelerometer =
                    Eigen::Vector3d(config["motion_filter"]["noise"]["measurement"]["accelerometer"].as<Expression>())
                        .asDiagonal();
                this->config.motionFilter.noise.measurement.accelerometerMagnitude =
                    Eigen::Vector3d(
                        config["motion_filter"]["noise"]["measurement"]["accelerometer_magnitude"].as<Expression>())
                        .asDiagonal();
                this->config.motionFilter.noise.measurement.gyroscope =
                    Eigen::Vector3d(config["motion_filter"]["noise"]["measurement"]["gyroscope"].as<Expression>())
                        .asDiagonal();
                this->config.motionFilter.noise.measurement.flatFootOdometry =
                    Eigen::Vector3d(
                        config["motion_filter"]["noise"]["measurement"]["flat_foot_odometry"].as<Expression>())
                        .asDiagonal();
                this->config.motionFilter.noise.measurement.flatFootOrientation =
                    Eigen::Vector4d(
                        config["motion_filter"]["noise"]["measurement"]["flat_foot_orientation"].as<Expression>())
                        .asDiagonal();

                // Update our process noises
                this->config.motionFilter.noise.process.position =
                    config["motion_filter"]["noise"]["process"]["position"].as<Expression>();
                this->config.motionFilter.noise.process.velocity =
                    config["motion_filter"]["noise"]["process"]["velocity"].as<Expression>();
                this->config.motionFilter.noise.process.rotation =
                    config["motion_filter"]["noise"]["process"]["rotation"].as<Expression>();
                this->config.motionFilter.noise.process.rotationalVelocity =
                    config["motion_filter"]["noise"]["process"]["rotational_velocity"].as<Expression>();
                this->config.motionFilter.noise.process.gyroscopeBias =
                    config["motion_filter"]["noise"]["process"]["gyroscope_bias"].as<Expression>();

                // Set our process noise in our filter
                MotionModel<double>::StateVec process_noise;
                process_noise.segment<3>(MotionModel<double>::PX) = this->config.motionFilter.noise.process.position;
                process_noise.segment<3>(MotionModel<double>::VX) = this->config.motionFilter.noise.process.velocity;
                process_noise.segment<4>(MotionModel<double>::QX) = this->config.motionFilter.noise.process.rotation;
                process_noise.segment<3>(MotionModel<double>::WX) =
                    this->config.motionFilter.noise.process.rotationalVelocity;
                process_noise.segment<3>(MotionModel<double>::BX) =
                    this->config.motionFilter.noise.process.gyroscopeBias;
                motionFilter.model.process_noise = process_noise;

                // Update our mean configs and if it changed, reset the filter
                this->config.motionFilter.initial.mean.position =
                    config["motion_filter"]["initial"]["mean"]["position"].as<Expression>();
                this->config.motionFilter.initial.mean.velocity =
                    config["motion_filter"]["initial"]["mean"]["velocity"].as<Expression>();
                this->config.motionFilter.initial.mean.rotation =
                    config["motion_filter"]["initial"]["mean"]["rotation"].as<Expression>();
                this->config.motionFilter.initial.mean.rotationalVelocity =
                    config["motion_filter"]["initial"]["mean"]["rotational_velocity"].as<Expression>();
                this->config.motionFilter.initial.mean.gyroscopeBias =
                    config["motion_filter"]["initial"]["mean"]["gyroscope_bias"].as<Expression>();

                this->config.motionFilter.initial.covariance.position =
                    config["motion_filter"]["initial"]["covariance"]["position"].as<Expression>();
                this->config.motionFilter.initial.covariance.velocity =
                    config["motion_filter"]["initial"]["covariance"]["velocity"].as<Expression>();
                this->config.motionFilter.initial.covariance.rotation =
                    config["motion_filter"]["initial"]["covariance"]["rotation"].as<Expression>();
                this->config.motionFilter.initial.covariance.rotationalVelocity =
                    config["motion_filter"]["initial"]["covariance"]["rotational_velocity"].as<Expression>();
                this->config.motionFilter.initial.covariance.gyroscopeBias =
                    config["motion_filter"]["initial"]["covariance"]["gyroscope_bias"].as<Expression>();

                // Calculate our mean and covariance
                MotionModel<double>::StateVec mean;
                mean.segment<3>(MotionModel<double>::PX) = this->config.motionFilter.initial.mean.position;
                mean.segment<3>(MotionModel<double>::VX) = this->config.motionFilter.initial.mean.velocity;
                mean.segment<4>(MotionModel<double>::QX) = this->config.motionFilter.initial.mean.rotation;
                mean.segment<3>(MotionModel<double>::WX) = this->config.motionFilter.initial.mean.rotationalVelocity;
                mean.segment<3>(MotionModel<double>::BX) = this->config.motionFilter.initial.mean.gyroscopeBias;

                MotionModel<double>::StateVec covariance;
                covariance.segment<3>(MotionModel<double>::PX) = this->config.motionFilter.initial.covariance.position;
                covariance.segment<3>(MotionModel<double>::VX) = this->config.motionFilter.initial.covariance.velocity;
                covariance.segment<4>(MotionModel<double>::QX) = this->config.motionFilter.initial.covariance.rotation;
                covariance.segment<3>(MotionModel<double>::WX) =
                    this->config.motionFilter.initial.covariance.rotationalVelocity;
                covariance.segment<3>(MotionModel<double>::BX) =
                    this->config.motionFilter.initial.covariance.gyroscopeBias;
                motionFilter.set_state(mean, covariance.asDiagonal());
            });

            on<Configuration>("FootDownNetwork.yaml").then([this](const Configuration& config) {
                // Foot load sensor config
                load_sensor = VirtualLoadSensor<float>(config);
            });

            on<Last<20, Trigger<DarwinSensors>>, Single>().then(
                [this](const std::list<std::shared_ptr<const DarwinSensors>>& sensors) {
                    int leftCount   = 0;
                    int middleCount = 0;

                    // If we have any downs in the last 20 frames then we are button pushed
                    for (const auto& s : sensors) {
                        if (s->buttons.left && !s->cm740ErrorFlags) {
                            ++leftCount;
                        }
                        if (s->buttons.middle && !s->cm740ErrorFlags) {
                            ++middleCount;
                        }
                    }

                    bool newLeftDown   = leftCount > config.buttons.debounceThreshold;
                    bool newMiddleDown = middleCount > config.buttons.debounceThreshold;

                    if (newLeftDown != leftDown) {

                        leftDown = newLeftDown;

                        if (newLeftDown) {
                            log("Left Button Down");
                            emit(std::make_unique<ButtonLeftDown>());
                        }
                        else {
                            log("Left Button Up");
                            emit(std::make_unique<ButtonLeftUp>());
                        }
                    }
                    if (newMiddleDown != middleDown) {

                        middleDown = newMiddleDown;

                        if (newMiddleDown) {
                            log("Middle Button Down");
                            emit(std::make_unique<ButtonMiddleDown>());
                        }
                        else {
                            log("Middle Button Up");
                            emit(std::make_unique<ButtonMiddleUp>());
                        }
                    }
                });

            on<Trigger<DarwinSensors>, Optional<With<Sensors>>, With<KinematicsModel>, Single, Priority::HIGH>().then(
                "Main Sensors Loop",
                [this](const DarwinSensors& input,
                       std::shared_ptr<const Sensors> previousSensors,
                       const KinematicsModel& kinematicsModel) {
                    auto sensors = std::make_unique<Sensors>();

                    /************************************************
                     *                 Raw Sensors                  *
                     ************************************************/

                    // Set our timestamp to when the data was read
                    sensors->timestamp = input.timestamp;

                    sensors->voltage = input.voltage;


                    // This checks for an error on the CM740 and reports it
                    if (input.cm740ErrorFlags != DarwinSensors::Error::OK) {
                        NUClear::log<NUClear::WARN>(makeErrorString("CM740", input.cm740ErrorFlags));
                    }

                    // Output errors on the FSRs
                    if (input.fsr.left.errorFlags != DarwinSensors::Error::OK) {
                        NUClear::log<NUClear::WARN>(makeErrorString("Left FSR", input.fsr.left.errorFlags));
                    }

                    if (input.fsr.right.errorFlags != DarwinSensors::Error::OK) {
                        NUClear::log<NUClear::WARN>(makeErrorString("Right FSR", input.fsr.right.errorFlags));
                    }

                    // Read through all of our sensors
                    for (uint32_t i = 0; i < 20; ++i) {
                        auto& original = utility::platform::darwin::getDarwinServo(i, input);
                        auto& error    = original.errorFlags;

                        // Check for an error on the servo and report it
                        while (error != DarwinSensors::Error::OK) {
                            std::stringstream s;
                            s << "Error on Servo " << (i + 1) << " (" << static_cast<ServoID>(i) << "):";

                            if (error & DarwinSensors::Error::INPUT_VOLTAGE) {
                                s << " Input Voltage - " << original.voltage;
                            }
                            if (error & DarwinSensors::Error::ANGLE_LIMIT) {
                                s << " Angle Limit - " << original.presentPosition;
                            }
                            if (error & DarwinSensors::Error::OVERHEATING) {
                                s << " Overheating - " << original.temperature;
                            }
                            if (error & DarwinSensors::Error::OVERLOAD) {
                                s << " Overloaded - " << original.load;
                            }
                            if (error & DarwinSensors::Error::INSTRUCTION) {
                                s << " Bad Instruction ";
                            }
                            if (error & DarwinSensors::Error::CORRUPT_DATA) {
                                s << " Corrupt Data ";
                            }
                            if (error & DarwinSensors::Error::TIMEOUT) {
                                s << " Timeout ";
                            }

                            NUClear::log<NUClear::WARN>(s.str());
                            break;
                        }

                        // If we have previous sensors and our current sensors have an error
                        // we then use our previous sensor values with some updates
                        if (previousSensors && error != DarwinSensors::Error::OK) {
                            // Add the sensor values to the system properly
                            sensors->servo.push_back({error,
                                                      i,
                                                      original.torqueEnabled,
                                                      original.pGain,
                                                      original.iGain,
                                                      original.dGain,
                                                      original.goalPosition,
                                                      original.movingSpeed,
                                                      previousSensors->servo[i].present_position,
                                                      previousSensors->servo[i].present_velocity,
                                                      previousSensors->servo[i].load,
                                                      previousSensors->servo[i].voltage,
                                                      previousSensors->servo[i].temperature});
                        }
                        // Otherwise we can just use the new values as is
                        else {
                            // Add the sensor values to the system properly
                            sensors->servo.push_back({error,
                                                      i,
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
                                                      float(original.temperature)});
                        }
                    }

                    // gyro_x to the right
                    // gyro_y to the back
                    // gyro_z down

                    // acc_x to the back
                    // acc_y to the left
                    // acc_z up

                    // If we have a previous sensors and our cm740 has errors then reuse our last sensor value
                    if (previousSensors && (input.cm740ErrorFlags)) {
                        sensors->accelerometer = previousSensors->accelerometer;
                    }
                    else {
                        sensors->accelerometer =
                            Eigen::Vector3d(-input.accelerometer.x, input.accelerometer.y, input.accelerometer.z);
                    }

                    // If we have a previous sensors and our cm740 has errors then reuse our last sensor value
                    if (previousSensors
                        && (input.cm740ErrorFlags
                            || Eigen::Vector3d(input.gyroscope.x, input.gyroscope.y, input.gyroscope.z).norm()
                                   > 4.0 * M_PI)) {
                        NUClear::log<NUClear::WARN>(
                            "Bad gyroscope value",
                            Eigen::Vector3d(input.gyroscope.x, input.gyroscope.y, input.gyroscope.z).norm());
                        sensors->gyroscope = previousSensors->gyroscope;
                    }
                    else {
                        sensors->gyroscope = Eigen::Vector3d(input.gyroscope.y, input.gyroscope.x, -input.gyroscope.z);
                    }

                    // Put in our FSR information
                    sensors->fsr.emplace_back();
                    sensors->fsr.emplace_back();

                    sensors->fsr[LimbID::LEFT_LEG - 1].centre << input.fsr.left.centreX, input.fsr.left.centreY;
                    sensors->fsr[LimbID::LEFT_LEG - 1].value.reserve(4);
                    sensors->fsr[LimbID::LEFT_LEG - 1].value.push_back(input.fsr.left.fsr1);
                    sensors->fsr[LimbID::LEFT_LEG - 1].value.push_back(input.fsr.left.fsr2);
                    sensors->fsr[LimbID::LEFT_LEG - 1].value.push_back(input.fsr.left.fsr3);
                    sensors->fsr[LimbID::LEFT_LEG - 1].value.push_back(input.fsr.left.fsr4);

                    sensors->fsr[LimbID::RIGHT_LEG - 1].centre << input.fsr.right.centreX, input.fsr.right.centreY;
                    sensors->fsr[LimbID::RIGHT_LEG - 1].value.reserve(4);
                    sensors->fsr[LimbID::RIGHT_LEG - 1].value.push_back(input.fsr.right.fsr1);
                    sensors->fsr[LimbID::RIGHT_LEG - 1].value.push_back(input.fsr.right.fsr2);
                    sensors->fsr[LimbID::RIGHT_LEG - 1].value.push_back(input.fsr.right.fsr3);
                    sensors->fsr[LimbID::RIGHT_LEG - 1].value.push_back(input.fsr.right.fsr4);

                    /************************************************
                     *               Buttons and LEDs               *
                     ************************************************/
                    sensors->button.reserve(2);
                    sensors->button.push_back(Sensors::Button(0, input.buttons.left));
                    sensors->button.push_back(Sensors::Button(1, input.buttons.middle));
                    sensors->led.reserve(5);
                    sensors->led.push_back(Sensors::LED(0, input.ledPanel.led2 ? 0xFF0000 : 0));
                    sensors->led.push_back(Sensors::LED(1, input.ledPanel.led3 ? 0xFF0000 : 0));
                    sensors->led.push_back(Sensors::LED(2, input.ledPanel.led4 ? 0xFF0000 : 0));
                    sensors->led.push_back(Sensors::LED(3, input.headLED.RGB));  // Head
                    sensors->led.push_back(Sensors::LED(4, input.eyeLED.RGB));   // Eye

                    /************************************************
                     *                  Kinematics                  *
                     ************************************************/

                    auto forward_kinematics = calculateAllPositions(kinematicsModel, *sensors);
                    for (const auto& entry : forward_kinematics) {
                        sensors->forward_kinematics[entry.first] = entry.second.matrix();
                    }

                    /************************************************
                     *            Foot down information             *
                     ************************************************/
                    sensors->right_foot_down = true;
                    sensors->left_foot_down  = true;

                    std::array<bool, 2> feet_down = {true};
                    if (config.footDown.fromLoad) {
                        // Use our virtual load sensor class to work out which feet are down
                        feet_down = load_sensor.updateFeet(*sensors);

                        if (this->config.debug) {
                            emit(graph("Sensor/Foot Down/Load/Left", load_sensor.state[1]));
                            emit(graph("Sensor/Foot Down/Load/Right", load_sensor.state[0]));
                        }
                    }
                    else {
                        auto rightFootDisplacement = sensors->forward_kinematics[ServoID::R_ANKLE_ROLL].inverse()(2, 3);
                        auto leftFootDisplacement  = sensors->forward_kinematics[ServoID::L_ANKLE_ROLL].inverse()(2, 3);

                        if (rightFootDisplacement < leftFootDisplacement - config.footDown.certaintyThreshold) {
                            feet_down[0] = true;
                            feet_down[1] = false;
                        }
                        else if (leftFootDisplacement < rightFootDisplacement - config.footDown.certaintyThreshold) {
                            feet_down[0] = false;
                            feet_down[1] = true;
                        }
                        else {
                            feet_down[0] = true;
                            feet_down[1] = true;
                        }

                        if (this->config.debug) {
                            emit(graph("Sensor/Foot Down/Z/Left", feet_down[1]));
                            emit(graph("Sensor/Foot Down/Z/Right", feet_down[0]));
                        }
                    }

                    sensors->right_foot_down = feet_down[0];
                    sensors->left_foot_down  = feet_down[1];

                    /************************************************
                     *             Motion (IMU+Odometry)            *
                     ************************************************/

                    // Gyroscope measurement update
                    motionFilter.measure(sensors->gyroscope,
                                         config.motionFilter.noise.measurement.gyroscope,
                                         MeasurementType::GYROSCOPE());

                    // Calculate accelerometer noise factor
                    Eigen::Matrix3d acc_noise = config.motionFilter.noise.measurement.accelerometer
                                                + ((sensors->accelerometer.norm() - std::abs(G))
                                                   * (sensors->accelerometer.norm() - std::abs(G)))
                                                      * config.motionFilter.noise.measurement.accelerometerMagnitude;

                    // Accelerometer measurement update
                    motionFilter.measure(sensors->accelerometer, acc_noise, MeasurementType::ACCELEROMETER());

                    for (auto& side : {ServoSide::LEFT, ServoSide::RIGHT}) {
                        bool foot_down = side == ServoSide::LEFT ? sensors->left_foot_down : sensors->right_foot_down;
                        bool prev_foot_down = previous_foot_down[side];
                        Eigen::Affine3d Htf(
                            sensors->forward_kinematics[side == ServoSide::LEFT ? ServoID::L_ANKLE_ROLL
                                                                                : ServoID::R_ANKLE_ROLL]);

                        if (foot_down && !prev_foot_down) {
                            Eigen::Affine3d Hwt;
                            Hwt.linear() = Eigen::Quaterniond(motionFilter.get().segment<4>(MotionModel<double>::QX))
                                               .toRotationMatrix();
                            Hwt.translation() = Eigen::Vector3d(motionFilter.get().segment<3>(MotionModel<double>::PX));

                            Eigen::Affine3d Htg(utility::motion::kinematics::calculateGroundSpace(Htf, Hwt));

                            footlanding_Hwf[side]                   = Hwt * Htg;
                            footlanding_Hwf[side].translation().z() = 0.0;

                            previous_foot_down[side] = true;
                        }
                        else if (foot_down && prev_foot_down) {
                            // Use stored Hwf and Htf to calculate Hwt
                            Eigen::Affine3d footlanding_Hwt = footlanding_Hwf[side] * Htf.inverse();

                            // do a foot based position update
                            motionFilter.measure(Eigen::Vector3d(footlanding_Hwt.translation()),
                                                 config.motionFilter.noise.measurement.flatFootOdometry,
                                                 MeasurementType::FLAT_FOOT_ODOMETRY());

                            // do a foot based orientation update
                            Eigen::Quaterniond Rwt(footlanding_Hwt.linear());
                            motionFilter.measure(Rwt.coeffs(),
                                                 config.motionFilter.noise.measurement.flatFootOrientation,
                                                 MeasurementType::FLAT_FOOT_ORIENTATION());
                        }
                        else if (!foot_down) {
                            previous_foot_down[side] = false;
                        }
                    }

                    // Calculate our time offset from the last read
                    double deltaT = std::max(
                        (input.timestamp - (previousSensors ? previousSensors->timestamp : input.timestamp)).count()
                            / double(NUClear::clock::period::den),
                        0.0);

                    // Time update
                    motionFilter.time(deltaT);

                    // Gives us the quaternion representation
                    const auto& o = motionFilter.get();

                    // Map from world to torso coordinates (Rtw)
                    Eigen::Affine3d Hwt;
                    Hwt.linear()      = Eigen::Quaterniond(o.segment<4>(MotionModel<double>::QX)).toRotationMatrix();
                    Hwt.translation() = Eigen::Vector3d(o.segment<3>(MotionModel<double>::PX));
                    sensors->Htw      = Hwt.inverse().matrix();

                    // Integrate gyro to get angular positions
                    sensors->angular_position = o.segment<3>(MotionModel<double>::WX) / 90.0;

                    if (this->config.debug) {
                        log("p_x:",
                            sensors->angular_position.x(),
                            "p_y:",
                            sensors->angular_position.y(),
                            "p_z:",
                            sensors->angular_position.z());
                    }

                    sensors->robot_to_IMU = calculateRobotToIMU(Eigen::Affine3d(sensors->Htw));

                    /************************************************
                     *                  Mass Model                  *
                     ************************************************/
                    sensors->centre_of_mass =
                        calculateCentreOfMass(kinematicsModel, sensors->forward_kinematics, sensors->Htw.inverse());
                    sensors->inertial_tensor = calculateInertialTensor(kinematicsModel, sensors->forward_kinematics);

                    /************************************************
                     *                  Kinematics Horizon          *
                     ************************************************/
                    sensors->body_centre_height = Eigen::Vector3d(o.segment<3>(MotionModel<double>::PX)).z();

                    Eigen::Affine3d Rwt(sensors->Htw.inverse());
                    // remove translation components from the transform
                    Rwt.translation() = Eigen::Vector3d::Zero();
                    Eigen::Affine3d Rgt(
                        Eigen::AngleAxisd(-Rwt.rotation().eulerAngles(0, 1, 2).z(), Eigen::Vector3d::UnitZ()) * Rwt);
                    // sensors->Hgt : Mat size [4x4] (default identity)
                    // createRotationZ : Mat size [3x3]
                    // Rwt : Mat size [3x3]
                    sensors->Hgt = Rgt.matrix();
                    auto Htc     = sensors->forward_kinematics[ServoID::HEAD_PITCH];

                    // Get torso to world transform
                    Eigen::Affine3d yawlessWorldInvR(
                        Eigen::AngleAxisd(-Hwt.rotation().eulerAngles(0, 1, 2).z(), Eigen::Vector3d::UnitZ())
                        * Hwt.rotation());
                    Eigen::Affine3d Hgt(Hwt);
                    Hgt.translation() = Eigen::Vector3d(0, 0, Hwt.translation().z());
                    Hgt.linear()      = yawlessWorldInvR.linear();
                    sensors->Hgc      = Hgt * Htc;  // Rwt * Rth

                    /************************************************
                     *                  CENTRE OF PRESSURE          *
                     ************************************************/
                    sensors->centre_of_pressure =
                        utility::motion::kinematics::calculateCentreOfPressure(kinematicsModel, *sensors);

                    emit(std::move(sensors));
                });
        }
    }  // namespace darwin
}  // namespace platform
}  // namespace module
