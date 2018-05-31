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

#include "message/input/CameraParameters.h"
#include "message/input/Sensors.h"
//#include "message/localisation/ResetRobotHypotheses.h"
#include "message/platform/darwin/DarwinSensors.h"

#include "utility/input/LimbID.h"
#include "utility/input/ServoID.h"
#include "utility/math/geometry/UnitQuaternion.h"
#include "utility/math/matrix/Rotation2D.h"
#include "utility/motion/ForwardKinematics.h"
#include "utility/nusight/NUhelpers.h"
#include "utility/platform/darwin/DarwinSensors.h"
#include "utility/support/eigen_armadillo.h"
#include "utility/support/yaml_armadillo.h"

namespace module {
namespace platform {
    namespace darwin {

        using extension::Configuration;

        using message::input::CameraParameters;
        using message::input::Sensors;
        using message::motion::BodySide;
        using message::platform::darwin::ButtonLeftDown;
        using message::platform::darwin::ButtonLeftUp;
        using message::platform::darwin::ButtonMiddleDown;
        using message::platform::darwin::ButtonMiddleUp;
        using message::platform::darwin::DarwinSensors;

        using utility::input::ServoSide;
        using LimbID  = utility::input::LimbID;
        using ServoID = utility::input::ServoID;
        // using message::localisation::ResetRobotHypotheses;
        using message::motion::KinematicsModel;
        using utility::math::geometry::UnitQuaternion;
        using utility::math::matrix::Rotation2D;
        using utility::math::matrix::Rotation3D;
        using utility::math::matrix::Transform3D;
        using utility::motion::kinematics::calculateAllPositions;
        using utility::motion::kinematics::calculateCentreOfMass;
        using utility::motion::kinematics::calculateRobotToIMU;
        using utility::nusight::drawArrow;
        using utility::nusight::drawSphere;
        using utility::nusight::graph;

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
            : Reactor(std::move(environment))
            , motionFilter()
            , config()
            , leftFootDown()
            , rightFootDown()
            , footlanding_rFWw()
            , footlanding_Rfw()
            , footlanding_Rwf() {

            on<Configuration>("DarwinSensorFilter.yaml").then([this](const Configuration& config) {
                // Button config
                this->config.buttons.debounceThreshold = config["buttons"]["debounce_threshold"].as<int>();

                // Battery config
                this->config.battery.chargedVoltage = config["battery"]["charged_voltage"].as<float>();
                this->config.battery.nominalVoltage = config["battery"]["nominal_voltage"].as<float>();
                this->config.battery.flatVoltage    = config["battery"]["flat_voltage"].as<float>();

                // Foot load sensor config
                leftFootDown =
                    DarwinVirtualLoadSensor(config["foot_load_sensor"]["hidden_layer"]["weights"].as<arma::mat>(),
                                            config["foot_load_sensor"]["hidden_layer"]["bias"].as<arma::vec>(),
                                            config["foot_load_sensor"]["output_layer"]["weights"].as<arma::mat>(),
                                            config["foot_load_sensor"]["output_layer"]["bias"].as<arma::vec>(),
                                            config["foot_load_sensor"]["noise_factor"].as<double>(),
                                            config["foot_load_sensor"]["certainty_threshold"].as<double>(),
                                            config["foot_load_sensor"]["uncertainty_threshold"].as<double>());

                rightFootDown =
                    DarwinVirtualLoadSensor(config["foot_load_sensor"]["hidden_layer"]["weights"].as<arma::mat>(),
                                            config["foot_load_sensor"]["hidden_layer"]["bias"].as<arma::vec>(),
                                            config["foot_load_sensor"]["output_layer"]["weights"].as<arma::mat>(),
                                            config["foot_load_sensor"]["output_layer"]["bias"].as<arma::vec>(),
                                            config["foot_load_sensor"]["noise_factor"].as<double>(),
                                            config["foot_load_sensor"]["certainty_threshold"].as<double>(),
                                            config["foot_load_sensor"]["uncertainty_threshold"].as<double>());

                // Motion filter config
                // Update our velocity timestep dekay
                this->config.motionFilter.velocityDecay =
                    config["motion_filter"]["update"]["velocity_decay"].as<arma::vec3>();
                motionFilter.model.timeUpdateVelocityDecay = this->config.motionFilter.velocityDecay;

                // Update our measurement noises
                this->config.motionFilter.noise.measurement.accelerometer =
                    arma::diagmat(config["motion_filter"]["noise"]["measurement"]["accelerometer"].as<arma::vec3>());
                this->config.motionFilter.noise.measurement.accelerometerMagnitude = arma::diagmat(
                    config["motion_filter"]["noise"]["measurement"]["accelerometer_magnitude"].as<arma::vec3>());
                this->config.motionFilter.noise.measurement.gyroscope =
                    arma::diagmat(config["motion_filter"]["noise"]["measurement"]["gyroscope"].as<arma::vec3>());
                this->config.motionFilter.noise.measurement.footUpWithZ =
                    arma::diagmat(config["motion_filter"]["noise"]["measurement"]["foot_up_with_z"].as<arma::vec4>());
                this->config.motionFilter.noise.measurement.flatFootOdometry = arma::diagmat(
                    config["motion_filter"]["noise"]["measurement"]["flat_foot_odometry"].as<arma::vec3>());
                this->config.motionFilter.noise.measurement.flatFootOrientation = arma::diagmat(
                    config["motion_filter"]["noise"]["measurement"]["flat_foot_orientation"].as<arma::vec4>());

                // Update our process noises
                this->config.motionFilter.noise.process.position =
                    config["motion_filter"]["noise"]["process"]["position"].as<arma::vec3>();
                this->config.motionFilter.noise.process.velocity =
                    config["motion_filter"]["noise"]["process"]["velocity"].as<arma::vec3>();
                this->config.motionFilter.noise.process.rotation =
                    config["motion_filter"]["noise"]["process"]["rotation"].as<arma::vec4>();
                this->config.motionFilter.noise.process.rotationalVelocity =
                    config["motion_filter"]["noise"]["process"]["rotational_velocity"].as<arma::vec3>();

                // Set our process noise in our filter
                arma::vec::fixed<MotionModel::size> processNoise;
                processNoise.rows(MotionModel::PX, MotionModel::PZ) = this->config.motionFilter.noise.process.position;
                processNoise.rows(MotionModel::VX, MotionModel::VZ) = this->config.motionFilter.noise.process.velocity;
                processNoise.rows(MotionModel::QW, MotionModel::QZ) = this->config.motionFilter.noise.process.rotation;
                processNoise.rows(MotionModel::WX, MotionModel::WZ) =
                    this->config.motionFilter.noise.process.rotationalVelocity;
                motionFilter.model.processNoiseMatrix = arma::diagmat(processNoise);

                // Update our mean configs and if it changed, reset the filter
                this->config.motionFilter.initial.mean.position =
                    config["motion_filter"]["initial"]["mean"]["position"].as<arma::vec3>();
                this->config.motionFilter.initial.mean.velocity =
                    config["motion_filter"]["initial"]["mean"]["velocity"].as<arma::vec3>();
                this->config.motionFilter.initial.mean.rotation =
                    config["motion_filter"]["initial"]["mean"]["rotation"].as<arma::vec4>();
                this->config.motionFilter.initial.mean.rotationalVelocity =
                    config["motion_filter"]["initial"]["mean"]["rotational_velocity"].as<arma::vec3>();

                this->config.motionFilter.initial.covariance.position =
                    config["motion_filter"]["initial"]["covariance"]["position"].as<arma::vec3>();
                this->config.motionFilter.initial.covariance.velocity =
                    config["motion_filter"]["initial"]["covariance"]["velocity"].as<arma::vec3>();
                this->config.motionFilter.initial.covariance.rotation =
                    config["motion_filter"]["initial"]["covariance"]["rotation"].as<arma::vec4>();
                this->config.motionFilter.initial.covariance.rotationalVelocity =
                    config["motion_filter"]["initial"]["covariance"]["rotational_velocity"].as<arma::vec3>();

                // Calculate our mean and covariance
                arma::vec::fixed<MotionModel::size> mean;
                mean.rows(MotionModel::PX, MotionModel::PZ) = this->config.motionFilter.initial.mean.position;
                mean.rows(MotionModel::VX, MotionModel::VZ) = this->config.motionFilter.initial.mean.velocity;
                mean.rows(MotionModel::QW, MotionModel::QZ) = this->config.motionFilter.initial.mean.rotation;
                mean.rows(MotionModel::WX, MotionModel::WZ) = this->config.motionFilter.initial.mean.rotationalVelocity;

                arma::vec::fixed<MotionModel::size> covariance;
                covariance.rows(MotionModel::PX, MotionModel::PZ) =
                    this->config.motionFilter.initial.covariance.position;
                covariance.rows(MotionModel::VX, MotionModel::VZ) =
                    this->config.motionFilter.initial.covariance.velocity;
                covariance.rows(MotionModel::QW, MotionModel::QZ) =
                    this->config.motionFilter.initial.covariance.rotation;
                covariance.rows(MotionModel::WX, MotionModel::WZ) =
                    this->config.motionFilter.initial.covariance.rotationalVelocity;
                motionFilter.setState(mean, arma::diagmat(covariance));
            });


            // on<Trigger<ResetRobotHypotheses>>()
            //  .then("Localisation ResetRobotHypotheses", [this] {
            //     //this reset's the odometry position when localisation does a reset so that we don't have an odometry
            //     offset form our new position
            //     arma::vec::fixed<MotionModel::size> covariance;
            //     covariance.rows(MotionModel::PX, MotionModel::PZ) =
            //     this->config.motionFilter.initial.covariance.position;
            //     covariance.rows(MotionModel::VX, MotionModel::VZ) =
            //     this->config.motionFilter.initial.covariance.velocity;
            //     covariance.rows(MotionModel::QW, MotionModel::QZ) =
            //     this->config.motionFilter.initial.covariance.rotation;
            //     covariance.rows(MotionModel::WX, MotionModel::WZ) =
            //     this->config.motionFilter.initial.covariance.rotationalVelocity;

            //     arma::vec::fixed<MotionModel::size>  newFilter = motionFilter.get();
            //     newFilter.rows(MotionModel::PX, MotionModel::PY) *= 0.;
            //     motionFilter.setState(newFilter, arma::diagmat(covariance));
            // });

            on<Last<20, Trigger<DarwinSensors>>, Single>().then(
                [this](const std::list<std::shared_ptr<const DarwinSensors>>& sensors) {
                    int leftCount   = 0;
                    int middleCount = 0;

                    // If we have any downs in the last 20 frames then we are button pushed
                    for (const auto& s : sensors) {
                        if (s->buttons.left && !s->cm730ErrorFlags) {
                            ++leftCount;
                        }
                        if (s->buttons.middle && !s->cm730ErrorFlags) {
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

                    // Set our voltage and battery
                    sensors->voltage = input.voltage;

                    // Work out a battery charged percentage
                    sensors->battery = std::max(0.0f,
                                                (input.voltage - config.battery.flatVoltage)
                                                    / (config.battery.chargedVoltage - config.battery.flatVoltage));

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
                                                      previousSensors->servo[i].presentPosition,
                                                      previousSensors->servo[i].presentVelocity,
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

                    // If we have a previous sensors and our cm730 has errors then reuse our last sensor value
                    if (previousSensors && (input.cm730ErrorFlags)) {
                        sensors->accelerometer = previousSensors->accelerometer;
                    }
                    else {
                        sensors->accelerometer = {
                            -input.accelerometer.y, input.accelerometer.x, -input.accelerometer.z};
                    }

                    // If we have a previous sensors and our cm730 has errors then reuse our last sensor value
                    if (previousSensors
                        && (input.cm730ErrorFlags
                            || arma::norm(arma::vec({input.gyroscope.x, input.gyroscope.y, input.gyroscope.z}), 2)
                                   > 4 * M_PI)) {
                        NUClear::log<NUClear::WARN>(
                            "Bad gyroscope value",
                            arma::norm(arma::vec({input.gyroscope.x, input.gyroscope.y, input.gyroscope.z}), 2));
                        sensors->gyroscope = previousSensors->gyroscope;
                    }
                    else {
                        sensors->gyroscope = {input.gyroscope.x, input.gyroscope.y, -input.gyroscope.z};
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

                    auto forwardKinematics = calculateAllPositions(kinematicsModel, *sensors);
                    for (const auto& entry : forwardKinematics) {
                        sensors->forwardKinematics[entry.first] = convert<double, 4, 4>(entry.second);
                    }

                    /************************************************
                     *            Foot down information             *
                     ************************************************/
                    if (previousSensors) {
                        // Use our virtual load sensor class to work out if our foot is down
                        arma::vec leftFootFeatureVec = {
                            sensors->servo[ServoID::L_HIP_PITCH].presentVelocity,
                            sensors->servo[ServoID::L_HIP_PITCH].presentVelocity
                                - previousSensors->servo[ServoID::L_HIP_PITCH].presentVelocity,
                            sensors->servo[ServoID::L_HIP_PITCH].load,
                            sensors->servo[ServoID::L_KNEE].presentVelocity,
                            sensors->servo[ServoID::L_KNEE].presentVelocity
                                - previousSensors->servo[ServoID::L_KNEE].presentVelocity,
                            sensors->servo[ServoID::L_KNEE].load,
                            sensors->servo[ServoID::L_ANKLE_PITCH].presentVelocity,
                            sensors->servo[ServoID::L_ANKLE_PITCH].presentVelocity
                                - previousSensors->servo[ServoID::L_ANKLE_PITCH].presentVelocity,
                            sensors->servo[ServoID::L_ANKLE_PITCH].load};
                        sensors->leftFootDown = leftFootDown.updateFoot(leftFootFeatureVec);

                        arma::vec rightFootFeatureVec = {
                            sensors->servo[ServoID::R_HIP_PITCH].presentVelocity,
                            sensors->servo[ServoID::R_HIP_PITCH].presentVelocity
                                - previousSensors->servo[ServoID::R_HIP_PITCH].presentVelocity,
                            sensors->servo[ServoID::R_HIP_PITCH].load,
                            sensors->servo[ServoID::R_KNEE].presentVelocity,
                            sensors->servo[ServoID::R_KNEE].presentVelocity
                                - previousSensors->servo[ServoID::R_KNEE].presentVelocity,
                            sensors->servo[ServoID::R_KNEE].load,
                            sensors->servo[ServoID::R_ANKLE_PITCH].presentVelocity,
                            sensors->servo[ServoID::R_ANKLE_PITCH].presentVelocity
                                - previousSensors->servo[ServoID::R_ANKLE_PITCH].presentVelocity,
                            sensors->servo[ServoID::R_ANKLE_PITCH].load};
                        sensors->rightFootDown = rightFootDown.updateFoot(rightFootFeatureVec);
                    }
                    else {
                        sensors->leftFootDown  = false;
                        sensors->rightFootDown = false;
                    }

                    /************************************************
                     *             Motion (IMU+Odometry)            *
                     ************************************************/

                    // Calculate our time offset from the last read
                    double deltaT =
                        (input.timestamp - (previousSensors ? previousSensors->timestamp : input.timestamp)).count()
                        / double(NUClear::clock::period::den);

                    // Time update
                    motionFilter.timeUpdate(deltaT);

                    // Accelerometer measurment update
                    motionFilter.measurementUpdate(
                        convert<double, 3>(sensors->accelerometer),
                        config.motionFilter.noise.measurement.accelerometer
                            + arma::norm(convert<double, 3>(sensors->accelerometer))
                                  * config.motionFilter.noise.measurement.accelerometerMagnitude,
                        MotionModel::MeasurementType::ACCELEROMETER());

                    // Gyroscope measurement update
                    motionFilter.measurementUpdate(convert<double, 3>(sensors->gyroscope),
                                                   config.motionFilter.noise.measurement.gyroscope,
                                                   MotionModel::MeasurementType::GYROSCOPE());

                    if (sensors->leftFootDown or sensors->rightFootDown) {
                        // pre-calculate common foot-down variables - these are the torso to world transforms.
                        arma::vec3 rTWw = motionFilter.get().rows(MotionModel::PX, MotionModel::PZ);
                        Rotation3D Rtw(UnitQuaternion(motionFilter.get().rows(MotionModel::QW, MotionModel::QZ)));

                        // 3 points on the ground mean that we can assume this foot is flat
                        // We also have to ensure that the previous foot was also down for this to be valid
                        // Check if our foot is flat on the ground
                        for (auto& side : {ServoSide::LEFT, ServoSide::RIGHT}) {

                            auto servoid = side == ServoSide::LEFT ? ServoID::L_ANKLE_ROLL : ServoID::R_ANKLE_ROLL;

                            const bool& footDown =
                                side == ServoSide::LEFT ? sensors->leftFootDown : sensors->rightFootDown;

                            const bool& prevFootDown = previousSensors ? side == ServoSide::LEFT
                                                                             ? previousSensors->leftFootDown
                                                                             : previousSensors->rightFootDown
                                                                       : false;

                            if (footDown) {
                                Transform3D Htf = convert<double, 4, 4>(sensors->forwardKinematics[servoid]);
                                Transform3D Hft = Htf.i();

                                Rotation3D Rtf  = Htf.rotation();
                                arma::vec3 rFTt = Htf.translation();

                                Rotation3D Rft  = Hft.rotation();
                                arma::vec3 rTFf = Hft.translation();


                                if (!prevFootDown) {
                                    // NOTE: footflat measurements assume the foot is flat on the ground. These
                                    // decorrelate the accelerometer and gyro from translation.
                                    Rotation3D footflat_Rwt = Rotation3D::createRotationZ(Rtw.i().yaw());
                                    Rotation3D footflat_Rtf = Rotation3D::createRotationZ(Rtf.yaw());

                                    // Store the robot foot to world transform
                                    footlanding_Rfw[side] = footflat_Rtf.i() * footflat_Rwt.i();
                                    // Store robot foot in world-delta coordinates
                                    footlanding_Rwf[side]  = footflat_Rwt * footflat_Rtf;
                                    footlanding_rFWw[side] = footlanding_Rwf[side] * rTFf - rTWw;

                                    // Z is an absolute measurement, so we make sure it is an absolute offset
                                    footlanding_rFWw[side][2] = 0.;

                                    // NOTE: an optional foot up with Z calculation can be done here
                                }
                                else {
                                    // NOTE: translation and rotation updates are performed separately so that they can
                                    // be turned off independently for debugging

                                    // encode the old->new torso-world rotation as a quaternion
                                    UnitQuaternion Rtw_new(Rotation3D(Rtf * footlanding_Rfw[side]));

                                    // check if we need to reverse our quaternion
                                    if (arma::norm(Rtw_new + motionFilter.get().rows(MotionModel::QW, MotionModel::QZ))
                                        < 1.) {
                                        Rtw_new *= -1;
                                    }

                                    // do a foot based orientation update
                                    motionFilter.measurementUpdate(
                                        Rtw_new,
                                        config.motionFilter.noise.measurement.flatFootOrientation,
                                        MotionModel::MeasurementType::FLAT_FOOT_ORIENTATION());

                                    // calculate the old -> new world foot position updates
                                    arma::vec3 rFWw = footlanding_Rwf[side] * rTFf - footlanding_rFWw[side];


                                    // do a foot based position update
                                    motionFilter.measurementUpdate(
                                        rFWw,
                                        config.motionFilter.noise.measurement.flatFootOdometry,
                                        MotionModel::MeasurementType::FLAT_FOOT_ODOMETRY());
                                }
                            }
                        }
                    }

                    // emit(graph("LeftFootDown", sensors->leftFootDown));
                    // emit(graph("RightFootDown", sensors->rightFootDown));
                    // emit(graph("LeftLoadState", leftFootDown.state));
                    // emit(graph("RightLoadState", rightFootDown.state));

                    // Gives us the quaternion representation
                    const auto& o = motionFilter.get();

                    // Map from world to torso coordinates (Rtw)
                    Transform3D world;
                    world.eye();
                    world.rotation()    = Rotation3D(UnitQuaternion(o.rows(MotionModel::QW, MotionModel::QZ)));
                    world.translation() = -(world.rotation() * o.rows(MotionModel::PX, MotionModel::PZ));
                    // world.translation() = (o.rows(MotionModel::PX, MotionModel::PZ));
                    sensors->world = convert<double, 4, 4>(world);

                    sensors->robotToIMU = convert<double, 2, 2>(calculateRobotToIMU(world.rotation()));

                    /************************************************
                     *                  Mass Model                  *
                     ************************************************/
                    // FIXME: Causes crashes
                    // sensors->centreOfMass =
                    //     convert<double, 4>(calculateCentreOfMass(kinematicsModel, sensors->forwardKinematics, true));

                    /************************************************
                     *                  Kinematics Horizon          *
                     ************************************************/
                    sensors->bodyCentreHeight = motionFilter.get()[MotionModel::PZ];

                    Rotation3D Rwt = world.rotation().t();  // remove translation components from the transform
                    Rotation3D oBodyToGround = Rotation3D::createRotationZ(-Rwt.yaw()) * Rwt;
                    // sensors->bodyToGround : Mat size [4x4] (default identity)
                    // createRotationZ : Mat size [3x3]
                    // Rwt : Mat size [3x3]
                    sensors->bodyToGround    = convert<double, 4, 4>(Transform3D(oBodyToGround));
                    auto headPitchKinematics = sensors->forwardKinematics[ServoID::HEAD_PITCH];

                    // Get torso to world transform
                    Transform3D worldInv = world.i();

                    Rotation3D yawlessWorldInvR =
                        Rotation3D::createRotationZ(-Rotation3D(worldInv.rotation()).yaw()) * worldInv.rotation();
                    Transform3D torsoToGround   = worldInv;
                    torsoToGround.translation() = arma::vec3({0, 0, torsoToGround.translation()[2]});
                    torsoToGround.rotation()    = yawlessWorldInvR;
                    sensors->camToGround        = convert<double, 4, 4>(
                        Transform3D(torsoToGround * convert<double, 4, 4>(headPitchKinematics)));  // Rwt * Rth

                    /************************************************
                     *                  CENTRE OF PRESSURE          *
                     ************************************************/
                    sensors->centreOfPressure = convert<double, 3>(
                        utility::motion::kinematics::calculateCentreOfPressure(kinematicsModel, *sensors));

                    emit(std::move(sensors));
                });
        }
    }  // namespace darwin
}  // namespace platform
}  // namespace module
