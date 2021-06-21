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

#include "SensorFilter.hpp"

#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"
#include "message/motion/BodySide.hpp"
#include "message/platform/RawSensors.hpp"

#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/motion/ForwardKinematics.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/platform/RawSensors.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::input {

    using extension::Configuration;

    using message::input::Sensors;
    using message::motion::BodySide;
    using message::motion::KinematicsModel;
    using message::platform::ButtonLeftDown;
    using message::platform::ButtonLeftUp;
    using message::platform::ButtonMiddleDown;
    using message::platform::ButtonMiddleUp;
    using message::platform::RawSensors;

    using utility::input::LimbID;
    using utility::input::ServoID;
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


        if (errorCode & RawSensors::Error::INPUT_VOLTAGE) {
            s << " Input Voltage ";
        }
        if (errorCode & RawSensors::Error::ANGLE_LIMIT) {
            s << " Angle Limit ";
        }
        if (errorCode & RawSensors::Error::OVERHEATING) {
            s << " Overheating ";
        }
        if (errorCode & RawSensors::Error::OVERLOAD) {
            s << " Overloaded ";
        }
        if (errorCode & RawSensors::Error::INSTRUCTION) {
            s << " Bad Instruction ";
        }
        if (errorCode & RawSensors::Error::CORRUPT_DATA) {
            s << " Corrupt Data ";
        }
        if (errorCode & RawSensors::Error::TIMEOUT) {
            s << " Timeout ";
        }

        return s.str();
    }

    LegLoad::operator std::string() const {
        switch (value) {
            case Value::FSR: return "FSR";
            case Value::Z_HEIGHT: return "Z_HEIGHT";
            case Value::LOAD: return "LOAD";
            default: throw std::runtime_error("enum LegLoad value is corrupt, unknown value stored");
        }
    }

    SensorFilter::SensorFilter(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), theta(Eigen::Vector3d::Zero()) {

        on<Configuration>("SensorFilter.yaml").then([this](const Configuration& config) {
            this->config.debug = config["debug"].as<bool>();
            // Button config
            this->config.buttons.debounceThreshold = config["buttons"]["debounce_threshold"].as<int>();

            // Foot down config
            const std::string method = config["foot_down"]["method"].as<std::string>();
            std::map<std::string, float> thresholds;
            for (const auto& threshold : config["foot_down"]["known_methods"].config) {
                thresholds[threshold["name"].as<std::string>()] = threshold["certainty_threshold"].as<float>();
            }
            this->config.footDown.set_method(method, thresholds);

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
                Eigen::Vector3d(config["motion_filter"]["noise"]["measurement"]["flat_foot_odometry"].as<Expression>())
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

            // Set our process noise in our filter
            MotionModel<double>::StateVec process_noise;
            process_noise.rTWw               = this->config.motionFilter.noise.process.position;
            process_noise.vTw                = this->config.motionFilter.noise.process.velocity;
            process_noise.Rwt                = this->config.motionFilter.noise.process.rotation;
            process_noise.omegaTTt           = this->config.motionFilter.noise.process.rotationalVelocity;
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

            this->config.motionFilter.initial.covariance.position =
                config["motion_filter"]["initial"]["covariance"]["position"].as<Expression>();
            this->config.motionFilter.initial.covariance.velocity =
                config["motion_filter"]["initial"]["covariance"]["velocity"].as<Expression>();
            this->config.motionFilter.initial.covariance.rotation =
                config["motion_filter"]["initial"]["covariance"]["rotation"].as<Expression>();
            this->config.motionFilter.initial.covariance.rotationalVelocity =
                config["motion_filter"]["initial"]["covariance"]["rotational_velocity"].as<Expression>();

            // Calculate our mean and covariance
            MotionModel<double>::StateVec mean;
            mean.rTWw     = this->config.motionFilter.initial.mean.position;
            mean.vTw      = this->config.motionFilter.initial.mean.velocity;
            mean.Rwt      = this->config.motionFilter.initial.mean.rotation;
            mean.omegaTTt = this->config.motionFilter.initial.mean.rotationalVelocity;

            MotionModel<double>::StateVec covariance;
            covariance.rTWw     = this->config.motionFilter.initial.covariance.position;
            covariance.vTw      = this->config.motionFilter.initial.covariance.velocity;
            covariance.Rwt      = this->config.motionFilter.initial.covariance.rotation;
            covariance.omegaTTt = this->config.motionFilter.initial.covariance.rotationalVelocity;
            motionFilter.set_state(mean.getStateVec(), covariance.asDiagonal());

            // Set our initial position
            rTWw = config["motion_filter"]["initial"]["mean"]["position"].as<Expression>();

            // Don't filter any sensors until we have initialised the filter
            update_loop.disable();
            reset_filter.store(true);
        });

        on<Configuration>("FootDownNetwork.yaml").then([this](const Configuration& config) {
            // Foot load sensor config
            load_sensor = VirtualLoadSensor<float>(config);
        });

        on<Last<20, Trigger<RawSensors>>, With<KinematicsModel>, Single>().then(
            [this](const std::list<std::shared_ptr<const RawSensors>>& sensors, const KinematicsModel& model) {
                // If we need to reset the filter, do that here
                if (reset_filter.load()) {
                    Eigen::Vector3d acc   = Eigen::Vector3d::Zero();
                    Eigen::Vector3d gyro  = Eigen::Vector3d::Zero();
                    Eigen::Vector3d rMFt  = Eigen::Vector3d::Zero();
                    auto filtered_sensors = std::make_unique<Sensors>();

                    for (const auto& s : sensors) {
                        // Accumulate accelerometer and gyroscope readings
                        acc += Eigen::Vector3d(s->accelerometer.x, s->accelerometer.y, s->accelerometer.z);
                        gyro += Eigen::Vector3d(s->gyroscope.x, s->gyroscope.y, s->gyroscope.z);

                        // Make sure we have servo positions
                        for (uint32_t id = 0; id < 20; ++id) {
                            auto& original = utility::platform::getRawServo(id, *s);
                            // Add the sensor values to the system properly
                            filtered_sensors->servo.push_back({0,
                                                               id,
                                                               original.torque_enabled,
                                                               original.p_gain,
                                                               original.i_gain,
                                                               original.d_gain,
                                                               original.goal_position,
                                                               original.moving_speed,
                                                               original.present_position,
                                                               original.present_speed,
                                                               original.load,
                                                               original.voltage,
                                                               float(original.temperature)});
                        }

                        // Calculate forward kinematics
                        const auto Htx = calculateAllPositions(model, *filtered_sensors);
                        for (const auto& entry : Htx) {
                            filtered_sensors->Htx[entry.first] = entry.second.matrix();
                        }

                        // Calculate the average length of both legs from the torso and accumulate this measurement
                        const Eigen::Affine3d Htr(filtered_sensors->Htx[ServoID::R_ANKLE_ROLL]);
                        const Eigen::Affine3d Htl(filtered_sensors->Htx[ServoID::L_ANKLE_ROLL]);
                        const Eigen::Vector3d rTFt = (Htr.translation() + Htl.translation()) * 0.5;

                        // Accumulator CoM readings
                        rMFt += calculateCentreOfMass(model, filtered_sensors->Htx).head<3>() + rTFt;
                    }

                    // Average all accumulated readings
                    acc /= double(sensors.size());
                    gyro /= double(sensors.size());
                    rMFt /= double(sensors.size());

                    // Average time per sensor reading
                    double deltaT = std::chrono::duration_cast<std::chrono::duration<double>>(
                                        sensors.back()->timestamp - sensors.front()->timestamp)
                                        .count()
                                    / double(sensors.size());

                    // Find the rotation from the average accelerometer reading to world UnitZ
                    // Rotating from torso acceleration vector to world z vector ===> this makes it Rwt and not Rtw
                    Eigen::Quaterniond Rwt = Eigen::Quaterniond::FromTwoVectors(acc, Eigen::Vector3d::UnitZ());
                    Rwt.normalize();

                    MotionModel<double>::StateVec mean;
                    // Rotate rMFt (Foot to Torso CoM) into world space
                    mean.rTWw = Rwt.toRotationMatrix() * rMFt;
                    // Remove gravity from accelerometer average and integrate to get velocity
                    mean.vTw = (acc - (Rwt.conjugate() * Eigen::Quaterniond(0.0, 0.0, 0.0, G) * Rwt).vec()) * deltaT;
                    mean.Rwt = Rwt;
                    mean.omegaTTt = gyro;

                    MotionModel<double>::StateVec covariance;
                    covariance.rTWw     = config.motionFilter.initial.covariance.position;
                    covariance.vTw      = config.motionFilter.initial.covariance.velocity;
                    covariance.Rwt      = config.motionFilter.initial.covariance.rotation;
                    covariance.omegaTTt = config.motionFilter.initial.covariance.rotationalVelocity;

                    // We have finished resetting the filter now
                    switch (motionFilter.reset(mean.getStateVec(), covariance.asDiagonal())) {
                        case Eigen::Success:
                            log<NUClear::INFO>("Motion Model UKF has been reset");
                            reset_filter.store(false);
                            update_loop.enable();
                            break;
                        case Eigen::NumericalIssue:
                            log<NUClear::ERROR>(
                                "Cholesky decomposition failed. The provided data did not satisfy the "
                                "prerequisites.");
                            break;
                        case Eigen::NoConvergence:
                            log<NUClear::ERROR>("Cholesky decomposition failed. Iterative procedure did not converge.");
                            break;
                        case Eigen::InvalidInput:
                            log<NUClear::ERROR>(
                                "Cholesky decomposition failed. The inputs are invalid, or the algorithm has been "
                                "improperly called. When assertions are enabled, such errors trigger an assert.");
                            break;
                        default: log<NUClear::ERROR>("Cholesky decomposition failed. Some other reason."); break;
                    }
                }

                int leftCount   = 0;
                int middleCount = 0;

                // If we have any downs in the last 20 frames then we are button pushed
                for (const auto& s : sensors) {
                    if (s->buttons.left && !s->platform_error_flags) {
                        ++leftCount;
                    }
                    if (s->buttons.middle && !s->platform_error_flags) {
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

        update_loop =
            on<Trigger<RawSensors>, Optional<With<Sensors>>, With<KinematicsModel>, Single, Priority::HIGH>()
                .then(
                    "Main Sensors Loop",
                    [this](const RawSensors& input,
                           std::shared_ptr<const Sensors> previousSensors,
                           const KinematicsModel& kinematicsModel) {
                        auto sensors = std::make_unique<Sensors>();

                        /************************************************
                         *                 Raw Sensors                  *
                         ************************************************/

                        // Set our timestamp to when the data was read
                        sensors->timestamp = input.timestamp;

                        sensors->voltage = input.voltage;


                        // This checks for an error on the platform and reports it
                        if (input.platform_error_flags != RawSensors::Error::OK) {
                            NUClear::log<NUClear::WARN>(makeErrorString("Platform", input.platform_error_flags));
                        }

                        // Output errors on the FSRs
                        if (input.fsr.left.error_flags != RawSensors::Error::OK) {
                            NUClear::log<NUClear::WARN>(makeErrorString("Left FSR", input.fsr.left.error_flags));
                        }

                        if (input.fsr.right.error_flags != RawSensors::Error::OK) {
                            NUClear::log<NUClear::WARN>(makeErrorString("Right FSR", input.fsr.right.error_flags));
                        }

                        // Read through all of our sensors
                        for (uint32_t id = 0; id < 20; ++id) {
                            auto& original = utility::platform::getRawServo(id, input);
                            auto& error    = original.error_flags;

                            // Check for an error on the servo and report it
                            if (error != RawSensors::Error::OK) {
                                std::stringstream s;
                                s << "Error on Servo " << (id + 1) << " (" << static_cast<ServoID>(id) << "):";

                                if (error & RawSensors::Error::INPUT_VOLTAGE) {
                                    s << " Input Voltage - " << original.voltage;
                                }
                                if (error & RawSensors::Error::ANGLE_LIMIT) {
                                    s << " Angle Limit - " << original.present_position;
                                }
                                if (error & RawSensors::Error::OVERHEATING) {
                                    s << " Overheating - " << original.temperature;
                                }
                                if (error & RawSensors::Error::OVERLOAD) {
                                    s << " Overloaded - " << original.load;
                                }
                                if (error & RawSensors::Error::INSTRUCTION) {
                                    s << " Bad Instruction ";
                                }
                                if (error & RawSensors::Error::CORRUPT_DATA) {
                                    s << " Corrupt Data ";
                                }
                                if (error & RawSensors::Error::TIMEOUT) {
                                    s << " Timeout ";
                                }

                                NUClear::log<NUClear::WARN>(s.str());
                            }

                            // If we have previous sensors and our current sensors have an error
                            // we then use our previous sensor values with some updates
                            if (previousSensors && error != RawSensors::Error::OK) {
                                // Add the sensor values to the system properly
                                sensors->servo.push_back({error,
                                                          id,
                                                          original.torque_enabled,
                                                          original.p_gain,
                                                          original.i_gain,
                                                          original.d_gain,
                                                          original.goal_position,
                                                          original.moving_speed,
                                                          previousSensors->servo[id].present_position,
                                                          previousSensors->servo[id].present_velocity,
                                                          previousSensors->servo[id].load,
                                                          previousSensors->servo[id].voltage,
                                                          previousSensors->servo[id].temperature});
                            }
                            // Otherwise we can just use the new values as is
                            else {
                                // Add the sensor values to the system properly
                                sensors->servo.push_back({error,
                                                          id,
                                                          original.torque_enabled,
                                                          original.p_gain,
                                                          original.i_gain,
                                                          original.d_gain,
                                                          original.goal_position,
                                                          original.moving_speed,
                                                          original.present_position,
                                                          original.present_speed,
                                                          original.load,
                                                          original.voltage,
                                                          float(original.temperature)});
                            }
                        }

                        /************************************************
                         *          Accelerometer and Gyroscope         *
                         ************************************************/

                        // We assume that the accelerometer and gyroscope are oriented to conform with the standard
                        // coordinate system x-axis out the front of the robot y-axis to the left z-axis up
                        //
                        // For the accelerometer the orientation should be as follows
                        // x axis reports a +1g acceleration when robot is laying on its back
                        // y axis reports a +1g acceleration when robot is laying on its right side
                        // z axis reports a +1g acceleration when robot is vertical

                        // If we have a previous sensors and our platform has errors then reuse our last sensor value
                        if (previousSensors && (input.platform_error_flags)) {
                            sensors->accelerometer = previousSensors->accelerometer;
                        }
                        else {
                            sensors->accelerometer =
                                Eigen::Vector3d(input.accelerometer.x, input.accelerometer.y, input.accelerometer.z);
                        }

                        // If we have a previous sensors and our platform has errors then reuse our last sensor value
                        if (previousSensors
                            && (input.platform_error_flags
                                || Eigen::Vector3d(input.gyroscope.x, input.gyroscope.y, input.gyroscope.z).norm()
                                       > 4.0 * M_PI)) {
                            NUClear::log<NUClear::WARN>(
                                "Bad gyroscope value",
                                Eigen::Vector3d(input.gyroscope.x, input.gyroscope.y, input.gyroscope.z).norm());
                            sensors->gyroscope = previousSensors->gyroscope;
                        }
                        else {
                            sensors->gyroscope =
                                Eigen::Vector3d(input.gyroscope.x, input.gyroscope.y, input.gyroscope.z);
                        }

                        /************************************************
                         *               Buttons and LEDs               *
                         ************************************************/
                        sensors->button.reserve(2);
                        sensors->button.push_back(Sensors::Button(0, input.buttons.left));
                        sensors->button.push_back(Sensors::Button(1, input.buttons.middle));
                        sensors->led.reserve(5);
                        sensors->led.push_back(Sensors::LED(0, input.led_panel.led2 ? 0xFF0000 : 0));
                        sensors->led.push_back(Sensors::LED(1, input.led_panel.led3 ? 0xFF0000 : 0));
                        sensors->led.push_back(Sensors::LED(2, input.led_panel.led4 ? 0xFF0000 : 0));
                        sensors->led.push_back(Sensors::LED(3, input.head_led.RGB));  // Head
                        sensors->led.push_back(Sensors::LED(4, input.eye_led.RGB));   // Eye

                        /************************************************
                         *                  Kinematics                  *
                         ************************************************/

                        auto Htx = calculateAllPositions(kinematicsModel, *sensors);
                        for (const auto& entry : Htx) {
                            sensors->Htx[entry.first] = entry.second.matrix();
                        }

                        /************************************************
                         *                  Mass Model                  *
                         ************************************************/
                        sensors->rMTt           = calculateCentreOfMass(kinematicsModel, sensors->Htx);
                        sensors->inertia_tensor = calculateInertialTensor(kinematicsModel, sensors->Htx);

                        /************************************************
                         *            Foot down information             *
                         ************************************************/
                        sensors->feet.resize(2);
                        sensors->feet[BodySide::RIGHT].down = true;
                        sensors->feet[BodySide::LEFT].down  = true;

                        std::array<bool, 2> feet_down = {true, true};
                        if (config.footDown.method() == "LOAD") {
                            // Use our virtual load sensor class to work out which feet are down
                            feet_down = load_sensor.updateFeet(*sensors);
                        }
                        else if (config.footDown.method() == "Z_HEIGHT") {
                            Eigen::Affine3d Htr(sensors->Htx[ServoID::R_ANKLE_ROLL]);
                            Eigen::Affine3d Htl(sensors->Htx[ServoID::L_ANKLE_ROLL]);
                            Eigen::Affine3d Hlr  = Htl.inverse() * Htr;
                            Eigen::Vector3d rRLl = Hlr.translation();

                            // Right foot is below left foot in left foot space
                            if (rRLl.z() < -config.footDown.threshold()) {
                                feet_down[BodySide::RIGHT] = true;
                                feet_down[BodySide::LEFT]  = false;
                            }
                            // Right foot is above left foot in left foot space
                            else if (rRLl.z() > config.footDown.threshold()) {
                                feet_down[BodySide::RIGHT] = false;
                                feet_down[BodySide::LEFT]  = true;
                            }
                            // Right foot and left foot are roughly the same height in left foot space
                            else {
                                feet_down[BodySide::RIGHT] = true;
                                feet_down[BodySide::LEFT]  = true;
                            }
                        }
                        else if (config.footDown.method() == "FSR") {
                            // For a foot to be on the ground we want a minimum of 2 diagonally opposite studs
                            // in contact with the ground
                            // So fsr1 and fsr3, or fsr2 and fsr4
                            //
                            // A FSR is in contact with the ground if its value is greater than the certainty threshold

                            feet_down[BodySide::LEFT] = (((input.fsr.left.fsr1 > config.footDown.threshold())
                                                          && (input.fsr.left.fsr3 > config.footDown.threshold()))
                                                         || ((input.fsr.left.fsr2 > config.footDown.threshold())
                                                             && (input.fsr.left.fsr4 > config.footDown.threshold())));

                            feet_down[BodySide::RIGHT] = (((input.fsr.right.fsr1 > config.footDown.threshold())
                                                           && (input.fsr.right.fsr3 > config.footDown.threshold()))
                                                          || ((input.fsr.right.fsr2 > config.footDown.threshold())
                                                              && (input.fsr.right.fsr4 > config.footDown.threshold())));
                        }

                        if (this->config.debug) {
                            emit(graph(fmt::format("Sensor/Foot Down/{}/Left", config.footDown.method()),
                                       feet_down[BodySide::LEFT]));
                            emit(graph(fmt::format("Sensor/Foot Down/{}/Right", config.footDown.method()),
                                       feet_down[BodySide::RIGHT]));
                        }

                        sensors->feet[BodySide::RIGHT].down = feet_down[BodySide::RIGHT];
                        sensors->feet[BodySide::LEFT].down  = feet_down[BodySide::LEFT];

                        /************************************************
                         *             Motion (IMU+Odometry)            *
                         ************************************************/

                        // Gyroscope measurement update
                        motionFilter.measure(sensors->gyroscope,
                                             config.motionFilter.noise.measurement.gyroscope,
                                             MeasurementType::GYROSCOPE());

                        // Calculate accelerometer noise factor
                        Eigen::Matrix3d acc_noise =
                            config.motionFilter.noise.measurement.accelerometer
                            + ((sensors->accelerometer.norm() - std::abs(G))
                               * (sensors->accelerometer.norm() - std::abs(G)))
                                  * config.motionFilter.noise.measurement.accelerometerMagnitude;

                        // Accelerometer measurement update
                        motionFilter.measure(sensors->accelerometer, acc_noise, MeasurementType::ACCELEROMETER());

                        for (const auto& side : {BodySide::LEFT, BodySide::RIGHT}) {
                            bool foot_down      = sensors->feet[side].down;
                            bool prev_foot_down = previous_foot_down[side];
                            Eigen::Affine3d Htf(
                                sensors->Htx[side == BodySide::LEFT ? ServoID::L_ANKLE_ROLL : ServoID::R_ANKLE_ROLL]);

                            if (foot_down && !prev_foot_down) {
                                const auto filterState = MotionModel<double>::StateVec(motionFilter.get());
                                Eigen::Affine3d Hwt;
                                Hwt.linear()      = filterState.Rwt.toRotationMatrix();
                                Hwt.translation() = filterState.rTWw;

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

                            sensors->feet[side].Hwf = footlanding_Hwf[side].matrix();
                        }

                        // Calculate our time offset from the last read
                        double deltaT = std::max(
                            (input.timestamp - (previousSensors ? previousSensors->timestamp : input.timestamp)).count()
                                / double(NUClear::clock::period::den),
                            0.0);

                        // Time update
                        switch (motionFilter.time(deltaT)) {
                            case Eigen::Success: break;
                            case Eigen::NumericalIssue:
                                log<NUClear::ERROR>(
                                    "Cholesky decomposition failed. The provided data did not satisfy the "
                                    "prerequisites.");
                                // Disable the sensor update loop to reset the filter post cholesky
                                update_loop.disable();
                                reset_filter.store(true);
                                break;
                            case Eigen::NoConvergence:
                                log<NUClear::ERROR>(
                                    "Cholesky decomposition failed. Iterative procedure did not converge.");
                                // Disable the sensor update loop to reset the filter post cholesky
                                update_loop.disable();
                                reset_filter.store(true);
                                break;
                            case Eigen::InvalidInput:
                                log<NUClear::ERROR>(
                                    "Cholesky decomposition failed. The inputs are invalid, or the algorithm has been "
                                    "improperly called. When assertions are enabled, such errors trigger an assert.");
                                // Disable the sensor update loop to reset the filter post cholesky
                                update_loop.disable();
                                reset_filter.store(true);
                                break;
                            default:
                                log<NUClear::ERROR>("Cholesky decomposition failed. Some other reason.");
                                // Disable the sensor update loop to reset the filter post cholesky
                                update_loop.disable();
                                reset_filter.store(true);
                                break;
                        }

                        // Filter is not reliable, just use previous sensors
                        if (reset_filter.load()) {
                            if (previousSensors) {
                                sensors->Htw              = previousSensors->Htw;
                                sensors->Hgt              = previousSensors->Hgt;
                                sensors->Hgc              = previousSensors->Hgc;
                                sensors->angular_position = previousSensors->angular_position;
                            }
                        }
                        else {
                            /************************************************
                             *       Torso CoM Position in World (rMWw)     *
                             ************************************************/
                            bool update_done = false;

                            for (const auto& side : {BodySide::LEFT, BodySide::RIGHT}) {
                                const bool& foot_down      = sensors->feet[side].down;
                                const bool& prev_foot_down = previous_foot_down[side];

                                // Get the Foot to Torso transform for this foot
                                const Eigen::Affine3d Htf(sensors->Htx[side == BodySide::LEFT ? ServoID::L_ANKLE_ROLL
                                                                                              : ServoID::R_ANKLE_ROLL]);

                                // Calculate our current Foot to CoM vector for this foot
                                const Eigen::Vector3d current_rMFt =
                                    Htf.translation().head<3>() + sensors->rMTt.head<3>();

                                // We just put this foot on the ground (i.e. it wasn't on the ground in the last time
                                // step)
                                if (foot_down && !prev_foot_down) {
                                    // Update our Foot to CoM vector for this foot
                                    rMFt[side]               = current_rMFt;
                                    previous_foot_down[side] = true;
                                }
                                // Our foot is on the ground and was also on the ground in the last time step
                                else if (foot_down && prev_foot_down) {
                                    // If both feet are on the ground then we don't need to do another update
                                    if (!update_done) {
                                        // The difference between our current and previous Foot to torso CoM vectors is
                                        // how much our torso has moved in the last time step in torso space We need to
                                        // rotate this into world space to update our current Torso CoM position
                                        const Eigen::Quaterniond& Rwt =
                                            MotionModel<double>::StateVec(motionFilter.get()).Rwt;
                                        const Eigen::Vector3d rMFt_update = current_rMFt - rMFt[side];
                                        const Eigen::Quaterniond q(
                                            Eigen::Vector4d(rMFt_update.x(), rMFt_update.y(), rMFt_update.z(), 0.0));
                                        rTWw += (Rwt * q * Rwt.conjugate()).vec();

                                        // Make sure we don't do another update
                                        update_done = true;
                                    }
                                    previous_foot_down[side] = true;
                                    rMFt[side]               = current_rMFt;
                                }
                                else {
                                    previous_foot_down[side] = false;
                                }
                            }

                            /************************************************
                             *       Construct Odometry Output (Htw)        *
                             ************************************************/
                            // Gives us the quaternion representation
                            const auto o = MotionModel<double>::StateVec(motionFilter.get());

                            // Map from world to torso coordinates (Rtw)
                            Eigen::Affine3d Hwt;
                            Hwt.linear()      = o.Rwt.toRotationMatrix();
                            Hwt.translation() = o.rTWw;
                            sensors->Htw      = Hwt.inverse().matrix();

                            // Integrate gyro to get angular positions
                            sensors->angular_position = o.omegaTTt / 90.0;

                            if (this->config.debug) {
                                log("p_x:",
                                    sensors->angular_position.x(),
                                    "p_y:",
                                    sensors->angular_position.y(),
                                    "p_z:",
                                    sensors->angular_position.z());
                            }

                            /************************************************
                             *                  Kinematics Horizon          *
                             ************************************************/
                            Eigen::Affine3d Rwt(sensors->Htw.inverse());
                            // remove translation components from the transform
                            Rwt.translation() = Eigen::Vector3d::Zero();
                            Eigen::Affine3d Rgt(
                                Eigen::AngleAxisd(-Rwt.rotation().eulerAngles(0, 1, 2).z(), Eigen::Vector3d::UnitZ())
                                * Rwt);
                            // sensors->Hgt : Mat size [4x4] (default identity)
                            // createRotationZ : Mat size [3x3]
                            // Rwt : Mat size [3x3]
                            sensors->Hgt = Rgt.matrix();
                            auto Htc     = sensors->Htx[ServoID::HEAD_PITCH];

                            // Get torso to world transform
                            Eigen::Affine3d yawlessWorldInvR(
                                Eigen::AngleAxisd(-Hwt.rotation().eulerAngles(0, 1, 2).z(), Eigen::Vector3d::UnitZ())
                                * Hwt.rotation());
                            Eigen::Affine3d Hgt(Hwt);
                            Hgt.translation() = Eigen::Vector3d(0, 0, Hwt.translation().z());
                            Hgt.linear()      = yawlessWorldInvR.linear();
                            sensors->Hgc      = Hgt * Htc;  // Rwt * Rth
                        }

                        emit(std::move(sensors));
                    })
                .disable();
    }
}  // namespace module::input
