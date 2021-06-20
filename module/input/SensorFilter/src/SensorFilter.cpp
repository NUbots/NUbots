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

    SensorFilter::SensorFilter(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), theta(Eigen::Vector3d::Zero()) {

        on<Configuration>("SensorFilter.yaml").then([this](const Configuration& config) {
            this->config.debug = config["debug"].as<bool>();
            // Button config
            this->config.buttons.debounceThreshold = config["buttons"]["debounce_threshold"].as<int>();

            // Foot down config
            this->config.footDown.fromLoad           = config["foot_down"]["from_load"].as<bool>();
            this->config.footDown.certaintyThreshold = config["foot_down"]["certainty_threshold"].as<float>();

            // Set velocity decay
            this->config.motionFilter.velocityDecay =
                config["motion_filter"]["update"]["velocity_decay"].as<Expression>();
            motionFilter.model.timeUpdateVelocityDecay = this->config.motionFilter.velocityDecay;

            // Set our measurement noises
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

            // Set our process noises
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

            // Set our initial process noise in our filter
            MotionModel<double>::StateVec process_noise;
            process_noise.rTWw               = this->config.motionFilter.noise.process.position;
            process_noise.vTw                = this->config.motionFilter.noise.process.velocity;
            process_noise.Rwt                = this->config.motionFilter.noise.process.rotation;
            process_noise.omegaTTt           = this->config.motionFilter.noise.process.rotationalVelocity;
            process_noise.omegaTTt_bias      = this->config.motionFilter.noise.process.gyroscopeBias;
            motionFilter.model.process_noise = process_noise;

            // Set our initial means
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

            // Set our initial mean
            MotionModel<double>::StateVec mean;
            mean.rTWw          = this->config.motionFilter.initial.mean.position;
            mean.vTw           = this->config.motionFilter.initial.mean.velocity;
            mean.Rwt           = this->config.motionFilter.initial.mean.rotation;
            mean.omegaTTt      = this->config.motionFilter.initial.mean.rotationalVelocity;
            mean.omegaTTt_bias = this->config.motionFilter.initial.mean.gyroscopeBias;

            // Set our initial covariance matrix, as a diagonal matrix
            MotionModel<double>::StateVec covariance;
            covariance.rTWw          = this->config.motionFilter.initial.covariance.position;
            covariance.vTw           = this->config.motionFilter.initial.covariance.velocity;
            covariance.Rwt           = this->config.motionFilter.initial.covariance.rotation;
            covariance.omegaTTt      = this->config.motionFilter.initial.covariance.rotationalVelocity;
            covariance.omegaTTt_bias = this->config.motionFilter.initial.covariance.gyroscopeBias;
            // Set the filter's state with those initial state parameters
            motionFilter.set_state(mean.getStateVec(), covariance.asDiagonal());

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
                        for (uint32_t i = 0; i < 20; ++i) {
                            auto& original = utility::platform::getRawServo(i, *s);
                            // Add the sensor values to the system properly
                            filtered_sensors->servo.push_back({0,
                                                               i,
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

                    // Find the rotation from the average accelerometer reading to world UnitZ
                    // Rotating from torso acceleration vector to world z vector ===> torso to world rotation
                    Eigen::Quaterniond Rwt = Eigen::Quaterniond::FromTwoVectors(acc, Eigen::Vector3d::UnitZ());
                    Rwt.normalize();

                    MotionModel<double>::StateVec mean;
                    // Rotate rMFt into world space. We are assuming that CoM (M) and torso are close enough to each
                    // other
                    mean.rTWw          = Rwt.toRotationMatrix() * rMFt;
                    mean.vTw           = Eigen::Vector3d::Zero();
                    mean.Rwt           = Rwt;
                    mean.omegaTTt      = Eigen::Vector3d::Zero();
                    mean.omegaTTt_bias = Eigen::Vector3d::Zero();

                    MotionModel<double>::StateVec covariance;
                    covariance.rTWw          = this->config.motionFilter.initial.covariance.position;
                    covariance.vTw           = this->config.motionFilter.initial.covariance.velocity;
                    covariance.Rwt           = this->config.motionFilter.initial.covariance.rotation;
                    covariance.omegaTTt      = this->config.motionFilter.initial.covariance.rotationalVelocity;
                    covariance.omegaTTt_bias = this->config.motionFilter.initial.covariance.gyroscopeBias;

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

                const bool newLeftDown   = leftCount > config.buttons.debounceThreshold;
                const bool newMiddleDown = middleCount > config.buttons.debounceThreshold;

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

                        for (uint32_t id = 0; id < 20; ++id) {
                            const auto& original = utility::platform::getRawServo(id, input);
                            const auto& error    = original.error_flags;

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

                            // If we have a previous Sensors message and our current Sensors message for this servo has
                            // an error, then we use our previous sensor values with some updates
                            if (error != RawSensors::Error::OK && previousSensors) {
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
                            // Otherwise we use the new values as is
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

                        // If we have a previous Sensors message and our platform has errors, then reuse our last sensor
                        // value
                        if (input.platform_error_flags && previousSensors) {
                            sensors->accelerometer = previousSensors->accelerometer;
                        }
                        else {
                            sensors->accelerometer =
                                Eigen::Vector3d(input.accelerometer.x, input.accelerometer.y, input.accelerometer.z);
                        }

                        // If we have a previous Sensors message and (our platform has errors or we are spinning too
                        // quickly), then reuse our last sensor value
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

                        // Htx is a map from ServoID to homogeneous transforms from each ServoID to the torso
                        auto Htx = calculateAllPositions(kinematicsModel, *sensors);
                        // copying the map to the sensors->Htx Map
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

                        // If we're using the load value on the foot to work out if our foot is down, do that
                        if (config.footDown.fromLoad) {
                            // Use our load sensor to work out which foot is down
                            feet_down = load_sensor.updateFeet(*sensors);

                            if (this->config.debug) {
                                emit(graph("Sensor/Foot Down/Load/Left", feet_down[BodySide::LEFT]));
                                emit(graph("Sensor/Foot Down/Load/Right", feet_down[BodySide::RIGHT]));
                            }
                        }
                        // Otherwise, guess which foot is down by comparing the feet positions to the
                        // footDown certainty threshold
                        else {
                            const Eigen::Affine3d Htr(sensors->Htx[ServoID::R_ANKLE_ROLL]);
                            const Eigen::Affine3d Htl(sensors->Htx[ServoID::L_ANKLE_ROLL]);
                            const Eigen::Affine3d Hlr  = Htl.inverse() * Htr;
                            const Eigen::Vector3d rRLl = Hlr.translation();

                            // Right foot is below left foot in left foot space by more than the certainty threshold
                            if (rRLl.z() < -config.footDown.certaintyThreshold) {
                                feet_down[BodySide::RIGHT] = true;
                                feet_down[BodySide::LEFT]  = false;
                            }
                            // Right foot is above left foot in left foot space by more than the certainty threshold
                            else if (rRLl.z() > config.footDown.certaintyThreshold) {
                                feet_down[BodySide::RIGHT] = false;
                                feet_down[BodySide::LEFT]  = true;
                            }
                            // Right foot and left foot are roughly the same height in left foot space
                            else {
                                feet_down[BodySide::RIGHT] = true;
                                feet_down[BodySide::LEFT]  = true;
                            }

                            if (this->config.debug) {
                                emit(graph("Sensor/Foot Down/Z/Left", feet_down[BodySide::LEFT]));
                                emit(graph("Sensor/Foot Down/Z/Right", feet_down[BodySide::RIGHT]));
                            }
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
                        const Eigen::Matrix3d acc_noise =
                            config.motionFilter.noise.measurement.accelerometer
                            // Add noise which is proportional to the square of how much we are moving, minus gravity
                            // This means that the more we're accelerating, the noisier we think the measurements are
                            + ((sensors->accelerometer.norm() - std::abs(G))
                               * (sensors->accelerometer.norm() - std::abs(G)))
                                  * config.motionFilter.noise.measurement.accelerometerMagnitude;

                        // Accelerometer measurement update
                        motionFilter.measure(sensors->accelerometer, acc_noise, MeasurementType::ACCELEROMETER());
                        // This loop calculates the Hwf transform for feet if they have just hit the ground. If they
                        // have not just hit the ground, it uses the previous Hwf value. This assumes that once the foot
                        // hits the ground, it doesn't move at all i.e. we're ASSUMING the foot cannot slip/slide
                        for (auto&& side : {BodySide::LEFT, BodySide::RIGHT}) {
                            const bool foot_down      = sensors->feet[side].down;
                            const bool prev_foot_down = previous_foot_down[side];
                            const Eigen::Affine3d Htf(
                                sensors->Htx[side == BodySide::LEFT ? ServoID::L_ANKLE_ROLL : ServoID::R_ANKLE_ROLL]);
                            // If this side's foot is down, and it was not down at the previous time step, then we
                            // calculate our new footlanding_Hwf value, because our foot has just landed
                            if (foot_down && !prev_foot_down) {
                                const auto filterState = MotionModel<double>::StateVec(motionFilter.get());
                                Eigen::Affine3d Hwt;
                                Hwt.linear()      = filterState.Rwt.toRotationMatrix();
                                Hwt.translation() = filterState.rTWw;
                                // Htg is intended to be such that the "foot down" position is where the foot would be
                                // if it were flat, even if it's not flat when first touches the ground. As the foot
                                // flattens, it's meant to becomes true. This means that even if the foot hits the
                                // ground at an angle, it doesn't store that angled position as the footlanding_Hwf, but
                                // instead stores the position that foot would be if/when it becomes flat on the ground
                                Eigen::Affine3d Htg(utility::motion::kinematics::calculateGroundSpace(Htf, Hwt));

                                footlanding_Hwf[side]                   = Hwt * Htg;
                                footlanding_Hwf[side].translation().z() = 0.0;

                                // Store the current foot down state for next time
                                previous_foot_down[side] = true;
                            }
                            // This sides foot is down, but it didn't hit the ground this time step
                            else if (foot_down && prev_foot_down) {
                                // Use stored Hwf and Htf to calculate Hwt
                                const Eigen::Affine3d footlanding_Hwt = footlanding_Hwf[side] * Htf.inverse();

                                // do a foot based position update
                                motionFilter.measure(Eigen::Vector3d(footlanding_Hwt.translation()),
                                                     config.motionFilter.noise.measurement.flatFootOdometry,
                                                     MeasurementType::FLAT_FOOT_ODOMETRY());

                                // do a foot based orientation update
                                const Eigen::Quaterniond Rwt(footlanding_Hwt.linear());
                                motionFilter.measure(Rwt.coeffs(),
                                                     config.motionFilter.noise.measurement.flatFootOrientation,
                                                     MeasurementType::FLAT_FOOT_ORIENTATION());
                            }
                            // Otherwise this side's foot is off the ground, so we make sure that for the next time
                            // step, we know that this time step, the foot was off the ground
                            else if (!foot_down) {
                                previous_foot_down[side] = false;
                            }
                            // Note that the Hwf is set, even if the foot is not down. This means that moving feet in
                            // the air will have an Hwf associated with them which is the transform from when that foot
                            // last hit the ground
                            sensors->feet[side].Hwf = footlanding_Hwf[side].matrix();
                        }

                        // Calculate our time offset from the last read then update the filter's time
                        {
                            using namespace std::chrono;
                            const double deltaT = std::max(
                                duration_cast<duration<double>>(
                                    input.timestamp - (previousSensors ? previousSensors->timestamp : input.timestamp))
                                    .count(),
                                0.0);

                            // Time update
                            switch (motionFilter.time(deltaT)) {
                                case Eigen::Success: break;
                                case Eigen::NumericalIssue:
                                    log<NUClear::ERROR>(
                                        "Cholesky decomposition failed. The provided data did not satisfy the "
                                        "prerequisites.");
                                    update_loop.disable();
                                    reset_filter.store(true);
                                    break;
                                case Eigen::NoConvergence:
                                    log<NUClear::ERROR>(
                                        "Cholesky decomposition failed. Iterative procedure did not converge.");
                                    update_loop.disable();
                                    reset_filter.store(true);
                                    break;
                                case Eigen::InvalidInput:
                                    log<NUClear::ERROR>(
                                        "Cholesky decomposition failed. The inputs are invalid, or the algorithm has "
                                        "been "
                                        "improperly called. When assertions are enabled, such errors trigger an "
                                        "assert.");
                                    update_loop.disable();
                                    reset_filter.store(true);
                                    break;
                                default:
                                    log<NUClear::ERROR>("Cholesky decomposition failed. Some other reason.");
                                    update_loop.disable();
                                    reset_filter.store(true);
                                    break;
                            }
                        }

                        // Filter is not reliable, just use previous sensors
                        if (reset_filter.load()) {
                            if (previousSensors) {
                                sensors->Htw = previousSensors->Htw;
                                sensors->Hgt = previousSensors->Hgt;
                            }
                        }
                        else {
                            // Convert the motion filter's state vector to a nicer representation, so we can access its
                            // elements
                            const auto o = MotionModel<double>::StateVec(motionFilter.get());

                            // We make Hwt first, because `o` is in world space
                            Eigen::Affine3d Hwt;
                            Hwt.linear()      = o.Rwt.toRotationMatrix();
                            Hwt.translation() = o.rTWw;
                            sensors->Htw      = Hwt.inverse().matrix();

                            /************************************************
                             *                  Calculate Hgt               *
                             ************************************************/
                            Eigen::Affine3d Rwt(sensors->Htw.inverse());
                            // remove translation components from the transform
                            Rwt.translation() = Eigen::Vector3d::Zero();
                            Eigen::Affine3d Rgt(
                                Eigen::AngleAxisd(-Rwt.rotation().eulerAngles(0, 1, 2).z(), Eigen::Vector3d::UnitZ())
                                * Rwt);
                            sensors->Hgt = Rgt.matrix();
                        }

                        emit(std::move(sensors));
                    })
                .disable();
    }
}  // namespace module::input
