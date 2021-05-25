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
#include "message/platform//RawSensors.hpp"

#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/math/quaternion.hpp"
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

    /// @brief Calculates the mean state, by averaging the vectors and calculating the mean quaternion
    /// @param states The states being averaged
    MotionModel<double>::StateVec calculateMeanState(std::vector<MotionModel<double>::StateVec> states) {

        // We'll add the vectors to a zero-initialised state, then coefficient-wise divide them by the number of states
        MotionModel<double>::StateVec mean_state{};
        // We'll make a vector of the quaternions to take the mean of them
        std::vector<Eigen::Quaterniond> rotations{};

        for (const auto& state : states) {
            // clang-format off
            mean_state.rTWw          += state.rTWw;
            mean_state.vTw           += state.vTw;
            mean_state.omegaTTt      += state.omegaTTt;
            mean_state.omegaTTt_bias += state.omegaTTt_bias;
            // clang-format on
            rotations.push_back(state.Rwt);
        }

        // clang-format off
        mean_state.rTWw     /= states.size();
        mean_state.vTw      /= states.size();
        mean_state.omegaTTt /= states.size();
        mean_state.omegaTTt /= states.size();
        // clang-format on
        mean_state.Rwt = utility::math::quaternion::mean(rotations.begin(), rotations.end());

        return mean_state;
    }

    MotionModel<double>::StateVec sensorsToState(RawSensors sensors) {
        MotionModel<double>::StateVec state;

        // TODO: I have no idea how to construct a state from sensors
        // maybe look at sims/kalman_filter for ideas
        return state;
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

            // Set our initial noise vector
            MotionModel<double>::StateVec process_noise;
            process_noise.rTWw               = this->config.motionFilter.noise.process.position;
            process_noise.vTw                = this->config.motionFilter.noise.process.velocity;
            process_noise.Rwt                = this->config.motionFilter.noise.process.rotation;
            process_noise.omegaTTt           = this->config.motionFilter.noise.process.rotationalVelocity;
            process_noise.omegaTTt_bias      = this->config.motionFilter.noise.process.gyroscopeBias;
            motionFilter.model.process_noise = process_noise;

            // Configuration is a reset event
            updates_since_reset_event = 0;
        });

        on<Configuration>("FootDownNetwork.yaml").then([this](const Configuration& config) {
            // Foot load sensor config
            load_sensor = VirtualLoadSensor<float>(config);
        });

        on<Last<20, Trigger<RawSensors>>, Single>().then(
            [this](const std::list<std::shared_ptr<const RawSensors>>& sensors) {
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

        on<Trigger<RawSensors>, Optional<With<Sensors>>, With<KinematicsModel>, Single, Priority::HIGH>().then(
            "Main Sensors Loop",
            [this](const RawSensors& input,
                   std::shared_ptr<const Sensors> previousSensors,
                   const KinematicsModel& kinematicsModel) {
                // If we have a reset event recently, then we average the next MIN_UPDATES to set an initial position
                if (updates_since_reset_event < MIN_UPDATES) {
                    first_updates.push_back(sensorsToState(input));
                    updates_since_reset_event++;
                    return;
                }
                // Once we have the minimum number of updates since the last reset event, we actually reset the filter
                // Then, we can use the filter as normal
                else if (updates_since_reset_event == MIN_UPDATES) {

                    const auto& mean_state = calculateMeanState(first_updates);

                    // We use our config covariance as the diagonal for an initial covariance matrix
                    MotionModel<double>::StateVec covariance;
                    covariance.rTWw          = this->config.motionFilter.initial.covariance.position;
                    covariance.vTw           = this->config.motionFilter.initial.covariance.velocity;
                    covariance.Rwt           = this->config.motionFilter.initial.covariance.rotation;
                    covariance.omegaTTt      = this->config.motionFilter.initial.covariance.rotationalVelocity;
                    covariance.omegaTTt_bias = this->config.motionFilter.initial.covariance.gyroscopeBias;

                    // Reset the filter
                    Eigen::ComputationInfo cholesky_result = motionFilter.reset(mean_state.getStateVec(),
                                                                                covariance.asDiagonal(),
                                                                                motionFilter.ALPHA_DEFAULT,
                                                                                motionFilter.KAPPA_DEFAULT,
                                                                                motionFilter.BETA_DEFAULT);

                    // If a Cholesky decomposition failed, log the error, then wait another MIN_UPDATES and try again
                    switch (cholesky_result) {
                        // Cholesky decomp successful. Business as usual
                        case Eigen::Success: break;
                        // Other results are bad news, and require a motion filter reset
                        case Eigen::NumericalIssue:
                            NUClear::log<NUClear::WARN>(
                                "Cholesky decomposition failed. The provided data did not satisfy the "
                                "prerequisites.");
                            updates_since_reset_event = 0;
                            break;
                        case Eigen::NoConvergence:
                            NUClear::log<NUClear::WARN>(
                                "Cholesky decomposition failed. Iterative procedure did not converge.");
                            updates_since_reset_event = 0;
                            break;
                        case Eigen::InvalidInput:
                            NUClear::log<NUClear::WARN>(
                                "Cholesky decomposition failed. The inputs are invalid, or the algorithm has "
                                "been improperly called. When assertions are enabled, such errors trigger an "
                                "assert.");
                            updates_since_reset_event = 0;
                            break;
                        default:
                            NUClear::log<NUClear::WARN>("Cholesky decomposition failed. Some other reason.");
                            updates_since_reset_event = 0;
                            break;
                    }

                    // Reset the first updates vector, since we want it empty at the next reset event
                    first_updates.clear();
                }
                updates_since_reset_event++;

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

                // Loop through all the servos, emitting a message::input::Sensors for each one
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

                    // If we have a previous Sensors message and our current Sensors message for this servo has an
                    // error, then we use our previous sensor values with some updates
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

                // We assume that the accelerometer and gyroscope are oriented to conform with the standard coordinate
                // system
                // x-axis out the front of the robot
                // y-axis to the left
                // z-axis up
                //
                // For the accelerometer the orientation should be as follows
                // x axis reports a +1g acceleration when robot is laying on its back
                // y axis reports a +1g acceleration when robot is laying on its right side
                // z axis reports a +1g acceleration when robot is vertical

                // If we have a previous Sensors message and our platform has errors, then reuse our last sensor value
                if (previousSensors && (input.platform_error_flags)) {
                    sensors->accelerometer = previousSensors->accelerometer;
                }
                else {
                    sensors->accelerometer =
                        Eigen::Vector3d(input.accelerometer.x, input.accelerometer.y, input.accelerometer.z);
                }

                // If we have a previous Sensors message and (our platform has errors or we are spinning too quickly),
                // then reuse our last sensor value
                if (previousSensors
                    && (input.platform_error_flags
                        // One of the gyros would occasionally throw massive numbers without an error flag
                        // If our hardware is working as intended, it should never read that we're spinning at 2 revs/s
                        || Eigen::Vector3d(input.gyroscope.x, input.gyroscope.y, input.gyroscope.z).norm()
                               > 4.0 * M_PI)) {
                    NUClear::log<NUClear::WARN>(
                        "Bad gyroscope value",
                        Eigen::Vector3d(input.gyroscope.x, input.gyroscope.y, input.gyroscope.z).norm());
                    sensors->gyroscope = previousSensors->gyroscope;
                }
                else {
                    // gyro_x to the right
                    // gyro_y to the back
                    // gyro_z down
                    sensors->gyroscope = Eigen::Vector3d(input.gyroscope.x, input.gyroscope.y, input.gyroscope.z);
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

                // Htx is a Map from ServoID to (a homogeneous transform from that joint to the torso)
                auto Htx = calculateAllPositions(kinematicsModel, *sensors);
                // copying the map to the sensors->Htx Map
                for (const auto& entry : Htx) {
                    sensors->Htx[entry.first] = entry.second.matrix();
                }

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
                    // This means that the faster we move, the noisier we think the measurements are
                    + ((sensors->accelerometer.norm() - std::abs(G)) * (sensors->accelerometer.norm() - std::abs(G)))
                          * config.motionFilter.noise.measurement.accelerometerMagnitude;

                // Accelerometer measurement update
                motionFilter.measure(sensors->accelerometer, acc_noise, MeasurementType::ACCELEROMETER());

                // This loop calculates the Hwf transform for feet if they have just hit the ground. If they have not
                // just hit the ground, it uses the previous Hwf value. This assumes that once the foot hits the ground,
                // it doesn't move at all
                for (auto&& side : {BodySide::LEFT, BodySide::RIGHT}) {
                    const bool foot_down      = sensors->feet[side].down;
                    const bool prev_foot_down = previous_foot_down[side];
                    const Eigen::Affine3d Htf(
                        sensors->Htx[side == BodySide::LEFT ? ServoID::L_ANKLE_ROLL : ServoID::R_ANKLE_ROLL]);

                    // If this sides foot is down, and it was not down at the previous time step, then we calculate our
                    // new footlanding_Hwf value, because our foot has just landed
                    if (foot_down && !prev_foot_down) {
                        const auto filterState = MotionModel<double>::StateVec(motionFilter.get());
                        Eigen::Affine3d Hwt;
                        Hwt.linear()      = filterState.Rwt.toRotationMatrix();
                        Hwt.translation() = filterState.rTWw;

                        // Htg is intended to be such that the "foot down" position is where the foot would be if it
                        // were flat, even if it's not flat when first touches the ground.
                        // As the foot flattens, it's meant to becomes true.
                        // This means that even if the foot hits the ground at an angle, it doesn't store that angled
                        // position as the footlanding_Hwf, but instead stores the position that foot would be if/when
                        // it becomes flat on the ground
                        const Eigen::Affine3d Htg(utility::motion::kinematics::calculateGroundSpace(Htf, Hwt));

                        footlanding_Hwf[side]                   = Hwt * Htg;
                        footlanding_Hwf[side].translation().z() = 0.0;

                        // Store the current foot down state for next time
                        previous_foot_down[side] = true;
                    }
                    // Else is down, and didn't hit the ground this time step
                    else if (foot_down && prev_foot_down) {
                        // Use stored Hwf and Htf to calculate Hwt
                        const Eigen::Affine3d Hft = Htf.inverse();
                        // Guaranteed to be initialised, because the prev_foot_down is initialised as false, so the
                        // previous if() condition will be done before this one
                        const Eigen::Affine3d footlanding_Hwt = footlanding_Hwf[side] * Hft;

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
                    // Else the foot is off the ground, so we make sure that for the next time step, we know that
                    // this time step, the foot was off the ground
                    else if (!foot_down) {
                        previous_foot_down[side] = false;
                    }

                    // Note that the Hwf is set, even if the foot is not down. This means that moving feet in the air
                    // will have an Hwf associated with them which is the transform from when that foot last hit
                    // the ground
                    sensors->feet[side].Hwf = footlanding_Hwf[side].matrix();
                }

                // Calculate our time offset from the last read then update the filter's time
                {
                    using namespace std::chrono;

                    const double deltaT =
                        std::max(duration_cast<duration<double>>(
                                     input.timestamp - (previousSensors ? previousSensors->timestamp : input.timestamp))
                                     .count(),
                                 0.0);

                    // Do a filter time update. If the cholesky decomposition(s) it did were not successful, we need to
                    // reset the filter. The reset will be handled if we reset the updates since reset event count
                    Eigen::ComputationInfo cholesky_result = motionFilter.time(deltaT);
                    switch (cholesky_result) {
                        // Cholesky decomp successful. Business as usual
                        case Eigen::Success: break;
                        // Other results are bad news, and require a motion filter reset
                        case Eigen::NumericalIssue:
                            NUClear::log<NUClear::WARN>(
                                "Cholesky decomposition failed. The provided data did not satisfy the "
                                "prerequisites.");
                            updates_since_reset_event = 0;
                            break;
                        case Eigen::NoConvergence:
                            NUClear::log<NUClear::WARN>(
                                "Cholesky decomposition failed. Iterative procedure did not converge.");
                            updates_since_reset_event = 0;
                            break;
                        case Eigen::InvalidInput:
                            NUClear::log<NUClear::WARN>(
                                "Cholesky decomposition failed. The inputs are invalid, or the algorithm has "
                                "been improperly called. When assertions are enabled, such errors trigger an "
                                "assert.");
                            updates_since_reset_event = 0;
                            break;
                        default:
                            NUClear::log<NUClear::WARN>("Cholesky decomposition failed. Some other reason.");
                            updates_since_reset_event = 0;
                            break;
                    }
                }

                // Convert the motion filter's state vector to a nicer representation, so we can access its elements
                const auto o = MotionModel<double>::StateVec(motionFilter.get());

                // We make Hwt first, because `o` is in world space
                Eigen::Affine3d Hwt;
                Hwt.linear()      = o.Rwt.toRotationMatrix();
                Hwt.translation() = o.rTWw;
                sensors->Htw      = Hwt.inverse().matrix();

                /************************************************
                 *                  Mass Model                  *
                 ************************************************/
                sensors->rMTt           = calculateCentreOfMass(kinematicsModel, sensors->Htx, sensors->Htw.inverse());
                sensors->inertia_tensor = calculateInertialTensor(kinematicsModel, sensors->Htx);

                /************************************************
                 *                  Calculate Hgt               *
                 ************************************************/
                // Extract the inverse of the rotation component of Htw
                const Eigen::Matrix3d Rwt(sensors->Htw.topLeftCorner<3, 3>().transpose());
                // We remove the yaw by making an angleaxis which has just the negative yaw,
                // then multiplying it back in, taking the yaw away
                const Eigen::AngleAxisd Rwt_negative_yaw(-Rwt.eulerAngles(0, 1, 2).z(), Eigen::Vector3d::UnitZ());

                Eigen::Affine3d Hgt;
                Hgt.linear()      = Rwt_negative_yaw * Rwt;
                Hgt.translation() = Eigen::Vector3d(0, 0, Hwt.translation().z());
                sensors->Hgt      = Hgt.matrix();

                emit(std::move(sensors));
            });
    }
}  // namespace module::input
