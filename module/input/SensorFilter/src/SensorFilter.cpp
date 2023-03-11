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

#include "utility/actuation/ForwardKinematics.hpp"
#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/math/euler.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/platform/RawSensors.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::input {

    using extension::Configuration;

    using utility::actuation::kinematics::calculateAllPositions;
    using utility::actuation::kinematics::calculateCentreOfMass;
    using utility::actuation::kinematics::calculateInertialTensor;
    using utility::input::ServoID;
    using utility::math::euler::MatrixToEulerIntrinsic;
    using utility::nusight::graph;
    using utility::platform::getRawServo;
    using utility::platform::make_error_string;
    using utility::platform::make_servo_error_string;
    using utility::support::Expression;

    SensorFilter::SensorFilter(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), theta(Eigen::Vector3d::Zero()) {

        on<Configuration>("SensorFilter.yaml").then([this](const Configuration& config) {
            log_level = config["log_level"].as<NUClear::LogLevel>();

            // Button config
            cfg.buttons.debounceThreshold = config["buttons"]["debounce_threshold"].as<int>();

            // Foot down config
            const FootDownMethod method = config["foot_down"]["method"].as<std::string>();
            std::map<FootDownMethod, float> thresholds;
            for (const auto& threshold : config["foot_down"]["known_methods"]) {
                thresholds[threshold["name"].as<std::string>()] = threshold["certainty_threshold"].as<float>();
            }
            cfg.footDown.set_method(method, thresholds);

            // Motion filter config
            // Set velocity decay
            cfg.motionFilter.velocityDecay = config["motion_filter"]["update"]["velocity_decay"].as<Expression>();
            motionFilter.model.timeUpdateVelocityDecay = cfg.motionFilter.velocityDecay;

            // Set our measurement noises
            cfg.motionFilter.noise.measurement.accelerometer =
                Eigen::Vector3d(config["motion_filter"]["noise"]["measurement"]["accelerometer"].as<Expression>())
                    .asDiagonal();
            cfg.motionFilter.noise.measurement.accelerometerMagnitude =
                Eigen::Vector3d(
                    config["motion_filter"]["noise"]["measurement"]["accelerometer_magnitude"].as<Expression>())
                    .asDiagonal();
            cfg.motionFilter.noise.measurement.gyroscope =
                Eigen::Vector3d(config["motion_filter"]["noise"]["measurement"]["gyroscope"].as<Expression>())
                    .asDiagonal();
            cfg.motionFilter.noise.measurement.flatFootOdometry =
                Eigen::Vector3d(config["motion_filter"]["noise"]["measurement"]["flat_foot_odometry"].as<Expression>())
                    .asDiagonal();
            cfg.motionFilter.noise.measurement.flatFootOrientation =
                Eigen::Vector4d(
                    config["motion_filter"]["noise"]["measurement"]["flat_foot_orientation"].as<Expression>())
                    .asDiagonal();

            // Set our process noises
            cfg.motionFilter.noise.process.position =
                config["motion_filter"]["noise"]["process"]["position"].as<Expression>();
            cfg.motionFilter.noise.process.velocity =
                config["motion_filter"]["noise"]["process"]["velocity"].as<Expression>();
            cfg.motionFilter.noise.process.rotation =
                config["motion_filter"]["noise"]["process"]["rotation"].as<Expression>();
            cfg.motionFilter.noise.process.rotationalVelocity =
                config["motion_filter"]["noise"]["process"]["rotational_velocity"].as<Expression>();

            // Set our motion model's process noise
            MotionModel<double>::StateVec process_noise;
            process_noise.rTWw               = cfg.motionFilter.noise.process.position;
            process_noise.vTw                = cfg.motionFilter.noise.process.velocity;
            process_noise.Rwt                = cfg.motionFilter.noise.process.rotation;
            process_noise.omegaTTt           = cfg.motionFilter.noise.process.rotationalVelocity;
            motionFilter.model.process_noise = process_noise;

            // Set our initial means
            cfg.motionFilter.initial.mean.position =
                config["motion_filter"]["initial"]["mean"]["position"].as<Expression>();
            cfg.motionFilter.initial.mean.velocity =
                config["motion_filter"]["initial"]["mean"]["velocity"].as<Expression>();
            cfg.motionFilter.initial.mean.rotation =
                config["motion_filter"]["initial"]["mean"]["rotation"].as<Expression>();
            cfg.motionFilter.initial.mean.rotationalVelocity =
                config["motion_filter"]["initial"]["mean"]["rotational_velocity"].as<Expression>();

            // Set out initial covariance
            cfg.motionFilter.initial.covariance.position =
                config["motion_filter"]["initial"]["covariance"]["position"].as<Expression>();
            cfg.motionFilter.initial.covariance.velocity =
                config["motion_filter"]["initial"]["covariance"]["velocity"].as<Expression>();
            cfg.motionFilter.initial.covariance.rotation =
                config["motion_filter"]["initial"]["covariance"]["rotation"].as<Expression>();
            cfg.motionFilter.initial.covariance.rotationalVelocity =
                config["motion_filter"]["initial"]["covariance"]["rotational_velocity"].as<Expression>();

            // Set our initial state with the config means and covariances, flagging the filter to reset it
            MotionModel<double>::StateVec mean;
            mean.rTWw     = cfg.motionFilter.initial.mean.position;
            mean.vTw      = cfg.motionFilter.initial.mean.velocity;
            mean.Rwt      = cfg.motionFilter.initial.mean.rotation;
            mean.omegaTTt = cfg.motionFilter.initial.mean.rotationalVelocity;

            MotionModel<double>::StateVec covariance;
            covariance.rTWw     = cfg.motionFilter.initial.covariance.position;
            covariance.vTw      = cfg.motionFilter.initial.covariance.velocity;
            covariance.Rwt      = cfg.motionFilter.initial.covariance.rotation;
            covariance.omegaTTt = cfg.motionFilter.initial.covariance.rotationalVelocity;
            motionFilter.set_state(mean.getStateVec(), covariance.asDiagonal());

            // Set our initial position
            rTWw = config["motion_filter"]["initial"]["mean"]["position"].as<Expression>();

            // Don't filter any sensors until we have initialised the filter
            update_loop.disable();
            reset_filter.store(true);
        });

        on<Last<20, Trigger<RawSensors>>, With<KinematicsModel>, Single>().then(
            [this](const std::list<std::shared_ptr<const RawSensors>>& sensors, const KinematicsModel& model) {
                // If we need to reset the filter, do that here
                if (reset_filter.load()) {
                    Eigen::Vector3d acc  = Eigen::Vector3d::Zero();
                    Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
                    Eigen::Vector3d rMFt = Eigen::Vector3d::Zero();

                    for (const auto& s : sensors) {
                        Sensors filtered_sensors{};

                        // Accumulate accelerometer and gyroscope readings
                        acc += s->accelerometer.cast<double>();
                        gyro += s->gyroscope.cast<double>();
                        // Make sure we have servo positions
                        for (uint32_t id = 0; id < 20; ++id) {
                            const auto& original = utility::platform::getRawServo(id, *s);
                            // Add the sensor values to the system properly
                            filtered_sensors.servo.emplace_back(0,
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
                                                                static_cast<float>(original.temperature));
                        }

                        // Calculate forward kinematics
                        const auto Htx = calculateAllPositions(model, filtered_sensors);
                        for (const auto& entry : Htx) {
                            filtered_sensors.Htx[entry.first] = entry.second.matrix();
                        }

                        // Calculate the average length of both legs from the torso and accumulate this measurement
                        const Eigen::Isometry3d Htr(filtered_sensors.Htx[ServoID::R_ANKLE_ROLL]);
                        const Eigen::Isometry3d Htl(filtered_sensors.Htx[ServoID::L_ANKLE_ROLL]);
                        const Eigen::Vector3d rTFt = (Htr.translation() + Htl.translation()) * 0.5;

                        // Accumulator CoM readings
                        rMFt += calculateCentreOfMass(model, filtered_sensors.Htx).head<3>() + rTFt;
                    }

                    // Average all accumulated readings
                    acc /= static_cast<double>(sensors.size());
                    gyro /= static_cast<double>(sensors.size());
                    rMFt /= static_cast<double>(sensors.size());

                    // Average time per sensor reading
                    double deltaT = std::chrono::duration_cast<std::chrono::duration<double>>(
                                        sensors.back()->timestamp - sensors.front()->timestamp)
                                        .count()
                                    / static_cast<double>(sensors.size());

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
                    covariance.rTWw     = cfg.motionFilter.initial.covariance.position;
                    covariance.vTw      = cfg.motionFilter.initial.covariance.velocity;
                    covariance.Rwt      = cfg.motionFilter.initial.covariance.rotation;
                    covariance.omegaTTt = cfg.motionFilter.initial.covariance.rotationalVelocity;

                    // We have finished resetting the filter now
                    switch (motionFilter.reset(mean.getStateVec(), covariance.asDiagonal())) {
                        case Eigen::Success:
                            log<NUClear::INFO>("Motion Model UKF has been reset");
                            reset_filter.store(false);
                            update_loop.enable();
                            break;
                        case Eigen::NumericalIssue:
                            log<NUClear::WARN>(
                                "Cholesky decomposition failed. The provided data did not satisfy the "
                                "prerequisites.");
                            break;
                        case Eigen::NoConvergence:
                            log<NUClear::WARN>("Cholesky decomposition failed. Iterative procedure did not converge.");
                            break;
                        case Eigen::InvalidInput:
                            log<NUClear::WARN>(
                                "Cholesky decomposition failed. The inputs are invalid, or the algorithm has been "
                                "improperly called. When assertions are enabled, such errors trigger an assert.");
                            break;
                        default: log<NUClear::WARN>("Cholesky decomposition failed. Some other reason."); break;
                    }
                }

                int leftCount   = 0;
                int middleCount = 0;

                // If we have any downs in the last 20 frames then we are button pushed
                for (const auto& s : sensors) {
                    if (s->buttons.left && (s->platform_error_flags == 0u)) {
                        ++leftCount;
                    }
                    if (s->buttons.middle && (s->platform_error_flags == 0u)) {
                        ++middleCount;
                    }
                }

                bool newLeftDown   = leftCount > cfg.buttons.debounceThreshold;
                bool newMiddleDown = middleCount > cfg.buttons.debounceThreshold;

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

        update_loop = on<Trigger<RawSensors>, Optional<With<Sensors>>, With<KinematicsModel>, Single, Priority::HIGH>()
                          .then("Main Sensors Loop",
                                [this](const RawSensors& raw_sensors,
                                       const std::shared_ptr<const Sensors>& previous_sensors,
                                       const KinematicsModel& kinematics_model) {
                                    auto sensors = std::make_unique<Sensors>();

                                    update_raw_sensors(sensors, previous_sensors, raw_sensors);

                                    update_kinematics(sensors, kinematics_model, raw_sensors);

                                    update_odometry_ukf(sensors, previous_sensors, raw_sensors);

                                    // Debug sensor filter
                                    if (log_level <= NUClear::DEBUG) {
                                        debug_sensor_filter(sensors, raw_sensors);
                                    }

                                    emit(std::move(sensors));
                                })
                          .disable();
    }

    void SensorFilter::update_raw_sensors(std::unique_ptr<Sensors>& sensors,
                                          const std::shared_ptr<const Sensors>& previous_sensors,
                                          const RawSensors& raw_sensors) {
        // Set our timestamp to when the data was read
        sensors->timestamp = raw_sensors.timestamp;

        // Update the current battery voltage of the whole robot
        sensors->voltage = raw_sensors.voltage;

        // Check for errors on the platform and FSRs
        if (raw_sensors.platform_error_flags != RawSensors::Error::OK) {
            NUClear::log<NUClear::WARN>(make_error_string("Platform", raw_sensors.platform_error_flags));
        }
        if (raw_sensors.fsr.left.error_flags != RawSensors::Error::OK) {
            NUClear::log<NUClear::WARN>(make_error_string("Left FSR", raw_sensors.fsr.left.error_flags));
        }
        if (raw_sensors.fsr.right.error_flags != RawSensors::Error::OK) {
            NUClear::log<NUClear::WARN>(make_error_string("Right FSR", raw_sensors.fsr.right.error_flags));
        }

        // Read through all of our sensors
        for (uint32_t id = 0; id < 20; ++id) {
            const auto& original = getRawServo(id, raw_sensors);
            const auto& error    = original.error_flags;

            // Check for an error on the servo and report it
            if (error != RawSensors::Error::OK) {
                NUClear::log<NUClear::WARN>(utility::platform::make_servo_error_string(original, id));
            }
            // If current Sensors message for this servo has an error and we have a previous sensors
            // message available, then we use our previous sensor values with some updates
            if (error != RawSensors::Error::OK && previous_sensors) {
                // Add the sensor values to the system properly
                sensors->servo.emplace_back(error,
                                            id,
                                            original.torque_enabled,
                                            original.p_gain,
                                            original.i_gain,
                                            original.d_gain,
                                            original.goal_position,
                                            original.moving_speed,
                                            previous_sensors->servo[id].present_position,
                                            previous_sensors->servo[id].present_velocity,
                                            previous_sensors->servo[id].load,
                                            previous_sensors->servo[id].voltage,
                                            previous_sensors->servo[id].temperature);
            }
            // Otherwise we can just use the new values as is
            else {
                // Add the sensor values to the system properly
                sensors->servo.emplace_back(error,
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
                                            static_cast<float>(original.temperature));
            }

            // If we have a previous sensors and our platform has errors then reuse our last sensor value
            if ((raw_sensors.platform_error_flags != 0u) && previous_sensors) {
                sensors->accelerometer = previous_sensors->accelerometer;
            }
            else {
                sensors->accelerometer = raw_sensors.accelerometer.cast<double>();
            }

            // If we have a previous Sensors message and (our platform has errors or we are spinning too
            // quickly), then reuse our last sensor value
            if (previous_sensors
                && ((raw_sensors.platform_error_flags != 0u)
                    // One of the gyros would occasionally throw massive numbers without an error flag
                    // If our hardware is working as intended, it should never read that we're spinning at 2
                    // revs/s
                    || raw_sensors.gyroscope.norm() > 4.0 * M_PI)) {
                NUClear::log<NUClear::WARN>("Bad gyroscope value", raw_sensors.gyroscope.norm());
                sensors->gyroscope = previous_sensors->gyroscope;
            }
            else {
                sensors->gyroscope = raw_sensors.gyroscope.cast<double>();
            }
        }

        // Buttons and LEDs
        sensors->button.reserve(2);
        sensors->button.emplace_back(0, raw_sensors.buttons.left);
        sensors->button.emplace_back(1, raw_sensors.buttons.middle);
        sensors->led.reserve(5);
        sensors->led.emplace_back(0, raw_sensors.led_panel.led2 ? 0xFF0000 : 0);
        sensors->led.emplace_back(1, raw_sensors.led_panel.led3 ? 0xFF0000 : 0);
        sensors->led.emplace_back(2, raw_sensors.led_panel.led4 ? 0xFF0000 : 0);
        sensors->led.emplace_back(3, raw_sensors.head_led.RGB);  // Head
        sensors->led.emplace_back(4, raw_sensors.eye_led.RGB);   // Eye
    }

    void SensorFilter::update_kinematics(std::unique_ptr<Sensors>& sensors,
                                         const KinematicsModel& kinematics_model,
                                         const RawSensors& raw_sensors) {
        // Htx is a map from ServoID to homogeneous transforms from each ServoID to the torso
        auto Htx = calculateAllPositions(kinematics_model, *sensors);
        for (const auto& entry : Htx) {
            sensors->Htx[entry.first] = entry.second.matrix();
        }

        // Update centre of mass and inertia tensor
        sensors->rMTt           = calculateCentreOfMass(kinematics_model, sensors->Htx);
        sensors->inertia_tensor = calculateInertialTensor(kinematics_model, sensors->Htx);

        // Determine if feet are on the ground using the selected method
        sensors->feet.resize(2);
        sensors->feet[BodySide::RIGHT].down = true;
        sensors->feet[BodySide::LEFT].down  = true;

        // Calculate Z distance between feet in torso space {t} and determine which foot is lower
        const Eigen::Isometry3d Htr(sensors->Htx[ServoID::R_ANKLE_ROLL]);
        const Eigen::Isometry3d Htl(sensors->Htx[ServoID::L_ANKLE_ROLL]);
        const Eigen::Isometry3d Hlr = Htl.inverse() * Htr;
        const Eigen::Vector3d rRLl  = Hlr.translation();
        switch (cfg.footDown.method()) {
            case FootDownMethod::Z_HEIGHT:
                if (rRLl.z() < -cfg.footDown.threshold()) {
                    sensors->feet[BodySide::LEFT].down = false;
                }
                else if (rRLl.z() > cfg.footDown.threshold()) {
                    sensors->feet[BodySide::RIGHT].down = false;
                }
                break;
            case FootDownMethod::FSR:
                // Determine if any two diagonally opposite FSRs are in contact with the ground, which is either fsr1
                // and fsr3, or fsr2 and fsr4. A FSR is in contact with the ground if its value is greater than the
                // certainty threshold
                sensors->feet[BodySide::LEFT].down = (raw_sensors.fsr.left.fsr1 > cfg.footDown.threshold()
                                                      && raw_sensors.fsr.left.fsr3 > cfg.footDown.threshold())
                                                     || (raw_sensors.fsr.left.fsr2 > cfg.footDown.threshold()
                                                         && raw_sensors.fsr.left.fsr4 > cfg.footDown.threshold());
                sensors->feet[BodySide::RIGHT].down = (raw_sensors.fsr.right.fsr1 > cfg.footDown.threshold()
                                                       && raw_sensors.fsr.right.fsr3 > cfg.footDown.threshold())
                                                      || (raw_sensors.fsr.right.fsr2 > cfg.footDown.threshold()
                                                          && raw_sensors.fsr.right.fsr4 > cfg.footDown.threshold());
                break;
            default: log<NUClear::WARN>("Unknown foot down method"); break;
        }
    }

    void SensorFilter::update_odometry_ukf(std::unique_ptr<Sensors>& sensors,
                                           const std::shared_ptr<const Sensors>& previous_sensors,
                                           const RawSensors& raw_sensors) {
        // Gyroscope measurement update
        motionFilter.measure(sensors->gyroscope,
                             cfg.motionFilter.noise.measurement.gyroscope,
                             MeasurementType::GYROSCOPE());

        // Calculate accelerometer noise factor
        Eigen::Matrix3d acc_noise =
            cfg.motionFilter.noise.measurement.accelerometer
            // Add noise which is proportional to the square of how much we are moving, minus gravity
            // This means that the more we're accelerating, the noisier we think the measurements are
            + ((sensors->accelerometer.norm() - std::abs(G)) * (sensors->accelerometer.norm() - std::abs(G)))
                  * cfg.motionFilter.noise.measurement.accelerometerMagnitude;

        // Accelerometer measurement update
        motionFilter.measure(sensors->accelerometer, acc_noise, MeasurementType::ACCELEROMETER());

        // This loop calculates the Hwf transform for feet if they have just hit the ground. If they
        // have not just hit the ground, it uses the previous Hwf value. This assumes that once the foot
        // hits the ground, it doesn't move at all i.e. we're ASSUMING the foot cannot slip/slide
        for (const auto& side : {BodySide::LEFT, BodySide::RIGHT}) {
            bool foot_down      = sensors->feet[side].down;
            bool prev_foot_down = previous_foot_down[side];
            Eigen::Isometry3d Htf(sensors->Htx[side == BodySide::LEFT ? ServoID::L_ANKLE_ROLL : ServoID::R_ANKLE_ROLL]);

            // If this side's foot is down, and it was not down at the previous time step, then we
            // calculate our new footlanding_Hwf value, because our foot has just landed
            if (foot_down && !prev_foot_down) {
                const MotionModel<double>::StateVec filterState = MotionModel<double>::StateVec(motionFilter.get());
                Eigen::Isometry3d Hwt;
                Hwt.linear()      = filterState.Rwt.toRotationMatrix();
                Hwt.translation() = filterState.rTWw;

                // Htg is intended to be such that the "foot down" position is where the foot would be
                // if it were flat, even if it's not flat when first touches the ground. As the foot
                // flattens, it's meant to becomes true. This means that even if the foot hits the
                // ground at an angle, it doesn't store that angled position as the footlanding_Hwf, but
                // instead stores the position that foot would be if/when it becomes flat on the ground
                Eigen::Isometry3d Htg(utility::actuation::kinematics::calculateGroundSpace(Htf, Hwt));

                footlanding_Hwf[side]                   = Hwt * Htg;
                footlanding_Hwf[side].translation().z() = 0.0;

                // Store the current foot down state for next time
                previous_foot_down[side] = true;
            }
            // This sides foot is down, but it didn't hit the ground this time step
            else if (foot_down && prev_foot_down) {
                // Use stored Hwf and Htf to calculate Hwt
                Eigen::Isometry3d footlanding_Hwt = footlanding_Hwf[side] * Htf.inverse();

                // do a foot based position update
                motionFilter.measure(Eigen::Vector3d(footlanding_Hwt.translation()),
                                     cfg.motionFilter.noise.measurement.flatFootOdometry,
                                     MeasurementType::FLAT_FOOT_ODOMETRY());

                // do a foot based orientation update
                Eigen::Quaterniond Rwt(footlanding_Hwt.linear());
                motionFilter.measure(Rwt.coeffs(),
                                     cfg.motionFilter.noise.measurement.flatFootOrientation,
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
        /* using namespace std::chrono */ {
            using namespace std::chrono;
            const double deltaT = std::max(
                duration_cast<duration<double>>(
                    raw_sensors.timestamp - (previous_sensors ? previous_sensors->timestamp : raw_sensors.timestamp))
                    .count(),
                0.0);

            // Time update
            switch (motionFilter.time(deltaT)) {
                // If we succeeded doing the time update, we don't have to reset the filter
                case Eigen::Success: break;
                // Otherwise, we log the error and set the flag to reset the filter
                case Eigen::NumericalIssue:
                    log<NUClear::WARN>(
                        "Cholesky decomposition failed. The provided data did not satisfy the "
                        "prerequisites.");
                    // Disable the sensor update loop to reset the filter post cholesky
                    update_loop.disable();
                    reset_filter.store(true);
                    break;
                case Eigen::NoConvergence:
                    log<NUClear::WARN>("Cholesky decomposition failed. Iterative procedure did not converge.");
                    // Disable the sensor update loop to reset the filter post cholesky
                    update_loop.disable();
                    reset_filter.store(true);
                    break;
                case Eigen::InvalidInput:
                    log<NUClear::WARN>(
                        "Cholesky decomposition failed. The inputs are invalid, or the algorithm has "
                        "been "
                        "improperly called. When assertions are enabled, such errors trigger an "
                        "assert.");
                    // Disable the sensor update loop to reset the filter post cholesky
                    update_loop.disable();
                    reset_filter.store(true);
                    break;
                default:
                    log<NUClear::WARN>("Cholesky decomposition failed. Some other reason.");
                    // Disable the sensor update loop to reset the filter post cholesky
                    update_loop.disable();
                    reset_filter.store(true);
                    break;
            }
        }

        // If the filter is not reliable, just use previous sensors
        if (reset_filter.load() && previous_sensors) {
            sensors->Htw = previous_sensors->Htw;
            sensors->Hgt = previous_sensors->Hgt;
            return;
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
                const Eigen::Isometry3d Htf(
                    sensors->Htx[side == BodySide::LEFT ? ServoID::L_ANKLE_ROLL : ServoID::R_ANKLE_ROLL]);

                // Calculate our current Foot to CoM vector for this foot
                const Eigen::Vector3d current_rMFt = Htf.translation() + sensors->rMTt.head<3>();

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
                        const Eigen::Quaterniond& Rwt     = MotionModel<double>::StateVec(motionFilter.get()).Rwt;
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
            Eigen::Isometry3d Hwt;
            Hwt.linear()      = o.Rwt.toRotationMatrix();
            Hwt.translation() = o.rTWw;
            // Remove the yaw component of the rotation
            Hwt.linear() =
                Eigen::AngleAxisd(-std::atan2(Hwt(1, 0), Hwt(0, 0)), Eigen::Vector3d::UnitZ()).toRotationMatrix()
                * Hwt.linear();
            sensors->Htw = Hwt.inverse().matrix();


            /************************************************
             *                  Kinematics Horizon          *
             ************************************************/
            Eigen::Isometry3d Rwt(sensors->Htw.inverse());
            // remove translation components from the transform
            Rwt.translation() = Eigen::Vector3d::Zero();
            Eigen::Isometry3d Rgt(Eigen::AngleAxisd(-Rwt.rotation().eulerAngles(0, 1, 2).z(), Eigen::Vector3d::UnitZ())
                                  * Rwt);
            sensors->Hgt = Rgt.matrix();
        }
    }

    void SensorFilter::debug_sensor_filter(std::unique_ptr<Sensors>& sensors, const RawSensors& raw_sensors) {
        // If we have ground truth odometry, then we can debug the error between our estimate and the ground truth
        if (raw_sensors.odometry_ground_truth.exists) {
            Eigen::Isometry3d true_Htw(raw_sensors.odometry_ground_truth.Htw);
            Eigen::Isometry3d Hwt = Eigen::Isometry3d(sensors->Htw).inverse();
            // Determine translational distance error
            Eigen::Vector3d est_rWTt   = Hwt.inverse().translation();
            Eigen::Vector3d true_rWTt  = true_Htw.translation();
            Eigen::Vector3d error_rWTt = (true_rWTt - est_rWTt).cwiseAbs();

            // Determine yaw, pitch and roll error
            Eigen::Vector3d true_Rtw  = MatrixToEulerIntrinsic(true_Htw.rotation());
            Eigen::Vector3d est_Rtw   = MatrixToEulerIntrinsic(Hwt.inverse().rotation());
            Eigen::Vector3d error_Rtw = (true_Rtw - est_Rtw).cwiseAbs();
            double quat_rot_error     = Eigen::Quaterniond(true_Htw.linear() * Hwt.linear()).w();

            // Graph translation and its error
            emit(graph("Htw est translation (rWTt)", est_rWTt.x(), est_rWTt.y(), est_rWTt.z()));
            emit(graph("Htw true translation (rWTt)", true_rWTt.x(), true_rWTt.y(), true_rWTt.z()));
            emit(graph("Htw translation error", error_rWTt.x(), error_rWTt.y(), error_rWTt.z()));

            // Graph angles and error
            emit(graph("Rtw est angles (rpy)", est_Rtw.x(), est_Rtw.y(), est_Rtw.z()));
            emit(graph("Rtw true angles (rpy)", true_Rtw.x(), true_Rtw.y(), true_Rtw.z()));
            emit(graph("Rtw error (rpy)", error_Rtw.x(), error_Rtw.y(), error_Rtw.z()));
            emit(graph("Quaternion rotational error", quat_rot_error));
        }

        // Graph the raw accelerometer and gyroscope data
        emit(graph("Gyroscope", sensors->gyroscope.x(), sensors->gyroscope.y(), sensors->gyroscope.z()));
        emit(
            graph("Accelerometer", sensors->accelerometer.x(), sensors->accelerometer.y(), sensors->accelerometer.z()));

        // Graph the foot down sensors state for each foot
        emit(graph(fmt::format("Sensor/Foot Down/{}/Left", std::string(cfg.footDown.method())),
                   sensors->feet[BodySide::LEFT].down));
        emit(graph(fmt::format("Sensor/Foot Down/{}/Right", std::string(cfg.footDown.method())),
                   sensors->feet[BodySide::RIGHT].down));

        // Graph the position and orientation of the feet
        const Eigen::Isometry3d Htl(sensors->Htx[ServoID::L_ANKLE_ROLL]);
        const Eigen::Isometry3d Htr(sensors->Htx[ServoID::R_ANKLE_ROLL]);
        Eigen::Matrix<double, 3, 3> Rtl     = Htl.linear();
        Eigen::Matrix<double, 3, 1> Rtl_rpy = MatrixToEulerIntrinsic(Rtl);
        emit(graph("Left Foot Actual Position", Htl(0, 3), Htl(1, 3), Htl(2, 3)));
        emit(graph("Left Foot Actual Orientation (r,p,y)", Rtl_rpy.x(), Rtl_rpy.y(), Rtl_rpy.z()));
        Eigen::Matrix<double, 3, 3> Rtr     = Htr.linear();
        Eigen::Matrix<double, 3, 1> Rtr_rpy = MatrixToEulerIntrinsic(Rtr);
        emit(graph("Right Foot Actual Position", Htr(0, 3), Htr(1, 3), Htr(2, 3)));
        emit(graph("Right Foot Actual Orientation (r,p,y)", Rtr_rpy.x(), Rtr_rpy.y(), Rtr_rpy.z()));
    }
}  // namespace module::input
