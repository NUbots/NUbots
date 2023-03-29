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
 * Copyright 2023 NUbots <nubots@nubots.net>
 */

#ifndef MODULE_INPUT_ODOMETRY_HPP
#define MODULE_INPUT_ODOMETRY_HPP

#include "SensorFilter.hpp"

namespace module::input {

    void SensorFilter::update_odometry_ukf(std::unique_ptr<Sensors>& sensors,
                                           const std::shared_ptr<const Sensors>& previous_sensors,
                                           const RawSensors& raw_sensors) {

        // **************** UKF Measurement Update ****************
        // Gyroscope measurement update
        ukf.measure(sensors->gyroscope, cfg.ukf.noise.measurement.gyroscope, MeasurementType::GYROSCOPE());

        // Calculate accelerometer noise factor
        Eigen::Matrix3d acc_noise =
            cfg.ukf.noise.measurement.accelerometer
            // Add noise which is proportional to the square of how much we are moving, minus gravity
            // This means that the more we're accelerating, the noisier we think the measurements are
            + ((sensors->accelerometer.norm() - std::abs(G)) * (sensors->accelerometer.norm() - std::abs(G)))
                  * cfg.ukf.noise.measurement.accelerometer_magnitude;

        // Accelerometer measurement update
        ukf.measure(sensors->accelerometer, acc_noise, MeasurementType::ACCELEROMETER());

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
                const MotionModel<double>::StateVec filterState = MotionModel<double>::StateVec(ukf.get_state());
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
                ukf.measure(Eigen::Vector3d(footlanding_Hwt.translation()),
                            cfg.ukf.noise.measurement.flat_foot_odometry,
                            MeasurementType::FLAT_FOOT_ODOMETRY());

                // do a foot based orientation update
                Eigen::Quaterniond Rwt(footlanding_Hwt.linear());
                ukf.measure(Rwt.coeffs(),
                            cfg.ukf.noise.measurement.flat_foot_orientation,
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
        const double dt = std::max(
            std::chrono::duration_cast<std::chrono::duration<double>>(
                raw_sensors.timestamp - (previous_sensors ? previous_sensors->timestamp : raw_sensors.timestamp))
                .count(),
            0.0);

        // **************** UKF Time Update ****************
        switch (ukf.time(dt)) {
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


        // If the filter failed, use previous sensors
        if (reset_filter.load() && previous_sensors) {
            sensors->Htw = previous_sensors->Htw;
            sensors->Hgt = previous_sensors->Hgt;
        }
        else {
            // **************** Construct Odometry Output (Htw) ****************
            // Gives us the quaternion representation
            const auto o = MotionModel<double>::StateVec(ukf.get_state());
            // Map from world to torso coordinates (Rtw)
            Eigen::Isometry3d Hwt;
            Hwt.linear()      = o.Rwt.toRotationMatrix();
            Hwt.translation() = o.rTWw;
            sensors->Htw      = Hwt.inverse().matrix();

            // **************** Kinematics Horizon (Hgt) ****************
            Eigen::Isometry3d Rwt(sensors->Htw.inverse());
            // remove translation components from the transform
            Rwt.translation() = Eigen::Vector3d::Zero();
            Eigen::Isometry3d Rgt(Eigen::AngleAxisd(-Rwt.rotation().eulerAngles(0, 1, 2).z(), Eigen::Vector3d::UnitZ())
                                  * Rwt);
            sensors->Hgt = Rgt.matrix();
        }
    }

    void SensorFilter::integrate_walkcommand(const double dt) {
        // Integrate the walk command to estimate the change in position and yaw orientation
        double dx = walk_command.x() * dt * cfg.deadreckoning_scale.x();
        double dy = walk_command.y() * dt * cfg.deadreckoning_scale.y();
        yaw += walk_command.z() * dt * cfg.deadreckoning_scale.z();
        // Rotate the change in position into world coordinates before adding it to the current position
        Hwt.translation().x() += dx * cos(yaw) - dy * sin(yaw);
        Hwt.translation().y() += dy * cos(yaw) + dx * sin(yaw);
    }

    void SensorFilter::update_odometry_kf(std::unique_ptr<Sensors>& sensors,
                                          const std::shared_ptr<const Sensors>& previous_sensors,
                                          const RawSensors& raw_sensors) {
        // **************** Time Update ****************
        // Calculate our time offset from the last read then update the filter's time
        const double dt = std::max(
            std::chrono::duration_cast<std::chrono::duration<double>>(
                raw_sensors.timestamp - (previous_sensors ? previous_sensors->timestamp : raw_sensors.timestamp))
                .count(),
            0.0);

        // Integrate the walk command to estimate the change in position (x,y) and yaw orientation
        integrate_walkcommand(dt);

        // Integrate the rotational velocity to predict the change in orientation (roll, pitch)
        Eigen::Matrix<double, n_inputs, 1> u;
        kf.time(u, dt);

        // **************** Roll/Pitch Orientation Measurement Update ****************
        // Calculate the roll and pitch estimates from the accelerometer
        double est_roll  = std::atan2(sensors->accelerometer.y(), sensors->accelerometer.z());
        double est_pitch = std::atan2(-sensors->accelerometer.x(),
                                      std::sqrt(sensors->accelerometer.y() * sensors->accelerometer.y()
                                                + sensors->accelerometer.z() * sensors->accelerometer.z()));

        // Perform a gyroscope and accelerometer measurement based correction of the predicted roll and pitch
        Eigen::Matrix<double, n_measurements, 1> y;
        y << est_roll, est_pitch, sensors->gyroscope.x(), sensors->gyroscope.y();
        kf.measure(y);

        // Update the height of the torso using the kinematics from a foot which is on the ground
        if (sensors->feet[BodySide::LEFT].down) {
            Hwt.translation().z() = Eigen::Isometry3d(sensors->Htx[ServoID::L_ANKLE_ROLL]).inverse().translation().z();
        }
        else if (sensors->feet[BodySide::RIGHT].down) {
            Hwt.translation().z() = Eigen::Isometry3d(sensors->Htx[ServoID::R_ANKLE_ROLL].inverse()).translation().z();
        }

        // **************** Construct Odometry Output (Htw) ****************
        // Use the roll and pitch from the Mahony filter and the yaw from the dead reckoning of walk command
        const double roll  = kf.get_state()(0);
        const double pitch = kf.get_state()(1);
        Hwt.linear()       = EulerIntrinsicToMatrix(Eigen::Vector3d(roll, pitch, yaw));
        sensors->Htw       = Hwt.inverse().matrix();
    }

    void SensorFilter::update_odometry_mahony(std::unique_ptr<Sensors>& sensors,
                                              const std::shared_ptr<const Sensors>& previous_sensors,
                                              const RawSensors& raw_sensors) {
        // **************** Time Update ****************
        // Calculate our time offset from the last read then update the filter's time
        const double dt = std::max(
            std::chrono::duration_cast<std::chrono::duration<double>>(
                raw_sensors.timestamp - (previous_sensors ? previous_sensors->timestamp : raw_sensors.timestamp))
                .count(),
            0.0);

        // Integrate the walk command to estimate the change in position (x,y) and yaw orientation
        integrate_walkcommand(dt);

        // **************** Roll/Pitch Orientation Measurement Update ****************
        Eigen::Quaterniond quat_Rwt = Eigen::Quaterniond(Hwt.rotation());
        utility::math::filter::MahonyUpdate(sensors->accelerometer,
                                            sensors->gyroscope,
                                            dt,
                                            cfg.Ki,
                                            cfg.Kp,
                                            quat_Rwt,
                                            bias);
        // Extract the roll and pitch from the orientation quaternion
        Eigen::Vector3d rpy = MatrixToEulerIntrinsic(quat_Rwt.toRotationMatrix());

        // Compute the height of the torso using the kinematics from a foot which is on the ground
        if (sensors->feet[BodySide::LEFT].down) {
            Hwt.translation().z() = Eigen::Isometry3d(sensors->Htx[ServoID::L_ANKLE_ROLL]).inverse().translation().z();
        }
        else if (sensors->feet[BodySide::RIGHT].down) {
            Hwt.translation().z() = Eigen::Isometry3d(sensors->Htx[ServoID::R_ANKLE_ROLL].inverse()).translation().z();
        }

        // **************** Construct Odometry Output (Htw) ****************
        // Use the roll and pitch from the Mahony filter and the yaw from the dead reckoning of walk command
        const double roll  = rpy(0);
        const double pitch = rpy(1);
        Hwt.linear()       = EulerIntrinsicToMatrix(Eigen::Vector3d(roll, pitch, yaw));
        sensors->Htw       = Hwt.inverse().matrix();
    }
}  // namespace module::input
#endif  // MODULE_INPUT_ODOMETRYKF_HPP
