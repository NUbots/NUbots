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

#ifndef MODULE_INPUT_ODOMETRYUKF_HPP
#define MODULE_INPUT_ODOMETRYUKF_HPP

#include "SensorFilter.hpp"

namespace module::input {

    void SensorFilter::update_odometry_ukf(std::unique_ptr<Sensors>& sensors,
                                           const std::shared_ptr<const Sensors>& previous_sensors,
                                           const RawSensors& raw_sensors) {

        // **************** UKF Measurement Update ****************
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
                  * cfg.motionFilter.noise.measurement.accelerometer_magnitude;

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
                                     cfg.motionFilter.noise.measurement.flat_foot_odometry,
                                     MeasurementType::FLAT_FOOT_ODOMETRY());

                // do a foot based orientation update
                Eigen::Quaterniond Rwt(footlanding_Hwt.linear());
                motionFilter.measure(Rwt.coeffs(),
                                     cfg.motionFilter.noise.measurement.flat_foot_orientation,
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
        switch (motionFilter.time(dt)) {
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
            return;
        }
        else {
            // **************** Construct Odometry Output (Htw) ****************
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

            // **************** Kinematics Horizon (Hgt) ****************
            Eigen::Isometry3d Rwt(sensors->Htw.inverse());
            // remove translation components from the transform
            Rwt.translation() = Eigen::Vector3d::Zero();
            Eigen::Isometry3d Rgt(Eigen::AngleAxisd(-Rwt.rotation().eulerAngles(0, 1, 2).z(), Eigen::Vector3d::UnitZ())
                                  * Rwt);
            sensors->Hgt = Rgt.matrix();
        }
    }

}  // namespace module::input
#endif  // MODULE_INPUT_ODOMETRYUKF_HPP
