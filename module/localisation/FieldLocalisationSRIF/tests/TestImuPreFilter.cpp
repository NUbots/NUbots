/*
 * MIT License
 *
 * Copyright (c) 2026 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <catch2/catch_test_macros.hpp>
#include <cmath>

#include "srif/ImuPreFilter.hpp"

using module::localisation::srif::ImuPreFilter;

namespace {

    constexpr double g  = 9.80665;
    constexpr double dt = 0.002;  // 500 Hz

    /// Angle between two vectors [rad]
    double angle_between(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
        return std::atan2(a.cross(b).norm(), a.dot(b));
    }

}  // namespace

SCENARIO("The IMU pre-filter returns gravity for a still, tilted torso", "[localisation][srif][imu]") {
    ImuPreFilter imu;
    // Torso pitched and rolled; specific force is +g along world z, seen in the torso frame
    const Eigen::Matrix3d Rwb =
        (Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    const Eigen::Vector3d acc = Rwb.transpose() * Eigen::Vector3d(0, 0, g);
    for (int k = 0; k < 2000; ++k) {
        imu.update(k * dt, Eigen::Vector3d::Zero(), acc, Eigen::Vector3d::Zero());
    }
    CHECK((imu.gravity() - acc).norm() < 1e-9);
}

SCENARIO("The IMU pre-filter averages out walking accelerations while the torso turns", "[localisation][srif][imu]") {
    ImuPreFilter imu;

    // The torso yaws at a constant rate and sways (roll) at the step frequency, while the world-frame
    // linear acceleration oscillates horizontally and vertically like a gait.
    const double yaw_rate  = 0.5;   // [rad/s]
    const double step_freq = 2.0;   // [Hz]
    const double sway      = 0.05;  // roll amplitude [rad]
    const double w         = 2.0 * M_PI * step_freq;
    const Eigen::Vector3d bias(0.01, -0.02, 0.03);  // known gyro bias, passed in as the estimate

    double worst_raw      = 0.0;
    double worst_filtered = 0.0;
    for (int k = 0; k < 15000; ++k) {  // 30 s
        const double t = k * dt;
        // Rwb = Rz(yaw) * Rx(roll)
        const double yaw  = yaw_rate * t;
        const double roll = sway * std::sin(w * t);
        const Eigen::Matrix3d Rwb =
            (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
                .toRotationMatrix();
        // Body angular velocity: omega_b = Rx(roll)^T * (0, 0, yaw_rate) + (roll_rate, 0, 0)
        const double roll_rate        = sway * w * std::cos(w * t);
        const Eigen::Vector3d omega_b = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix().transpose()
                                            * Eigen::Vector3d(0, 0, yaw_rate)
                                        + Eigen::Vector3d(roll_rate, 0, 0);

        // Gait accelerations in the world frame [m/s^2]
        const Eigen::Vector3d a_w(1.5 * std::sin(w * t), 1.0 * std::cos(w * t), 2.0 * std::sin(2.0 * w * t));
        const Eigen::Vector3d acc  = Rwb.transpose() * (a_w + Eigen::Vector3d(0, 0, g));
        const Eigen::Vector3d up_b = Rwb.transpose() * Eigen::Vector3d::UnitZ();

        imu.update(t, omega_b + bias, acc, bias);

        // After the filter has settled
        if (t > 15.0) {
            worst_raw      = std::max(worst_raw, angle_between(acc, up_b));
            worst_filtered = std::max(worst_filtered, angle_between(imu.gravity(), up_b));
        }
    }

    INFO("worst raw tilt error " << worst_raw << " rad, filtered " << worst_filtered << " rad");
    CHECK(worst_raw > 0.1);
    CHECK(worst_filtered < 0.02);
}

SCENARIO("The IMU pre-filter follows a fast topple through the gyroscope", "[localisation][srif][imu]") {
    ImuPreFilter imu;
    const Eigen::Vector3d up(0, 0, g);
    int k = 0;
    // 5 s upright and still
    for (; k < 2500; ++k) {
        imu.update(k * dt, Eigen::Vector3d::Zero(), up, Eigen::Vector3d::Zero());
    }
    // Pitch forward 90 degrees in 0.5 s, then lie still
    const double pitch_rate = (M_PI / 2.0) / 0.5;
    double pitch            = 0.0;
    for (int j = 0; j < 250; ++j, ++k) {
        pitch += pitch_rate * dt;
        const Eigen::Matrix3d Rwb = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix();
        imu.update(k * dt, Eigen::Vector3d(0, pitch_rate, 0), Rwb.transpose() * up, Eigen::Vector3d::Zero());
    }
    const Eigen::Matrix3d Rwb = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
    const Eigen::Vector3d acc = Rwb.transpose() * up;
    imu.update(k * dt, Eigen::Vector3d::Zero(), acc, Eigen::Vector3d::Zero());

    // Right after the topple the filtered gravity already points the new way, without waiting tau_acc
    CHECK(angle_between(imu.gravity(), acc) < 0.02);
}

SCENARIO("The IMU pre-filter restarts after a gap in the samples", "[localisation][srif][imu]") {
    ImuPreFilter imu;
    imu.update(0.0, Eigen::Vector3d::Zero(), Eigen::Vector3d(0, 0, g), Eigen::Vector3d::Zero());
    imu.update(0.002, Eigen::Vector3d::Zero(), Eigen::Vector3d(0, 0, g), Eigen::Vector3d::Zero());
    // A second of silence, then a different reading: taken as-is rather than blended with the old one
    imu.update(1.0, Eigen::Vector3d::Zero(), Eigen::Vector3d(g, 0, 0), Eigen::Vector3d::Zero());
    CHECK((imu.gravity() - Eigen::Vector3d(g, 0, 0)).norm() < 1e-9);
}
