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

/**
 * @file ImuPreFilter.hpp
 * @brief IMU pre-processing for the SRIF, after VQF (Laidig & Seel, 2023).
 *
 * D. Laidig and T. Seel, "VQF: Highly Accurate IMU Orientation Estimation with Bias Estimation and
 * Magnetic Disturbance Rejection", Information Fusion 91 (2023) 187-204, arXiv:2203.17024.
 *
 * Run at the full Sensors rate rather than the vision rate.
 *
 * Gravity is filtered in an almost-inertial frame {i}. The gyroscope (minus the SRIF's current bias
 * estimate) is integrated into Rib, the accelerometer is rotated into {i}, low-pass filtered
 * there with a second-order Butterworth filter, and rotated back into the torso frame. Gravity is
 * constant in {i} while the walking robot's own accelerations average out, and since the
 * rotation into and out of {i} use the same Rib, only the drift of Rib within the filter time
 * constant matters. A single raw accelerometer sample is a poor gravity reading on a walking robot.
 */
#ifndef MODULE_LOCALISATION_SRIF_IMUPREFILTER_HPP
#define MODULE_LOCALISATION_SRIF_IMUPREFILTER_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

namespace module::localisation::srif {

    /// @brief Second-order Butterworth low-pass filter of a 3-vector, parametrised by a time constant
    ///        as in VQF (cut-off frequency fc = sqrt(2) / (2 pi tau)).
    class Butterworth3 {
    public:
        /**
         * @brief Filter one sample.
         * @param x The input sample
         * @param dt Time since the previous sample [s]
         * @param tau Filter time constant [s]
         * @return The filtered value
         */
        Eigen::Vector3d update(const Eigen::Vector3d& x, double dt, double tau);

        /// @brief Forget the filter state; the next sample initialises it
        void reset() {
            initialised = false;
            pending     = false;
        }

        /// @brief The latest filtered value
        [[nodiscard]] const Eigen::Vector3d& value() const {
            return y;
        }

    private:
        bool initialised   = false;
        bool pending       = false;  ///< A first sample is held in y, waiting for a time step
        Eigen::Vector3d y  = Eigen::Vector3d::Zero();
        Eigen::Vector3d s1 = Eigen::Vector3d::Zero();  ///< Direct form II transposed state
        Eigen::Vector3d s2 = Eigen::Vector3d::Zero();  ///< Direct form II transposed state
    };

    class ImuPreFilter {
    public:
        struct Parameters {
            /// @brief Time constant of the gravity low-pass filter in {i} [s] (VQF tau_acc)
            double tau_acc = 3.0;
            /// @brief Sample gaps longer than this reset the filters [s]
            double max_gap = 0.2;
        };

        Parameters params{};

        /**
         * @brief Process one IMU sample.
         * @param t Sample time [s]
         * @param gyroscope Measured angular velocity in the torso frame [rad/s]
         * @param accelerometer Measured specific force in the torso frame [m/s^2]
         * @param gyro_bias The current gyroscope bias estimate, subtracted before integration [rad/s]
         */
        void update(double t,
                    const Eigen::Vector3d& gyroscope,
                    const Eigen::Vector3d& accelerometer,
                    const Eigen::Vector3d& gyro_bias);

        /// @brief Forget all filter state
        void reset();

        /// @brief Low-pass filtered specific force in the torso frame [m/s^2]; non-finite until the first sample
        [[nodiscard]] Eigen::Vector3d gravity() const;

    private:
        bool initialised = false;
        double last_t    = 0.0;
        /// @brief Torso to almost-inertial frame rotation, from integrating the bias-corrected gyroscope
        Eigen::Quaterniond qib = Eigen::Quaterniond::Identity();
        /// @brief Specific force low-pass filtered in {i}
        Butterworth3 acc_i_lp;
    };

}  // namespace module::localisation::srif

#endif  // MODULE_LOCALISATION_SRIF_IMUPREFILTER_HPP
