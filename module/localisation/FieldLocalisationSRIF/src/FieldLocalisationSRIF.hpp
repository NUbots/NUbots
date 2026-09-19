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
#ifndef MODULE_LOCALISATION_FIELDLOCALISATIONSRIF_HPP
#define MODULE_LOCALISATION_FIELDLOCALISATIONSRIF_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <limits>
#include <memory>
#include <nuclear>
#include <vector>

#include "measurement/MeasurementFieldLandmarks.hpp"
#include "srif/FieldMap.hpp"
#include "srif/FieldSamples.hpp"
#include "srif/SideDisambiguator.hpp"
#include "srif/SystemLocalisation.hpp"

#include "message/input/Image.hpp"

#include "utility/gaussian_filtering/Pose.hpp"

namespace module::localisation {

    /// @brief Short local aliases for the estimator types this module drives.
    namespace filter {
        using measurement::MeasurementFieldLandmarks;
        using srif::Detection;
        using srif::FieldDimensions;
        using srif::FieldMap;
        using srif::SensorsSample;
        using srif::SideDisambiguator;
        using srif::SystemLocalisation;
        using srif::VisionSample;
        using utility::gaussian_filtering::Pose;
    }  // namespace filter

    /**
     * @brief Square-root information filter field localisation.
     *
     * Estimates the torso pose in the field frame {f}, its body-fixed velocity, the gyroscope bias
     * and a 2-DOF camera-mount attitude bias, as a Gaussian in square-root information form.
     * Landmarks, gyroscope, odometry velocity, gravity and kinematic height all enter as
     * measurements; updates are MAP optimisations (trust-region Newton). Initialisation is a coarse
     * grid search over (x, y, yaw) on the first usable vision frame.
     *
     * See srif::SystemLocalisation for the state layout, and the module README for the design.
     */
    class FieldLocalisationSRIF : public NUClear::Reactor {
    public:
        /// @brief Called by the powerplant to build and set up the FieldLocalisationSRIF reactor.
        explicit FieldLocalisationSRIF(std::unique_ptr<NUClear::Environment> environment);

    private:
        /// @brief Parameters tuned once and left alone. What changes per robot, venue or game
        ///        is in the yaml.
        struct Config {
            /// @brief Process noise PSDs [m/sqrt(s), rad/sqrt(s)]
            filter::SystemLocalisation::Parameters process{};
            /// @brief Multi-hypothesis (field-symmetry) mixture parameters
            filter::SystemLocalisation::HypothesisParameters hypothesis{};
            /// @brief Landmark measurement noise/association options
            filter::MeasurementFieldLandmarks::Options measurement{};
            /// @brief Initial sqrt-covariance diagonal for the 18-dim state after the grid solve.
            ///
            /// Bigger means less certain. Never set an entry near zero: these are stored inverted
            /// (square-root information form), so a near-zero std dev becomes near-infinite
            /// information and the estimate eventually goes non-finite.
            // clang-format off
            Eigen::Matrix<double, 18, 1> initial_sqrt_covariance =
                (Eigen::Matrix<double, 18, 1>() <<
                 1.00, 1.00, 0.05,              // x, y, z [m]
                 0.25, 0.25, 0.25, 0.25,        // quaternion (w, x, y, z)
                 0.30, 0.30, 0.10,              // v [m/s]
                 0.50, 0.50, 0.50,              // omega [rad/s]
                 0.05, 0.05, 0.05,              // gyroscope bias [rad/s]
                 0.02, 0.02                     // camera mount bias (roll, pitch) [rad]
                 ).finished();
            // clang-format on
            /// @brief Grid search steps for the initial pose solve
            double grid_step_xy  = 0.35;
            double grid_step_yaw = 18.0 * M_PI / 180.0;
            /// @brief Minimum landmark associations to trust an initial grid solve
            int min_init_associations = 4;
            /// @brief Enable the multi-hypothesis (field-symmetry) Gaussian mixture bank
            bool use_hypothesis_bank = false;
            /// @brief Enable out-of-field side disambiguation (see SideDisambiguator)
            bool use_side_disambiguator = true;
            /// @brief Enable the accelerometer gravity measurement of torso roll/pitch
            bool use_gravity = true;
            /// @brief Accelerometer gravity-direction noise std dev [m/s^2]
            double gravity_sigma = 1.0;
            /// @brief Maximum deviation of the specific-force magnitude from standard gravity that
            ///        is still treated as a gravity reading [m/s^2]
            double gravity_quasi_static_tolerance = 3.0;
            /// @brief Enable the kinematic torso-height measurement
            bool use_kinematic_height = true;
            /// @brief Kinematic torso-height noise std dev [m]
            double height_sigma = 0.02;
            /// @brief Std dev on the |q| = 1 pseudo-measurement (dimensionless). Without it the
            ///        MAP Hessian is singular along |q|.
            double quaternion_norm_sigma = 1e-3;

            // --- body-rate measurements ---
            /// @brief Gyroscope noise std dev per axis [rad/s]
            double gyroscope_sigma = 0.02;
            /// @brief Which signal measures the body linear velocity vBb.
            enum class OdometryVelocitySource {
                /// Finite difference of consecutive Sensors.Htw. Available on every platform.
                HTW_DIFFERENCE,
                /// Sensors.vTw rotated into the torso frame. For platforms whose odometry is a
                /// real state estimator.
                SENSORS_VTW,
            };
            /// @brief Where vBb is measured from
            OdometryVelocitySource odometry_velocity_source = OdometryVelocitySource::HTW_DIFFERENCE;
            /// @brief Body linear velocity measurement noise std dev per axis [m/s]
            double odometry_velocity_sigma = 0.15;
            /// @brief Zero-velocity update noise while FALLEN (lying still) [m/s]
            double zupt_sigma = 0.02;
            /// @brief Zero-velocity update noise while FALLING or getting up [m/s]
            double zupt_dynamic_sigma = 0.30;

            // --- fall handling ---
            // How long the elevated PSDs apply is process.disturbed_window.
            /// @brief Horizontal position std restored on recovering from a fall [m]
            double recovery_pos_std = 0.5;
            /// @brief Yaw std restored on recovering from a fall [rad]
            double recovery_yaw_std = 0.6;
            /// @brief Maximum odometry sample spacing to finite-difference across [s]
            double max_odometry_gap = 0.1;
            /// @brief Length of the rolling odometry window vision frames are paired against [s]
            double sensors_window_seconds = 2.0;
            /// @brief Maximum age of the odometry sample paired with a vision frame [s]
            double max_sensor_pairing_age = 0.1;
        } cfg;

        /// @brief Time origin (first Sensors capture) that all event times are measured from
        NUClear::clock::time_point t0{};
        /// @brief Whether t0 has been captured yet
        bool have_t0 = false;

        /// @brief Rolling window of recent odometry samples, newest last, each carrying the
        ///        velocity it was measured with (bounded by sensors_window_seconds)
        std::vector<filter::SensorsSample> sensors_window;
        /// @brief The estimator (null until the first successful initial-pose solve)
        std::unique_ptr<filter::SystemLocalisation> system;
        /// @brief Field landmark map (null until the first FieldDescription arrives)
        std::unique_ptr<filter::FieldMap> map;
        /// @brief Out-of-field side disambiguator (null until the first image, which carries the lens)
        std::unique_ptr<filter::SideDisambiguator> side;
        /// @brief Time of the last mirror re-seed, so a latched flip request cannot respawn every frame
        double last_respawn_t = -std::numeric_limits<double>::infinity();
        /// @brief Whether the estimator has been initialised from a landmark frame
        bool initialised = false;

        /// @brief Whether the robot was upright on the previous vision frame
        bool was_upright = true;
        /// @brief Time the current non-upright episode began [s since t0]
        double fall_start_t = 0.0;

        /// @brief Seconds since the module time origin t0
        [[nodiscard]] double seconds(const NUClear::clock::time_point& tp) const {
            return std::chrono::duration_cast<std::chrono::duration<double>>(tp - t0).count();
        }

        /**
         * @brief The odometry sample in the rolling window nearest a given time.
         * @param t Vision capture time [s since t0]
         * @return The nearest sample, or nullptr if the window is empty or the nearest is too old.
         */
        [[nodiscard]] const filter::SensorsSample* nearest_sensors(double t) const;

        /**
         * @brief Hand confidence back to the belief on standing up again.
         * @param t Time the robot became upright again [s since t0]
         */
        void apply_fall_recovery(double t);

        /**
         * @brief Emit the localisation Field message (and debug graphs) from the current belief.
         * @param Htw World-to-torso pose paired with the processed vision frame (from Sensors)
         * @param measurement The processed landmark measurement (nullptr on the bootstrap frame)
         */
        void emit_field(const filter::Pose<double>& Htw, const filter::MeasurementFieldLandmarks* measurement);

        /**
         * @brief Run out-of-field side disambiguation on one camera frame and act on the verdict.
         * @param image The camera frame (carries its own lens, dimensions and capture-time Hcw)
         */
        void run_side_disambiguation(const message::input::Image& image);

        /**
         * @brief Publish one frame of out-of-field working state for the NUsight vision pane.
         * @param image The camera frame the corners were detected in
         * @param result The disambiguator's per-frame result
         */
        void emit_out_of_field(const message::input::Image& image,
                               const filter::SideDisambiguator::FrameResult& result);

        /**
         * @brief Coarse global grid search over (x, y, yaw) for the initial pose. Roll, pitch and
         *        height come from the kinematic chain; the own-half convention breaks the mirror.
         * @param sample Landmark rays extracted from the vision messages
         * @param Tbc Camera pose w.r.t. torso at capture time
         * @param Twt Torso pose in the odometry world frame at capture time
         * @param eta0 Output: initial 18-dim state estimate (attitude as a quaternion, rates zeroed)
         * @return True if enough landmarks associated to trust the solve
         */
        [[nodiscard]] bool solve_initial_pose(const filter::VisionSample& sample,
                                              const filter::Pose<double>& Tbc,
                                              const filter::Pose<double>& Twt,
                                              Eigen::Matrix<double, 18, 1>& eta0) const;
    };

}  // namespace module::localisation

#endif  // MODULE_LOCALISATION_FIELDLOCALISATIONSRIF_HPP
