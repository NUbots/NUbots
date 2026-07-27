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
#include <memory>
#include <nuclear>
#include <vector>

#include "utility/slam/FieldMap.hpp"
#include "utility/slam/FieldSamples.hpp"
#include "utility/slam/camera/Pose.hpp"
#include "utility/slam/measurement/MeasurementFieldLandmarks.hpp"
#include "utility/slam/system/SystemLocalisation.hpp"

namespace module::localisation {

    /**
     * @brief Short local aliases for the utility::slam estimator types this module drives.
     *
     * The estimator core lives in shared/utility/slam (namespace utility::slam::...); this
     * re-exports the handful of types the reactor names so the class body reads cleanly.
     */
    namespace filter {
        using utility::slam::Detection;
        using utility::slam::FieldDimensions;
        using utility::slam::FieldMap;
        using utility::slam::SensorsSample;
        using utility::slam::VisionSample;
        using utility::slam::camera::Pose;
        using utility::slam::measurement::MeasurementFieldLandmarks;
        using utility::slam::system::BodyTwistSample;
        using utility::slam::system::SystemLocalisation;
    }  // namespace filter

    /**
     * @brief Square-root information filter field localisation
     *
     * Estimates the 6-DOF torso pose in the field frame {f} plus a 2-DOF camera-mount attitude bias, as a Gaussian in
     * square-root information form. Prediction is driven by body-twist inputs finite-differenced from the odometry
     * stream (Sensors.Htw); measurement updates are MAP optimisations (trust-region Newton) over the robust landmark
     * ray likelihood, yielding a Laplace-approximation posterior with a covariance that quantifies the estimate's
     * uncertainty.
     *
     * Landmarks are the YOLO field-line intersections (L/T/X) and goal posts, consumed as unit rays in the camera
     * frame; gravity (accelerometer) and the kinematic torso height provide additional low-rate corrections.
     *
     * Initialisation is a coarse grid-search over (x, y, yaw) on the first usable vision frame, scored by the same
     * landmark likelihood. The field's 180 degree symmetry is broken with the game-context prior that the robot starts
     * in its own half (own_half_x_sign).
     */
    class FieldLocalisationSRIF : public NUClear::Reactor {
    public:
        /// @brief Called by the powerplant to build and set up the FieldLocalisationSRIF reactor.
        explicit FieldLocalisationSRIF(std::unique_ptr<NUClear::Environment> environment);

    private:
        /// @brief Configuration parameters for this module
        struct Config {
            /// @brief Process noise PSDs [m/sqrt(s), rad/sqrt(s)]
            filter::SystemLocalisation::Parameters process{};
            /// @brief Multi-hypothesis (field-symmetry) mixture parameters
            filter::SystemLocalisation::HypothesisParameters hypothesis{};
            /// @brief Landmark measurement noise/association options
            filter::MeasurementFieldLandmarks::Options measurement{};
            /// @brief Initial sqrt-covariance diagonal for the 9-dim state after the grid solve.
            ///
            /// Indices 3..6 are the attitude quaternion's components, not roll/pitch/yaw. A
            /// tangent-space std of s becomes a component std of s/2 (see
            /// SystemLocalisation::quaternionSigma), so the 0.5 rad of initial yaw doubt the old
            /// layout carried at index 5 is 0.25 spread across the four components here.
            Eigen::Matrix<double, 9, 1> initial_sqrt_covariance =
                (Eigen::Matrix<double, 9, 1>() << 1.0, 1.0, 0.05, 0.25, 0.25, 0.25, 0.25, 0.02, 0.02).finished();
            /// @brief Sign of field-x for the starting half (from game context; breaks the field symmetry)
            double own_half_x_sign = 1.0;
            /// @brief Grid search steps for the initial pose solve
            double grid_step_xy  = 0.35;
            double grid_step_yaw = 18.0 * M_PI / 180.0;
            /// @brief Minimum landmark associations to trust an initial grid solve
            int min_init_associations = 4;
            /// @brief Enable the multi-hypothesis (field-symmetry) Gaussian mixture bank
            bool use_hypothesis_bank = false;
            /// @brief Feed Sensors.gyroscope into the body-twist angular velocity.
            ///
            /// Sensors.gyroscope is a calibrated rad/s torso-frame rate on both hardware and webots (the
            /// same NUgus.proto lookup table decodes it in each). When off, the angular velocity is
            /// finite-differenced from the odometry attitude in Htw instead.
            bool use_gyroscope = true;
            /// @brief Enable the accelerometer gravity measurement.
            ///
            /// Uses the calibrated m/s^2 Sensors.accelerometer (physical on both hardware and webots).
            /// The gravity-aligned roll/pitch is also estimated upstream by the Mahony filter and
            /// delivered via Htw, so this is a secondary attitude anchor and can be disabled if Htw is
            /// trusted on its own.
            bool use_gravity = true;
            /// @brief Accelerometer gravity-direction noise std dev [m/s^2]
            double gravity_sigma = 1.0;
            /// @brief How far the specific-force magnitude may sit from standard gravity and still
            ///        be treated as a gravity reading [m/s^2].
            ///
            /// The accelerometer measures gravity whenever the torso is not being accelerated,
            /// which is true of a robot lying still on the carpet and false of one in free fall or
            /// hitting the ground -- a property of the specific force, not of the posture, so this
            /// is the condition rather than "is the robot upright". Loose on purpose: ordinary gait
            /// swings the magnitude by a couple of m/s^2 and the model already carries
            /// gravity_sigma of noise.
            double gravity_quasi_static_tolerance = 3.0;
            /// @brief Enable the kinematic torso-height measurement
            bool use_kinematic_height = true;
            /// @brief Kinematic torso-height noise std dev [m]
            double height_sigma = 0.02;
            /// @brief Std dev on the |q| = 1 pseudo-measurement (dimensionless)
            double quaternion_norm_sigma = 1e-3;

            // --- fall handling ---
            /// @brief How long into a fall the disturbed process PSDs apply [s].
            ///
            /// A fall is a bounded event. Running the disturbed PSDs for its whole duration models
            /// a robot lying still as a random walk, so the belief would report how long it had
            /// been down rather than how far it could have gone.
            double disturbed_window = 2.0;
            /// @brief Horizontal position std restored on recovering from a fall [m]
            double recovery_pos_std = 0.5;
            /// @brief Yaw std restored on recovering from a fall [rad]
            double recovery_yaw_std = 0.6;
            /// @brief Maximum odometry sample spacing to finite-difference across [s]
            double max_odometry_gap = 0.1;
            /// @brief Length of the rolling odometry window used to build the twist buffer [s]
            double twist_window_seconds = 2.0;
            /// @brief Maximum age of the odometry sample paired with a vision frame [s]
            double max_sensor_pairing_age = 0.1;
        } cfg;

        /// @brief Time origin (first Sensors capture) that all event times are measured from
        NUClear::clock::time_point t0{};
        /// @brief Whether t0 has been captured yet
        bool have_t0 = false;

        /// @brief Rolling window of recent odometry samples, newest last (bounded by twist_window_seconds)
        std::vector<filter::SensorsSample> sensors_window;
        /// @brief Body-twist input buffer the system predicts against (owned; system_ holds a pointer to it)
        std::vector<filter::BodyTwistSample> twist_buffer;
        /// @brief The estimator (null until the first successful initial-pose solve)
        std::unique_ptr<filter::SystemLocalisation> system;
        /// @brief Field landmark map (null until the first FieldDescription arrives)
        std::unique_ptr<filter::FieldMap> map;
        /// @brief Whether the estimator has been initialised from a landmark frame
        bool initialised = false;

        /// @brief Whether the robot was upright on the previous vision frame.
        ///
        /// Recovery fires on the transition back to upright, not merely on "the last frame was
        /// not upright": measurement updates keep running while fallen, so the latter would
        /// re-inflate the belief on every frame of the fall.
        bool was_upright = true;
        /// @brief Time the current non-upright episode began [s since t0]
        double fall_start_t = 0.0;

        /// @brief Seconds since the module time origin t0
        [[nodiscard]] double seconds(const NUClear::clock::time_point& tp) const {
            return std::chrono::duration_cast<std::chrono::duration<double>>(tp - t0).count();
        }

        /**
         * @brief The odometry sample in the rolling window nearest a given time.
         *
         * A vision frame carries Hcw at its capture time, so the torso pose used to build the
         * camera-to-torso transform must be the odometry sample from that same time, not the latest
         * one (the torso moves between capture and now). Mirrors the offline nearest-timestamp pairing.
         *
         * @param t Vision capture time [s since t0]
         * @return The nearest sample, or nullptr if the window is empty or the nearest is too old.
         */
        [[nodiscard]] const filter::SensorsSample* nearest_sensors(double t) const;

        /**
         * @brief Emit the localisation Field message (and debug graphs) from the current belief.
         * @param Htw World-to-torso pose paired with the processed vision frame (from Sensors)
         * @param measurement The processed landmark measurement (nullptr on the bootstrap frame)
         */
        void emit_field(const filter::Pose<double>& Htw, const filter::MeasurementFieldLandmarks* measurement);

        /**
         * @brief Coarse global grid search for the initial pose on one vision frame.
         *
         * Roll, pitch and torso height come from the kinematic chain (the odometry
         * world frame is gravity-aligned); (x, y, yaw) are found by maximising the
         * landmark measurement log-likelihood over a grid, and the own-half prior
         * (own_half_x_sign) selects between the maximum and its 180 degree mirror.
         *
         * @param sample Landmark rays extracted from the vision messages
         * @param Tbc Camera pose w.r.t. torso at capture time
         * @param Twt Torso pose in the odometry world frame at capture time
         * @param eta0 Output: initial 9-dim state estimate (attitude as a quaternion)
         * @return True if enough landmarks associated to trust the solve
         */
        [[nodiscard]] bool solve_initial_pose(const filter::VisionSample& sample,
                                              const filter::Pose<double>& Tbc,
                                              const filter::Pose<double>& Twt,
                                              Eigen::Matrix<double, 9, 1>& eta0) const;
    };

}  // namespace module::localisation

#endif  // MODULE_LOCALISATION_FIELDLOCALISATIONSRIF_HPP
