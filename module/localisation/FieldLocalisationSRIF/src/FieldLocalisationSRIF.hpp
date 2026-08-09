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

    /**
     * @brief Short local aliases for the estimator types this module drives.
     *
     * The filter itself is module-local: its state layout, field map and measurement models all live
     * under src/srif and src/measurement. Only the generic estimator scaffolding it is built on
     * (Pose, Event/Measurement, SystemEstimator, GaussianInfo, funcmin) is shared, in utility::gaussian_filtering.
     * This re-exports the handful of types the reactor names so the class body reads cleanly.
     */
    namespace filter {
        using measurement::MeasurementFieldLandmarks;
        using srif::BodyTwistSample;
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
     * @brief Square-root information filter field localisation
     *
     * Estimates the torso pose in the field frame {f}, its body-fixed velocity, the gyroscope bias and a 2-DOF
     * camera-mount attitude bias, as a Gaussian in square-root information form. See
     * srif::SystemLocalisation for the state layout and why attitude is a quaternion.
     *
     * Nothing is a known input. The process model is rigid-body kinematics driven by the velocity states, and every
     * sensor enters as a measurement carrying its own noise: YOLO field-line intersections (L/T/X) and goal posts as
     * unit rays, the gyroscope as the body angular velocity, the walk-engine odometry as the body linear velocity,
     * plus gravity and the kinematic torso height as low-rate corrections. Updates are MAP optimisations
     * (trust-region Newton) over the robust landmark ray likelihood, yielding a Laplace-approximation posterior whose
     * covariance quantifies the estimate's uncertainty.
     *
     * Initialisation is a coarse grid-search over (x, y, yaw) on the first usable vision frame, scored by the same
     * landmark likelihood; roll, pitch and height come from the kinematic chain and the rates start at zero. The
     * field's 180 degree symmetry is broken at init with the rule that every robot starts in its own half, which
     * is +x by the codebase's field-frame convention. That prior is only true at kickoff, so recovering a
     * mid-game kidnap is the out-of-field disambiguator's job (use_side_disambiguator).
     *
     * A fall gates each measurement separately rather than suppressing all of them; see the posture block in the
     * vision reaction.
     */
    class FieldLocalisationSRIF : public NUClear::Reactor {
    public:
        /// @brief Called by the powerplant to build and set up the FieldLocalisationSRIF reactor.
        explicit FieldLocalisationSRIF(std::unique_ptr<NUClear::Environment> environment);

    private:
        /**
         * @brief Configuration parameters for this module.
         *
         * What the filter runs, how far each sensor is trusted, and how fast the belief may
         * move are in the yaml; everything tuned once and then left alone lives here, next to
         * the reasoning for its value.
         */
        struct Config {
            /// @brief Process noise PSDs [m/sqrt(s), rad/sqrt(s)]
            filter::SystemLocalisation::Parameters process{};
            /// @brief Multi-hypothesis (field-symmetry) mixture parameters
            filter::SystemLocalisation::HypothesisParameters hypothesis{};
            /// @brief Landmark measurement noise/association options
            filter::MeasurementFieldLandmarks::Options measurement{};
            /**
             * @brief Initial sqrt-covariance diagonal for the 18-dim state after the grid solve.
             *
             * The belief the filter starts from. Bigger means less certain:
             *
             *   x, y            how far the grid solve could be out. Its step is grid_step_xy, so a
             *                   cell or three.
             *   z, roll, pitch  how far the kinematic chain could be out -- the grid takes these from
             *                   it, unsearched.
             *   quaternion      the grid's attitude doubt (predominantly yaw). Yaw is not separable
             *                   across components, so all four carry the loose figure; 0.25 here is
             *                   about 0.5 rad of heading.
             *   v, omega        the robot is probably not moving yet.
             *   bg              prior on the gyroscope bias. ~3 deg/s covers the drift it absorbs.
             *   cam             prior on the camera extrinsics.
             *
             * Do not set an entry near zero. The filter stores the INVERSE of these (square-root
             * information form), so a near-zero std dev becomes a near-infinite information and the
             * estimate goes non-finite -- typically hundreds of frames later, far from the cause.
             */
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
            /// @brief Enable out-of-field side disambiguation.
            ///
            /// Maps background corners (walls, posters, spectators) from the raw camera frame and
            /// compares how well the current pose and its 180 degree mirror explain them. This is
            /// the only evidence that can separate the two, since on-field landmarks fit both
            /// equally -- see SideDisambiguator. Costs roughly 4 ms per frame, so it is a switch.
            bool use_side_disambiguator = true;
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
            /// @brief Std dev on the |q| = 1 pseudo-measurement (dimensionless).
            ///
            /// The four attitude states carry three degrees of freedom, and quat2rot normalises, so
            /// no other model here can see |q|. Without this the MAP Hessian is singular along it.
            double quaternion_norm_sigma = 1e-3;

            // --- body-rate measurements ---
            /// @brief Gyroscope noise std dev per axis [rad/s]
            double gyroscope_sigma = 0.02;
            /// @brief Feed the walk-engine odometry velocity as a measurement of vBb.
            bool use_odometry_velocity = true;
            /// @brief Walk-odometry velocity noise std dev per axis [m/s].
            ///
            /// Deliberately loose, comparable to the walk speed itself: the odometry slips on
            /// foot contact and reports the gait the engine believes it is executing. As a
            /// measurement that is evidence the filter can weigh, never truth it must accept.
            double odometry_velocity_sigma = 0.15;
            /// @brief Zero-velocity update noise while FALLEN (lying still) [m/s]
            double zupt_sigma = 0.02;
            /// @brief Zero-velocity update noise while FALLING or getting up [m/s].
            ///
            /// Toppling and being levered upright genuinely move the torso, just not anywhere.
            double zupt_dynamic_sigma = 0.30;

            // --- fall handling ---
            // Note: how long the elevated PSDs apply lives in process.disturbed_window, next to
            // the PSDs it governs, so it cannot be confused with the separate (and unbounded)
            // question of whether the odometry velocity is usable. See SystemLocalisation::
            // setPosture().
            /// @brief Horizontal position std restored on recovering from a fall [m]
            double recovery_pos_std = 0.5;
            /// @brief Yaw std restored on recovering from a fall [rad]
            double recovery_yaw_std = 0.6;
            /// @brief Maximum odometry sample spacing to finite-difference across [s]
            double max_odometry_gap = 0.1;
            /// @brief Length of the rolling odometry window the velocity samples are built from [s]
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
        /// @brief Body-fixed velocity samples finite-differenced from the odometry window.
        ///
        /// Feeds MeasurementBodyVelocity. Not an input to the process model -- the system holds no
        /// reference to it, and the vision reaction reads the sample nearest each frame.
        std::vector<filter::BodyTwistSample> twist_buffer;
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
         * @brief The body-velocity sample in the twist buffer nearest a given time.
         *
         * Feeds MeasurementBodyVelocity. Returns nullptr when the buffer is empty or its
         * nearest sample is too old to describe this frame.
         *
         * @param t Vision capture time [s since t0]
         * @return The nearest sample, or nullptr if there is nothing usable.
         */
        [[nodiscard]] const filter::BodyTwistSample* nearest_twist(double t) const;

        /**
         * @brief Hand confidence back to the belief on standing up again.
         *
         * Called on the upright transition only, from the posture block that runs before any early
         * return. It has to stay there: a getup ends in motion blur, so the frame the robot first
         * reads upright again often carries no usable detections, and anything inside the update
         * path is skipped on exactly those falls.
         *
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
         *
         * Detects background corners, scores the current pose against its 180 degree mirror, and
         * folds the result back in: as mixture-weight evidence when the hypothesis bank is running,
         * or as an outright state flip when it is not. Also publishes the per-frame working state
         * for NUsight.
         *
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
         * @brief Coarse global grid search for the initial pose on one vision frame.
         *
         * Roll, pitch and torso height come from the kinematic chain (the odometry
         * world frame is gravity-aligned); (x, y, yaw) are found by maximising the
         * landmark measurement log-likelihood over a grid, and the own-half convention
         * (own goal at +x) selects between the maximum and its 180 degree mirror.
         *
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
