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
#ifndef MODULE_LOCALISATION_COOPERATIVELOCALISATION_HPP
#define MODULE_LOCALISATION_COOPERATIVELOCALISATION_HPP

#include <Eigen/Core>
#include <map>
#include <nuclear>
#include <set>
#include <vector>

#include "SquareRootEKF.hpp"

#include "message/localisation/Cooperative.hpp"
#include "message/support/FieldDescription.hpp"

namespace module::localisation {

    /**
     * @brief Cooperative (joint) tracking of the ball and the other robots, fusing this robot's detections with the
     * detections its teammates share over team communication into one square-root extended Kalman filter per target.
     *
     * Ported from the cooperative tracking evaluation in Sebastian Brooker's soccer-sim (FYP), where the joint filter
     * tracked the ball and opponents better than each robot tracking on its own. See the README for the models.
     */
    class CooperativeLocalisation : public NUClear::Reactor {
    public:
        /// @brief What a track (and a detection) is of
        enum class Target { BALL, ROBOT };

    private:
        struct Config {
            /// @brief Process model of one kind of target: constant velocity with velocity damping
            struct Motion {
                /// @brief Velocity damping (1/s), 0 for constant velocity
                double friction = 0.0;
                /// @brief Std of the white acceleration noise driving the velocity (m/s^2/sqrt(Hz))
                double acceleration_std = 0.0;
                /// @brief Velocity std of a new track (m/s)
                double initial_velocity_std = 0.0;
            };
            Motion ball{};
            Motion robot{};

            /// @brief Range/bearing measurement noise of one kind of detection
            struct Measurement {
                /// @brief Range std (m) is range_base + range_quad * r^2
                double range_base = 0.0;
                double range_quad = 0.0;
                /// @brief Bearing std (rad)
                double bearing_std = 0.0;
                /// @brief Multiplier on the observer's position covariance, covering its correlation over time
                double observer_inflation = 1.0;
            };
            Measurement ball_measurement{};
            Measurement robot_measurement{};
            /// @brief Std of the observer's own position on the field, per axis (m)
            double observer_position_std = 0.0;
            /// @brief Std of the observer's own heading (rad)
            double observer_heading_std = 0.0;

            /// @brief A detection can update a track only if its squared Mahalanobis distance is at most this
            double gate_chi2 = 0.0;
            /// @brief Add log det of the innovation covariance to the association cost
            bool association_log_det = true;
            /// @brief A robot track this close (m) to the observer cannot take its detections (it never sees itself)
            double self_exclusion = 0.0;
            /// @brief No robot track is started from a detection closer than this (m) to the observer
            double min_spawn_range = 0.0;
            /// @brief Detections further than this (m) outside the field are ignored, and tracks removed
            double max_distance_from_field = 0.0;
            /// @brief Detections older than this (s) when they arrive are ignored
            double max_detection_age = 0.0;

            /// @brief A robot detection consistent with a teammate's reported position (chi-squared, 2 dof) is that
            /// teammate, and is not used for tracking
            double teammate_gate_chi2 = 0.0;
            /// @brief Reported teammate positions older than this (s) are not used
            double teammate_max_age = 0.0;
            /// @brief Speed (m/s) a teammate could have moved at since its report
            double teammate_max_speed = 0.0;

            /// @brief Remove a robot track whose largest position std grows beyond this (m)
            double robot_lost_std = 0.0;
            /// @brief Remove a ball track whose largest position std grows beyond this (m)
            double ball_lost_std = 0.0;
            /// @brief Two tracks within this squared Mahalanobis distance of each other are duplicates
            double merge_chi2 = 0.0;
            /// @brief Only tracks with largest position std at most this (m) can be called duplicates
            double merge_max_std = 0.0;
            /// @brief Restart the ball after this many scans in a row whose ball detections all fail its gate
            int ball_reset_count = 0;
            /// @brief A ball detection with range std above this (m) never starts a track on its own ...
            double spawn_max_range_std = 0.0;
            /// @brief ... it waits this long (s) for a second robot's detection to corroborate it
            double pending_window = 0.0;
            /// @brief Robots whose detection was fused within this time (s) are listed as observers of the ball
            double observer_window = 0.0;
        } cfg;

        /// @brief One detection on the field {f}, from one camera frame of one robot
        struct Observation {
            /// @brief What was detected
            Target target = Target::BALL;
            /// @brief Position of the observing camera on the field
            Eigen::Vector2d rCFf = Eigen::Vector2d::Zero();
            /// @brief The detected position on the field
            Eigen::Vector2d rXFf = Eigen::Vector2d::Zero();
            /// @brief When the camera frame was taken
            NUClear::clock::time_point time{};
            /// @brief Player id of the observing robot
            uint32_t observer = 0;
        };

        /// @brief The range/bearing measurement model of one observation against one track
        struct Prediction {
            /// @brief Measurement [u; r]: the unit bearing on the field and the range
            Eigen::Vector3d y = Eigen::Vector3d::Zero();
            /// @brief Predicted measurement at the track mean
            Eigen::Vector3d h = Eigen::Vector3d::Zero();
            /// @brief Jacobian of the measurement with respect to the state
            Eigen::Matrix<double, 3, 4> H = Eigen::Matrix<double, 3, 4>::Zero();
            /// @brief Measurement noise covariance
            Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        };

        struct Track {
            /// @brief Unique id of the track
            unsigned long id = 0;
            /// @brief What the track is of
            Target target = Target::BALL;
            /// @brief Filter over [x, y, vx, vy] on the field {f}
            SquareRootEKF<4> filter{};
            /// @brief The time the filter state is at
            NUClear::clock::time_point time{};
            /// @brief The time of the most recent detection fused (or that started the track)
            NUClear::clock::time_point last_measurement{};
            /// @brief Number of detections fused (including the one that started the track)
            unsigned long fusions = 0;
            /// @brief Ball only: scans in a row whose ball detections all failed this track's gate
            int rejections = 0;
            /// @brief Tracks that took a different detection from the same scan, so are different objects
            std::set<unsigned long> distinct{};
            /// @brief Last time a detection of each observer (player id) was fused
            std::map<uint32_t, NUClear::clock::time_point> observers{};

            [[nodiscard]] Eigen::Vector2d position() const {
                return filter.mean().head<2>();
            }
            [[nodiscard]] Eigen::Matrix2d position_covariance() const {
                return filter.covariance().topLeftCorner<2, 2>();
            }
            /// @brief The largest position standard deviation
            [[nodiscard]] double max_std() const;
        };

        /// @brief A ball detection too poorly ranged to start a track by itself, waiting for a second robot's
        struct Pending {
            Observation observation{};
        };

        /// @brief A teammate's (or this robot's own) last known position on the field
        struct TeammatePosition {
            NUClear::clock::time_point time{};
            Eigen::Vector2d rRFf = Eigen::Vector2d::Zero();
        };

        /// @brief The tracks, of every target
        std::vector<Track> tracks{};
        /// @brief Ball detections waiting for corroboration
        std::vector<Pending> pending{};
        /// @brief Last known position of each teammate and of this robot, by player id
        std::map<uint32_t, TeammatePosition> teammates{};
        /// @brief This robot's most recent ball and robot detections, for network::RobotCommunication to share
        message::localisation::CooperativeDetections detections{};
        /// @brief This robot's player id
        uint32_t player_id = 0;
        /// @brief The id of the next track
        unsigned long next_id = 1;

        /// @brief The field description, for the field bounds and ball radius
        message::support::FieldDescription field_description{};
        bool have_field_description = false;

        /// @brief How many times per second to run maintenance and emit the estimates
        static constexpr int UPDATE_RATE = 15;

        /// @brief Process model of a target
        [[nodiscard]] const Config::Motion& motion(Target target) const {
            return target == Target::BALL ? cfg.ball : cfg.robot;
        }
        /// @brief Measurement model of a target
        [[nodiscard]] const Config::Measurement& measurement(Target target) const {
            return target == Target::BALL ? cfg.ball_measurement : cfg.robot_measurement;
        }

        /// @brief Predict every track to the given time
        void predict(const NUClear::clock::time_point& time);

        /// @brief Fuse one scan: every detection from one camera frame of one robot
        void process_scan(std::vector<Observation> observations);

        /// @brief Whether a position is within the field plus max_distance_from_field
        [[nodiscard]] bool on_field(const Eigen::Vector2d& rXFf) const;

        /// @brief Whether a robot detection is one of the observer's teammates (or this robot)
        [[nodiscard]] bool is_teammate(const Observation& observation) const;

        /// @brief The range std of a detection at range r
        [[nodiscard]] double range_std(Target target, double r) const;

        /// @brief The covariance of an observation's position on the field
        [[nodiscard]] Eigen::Matrix2d observation_covariance(const Observation& observation) const;

        /// @brief The range/bearing measurement model of an observation against a track
        [[nodiscard]] Prediction predict_measurement(const Track& track, const Observation& observation) const;

        /// @brief Squared Mahalanobis distance of an observation from a track, and the association cost
        [[nodiscard]] std::pair<double, double> gate(const Track& track, const Observation& observation) const;

        /// @brief Fuse an observation into a track
        void fuse(Track& track, const Observation& observation);

        /// @brief Start a new track from an observation
        Track& spawn(const Observation& observation);

        /// @brief Match a poor ball detection against the pending ones from other robots. If two rays agree, start
        /// a ball track at their intersection and return true, otherwise park the detection and return false
        bool corroborate(const Observation& observation);

        /// @brief Remove lost, out-of-field and duplicate tracks, and all but the best ball track
        void maintenance();

    public:
        /// @brief Called by the powerplant to build and setup the CooperativeLocalisation reactor.
        explicit CooperativeLocalisation(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::localisation

#endif  // MODULE_LOCALISATION_COOPERATIVELOCALISATION_HPP
