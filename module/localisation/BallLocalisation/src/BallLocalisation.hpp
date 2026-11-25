/*
 * MIT License
 *
 * Copyright (c) 2022 NUbots
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

#ifndef MODULE_LOCALISATION_BALLLOCALISATION_HPP
#define MODULE_LOCALISATION_BALLLOCALISATION_HPP

#include <Eigen/Core>
#include <nuclear>
#include <vector>

#include "BallModel.hpp"

#include "message/input/Sensors.hpp"
#include "message/vision/Ball.hpp"

#include "utility/math/filter/UKF.hpp"

namespace module::localisation {

    class BallLocalisation : public NUClear::Reactor {
    private:
        struct Config {
            Config() = default;
            struct UKF {
                /// @brief Detection position standard deviation (m): base + per_metre * range from the camera
                double measurement_base      = 0.05;
                double measurement_per_metre = 0.1;
                /// @brief White-noise-acceleration spectral density (m^2/s^3)
                double acceleration_noise = 1.0;
                /// @brief Rolling deceleration of the ball (m/s^2)
                double rolling_deceleration = 0.0;
                /// @brief Covariance the filter restarts with when it (re)acquires a ball
                BallModel<double>::StateVec initial_covariance{};
            } ukf{};
            struct Association {
                /// @brief Squared Mahalanobis distance below which a detection updates the track
                double gate = 9.21;
                /// @brief Fastest the ball can travel (m/s), bounds which detections outside the gate can be a kick
                double max_ball_speed = 8.0;
                /// @brief Velocity standard deviation (m/s) injected when a kick is detected
                double kick_velocity_std = 4.0;
                /// @brief Extra distance (m) a detection may move between consecutive images and still confirm itself
                double confirm_radius = 0.3;
                /// @brief Seconds without an accepted detection before the track is lost
                double reacquire_after = 0.5;
                /// @brief Weight of the newest innovation in the running innovation average (0-1]
                double manoeuvre_smoothing = 0.5;
                /// @brief Normalised running innovation average above which the velocity covariance is opened up
                double manoeuvre_threshold = 3.0;
            } association{};
            /// @brief Whether or not to use teammate balls
            bool use_r2r_balls = false;
            /// @brief Timeout on stale teammate ball guesses
            double team_ball_recency = 0.0;
            /// @brief Max allowed std on teammate guesses
            double team_guess_error = 0.0;
            /// @brief Timeout for switching from own balls to teammate balls
            double team_guess_default_timer = 0.0;
            /// @brief Maximum distance from the field that a ball can be before it is ignored
            double max_distance_from_field = 0.0;
        } cfg;

        /// @brief A ball detection in world space, ready for association
        struct Candidate {
            Eigen::Vector2d rBWw = Eigen::Vector2d::Zero();
            /// @brief Measurement standard deviation (m) for this detection
            double sigma = 0.0;
        };

        /// @brief Whether the filter is tracking a ball at all (false until the first confirmed detection)
        bool tracking = false;
        /// @brief Image time the filter state refers to
        NUClear::clock::time_point filter_time{};
        /// @brief Image time of the last detection the track accepted
        NUClear::clock::time_point last_accept_time{};
        /// @brief Running average of the innovations, for manoeuvre detection
        Eigen::Vector2d innovation_bias = Eigen::Vector2d::Zero();
        /// @brief Detections from the previous image and its time, used to confirm kicks and re-acquisitions
        std::vector<Candidate> previous_candidates{};
        NUClear::clock::time_point previous_time{};

        /// @brief The time we last emitted a ball of our own (the teammate-ball fallback waits on it)
        NUClear::clock::time_point last_time_update;

        /// @brief The reactions that update the ball estimate. Disabled while input::NUSimGroundTruth is providing
        /// Ball, so this module does not emit its own
        ReactionHandle vision_ball_reaction{};
        ReactionHandle team_ball_reaction{};

        /// @brief Unscented Kalman Filter for ball filtering
        utility::math::filter::UKF<double, BallModel> ukf{};

        /// @brief Restart the track at a detection with zero velocity and the reacquisition covariance
        void reset_track(const Candidate& candidate, const NUClear::clock::time_point& time);

        /// @brief Whether a detection has a counterpart in the previous image it could have moved from
        [[nodiscard]] bool confirmed(const Candidate& candidate, const NUClear::clock::time_point& time) const;

        /// @brief Calculates ball position using robot to robot communication
        /// @return Whether the teammate ball is a valid guess and the average position of the ball in field space
        std::pair<bool, Eigen::Vector3d> get_average_team_rBFf();

        /// @brief A struct to hold the guess from a teammate
        struct TeamGuess {
            /// @brief The time the guess was given
            NUClear::clock::time_point last_heard = NUClear::clock::now();
            /// @brief The position of the ball in field space
            Eigen::Vector3d rBFf = Eigen::Vector3d::Zero();
        };

        /// @brief A vector of guesses from teammates, where the index is the player ID - 1
        std::vector<TeamGuess> team_guesses{};

        /// @brief The last Hcw from a ball measurement, to use with teammate balls
        Eigen::Isometry3d last_Hcw = Eigen::Isometry3d::Identity();

    public:
        /// @brief Called by the powerplant to build and setup the BallLocalisation reactor.
        explicit BallLocalisation(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace module::localisation

#endif  // MODULE_LOCALISATION_BALLLOCALISATION_HPP
