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

#include <nuclear>

#include "BallModel.hpp"

#include "message/input/Sensors.hpp"
#include "message/vision/Ball.hpp"

#include "utility/math/filter/UKF.hpp"

namespace module::localisation {

    using VisionBalls = message::vision::Balls;

    class BallLocalisation : public NUClear::Reactor {
    private:
        struct Config {
            Config() = default;
            /// @brief UKF config
            struct UKF {
                struct Noise {
                    Noise() = default;
                    struct Measurement {
                        Eigen::Matrix2d position = Eigen::Matrix2d::Zero();
                    } measurement{};
                    struct Process {
                        Eigen::Vector2d position = Eigen::Vector2d::Zero();
                        Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
                    } process{};
                } noise{};
                struct Initial {
                    Initial() = default;
                    struct Mean {
                        Eigen::Vector2d position = Eigen::Vector2d::Zero();
                        Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
                    } mean{};
                    struct Covariance {
                        Eigen::Vector2d position = Eigen::Vector2d::Zero();
                        Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
                    } covariance{};
                } initial{};
            } ukf{};

            /// @brief Initial state of the for the UKF filter
            BallModel<double>::StateVec initial_mean;

            /// @brief Initial covariance of the for the UKF filter
            BallModel<double>::StateVec initial_covariance;

            /// @brief Acceptance radius for a ball measurement
            double acceptance_radius = 0.0;

            /// @brief Maximum number of detections of a ball not being accepted before it is accepted
            int max_rejections = 0;

            /// @brief Whether or not to use teammate balls
            bool use_r2r_balls = false;
            /// @brief Timeout on stale teammate ball guesses
            double team_ball_recency = 0.0;
            /// @brief Max allowed std on teammate guesses
            double team_guess_error = 0.0;
            /// @brief Timeout for switching from own balls to teammate balls
            double team_guess_default_timer = 0.0;

        } cfg;

        /// @brief Rejection count
        int rejection_count = 0;

        /// @brief Whether or not this is the first time we have seen a ball
        bool first_ball_seen = true;

        /// @brief The time of the last time update
        NUClear::clock::time_point last_time_update;

        /// @brief Unscented Kalman Filter for ball filtering
        utility::math::filter::UKF<double, BallModel> ukf{};

        /// @brief Calculates ball position using robot to robot communication
        /// @param average_rBFf The average position of the ball in field space
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
