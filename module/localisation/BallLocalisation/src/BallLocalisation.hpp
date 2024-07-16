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

#include "message/eye/DataPoint.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/Ball.hpp"
#include "message/support/FieldDescription.hpp"
#include "message/vision/Ball.hpp"

#include "utility/math/filter/UKF.hpp"

namespace module::localisation {

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

            /// @brief Number of consistent measurements required to initialize the ball estimate
            int required_measurements = 3;

            /// @brief Maximum time window for clustering measurements (in seconds)
            double cluster_time_window = 1.0;

            /// @brief Maximum distance for clustering measurements (in meters)
            double cluster_distance_threshold = 0.5;

        } cfg;

        /// @brief Rejection count
        int rejection_count = 0;

        /// @brief Whether or not this is the first time we have seen a ball
        bool first_ball_seen = true;

        /// @brief Vector to store recent ball measurements for clustering
        std::vector<std::pair<Eigen::Vector2d, NUClear::clock::time_point>> recent_measurements;

        /// @brief Flag to indicate if the ball estimate has been initialized
        bool ball_estimate_initialized = false;

        /// @brief The time of the last time update
        NUClear::clock::time_point last_time_update;

        /// @brief Unscented Kalman Filter for ball filtering
        utility::math::filter::UKF<double, BallModel> ukf{};

    public:
        /// @brief Called by the powerplant to build and setup the BallLocalisation reactor.
        explicit BallLocalisation(std::unique_ptr<NUClear::Environment> environment);

        /// @brief Function to cluster recent measurements and initialize the ball estimate if criteria are met
        void cluster_and_initialize();

        /// @brief Function to update the ball estimate using the UKF
        void update_ball_estimate(const message::vision::Balls& balls, const message::support::FieldDescription& fd);
    };
}  // namespace module::localisation

#endif  // MODULE_LOCALISATION_BALLLOCALISATION_HPP
