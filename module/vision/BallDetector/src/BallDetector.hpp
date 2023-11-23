/*
 * MIT License
 *
 * Copyright (c) 2013 NUbots
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

#ifndef MODULES_VISION_BALLDETECTOR_HPP
#define MODULES_VISION_BALLDETECTOR_HPP

#include <Eigen/Core>
#include <nuclear>

namespace module::vision {

    class BallDetector : public NUClear::Reactor {
    private:
        /// Configuration values
        struct {
            /// @brief Minimum confidence required for a ball point to be a ball point
            float confidence_threshold = 0.0f;
            /// @brief Minimum number of points for a cluster to be a viable ball
            int cluster_points = 0;
            /// @brief Minimum distance for a cluster to be a viable ball
            float minimum_ball_distance = 0.0f;
            /// @brief Percentage difference between width and projection based distances. 0.0 means that the distance
            /// measurements must match perfectly
            float distance_disagreement = 0.0f;
            /// @brief A threshold on how large the standard deviation of the angle between ray and cone axis can be
            float maximum_deviation = 0.0f;
            /// @brief Measurement certainties for ball localisation
            Eigen::Vector3f ball_angular_cov = Eigen::Vector3f::Zero();
        } cfg{};

    public:
        /// @brief Called by the powerplant to build and setup the BallDetector reactor.
        explicit BallDetector(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace module::vision


#endif
