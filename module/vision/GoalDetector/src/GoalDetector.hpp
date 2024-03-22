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

#ifndef MODULES_VISION_GOALDETECTOR_HPP
#define MODULES_VISION_GOALDETECTOR_HPP

#include <Eigen/Core>
#include <nuclear>

namespace module::vision {

    class GoalDetector : public NUClear::Reactor {
    private:
        struct {
            /// @brief Confidence threshold for point to be considered a goal post
            double confidence_threshold = 0.0f;
            /// @brief Minimum number of points in a cluster to be considered a goal post
            int cluster_points = 0;
            /// @brief Disagreement ratio threshold for goal post to be considered pair
            double disagreement_ratio = 0.0f;
        } cfg{};

    public:
        /// @brief Called by the powerplant to build and setup the GoalDetector reactor.
        explicit GoalDetector(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace module::vision


#endif
