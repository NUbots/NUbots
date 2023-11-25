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

#include "message/support/FieldDescription.hpp"
#include "message/vision/Goal.hpp"
#include "message/vision/GreenHorizon.hpp"

namespace module::vision {

    class GoalDetector : public NUClear::Reactor {
    private:
        struct {
            float confidence_threshold                 = 0.0f;
            int cluster_points                         = 0;
            float disagreement_ratio                   = 0.0f;
            Eigen::Vector3f goal_projection_covariance = Eigen::Vector3f::Zero();
            bool use_median                            = false;
            float max_goal_distance                    = 0;
            float max_benchmark_error                  = 1.0;
        } config{};

    public:
        /// @brief Called by the powerplant to build and setup the GoalDetector reactor.
        explicit GoalDetector(std::unique_ptr<NUClear::Environment> environment);

        void benchmark_goals(const message::support::FieldDescription& field,
                             const message::vision::GreenHorizon& horizon,
                             std::unique_ptr<message::vision::Goals>& goals);
    };
}  // namespace module::vision


#endif
