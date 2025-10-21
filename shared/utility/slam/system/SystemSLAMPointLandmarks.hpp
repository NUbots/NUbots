/*
 * MIT License
 *
 * Copyright (c) 2025 NUbots
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

#ifndef UTILITY_SLAM_SYSTEM_SLAM_POINT_LANDMARKS_HPP
#define UTILITY_SLAM_SYSTEM_SLAM_POINT_LANDMARKS_HPP

#include <Eigen/Core>

#include "../gaussian/GaussianInfo.hpp"
#include "SystemSLAM.hpp"

namespace utility::slam::system {

    /*
     * State containing body velocities, body pose and landmark positions
     *
     *     [ vBNb     ]  Body translational velocity (body-fixed)
     *     [ omegaBNb ]  Body angular velocity (body-fixed)
     *     [ rBNn     ]  Body position (world-fixed)
     * x = [ Thetanb  ]  Body orientation (world-fixed)
     *     [ rL1Nn    ]  Landmark 1 position (world-fixed)
     *     [ rL2Nn    ]  Landmark 2 position (world-fixed)
     *     [ ...      ]  ...
     *
     */
    class SystemSLAMPointLandmarks : public SystemSLAM {
    public:
        explicit SystemSLAMPointLandmarks(const gaussian::GaussianInfo<double>& density);
        SystemSLAM* clone() const override;
        virtual std::size_t numberLandmarks() const override;
        virtual std::size_t landmarkPositionIndex(std::size_t idxLandmark) const override;

        // Track consecutive association failures for landmark deletion
        std::vector<int> consecutiveFailures_;
    };

}  // namespace utility::slam::system

#endif  // UTILITY_SLAM_SYSTEM_SLAM_POINT_LANDMARKS_HPP
