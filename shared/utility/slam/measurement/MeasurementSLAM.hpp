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

#ifndef UTILITY_SLAM_MEASUREMENT_MEASUREMENT_SLAM_HPP
#define UTILITY_SLAM_MEASUREMENT_MEASUREMENT_SLAM_HPP

#include <cstddef>
#include <vector>

#include "../camera/Camera.hpp"
#include "../gaussian/GaussianInfo.hpp"
#include "Measurement.hpp"
#include "../system/SystemSLAM.hpp"

namespace utility::slam::measurement {

    // Bring types into scope
    using camera::Camera;
    using system::SystemSLAM;

    class MeasurementSLAM : public Measurement {
    public:
        MeasurementSLAM(double time, const Camera& camera);
        virtual MeasurementSLAM* clone() const                                    = 0;
        virtual ~MeasurementSLAM() override;

        virtual gaussian::GaussianInfo<double> predictFeatureDensity(const SystemSLAM& system,
                                                                    std::size_t idxLandmark) const = 0;
        virtual gaussian::GaussianInfo<double> predictFeatureBundleDensity(
            const SystemSLAM& system,
            const std::vector<std::size_t>& idxLandmarks) const                                     = 0;
        virtual const std::vector<int>& associate(const SystemSLAM& system,
                                                const std::vector<std::size_t>& idxLandmarks) = 0;

    protected:
        const Camera& camera_;
    };

}  // namespace utility::slam::measurement

#endif  // UTILITY_SLAM_MEASUREMENT_MEASUREMENT_SLAM_HPP
