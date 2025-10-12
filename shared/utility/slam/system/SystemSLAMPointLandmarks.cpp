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

#include "SystemSLAMPointLandmarks.hpp"

#include <cmath>

#include <Eigen/Core>

namespace utility::slam::system {

    SystemSLAMPointLandmarks::SystemSLAMPointLandmarks(const gaussian::GaussianInfo<double>& density)
        : SystemSLAM(density) {}

    SystemSLAM* SystemSLAMPointLandmarks::clone() const {
        return new SystemSLAMPointLandmarks(*this);
    }

    std::size_t SystemSLAMPointLandmarks::numberLandmarks() const {
        return (density.dim() - 12) / 3;
    }

    std::size_t SystemSLAMPointLandmarks::landmarkPositionIndex(std::size_t idxLandmark) const {
        assert(idxLandmark < numberLandmarks());
        return 12 + 3 * idxLandmark;
    }

}  // namespace utility::slam::system
