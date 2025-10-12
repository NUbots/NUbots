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

#ifndef UTILITY_SLAM_ASSOCIATION_UTIL_HPP
#define UTILITY_SLAM_ASSOCIATION_UTIL_HPP

#include <cstddef>
#include <vector>

#include <Eigen/Core>

#include "camera/Camera.hpp"
#include "gaussian/GaussianInfo.hpp"
#include "system/SystemSLAM.hpp"

namespace utility::slam {

    // Bring types into scope
    using camera::Camera;
    using gaussian::GaussianInfo;
    using system::SystemSLAM;

    /**
     * @brief Performs Sequential Nearest Neighbor (SNN) data association for SLAM
     *
     * @param system                        The SLAM system containing landmark information
     * @param featureBundleDensity          Joint Gaussian distribution of the feature bundle
     * @param idxLandmarks                  Indices of landmarks to be associated
     * @param Y                             Matrix of observed features (2 x num_features)
     * @param camera                        Camera parameters including image dimensions
     * @param idxFeatures                   Output vector of feature indices associated with landmarks (-1 for unassociated)
     * @param enforceJointCompatibility     Whether to enforce joint compatibility constraint
     * @return                              Total surprisal value of the association
     */
    double snn(const SystemSLAM& system,
            const GaussianInfo<double>& featureBundleDensity,
            const std::vector<std::size_t>& idxLandmarks,
            const Eigen::Matrix<double, 2, Eigen::Dynamic>& Y,
            const Camera& camera,
            std::vector<int>& idxFeatures,
            bool enforceJointCompatibility = false);

    /**
     * @brief Checks individual compatibility between a feature and a landmark
     *
     * @param i         Index of the feature in Y
     * @param j         Index of the landmark in the density
     * @param Y         Matrix of observed features (2 x num_features)
     * @param density   Joint Gaussian distribution
     * @param nSigma    Number of standard deviations for confidence region
     * @return          True if the feature-landmark pair is individually compatible
     */
    bool individualCompatibility(const int& i,
                                const int& j,
                                const Eigen::Matrix<double, 2, Eigen::Dynamic>& Y,
                                const GaussianInfo<double>& density,
                                const double& nSigma);

    /**
     * @brief Checks individual compatibility between a measurement and a marginal distribution
     *
     * @param y         Observed feature measurement (2D)
     * @param marginal  Marginal Gaussian distribution for the feature
     * @param nSigma    Number of standard deviations for confidence region
     * @return          True if the measurement is within the confidence region
     */
    bool individualCompatibility(const Eigen::Vector2d& y,
                                const GaussianInfo<double>& marginal,
                                const double& nSigma);

    /**
     * @brief Checks joint compatibility of a set of feature-landmark associations
     *
     * @param idx       Vector of feature indices associated with each landmark (-1 for unassociated)
     * @param sU        Surprisal per unassociated landmark
     * @param Y         Matrix of observed features (2 x num_features)
     * @param density   Joint Gaussian distribution of the feature bundle
     * @param nSigma    Number of standard deviations for confidence region
     * @param surprisal Output total surprisal value
     * @return          True if the association set is jointly compatible
     */
    bool jointCompatibility(const std::vector<int>& idx,
                        const double& sU,
                        const Eigen::Matrix<double, 2, Eigen::Dynamic>& Y,
                        const GaussianInfo<double>& density,
                        const double& nSigma,
                        double& surprisal);

}  // namespace utility::slam

#endif  // UTILITY_SLAM_ASSOCIATION_UTIL_HPP
