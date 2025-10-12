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

#include "association_util.hpp"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <limits>
#include <vector>

#include <Eigen/Core>

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/features2d.hpp>

namespace utility::slam {

    double snn(const SystemSLAM& system,
            const GaussianInfo<double>& featureBundleDensity,
            const std::vector<std::size_t>& idxLandmarks,
            const Eigen::Matrix<double, 2, Eigen::Dynamic>& Y,
            const Camera& camera,
            std::vector<int>& idxFeatures,
            bool enforceJointCompatibility) {

        double nSigma = 3.0;  // Number of standard deviations for confidence region

        assert(idxLandmarks.size() <= system.numberLandmarks());
        for (const auto& k : idxLandmarks) {
            assert(k < system.numberLandmarks());
        }

        const std::size_t& nL = idxLandmarks.size();
        assert(nL > 0);

        assert(Y.rows() == 2);
        assert(Y.cols() > 0);
        int m = Y.cols();

        // Index
        idxFeatures.clear();
        idxFeatures.resize(nL, -1);  // -1 is the sentinel index for unassociated landmarks

        std::vector<int> midx;
        midx.resize(m);
        for (int i = 0; i < m; ++i) {
            midx[i] = i;
        }

        // Surprisal per unassociated landmark
        double sU = std::log(camera.imageSize.width) + std::log(camera.imageSize.height);

        double s = nL * sU;

        double smin = std::numeric_limits<double>::infinity();
        std::vector<int> diff;
        std::vector<int>::iterator it, ls, space;
        diff.resize(m);
        for (int j = 0; j < nL; ++j) {
            GaussianInfo<double> featureDensity = featureBundleDensity.marginal(Eigen::seqN(2 * j, 2));

            double dsmin = std::numeric_limits<double>::infinity();
            double scur  = s;

            double snext = 0;
            bool jcnext;

            std::vector<int> idxcur;
            idxcur = idxFeatures;
            space  = idxcur.begin();
            std::advance(space, j);
            ls = std::set_difference(midx.begin(), midx.end(), idxcur.begin(), space, diff.begin());

            // associate landmark j with each unassociated feature
            for (it = diff.begin(); it < ls; ++it) {
                int i = *it;

                if (!individualCompatibility(Y.col(i), featureDensity, nSigma)) {
                    continue;
                }

                std::vector<int> idxnext = idxcur;
                idxnext[j]               = i;

                jcnext = jointCompatibility(idxnext, sU, Y, featureBundleDensity, nSigma, snext);
                if (enforceJointCompatibility && !jcnext) {
                    continue;
                }

                double ds = snext - scur;
                if (ds < dsmin) {
                    idxFeatures = idxnext;
                    dsmin       = ds;
                    s           = snext;
                }
            }

            // landmark j unassociated
            std::vector<int> idxnext = idxcur;
            jointCompatibility(idxnext, sU, Y, featureBundleDensity, nSigma, snext);  // Only compute the surprisal

            // Change in surprisal
            double ds = snext - scur;
            if (ds < dsmin) {
                idxFeatures = idxnext;
                s           = snext;
            }
        }

        if (smin < std::numeric_limits<double>::infinity()) {
            s = smin;
        }

        return s;
    }

    bool individualCompatibility(const int& i,
                                const int& j,
                                const Eigen::Matrix<double, 2, Eigen::Dynamic>& Y,
                                const GaussianInfo<double>& density,
                                const double& nSigma) {
        GaussianInfo<double> marginal = density.marginal(Eigen::seqN(2 * j, 2));
        return individualCompatibility(Y.col(i), marginal, nSigma);
    }

    bool individualCompatibility(const Eigen::Vector2d& y,
                                const GaussianInfo<double>& marginal,
                                const double& nSigma) {
        return marginal.isWithinConfidenceRegion(y, nSigma);
    }

    bool jointCompatibility(const std::vector<int>& idx,
                        const double& sU,
                        const Eigen::Matrix<double, 2, Eigen::Dynamic>& Y,
                        const GaussianInfo<double>& density,
                        const double& nSigma,
                        double& surprisal) {
        int n = idx.size();  // Number of landmarks expected to be visible

        std::vector<int> idxi;   // Indices of measured features for associated landmarks
        idxi.reserve(n);         // Reserve maximum possible size to avoid reallocation
        std::vector<int> idxyj;  // Indices of predicted feature vector for associated landmarks
        idxyj.reserve(2 * n);    // Reserve maximum possible size to avoid reallocation

        for (int k = 0; k < n; ++k) {
            if (idx[k] >= 0)  // If feature is associated with a landmark
            {
                idxi.push_back(idx[k]);
                idxyj.insert(idxyj.end(), {2 * k, 2 * k + 1});
            }
        }
        assert(2 * idxi.size() == idxyj.size());

        // Number of associated landmarks
        int nA = idxi.size();

        // Number of unassociated landmarks
        int nU = n - nA;

        // Set surprisal and return joint compatibility
        if (nA > 0) {
            // Extract marginal distribution for associated landmarks
            GaussianInfo<double> marginalA = density.marginal(idxyj);

            // Build the measurement vector for associated landmarks using Eigen indexing
            Eigen::VectorXd yA(2 * nA);
            Eigen::Map<Eigen::VectorXi> indices(idxi.data(), idxi.size());
            yA = Y(Eigen::all, indices).reshaped();

            // Compute surprisal for associated landmarks
            double surprisalA = -marginalA.log(yA);

            // Total surprisal = unassociated + associated
            surprisal = nU * sU + surprisalA;

            // Joint compatibility
            return marginalA.isWithinConfidenceRegion(yA, nSigma);
        }
        else {
            // Surprisal for unassociated landmarks only
            surprisal = nU * sU;

            // Joint compatibility with no associated landmarks is vacuously true
            return true;
        }
    }

}  // namespace utility::slam
