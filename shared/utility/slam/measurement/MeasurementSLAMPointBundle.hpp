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

#ifndef UTILITY_SLAM_MEASUREMENT_MEASUREMENT_SLAM_POINT_BUNDLE_HPP
#define UTILITY_SLAM_MEASUREMENT_MEASUREMENT_SLAM_POINT_BUNDLE_HPP

#include <Eigen/Core>

#include "../camera/Camera.hpp"
#include "../camera/Pose.hpp"
#include "../system/SystemBase.hpp"
#include "../system/SystemEstimator.hpp"
#include "MeasurementSLAM.hpp"

namespace utility::slam::measurement {

    // Bring additional types into scope
    using camera::Pose;

    class MeasurementPointBundle : public MeasurementSLAM {
    public:
        MeasurementPointBundle(double time, const Eigen::Matrix<double, 2, Eigen::Dynamic>& Y, const Camera& camera);
        MeasurementSLAM* clone() const override;
        virtual Eigen::VectorXd simulate(const Eigen::VectorXd& x, const SystemEstimator& system) const override;
        virtual double logLikelihood(const Eigen::VectorXd& x, const SystemEstimator& system) const override;
        virtual double logLikelihood(const Eigen::VectorXd& x,
                                     const SystemEstimator& system,
                                     Eigen::VectorXd& g) const override;
        virtual double logLikelihood(const Eigen::VectorXd& x,
                                     const SystemEstimator& system,
                                     Eigen::VectorXd& g,
                                     Eigen::MatrixXd& H) const override;

        template <typename Scalar>
        Eigen::Vector2<Scalar> predictFeature(const Eigen::VectorX<Scalar>& x,
                                              const SystemSLAM& system,
                                              std::size_t idxLandmark) const;
        Eigen::Vector2d predictFeature(const Eigen::VectorXd& x,
                                       Eigen::MatrixXd& J,
                                       const SystemSLAM& system,
                                       std::size_t idxLandmark) const;
        virtual gaussian::GaussianInfo<double> predictFeatureDensity(const SystemSLAM& system,
                                                                     std::size_t idxLandmark) const override;

        template <typename Scalar>
        Eigen::VectorX<Scalar> predictFeatureBundle(const Eigen::VectorX<Scalar>& x,
                                                    const SystemSLAM& system,
                                                    const std::vector<std::size_t>& idxLandmarks) const;
        Eigen::VectorXd predictFeatureBundle(const Eigen::VectorXd& x,
                                             Eigen::MatrixXd& J,
                                             const SystemSLAM& system,
                                             const std::vector<std::size_t>& idxLandmarks) const;
        virtual gaussian::GaussianInfo<double> predictFeatureBundleDensity(
            const SystemSLAM& system,
            const std::vector<std::size_t>& idxLandmarks) const override;

        virtual const std::vector<int>& associate(const SystemSLAM& system,
                                                  const std::vector<std::size_t>& idxLandmarks) override;
        template <typename Scalar>
        Scalar logLikelihoodTemplated(const Eigen::VectorX<Scalar>& x, const SystemSLAM& system) const;

        // Accessors for plotting
        const std::vector<std::size_t>& getVisibleLandmarks() const {
            return visibleLandmarks_;
        }
        const std::vector<int>& getAssociations() const {
            return idxFeatures_;
        }

    protected:
        virtual void update(SystemBase& system) override;
        Eigen::Matrix<double, 2, Eigen::Dynamic> Y_;  // Feature bundle
        double sigma_;                                // Feature error standard deviation (in pixels)
        std::vector<int> idxFeatures_;                // Features associated with visible landmarks
        std::vector<std::size_t> visibleLandmarks_;   // Visible landmarks
    };

    // Image feature location for a given landmark
    template <typename Scalar>
    Eigen::Vector2<Scalar> MeasurementPointBundle::predictFeature(const Eigen::VectorX<Scalar>& x,
                                                                  const SystemSLAM& system,
                                                                  std::size_t idxLandmark) const {
        // Obtain camera pose from state
        Pose<Scalar> Tnc;
        Tnc.translationVector = system.cameraPosition(camera_, x);     // rCNn
        Tnc.rotationMatrix    = system.cameraOrientation(camera_, x);  // Rnc

        // Obtain landmark position from state
        std::size_t idx             = system.landmarkPositionIndex(idxLandmark);
        Eigen::Vector3<Scalar> rPNn = x.template segment<3>(idx);

        // Camera vector
        Eigen::Vector3<Scalar> rPCc;
        rPCc = Tnc.rotationMatrix.transpose() * (rPNn - Tnc.translationVector);

        // Pixel coordinates
        Eigen::Vector2<Scalar> rQOi;
        rQOi = camera_.vectorToPixel(rPCc);
        return rQOi;
    }

    // Image feature locations for a bundle of landmarks
    template <typename Scalar>
    Eigen::VectorX<Scalar> MeasurementPointBundle::predictFeatureBundle(
        const Eigen::VectorX<Scalar>& x,
        const SystemSLAM& system,
        const std::vector<std::size_t>& idxLandmarks) const {
        assert(x.size() == system.density.dim());

        const std::size_t& nL = idxLandmarks.size();
        Eigen::VectorX<Scalar> h(2 * nL);
        for (std::size_t i = 0; i < nL; ++i) {
            Eigen::Vector2<Scalar> rQOi = predictFeature(x, system, idxLandmarks[i]);
            // Set pair of elements of h
        }
        return h;
    }

    // Templated log-likelihood for autodiff support
    template <typename Scalar>
    Scalar MeasurementPointBundle::logLikelihoodTemplated(const Eigen::VectorX<Scalar>& x,
                                                          const SystemSLAM& system) const {
        const SystemSLAM& systemSLAM = dynamic_cast<const SystemSLAM&>(system);

        Scalar logLik = Scalar(0.0);

        // Count unassociated landmarks for penalty term
        int numUnassociated = 0;
        for (int assoc : idxFeatures_) {
            if (assoc < 0)
                numUnassociated++;
        }

        // Create measurement noise model
        Eigen::MatrixX<Scalar> S                        = Scalar(sigma_) * Eigen::MatrixX<Scalar>::Identity(2, 2);
        gaussian::GaussianInfo<Scalar> measurementModel = gaussian::GaussianInfo<Scalar>::fromSqrtMoment(S);

        // Sum log-likelihoods over all associated feature/landmark pairs
        for (std::size_t j = 0; j < idxFeatures_.size(); ++j) {
            if (idxFeatures_[j] >= 0) {  // This landmark is associated
                int detectionIdx   = idxFeatures_[j];
                size_t landmarkIdx = visibleLandmarks_[j];
                // Predict single point feature for this landmark
                Eigen::Vector2<Scalar> h_pred = predictFeature(x, systemSLAM, landmarkIdx);

                // Get measured point position (always double)
                Eigen::Vector2d y_i = Y_.col(detectionIdx);

                // Compute residual
                Eigen::Vector2<Scalar> residual;
                residual(0) = Scalar(y_i(0)) - h_pred(0);
                residual(1) = Scalar(y_i(1)) - h_pred(1);

                // Add log-likelihood contribution
                logLik += measurementModel.log(residual);
            }
        }

        // Add penalty term for unassociated visible landmarks
        double imageArea = camera_.imageSize.width * camera_.imageSize.height;
        logLik -= Scalar(numUnassociated * std::log(imageArea));

        return logLik;
    }

}  // namespace utility::slam::measurement

#endif  // UTILITY_SLAM_MEASUREMENT_MEASUREMENT_SLAM_POINT_BUNDLE_HPP
