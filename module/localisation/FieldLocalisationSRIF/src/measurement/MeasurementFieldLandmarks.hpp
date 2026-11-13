/*
 * MIT License
 *
 * Copyright (c) 2026 NUbots
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
/**
 * @file MeasurementFieldLandmarks.hpp
 * @brief Vision measurement of known field landmarks from YOLO detections.
 */
#ifndef MODULE_LOCALISATION_MEASUREMENT_MEASUREMENTFIELDLANDMARKS_HPP
#define MODULE_LOCALISATION_MEASUREMENT_MEASUREMENTFIELDLANDMARKS_HPP

#include <Eigen/Core>
#include <string>
#include <vector>

#include "srif/FieldMap.hpp"
#include "srif/FieldSamples.hpp"
#include "srif/SystemLocalisation.hpp"

#include "utility/gaussian_filtering/Pose.hpp"
#include "utility/gaussian_filtering/measurement/Measurement.hpp"
#include "utility/gaussian_filtering/rotation.hpp"
#include "utility/gaussian_filtering/system/SystemEstimator.hpp"

/**
 * @class MeasurementFieldLandmarks
 * @brief Measurement event for YOLO field landmark detections (goal posts, L/T/X intersections).
 *
 * Each detection gives a unit ray in the camera frame {c}: to the bounding box centre
 * for intersections, to its bottom-centre for goal posts. Rays are associated to mapped
 * landmarks of the same class by greedy nearest-surprisal assignment (see associate()).
 *
 * Each pair contributes a Gaussian on the chordal residual e = u_meas - u_pred, mixed with
 * uniform clutter over the sphere and weighted by the detection's confidence:
 *   log p = sum_j log[ w_j N(e_j; sigma^2 I) + (1 - w_j)/(4 pi) ]
 */

namespace module::localisation::measurement {

    using srif::Detection;
    using srif::FieldMap;
    using srif::LandmarkType;
    using srif::SystemLocalisation;
    using srif::VisionSample;
    using utility::gaussian_filtering::Pose;
    using utility::gaussian_filtering::tangent_basis;
    using utility::gaussian_filtering::gaussian::GaussianInfo;
    using utility::gaussian_filtering::measurement::Measurement;
    using utility::gaussian_filtering::system::SystemBase;
    using utility::gaussian_filtering::system::SystemEstimator;

    class MeasurementFieldLandmarks : public Measurement {
    public:
        /// @brief Association and noise options
        struct Options {
            double sigma_angular = 0.25;  ///< Inlier ray angular noise std dev [rad]
            double gate_angle    = 0.35;  ///< Max association residual angle [rad]

            /// Yaw std devs the pre-gate widens by
            double gate_yaw_scale = 2.0;
            double gate_angle_max = 1.0;  ///< Ceiling on the widened pre-gate [rad]

            double min_confidence     = 0.5;  ///< Reject detections below this confidence
            double inlier_probability = 0.7;  ///< Inlier mixture weight at confidence_reference

            double confidence_reference   = 0.7;   ///< Confidence that maps to inlier_probability
            double max_inlier_probability = 0.95;  ///< Cap on the inlier mixture weight
        };

        /**
         * @brief Construct and associate a landmark measurement.
         * @param time Event time [s]
         * @param sample Vision sample containing YOLO detections (rays in {c})
         * @param Tbc Camera pose w.r.t. torso at capture time
         * @param map Field landmark map
         * @param system System whose prior mean is used for data association
         * @param options Association and noise options
         */
        MeasurementFieldLandmarks(double time,
                                  const VisionSample& sample,
                                  const Pose<double>& Tbc,
                                  const FieldMap& map,
                                  const SystemLocalisation& system,
                                  const Options& options);

        /// @brief Construct with default options
        MeasurementFieldLandmarks(double time,
                                  const VisionSample& sample,
                                  const Pose<double>& Tbc,
                                  const FieldMap& map,
                                  const SystemLocalisation& system);

        virtual Eigen::VectorXd simulate(const Eigen::VectorXd& x, const SystemEstimator& system) const override;
        virtual double log_likelihood(const Eigen::VectorXd& x, const SystemEstimator& system) const override;
        virtual double log_likelihood(const Eigen::VectorXd& x,
                                      const SystemEstimator& system,
                                      Eigen::VectorXd& g) const override;
        virtual double log_likelihood(const Eigen::VectorXd& x,
                                      const SystemEstimator& system,
                                      Eigen::VectorXd& g,
                                      Eigen::MatrixXd& H) const override;

        /// @brief Templated log-likelihood for autodiff
        template <typename Scalar>
        Scalar log_likelihood_impl(const Eigen::VectorX<Scalar>& x) const;

        /// @brief Predicted unit rays in {c} for the associated landmarks
        template <typename Scalar>
        Eigen::Matrix<Scalar, 3, Eigen::Dynamic> predict_rays(const Eigen::VectorX<Scalar>& x) const;

        std::size_t num_associated() const {
            return static_cast<std::size_t>(u_meas_.cols());
        }
        const Eigen::Matrix<double, 3, Eigen::Dynamic>& measured_rays() const {
            return u_meas_;
        }

        /// @brief Re-associate the detections against the system's current pose, so each
        ///        hypothesis is scored on its own assignment
        void reassociate(const SystemEstimator& system) override {
            assoc_keys_ = associate(system.density.mean(), system.density.cov());
        }

    protected:
        /**
         * @brief MAP update, re-associating at the posterior mean and re-running the
         * optimisation from the prior if the association set changed.
         */
        virtual void update(SystemBase& system) override;

        /**
         * @brief Extract the measurement ray and landmark class for a detection.
         * @return true if the detection is a usable landmark class
         */
        static bool detection_ray(const Detection& det, Eigen::Vector3d& ray, LandmarkType& type);

        /// @brief A usable detection before association
        struct CandidateDetection {
            Eigen::Vector3d ray;
            LandmarkType type;
            std::size_t detection;  ///< Index into the vision sample's detection list
            double inlier_weight;   ///< Mixture inlier weight implied by this detection's confidence
        };

        /**
         * @brief Rebuild the associated pairs from the stored candidates at the given state,
         *        ranked by how much better the inlier component explains them than clutter.
         *
         * @param x State to predict the landmarks at
         * @param P State covariance, which inflates each predicted bearing
         * @return Association keys (candidate index, landmark index) for change detection
         */
        std::vector<std::pair<std::size_t, std::size_t>> associate(const Eigen::VectorXd& x, const Eigen::MatrixXd& P);

        const FieldMap& map_;                                          ///< Field landmark map
        std::vector<CandidateDetection> candidates_;                   ///< Usable detections (rays in {c})
        Pose<double> Tbc_;                                             ///< Camera pose w.r.t. torso at capture time
        Eigen::Matrix<double, 3, Eigen::Dynamic> u_meas_;              ///< Measured unit rays in {c} (associated only)
        Eigen::Matrix<double, 3, Eigen::Dynamic> rLFf_;                ///< Associated landmark positions in {f}
        std::vector<double> inlier_weight_;                            ///< Per-column mixture inlier weight
        std::vector<std::pair<std::size_t, std::size_t>> assoc_keys_;  ///< Last association (candidate, landmark)
        Options options_;                                              ///< Association and noise options
        int max_association_iterations_ = 1;                           ///< Maximum association/optimisation passes
    };

    template <typename Scalar>
    Eigen::Matrix<Scalar, 3, Eigen::Dynamic> MeasurementFieldLandmarks::predict_rays(
        const Eigen::VectorX<Scalar>& x) const {
        // Camera pose in field frame with mount-bias correction: Tfc = Tfb(x) * Tbc * R(deltaC)
        Pose<Scalar> Tfb = SystemLocalisation::field_pose(x);
        Pose<Scalar> Tbias(SystemLocalisation::camera_bias_rotation(x), Eigen::Vector3<Scalar>::Zero());
        Pose<Scalar> Tfc = Tfb * Pose<Scalar>(Tbc_) * Tbias;

        const Eigen::Matrix3<Scalar> Rcf  = Tfc.rotation_matrix.transpose();
        const Eigen::Vector3<Scalar> rCFf = Tfc.translation_vector;

        Eigen::Matrix<Scalar, 3, Eigen::Dynamic> u_pred(3, rLFf_.cols());
        for (Eigen::Index j = 0; j < rLFf_.cols(); ++j) {
            Eigen::Vector3<Scalar> rLCc = Rcf * (rLFf_.col(j).cast<Scalar>() - rCFf);
            u_pred.col(j)               = rLCc / rLCc.norm();
        }
        return u_pred;
    }

    template <typename Scalar>
    Scalar MeasurementFieldLandmarks::log_likelihood_impl(const Eigen::VectorX<Scalar>& x) const {
        const Eigen::Index n = u_meas_.cols();
        if (n == 0) {
            return Scalar(0);
        }

        Eigen::Matrix<Scalar, 3, Eigen::Dynamic> u_pred = predict_rays<Scalar>(x);

        const double sigma2         = options_.sigma_angular * options_.sigma_angular;
        const double log_norm_const = -std::log(2.0 * M_PI * sigma2);  // 2 effective DOF per ray

        using std::exp, std::log;

        Scalar log_lik = Scalar(0);
        for (Eigen::Index j = 0; j < n; ++j) {
            // Inlier Gaussian on the chordal residual, mixed with uniform clutter over the
            // unit sphere
            const double w                 = inlier_weight_[static_cast<std::size_t>(j)];
            const double log_inlier_weight = std::log(w);
            const double log_clutter       = std::log(1.0 - w) - std::log(4.0 * M_PI);

            Eigen::Vector3<Scalar> e = u_meas_.col(j).cast<Scalar>() - u_pred.col(j);
            Scalar a = Scalar(log_inlier_weight + log_norm_const) - Scalar(0.5) * e.squaredNorm() / Scalar(sigma2);
            Scalar b = Scalar(log_clutter);
            // Stable log-sum-exp of the two mixture components
            if (a > b) {
                log_lik += a + log(Scalar(1) + exp(b - a));
            }
            else {
                log_lik += b + log(Scalar(1) + exp(a - b));
            }
        }
        return log_lik;
    }
}  // namespace module::localisation::measurement

#endif  // MODULE_LOCALISATION_MEASUREMENT_MEASUREMENTFIELDLANDMARKS_HPP
