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
 * @file MeasurementFieldLines.h
 * @brief Field-line point measurement against the analytic field line map.
 */
#ifndef MODULE_LOCALISATION_MEASUREMENT_MEASUREMENTFIELDLINES_HPP
#define MODULE_LOCALISATION_MEASUREMENT_MEASUREMENTFIELDLINES_HPP

#include <Eigen/Core>
#include <vector>

#include "srif/FieldMap.hpp"
#include "srif/FieldSamples.hpp"
#include "srif/SystemLocalisation.hpp"

#include "utility/gaussian_filtering/Pose.hpp"
#include "utility/gaussian_filtering/measurement/Measurement.hpp"
#include "utility/gaussian_filtering/rotation.hpp"
#include "utility/gaussian_filtering/system/SystemEstimator.hpp"

/**
 * @class MeasurementFieldLines
 * @brief Dense field-line point measurement (map-matching likelihood).
 *
 * Each field-line detection is a unit ray in the camera frame {c}. The ray is intersected
 * with the ground plane and the point compared against the nearest painted line:
 *
 *   log p = sum_j logsumexp( log(w_in) + logN(d_j; 0, sigma_j^2),
 *                            log(1 - w_in) + log p_clutter )
 *
 * where d_j is the distance to the nearest line and sigma_j grows with the projection
 * range. Rays with an invalid ground intersection at the prior mean are excluded at
 * construction.
 */

namespace module::localisation::measurement {

    using srif::FieldMap;
    using srif::LinePointsSample;
    using srif::SystemLocalisation;
    using utility::gaussian_filtering::Pose;
    using utility::gaussian_filtering::gaussian::GaussianInfo;
    using utility::gaussian_filtering::measurement::Measurement;
    using utility::gaussian_filtering::system::SystemBase;
    using utility::gaussian_filtering::system::SystemEstimator;

    class MeasurementFieldLines : public Measurement {
    public:
        /// @brief Noise and selection options
        struct Options {
            double sigma_distance     = 0.25;  ///< Base line-distance noise std dev [m]
            double sigma_angular      = 0.02;  ///< Ray angular noise mapped to ground range [rad]
            double inlier_probability = 0.55;  ///< Mixture weight of the inlier component
            double clutter_area       = 40.0;  ///< Clutter uniform density area [m^2]
            std::size_t max_points    = 20;    ///< Subsample cap on line points per update
            double max_range          = 2.5;   ///< Maximum ground projection range at the prior [m]
            double min_downward       = 0.05;  ///< Minimum downward ray component at the prior
        };

        /**
         * @brief Construct a field-line measurement.
         * @param time Event time [s]
         * @param sample Field-line points as unit rays in {c}
         * @param Tbc Camera pose w.r.t. torso at capture time
         * @param map Field map providing the analytic line distance
         * @param system System whose prior mean selects the usable rays
         * @param options Noise and selection options
         */
        MeasurementFieldLines(double time,
                              const LinePointsSample& sample,
                              const Pose<double>& Tbc,
                              const FieldMap& map,
                              const SystemLocalisation& system,
                              const Options& options);

        /// @brief Construct with default options
        MeasurementFieldLines(double time,
                              const LinePointsSample& sample,
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

        std::size_t num_points() const {
            return static_cast<std::size_t>(rays_.cols());
        }

    protected:
        virtual void update(SystemBase& system) override;

        const FieldMap& map_;                            ///< Field map (line primitives)
        Pose<double> Tbc_;                               ///< Camera pose w.r.t. torso at capture time
        Eigen::Matrix<double, 3, Eigen::Dynamic> rays_;  ///< Selected unit rays in {c}
        Eigen::VectorXd sigma2_;                         ///< Per-ray distance noise variance [m^2]
        Options options_;
    };

    template <typename Scalar>
    Scalar MeasurementFieldLines::log_likelihood_impl(const Eigen::VectorX<Scalar>& x) const {
        using std::exp, std::log;

        const Eigen::Index n = rays_.cols();
        if (n == 0) {
            return Scalar(0);
        }

        // Camera pose in field frame with mount-bias correction: Tfc = Tfb(x) * Tbc * R(deltaC)
        Pose<Scalar> Tbias(SystemLocalisation::camera_bias_rotation(x), Eigen::Vector3<Scalar>::Zero());
        Pose<Scalar> Tfc                  = SystemLocalisation::field_pose(x) * Pose<Scalar>(Tbc_) * Tbias;
        const Eigen::Matrix3<Scalar> Rfc  = Tfc.rotation_matrix;
        const Eigen::Vector3<Scalar> rCFf = Tfc.translation_vector;

        const double log_inlier_weight = std::log(options_.inlier_probability);
        const double log_clutter       = std::log(1.0 - options_.inlier_probability) - std::log(options_.clutter_area);

        Scalar log_lik = Scalar(0);
        for (Eigen::Index j = 0; j < n; ++j) {
            // Intersect the ray with the field ground plane z = 0
            Eigen::Vector3<Scalar> d = Rfc * rays_.col(j).cast<Scalar>();
            Scalar dz                = d.z();
            if (dz > Scalar(-1e-3)) {
                dz = Scalar(-1e-3);
            }
            Scalar lambda = -rCFf.z() / dz;
            Eigen::Vector2<Scalar> p(rCFf.x() + lambda * d.x(), rCFf.y() + lambda * d.y());

            Scalar d2 = map_.distance_squared_to_nearest_line(p);

            const double s2 = sigma2_(j);
            Scalar a = Scalar(log_inlier_weight - 0.5 * std::log(2.0 * M_PI * s2)) - Scalar(0.5) * d2 / Scalar(s2);
            Scalar b = Scalar(log_clutter);
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

#endif  // MODULE_LOCALISATION_MEASUREMENT_MEASUREMENTFIELDLINES_HPP
