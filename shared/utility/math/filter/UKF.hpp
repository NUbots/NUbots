/*
 * MIT License
 *
 * Copyright (c) 2019 NUbots
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

#ifndef UTILITY_MATH_FILTER_UKF_HPP
#define UTILITY_MATH_FILTER_UKF_HPP

#include <Eigen/Cholesky>
#include <Eigen/Core>

#include "utility/support/LazyEvaluation.hpp"

namespace utility::math::filter {

    template <typename Scalar, template <typename> class FilterModel>
    class UKF {

    public:
        // The model
        using Model = FilterModel<Scalar>;
        Model model;

        // Dimension types for vectors and square matrices
        using StateVec = Eigen::Matrix<Scalar, Model::size, 1>;
        using StateMat = Eigen::Matrix<Scalar, Model::size, Model::size>;

        static constexpr Scalar ALPHA_DEFAULT = Scalar(0.1);
        static constexpr Scalar KAPPA_DEFAULT = Scalar(0.0);
        static constexpr Scalar BETA_DEFAULT  = Scalar(2.0);

    private:
        // The number of sigma points
        static constexpr unsigned int NUM_SIGMA_POINTS = (Model::size * 2) + 1;

        using SigmaVec       = Eigen::Matrix<Scalar, NUM_SIGMA_POINTS, 1>;
        using SigmaMat       = Eigen::Matrix<Scalar, Model::size, NUM_SIGMA_POINTS>;
        using SigmaSquareMat = Eigen::Matrix<Scalar, NUM_SIGMA_POINTS, NUM_SIGMA_POINTS>;

        // Our estimate and covariance
        StateVec mean;
        StateMat covariance;

        // Our sigma points for UKF
        StateVec sigma_mean;
        SigmaMat sigma_points;

        SigmaMat centred_sigma_points;  // X in Steves kalman theory
        SigmaVec d;
        SigmaSquareMat covariance_update;  // C in Steves kalman theory

        // The mean and covariance weights
        SigmaVec mean_weights;
        SigmaVec covariance_weights;


        // UKF variables
        Scalar covariance_sigma_weight;

        /**
         * @brief Generate new sigma points given our mean and covariance.
         */
        template <typename T, int S>
        [[nodiscard]] static Eigen::ComputationInfo generate_sigma_points(const Eigen::Matrix<T, S, 1>& mean,
                                                                          const Eigen::Matrix<T, S, S>& covariance,
                                                                          const T& sigma_weight,
                                                                          SigmaMat& points) {

            // Our first row is always the mean
            points.col(0) = mean;

            // Get our Cholesky decomposition
            // Impose positive semi-definiteness on the covariance matrix
            Eigen::LLT<StateMat> cholesky(sigma_weight
                                          * covariance.unaryExpr([](const Scalar& c) { return std::abs(c); }));

            if (cholesky.info() == Eigen::Success) {
                // Put our values in either end of the matrix
                StateMat chol = cholesky.matrixU().toDenseMatrix();
                for (unsigned int i = 1; i < Model::size + 1; ++i) {
                    points.col(i)               = (mean + chol.col(i - 1));
                    points.col(i + Model::size) = (mean - chol.col(i - 1));
                }
            }

            return cholesky.info();
        }

        /**
         * @brief Calculate the the mean given a set of sigma points.
         *
         * @param mean          the mean to set
         * @param sigma_points  the sigma points to calculate the mean from
         */
        template <typename T1, typename T2, int S>
        // model size, num sigma points
        static Eigen::Matrix<T1, S, 1> mean_from_sigmas(const Eigen::Matrix<T1, S, NUM_SIGMA_POINTS>& sigma_points,
                                                        const Eigen::Matrix<T2, NUM_SIGMA_POINTS, 1>& weights) {
            return sigma_points * weights.template cast<T1>();
        }

        template <typename T1, typename T2, int S>
        static Eigen::Matrix<T1, S, S> covariance_from_sigmas(
            const Eigen::Matrix<T1, S, NUM_SIGMA_POINTS>& sigma_points,
            const Eigen::Matrix<T1, S, 1>& mean,
            const Eigen::Matrix<T2, NUM_SIGMA_POINTS, 1>& weights) {

            Eigen::Matrix<T1, S, NUM_SIGMA_POINTS> mean_centred = sigma_points.colwise() - mean;
            return mean_centred * weights.template cast<T1>().asDiagonal() * mean_centred.transpose();
        }

        template <typename T1, typename T2, int S>
        static Eigen::Matrix<T1, S, S> covariance_from_sigmas(
            const Eigen::Matrix<T1, S, NUM_SIGMA_POINTS>& sigma_points,
            const T1& mean,
            const Eigen::Matrix<T2, NUM_SIGMA_POINTS, 1>& weights) {

            Eigen::Matrix<T1, S, NUM_SIGMA_POINTS> mean_centred = (sigma_points.array() - mean).matrix();
            return mean_centred * weights.template cast<T1>().asDiagonal() * mean_centred.transpose();
        }

    public:
        UKF(StateVec initial_mean       = StateVec::Zero(),
            StateMat initial_covariance = StateMat::Identity() * 0.1,
            Scalar alpha                = ALPHA_DEFAULT,
            Scalar kappa                = KAPPA_DEFAULT,
            Scalar beta                 = BETA_DEFAULT)
            : model()
            , mean(initial_mean)
            , covariance(initial_covariance)
            , sigma_mean(StateVec::Zero())
            , sigma_points(SigmaMat::Zero())
            , centred_sigma_points(SigmaMat::Zero())
            , d(SigmaVec::Zero())
            , covariance_update(SigmaSquareMat::Identity())
            , mean_weights(SigmaVec::Zero())
            , covariance_weights(SigmaVec::Zero())
            , covariance_sigma_weight(0.0) {

            reset(initial_mean, initial_covariance, alpha, kappa, beta);
        }

        Eigen::ComputationInfo reset(StateVec initial_mean,
                                     StateMat initial_covariance,
                                     Scalar alpha = ALPHA_DEFAULT,
                                     Scalar kappa = KAPPA_DEFAULT,
                                     Scalar beta  = BETA_DEFAULT) {

            Scalar lambda = alpha * alpha * (Model::size + kappa) - Model::size;

            covariance_sigma_weight = Model::size + lambda;

            mean_weights.fill(1.0 / (2.0 * covariance_sigma_weight));
            mean_weights[0] = lambda / covariance_sigma_weight;

            covariance_weights.fill(1.0 / (2.0 * covariance_sigma_weight));
            covariance_weights[0] = lambda / covariance_sigma_weight + (1.0 - (alpha * alpha) + beta);

            return set_state(initial_mean, initial_covariance);
        }

        Eigen::ComputationInfo set_state(StateVec initial_mean, StateMat initial_covariance) {
            mean       = initial_mean;
            covariance = initial_covariance;

            // Calculate our sigma points
            sigma_mean = mean;
            Eigen::ComputationInfo cholesky_info =
                generate_sigma_points(mean, covariance, covariance_sigma_weight, sigma_points);

            if (cholesky_info == Eigen::Success) {
                // Reset our state for more measurements
                covariance_update = covariance_weights.asDiagonal();
                d.setZero();
                centred_sigma_points = sigma_points.colwise() - sigma_mean;
            }

            return cholesky_info;
        }

        template <typename... Args>
        Eigen::ComputationInfo time(const Scalar& dt, const Args&... params) {
            // Generate our sigma points
            Eigen::ComputationInfo cholesky_info =
                generate_sigma_points(mean, covariance, covariance_sigma_weight, sigma_points);

            if (cholesky_info == Eigen::Success) {
                // Write the propagated version of the sigma point
                for (unsigned int i = 0; i < NUM_SIGMA_POINTS; ++i) {
                    sigma_points.col(i) = model.time(sigma_points.col(i), dt, params...);
                }

                // Calculate the new mean and covariance values.
                mean       = mean_from_sigmas(sigma_points, mean_weights);
                mean       = model.limit(mean);
                covariance = covariance_from_sigmas(sigma_points, mean, covariance_weights);
                covariance += model.noise(dt);

                // Re calculate our sigma points
                sigma_mean    = mean;
                cholesky_info = generate_sigma_points(mean, covariance, covariance_sigma_weight, sigma_points);

                if (cholesky_info == Eigen::Success) {
                    // Reset our state for more measurements
                    covariance_update = covariance_weights.asDiagonal();
                    d.setZero();
                    centred_sigma_points = sigma_points.colwise() - sigma_mean;
                }
            }
            return cholesky_info;
        }


        /**
         * Perform a measurement update using the given measurement and covariance
         */
        template <typename MeasurementScalar, int S, int... VArgs, int... MArgs, typename... Args>
        utility::support::LazyEvaluation<MeasurementScalar> measure(
            const Eigen::Matrix<MeasurementScalar, S, 1, VArgs...>& measurement,
            const Eigen::Matrix<MeasurementScalar, S, S, MArgs...>& measurement_variance,
            const Args&... params) {

            // Allocate room for our predictions
            Eigen::Matrix<MeasurementScalar, S, NUM_SIGMA_POINTS> predictions;

            // First step is to calculate the predicted measurement for each sigma point.
            for (unsigned int i = 0; i < NUM_SIGMA_POINTS; ++i) {
                predictions.col(i) = model.predict(sigma_points.col(i), params...);
            }

            // Now calculate the mean of these measurement sigmas.
            Eigen::Matrix<MeasurementScalar, S, 1> predicted_mean         = mean_from_sigmas(predictions, mean_weights);
            Eigen::Matrix<MeasurementScalar, S, NUM_SIGMA_POINTS> centred = predictions.colwise() - predicted_mean;

            // Create a state update in our measurement units
            Eigen::Matrix<MeasurementScalar, NUM_SIGMA_POINTS, NUM_SIGMA_POINTS> update =
                covariance_update.template cast<MeasurementScalar>();
            update = update.transpose() * centred.transpose()
                     * (measurement_variance + centred * update * centred.transpose())
                           .llt()
                           .solve(Eigen::Matrix<MeasurementScalar, S, S>::Identity())
                     * centred * update;
            covariance_update -= update.template cast<Scalar>();

            Eigen::Matrix<MeasurementScalar, S, 1> innovation = model.difference(measurement, predicted_mean);
            d += (centred.transpose()
                  * measurement_variance.llt().solve(Eigen::Matrix<MeasurementScalar, S, S>::Identity()) * innovation)
                     .template cast<Scalar>();

            // Update our mean and covariance
            mean       = sigma_mean + centred_sigma_points * covariance_update * d;
            mean       = model.limit(mean);
            covariance = centred_sigma_points * covariance_update * centred_sigma_points.transpose();

            // Calculate and return the likelihood of the prior mean and covariance given the new measurement
            // (i.e. the prior probability density of the measurement):
            return utility::support::LazyEvaluation<MeasurementScalar>(
                [predictions, predicted_mean, measurement_variance, innovation, cw = this->covariance_weights] {
                    Eigen::Matrix<MeasurementScalar, S, S> predicted_covariance =
                        covariance_from_sigmas(predictions, predicted_mean, cw);

                    Eigen::Matrix<MeasurementScalar, S, S> innovation_variance =
                        predicted_covariance + measurement_variance;

                    MeasurementScalar likelihood_exponent =
                        (innovation.transpose()
                         * innovation_variance.llt().solve(Eigen::Matrix<MeasurementScalar, S, S>::Identity())
                         * innovation)
                            .x();

                    MeasurementScalar loglikelihood =
                        0.5
                        * (std::log(innovation_variance.determinant()) + std::abs(likelihood_exponent)
                           + innovation.size() * std::log(2 * M_PI));

                    return -loglikelihood;
                });
        }

        [[nodiscard]] const StateVec& get_state() const {
            return mean;
        }

        [[nodiscard]] const StateMat& get_covariance() const {
            return covariance;
        }
    };
}  // namespace utility::math::filter


#endif
