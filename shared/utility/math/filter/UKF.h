/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef UTILITY_MATH_FILTER_UKF_H
#define UTILITY_MATH_FILTER_UKF_H

#include <armadillo>
#include <nuclear>

#include "utility/support/LazyEvaluation.h"

namespace utility {
namespace math {
    namespace filter {

        template <typename Model>  // model is is a template parameter that Kalman also inherits
        class UKF {
        public:
            // The model
            Model model;

            // Dimension types for vectors and square matricies
            using StateVec = arma::vec::fixed<Model::size>;
            using StateMat = arma::mat::fixed<Model::size, Model::size>;

        private:
            // The number of sigma points
            static constexpr uint NUM_SIGMA_POINTS = (Model::size * 2) + 1;

            using SigmaVec       = arma::vec::fixed<NUM_SIGMA_POINTS>;
            using SigmaRowVec    = arma::rowvec::fixed<NUM_SIGMA_POINTS>;
            using SigmaMat       = arma::mat::fixed<Model::size, NUM_SIGMA_POINTS>;
            using SigmaSquareMat = arma::mat::fixed<NUM_SIGMA_POINTS, NUM_SIGMA_POINTS>;

            // Our estimate and covariance
            StateVec mean;
            StateMat covariance;

            // Our sigma points for UKF
            StateVec sigmaMean;
            SigmaMat sigmaPoints;

            SigmaMat centredSigmaPoints;  // X in Steves kalman theory
            SigmaVec d;
            SigmaSquareMat covarianceUpdate;  // C in Steves kalman theory

            SigmaSquareMat defaultCovarianceUpdate;

            // The mean and covariance weights
            SigmaVec meanWeights;
            SigmaRowVec covarianceWeights;

        private:
            // UKF variables
            double covarianceSigmaWeights;

            void generateSigmaPoints(SigmaMat& points, const StateVec& mean, const StateMat& covariance) {

                // Our first row is always the mean
                points.col(0) = mean;

                // Get our cholskey decomposition
                arma::mat chol = arma::chol(covarianceSigmaWeights * covariance);

                // Put our values in either end of the matrix
                for (uint i = 1; i < Model::size + 1; ++i) {

                    auto deviation              = chol.col(i - 1);
                    points.col(i)               = (mean + deviation);
                    points.col(i + Model::size) = (mean - deviation);
                }
            }

            void meanFromSigmas(StateVec& mean, const SigmaMat& sigmaPoints) const {
                mean = sigmaPoints * meanWeights;
            }

            void covarianceFromSigmas(StateMat& covariance, const SigmaMat& sigmaPoints, const StateVec& mean) const {

                SigmaMat meanCentered = sigmaPoints - arma::repmat(mean, 1, NUM_SIGMA_POINTS);
                covariance = (arma::repmat(covarianceWeights, Model::size, 1) % meanCentered) * meanCentered.t();
            }

            void meanFromSigmas(arma::vec& mean, const arma::mat& sigmaPoints) const {
                mean = sigmaPoints * meanWeights;
            }

            void covarianceFromSigmas(arma::mat& covariance,
                                      const arma::mat& sigmaPoints,
                                      const arma::vec& mean) const {

                arma::mat meanCentered = sigmaPoints - arma::repmat(mean, 1, NUM_SIGMA_POINTS);
                covariance = (arma::repmat(covarianceWeights, mean.size(), 1) % meanCentered) * meanCentered.t();
            }

        public:
            UKF(StateVec initialMean       = arma::zeros(Model::size),
                StateMat initialCovariance = arma::eye(Model::size, Model::size) * 0.1,
                double alpha               = 1e-1,
                double kappa               = 0.f,
                double beta                = 2.f)
                : model()
                , mean(arma::fill::zeros)
                , covariance(arma::fill::eye)
                , sigmaMean(arma::fill::zeros)
                , sigmaPoints(arma::fill::zeros)
                , centredSigmaPoints(arma::fill::zeros)
                , d(arma::fill::zeros)
                , covarianceUpdate(arma::fill::eye)
                , defaultCovarianceUpdate(arma::fill::eye)
                , meanWeights(arma::fill::zeros)
                , covarianceWeights(arma::fill::zeros)
                , covarianceSigmaWeights(0.0) {

                reset(initialMean, initialCovariance, alpha, kappa, beta);
            }

            void reset(StateVec initialMean, StateMat initialCovariance, double alpha, double kappa, double beta) {
                double lambda = pow(alpha, 2) * (Model::size + kappa) - Model::size;

                covarianceSigmaWeights = Model::size + lambda;

                meanWeights.fill(1.0 / (2.0 * (Model::size + lambda)));
                meanWeights[0] = lambda / (Model::size + lambda);

                covarianceWeights.fill(1.0 / (2.0 * (Model::size + lambda)));
                covarianceWeights[0] = lambda / (Model::size + lambda) + (1.0 - pow(alpha, 2) + beta);

                defaultCovarianceUpdate = arma::diagmat(covarianceWeights);

                setState(initialMean, initialCovariance);
            }

            void setState(StateVec initialMean, StateMat initialCovariance) {
                mean       = initialMean;
                covariance = initialCovariance;

                // Calculate our sigma points
                sigmaMean = mean;
                generateSigmaPoints(sigmaPoints, mean, covariance);

                // Reset our state for more measurements
                covarianceUpdate = defaultCovarianceUpdate;
                d.zeros();
                centredSigmaPoints = sigmaPoints - arma::repmat(sigmaMean, 1, NUM_SIGMA_POINTS);
            }

            template <typename... TAdditionalParameters>
            void timeUpdate(double deltaT, const TAdditionalParameters&... additionalParameters) {
                // Generate our sigma points
                generateSigmaPoints(sigmaPoints, mean, covariance);

                // Write the propagated version of the sigma point
                for (uint i = 0; i < NUM_SIGMA_POINTS; ++i) {
                    sigmaPoints.col(i) = model.timeUpdate(sigmaPoints.col(i), deltaT, additionalParameters...);
                }

                // Calculate the new mean and covariance values.
                meanFromSigmas(mean, sigmaPoints);
                mean = model.limitState(mean);
                covarianceFromSigmas(covariance, sigmaPoints, mean);
                covariance += model.processNoise();

                // Re calculate our sigma points
                sigmaMean = mean;
                generateSigmaPoints(sigmaPoints, mean, covariance);

                // Reset our state for more measurements
                covarianceUpdate = defaultCovarianceUpdate;
                d.zeros();
                centredSigmaPoints = sigmaPoints - arma::repmat(sigmaMean, 1, NUM_SIGMA_POINTS);
            }

            template <typename TMeasurement, typename... TMeasurementArgs>
            utility::support::LazyEvaluation<double> measurementUpdate(const TMeasurement& measurement,
                                                                       const arma::mat& measurementVariance,
                                                                       const TMeasurementArgs&... measurementArgs) {

                // Allocate room for our predictions
                arma::mat predictedObservations(measurement.n_elem, NUM_SIGMA_POINTS);

                // First step is to calculate the expected measurement for each sigma point.
                for (uint i = 0; i < NUM_SIGMA_POINTS; ++i) {
                    predictedObservations.col(i) = model.predictedObservation(sigmaPoints.col(i), measurementArgs...);
                }

                // Now calculate the mean of these measurement sigmas.
                arma::vec predictedMean;
                meanFromSigmas(predictedMean, predictedObservations);
                auto centredObservations = predictedObservations - arma::repmat(predictedMean, 1, NUM_SIGMA_POINTS);

                // Update our state
                covarianceUpdate -=
                    covarianceUpdate.t() * centredObservations.t()
                    * arma::inv_sympd(measurementVariance
                                      + centredObservations * covarianceUpdate * centredObservations.t())
                    * centredObservations * covarianceUpdate;

                const arma::mat innovation = model.observationDifference(measurement, predictedMean);
                d += (centredObservations.t()) * measurementVariance.i() * innovation;

                // Update our mean and covariance
                mean       = sigmaMean + centredSigmaPoints * covarianceUpdate * d;
                mean       = model.limitState(mean);
                covariance = centredSigmaPoints * covarianceUpdate * centredSigmaPoints.t();

                // Calculate and return the likelihood of the prior mean
                // and covariance given the new measurement (i.e. the
                // prior probability density of the measurement):
                return utility::support::LazyEvaluation<double>(
                    [this, predictedObservations, predictedMean, measurementVariance, innovation] {
                        arma::mat predictedCovariance;
                        covarianceFromSigmas(predictedCovariance, predictedObservations, predictedMean);
                        arma::mat innovationVariance       = predictedCovariance + measurementVariance;
                        arma::mat scalarlikelihoodExponent = ((innovation.t() * innovationVariance.i()) * innovation);
                        double loglikelihood =
                            0.5
                            * (std::log(arma::det(innovationVariance)) + std::abs(scalarlikelihoodExponent[0])
                               + innovation.n_elem * std::log(2 * M_PI));
                        return -loglikelihood;
                    });
            }

            const StateVec& get() const {
                return mean;
            }

            const StateMat& getCovariance() const {
                return covariance;
            }
        };
    }  // namespace filter
}  // namespace math
}  // namespace utility


#endif
