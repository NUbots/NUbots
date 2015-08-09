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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef UTILITY_MATH_FILTER_UKF_H
#define UTILITY_MATH_FILTER_UKF_H

#include <nuclear>
#include <armadillo>

namespace utility {
    namespace math {
        namespace filter {

            template <typename Model> //model is is a template parameter that Kalman also inherits
            class UKF {
            public:
                // The model
                Model model;

            private:
                // The number of sigma points
                static constexpr uint NUM_SIGMA_POINTS = (Model::size * 2) + 1;

                // Dimension types for vectors and square matricies
                using StateVec = arma::vec::fixed<Model::size>;
                using StateMat = arma::mat::fixed<Model::size, Model::size>;

                using SigmaVec = arma::vec::fixed<NUM_SIGMA_POINTS>;
                using SigmaRowVec = arma::rowvec::fixed<NUM_SIGMA_POINTS>;
                using SigmaMat = arma::mat::fixed<Model::size, NUM_SIGMA_POINTS>;
                using SigmaSquareMat = arma::mat::fixed<NUM_SIGMA_POINTS, NUM_SIGMA_POINTS>;

                // Our estimate and covariance
                StateVec mean;
                StateMat covariance;

                // Our sigma points for UKF
                StateVec sigmaMean;
                SigmaMat sigmaPoints;

                SigmaMat centredSigmaPoints; // X in Steves kalman theory
                SigmaVec d;
                SigmaSquareMat covarianceUpdate; // C in Steves kalman theory

                SigmaSquareMat defaultCovarianceUpdate;

                // The mean and covariance weights
                SigmaVec meanWeights;
                SigmaRowVec covarianceWeights;

                // UKF variables
                double covarianceSigmaWeights;

                SigmaMat generateSigmaPoints(const StateVec& mean, const StateMat& covariance) {

                    // Allocate memory for our points
                    SigmaMat points;

                    // Our first row is always the mean
                    points.col(0) = mean;

                    // Get our cholskey decomposition
                    arma::mat chol;
                    try {
                        chol = arma::chol(covarianceSigmaWeights * covariance);
                    } catch (...) {
                        std::cerr << __FILE__ << " " << __LINE__ << " : covarianceSigmaWeights * covariance was NOT positive-definite and the cholskey "
                                  << "decomposition failed.\ncovarianceSigmaWeights * covariance = \n" << std::endl
                                  << covarianceSigmaWeights * covariance << std::endl;
                        throw std::exception();
                    }

                    // Put our values in either end of the matrix
                    for (uint i = 1; i < Model::size + 1; ++i) {

                        auto deviation = chol.col(i - 1);
                        points.col(i)               = (mean + deviation);
                        points.col(i + Model::size) = (mean - deviation);

                    }

                    return points;
                }

                StateVec meanFromSigmas(const SigmaMat& sigmaPoints) const {
                    return sigmaPoints * meanWeights;
                }

                StateMat covarianceFromSigmas(const SigmaMat& sigmaPoints, const StateVec& mean) const {

                    auto meanCentered = sigmaPoints - arma::repmat(mean, 1, NUM_SIGMA_POINTS);
                    return (arma::repmat(covarianceWeights, Model::size, 1) % meanCentered) * meanCentered.t();
                }

                arma::vec meanFromSigmas(const arma::mat& sigmaPoints) const {
                    return sigmaPoints * meanWeights;
                }

                arma::mat covarianceFromSigmas(const arma::mat& sigmaPoints, const arma::vec& mean) const {

                    auto meanCentered = sigmaPoints - arma::repmat(mean, 1, NUM_SIGMA_POINTS);
                    return (arma::repmat(covarianceWeights, mean.size() , 1) % meanCentered) * meanCentered.t();
                }

            public:
                UKF(StateVec initialMean = arma::zeros(Model::size),
                    StateMat initialCovariance = arma::eye(Model::size, Model::size) * 0.1,
                    double alpha = 1e-1,
                    double kappa = 0.f,
                    double beta = 2.f) {

                    reset(initialMean, initialCovariance, alpha, kappa, beta);
                }

                void reset(StateVec initialMean, StateMat initialCovariance,
                           double alpha, double kappa, double beta) {
                    double lambda = pow(alpha, 2) * (Model::size + kappa) - Model::size;

                    covarianceSigmaWeights = Model::size + lambda;

                    meanWeights.fill(1.0 / (2.0 * (Model::size + lambda)));
                    meanWeights[0] = lambda / (Model::size + lambda);

                    covarianceWeights.fill(1.0 / (2.0 * (Model::size + lambda)));
                    covarianceWeights[0] = lambda / (Model::size + lambda) + (1.0 - pow(alpha,2) + beta);

                    defaultCovarianceUpdate = arma::diagmat(covarianceWeights);

                    setState(initialMean, initialCovariance);
                }

                void setState(StateVec initialMean, StateMat initialCovariance) {
                    mean = initialMean;
                    covariance = initialCovariance;

                    // Calculate our sigma points
                    sigmaMean = mean;
                    sigmaPoints = generateSigmaPoints(mean, covariance);

                    // Reset our state for more measurements
                    covarianceUpdate = defaultCovarianceUpdate;
                    d.zeros();
                    centredSigmaPoints = sigmaPoints - arma::repmat(sigmaMean, 1, NUM_SIGMA_POINTS);
                }

                template <typename... TAdditionalParameters>
                void timeUpdate(double deltaT, const TAdditionalParameters&... additionalParameters) {
                    // Generate our sigma points
                    sigmaPoints = generateSigmaPoints(mean, covariance);

                    // Write the propagated version of the sigma point
                    for(uint i = 0; i < NUM_SIGMA_POINTS; ++i) {
                        sigmaPoints.col(i) = model.timeUpdate(sigmaPoints.col(i), deltaT, additionalParameters...);
                    }

                    // Calculate the new mean and covariance values.
                    mean = meanFromSigmas(sigmaPoints);
                    mean = model.limitState(mean);
                    covariance = covarianceFromSigmas(sigmaPoints, mean);
                    covariance += model.processNoise();

                    // Re calculate our sigma points
                    sigmaMean = mean;
                    sigmaPoints = generateSigmaPoints(mean, covariance);

                    // Reset our state for more measurements
                    covarianceUpdate = defaultCovarianceUpdate;
                    d.zeros();
                    centredSigmaPoints = sigmaPoints - arma::repmat(sigmaMean, 1, NUM_SIGMA_POINTS);
                }

                template <typename TMeasurement, typename... TMeasurementArgs>
                double measurementUpdate(const TMeasurement& measurement,
                                         const arma::mat& measurementVariance,
                                         const TMeasurementArgs&... measurementArgs) {

                    // Allocate room for our predictions
                    arma::mat predictedObservations(measurement.n_elem, NUM_SIGMA_POINTS);

                    // First step is to calculate the expected measurement for each sigma point.
                    for(uint i = 0; i < NUM_SIGMA_POINTS; ++i) {
                        predictedObservations.col(i) = model.predictedObservation(sigmaPoints.col(i), measurementArgs...);
                    }

                    // Now calculate the mean of these measurement sigmas.
                    arma::vec predictedMean = meanFromSigmas(predictedObservations);
                    auto centredObservations = predictedObservations - arma::repmat(predictedMean, 1, NUM_SIGMA_POINTS);

                    // Update our state
                    covarianceUpdate -= covarianceUpdate.t() * centredObservations.t() *
                                        (measurementVariance + centredObservations * covarianceUpdate * centredObservations.t()).i() *
                                        centredObservations * covarianceUpdate;

                    const arma::mat innovation = model.observationDifference(measurement, predictedMean);
                    d += (centredObservations.t()) * measurementVariance.i() * innovation;

                    // Update our mean and covariance
                    mean = sigmaMean + centredSigmaPoints * covarianceUpdate * d;
                    mean = model.limitState(mean);
                    covariance = centredSigmaPoints * covarianceUpdate * centredSigmaPoints.t();

                    // Calculate and return the likelihood of the prior mean
                    // and covariance given the new measurement (i.e. the
                    // prior probability density of the measurement):
                    arma::mat predictedCovariance = covarianceFromSigmas(predictedObservations, predictedMean);
                    arma::mat innovationVariance = predictedCovariance + measurementVariance;
                    arma::mat scalarlikelihoodExponent = ((innovation.t() * innovationVariance.i()) * innovation);

                    double expTerm = -0.5 * scalarlikelihoodExponent(0, 0);
                    double normalisationFactor = pow(2 * M_PI, measurementVariance.n_rows) * arma::det(innovationVariance);
                    double fract = 1 / sqrt(normalisationFactor);
                    const float outlierProbability = 0.05;

                    return (1.0 - outlierProbability) * fract * exp(expTerm) + outlierProbability;
                }

                StateVec get() const {
                    return mean;
                }

                StateMat getCovariance() const {
                    return covariance;
                }
            };
        }
    }
}


#endif
