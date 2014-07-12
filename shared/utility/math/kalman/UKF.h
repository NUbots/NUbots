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

#ifndef UTILITY_MATH_KALMAN_UKF_H
#define UTILITY_MATH_KALMAN_UKF_H

#include <nuclear>
#include <armadillo>

namespace utility {
    namespace math {
        namespace kalman {

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
                    } catch (const std::runtime_error& e) {
                        if (Model::size == 3) std::cerr << __FILE__ << " " << __LINE__ << " : covarianceSigmaWeights * covariance was NOT positive-definite and the cholskey "
                                  << "decomposition failed.\ncovarianceSigmaWeights * covariance = \n" << std::endl
                                  << covarianceSigmaWeights * covariance << std::endl;
                        throw e;
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

                void reset(StateVec initialMean, StateMat initialCovariance, double alpha, double kappa, double beta) {

                    double lambda = pow(alpha, 2) * (Model::size + kappa) - Model::size;

                    covarianceSigmaWeights = Model::size + lambda;

                    mean = initialMean;
                    covariance = initialCovariance;

                    meanWeights.fill(1.0 / (2.0 * (Model::size + lambda)));
                    meanWeights[0] = lambda / (Model::size + lambda);

                    covarianceWeights.fill(1.0 / (2.0 * (Model::size + lambda)));
                    covarianceWeights[0] = lambda / (Model::size + lambda) + (1.0 - pow(alpha,2) + beta);

                    defaultCovarianceUpdate = arma::diagmat(covarianceWeights);

                    // Calculate our sigma points
                    sigmaMean = mean;
                    sigmaPoints = generateSigmaPoints(mean, covariance);

                    // Reset our state for more measurements
                    covarianceUpdate = defaultCovarianceUpdate;
                    d.zeros();
                    centredSigmaPoints = sigmaPoints - arma::repmat(sigmaMean, 1, NUM_SIGMA_POINTS);
                }

                void timeUpdate(double deltaT) {
                    // Generate our sigma points
                    sigmaPoints = generateSigmaPoints(mean, covariance);


                    // Write the propagated version of the sigma point
                    for(uint i = 0; i < NUM_SIGMA_POINTS; ++i) {
                        sigmaPoints.col(i) = model.timeUpdate(sigmaPoints.col(i), deltaT);
                    }


                    // Calculate the new mean and covariance values.
                    mean = meanFromSigmas(sigmaPoints);                    mean = model.limitState(mean);
                    covariance = covarianceFromSigmas(sigmaPoints, mean) + model.processNoise();

                    // Re calculate our sigma points
                    sigmaMean = mean;
                    sigmaPoints = generateSigmaPoints(mean, covariance);

                    // Reset our state for more measurements
                    covarianceUpdate = defaultCovarianceUpdate;
                    d.zeros();
                    centredSigmaPoints = sigmaPoints - arma::repmat(sigmaMean, 1, NUM_SIGMA_POINTS);
                }

                template <typename TMeasurement, typename... TMeasurementType>
                double measurementUpdate(const TMeasurement& measurement,
                                         const arma::mat& measurement_variance,
                                         const TMeasurementType&... measurementArgs) {

                    // Allocate room for our predictions
                    arma::mat predictedObservations(measurement.n_elem, NUM_SIGMA_POINTS);

                    // First step is to calculate the expected measurement for each sigma point.
                    for(uint i = 0; i < NUM_SIGMA_POINTS; ++i) {
                        predictedObservations.col(i) = model.predictedObservation(sigmaPoints.col(i), measurementArgs...);
                    }

                    // Now calculate the mean of these measurement sigmas.
                    arma::vec predictedMean = meanFromSigmas(predictedObservations);
                    predictedObservations.each_col() -= predictedMean;


                    arma::mat predictedCovariance = covarianceFromSigmas(predictedObservations, predictedMean);

                    const arma::mat innovation = model.observationDifference(measurement, predictedMean);

                    // Update our state
                    covarianceUpdate -= covarianceUpdate.t() * predictedObservations.t() *
                                        (measurement_variance + predictedObservations * covarianceUpdate * predictedObservations.t()).i() *
                                        predictedObservations * covarianceUpdate;

                    d += (predictedObservations.t()) * measurement_variance.i() * innovation;

                    // Update our mean and covariance
                    mean = sigmaMean + centredSigmaPoints * covarianceUpdate * d;
                    mean = model.limitState(mean);
                    covariance = centredSigmaPoints * covarianceUpdate * centredSigmaPoints.t();

                    // Magical quality calculation
                    arma::mat innovationVariance = predictedCovariance + measurement_variance;
                    arma::mat innovationCovariance = ((innovation.t() * innovationVariance.i()) * innovation);

                    double expTerm = -0.5 * innovationCovariance(0, 0);
                    double fract = 1 / sqrt(pow(2 * M_PI, measurement_variance.n_rows) * arma::det(innovationVariance));
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
