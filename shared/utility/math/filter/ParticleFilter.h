/*
 * Particle filter substitute for UKF
 *
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

#ifndef UTILITY_MATH_FILTER_PARTICLEFILTER_H
#define UTILITY_MATH_FILTER_PARTICLEFILTER_H

#include <armadillo>
#include <iostream>
#include <nuclear>
#include <random>
#include <vector>

namespace utility {
namespace math {
    namespace filter {

        template <typename Model>  // model is is a template parameter that Kalman also inherits
        class ParticleFilter {
        public:
            // The model
            Model model;

            using StateVec = arma::vec::fixed<Model::size>;
            using StateMat = arma::mat::fixed<Model::size, Model::size>;

        private:
            // Dimension types for vectors and square matricies
            using ParticleList = arma::mat;

            /* particles.n_cols = number of particles
               particle.col(i) = particle i
            */
            ParticleList particles;

            StateVec sigma_sq;

        public:
            ParticleFilter(StateVec initialMean       = arma::zeros(Model::size),
                           StateMat initialCovariance = arma::eye(Model::size, Model::size) * 0.1,
                           int number_of_particles_   = 100,
                           StateVec sigma_sq_         = 0.1 * arma::ones(Model::size)) {
                sigma_sq = arma::abs(sigma_sq_);
                reset(initialMean, initialCovariance, number_of_particles_);
            }

            void reset(StateVec initialMean, StateMat initialCovariance, int number_of_particles_ = 100) {
                particles = getParticles(initialMean, initialCovariance, number_of_particles_);
            }

            void resetAmbiguous(const std::vector<StateVec>& initialMeans,
                                const std::vector<StateMat>& initialCovariances,
                                const int& number_of_particles_ = 100) {
                if (initialMeans.size() != initialCovariances.size()) {
                    throw std::runtime_error(std::string(__FILE__) + " : " + std::to_string(__LINE__)
                                             + " different number of means vs covariances provided.");
                }
                particles = arma::zeros(Model::size, number_of_particles_);
                // Sample the same number of particles for each possibility
                const int particlesPerInit = number_of_particles_ / initialMeans.size();
                const int remainder        = number_of_particles_ % initialMeans.size();
                // Generate remainder using first hypotheses
                // Cols are accessed cols(first,last_inclusive)
                if (remainder > 0) {
                    particles.cols(0, remainder - 1) = getParticles(initialMeans[0], initialCovariances[0], remainder);
                }
                // Generate the rest equally
                for (unsigned int i = 0, currentStart = remainder; currentStart < particles.n_cols;
                     ++i, currentStart += particlesPerInit) {
                    particles.cols(currentStart, currentStart + particlesPerInit - 1) =
                        getParticles(initialMeans[i], initialCovariances[i], particlesPerInit);
                }
            }

            ParticleList getParticles(const StateVec& initialMean,
                                      const StateMat& initialCovariance,
                                      const int& n_particles) const {
                // Sample single gaussian (represented by a gaussian mixture model of size 1)
                ParticleList new_particles = arma::zeros(n_particles, Model::size);

                arma::gmm_diag gaussian;
                gaussian.set_params(arma::mat(initialMean), arma::mat(initialCovariance.diag()), arma::ones(1));

                return gaussian.generate(n_particles);
            }


            template <typename... TAdditionalParameters>
            void timeUpdate(const double& deltaT, const TAdditionalParameters&... additionalParameters) {
                // Sample single zero mean gaussian with process noise (represented by a gaussian mixture model of size
                // 1)
                arma::gmm_diag gaussian;
                gaussian.set_params(arma::mat(arma::zeros(Model::size)),
                                    arma::mat(model.processNoise().diag() * deltaT),
                                    arma::ones(1));

                for (unsigned int i = 0; i < particles.n_cols; ++i) {
                    particles.col(i) = model.timeUpdate(particles.col(i), deltaT, additionalParameters...);
                }
                particles += gaussian.generate(particles.n_cols);
            }

            template <typename TMeasurement, typename... TMeasurementType>
            double measurementUpdate(const TMeasurement& measurement,
                                     const arma::mat& measurement_variance,
                                     const TMeasurementType&... measurementArgs) {
                ParticleList candidateParticles =
                    arma::join_rows(particles, arma::zeros(Model::size, model.getRogueCount()));
                // Resample some rogues
                for (int i = 0; i < model.getRogueCount(); i++) {
                    candidateParticles.col(i + particles.n_cols) =
                        particles.col(i) + (model.getRogueRange() % (0.5 - arma::randu(Model::size)));
                }

                arma::mat observationDifferences = arma::mat(measurement.n_elem, candidateParticles.n_cols);
                for (unsigned int i = 0; i < candidateParticles.n_cols; ++i) {
                    arma::vec predictedObservation =
                        model.predictedObservation(candidateParticles.col(i), measurementArgs...);
                    observationDifferences.col(i) = model.observationDifference(predictedObservation, measurement);
                }

                // Calculate log probabilities
                arma::vec logits =
                    (-arma::sum(observationDifferences % (measurement_variance.i() * observationDifferences), 0)).t();

                // Subtract the max log prob for numerical stability and then exponentiate
                logits = arma::exp(logits - logits.max());

                // Resample
                std::random_device rd;
                std::mt19937 gen(rd());
                std::discrete_distribution<> multinomial(logits.begin(),
                                                         logits.end());  // class incorrectly named by cpp devs
                for (unsigned int i = 0; i < particles.n_cols; i++) {
                    particles.col(i) = model.limitState(candidateParticles.col(multinomial(gen)));
                }
                return arma::mean(logits);
            }


            template <typename TMeasurement, typename... TMeasurementType>
            double ambiguousMeasurementUpdate(const TMeasurement& measurement,
                                              const arma::mat& measurement_variance,
                                              const std::vector<arma::vec>& possibilities,
                                              const TMeasurementType&... measurementArgs) {
                // Expand candidate particles with
                ParticleList candidateParticles =
                    arma::join_rows(particles, arma::zeros(Model::size, model.getRogueCount()));
                // Resample rogues
                for (int i = 0; i < model.getRogueCount(); i++) {
                    candidateParticles.col(i + particles.n_cols) =
                        particles.col(i) + (model.getRogueRange() % (0.5 - arma::randu(Model::size)));
                }
                // Repeat each particle for each possibility
                ParticleList repCandidateParticles = arma::repmat(candidateParticles, 1, possibilities.size());

                // Compute weights
                arma::mat observationDifferences = arma::mat(measurement.n_elem, repCandidateParticles.n_cols);
                for (unsigned int i = 0; i < repCandidateParticles.n_cols; ++i) {
                    arma::vec predictedObservation = model.predictedObservation(
                        repCandidateParticles.col(i), possibilities[i / candidateParticles.n_cols], measurementArgs...);
                    observationDifferences.col(i) = model.observationDifference(predictedObservation, measurement);
                }
                arma::vec weights =
                    arma::exp(
                        -arma::sum(observationDifferences % (measurement_variance.i() * observationDifferences), 0))
                        .t();

                // Resample
                std::random_device rd;
                std::mt19937 gen(rd());
                std::discrete_distribution<> multinomial(weights.begin(),
                                                         weights.end());  // class incorrectly named by cpp devs
                // Only sample N particles
                arma::vec new_weights = arma::zeros(particles.n_cols);
                for (unsigned int i = 0; i < particles.n_cols; i++) {
                    int index        = multinomial(gen);
                    particles.col(i) = repCandidateParticles.col(index);
                    new_weights(i)   = weights(index);
                }
                // Sort particles by descending weight

                arma::uvec sorted_index  = sort_index(new_weights, "decend");
                arma::mat particles_temp = particles;
                for (unsigned int i = 0; i < sorted_index.n_rows; i++) {
                    particles.col(i) = model.limitState(particles_temp.col(sorted_index[i]));
                }

                // Return mean weight
                return new_weights[sorted_index[0]];
            }

            StateVec get() const {
                return arma::mean(particles, 1);
            }

            StateVec getBest() const {
                return particles.col(0);
            }

            StateMat getCovariance() const {
                return arma::cov(particles.t());
            }

            ParticleList getParticles() const {
                return particles;
            }
        };
    }  // namespace filter
}  // namespace math
}  // namespace utility


#endif
