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

#include <nuclear>
#include <armadillo>
#include <random>
#include <vector>

namespace utility {
    namespace math {
        namespace filter {

            template <typename Model> //model is is a template parameter that Kalman also inherits
            class ParticleFilter {
            public:
                // The model
                Model model;

            private:
                // Dimension types for vectors and square matricies
                using StateVec = arma::vec::fixed<Model::size>;
                using ParticleList = arma::mat;
                using StateMat = arma::mat::fixed<Model::size, Model::size>;

                /* particles.n_rows = number of particles
                   particle.row(i) = particle i
                */
                ParticleList particles;

                StateVec sigma_sq;

            public:
                ParticleFilter(StateVec initialMean = arma::zeros(Model::size),
                               StateMat initialCovariance = arma::eye(Model::size, Model::size) * 0.1,
                               int number_of_particles_ = 100,
                               StateVec sigma_sq_ = 0.1 * arma::ones(Model::size))
                {
                    sigma_sq = arma::abs(sigma_sq_);
                    reset(initialMean, initialCovariance, number_of_particles_);
                }

                void reset(StateVec initialMean, StateMat initialCovariance, int number_of_particles_)
                {
                    particles = arma::zeros(number_of_particles_,Model::size);
                    setState(initialMean, initialCovariance);
                }

                void setState(StateVec initialMean, StateMat initialCovariance) {
                    //Sample single gaussian (represented by a gaussian mixture model of size 1)
                    arma::gmm_diag gaussian;
                    gaussian.set_params(arma::mat(initialMean), arma::mat(initialCovariance.diag()),arma::ones(1));
                    for(unsigned int i = 0; i < particles.n_rows; ++i) {
                        particles.row(i) = gaussian.generate().t();
                    }
                }

                template <typename... TAdditionalParameters>
                void timeUpdate(double deltaT, const TAdditionalParameters&... additionalParameters)
                {
                    //Sample single zero mean gaussian with process noise (represented by a gaussian mixture model of size 1)
                    arma::gmm_diag gaussian;
                    gaussian.set_params(arma::mat(arma::zeros(Model::size)), arma::mat(model.processNoise().diag()),arma::ones(1));
                    for(unsigned int i = 0; i < particles.n_rows; ++i) {
                        //TODO: add noise?
                        StateVec newpcle = model.timeUpdate(particles.row(i).t(), deltaT, additionalParameters...) + gaussian.generate();
                        particles.row(i) = newpcle.t();
                    }
                }

                template <typename TMeasurement, typename... TMeasurementType>
                double measurementUpdate(const TMeasurement& measurement,
                                         const arma::mat& measurement_variance,
                                         const TMeasurementType&... measurementArgs)
                {
                    arma::vec weights = arma::zeros(particles.n_rows);

                    for (unsigned int i = 0; i < particles.n_rows; i++){
                        arma::vec predictedObservation = model.predictedObservation(particles.row(i).t(), measurementArgs...);
                        assert(predictedObservation.size() == measurement.size());
                        arma::vec difference = predictedObservation-measurement;
                        weights[i] = std::exp(- arma::dot(difference, (measurement_variance.i() * difference)));
                    }
                    // std::cout << "weights = \n" << weights << std::endl;
                    //Resample
                    std::random_device rd;
                    std::mt19937 gen(rd());
                    std::discrete_distribution<> multinomial(weights.begin(),weights.end());//class incorrectly named by cpp devs
                    ParticleList candidateParticles = particles;
                    for (unsigned int i = 0; i < particles.n_rows; i++){
                        particles.row(i) = candidateParticles.row(multinomial(gen));
                    }
                    return arma::mean(weights);
                }

                StateVec get() const
                {
                    return arma::mean(particles, 0).t();
                }

                StateMat getCovariance() const
                {
                    return arma::cov(particles, 0);
                }
            };
        }
    }
}


#endif
