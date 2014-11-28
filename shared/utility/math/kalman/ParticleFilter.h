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

#ifndef UTILITY_MATH_KALMAN_PARTICLEFILTER_H
#define UTILITY_MATH_KALMAN_PARTICLEFILTER_H

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
                using StateMat = arma::mat::fixed<Model::size, Model::size>;

                arma::vec<StateVec> particles;

                double sigma_sq;

                StateVec sampleParticle(const& StateVec mu, const& StateMat sigma){
                    //TODO
                    return mu;
                }

            public:
                ParticleFilter(StateVec initialMean = arma::zeros(Model::size),
                               StateMat initialCovariance = arma::eye(Model::size, Model::size) * 0.1,
                               int number_of_particles_,
                               double sigma_sq_ = 10) 
                {
                    sigma_sq = sigma_sq_;
                    reset(initialMean, initialCovariance, number_of_particles_);
                }

                void reset(StateVec initialMean, StateMat initialCovariance, int number_of_particles_) {
                    particles.clear()
                    for(int i = 0; i < number_of_particles_; i++){
                        particles.push_back(sampleParticle(initialMean, initialCovariance));
                    }
                }

                template <typename... TAdditionalParameters>
                void timeUpdate(double deltaT, const TAdditionalParameters&... additionalParameters) {
                    for(uint i = 0; i < particles.size(); ++i) {
                        particles[i] = model.timeUpdate(particles[i], deltaT, additionalParameters...);
                    }
                }

                template <typename TMeasurement, typename... TMeasurementType>
                double measurementUpdate(const TMeasurement& measurement,
                                         const arma::mat& measurement_variance,
                                         const TMeasurementType&... measurementArgs) {
                    //TODO
                    arma::vec weights = arma::zeros(particles.size());

                    for (int i = 0; i < particles.size(); i++){
                        arma::vec predictedObservation = model.predictedObservation(particles[i], measurementArgs...));
                        assert(predictedObservation.size() == measurement.size());
                        double difference = arma::norm(predictedObservation-measurement)
                        weights[i] = std::exp(- difference * difference / sigma_sq);
                    }
                    
                    std::discrete_distribution multinomial(weights.begin(),weights.end());//class incorrectly named by cpp devs
                    std::vec resample = zeros(particles.size());
                    //TODO: RESAMPLE
                    arma::vec<StateVec> candidateParticles = particles;
                    for (int i = 0; i < particles.size(); i++){
                        particles[i] = candidateParticles[resample[i]];
                    }

                }

                StateVec get() const {
                    return StateVec();
                    // return arma::mean(particles);
                }

                StateMat getCovariance() const {
                    return StateMat();
                    // return arma::covariance(particles);
                }
            };
        }
    }
}


#endif
