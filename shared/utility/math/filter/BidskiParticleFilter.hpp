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
 * Copyright 2019 NUbots <nubots@nubots.net>
 */

#ifndef UTILITY_MATH_FILTER_BIDSKI_PARTICLEFILTER_HPP
#define UTILITY_MATH_FILTER_BIDSKI_PARTICLEFILTER_HPP

#include <Eigen/Dense>
#include <random>
#include <vector>

#include "utility/math/stats/multivariate.hpp"

namespace utility::math::filter {

    using utility::math::stats::MultivariateNormal;

    template <typename Scalar, template <typename> class FilterModel>
    class BidskiParticleFilter {
    public:
        using Model = FilterModel<Scalar>;
        // The model
        Model model;

        using StateVec = Eigen::Matrix<Scalar, Model::size, 1>;
        using StateMat = Eigen::Matrix<Scalar, Model::size, Model::size>;

        BidskiParticleFilter() {
            set_state(StateVec::Zero(), StateMat::Identity() * 0.1);
        }
        BidskiParticleFilter(const StateVec& mean, const StateMat& covariance) {
            set_state(mean, covariance);
        }

        void set_state(const StateVec& mean, const StateMat& covariance) {
            // Initialise all of the particles
            init(mean, covariance);

            // Limit the state of each particle to ensure they are still valid
            for (unsigned int i = 0; i < particles.cols(); i++) {
                particles.col(i) = model.limit(particles.col(i));
            }
        }

    private:
        using ParticleList    = Eigen::Matrix<Scalar, Model::size, Eigen::Dynamic>;
        using ParticleWeights = std::vector<Scalar>;

        ParticleList particles;
        ParticleWeights weights;

        void init(const StateVec& mean, const StateMat& covariance) {
            // Make sure our particle list has the right shape
            if (particles.cols() != model.n_particles + model.n_rogues || particles.rows() != Model::size) {
                particles.resize(Model::size, model.n_particles + model.n_rogues);
                weights.resize(model.n_particles + model.n_rogues);
            }

            // Setup our multivariate normal distribution so we can initialise the particles
            MultivariateNormal<Scalar, Model::size> multivariate(mean, covariance);

            // Sample a random vector from the multivariate distribution for each particle
            for (int i = 0; i < particles.cols(); ++i) {
                particles.col(i) = multivariate.sample();
            }

            // Start all weights at 1
            std::fill(weights.begin(), weights.end(), Scalar(1));
        }

        // Resampling techniques!!!
        // http://users.isy.liu.se/rt/schon/Publications/HolSG2006.pdf

        // void resample(const Scalar& dt) {
        //     // Create a multivariate normal distribution with zero mean and the model's process noise as the
        //     covariance MultivariateNormal<Scalar, Model::size> multivariate(model.noise(dt));

        //     // Create a weighted sampler
        //     std::random_device rd;
        //     std::mt19937 gen(rd());
        //     std::discrete_distribution<int> weighted_sample(weights.begin(), weights.end());

        //     // Create a new particle list
        //     ParticleList resampled_particles(Model::size, particles.cols());

        //     for (int i = 0; i < model.n_particles; ++i) {
        //         // Randomly select a particle from all particles weighted by the particle weights
        //         const int sample_idx = weighted_sample(gen);

        //         // Create a new particle by using the selected particle as the mean + value from process noise
        //         resampled_particles.col(i) = particles.col(sample_idx) + multivariate.sample();
        //     }

        //     // Get the model to give us some rogue particles
        //     for (int i = 0; i < model.n_rogues; ++i) {
        //         resampled_particles.col(model.n_particles + i) = model.get_rogue();
        //     }

        //     // Update our particles with our resampled particles
        //     // Limit the state of each particle to ensure they are still valid
        //     for (int i = 0; i < particles.cols(); i++) {
        //         particles.col(i) = model.limit(resampled_particles.col(i));
        //     }

        //     // Reset all weights to 1
        //     std::fill(weights.begin(), weights.end(), Scalar(1));
        // }

        void multinomial_resample(const Scalar& dt) {
            // Create a multivariate normal distribution with zero mean and the model's process noise as the covariance
            MultivariateNormal<Scalar, Model::size> multivariate(model.noise(dt));

            // Create a weighted sampler
            std::random_device rd;
            std::mt19937 gen(rd());
            std::discrete_distribution<int> weighted_sample(weights.begin(), weights.end());

            // Create a new particle list
            ParticleList resampled_particles(Model::size, particles.cols());

            for (int i = 0; i < model.n_particles; ++i) {
                // Randomly select a particle from all particles weighted by the particle weights
                const int sample_idx = weighted_sample(gen);

                // Create a new particle by using the selected particle as the mean + value from process noise
                resampled_particles.col(i) = particles.col(sample_idx) + multivariate.sample();
            }

            // Get the model to give us some rogue particles
            for (int i = 0; i < model.n_rogues; ++i) {
                resampled_particles.col(model.n_particles + i) = model.get_rogue();
            }

            // Update our particles with our resampled particles
            // Limit the state of each particle to ensure they are still valid
            for (int i = 0; i < particles.cols(); i++) {
                particles.col(i) = model.limit(resampled_particles.col(i));
            }

            // Reset all weights to 1
            std::fill(weights.begin(), weights.end(), Scalar(1));
        }

        void stratified_resample(const Scalar& dt) {
            // Create a multivariate normal distribution with zero mean and the model's process noise as the covariance
            MultivariateNormal<Scalar, Model::size> multivariate(model.noise(dt));

            // Normalise the weights
            ParticleWeights normalised(weights.size(), Scalar(0));
            const Scalar sum = std::accumulate(weights.begin(), weights.end(), Scalar(0));
            std::transform(weights.begin(), weights.end(), normalised.begin(), [&](const Scalar& w) {
                return w / sum;
            });

            // Make a cumulative sum of the weights
            ParticleWeights cumsum(weights.size(), Scalar(0));
            std::partial_sum(normalised.begin(), normalised.end(), cumsum.begin());

            // Create a new particle list
            ParticleList resampled_particles(Model::size, particles.cols());

            std::random_device rd;   // Will be used to obtain a seed for the random number engine
            std::mt19937 gen(rd());  // Standard mersenne_twister_engine seeded with rd()
            std::uniform_real_distribution<Scalar> dis(Scalar(0), Scalar(1));

            for (int i = 0; i < model.n_particles; ++i) {
                // Pick a random number, uniformly, from [0, 1)
                // Use this to sample a particle
                const Scalar sample_weight = dis(gen);
                const Scalar sample_idx =
                    std::distance(cumsum.begin(), std::lower_bound(cumsum.begin(), cumsum.end(), sample_weight));

                // Create a new particle by using the selected particle as the mean + value from process noise
                resampled_particles.col(i) = particles.col(sample_idx) + multivariate.sample();
            }

            // Get the model to give us some rogue particles
            for (int i = 0; i < model.n_rogues; ++i) {
                resampled_particles.col(model.n_particles + i) = model.get_rogue();
            }

            // Reset all weights to 1
            std::fill(weights.begin(), weights.end(), Scalar(1));
        }

        void systematic_resample(const Scalar& dt) {
            // Create a multivariate normal distribution with zero mean and the model's process noise as the covariance
            MultivariateNormal<Scalar, Model::size> multivariate(model.noise(dt));

            // Normalise the weights
            ParticleWeights normalised(weights.size(), Scalar(0));
            const Scalar sum = std::accumulate(weights.begin(), weights.end(), Scalar(0));
            std::transform(weights.begin(), weights.end(), normalised.begin(), [&](const Scalar& w) {
                return w / sum;
            });

            // Make a cumulative sum of the weights
            ParticleWeights cumsum(weights.size(), Scalar(0));
            std::partial_sum(normalised.begin(), normalised.end(), cumsum.begin());

            // Create a new particle list
            ParticleList resampled_particles(Model::size, particles.cols());

            std::random_device rd;   // Will be used to obtain a seed for the random number engine
            std::mt19937 gen(rd());  // Standard mersenne_twister_engine seeded with rd()
            std::uniform_real_distribution<Scalar> dis(Scalar(0), Scalar(1));

            // Guaranteed random. I rolled a dice once
            const Scalar random_sample = dis(gen);

            for (int i = 0; i < model.n_particles; ++i) {
                // Pick a random number, uniformly, from [0, 1)
                // Use this to sample a particle
                const Scalar sample_weight = (i + random_sample) / particles.cols();
                const Scalar sample_idx =
                    std::distance(cumsum.begin(), std::lower_bound(cumsum.begin(), cumsum.end(), sample_weight));

                // Create a new particle by using the selected particle as the mean + value from process noise
                resampled_particles.col(i) = particles.col(sample_idx) + multivariate.sample();
            }

            // Get the model to give us some rogue particles
            for (int i = 0; i < model.n_rogues; ++i) {
                resampled_particles.col(model.n_particles + i) = model.get_rogue();
            }

            // Reset all weights to 1
            std::fill(weights.begin(), weights.end(), Scalar(1));
        }

        void residual_resample(const Scalar& dt) {
            // Create a multivariate normal distribution with zero mean and the model's process noise as the covariance
            MultivariateNormal<Scalar, Model::size> multivariate(model.noise(dt));

            // Normalise the weights
            ParticleWeights normalised(weights.size(), Scalar(0));
            const Scalar sum = std::accumulate(weights.begin(), weights.end(), Scalar(0));
            std::transform(weights.begin(), weights.end(), normalised.begin(), [&](const Scalar& w) {
                return std::floor(particles.cols() * w / sum);
            });

            // Create a new particle list
            ParticleList resampled_particles(Model::size, particles.cols());

            int particle_count = 0;
            for (int i = 0; i < model.n_particles; ++i) {
                // Replicate each particle n time
                for (int n = 0; n < normalised[i]; ++n) {
                    resampled_particles.col(particle_count + n) = particles.col(i) + multivariate.sample();
                }
                particle_count += normalised[i];
            }

            // Now sample the residual particles
            int num_residuals = model.n_particles - particle_count;

            // Adjust the weights
            std::transform(weights.begin(), weights.end(), normalised.begin(), [&](const Scalar& w) {
                return (particles.cols() * w / sum) - std::floor(particles.cols() * w / sum);
            });
        }

    public:
        template <typename... Args>
        void time(const Scalar& dt, const Args&... params) {
            // Resample our particles
            systematic_resample(dt);

            // For each particle update its state using the time update
            for (int i = 0; i < particles.cols(); ++i) {
                particles.col(i) = model.time(particles.col(i), dt, params...);
            }
        }

        template <typename MeasurementScalar, int S, int... VArgs, int... MArgs, typename... Args>
        MeasurementScalar measure(const Eigen::Matrix<MeasurementScalar, S, 1, VArgs...>& measurement,
                                  const Eigen::Matrix<MeasurementScalar, S, S, MArgs...>& measurement_covariance,
                                  const Args&... params) {
            // Get the model to make a prediction for each particle
            Eigen::Matrix<MeasurementScalar, S, Eigen::Dynamic> differences(S, particles.cols());
            for (int i = 0; i < particles.cols(); ++i) {
                differences.col(i) = model.difference(model.predict(particles.col(i), params...), measurement);
            }

            // Recalculate the weights of each particle
            // The weight comes from the density function of a zero mean, measurement_covariance covariance multivariate
            // normal distribution
            MultivariateNormal<Scalar, S> multivariate(measurement_covariance);
            for (int i = 0; i < particles.cols(); ++i) {
                weights[i] *= multivariate.density(differences.col(i));
            }

            // Calculate log probabilities
            Eigen::Matrix<MeasurementScalar, Eigen::Dynamic, 1> logits =
                -differences.cwiseProduct(measurement_covariance.inverse() * differences).colwise().sum();

            // Subtract the max log prob for numerical stability and then exponentiate
            logits = Eigen::exp(logits.array() - logits.maxCoeff());

            return logits.mean();
        }

        // Take the mean of all particles, pay no attention to the type of the data
        StateVec get() const {
            return particles.rowwise().mean();
        }

        StateMat getCovariance() const {
            auto mean_centered = particles.transpose().rowwise() - particles.transpose().colwise().mean();
            return (mean_centered.transpose() * mean_centered) / (particles.transpose().rows() - 1);
        }

        const ParticleList& getParticles() const {
            return particles;
        }
    };
}  // namespace utility::math::filter


#endif
