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

#ifndef UTILITY_MATH_FILTER_PARTICLEFILTER_H
#define UTILITY_MATH_FILTER_PARTICLEFILTER_H

#include <Eigen/Dense>
#include <random>
#include <vector>

namespace utility {
namespace math {
    namespace filter {

        template <typename Scalar, template <typename> class FilterModel>
        class ParticleFilter {
        public:
            using Model = FilterModel<Scalar>;
            // The model
            Model model;

            using StateVec = Eigen::Matrix<Scalar, Model::size, 1>;
            using StateMat = Eigen::Matrix<Scalar, Model::size, Model::size>;

        private:
            // Our random number generator
            std::mt19937 rng;
            std::normal_distribution<Scalar> norm;

            // Dimension types for vectors and square matricies
            using ParticleList = Eigen::Matrix<Scalar, Model::size, Eigen::Dynamic>;

            ParticleList sample_particles(const StateVec& mean, const StateMat& covariance, const int& n_particles) {
                // Sample single gaussian (represented by a gaussian mixture model of size 1)

                // Implementation based on the work presented in
                // Conrad Sanderson and Ryan Curtin.
                // An Open Source C++ Implementation of Multi-Threaded Gaussian Mixture Models, k-Means and Expectation
                // Maximisation. International Conference on Signal Processing and Communication Systems, 2017.

                ParticleList new_particles =
                    ParticleList::NullaryExpr(Model::size, n_particles, [&]() { return norm(rng); });

                const StateMat sqrt_covariance = covariance.cwiseSqrt();

                for (int i = 0; i < n_particles; ++i) {
                    new_particles.col(i) = new_particles.col(i).cwiseProduct(sqrt_covariance.col(0)) + mean;
                }

                return new_particles;
            }

            ParticleList sample_particles(const StateVec& mean, const StateVec& covariance, const int& n_particles) {
                // Sample single gaussian (represented by a gaussian mixture model of size 1)

                // Implementation based on the work presented in
                // Conrad Sanderson and Ryan Curtin.
                // An Open Source C++ Implementation of Multi-Threaded Gaussian Mixture Models, k-Means and Expectation
                // Maximisation. International Conference on Signal Processing and Communication Systems, 2017.

                ParticleList new_particles =
                    ParticleList::NullaryExpr(Model::size, n_particles, [&]() { return norm(rng); });

                const StateVec sqrt_covariance = covariance.cwiseSqrt();

                for (int i = 0; i < n_particles; ++i) {
                    new_particles.col(i) = new_particles.col(i).cwiseProduct(sqrt_covariance) + mean;
                }

                return new_particles;
            }

        public:
            ParticleFilter(const StateVec& initial_mean      = StateVec::Zero(),
                           const StateMat& initialCovariance = StateMat::Identity() * 0.1,
                           const int& number_of_particles    = 100) {
                set_state(initial_mean, initialCovariance, number_of_particles);
            }

            void set_state(const StateVec& initial_mean,
                           const StateMat& initialCovariance,
                           const int& number_of_particles = 100) {
                particles = sample_particles(initial_mean, initialCovariance, number_of_particles);

                // Limit the state of each particle to ensure they are still valid
                for (unsigned int i = 0; i < particles.cols(); i++) {
                    particles.col(i) = model.limit(particles.col(i));
                }
            }

            template <typename... Args>
            void time(const Scalar& dt, const Args&... params) {
                // Time update our particles
                for (unsigned int i = 0; i < particles.cols(); ++i) {
                    particles.col(i) = model.time(particles.col(i), dt, params...);
                }

                // Perturb them by noise
                particles += sample_particles(StateVec::Zero(), StateVec(model.noise(dt).diagonal()), particles.cols());

                // Limit the state of each particle to ensure they are still valid
                for (unsigned int i = 0; i < particles.cols(); i++) {
                    particles.col(i) = model.limit(particles.col(i));
                }
            }

            template <typename MeasurementScalar, int S, int... VArgs, int... MArgs, typename... Args>
            MeasurementScalar measure(const Eigen::Matrix<MeasurementScalar, S, 1, VArgs...>& measurement,
                                      const Eigen::Matrix<MeasurementScalar, S, S, MArgs...>& measurement_variance,
                                      const Args&... params) {

                ParticleList candidate_particles =
                    ParticleList::Zero(Model::size, particles.cols() + model.getRogueCount());
                candidate_particles.leftCols(particles.cols()) = particles;

                // Resample some rogues
                for (int i = 0; i < model.getRogueCount(); ++i) {
                    candidate_particles.col(i + particles.cols()) =
                        particles.col(i) + model.getRogueRange().cwiseProduct(StateVec::Random() * 0.5);
                }

                Eigen::Matrix<MeasurementScalar, S, Eigen::Dynamic> differences(S, particles.cols());
                for (int i = 0; i < candidate_particles.cols(); ++i) {
                    differences.col(i) = model.difference(model.predict(particles.col(i), params...), measurement);
                }

                // Calculate log probabilities
                Eigen::Matrix<MeasurementScalar, Eigen::Dynamic, 1> logits =
                    -differences.cwiseProduct(measurement_variance.inverse() * differences).colwise().sum();

                // Subtract the max log prob for numerical stability and then exponentiate
                logits = Eigen::exp(logits.array() - logits.maxCoeff());

                // Resample
                std::discrete_distribution<> multinomial(logits.data(), logits.data() + logits.size());
                for (unsigned int i = 0; i < particles.cols(); i++) {
                    particles.col(i) = model.limit(candidate_particles.col(multinomial(rng)));
                }
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

        private:
            ParticleList particles;
        };
    }  // namespace filter
}  // namespace math
}  // namespace utility


#endif
