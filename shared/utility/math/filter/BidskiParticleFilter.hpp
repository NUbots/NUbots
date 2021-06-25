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
#include "utility/math/stats/resample/multinomial.hpp"
#include "utility/math/stats/resample/resample.hpp"
#include "utility/math/stats/resample/residual.hpp"
#include "utility/math/stats/resample/stratified.hpp"
#include "utility/math/stats/resample/systematic.hpp"

namespace utility::math::filter {

    using utility::math::stats::MultivariateNormal;

    /**
     * Particle Filter
     *
     * @author Alex Biddulph, Trent Houliston
     *
     * @details Uses a set of particles (or samples) to represent possible states in a system. Each particle is randomly
     * selected from a multivariate normal distribution. Each particle is weighted by how likely it is to be the real
     * state of the system. Particles with a higher weight are more likely to be resampled (with noise) during the time
     * update.
     *
     * Time Update: Resamples all particles, perturbing the resampled particles with the model's process noise, and asks
     * the model to predict what should happen to each particle after an elapsed amount of time. Also known as the
     * predict step.
     *
     * Measurement Update: Asks the model to predict what the next measurement will be (based on current state/particle)
     * and the finds the difference between the model's prediction and the actual measurement. The likelihood of this
     * measurement occuring is then determined using a multivariate normal distribution probability density function.
     *
     * @tparam Scalar The scalar type to use for all calculations. This puts an upper bound on the numerical precision
     * that is achievable by the filter
     * @tparam FilterModel The model to use in the filtering process. This model needs to provide the equations that
     * models the system.
     */
    template <typename Scalar, template <typename> class FilterModel>
    class BidskiParticleFilter {
    public:
        using Model = FilterModel<Scalar>;
        // The model
        Model model;

        using StateVec = Eigen::Matrix<Scalar, Model::size, 1>;
        using StateMat = Eigen::Matrix<Scalar, Model::size, Model::size>;

        /**
         * @brief Constructs the particle filter using default values. Mean is set to 0 and covariance is set to
         * \f$0.1 * I_{n}\f$, where n is the size of the model. The residual method is used for resampling particles and
         * the residual will use the systematic method for resampling the residual particles.
         */
        BidskiParticleFilter() {
            set_state(StateVec::Zero(), StateMat::Identity() * 0.1);
        }
        BidskiParticleFilter(const StateVec& mean, const StateMat& covariance) {
        /**
         * @brief Constructs the particle filter with the provided values. The resampling method with default to the
         * residual method with the systematic method for resampling the residual particles. It is an error to use the
         * residual method for resampling residuals and an exception will be thrown.
         *
         * @param mean The mean of the multivariate normal distribution. Used for sampling the initial particles in the
         * filter.
         * @param covariance The covariance of the multivariate normal distribution. Used for sampling the initial
         * particles in the filter.
         * @param resample_method_ The method to use when resampling particles during the time update step.
         * @param residual_method_ When using ResampleMethod::RESIDUAL for the resample method, this method will be used
         * when the residual method needs to resample residual particles.
         */
            set_state(mean, covariance);
        }

        /**
         * @brief Clears all particles in the filter and samples new particles from the multivariate normal
         * distribution with the given mean and covariance. Sampled particles are limited (using model::limit)
         * function to ensure all particles are valid. Particle weights are initialised to 1.
         *
         * @param mean The mean of the multivariate normal distribution.
         * @param covariance The covariance of the multivariate normal distribution.
         */
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

        /**
         * @brief Clears all particles in the filter and samples new particles from the multivariate normal
         * distribution with the given mean and covariance. Particle weights are initialised to 1.
         *
         * @param mean The mean of the multivariate normal distribution.
         * @param covariance The covariance of the multivariate normal distribution.
         */
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

        /**
        /**
         * @brief Do a weighted resampling of all of the particles in the filter. Resampled particles are then perturbed
         * (using model::noise) and the model is asked to provide new rogue particles (if the model is using
         * them). Resampled particles are then limited (using model::limit) to ensure validity and all particle weights
         * are reset to 1.
         *
         * @param dt The amount of time that has elapsed since the previous time update step.
         */
        void resample(const Scalar& dt) {

            // Make a multivariate normal distribution with zero mean and the model's process noise as the covariance
            // This will be used to perturb the resampled particles
            MultivariateNormal<Scalar, Model::size> multivariate(model.noise(dt));

            // Resample the particles
            // For each index get the corresponding particle and perturb it with the model's process noise
            ParticleList resampled_particles(Model::size, particles.cols());
            std::vector<int> idx = stats::resample::residual(model.n_particles, weights.begin(), weights.end());
            for (int i = 0; i < int(idx.size()); ++i) {
                // Create a new particle by using the selected particle as the mean + value from process noise
                resampled_particles.col(i) = particles.col(idx[i]) + multivariate.sample();
            }

            // Get the model to give us some rogue particles
            for (int i = 0; i < model.n_rogues; ++i) {
                resampled_particles.col(model.n_particles + i) = model.get_rogue();
            }

            // Update our particles with our resampled particles and limit each one to ensure it remains valid
            for (int i = 0; i < particles.cols(); i++) {
                particles.col(i) = model.limit(resampled_particles.col(i));
            }

            // Reset all weights to 1
            std::fill(weights.begin(), weights.end(), Scalar(1));
        }

    public:
        /**
         * @brief Resample all particles in the filter and set their weights back to 1. The model is asked to give a
         * prediction on what happens to each particle after dt time has elapsed (using model::time).
         *
         * @tparam Args the types of any extra arguments that are provided to the model's time update function.
         *
         * @param dt The amount of time that has elapsed since the previous time update step.
         * @param params Any extra arguments that are to be forwarded to the model's time update function.
         */
        template <typename... Args>
        void time(const Scalar& dt, const Args&... params) {

            // Resample our particles
            resample(dt);

            // Get the model to apply a time update to each particle
            for (int i = 0; i < particles.cols(); ++i) {
                particles.col(i) = model.time(particles.col(i), dt, params...);
            }
        }

        /**
         * @brief Get the model to predict (using model::predict) what the next measurement will be then determine how
         * accurate this prediction is (using model::difference). The likelihood of the measurement is determined using
         * the multivariate normal distribution probability density function, this likelihood (determined for each
         * particle) re-weights each particle.
         *
         * @tparam MeasurementScalar The underlying numerical type of the measurement
         * @tparam S The dimensionality of the measurement (and associated covariance matrix)
         * @tparam VArgs The types of any extra arguments associated with constructing the measurement vector.
         * @tparam MArgs The types of any extra arguments associated with constructing the measurement covariance
         * matrix.
         * @tparam Args the types of any extra arguments that are provided to the model's predict function.
         *
         * @param measurement The measurement (from sensors, etc).
         * @param measurement_covariance The covariance of the measurement.
         * @param params Any extra arguments that are to be forwarded to the model's predict function.
         */
        template <typename MeasurementScalar, int S, int... VArgs, int... MArgs, typename... Args>
        MeasurementScalar measure(const Eigen::Matrix<MeasurementScalar, S, 1, VArgs...>& measurement,
                                  const Eigen::Matrix<MeasurementScalar, S, S, MArgs...>& measurement_covariance,
                                  const Args&... params) {

            // Get the model to make a prediction for each particle
            // Also get the model to tell us how much a prediction deviates from the observed measurement
            Eigen::Matrix<MeasurementScalar, S, Eigen::Dynamic> differences(S, particles.cols());
            for (int i = 0; i < particles.cols(); ++i) {
                differences.col(i) = model.difference(model.predict(particles.col(i), params...), measurement);
            }

            // Recalculate the weights of each particle
            // The weight update factor comes from the density function of a zero mean, measurement_covariance
            // covariance multivariate normal distribution
            MultivariateNormal<Scalar, S> multivariate(measurement_covariance);
            for (int i = 0; i < particles.cols(); ++i) {
                weights[i] *= multivariate.density(differences.col(i));
            }

            // Calculate log probabilities
            Eigen::Matrix<MeasurementScalar, Eigen::Dynamic, 1> logits =
                -differences.cwiseProduct(measurement_covariance.inverse() * differences).colwise().sum();

            // Subtract the max log probability for numerical stability and then exponentiate
            logits = Eigen::exp(logits.array() - logits.maxCoeff());

            // Return the mean log probability
            return logits.mean();
        }

        /**
         * @brief Calculates and returns the mean of all particles.
         */
        [[nodiscard]] StateVec get() const {
            return particles.rowwise().mean();
        }

        /**
         * @brief Calculates and returns the covariance of the particles.
         */
        [[nodiscard]] StateMat getCovariance() const {
            auto mean_centered = particles.transpose().rowwise() - particles.transpose().colwise().mean();
            return (mean_centered.transpose() * mean_centered) / (particles.transpose().rows() - 1);
        }

        /**
         * @brief Returns the underlying particle list.
         */
        [[nodiscard]] const ParticleList& getParticles() const {
            return particles;
        }
    };
}  // namespace utility::math::filter


#endif
