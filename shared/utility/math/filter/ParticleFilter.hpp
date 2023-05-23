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
 * Copyright 2023 NUbots <nubots@nubots.net>
 */

#ifndef UTILITY_MATH_FILTER_PARTICLEFILTER_HPP
#define UTILITY_MATH_FILTER_PARTICLEFILTER_HPP

#include <Eigen/Dense>
#include <random>
#include <utility>
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
     * @author Alex Biddulph, Trent Houliston
     *
     * @details Uses a set of particles (or samples) to represent possible states in a system. Each particle is randomly
     * selected from a multivariate normal distribution. Each particle is weighted by how likely it is to be the real
     * state of the system. Particles with a higher weight are more likely to be resampled (with noise) during the time
     * update.
     *
     * Time Update: Resamples all particles, perturbing the resampled particles with the model's process noise, and asks
     * the model to predict what should happen to each particle after an elapsed amount of time.
     *
     * Measurement Update: Asks the model to predict what the next measurement will be (based on current state/particle)
     * and the finds the difference between the model's prediction and the actual measurement. The likelihood of this
     * prediction occuring, given the measurement, is then determined using a multivariate normal distribution
     * probability density function.
     *
     * @tparam Scalar The scalar type to use for all calculations. This puts an upper bound on the numerical precision
     * that is achievable by the filter
     * @tparam FilterModel The model to use in the filtering process. This model needs to provide the equations that
     * models the system.
     */
    template <typename Scalar, template <typename> class FilterModel>
    class ParticleFilter {
    public:
        /// The possible resampling methods
        /// @see http://users.isy.liu.se/rt/schon/Publications/HolSG2006.pdf for an explanation on each one
        enum class ResampleMethod { MULTINOMIAL, STRATIFIED, SYSTEMATIC };
        ResampleMethod resample_method = ResampleMethod::SYSTEMATIC;

        bool use_residual_resampling = true;

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
        ParticleFilter() {
            set_state(StateVec::Zero(), StateMat::Identity() * 0.1);
        }

        /**
         * @brief Constructs the particle filter with the provided values. The resampling method with default to the
         * residual method with the systematic method for resampling the residual particles. It is an error to use the
         * residual method for resampling residuals and an exception will be thrown.
         *
         * @param mean The mean of the multivariate normal distribution. Used for sampling the initial particles in the
         * filter.
         * @param covariance The covariance of the multivariate normal distribution. Used for sampling the initial
         * particles in the filter.
         * @param resample_method_ The method to use when resampling particles during the time update step
         * @param use_residual_resampling_ Whether residual resampling should be used with the other method
         */
        ParticleFilter(const StateVec& mean,
                       const StateMat& covariance,
                       const ResampleMethod& resample_method_ = ResampleMethod::SYSTEMATIC,
                       const bool& use_residual_resampling_   = true)
            : resample_method(resample_method_), use_residual_resampling(use_residual_resampling_) {
            set_state(mean, covariance);
        }

        /**
         * @brief Constructs the particle filter with the provided values. The resampling method with default to the
         * residual method with the systematic method for resampling the residual particles. It is an error to use the
         * residual method for resampling residuals and an exception will be thrown.
         *
         * This version initialises the particles using multiple initial hypotheses. The particle count is split evenly
         * across all hypotheses. If the division between the number of hypotheses and the number of particles is not
         * even, then the remaining particles will be treated as rogues. It is an error for the numbers of means and
         * covariances to differ.
         *
         * @param mean The mean of the multivariate normal distribution. Used for sampling the initial particles in the
         * filter. There is one mean for each initial hypothesis.
         * @param covariance The covariance of the multivariate normal distribution. Used for sampling the initial
         * particles in the filter.  There is one covariance for each initial hypothesis.
         * @param resample_method_ The method to use when resampling particles during the time update step.
         * @param use_residual_resampling_ Whether residual resampling should be used with the other method
         */
        ParticleFilter(const std::vector<std::pair<StateVec, StateMat>>& hypotheses,
                       const ResampleMethod& resample_method_ = ResampleMethod::SYSTEMATIC,
                       const bool& use_residual_resampling_   = true)
            : resample_method(resample_method_), use_residual_resampling(use_residual_resampling_) {

            set_state(hypotheses);
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

        /**
         * @brief Clears all particles in the filter and samples new particles from the multivariate normal
         * distribution with the given mean and covariance. Sampled particles are limited (using model::limit)
         * function to ensure all particles are valid. Particle weights are initialised to 1.
         *
         * This version initialises the particles using multiple initial hypotheses. The particle count is split evenly
         * across all hypotheses. If the division between the number of hypotheses and the number of particles is not
         * even, then the remaining particles will be treated as rogues. It is an error for the numbers of means and
         * covariances to differ.
         *
         * @param mean The mean of the multivariate normal distribution. There is one mean for each initial hypothesis.
         * @param covariance The covariance of the multivariate normal distribution. There is one covariance for each
         * initial hypothesis.
         */
        void set_state(const std::vector<std::pair<StateVec, StateMat>>& hypotheses) {
            // Make sure we have sane inputs
            if (hypotheses.empty()) {
                throw std::runtime_error("ParticleFilter::set_state called with invalid data");
            }

            // Initialise all of the particles
            if (hypotheses.size() == 1) {
                init(hypotheses[0].first, hypotheses[0].second);
            }
            else {
                init(hypotheses);
            }

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
            for (int i = 0; i < model.n_particles; ++i) {
                particles.col(i) = multivariate.sample();
            }

            // Get the model to give us some rogues
            for (int i = 0; i < model.n_rogues; ++i) {
                particles.col(model.n_particles + i) = model.get_rogue();
            }

            // Start all weights at 1
            std::fill(weights.begin(), weights.end(), Scalar(1));
        }

        /**
         * @brief Clears all particles in the filter and samples new particles from the multivariate normal
         * distribution with the given mean and covariance. Particle weights are initialised to 1.
         *
         * This version initialises the particles using multiple initial hypotheses. The particle count is split evenly
         * across all hypotheses. If the division between the number of hypotheses and the number of particles is not
         * even, then the remaining particles will be treated as rogues.
         *
         * @param mean The mean of the multivariate normal distribution.
         * @param covariance The covariance of the multivariate normal distribution.
         */
        void init(const std::vector<std::pair<StateVec, StateMat>>& hypotheses) {
            // Make sure our particle list has the right shape
            if (particles.cols() != model.n_particles + model.n_rogues || particles.rows() != Model::size) {
                particles.resize(Model::size, model.n_particles + model.n_rogues);
                weights.resize(model.n_particles + model.n_rogues);
            }

            // We want to evenly distribute the particles across all hypotheses
            // If the distribution is not even then the remaining particles will be treated as extra rogues
            const int particles_per_state = model.n_particles / hypotheses.size();

            // For each hypothesis sample a random vector from the multivariate distribution for each particle
            for (int hypothesis = 0; hypothesis < int(hypotheses.size()); ++hypothesis) {
                // Setup our multivariate normal distribution for this hypothesis so we can initialise the particles
                MultivariateNormal<Scalar, Model::size> multivariate(hypotheses[hypothesis].first,
                                                                     hypotheses[hypothesis].second);

                const int start_col = hypothesis * particles_per_state;
                for (int i = 0; i < particles_per_state; ++i) {
                    particles.col(start_col + i) = multivariate.sample();
                }
            }

            // Get the model to give us some rogues
            const int num_particles = particles_per_state * hypotheses.size();
            const int num_rogues    = model.n_particles - num_particles + model.n_rogues;
            for (int i = 0; i < num_rogues; ++i) {
                particles.col(num_particles + i) = model.get_rogue();
            }

            // Start all weights at 1
            std::fill(weights.begin(), weights.end(), Scalar(1));
        }

        /**
         * @brief Use the chosen resample and residual method to resample the particles in the filter based on their
         * weights.
         */
        std::vector<int> resample_particles() {

            // Select the resampling method to use
            if (use_residual_resampling) {
                // The residual method uses a secondary method when it resamples the residual particles
                switch (resample_method) {
                    case ResampleMethod::MULTINOMIAL:
                        return stats::resample::residual(model.n_particles,
                                                         weights.begin(),
                                                         weights.end(),
                                                         stats::resample::multinomial<decltype(weights.begin())>);
                    case ResampleMethod::STRATIFIED:
                        return stats::resample::residual(model.n_particles,
                                                         weights.begin(),
                                                         weights.end(),
                                                         stats::resample::stratified<decltype(weights.begin())>);
                    case ResampleMethod::SYSTEMATIC:
                        return stats::resample::residual(model.n_particles,
                                                         weights.begin(),
                                                         weights.end(),
                                                         stats::resample::systematic<decltype(weights.begin())>);
                }
            }
            switch (resample_method) {
                case ResampleMethod::MULTINOMIAL:
                    return stats::resample::multinomial(model.n_particles, weights.begin(), weights.end());
                case ResampleMethod::STRATIFIED:
                    return stats::resample::stratified(model.n_particles, weights.begin(), weights.end());
                case ResampleMethod::SYSTEMATIC:
                    return stats::resample::systematic(model.n_particles, weights.begin(), weights.end());
            }
            // This shouldn't happen unless a new enum element is added and the switches aren't updated
            throw std::runtime_error("Invalid setting for resample method.");
        }

        /**
         * @brief Do a weighted resampling of all of the particles in the filter. The model is also asked to provide new
         * rogues. All particle weights are reset to 1.
         */
        void resample() {

            // Get indexes to the particles that are being resampled
            // Some particles may be resampled multiple times
            std::vector<int> idx = resample_particles();

            // Resample the particles
            // For each index get the corresponding particle and perturb it with the model's process noise
            ParticleList resampled_particles(Model::size, particles.cols());
            for (int i = 0; i < int(idx.size()); ++i) {
                // Create a new particle by using the selected particle as the mean + value from process noise
                resampled_particles.col(i) = particles.col(idx[i]);
            }

            // Get the model to give us some rogue particles
            for (int i = 0; i < model.n_rogues; ++i) {
                resampled_particles.col(model.n_particles + i) = model.get_rogue();
            }

            // Update our particles with our resampled particles
            for (int i = 0; i < particles.cols(); i++) {
                particles.col(i) = resampled_particles.col(i);
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
        void time(const Scalar& dt, Args&&... params) {

            // Resample our particles
            resample();

            // Make a multivariate normal distribution with zero mean and the model's process noise as the covariance
            // This will be used to perturb the resampled particles
            MultivariateNormal<Scalar, Model::size> multivariate(model.noise(dt));

            // Perturb each particle and get the model to apply a time update. Then limit particle to ensure it is valid
            for (int i = 0; i < particles.cols(); ++i) {
                particles.col(i) = model.limit(
                    model.time(particles.col(i) + multivariate.sample(), dt, std::forward<Args>(params)...));
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
                                  Args&&... params) {

            // Get the model to make a prediction for each particle
            // Also get the model to tell us how much a prediction deviates from the observed measurement
            Eigen::Matrix<MeasurementScalar, S, Eigen::Dynamic> differences(S, particles.cols());
            for (int i = 0; i < particles.cols(); ++i) {
                differences.col(i) =
                    model.difference(model.predict(particles.col(i), std::forward<Args>(params)...), measurement);
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
        [[nodiscard]] StateVec get_state() const {
            return particles.rowwise().mean();
        }

        /**
         * @brief Calculates and returns the covariance of the particles.
         */
        [[nodiscard]] StateMat get_covariance() const {
            auto mean_centered = particles.transpose().rowwise() - particles.transpose().colwise().mean();
            return (mean_centered.transpose() * mean_centered) / (particles.transpose().rows() - 1);
        }

        /**
         * @brief Returns the underlying particle list.
         */
        [[nodiscard]] const ParticleList& getParticles() const {
            return particles;
        }

        /**
         * @brief Returns the underlying particle weights.
         */
        [[nodiscard]] const ParticleWeights& getParticleWeights() const {
            return weights;
        }
    };
}  // namespace utility::math::filter


#endif
