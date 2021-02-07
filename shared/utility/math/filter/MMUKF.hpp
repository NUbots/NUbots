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
 * Copyright 2016 NUbots <nubots@nubots.net>
 */

#ifndef UTILITY_MATH_FILTER_MMUKF_HPP
#define UTILITY_MATH_FILTER_MMUKF_HPP

#include <armadillo>
#include <nuclear>

#include "UKF.hpp"

namespace utility {
namespace math {
    namespace filter {

        template <typename Model>
        class MMUKF {
        public:
            struct Filter {

                Filter(double weight = 1.0, UKF<Model> filter = UKF<Model>()) : weight(weight), filter(filter) {}

                double weight;
                UKF<Model> filter;

                bool operator<(const Filter& other) {
                    // Sort from big to small
                    return weight > other.weight;
                }
            };

            std::vector<Filter> filters;
            size_t maxModels        = 2;
            double mergeProbability = 0.9;

            static double bhattacharyyaDistance(const UKF<Model>& a, const UKF<Model>& b) {
                // Get our state difference
                const arma::vec ud = a.model.observationDifference(a.get(), b.get());

                // Get our 3 covariance matricies we need
                const auto& s1 = a.getCovariance();
                const auto& s2 = b.getCovariance();
                arma::mat s    = (s1 + s2) * 0.5;

                double d = arma::as_scalar((0.125 * ud).t() * arma::inv_sympd(s) * ud)
                           + 0.5 * std::log(arma::det(s) / std::sqrt(arma::det(s1) * arma::det(s2)));
                return d;
            }

            void renormalise() {

                // Normalise our weights
                double totalWeight = 0;
                for (auto& filter : filters) {
                    totalWeight = std::min(filter.weight, totalWeight);
                }
                // totalWeight = 1.0 / totalWeight;
                for (auto& filter : filters) {
                    filter.weight -= totalWeight;
                }
            }

            void prune() {
                // Sort so the most likely model is first
                std::sort(std::begin(filters), std::end(filters));

                // First we merge similar models
                // Now we merge to the number of models we need
                // We can do this because merging models that we are going
                // to cut off later is pointless. We only need the first
                // n most probable models
                for (uint i = 0; i < std::min(maxModels, filters.size()); ++i) {

                    auto end = std::remove_if(filters.begin() + i + 1, filters.end(), [this, i](const Filter& f) {
                        return mergeProbability < bhattacharyyaDistance(filters[i].filter, f.filter);
                    });

                    filters.erase(end, filters.end());
                }

                // Now we cut off the models we don't need
                filters.resize(maxModels);

                // Normalise our weights
                renormalise();
            }

            template <typename... TArgs, size_t... I>
            double applyMeasurement(Filter& filter,
                                    const std::tuple<TArgs...>& args,
                                    const std::index_sequence<I...>&) {
                return filter.filter.measurementUpdate(std::get<I>(args)...);
            }

        public:
            MMUKF(size_t maxModels = 2, double mergeProbability = 0.9)
                : filters(), maxModels(maxModels), mergeProbability(mergeProbability) {}

            template <typename... TAdditionalParameters>
            void timeUpdate(double deltaT, const TAdditionalParameters&... additionalParameters) {

                // Prune before time update
                prune();

                for (auto& filter : filters) {
                    filter.filter.timeUpdate(deltaT, additionalParameters...);
                }
            }

            template <typename TMeasurement, typename... TMeasurementArgs>
            void measurementUpdate(const TMeasurement& measurement,
                                   const arma::mat& measurementVariance,
                                   const TMeasurementArgs&... measurementArgs) {

                for (auto& filter : filters) {
                    double weight =
                        filter.filter.measurementUpdate(measurement, measurementVariance, measurementArgs...);
                    filter.weight += weight;
                }
            }

            template <typename TMeasurement, typename... TMeasurementArgs>
            void measurementUpdate(
                const std::initializer_list<std::tuple<TMeasurement, arma::mat, TMeasurementArgs...>>& measurements) {

                std::vector<Filter> newFilters;
                for (auto& filter : filters) {

                    for (auto& measurement : measurements) {
                        Filter split = filter;

                        double weight = applyMeasurement(split,
                                                         measurement,
                                                         std::make_index_sequence<2 + sizeof...(TMeasurementArgs)>());
                        split.weight += weight;
                        // std::cerr << split.weight << std::endl;
                        newFilters.push_back(split);
                    }
                }

                std::sort(newFilters.begin(), newFilters.end());

                filters = std::move(newFilters);

                // Normalise our weights
                renormalise();
            }

            const typename UKF<Model>::StateVec& get() const {
                return filters[0].filter.get();
            }

            const typename UKF<Model>::StateMat& getCovariance() const {
                return filters[0].filter.getCovariance();
            }
        };
    }  // namespace filter
}  // namespace math
}  // namespace utility

#endif  // UTILITY_MATH_FILTER_MMUKF_HPP
