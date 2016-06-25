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

#ifndef UTILITY_MATH_FILTER_MMUKF_H
#define UTILITY_MATH_FILTER_MMUKF_H

#include <nuclear>
#include <armadillo>

namespace utility {
    namespace math {
        namespace filter {

            template <typename Model>
            class MMUKF {

                struct Filter {
                    double weight;
                    UKF<Model> filter;
                }

                std::vector<Filter> filters;
                int maxModels = 2;
                double mergeProbability = 0.9;

                double bhattacharyyaDistance(const UKF<Model>& a, const UKF<Model>& b) {
                    // Get our state difference
                    const arma::vec ud = a.model.observationDistance(a.get(), b.get());

                    // Get our 3 covariance matricies we need
                    const auto& s1 = a.getCovariance();
                    const auto& s2 = b.getCovariance();
                    arma::mat s = (s1 + s2) * 0.5;

                    return (0.125 * ud).t() * arma::sympd_inv(s) * ud + 0.5 * std::ln(arma::det(s) / std::sqrt(arma::det(s1) * arma::det(s2)));
                }

                void prune() {

                    // First we merge similar models
                    std::sort(std::begin(filters), std::end(filters));

                    // Now we merge to the number of models we need
                    // We can do this because merging models that we are going
                    // to cut off later is pointless. We only need the first
                    // n most probable models
                    for (int i = 0; i < std::min(maxModels, filters.size()); ++i) {

                        auto end = std::remove_if(filters.begin() + i + 1, filters.end(); [](const Filter& f) {
                            return mergeProbability < bhattacharyyaDistance(filters[i].filter, filter.filter);
                        });

                        filters.erase(end, filters.end());
                    }

                    // Now we cut off the models we don't need
                    filters.resize(maxModels);

                    // Normalise our weights
                    double totalWeight = 0;
                    for (auto& filter : filters) {
                        totalWeight += filters.weight;
                    }
                    totalWeight = 1.0 / totalWeight;
                    for (auto& & filter : filters) {
                        filters.weight *= totalWeight;
                    }
                }

                template <typename... TAdditionalParameters>
                void timeUpdate(double deltaT, const TAdditionalParameters&... additionalParameters) {

                    // Prune before time update
                    prune();

                    for (auto& filter : filters) {
                        filter.filter.timeUpdate(deltaT, additionalParameters...);
                    }
                }


                template <typename TMeasurement, typename... TMeasurementArgs>
                void measurementUpdate(const TMeasurement& measurement
                                     , const arma::mat& measurementVariance
                                     , const TMeasurementArgs&... measurementArgs) {

                    for (auto& filter : filters) {
                        double weight = filter.filter.measurementUpdate(measurement, measurementVariance, measurementArgs...);
                        filter.weight *= weight;
                    }
                }

                template <typename... TArgs, size_t... I>
                void applyMeasurement(Filter& filter, const std::tuple<TArgs...>& args, const std::index_sequence<I...>&) {
                    filter.filter.measurementUpdate(std::get<I>(args)...);
                }

                template <typename TMeasurement, typename... TMeasurementArgs>
                void measurementUpdate(const std::vector<std::tuple<TMeasurement, arma::mat, TMeasurementArgs...>>& measurements) {

                    std::vector<Filter> newFilters;

                    for (auto& filter : filters) {

                        for (int i = 0; i < measurements.size(); ++i) {
                            Filter split = filter;

                            std::get<0>(measurements), std::get<1>(measurements), std::get<2...N>(measurements);

                            double weight = split.filter.measurementUpdate(measurement[i], measurementVariances[i], measurementArgs[i]...);
                            split.weight *= weight;

                            newFilters.push_back(split);
                        }
                    }
                }
            }
        }
    }
}

#endif  // UTILITY_MATH_FILTER_MMUKF_H
