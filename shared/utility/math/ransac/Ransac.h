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

#ifndef UTILITY_MATH_RANSAC_RANSAC_H
#define UTILITY_MATH_RANSAC_RANSAC_H

#include <nuclear>
#include <utility>
#include <vector>

namespace utility {
namespace math {
namespace ransac {

    template <typename Model>
    struct Ransac {

        template <typename Iterator>
        struct RansacResult {
            bool valid;
            Model model;
            Iterator first;
            Iterator last;
        };

        using DataPoint = typename Model::DataPoint;

        static uint64_t xorShift() {
            static thread_local uint64_t s[2] = { uint64_t(rand()), uint64_t(rand()) };

            uint64_t s1 = s[0];
            const uint64_t s0 = s[1];
            s[0] = s0;
            s1  ^= s1 << 23;
            return (s[1] = (s1 ^ s0 ^ (s1 >> 17) ^ (s0 >> 26))) + s0;
        }

        template <typename Iterator>
        static void regenerateRandomModel(Model& model, const Iterator first, const Iterator last) {

            // Get random points between first and last
            uint range = std::distance(first, last);
            std::array<DataPoint, Model::REQUIRED_POINTS> points;
            do {
                std::set<uint64_t> indices;

                while(indices.size() < Model::REQUIRED_POINTS) {
                    indices.insert(xorShift() % range);
                }

                uint i = 0;
                for(auto& index : indices) {
                    points[i++]= *std::next(first, index);
                }

                // If this returns false then it was an invalid model
            }
            while(!model.regenerate(points));
        }

        /**
         * Finds an individual ransac model that fits the data
         *
         * @return A pair containing an iterator to the start of the remaining set, and the best fitting model
         */
        template <typename Iterator>
        static std::pair<Iterator, RansacResult<Iterator>> findModel(Iterator first
                                                                   , Iterator last
                                                                   , uint minimumPointsForConsensus
                                                                   , uint maximumIterationsPerFitting
                                                                   , double consensusErrorThreshold) {

            // Check we have enough points
            if(std::distance(first, last) < int(minimumPointsForConsensus)) {
                return std::make_pair(last, RansacResult<Iterator>{ false, Model(), Iterator(), Iterator() });
            }

            uint largestConsensus = 0;
            double bestError = std::numeric_limits<double>::max();
            Model bestModel;
            Model model;

            for(uint i = 0; i < maximumIterationsPerFitting; ++i) {

                uint consensusSize = 0;
                double error = 0.0;

                regenerateRandomModel(model, first, last);
                for(auto it = first; it != last; ++it) {
                    if(model.calculateError(*it) < consensusErrorThreshold) {
                        ++consensusSize;
                        error += consensusErrorThreshold;
                    }
                }

                // If largest consensus
                if(consensusSize > largestConsensus or
                   (consensusSize == largestConsensus and error < bestError)) {
                    largestConsensus = consensusSize;
                    bestError = error;
                    bestModel = std::move(model);
                }
            }

            if(largestConsensus >= minimumPointsForConsensus) {

                model.refineModel(first,last,consensusErrorThreshold);

                auto newFirst = std::partition(first, last, [consensusErrorThreshold, bestModel] (const DataPoint& point) {
                    return consensusErrorThreshold > bestModel.calculateError(std::forward<const DataPoint&>(point));
                });

                return std::make_pair(newFirst, RansacResult<Iterator>{ true, bestModel, first, newFirst });
            }
            else {
                return std::make_pair(last, RansacResult<Iterator>{ false, Model(), Iterator(), Iterator() });
            }
        }

        template <typename Iterator>
        static std::vector<RansacResult<Iterator>> fitModels(Iterator first
                                                           , Iterator last
                                                           , uint minimumPointsForConsensus
                                                           , uint maximumIterationsPerFitting
                                                           , uint maximumFittedModels
                                                           , double consensusErrorThreshold) {

            std::vector<RansacResult<Iterator>> results;
            results.reserve(maximumFittedModels);

            while(results.size() < maximumFittedModels) {
                RansacResult<Iterator> result;
                std::tie(first, result) = findModel(first, last, minimumPointsForConsensus, maximumIterationsPerFitting, consensusErrorThreshold);

                // If we have more datapoints left then add this one and continue
                if(result.valid) {
                    results.push_back(std::move(result));
                }
                else {
                    return results;
                }
            }

            return results;
        }
    };

}
}
}

#endif
