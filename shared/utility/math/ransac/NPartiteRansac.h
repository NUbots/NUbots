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
 * Copyright 2015 NUBots <nubots@nubots.net>
 */

#ifndef UTILITY_MATH_RANSAC_NPARTITERANSAC_H
#define UTILITY_MATH_RANSAC_NPARTITERANSAC_H

#include <nuclear>
#include <utility>
#include <vector>

namespace utility {
namespace math {
namespace ransac {

    template <typename Model>
    struct NPartiteRansac {

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
        static void regenerateRandomModel(Model& model, const std::array<Iterator, Model::REQUIRED_POINTS + 1>& iterators) {

            std::array<DataPoint, Model::REQUIRED_POINTS> points;
            do {
                for (uint i = 0; i < Model::REQUIRED_POINTS; ++i) {
                    uint range = std::distance(iterators[i], iterators[i + 1]);
                    points[i] = *std::next(iterators[i], xorShift() % range);
                }
            }
            while(!model.regenerate(points));
        }

        /**
         * Finds an individual ransac model that fits the data
         *
         * @return A pair containing an iterator to the start of the remaining set, and the best fitting model
         */
        template <typename Iterator>
        static RansacResult<Iterator> fitModel(std::array<Iterator, Model::REQUIRED_POINTS + 1>& iterators
                                             , uint minimumPointsForConsensus
                                             , uint maximumIterationsPerFitting
                                             , double consensusErrorThreshold) {

            // Check we have enough points in each part
            for(uint i = 0; i < Model::REQUIRED_POINTS; ++i) {
                if(std::distance(iterators[i], iterators[i + 1]) < 1) {
                    return RansacResult<Iterator>{ false, Model(), Iterator(), Iterator() };
                }
            }

            uint largestConsensus = 0;
            double bestError = std::numeric_limits<double>::max();
            Model bestModel;
            Model model;

            for(uint i = 0; i < maximumIterationsPerFitting; ++i) {

                uint consensusSize = 0;
                double error = 0.0;

                // Make our model have new set of points
                regenerateRandomModel(model, iterators);

                // Look through all our points and see if they fall within our error bounds
                for(auto it = iterators.front(); it != iterators.back(); ++it) {
                    if(model.calculateError(*it) < consensusErrorThreshold) {
                        ++consensusSize;
                        error += consensusErrorThreshold;
                    }
                }

                // If this model has the largest consensus so far
                if(consensusSize > largestConsensus or
                   (consensusSize == largestConsensus and error < bestError)) {
                    largestConsensus = consensusSize;
                    bestError = error;
                    bestModel = std::move(model);
                }
            }

            // If the best model we found is good enough
            if(largestConsensus >= minimumPointsForConsensus) {

                // If we can, refine the model using the points in the consensus
                model.refineModel(iterators.front(), iterators.back(), consensusErrorThreshold);

                // Split off the valid points in each part to the start of it
                std::array<uint, Model::REQUIRED_POINTS> offsets;
                for(uint i = 0; i < Model::REQUIRED_POINTS; ++i) {

                    // Split the points from this list off to the start
                    auto newStart = std::partition(iterators[i], iterators[i + 1], [consensusErrorThreshold, bestModel] (const DataPoint& point) {
                       return consensusErrorThreshold > bestModel.calculateError(std::forward<const DataPoint&>(point));
                    });
                    offsets[i] = std::distance(newStart, iterators[i + 1]);
                }

                // Now we split off all our new points into a global partition
                auto newFirst = std::stable_partition(iterators.front(), iterators.back(), [consensusErrorThreshold, bestModel] (const DataPoint& point) {
                    return consensusErrorThreshold > bestModel.calculateError(std::forward<const DataPoint&>(point));
                });

                // We now start with this new first
                auto start = iterators.front();
                iterators.front() = newFirst;

                // Put in our new iterator points
                for(uint i = 0; i < Model::REQUIRED_POINTS; ++i) {
                    // Move each iterator along
                    iterators[i + 1] = std::next(iterators[i], offsets[i]);
                }

                // Return the result
                return RansacResult<Iterator>{ true, bestModel, start, newFirst };
            }
            else {
                // We couldn't find a good enough set, return a flag that we failed
                return RansacResult<Iterator>{ false, Model(), Iterator(), Iterator() };
            }
        }

        template <typename Iterator>
        static std::vector<RansacResult<Iterator>> fitModels(std::array<Iterator, Model::REQUIRED_POINTS + 1> iterators
                                                           , uint minimumPointsForConsensus
                                                           , uint maximumIterationsPerFitting
                                                           , uint maximumFittedModels
                                                           , double consensusErrorThreshold) {

            std::vector<RansacResult<Iterator>> results;
            results.reserve(maximumFittedModels);

            while(results.size() < maximumFittedModels) {
                RansacResult<Iterator> result = fitModel(iterators, minimumPointsForConsensus, maximumIterationsPerFitting, consensusErrorThreshold);

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
