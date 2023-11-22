/*
 * MIT License
 *
 * Copyright (c) 2013 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef UTILITY_MATH_RANSAC_NPARTITERANSAC_HPP
#define UTILITY_MATH_RANSAC_NPARTITERANSAC_HPP

#include <nuclear>
#include <utility>
#include <vector>

#include "RansacResult.hpp"

namespace utility::math::ransac {

    template <typename Model>
    struct NPartiteRansac {

        using DataPoint = typename Model::DataPoint;

        static uint64_t xorShift() {
            static thread_local uint64_t s[2] = {uint64_t(rand()), uint64_t(rand())};

            uint64_t s1       = s[0];
            const uint64_t s0 = s[1];
            s[0]              = s0;
            s1 ^= s1 << 23;
            return (s[1] = (s1 ^ s0 ^ (s1 >> 17) ^ (s0 >> 26))) + s0;
        }

        template <typename Iterator, typename... Args>
        static bool regenerateRandomModel(Model& model,
                                          const std::array<Iterator, Model::REQUIRED_POINTS + 1>& iterators,
                                          Args... args) {

            std::array<DataPoint, Model::REQUIRED_POINTS> points;
            for (uint i = 0; i < Model::REQUIRED_POINTS; ++i) {
                uint range = std::distance(iterators[i], iterators[i + 1]);
                points[i]  = *std::next(iterators[i], xorShift() % range);
            }
            return model.regenerate(points, std::forward<Args>(args)...);
        }

        /**
         * Finds an individual ransac model that fits the data
         *
         * @return A pair containing an iterator to the start of the remaining set, and the best fitting model
         */
        template <typename Iterator, typename... Args>
        static std::pair<bool, RansacResult<Iterator, Model>> fitModel(
            std::array<Iterator, Model::REQUIRED_POINTS + 1>& iterators,
            uint minimumPointsForConsensus,
            uint maximumIterationsPerFitting,
            double consensusErrorThreshold,
            Args... args) {

            // Check we have enough points in each part
            for (uint i = 0; i < Model::REQUIRED_POINTS; ++i) {
                if (std::distance(iterators[i], iterators[i + 1]) < 1) {
                    return std::make_pair(false, RansacResult<Iterator, Model>());
                }
            }

            uint largestConsensus = 0;
            double bestError      = std::numeric_limits<double>::max();
            Model bestModel;
            Model model;

            for (uint i = 0; i < maximumIterationsPerFitting; ++i) {

                uint consensusSize = 0;
                double error       = 0.0;

                // Make our model have new set of points
                if (!regenerateRandomModel(model, iterators, std::forward<Args>(args)...)) {
                    continue;
                }

                // Look through all our points and see if they fall within our error bounds
                for (auto it = iterators.front(); it != iterators.back(); ++it) {
                    if (model.calculateError(*it) < consensusErrorThreshold) {
                        ++consensusSize;
                        error += consensusErrorThreshold;
                    }
                }

                // If this model has the largest consensus so far
                if (consensusSize > largestConsensus or (consensusSize == largestConsensus and error < bestError)) {
                    largestConsensus = consensusSize;
                    bestError        = error;
                    bestModel        = std::move(model);
                }
            }

            // If the best model we found is good enough
            if (largestConsensus >= minimumPointsForConsensus) {

                // If we can, refine the model using the points in the consensus
                bestModel.refineModel(iterators.front(), iterators.back(), consensusErrorThreshold);

                // Split off the valid points in each part to the start of it
                std::array<uint, Model::REQUIRED_POINTS> offsets;
                for (uint i = 0; i < Model::REQUIRED_POINTS; ++i) {

                    // Split the points from this list off to the start
                    auto newStart = std::partition(iterators[i],
                                                   iterators[i + 1],
                                                   [consensusErrorThreshold, bestModel](const DataPoint& point) {
                                                       return consensusErrorThreshold > bestModel.calculateError(
                                                                  std::forward<const DataPoint&>(point));
                                                   });
                    offsets[i]    = std::distance(newStart, iterators[i + 1]);
                }

                // Now we split off all our new points into a global partition
                auto newFirst = std::stable_partition(iterators.front(),
                                                      iterators.back(),
                                                      [consensusErrorThreshold, bestModel](const DataPoint& point) {
                                                          return consensusErrorThreshold > bestModel.calculateError(
                                                                     std::forward<const DataPoint&>(point));
                                                      });

                // We now start with this new first
                auto start        = iterators.front();
                iterators.front() = newFirst;

                // Put in our new iterator points
                for (uint i = 0; i < Model::REQUIRED_POINTS; ++i) {
                    // Move each iterator along
                    iterators[i + 1] = std::next(iterators[i], offsets[i]);
                }

                // Return the result
                return std::make_pair(true, RansacResult<Iterator, Model>(std::move(bestModel), start, newFirst));
            }
            else {
                // We couldn't find a good enough set, return a flag that we failed
                return std::make_pair(false, RansacResult<Iterator, Model>());
            }
        }

        template <typename Iterator, typename... Args>
        static std::vector<RansacResult<Iterator, Model>> fitModels(
            std::array<Iterator, Model::REQUIRED_POINTS + 1> iterators,
            uint minimumPointsForConsensus,
            uint maximumIterationsPerFitting,
            uint maximumFittedModels,
            double consensusErrorThreshold,
            Args... args) {

            std::vector<RansacResult<Iterator, Model>> results;
            results.reserve(maximumFittedModels);

            while (results.size() < maximumFittedModels) {
                bool valid;
                RansacResult<Iterator, Model> result;
                std::tie(valid, result) = fitModel(iterators,
                                                   minimumPointsForConsensus,
                                                   maximumIterationsPerFitting,
                                                   consensusErrorThreshold,
                                                   std::forward<Args>(args)...);

                // If we have more datapoints left then add this one and continue
                if (valid) {
                    results.push_back(std::move(result));
                }
                else {
                    return results;
                }
            }

            return results;
        }
    };
}  // namespace utility::math::ransac
#endif
