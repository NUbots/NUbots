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

#ifndef UTILITY_MATH_RANSAC_RANSAC_HPP
#define UTILITY_MATH_RANSAC_RANSAC_HPP

#include <nuclear>
#include <utility>
#include <vector>

#include "RansacResult.hpp"

namespace utility::math::ransac {

    template <typename Model>
    struct Ransac {

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
        static bool regenerateRandomModel(Model& model, const Iterator first, const Iterator last, Args... args) {

            // Get random points between first and last
            uint range = std::distance(first, last);
            std::array<DataPoint, Model::REQUIRED_POINTS> points;

            // Grab a unique random set
            std::set<uint> indices;
            while (indices.size() < Model::REQUIRED_POINTS) {
                indices.insert(xorShift() % range);
            }

            uint i = 0;
            for (auto& index : indices) {
                points[i++] = *std::next(first, index);
            }
            return model.regenerate(points, std::forward<Args>(args)...);
        }

        /**
         * Finds an individual ransac model that fits the data
         *
         * @return A pair containing an iterator to the start of the remaining set, and the best fitting model
         */
        template <typename Iterator, typename... Args>
        static std::pair<bool, RansacResult<Iterator, Model>> findModel(Iterator& first,
                                                                        Iterator& last,
                                                                        uint minimumPointsForConsensus,
                                                                        uint maximumIterationsPerFitting,
                                                                        double consensusErrorThreshold,
                                                                        Args... args) {

            // Check we have enough points
            if (std::distance(first, last) < int(minimumPointsForConsensus)) {
                return std::make_pair(false, RansacResult<Iterator, Model>());
            }

            uint largestConsensus = 0;
            double bestError      = std::numeric_limits<double>::max();
            Model bestModel;
            Model model;

            for (uint i = 0; i < maximumIterationsPerFitting; ++i) {

                uint consensusSize = 0;
                double error       = 0.0;

                // Make our model have new set of points
                if (!regenerateRandomModel(model, first, last, std::forward<Args>(args)...)) {
                    continue;
                }

                for (auto it = first; it != last; ++it) {
                    float this_error = model.calculateError(*it);
                    if (this_error < consensusErrorThreshold) {
                        ++consensusSize;
                        error += this_error;
                    }
                }

                // If largest consensus
                if (consensusSize > largestConsensus or (consensusSize == largestConsensus and error < bestError)) {
                    largestConsensus = consensusSize;
                    bestError        = error;
                    bestModel        = std::move(model);
                }
            }

            if (largestConsensus >= minimumPointsForConsensus) {

                bestModel.refineModel(first, last, consensusErrorThreshold);

                auto newFirst =
                    std::partition(first, last, [consensusErrorThreshold, bestModel](const DataPoint& point) {
                        return consensusErrorThreshold
                               > bestModel.calculateError(std::forward<const DataPoint&>(point));
                    });
                first = newFirst;

                return std::make_pair(true, RansacResult<Iterator, Model>(bestModel, first, newFirst));
            }
            else {
                return std::make_pair(false, RansacResult<Iterator, Model>());
            }
        }

        template <typename Iterator, typename... Args>
        static std::vector<RansacResult<Iterator, Model>> fitModels(Iterator first,
                                                                    Iterator last,
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
                std::tie(valid, result) = findModel(first,
                                                    last,
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
