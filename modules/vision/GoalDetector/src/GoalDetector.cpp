/*
 * This file is part of NUbots Codebase.
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

#include "GoalDetector.h"

#include "messages/vision/ClassifiedImage.h"
#include "messages/support/Configuration.h"
#include "messages/vision/VisionObjects.h"
#include "utility/math/geometry/Quad.h"

#include "utility/math/geometry/Line.h"

#include "utility/math/ransac/RansacLineModel.h"

namespace modules {
namespace vision {

    using utility::math::geometry::Line;
    using utility::math::geometry::Quad;

    using utility::math::ransac::RansacLineModel;
    using utility::math::ransac::findMultipleModels;
    using utility::math::ransac::RansacSelectionMethod;

    using messages::vision::ObjectClass;
    using messages::vision::ClassifiedImage;
    using messages::vision::Goal;

    using messages::support::Configuration;


    GoalDetector::GoalDetector(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Trigger<Configuration<GoalDetector>>>([this](const Configuration<GoalDetector>& config) {

            std::string selectionMethod = config["ransac"]["selection_method"].as<std::string>();

            if (selectionMethod.compare("LARGEST_CONSENSUS") == 0) {
                SELECTION_METHOD = RansacSelectionMethod::LARGEST_CONSENSUS;
            }
            else if (selectionMethod.compare("BEST_FITTING_CONSENSUS") == 0) {
                SELECTION_METHOD = RansacSelectionMethod::BEST_FITTING_CONSENSUS;
            }
            else {
                SELECTION_METHOD = RansacSelectionMethod::LARGEST_CONSENSUS;
            }

            MINIMUM_POINTS = config["ransac"]["minimum_points"].as<uint>();
            MAX_ITERATIONS_PER_FITTING = config["ransac"]["max_iterations_per_fitting"].as<uint>();
            MAX_FITTING_ATTEMPTS = config["ransac"]["max_fitting_attempts"].as<uint>();
            CONSENSUS_THRESHOLD = config["ransac"]["consensus_threshold"].as<double>();
        });

        on<Trigger<ClassifiedImage<ObjectClass>>>([this](const ClassifiedImage<ObjectClass>& image) {

            std::vector<arma::vec2> startPoints, endPoints;
            std::vector<arma::mat22> starts, ends;

            auto hSegments = image.horizontalSegments.equal_range(ObjectClass::GOAL);
            for(auto it = hSegments.first; it != hSegments.second; ++it) {

                // We throw out points if they are:
                // Less the full quality (subsampled)
                // Do not have a transition on either side (are on an edge)
                if(it->second.subsample == 1
                    && it->second.previous
                    && it->second.next) {

                    startPoints.push_back({ double(it->second.start[0]), double(it->second.start[1]) });
                    endPoints.push_back({ double(it->second.end[0]), double(it->second.end[1]) });
                }
            }

            // Use ransac to find left edges.
            for (auto& line : findMultipleModels<RansacLineModel<arma::vec2>, arma::vec2>(startPoints,
                                                                                        CONSENSUS_THRESHOLD,
                                                                                        MINIMUM_POINTS,
                                                                                        MAX_ITERATIONS_PER_FITTING,
                                                                                        MAX_FITTING_ATTEMPTS,
                                                                                        SELECTION_METHOD)) {

                // Compare based on Y co-ordinate
                auto comp = [] (const arma::vec2& a, const arma::vec2& b) {
                    return a[1] < b[1];
                };

                // Find min and max y
                auto high = std::min_element(line.second.begin(), line.second.end(), comp);
                auto low = std::max_element(line.second.begin(), line.second.end(), comp);

                // Store the values
                starts.emplace_back();
                starts.back().col(0) = line.first.orthogonalProjection(*high);
                starts.back().col(1) = line.first.orthogonalProjection(*low);;
            }

            // Use ransac to find right edges.
            for (auto& line : findMultipleModels<RansacLineModel<arma::vec2>, arma::vec2>(endPoints,
                                                                                        CONSENSUS_THRESHOLD,
                                                                                        MINIMUM_POINTS,
                                                                                        MAX_ITERATIONS_PER_FITTING,
                                                                                        MAX_FITTING_ATTEMPTS,
                                                                                        SELECTION_METHOD)) {

                auto comp = [] (const arma::vec2& a, const arma::vec2& b) {
                    return a[1] < b[1];
                };

                // Find min and max y
                auto high = std::min_element(line.second.begin(), line.second.end(), comp);
                auto low = std::max_element(line.second.begin(), line.second.end(), comp);

                // Store the values
                ends.emplace_back();
                ends.back().col(0) = line.first.orthogonalProjection(*high);
                ends.back().col(1) = line.first.orthogonalProjection(*low);;
            }


            auto goals = std::make_unique<std::vector<Goal>>();

            for(auto& line : starts) {

                goals->emplace_back();
                goals->back().quad.set(line.col(1), line.col(0), line.col(0), line.col(1));
            }

            for(auto& line : ends) {

                goals->emplace_back();
                goals->back().quad.set(line.col(1), line.col(0), line.col(0), line.col(1));
            }

            emit(std::move(goals));

        });
    }

}
}
