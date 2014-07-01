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

#include "utility/math/geometry/Line.h"

#include "utility/math/ransac/RansacLineModel.h"

namespace modules {
namespace vision {

    using utility::math::geometry::Line;
    using utility::math::geometry::Quad;
    using utility::math::geometry::LSFittedLine;

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
            std::vector<LSFittedLine> startLines, endLines;

            // Get all our points for a horizontal line
            auto hSegments = image.horizontalSegments.equal_range(ObjectClass::GOAL);
            for(auto it = hSegments.first; it != hSegments.second; ++it) {

                // We throw out points if they are:
                // Less the full quality (subsampled)
                // Do not have a transition on the other side side (are on an edge with no other segment)
                if(it->second.subsample == 1 && it->second.previous) {

                    startPoints.push_back({ double(it->second.start[0]), double(it->second.start[1]) });
                }

                if(it->second.subsample == 1 && it->second.next) {

                    endPoints.push_back({ double(it->second.end[0]), double(it->second.end[1]) });
                }
            }

            // Use ransac to find all the left edges
            for (auto& l : findMultipleModels<RansacLineModel<arma::vec2>, arma::vec2>(startPoints,
                                                                                        CONSENSUS_THRESHOLD,
                                                                                        MINIMUM_POINTS,
                                                                                        MAX_ITERATIONS_PER_FITTING,
                                                                                        MAX_FITTING_ATTEMPTS,
                                                                                        SELECTION_METHOD) {
                startLines.push_back(LSFittedLine(l.second));
            }

            // Use ransac to find all the right edges
            for (auto& l : findMultipleModels<RansacLineModel<arma::vec2>, arma::vec2>(endPoints,
                                                                                        CONSENSUS_THRESHOLD,
                                                                                        MINIMUM_POINTS,
                                                                                        MAX_ITERATIONS_PER_FITTING,
                                                                                        MAX_FITTING_ATTEMPTS,
                                                                                        SELECTION_METHOD) {
                endLines.push_back(LSFittedLine(l.second));
            }

            // Make quads I guess?





            auto goals = std::make_unique<std::vector<Goal>>();

            for(auto& q : postCandidates) {

                Goal goal;

                goal.quad = q;

                goals->push_back(std::move(goal));

            }

            emit(std::move(goals));
        });
    }

}
}

