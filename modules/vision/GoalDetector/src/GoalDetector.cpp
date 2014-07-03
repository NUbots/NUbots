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

#include "utility/math/ransac/Ransac.h"
#include "utility/math/ransac/RansacLineModel.h"

namespace modules {
namespace vision {

    using utility::math::geometry::Line;
    using utility::math::geometry::Quad;

    using utility::math::ransac::Ransac;
    using utility::math::ransac::RansacLineModel;

    using messages::vision::ObjectClass;
    using messages::vision::ClassifiedImage;
    using messages::vision::Goal;

    using messages::support::Configuration;


    GoalDetector::GoalDetector(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Trigger<Configuration<GoalDetector>>>([this](const Configuration<GoalDetector>& config) {
            MINIMUM_POINTS_FOR_CONSENSUS = config["ransac"]["minimum_points_for_consensus"].as<uint>();
            CONSENSUS_ERROR_THRESHOLD = config["ransac"]["consensus_error_threshold"].as<double>();
            MAXIMUM_ITERATIONS_PER_FITTING = config["ransac"]["maximum_iterations_per_fitting"].as<uint>();
            MAXIMUM_FITTED_MODELS = config["ransac"]["maximum_fitted_models"].as<uint>();

            MINIMUM_ASPECT_RATIO = config["aspect_ratio_range"][0].as<double>();
            MAXIMUM_ASPECT_RATIO = config["aspect_ratio_range"][1].as<double>();
        });

        on<Trigger<ClassifiedImage<ObjectClass>>>([this](const ClassifiedImage<ObjectClass>& image) {

            std::vector<arma::vec2> startPoints, endPoints;
            std::vector<arma::mat22> starts, ends;

            auto hSegments = image.horizontalSegments.equal_range(ObjectClass::GOAL);
            for(auto it = hSegments.first; it != hSegments.second; ++it) {

                // We throw out points if they are:
                // Less the full quality (subsampled)
                // Do not have a transition on the other side
                if(it->second.subsample == 1 && it->second.previous) {
                    startPoints.push_back({ double(it->second.start[0]), double(it->second.start[1]) });
                }

                if(it->second.subsample == 1 && it->second.next) {
                    endPoints.push_back({ double(it->second.end[0]), double(it->second.end[1]) });
                }
            }

            // Use ransac to find left edges.
            for (auto& result : Ransac<RansacLineModel>::fitModels(startPoints.begin()
                                                               , startPoints.end()
                                                               , MINIMUM_POINTS_FOR_CONSENSUS
                                                               , MAXIMUM_ITERATIONS_PER_FITTING
                                                               , MAXIMUM_FITTED_MODELS
                                                               , CONSENSUS_ERROR_THRESHOLD)) {

                // Compare based on Y co-ordinate
                auto comp = [] (const arma::vec2& a, const arma::vec2& b) {
                    return a[1] < b[1];
                };

                // Find min and max y
                std::vector<arma::vec2>::iterator high, low;
                std::tie(high, low) = std::minmax_element(result.first, result.last, comp);

                // Store the values
                starts.emplace_back();
                starts.back().col(0) = result.model.orthogonalProjection(*high);
                starts.back().col(1) = result.model.orthogonalProjection(*low);
            }

            // Use ransac to find right edges.
            for (auto& result : Ransac<RansacLineModel>::fitModels(endPoints.begin()
                                                                , endPoints.end()
                                                                , MINIMUM_POINTS_FOR_CONSENSUS
                                                                , MAXIMUM_ITERATIONS_PER_FITTING
                                                                , MAXIMUM_FITTED_MODELS
                                                                , CONSENSUS_ERROR_THRESHOLD)) {

                auto comp = [] (const arma::vec2& a, const arma::vec2& b) {
                    return a[1] < b[1];
                };

                // Find min and max y
                std::vector<arma::vec2>::iterator high, low;
                std::tie(high, low) = std::minmax_element(result.first, result.last, comp);

                // Store the values
                ends.emplace_back();
                ends.back().col(0) = result.model.orthogonalProjection(*high);
                ends.back().col(1) = result.model.orthogonalProjection(*low);
            }

            // Our output goal vector
            auto goals = std::make_unique<std::vector<Goal>>();
            goals->reserve(starts.size() * ends.size());

            // Form quads from all of the lines
            for(auto& start : starts) {
                for(auto& end : ends) {

                    // If we can make a valid quad from the points
                    if(start(0, 0) < end(0, 1) && start(1, 0) < end(1, 1)) {
                        goals->emplace_back();
                        goals->back().quad.set(start.col(1), start.col(0), end.col(0), end.col(1));
                    }
                }
            }

            // Throwout invalid quads
            for(auto it = goals->begin(); it < goals->end();) {

                auto& quad = it->quad;

                // Check if we are within the aspect ratio range
                bool throwout = quad.aspectRatio() < MINIMUM_ASPECT_RATIO
                             || quad.aspectRatio() > MAXIMUM_ASPECT_RATIO
                // Check if we are close enough to the visual horizon
                             || (image.visualHorizonAtPoint(quad.getBottomLeft()[0]) > quad.getBottomLeft()[1]
                                 && image.visualHorizonAtPoint(quad.getBottomRight()[0]) > quad.getBottomLeft()[1])
                // Check we finish above the kinematics horizon or or kinematics horizon is off the screen
                             || image.horizon.y(quad.getTopLeft()[0]) > quad.getTopLeft()[1]
                             || image.horizon.y(quad.getTopRight()[0]) > quad.getTopRight()[1];

                if(throwout) {
                    it = goals->erase(it);
                }
                else {
                    ++it;
                }
            }

            // Combine all possible lines into quads

            // Throw out bad quads

            // bad aspect ratio
            // base must be below the vh
            // Top of goals is above the kinematics horizion OR kh is above the screen

            // Refine the tops and bottoms of the goals using the vertical segments

            // Try and assign left and right
                // Check for left side vs right side
                // Check for a crossbar


            emit(std::move(goals));

        });
    }

}
}
