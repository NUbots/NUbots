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

#include "AutoClassifierProvider.h"

#include "messages/vision/VisionObjects.h"
#include "messages/support/Configuration.h"
#include "messages/research/AutoClassifierPixels.h"

namespace modules {
namespace research {

    using messages::input::Image;
    using messages::vision::Ball;
    using messages::vision::Goal;
    using messages::vision::Colour;
    using messages::support::Configuration;
    using messages::research::AutoClassifierPixels;

    AutoClassifierProvider::AutoClassifierProvider(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Trigger<Configuration<AutoClassifierProvider>>>([this] (const Configuration<AutoClassifierProvider>& config) {

            ballProvider.enable(config["ball"]["enabled"].as<bool>());
            ballEdgeBuffer = config["ball"]["edge_buffer"].as<int>();
            goalProvider.enable(config["goal"]["enabled"].as<bool>());
            goalEdgeBuffer = config["goal"]["edge_buffer"].as<int>();
            // fieldProvider.enable(config["field"].as<bool>());
            // lineProvider.enable(config["line"].as<bool>());
        });

        ballProvider = on<Trigger<std::vector<Ball>>, Options<Single, Priority<NUClear::LOW>>>("Auto Classifier Provider Balls", [this](const std::vector<Ball>& balls) {

            auto pixels = std::make_unique<AutoClassifierPixels>();
            pixels->classification = Colour::ORANGE;

            // Loop through our balls
            for (auto& ball : balls) {
                auto& image = *ball.classifiedImage->image;
                auto& circle = ball.circle;

                double radius = circle.radius;
                arma::vec2 centre = circle.centre;

                // find the min and max y points on the circle
                // capped at the bounds of the image
                uint minY = std::max(std::ceil(centre[1] - radius), 0.0);
                uint maxY = std::min(std::floor(centre[1] + radius), double(image.height() - 1));

                // loop through pixels on the image in bounding box
                for (uint y = minY + ballEdgeBuffer; y <= maxY - ballEdgeBuffer; y++) {
                    auto edgePoints = circle.getEdgePoints(y);
                    uint minX = std::max(edgePoints[0], 0.0);
                    uint maxX = std::min(edgePoints[1], double(image.width() - 1));

                    for (uint x = minX + ballEdgeBuffer; x <= maxX - ballEdgeBuffer; x++) {
                        pixels->pixels.push_back(image(x, y));
                    }
                }

            }

            emit(std::move(pixels));
        });

        goalProvider = on<Trigger<std::vector<Goal>>, Options<Single, Priority<NUClear::LOW>>>("Auto Classifier Goals", [this](const std::vector<Goal>& goals) {

            auto pixels = std::make_unique<AutoClassifierPixels>();
            pixels->classification = Colour::YELLOW;

            for (auto& goal : goals) {
                auto& image = *goal.classifiedImage->image;
                auto& quad = goal.quad;

                // find the min and max y points on the quad
                // capped at the bounds of the image
                uint minY = std::max(std::min(quad.getTopLeft()[1], quad.getTopRight()[1]), 0.0);
                uint maxY = std::min(std::max(quad.getBottomLeft()[1], quad.getBottomRight()[1]), double(image.height() - 1));

                for (uint y = minY + goalEdgeBuffer; y <= maxY - goalEdgeBuffer; y++) {
                    arma::vec2 edgePoints;
                    try {
                        edgePoints = quad.getEdgePoints(y);
                    } catch (std::domain_error&) {
                        continue; // no intersection
                    }
                    uint minX = std::max(edgePoints[0], 0.0);
                    uint maxX = std::min(edgePoints[1], double(image.width() - 1));

                    for (uint x = minX + goalEdgeBuffer; x <= maxX - goalEdgeBuffer; x++) {
                        pixels->pixels.push_back(image(x, y));
                    }
                }
            }

            emit(std::move(pixels));
        });
    }
}
}

