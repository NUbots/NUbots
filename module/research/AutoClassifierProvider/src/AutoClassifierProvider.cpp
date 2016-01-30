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

#include "message/vision/VisionObjects.h"
#include "message/vision/ClassifiedImage.h"
#include "message/support/Configuration.h"
#include "message/research/AutoClassifierPixels.h"

namespace module {
namespace research {

    using message::input::Image;
    using message::vision::Ball;
    using message::vision::Goal;
    using message::vision::ObjectClass;
    using message::vision::ClassifiedImage;
    using message::vision::Colour;
    using message::support::Configuration;
    using message::research::AutoClassifierPixels;

    AutoClassifierProvider::AutoClassifierProvider(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("AutoClassifierProvider.yaml").then([this] (const Configuration& config) {

            ballProvider.enable(config["ball"]["enabled"].as<bool>());
            ballEdgeBuffer = config["ball"]["edge_buffer"].as<int>();
            ballLightnessMin = config["ball"]["lightness_range"][0].as<int>();
            ballLightnessMax = config["ball"]["lightness_range"][1].as<int>();
            goalProvider.enable(config["goal"]["enabled"].as<bool>());
            goalEdgeBuffer = config["goal"]["edge_buffer"].as<int>();
            goalLightnessMin = config["goal"]["lightness_range"][0].as<int>();
            goalLightnessMax = config["goal"]["lightness_range"][1].as<int>();
            fieldProvider.enable(config["field"]["enabled"].as<bool>());
            fieldEdgeBuffer = config["field"]["edge_buffer"].as<int>();
            fieldLightnessMin = config["field"]["lightness_range"][0].as<int>();
            fieldLightnessMax = config["field"]["lightness_range"][1].as<int>();
            // lineProvider.enable(config["field"]["enabled"].as<bool>());
            // lineEdgeBuffer = config["field"]["edge_buffer"].as<int>();

            // lineProvider.enable(config["line"].as<bool>());
        });

        ballProvider = on<Trigger<std::vector<Ball>>, Single, Priority::LOW>()
        .then("Auto Classifier Provider Balls", [this] (const std::vector<Ball>& balls) {

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
                uint maxY = std::min(std::floor(centre[1] + radius), double(image.height - 1));

                // loop through pixels on the image in bounding box
                for (uint y = minY + ballEdgeBuffer; y <= maxY - ballEdgeBuffer; ++y) {
                    auto edgePoints = circle.getEdgePoints(y);
                    uint minX = std::max(edgePoints[0], 0.0);
                    uint maxX = std::min(edgePoints[1], double(image.width - 1));

                    for (uint x = minX + ballEdgeBuffer; x <= maxX - ballEdgeBuffer; ++x) {
                        auto pixel = image(x, y);
                        if(pixel.y > ballLightnessMin && y < ballLightnessMax) {
                            pixels->pixels.push_back(pixel);
                        }
                    }
                }

            }

            emit(std::move(pixels));
        });

         goalProvider = on<Trigger<std::vector<Goal>>, Single, Priority::LOW>()
         .then("Auto Classifier Goals", [this](const std::vector<Goal>& goals) {

            auto pixels = std::make_unique<AutoClassifierPixels>();
            pixels->classification = Colour::YELLOW;

            for (auto& goal : goals) {
                auto& image = *goal.classifiedImage->image;
                auto& quad = goal.quad;

                // find the min and max y points on the quad
                // capped at the bounds of the image
                uint minY = std::max(std::min(quad.getTopLeft()[1], quad.getTopRight()[1]), 0.0);
                uint maxY = std::min(std::max(quad.getBottomLeft()[1], quad.getBottomRight()[1]), double(image.height - 1));

                for (uint y = minY + goalEdgeBuffer; y <= maxY - goalEdgeBuffer; ++y) {
                    arma::vec2 edgePoints;
                    try {
                        edgePoints = quad.getEdgePoints(y);
                    } catch (std::domain_error&) {
                        continue; // no intersection
                    }
                    uint minX = std::max(edgePoints[0], 0.0);
                    uint maxX = std::min(edgePoints[1], double(image.width - 1));

                    for (uint x = minX + goalEdgeBuffer; x <= maxX - goalEdgeBuffer; ++x) {
                        auto pixel = image(x, y);
                        if(pixel.y > goalLightnessMin && y < goalLightnessMax) {
                            pixels->pixels.push_back(pixel);
                        }
                    }
                }
            }

            emit(std::move(pixels));
        });

        fieldProvider = on<Trigger<ClassifiedImage<ObjectClass>>, Single, Priority::LOW>()
        .then("Auto Classifier Field", [this](const ClassifiedImage<ObjectClass>& classifiedImage) {

            auto pixels = std::make_unique<AutoClassifierPixels>();
            pixels->classification = Colour::GREEN;

            auto& image = *classifiedImage.image;

            for (uint x = fieldEdgeBuffer; x < classifiedImage.dimensions[0] - fieldEdgeBuffer; ++x) {

                for (uint y = classifiedImage.visualHorizonAtPoint(x) + fieldEdgeBuffer; y < classifiedImage.dimensions[1] - fieldEdgeBuffer; ++y) {
                    auto pixel = image(x, y);
                    if(pixel.y > fieldLightnessMin && y < fieldLightnessMax) {
                        pixels->pixels.push_back(pixel);
                    }
                }
            }

            emit(std::move(pixels));
        });
    }
}
}

