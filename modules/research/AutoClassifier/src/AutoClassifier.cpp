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

#include "AutoClassifier.h"
#include <armadillo>

#include "messages/vision/VisionObjects.h"
#include "messages/vision/ClassifiedImage.h"
#include "messages/support/Configuration.h"
#include "utility/math/geometry/ParametricLine.h"

namespace modules {
namespace research {

    using messages::vision::Ball;
    using messages::vision::Goal;
    using messages::vision::Colour;
    using messages::vision::LookUpTable;
    using messages::vision::ClassifiedImage;
    using messages::vision::ObjectClass;
    using messages::support::Configuration;
    using utility::math::geometry::ParametricLine;

    AutoClassifier::AutoClassifier(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Trigger<Configuration<AutoClassifier>>>([this] (const Configuration<AutoClassifier>& config) {
            auto& orange = config["colours"]["orange"];
            orangeClassifier.enable(orange["enabled"].as<bool>());
            orangeRange = orange["range"].as<double>();

            auto& yellow = config["colours"]["yellow"];
            yellowClassifier.enable(yellow["enabled"].as<bool>());
            yellowRange = yellow["range"].as<double>();

            auto& green = config["colours"]["green"];
            greenClassifier.enable(green["enabled"].as<bool>());
            greenRange = green["range"].as<double>();
        });

        orangeClassifier = on<Trigger<std::vector<Ball>>, With<LookUpTable>, Options<Single, Priority<NUClear::LOW>>>("Auto Classifier Balls", [this](
            const std::vector<Ball>& balls, const LookUpTable& lut) {

            // create a new lookup table
            auto newLutObj = std::make_unique<LookUpTable>(lut);
            auto& newLut = *newLutObj;

            if (!reference) {
                reference = std::make_unique<LookUpTable>(lut);
                cacheColours(*reference);
            }

            for (auto& ball : balls) {
                auto& image = *ball.classifiedImage->image;

                uint radius = ball.circle.radius;
                arma::vec2 centre = ball.circle.centre;

                // find the min and max y points on the circle
                // capped at the bounds of the image
                uint minY = std::max(std::round(centre[1] - radius), 0.0);
                uint maxY = std::min(std::round(centre[1] + radius), double(image.height() - 1));

                uint rangeSqr = std::pow(orangeRange, 2);

                // loop through pixels on the image in bounding box
                for (uint y = minY; y <= maxY; y++) {
                    // find the min and max x points on the circle for each given y
                    // uses the general equation of a circle and solves for x
                    // capped at the bounds of the image
                    double a = y - centre[1];
                    double b = std::sqrt(radius * radius - a * a);
                    uint minX = std::max(std::round(centre[0] - b), 0.0);
                    uint maxX = std::min(std::round(centre[0] + b), double(image.width() - 1));

                    for (uint x = minX; x <= maxX; x++) {
                        // get the pixel
                        auto& pixel = image(x, y);
                        // if pixel is unclassfied and close to 'orange' coloured, classify it
                        if (newLut(pixel) == Colour::UNCLASSIFIED) {
                            for (auto& matchedPixel : orangePixels) {
                                // find euclidean distance between the two pixels
                                uint distSqr = std::pow(pixel.y - matchedPixel.y, 2)
                                          + std::pow(pixel.cb - matchedPixel.cb, 2)
                                          + std::pow(pixel.cr - matchedPixel.cr, 2);
                                // check its within the given range
                                if (distSqr <= rangeSqr) {
                                    // classify!
                                    newLut(pixel) = Colour::ORANGE;
                                    break;
                                }
                            }
                        }
                    }
                }

            }

            emit(std::move(newLutObj));

        });

        yellowClassifier = on<Trigger<std::vector<Goal>>, With<LookUpTable>, Options<Single, Priority<NUClear::LOW>>>("Auto Classifier Goals", [this](
            const std::vector<Goal>& goals, const LookUpTable& lut) {

            // create a new lookup table
            auto newLutObj = std::make_unique<LookUpTable>(lut);
            auto& newLut = *newLutObj;

            if (!reference) {
                reference = std::make_unique<LookUpTable>(lut);
                cacheColours(*reference);
            }

            for (auto& goal : goals) {
                auto& image = *goal.classifiedImage->image;

                arma::vec2 topLeft = goal.quad.getTopLeft();
                arma::vec2 bottomLeft = goal.quad.getBottomLeft();
                arma::vec2 topRight = goal.quad.getTopRight();
                arma::vec2 bottomRight = goal.quad.getBottomRight();

                ParametricLine<2> topLine(topLeft, topRight, true);
                ParametricLine<2> rightLine(topRight, bottomRight, true);
                ParametricLine<2> bottomLine(bottomLeft, bottomRight, true);
                ParametricLine<2> leftLine(bottomLeft, topLeft, true);

                // find the min and max y points on the circle
                // capped at the bounds of the image
                uint minY = std::min(topLeft[1], topRight[1]);
                uint maxY = std::min(bottomLeft[1], bottomRight[1]);

                uint rangeSqr = std::pow(yellowRange, 2);

                // loop through pixels on the image in bounding box
                for (uint y = minY; y <= maxY; y++) {
                    // find the min and max x points on the circle for each given y
                    // uses the general equation of a circle and solves for x
                    // capped at the bounds of the image
                    ParametricLine<2> scanLine;
                    scanLine.setFromDirection({1, 0}, {0, double(y)});

                    std::vector<int> values;
                    std::vector<ParametricLine<2>> lines = {
                        topLine,
                        rightLine,
                        bottomLine,
                        leftLine
                    };

                    for (auto& line : lines) {
                        try {
                            values.push_back(scanLine.intersect(line)[0]);
                        } catch (std::domain_error&) { }
                    }

                    if (values.size() != 2) {
                        continue; // something went wrong!
                    }

                    uint minX = std::max(std::min(values[0], values[1]), 0);
                    uint maxX = std::min(std::max(values[0], values[1]), int(image.width() - 1));

                    for (uint x = minX; x <= maxX; x++) {
                        // get the pixel
                        auto& pixel = image(x, y);
                        // if pixel is unclassfied and close to 'yellow' coloured, classify it
                        if (newLut(pixel) == Colour::UNCLASSIFIED) {
                            for (auto& matchedPixel : yellowPixels) {
                                // find euclidean distance between the two pixels
                                uint distSqr = std::pow(pixel.y - matchedPixel.y, 2)
                                          + std::pow(pixel.cb - matchedPixel.cb, 2)
                                          + std::pow(pixel.cr - matchedPixel.cr, 2);
                                // check its within the given range
                                if (distSqr <= rangeSqr) {
                                    // classify!
                                    newLut(pixel) = Colour::YELLOW;
                                    break;
                                }
                            }
                        }
                    }
                }

            }

            emit(std::move(newLutObj));

        });

        greenClassifier = on<Trigger<ClassifiedImage<ObjectClass>>, With<LookUpTable>, Options<Single, Priority<NUClear::LOW>>>("Auto Classifier Field", [this](
            const ClassifiedImage<ObjectClass>& classifiedImage, const LookUpTable& lut) {

            // create a new lookup table
            auto newLutObj = std::make_unique<LookUpTable>(lut);
            auto& newLut = *newLutObj;

            if (!reference) {
                reference = std::make_unique<LookUpTable>(lut);
                cacheColours(*reference);
            }

            auto& image = *classifiedImage.image;

            uint rangeSqr = std::pow(greenRange, 2);

            for (uint x = 0; x < classifiedImage.dimensions[0]; x++) {
                for (uint y = classifiedImage.visualHorizonAtPoint(x); y < classifiedImage.dimensions[1]; y++) {
                    auto& pixel = image(x, y);
                    // if pixel is unclassfied and close to 'green' coloured, classify it
                    if (newLut(pixel) == Colour::UNCLASSIFIED) {
                        for (auto& matchedPixel : greenPixels) {
                            // find euclidean distance between the two pixels
                            uint distSqr = std::pow(pixel.y - matchedPixel.y, 2)
                                      + std::pow(pixel.cb - matchedPixel.cb, 2)
                                      + std::pow(pixel.cr - matchedPixel.cr, 2);
                            // check its within the given range
                            if (distSqr <= rangeSqr) {
                                // classify!
                                newLut(pixel) = Colour::GREEN;
                                break;
                            }
                        }
                    }
                }
            }

            emit(std::move(newLutObj));

        });

    }

    void AutoClassifier::cacheColours(const messages::vision::LookUpTable& lut) {
        uint i = 0;
        for (auto& colour : lut.getRawData()) {
            switch (colour) {
                case Colour::ORANGE: {
                    auto pixel = lut.getPixelFromIndex(i);
                    orangePixels.push_back(pixel);
                    break;
                }
                case Colour::YELLOW: {
                    auto pixel = lut.getPixelFromIndex(i);
                    yellowPixels.push_back(pixel);
                    break;
                }
                case Colour::GREEN: {
                    auto pixel = lut.getPixelFromIndex(i);
                    greenPixels.push_back(pixel);
                    break;
                }
                default:
                    break; // -Wswitch
            }
            i++;
        }
    }

}
}

