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

    using messages::input::Image;
    using messages::vision::Ball;
    using messages::vision::Goal;
    using messages::vision::Colour;
    using messages::vision::LookUpTable;
    using messages::vision::ClassifiedImage;
    using messages::vision::ObjectClass;
    using messages::vision::proto::LookUpTableDiff;
    using messages::support::Configuration;
    using utility::math::geometry::ParametricLine;

    AutoClassifier::AutoClassifier(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Trigger<Configuration<AutoClassifier>>>([this] (const Configuration<AutoClassifier>& config) {
            auto& orange = config["colours"]["orange"];
            orangeData.enabled = orange["enabled"].as<bool>();
            orangeData.range = orange["range"].as<double>();

            auto& yellow = config["colours"]["yellow"];
            yellowData.enabled = yellow["enabled"].as<bool>();
            yellowData.range = yellow["range"].as<double>();

            auto& green = config["colours"]["green"];
            greenData.enabled = green["enabled"].as<bool>();
            greenData.range = green["range"].as<double>();

            auto& white = config["colours"]["white"];
            whiteData.enabled = white["enabled"].as<bool>();
            whiteData.range = white["range"].as<double>();

            ballClassifier.enable(orangeData.enabled);
            goalClassifier.enable(yellowData.enabled);
            fieldClassifier.enable(greenData.enabled || whiteData.enabled);
        });

        ballClassifier = on<Trigger<std::vector<Ball>>, With<LookUpTable>, Options<Single, Priority<NUClear::LOW>>>("Auto Classifier Balls", [this](
            const std::vector<Ball>& balls, const LookUpTable& lut) {

            // create a new lookup table
            auto newLutObj = std::make_unique<LookUpTable>(lut);
            auto& newLut = *newLutObj;

            if (!reference) {
                reference = std::make_unique<LookUpTable>(lut);
                cacheColours(*reference);
            }

            uint rangeSqr = std::pow(orangeData.range, 2);

            auto tableDiff = std::make_unique<LookUpTableDiff>();

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
                for (uint y = minY; y <= maxY; y++) {
                    auto edgePoints = circle.getEdgePoints(y);
                    uint minX = std::max(edgePoints[0], 0.0);
                    uint maxX = std::min(edgePoints[1], double(image.width() - 1));

                    for (uint x = minX; x <= maxX; x++) {
                        classifyNear(x, y, image, newLut, orangeData.pixels, Colour::ORANGE, rangeSqr, *tableDiff);
                    }
                }

            }

            if (tableDiff->diff_size() > 0) {
                emit(std::move(newLutObj));
                emit(std::move(tableDiff));
            }

        });

        goalClassifier = on<Trigger<std::vector<Goal>>, With<LookUpTable>, Options<Single, Priority<NUClear::LOW>>>("Auto Classifier Goals", [this](
            const std::vector<Goal>& goals, const LookUpTable& lut) {

            // create a new lookup table
            auto newLutObj = std::make_unique<LookUpTable>(lut);
            auto& newLut = *newLutObj;

            if (!reference) {
                reference = std::make_unique<LookUpTable>(lut);
                cacheColours(*reference);
            }

            uint rangeSqr = std::pow(yellowData.range, 2);

            auto tableDiff = std::make_unique<LookUpTableDiff>();

            for (auto& goal : goals) {
                auto& image = *goal.classifiedImage->image;
                auto& quad = goal.quad;

                // find the min and max y points on the quad
                // capped at the bounds of the image
                uint minY = std::max(std::min(quad.getTopLeft()[1], quad.getTopRight()[1]), 0.0);
                uint maxY = std::min(std::max(quad.getBottomLeft()[1], quad.getBottomRight()[1]), double(image.height() - 1));

                for (uint y = minY; y <= maxY; y++) {
                    arma::vec2 edgePoints;
                    try {
                        edgePoints = quad.getEdgePoints(y);
                    } catch (std::domain_error&) {
                        continue; // no intersection
                    }
                    uint minX = std::max(edgePoints[0], 0.0);
                    uint maxX = std::min(edgePoints[1], double(image.width() - 1));

                    for (uint x = minX; x <= maxX; x++) {
                        classifyNear(x, y, image, newLut, yellowData.pixels, Colour::YELLOW, rangeSqr, *tableDiff);
                    }
                }

            }

            if (tableDiff->diff_size() > 0) {
                emit(std::move(newLutObj));
                emit(std::move(tableDiff));
            }

        });

        fieldClassifier = on<Trigger<ClassifiedImage<ObjectClass>>, With<LookUpTable>, Options<Single, Priority<NUClear::LOW>>>("Auto Classifier Field", [this](
            const ClassifiedImage<ObjectClass>& classifiedImage, const LookUpTable& lut) {

            // create a new lookup table
            auto newLutObj = std::make_unique<LookUpTable>(lut);
            auto& newLut = *newLutObj;

            if (!reference) {
                reference = std::make_unique<LookUpTable>(lut);
                cacheColours(*reference);
            }

            auto& image = *classifiedImage.image;

            uint greenRangeSqr = std::pow(greenData.range, 2);
            uint whiteRangeSqr = std::pow(whiteData.range, 2);

            auto tableDiff = std::make_unique<LookUpTableDiff>();

            for (uint x = 0; x < classifiedImage.dimensions[0]; x++) {
                for (uint y = classifiedImage.visualHorizonAtPoint(x); y < classifiedImage.dimensions[1]; y++) {
                    if (greenData.enabled) {
                        classifyNear(x, y, image, newLut, greenData.pixels, Colour::GREEN, greenRangeSqr, *tableDiff);
                    }
                    if (whiteData.enabled) {
                        classifyNear(x, y, image, newLut, whiteData.pixels, Colour::WHITE, whiteRangeSqr, *tableDiff);
                    }
                }
            }

            if (tableDiff->diff_size() > 0) {
                emit(std::move(newLutObj));
                emit(std::move(tableDiff));
            }

        });

    }

    void AutoClassifier::classifyNear(
        const uint x,
        const uint y,
        const Image& image,
        LookUpTable& lut,
        const std::vector<Image::Pixel>& pixels,
        const Colour& colour,
        const double rangeSqr,
        LookUpTableDiff& tableDiff
    ) {
        auto& pixel = image(x, y);
        // if pixel is unclassfied and close to 'green' coloured, classify it
        if (lut(pixel) == Colour::UNCLASSIFIED) {
            for (auto& matchedPixel : pixels) {
                // find euclidean distance between the two pixels
                uint distSqr = std::pow(pixel.y - matchedPixel.y, 2)
                          + std::pow(pixel.cb - matchedPixel.cb, 2)
                          + std::pow(pixel.cr - matchedPixel.cr, 2);
                // check its within the given range
                if (distSqr <= rangeSqr) {
                    // classify!
                    lut(pixel) = colour;
                    auto& diff = *tableDiff.add_diff();
                    diff.set_lut_index(lut.getLUTIndex(pixel));
                    diff.set_classification(colour);
                    break;
                }
            }
        }
    }

    void AutoClassifier::cacheColours(const messages::vision::LookUpTable& lut) {
        uint i = 0;
        for (auto& colour : lut.getRawData()) {
            switch (colour) {
                case Colour::ORANGE: {
                    orangeData.pixels.push_back(lut.getPixelFromIndex(i));
                    break;
                }
                case Colour::YELLOW: {
                    yellowData.pixels.push_back(lut.getPixelFromIndex(i));
                    break;
                }
                case Colour::GREEN: {
                    greenData.pixels.push_back(lut.getPixelFromIndex(i));
                    break;
                }
                case Colour::WHITE: {
                    whiteData.pixels.push_back(lut.getPixelFromIndex(i));
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

