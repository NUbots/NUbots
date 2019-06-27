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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#include "Classifier.h"

#include "utility/support/eigen_armadillo.h"
#include "utility/vision/LookUpTable.h"
#include "utility/vision/Vision.h"

namespace module {
namespace vision {
    using message::input::Image;
    using message::vision::ClassifiedImage;
    using message::vision::LookUpTable;
    using SegmentClass = message::vision::ClassifiedImage::SegmentClass;
    using FOURCC       = utility::vision::FOURCC;

    std::vector<ClassifiedImage::Segment> Classifier::classify(const Image& image,
                                                               const LookUpTable& lut,
                                                               const arma::ivec2& start,
                                                               const arma::ivec2& end,
                                                               const uint& subsample) {

        utility::vision::Colour colour;
        uint16_t segmentLength = 0;
        uint i                 = 0;
        size_t length          = 0;

        std::vector<ClassifiedImage::Segment> output;
        output.reserve(64);
        arma::ivec2 position = start;


        // For vertical runs
        if (start[0] == end[0]) {
            length = end[1] - start[1] + 1;

            for (i = 0; i < length / subsample; ++i) {
                utility::vision::Colour currentColour =
                    utility::vision::getPixelColour(lut,
                                                    utility::vision::getPixel(start[0],
                                                                              start[1] + (i * subsample),
                                                                              image.dimensions[0],
                                                                              image.dimensions[1],
                                                                              image.data,
                                                                              static_cast<FOURCC>(image.format)));

                segmentLength += subsample;
                if (currentColour != colour) {
                    if (segmentLength > 0) {
                        // Update our position
                        arma::ivec2 s = position;
                        position[1] += segmentLength;
                        arma::ivec2 m = (s + position) / 2;

                        output.push_back(ClassifiedImage::Segment(classifySegment(colour),
                                                                  segmentLength,
                                                                  subsample,
                                                                  convert(s),
                                                                  convert(position),
                                                                  convert(m),
                                                                  -1,
                                                                  -1));
                    }

                    segmentLength = 0;
                    colour        = currentColour;
                }
            }
        }

        // For horizontal runs
        else if (start[1] == end[1]) {
            length = end[0] - start[0] + 1;
            for (i = 0; i < length / subsample; ++i) {
                utility::vision::Colour currentColour =
                    utility::vision::getPixelColour(lut,
                                                    utility::vision::getPixel(start[0] + (i * subsample),
                                                                              start[1],
                                                                              image.dimensions[0],
                                                                              image.dimensions[1],
                                                                              image.data,
                                                                              static_cast<FOURCC>(image.format)));

                segmentLength += subsample;
                if (currentColour != colour) {
                    if (segmentLength > 0) {
                        // Update our position
                        arma::ivec2 s = position;
                        position[0] += segmentLength;
                        arma::ivec2 m = (s + position) / 2;

                        output.push_back(ClassifiedImage::Segment(classifySegment(colour),
                                                                  segmentLength,
                                                                  subsample,
                                                                  convert(s),
                                                                  convert(position),
                                                                  convert(m),
                                                                  -1,
                                                                  -1));
                    }

                    segmentLength = 0;
                    colour        = currentColour;
                }
            }
        }

        // Update position for the final segment
        segmentLength = segmentLength + length % subsample;

        // Add final segment for vertical runs
        if (start[0] == end[0]) {

            // Update our position
            arma::ivec2 s = position;
            position[1] += segmentLength;
            arma::ivec2 m = (s + position) / 2;

            output.push_back(ClassifiedImage::Segment(
                classifySegment(colour), segmentLength, subsample, convert(s), convert(position), convert(m), -1, -1));
        }
        // Add final segment for horizontal runs
        else if (start[1] == end[1]) {
            // Update our position
            arma::ivec2 s = position;
            position[0] += segmentLength;
            arma::ivec2 m = (s + position) / 2;

            output.push_back(ClassifiedImage::Segment(
                classifySegment(colour), segmentLength, subsample, convert(s), convert(position), convert(m), -1, -1));
        }


        return output;
    }

    ClassifiedImage::SegmentClass Classifier::classifySegment(const utility::vision::Colour& colour) const {
        /*
         w+                  => QUEX_TKN_LINE(number=(LexemeL));
         g+                  => QUEX_TKN_FIELD(number=(LexemeL));
         o+                  => QUEX_TKN_BALL(number=(LexemeL));
         y+                  => QUEX_TKN_GOAL(number=(LexemeL));
         m+                  => QUEX_TKN_MAGENTA_TEAM(number=(LexemeL));
         c+                  => QUEX_TKN_CYAN_TEAM(number=(LexemeL));
         u+                  => QUEX_TKN_UNCLASSIFIED(number=(LexemeL));
         */
        switch (colour.value) {
            case utility::vision::Colour::GREEN: return ClassifiedImage::SegmentClass::FIELD;
            case utility::vision::Colour::ORANGE: return ClassifiedImage::SegmentClass::BALL;
            case utility::vision::Colour::YELLOW: return ClassifiedImage::SegmentClass::GOAL;
            case utility::vision::Colour::WHITE: return ClassifiedImage::SegmentClass::LINE;
            case utility::vision::Colour::CYAN: return ClassifiedImage::SegmentClass::CYAN_TEAM;
            case utility::vision::Colour::MAGENTA: return ClassifiedImage::SegmentClass::MAGENTA_TEAM;
            default: return ClassifiedImage::SegmentClass::UNKNOWN_CLASS;
        }
    }
}  // namespace vision
}  // namespace module
