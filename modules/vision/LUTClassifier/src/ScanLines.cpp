/*
 * This file is part of ScanLines.
 *
 * ScanLines is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ScanLines is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ScanLines.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "ScanLines.h"

namespace modules {
    namespace vision {

        using messages::input::Image;
        using messages::support::Configuration;

        ScanLines::ScanLines(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
            on<Trigger<Configuration<ScanLines>>>([this](const Configuration<ScanLines>& constants) {
				HORIZONTAL_SCANLINE_SPACING = constants.config["HORIZONTAL_SCANLINE_SPACING"];
				VERTICAL_SCANLINE_SPACING = constants.config["VERTICAL_SCANLINE_SPACING"];
				VISION_SCANLINE_VERBOSITY =  constants.config["VISION_SCANLINE_VERBOSITY"];
            });

        }
		std::vector<int> ScanLines::generateScanLines(const Image& img, const std::vector<arma::vec>& greenHorizon) {
			std::vector<int> horizontalScanLines;
			const std::vector<arma::vec::fixed<2>>& horizonPoints = greenHorizon().getInterpolatedPoints();   // Need this to get the left and right

			arma::vec::fixed<2> left = horizonPoints.front();
			arma::vec::fixed<2> right = horizonPoints.back();

			if(left.y >= img.height())
				log<NUClear::WARN>("Left horizon limit exceeds image height: ", left.y)

			if(right.y >= img.height())
				log<NUClear::WARN>("Left horizon limit exceeds image height: ", right.y)

			/* Old code, commented out pre-NUClear
			unsigned int bottom_horizontal_scan = (left.y + right.y) / 2; 
			
			if(bottom_horizontal_scan >= img.height())
				errorlog << "avg: " << bottom_horizontal_scan << std::endl;
			*/

			int bottomHorizontalScan = img.height() - 1;    //we need h-scans under the GH for field lines

			for (int y = bottomHorizontalScan; y >= 0; y -= HORIZONTAL_SCANLINE_SPACING) {
				/* Old code, commented out pre-NUClear
				if(y >= img.height())
					errorlog << " y: " << y << std::endl;
				*/

				horizontalScanLines.push_back(y);
			}

			return horizontalScanLines;
		}

		std::vector<std::vector<ColourSegment>> ScanLines::classifyHorizontalScanLines(const Image& originalImage, const std::vector<int>& horizontalScanLines, const LookUpTable& LUT) {
			std::vector<std::vector<ColourSegment>> classifications;

		    for (auto scanLine : horizontalScanLines) {
				classifications.push_back(classifyHorizontalScan(LUT, originalImage, scanLine));
			}

			return classifications;
		}

		std::vector<std::vector<ColourSegment>> ScanLines::classifyVerticalScanLines(const Image& originalImage, const std::vector<arma::vec>& greenHorizon, const LookUpTable& LUT) {
			const std::vector<arma::vec>& verticalStartPoints = greenHorizon.getInterpolatedSubset(VERTICAL_SCANLINE_SPACING);
			std::vector<std::vector<ColourSegment>> classifications;

			for (auto startPoint : verticalStartPoints) {
				classifications.push_back(classifyVerticalScan(LUT, originalImage, startPoint));
			}
    		
			return classifications;
		}

        std::vector<ColourSegment> ScanLines::classifyHorizontalScan(const Image& image, unsigned int y, const LookUpTable& LUT) {
			std::vector<ColourSegment> result;

			if(y >= img.height()) {
				log<NUClear::ERROR>("ScanLines::classifyHorizontalScan invalid y: ", y)
				return result;
			}

			//simple and nasty first
			//Colour previous, current, next
			int startPosition = 0, x;
			Colour startColour = getColourFromIndex(lut.classifyPixel(img(0,y)));
			Colour currentColour;
			ColourSegment segment;

			for(x = 0; x < img.width(); x++) {
				currentColour = getColourFromIndex(lut.classifyPixel(img(x,y)));

				if(currentColour != startColour) {
					//start of new segment
					//make new segment and push onto std::vector
					segment.set(Point(startPosition, y), Point(x, y), startColour);
					result.push_back(segment);

					//start new segment
					startColour = currentColour;
					startPosition = x;
				}
			}

			segment.set(Point(startPosition, y), Point(x - 1, y), startColour);
			result.push_back(segment);

			return result;
		}

		std::vector<ColourSegment> ScanLines::classifyVerticalScan(const Image& image, const arma::vec& start, const LookUpTable& LUT) {
			std::vector<ColourSegment> result;

			if((start.y >= img.height()) || (start.y < 0) || (start.x >= img.width()) || (start.x < 0)) {
				log<NUClear::ERROR>("ScanLines::classifyVerticalScan invalid start position: ", start)
				return result;
			}

			//simple and nasty first
			//Colour previous, current, next
			Colour startColour = getColourFromIndex(lut.classifyPixel(img(start.x,start.y))), currentColour;
			ColourSegment segment;
			int startPosition = start.y, x = start.x, y;

			for(y = start.y; y < img.height(); y++) {
				currentColour = getColourFromIndex(lut.classifyPixel(img(x,y)));

				if(currentColour != startColour) {
					//start of new segment
					//make new segment and push onto std::vector
					segment.set(Point(x, startPosiiton), Point(x, y), startColour);
					result.push_back(segment);

					//start new segment
					startColour = currentColour;
					startPosition = y;
				}
			}

			segment.set(Point(x, startPosition), Point(x, y), startColour);
			result.push_back(segment);

			return result;
		}
	}
}