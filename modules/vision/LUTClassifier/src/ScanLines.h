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

#ifndef MODULES_VISION_SCANLINES_H
#define MODULES_VISION_SCANLINES_H

#include <nuclear> 
#include <string>
#include <armadillo>

#include "messages/input/Image.h"
#include "messages/support/Configuration.h"
<<<<<<< HEAD
#include "ColourSegment.h"
=======
>>>>>>> c55d84a304dc07d8af68bc9ca4a939f84df1f47d
#include "LookUpTable.h"

namespace modules {
    namespace vision {

        /**
         * Generates horizontal and vertical scan lines.
         *
         * @author Alex Biddulph
         */
        class ScanLines : public NUClear::Reactor {
        private:
            unsigned int HORIZONTAL_SCANLINE_SPACING;
            unsigned int VERTICAL_SCANLINE_SPACING;

            setParameters( unsigned int HORIZONTAL_SCANLINE_SPACING_,
                           unsigned int VERTICAL_SCANLINE_SPACING_){
                HORIZONTAL_SCANLINE_SPACING = HORIZONTAL_SCANLINE_SPACING_;
                VERTICAL_SCANLINE_SPACING = VERTICAL_SCANLINE_SPACING_;
            }
            /*! @brief Returns a std::vector of ColourSegments detailing the
            horizontal colour segments in the image.
            */
            std::vector<ColourSegment> classifyHorizontalScan(const messages::input::Image& image, unsigned int y, const LookUpTable& LUT);

            /*! @brief Returns a std::vector of ColourSegments detailing the
            vertical colour segments in the image.
            */
            std::vector<ColourSegment> classifyVerticalScan(const messages::input::Image& image, const arma::vec& start, const LookUpTable& LUT);

        public:
            /*! @brief Loads configuration file.
            */ 
            ScanLines(std::unique_ptr<NUClear::Environment> environment);

            /*! @brief Generates the scan lines
            */ 
            std::vector<int> generateScanLines(const messages::input::Image& image, const std::vector<arma::vec>& greenHorizonPoints);

            /*! @brief Returns a std::vector of ColourSegments relating classified 
            horizontal scan lines.
            */
            std::vector<std::vector<ColourSegment>> classifyHorizontalScanLines(const messages::input::Image& originalImage, const std::vector<int>& horizontalScanLines, const LookUpTable& LUT);

            /*! @brief Returns a std::vector of ColourSegments relating to classified 
            vertical scan lines.
            */
            std::vector<std::vector<ColourSegment>> classifyVerticalScanLines(const messages::input::Image& originalImage, const std::vector<arma::vec>& greenHorizon, const LookUpTable& LUT);

            static constexpr const char* CONFIGURATION_PATH = "ScanLines.json";
        };
    
    }  // vision
}  // modules

#endif  // MODULES_VISION_SCANLINES_H

