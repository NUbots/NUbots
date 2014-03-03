/*
 * This file is part of LUTClassifier.
 *
 * LUTClassifier is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * LUTClassifier is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with LUTClassifier.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_VISION_LUTCLASSIFIER_H
#define MODULES_VISION_LUTCLASSIFIER_H

#include <nuclear> 
#include <string>
#include <armadillo>
#include <chrono>

#include "messages/input/Image.h"
#include "messages/vision/ClassifiedImage.h"
#include "messages/support/Configuration.h"

#include "utility/vision/LookUpTable.h"
#include "GreenHorizon.h"
#include "ScanLines.h"
#include "SegmentFilter.h"
#include "ColourReplacementRule.h"
#include "ColourTransitionRule.h"

namespace modules {
    namespace vision {
        struct VisionConstants{
            static constexpr const char* CONFIGURATION_PATH = "VisionConstants.json";
        };

        struct LUTLocations{
            static constexpr const char* CONFIGURATION_PATH = "LUTLocations.json";
        };

        struct GreenHorizonConfig{
            static constexpr const char* CONFIGURATION_PATH = "GreenHorizon.json";
        };

        struct ScanLinesConfig{
            static constexpr const char* CONFIGURATION_PATH = "ScanLines.json";
        };

        struct RulesConfig{
            static constexpr const char* CONFIGURATION_PATH = "Rules.json";
        };
        
        /**
         * Classifies a raw image, producing the colour segments for object detection
         *
         * @author Jake Fountain
         */
        class LUTClassifier : public NUClear::Reactor {
        private:
            std::vector< std::shared_ptr<utility::vision::LookUpTable> > LUTs;
            unsigned int currentLUTIndex;

            GreenHorizon greenHorizon;
            ScanLines scanLines;
            SegmentFilter segmentFilter;

        public:
            explicit LUTClassifier(std::unique_ptr<NUClear::Environment> environment);
        };
    
    }  // vision
}  // modules

#endif  // MODULES_VISION_LUTCLASSIFIER_H

