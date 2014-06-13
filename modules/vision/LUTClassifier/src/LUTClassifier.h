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

#include "GreenHorizon.h"
#include "ScanLines.h"
#include "SegmentFilter.h"
#include "ColourReplacementRule.h"
#include "ColourTransitionRule.h"

namespace modules {
    namespace vision {
        struct VisionConstants{
            static constexpr const char* CONFIGURATION_PATH = "VisionConstants.yaml";
        };

        struct LUTLocations{
            static constexpr const char* CONFIGURATION_PATH = "LUTLocations.yaml";
        };

        struct GreenHorizonConfig{
            static constexpr const char* CONFIGURATION_PATH = "GreenHorizon.yaml";
        };

        struct ScanLinesConfig{
            static constexpr const char* CONFIGURATION_PATH = "ScanLines.yaml";
        };

        struct RulesConfig{
            static constexpr const char* CONFIGURATION_PATH = "Rules.yaml";
        };

        /**
         * Classifies a raw image, producing the colour segments for object detection
         *
         * @author Jake Fountain
         */
        class LUTClassifier : public NUClear::Reactor {
        private:
            GreenHorizon greenHorizon;
            ScanLines scanLines;
            SegmentFilter segmentFilter;

        public:
            explicit LUTClassifier(std::unique_ptr<NUClear::Environment> environment);
        };

    }  // vision
}  // modules

#endif  // MODULES_VISION_LUTCLASSIFIER_H

