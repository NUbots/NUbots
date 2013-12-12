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

#include "messages/input/Image.h"
#include "messages/vision/ClassifiedImage.h"
#include "messages/support/Configuration.h"

#include "LookUpTable.h"
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
            //Vision constants
            //Should this be moved to be accessible to other classes? Perhaps partition.
            // int WHITE_SIDE_IS_BLUE;
            // bool NON_WHITE_SIDE_CHECK;
            // int UPPER_WHITE_THRESHOLD;
            // int LOWER_WHITE_THRESHOLD;
            // bool DO_RADIAL_CORRECTION;
            // float RADIAL_CORRECTION_COEFFICIENT;
            // bool THROWOUT_ON_ABOVE_KIN_HOR_GOALS;
            // bool THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_GOALS;
            // float MAX_DISTANCE_METHOD_DISCREPENCY_GOALS;
            // bool THROWOUT_DISTANT_GOALS;
            // float MAX_GOAL_DISTANCE;
            // bool THROWOUT_INSIGNIFICANT_GOALS;
            // int MIN_TRANSITIONS_FOR_SIGNIFICANCE_GOALS;
            // bool THROWOUT_NARROW_GOALS;
            // int MIN_GOAL_WIDTH;
            // bool THROWOUT_SHORT_GOALS;
            // int MIN_GOAL_HEIGHT;
            // float GOAL_HEIGHT_TO_WIDTH_RATIO_MIN;
            // int GOAL_MAX_OBJECTS;
            // int GOAL_BINS;
            // int GOAL_MIN_THRESHOLD;
            // float GOAL_SDEV_THRESHOLD;
            // float GOAL_RANSAC_MATCHING_TOLERANCE;
            // bool THROWOUT_ON_ABOVE_KIN_HOR_BALL;
            // bool THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BALL;
            // float MAX_DISTANCE_METHOD_DISCREPENCY_BALL;
            // bool THROWOUT_SMALL_BALLS;
            // float MIN_BALL_DIAMETER_PIXELS;
            // bool THROWOUT_INSIGNIFICANT_BALLS;
            // int MIN_TRANSITIONS_FOR_SIGNIFICANCE_BALL;
            // bool THROWOUT_DISTANT_BALLS;
            // float MAX_BALL_DISTANCE;
            // bool D2P_INCLUDE_BODY_PITCH;
            // bool BALL_DISTANCE_POSITION_BOTTOM;
            // std::string BALL_DISTANCE_METHOD;
            // std::string GOAL_DISTANCE_METHOD;
            // std::string LINE_METHOD;
            // int BALL_EDGE_THRESHOLD;
            // int BALL_ORANGE_TOLERANCE;
            // float BALL_MIN_PERCENT_ORANGE;
            // float GOAL_MIN_PERCENT_YELLOW;
            // float GOAL_MIN_PERCENT_BLUE;
            // int MIN_GOAL_SEPARATION;
            // int MIN_DISTANCE_FROM_HORIZON;
            // int MIN_CONSECUTIVE_POINTS;
            // float GOAL_WIDTH;
            // float GOAL_HEIGHT;
            // float DISTANCE_BETWEEN_POSTS;
            // float BALL_WIDTH;
            // float CENTRE_CIRCLE_RADIUS;
            // unsigned int SAM_MAX_LINES;
            // unsigned int SAM_MIN_POINTS_OVER;
            // unsigned int SAM_MIN_POINTS_TO_LINE;
            // unsigned int SAM_MIN_POINTS_TO_LINE_FINAL;
            // float SAM_SPLIT_DISTANCE;
            // float SAM_MAX_ANGLE_DIFF_TO_MERGE;
            // float SAM_MAX_DISTANCE_TO_MERGE;
            // float SAM_MIN_LINE_R2_FIT;
            // float SAM_MAX_LINE_MSD;
            // bool SAM_CLEAR_SMALL;
            // bool SAM_CLEAR_DIRTY;
            // float RANSAC_MAX_ANGLE_DIFF_TO_MERGE;
            // float RANSAC_MAX_DISTANCE_TO_MERGE;
            std::vector<LookUpTable> LUTs;
            unsigned int current_LUT_index;

            GreenHorizon greenHorizon;
            ScanLines scanLines;
            SegmentFilter segmentFilter;

        public:
            explicit LUTClassifier(std::unique_ptr<NUClear::Environment> environment);
        };
    
    }  // vision
}  // modules

#endif  // MODULES_VISION_LUTCLASSIFIER_H

