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

#include "messages/input/Image.h"
#include "messages/input/Sensors.h"
#include "messages/vision/ClassifiedImage.h"
#include "messages/vision/LookUpTable.h"

namespace modules {
    namespace vision {

        struct LUTLocation {
            static constexpr const char* CONFIGURATION_PATH = "LookUpTable.yaml";
        };

        class QuexClassifier;

        /**
         * Classifies a raw image, producing the colour segments for object detection
         *
         * @author Trent Houliston
         */
        class LUTClassifier : public NUClear::Reactor {
        private:
            // A pointer to our quex class (since it is generated it is not defined at this point)
            QuexClassifier* quex;

            int VISUAL_HORIZON_SPACING = 100;
            int VISUAL_HORIZON_BUFFER = 0;
            uint VISUAL_HORIZON_MINIMUM_SEGMENT_SIZE = 0;
            int VISUAL_HORIZON_SUBSAMPLING = 1;

            int GOAL_LINE_SPACING = 100;
            int GOAL_SUBSAMPLING = 1;
            int GOAL_MAXIMUM_VERTICAL_CLUSTER_SPACING = 1;
            int GOAL_VERTICAL_CLUSTER_UPPER_BUFFER = 1;
            int GOAL_VERTICAL_CLUSTER_LOWER_BUFFER = 1;
            double GOAL_VERTICAL_SD_JUMP = 1;
            double GOAL_EXTENSION_SCALE = 2.0;

            double BALL_MINIMUM_INTERSECTIONS_COARSE = 1;
            double BALL_MINIMUM_INTERSECTIONS_FINE = 1;
            double BALL_SEARCH_CIRCLE_SCALE = 2;
            double BALL_MAXIMUM_VERTICAL_CLUSTER_SPACING = 1;
            double BALL_HORIZONTAL_SUBSAMPLE_FACTOR = 1;
            double BALL_RADIUS = 0.05;

            double FOCAL_LENGTH_PIXELS = 2.0;
            double ALPHA = 2.0;

            void insertSegments(messages::vision::ClassifiedImage<messages::vision::ObjectClass>& image
                , std::vector<messages::vision::ClassifiedImage<messages::vision::ObjectClass>::Segment>& segments
                , bool vertical);

            void findHorizon(const messages::input::Image& image, const messages::vision::LookUpTable& lut, messages::vision::ClassifiedImage<messages::vision::ObjectClass>& classifiedImage);

            void findVisualHorizon(const messages::input::Image& image, const messages::vision::LookUpTable& lut, messages::vision::ClassifiedImage<messages::vision::ObjectClass>& classifiedImage);

            void findBall(const messages::input::Image& image, const messages::vision::LookUpTable& lut, messages::vision::ClassifiedImage<messages::vision::ObjectClass>& classifiedImage);

            void findGoals(const messages::input::Image& image, const messages::vision::LookUpTable& lut, messages::vision::ClassifiedImage<messages::vision::ObjectClass>& classifiedImage);

            void enhanceBall(const messages::input::Image& image, const messages::vision::LookUpTable& lut, messages::vision::ClassifiedImage<messages::vision::ObjectClass>& classifiedImage);

            void enhanceGoals(const messages::input::Image& image, const messages::vision::LookUpTable& lut, messages::vision::ClassifiedImage<messages::vision::ObjectClass>& classifiedImage);

            void findGoalBases(const messages::input::Image& image, const messages::vision::LookUpTable& lut, messages::vision::ClassifiedImage<messages::vision::ObjectClass>& classifiedImage);

        public:
            static constexpr const char* CONFIGURATION_PATH = "LUTClassifier.yaml";

            explicit LUTClassifier(std::unique_ptr<NUClear::Environment> environment);
            ~LUTClassifier();
        };

    }  // vision
}  // modules

#endif  // MODULES_VISION_QUEXLUTCLASSIFIER_H

