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
#include <armadillo>

#include "message/input/Image.h"
#include "message/input/Sensors.h"
#include "message/vision/ClassifiedImage.h"
#include "message/vision/LookUpTable.h"

namespace module {
    namespace vision {

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

            arma::fvec3 greenCentroid;

            int VISUAL_HORIZON_SPACING = 100;
            int VISUAL_HORIZON_BUFFER = 0;
            uint VISUAL_HORIZON_MINIMUM_SEGMENT_SIZE = 0;
            int VISUAL_HORIZON_SUBSAMPLING = 1;

            int GOAL_LINE_SPACING = 100;
            int GOAL_SUBSAMPLING = 1;
            uint GOAL_RANSAC_MINIMUM_POINTS_FOR_CONSENSUS = 10;
            uint GOAL_RANSAC_MAXIMUM_ITERATIONS_PER_FITTING = 30;
            uint GOAL_RANSAC_MAXIMUM_FITTED_MODELS = 6;
            uint GOAL_MINIMUM_RANSAC_SEGMENT_SIZE = 1;
            double GOAL_RANSAC_CONSENSUS_ERROR_THRESHOLD = 10;
            double GOAL_MAX_HORIZON_ANGLE = M_PI / 6;
            double GOAL_VERTICAL_EXTENSION_SCALE = 2.0;
            double GOAL_HORIZONTAL_EXTENSION_SCALE = 2.0;
            int GOAL_LINE_DENSITY = 2;
            double GOAL_WIDTH_HEIGHT_RATIO = 3;
            int GOAL_LINE_INTERSECTIONS = 30;

            double BALL_MINIMUM_INTERSECTIONS_COARSE = 1;
            double BALL_MINIMUM_INTERSECTIONS_FINE = 1;
            double BALL_SEARCH_CIRCLE_SCALE = 2;
            double BALL_MAXIMUM_VERTICAL_CLUSTER_SPACING = 1;
            double BALL_HORIZONTAL_SUBSAMPLE_FACTOR = 1;
            double BALL_RADIUS = 0.05;

            double FOCAL_LENGTH_PIXELS = 2.0;
            double ALPHA = 2.0;

            void insertSegments(message::vision::ClassifiedImage<message::vision::ObjectClass>& image
                , std::vector<message::vision::ClassifiedImage<message::vision::ObjectClass>::Segment>& segments
                , bool vertical);

            void findHorizon(const message::input::Image& image, const message::vision::LookUpTable& lut, message::vision::ClassifiedImage<message::vision::ObjectClass>& classifiedImage);

            void findVisualHorizon(const message::input::Image& image, const message::vision::LookUpTable& lut, message::vision::ClassifiedImage<message::vision::ObjectClass>& classifiedImage);

            void findBall(const message::input::Image& image, const message::vision::LookUpTable& lut, message::vision::ClassifiedImage<message::vision::ObjectClass>& classifiedImage);

            void findGoals(const message::input::Image& image, const message::vision::LookUpTable& lut, message::vision::ClassifiedImage<message::vision::ObjectClass>& classifiedImage);

            void enhanceBall(const message::input::Image& image, const message::vision::LookUpTable& lut, message::vision::ClassifiedImage<message::vision::ObjectClass>& classifiedImage);

            void enhanceGoals(const message::input::Image& image, const message::vision::LookUpTable& lut, message::vision::ClassifiedImage<message::vision::ObjectClass>& classifiedImage);

        public:
            explicit LUTClassifier(std::unique_ptr<NUClear::Environment> environment);
            ~LUTClassifier();
        };

    }  // vision
}  // modules

#endif  // MODULES_VISION_QUEXLUTCLASSIFIER_H

