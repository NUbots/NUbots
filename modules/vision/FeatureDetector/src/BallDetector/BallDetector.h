/*
 * This file is part of FeatureDetector.
 *
 * FeatureDetector is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * FeatureDetector is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with FeatureDetector.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_VISION_BALLDETECTOR_H
#define MODULES_VISION_BALLDETECTOR_H

#include <armadillo>
#include <list>
#include <vector>

#include "messages/vision/ClassifiedImage.h"

#include "Ball.h"

namespace modules {
    namespace vision {
        
        class BallDetector {
        public:
            BallDetector();
            virtual ~BallDetector();

            /*! @brief Detects a single ball from orange transitions using a geometric mean for general location
              and close classification at the pixel level combined with occlusion detection for refinement.
            */
            std::vector<Ball> run(const std::vector<messages::vision::ColourSegment>& horizontalMatchedSegments, 
                                    const std::vector<messages::vision::ColourSegment>& verticalMatchedSegments, 
                                    const std::vector<arma::vec2>& greenHorizonInterpolatedPoints);

            void setParameters(int BALL_EDGE_THRESHOLD_, 
                                int BALL_ORANGE_TOLERANCE_, 
                                float BALL_MIN_PERCENT_ORANGE_,
                                bool THROWOUT_ON_ABOVE_KIN_HOR_BALL_,
                                float MAX_DISTANCE_METHOD_DISCREPENCY_BALL_,
                                bool THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BALL_,
                                bool THROWOUT_SMALL_BALLS_,
                                float MIN_BALL_DIAMETER_PIXELS_,
                                bool THROWOUT_DISTANT_BALLS_,
                                float MAX_BALL_DISTANCE_,
                                float BALL_WIDTH_,
                                const DISTANCE_METHOD& BALL_DISTANCE_METHOD_,
                                const VisionKinematics& transformer);

        private:
            void appendEdgesFromSegments(const std::vector<messages::vision::ColourSegment>& segments, 
                                        std::list<arma::vec2>& pointList, 
                                        const std::vector<arma::vec2>& greenHorizon);
            
            int BALL_EDGE_THRESHOLD;
            int BALL_ORANGE_TOLERANCE;
            float BALL_MIN_PERCENT_ORANGE;

            // Constants for construction of a Ball object.
			bool THROWOUT_ON_ABOVE_KIN_HOR_BALL;
			float MAX_DISTANCE_METHOD_DISCREPENCY_BALL;
			bool THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BALL;
			bool THROWOUT_SMALL_BALLS;
			float MIN_BALL_DIAMETER_PIXELS;
			bool THROWOUT_DISTANT_BALLS;
			float MAX_BALL_DISTANCE;
			float BALL_WIDTH;
			DISTANCE_METHOD BALL_DISTANCE_METHOD;
			VisionKinematics m_transformer;
        };

    }
}
#endif // MODULES_VISION_BALLDETECTOR_H
