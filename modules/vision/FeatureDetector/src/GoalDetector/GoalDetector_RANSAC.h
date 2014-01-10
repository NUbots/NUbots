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

#ifndef MODULES_VISION_GOALDETECTOR_RANSAC_H
#define MODULES_VISION_GOALDETECTOR_RANSAC_H

#include <vector>
#include <list>
#include <armadillo>

#include "messages/vision/ClassifiedImage.h"
#include "messages/vision/VisionObjects.h"

#include "Goal.h"
#include "Quad.h"
#include "LSFittedLine.h"
#include "RANSAC/RANSAC.h"
#include "RANSAC/RANSACLine.h"

namespace modules {
	namespace vision {
	
		class GoalDetector_RANSAC {
		public:
			GoalDetector_RANSAC();
            std::unique_ptr<std::vector<Goal>> run(const VisionKinematics& visionKinematics, const std::vector<messages::vision::ColourSegment>& horizontalSegments, 
                                                    const std::vector<messages::vision::ColourSegment>& verticalSegments);

            // TODO: Add in Kinematics horizon.
            void setParameters(unsigned int MINIMUM_POINTS_,
                                unsigned int MAX_ITERATIONS_PER_FITTING_,
                                double CONSENSUS_THRESHOLD_,
                                unsigned int MAX_FITTING_ATTEMPTS_,
                                double ANGLE_MARGIN_, 
//								Horizon& kinematicsHorizon,
                                RANSAC_SELECTION_METHOD SELECTION_METHOD_,
                                double RANSAC_MATCHING_TOLERANCE_,
                                int MIN_GOAL_SEPARATION_,
                                float GOAL_HEIGHT_TO_WIDTH_RATIO_MIN_,
                                bool THROWOUT_SHORT_GOALS_, 
                                bool THROWOUT_NARROW_GOALS_, 
                                bool THROWOUT_ON_ABOVE_KIN_HOR_GOALS_, 
                                bool THROWOUT_DISTANT_GOALS_, 
                                float MAX_GOAL_DISTANCE_, 
                                int MIN_GOAL_HEIGHT_, 
                                int MIN_GOAL_WIDTH_, 
                                float GOAL_WIDTH_, 
                                const DISTANCE_METHOD& GOAL_DISTANCE_METHOD_,
                                int EDGE_OF_SCREEN_MARGIN_);

		private:
			std::list<Quad> buildQuadsFromLines(const std::vector<LSFittedLine>& start_lines, 
                                                const std::vector<LSFittedLine>& end_lines, 
                                                double tolerance);

			unsigned int getClosestUntriedLine(const LSFittedLine& start, const std::vector<LSFittedLine>& end_lines, std::vector<bool>& tried);

			std::vector<Goal> assignGoals(const std::list<Quad>& candidates, const Quad& crossbar) const;
            std::vector<Goal> assignGoals(const std::list<Quad>& candidates) const;

            void removeInvalid(std::list<Quad>& posts);
            void mergeClose(std::list<Quad>& posts, double widthMultipleToMerge);

			std::vector<arma::vec2> getEdgePointsFromSegments(const std::vector<messages::vision::ColourSegment> &segments);


			unsigned int MINIMUM_POINTS;							// Minimum points needed to make a line (Min pts to line essentially)
			unsigned int MAX_ITERATIONS_PER_FITTING;				// Number of iterations per fitting attempt
			unsigned int MAX_FITTING_ATTEMPTS;						// Hard limit on number of fitting attempts
			
			double ANGLE_MARGIN;									// Used for filtering out goal posts which are on too much of a lean.
			double CONSENSUS_THRESHOLD; 							// Threshold dtermining what constitutes a good fit (Consensus margin)
			
			RANSAC_SELECTION_METHOD m_RANSACMethod;
//			Horizon& m_kinematicsHorizon;
			
			double RANSAC_MATCHING_TOLERANCE;
            
            int MIN_GOAL_SEPARATION;
            float GOAL_HEIGHT_TO_WIDTH_RATIO_MIN;
            
            // Constants for construction of a Goal object.
            bool THROWOUT_SHORT_GOALS;
            bool THROWOUT_NARROW_GOALS;
            bool THROWOUT_ON_ABOVE_KIN_HOR_GOALS;
            bool THROWOUT_DISTANT_GOALS;
            float MAX_GOAL_DISTANCE;
            int MIN_GOAL_HEIGHT;
            int MIN_GOAL_WIDTH;
            float GOAL_WIDTH;
            DISTANCE_METHOD GOAL_DISTANCE_METHOD;
            int EDGE_OF_SCREEN_MARGIN;
		};
		
	}
}

#endif //  MODULES_VISION_GOALDETECTOR_RANSAC_H
