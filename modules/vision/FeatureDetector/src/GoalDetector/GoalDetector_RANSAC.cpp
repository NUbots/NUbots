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

#include "GoalDetector_RANSAC.h"

namespace modules {
	namespace vision {

		using messages::vision::ColourSegment;
		
		GoalDetector_RANSAC::GoalDetector_RANSAC() {
			// Empty constructor.
		}

        // TODO: Add in Kinematics horizon.
		void GoalDetector_RANSAC::setParameters(unsigned int MINIMUM_POINTS_,
												unsigned int MAX_ITERATIONS_PER_FITTING_,
												double CONSENSUS_THRESHOLD_,
												unsigned int MAX_FITTING_ATTEMPTS_,
												double ANGLE_MARGIN_, 
//												Horizon& kinematicsHorizon,
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
                                                int EDGE_OF_SCREEN_MARGIN_) {
			MINIMUM_POINTS = MINIMUM_POINTS_;  									// Minimum points needed to make a line (Min pts to line essentially)
																					// Original value: 25 / VisionConstants::HORIZONTAL_SCANLINE_SPACING;
			MAX_ITERATIONS_PER_FITTING = MAX_ITERATIONS_PER_FITTING_;               // Number of iterations per fitting attempt
			CONSENSUS_THRESHOLD = CONSENSUS_THRESHOLD_;     						// Threshold dtermining what constitutes a good fit (Consensus margin)
			MAX_FITTING_ATTEMPTS = MAX_FITTING_ATTEMPTS_;       					// Hard limit on number of fitting attempts
			
		    ANGLE_MARGIN = ANGLE_MARGIN_;											// Used for filtering out goal posts which are on too much of a lean.
		    
//		    m_kinematicsHorizon = kinematicsHorizon;								// = vbb->getKinematicsHorizon();
		    SELECTION_METHOD = SELECTION_METHOD_;
		    
			RANSAC_MATCHING_TOLERANCE = RANSAC_MATCHING_TOLERANCE_;

            MIN_GOAL_SEPARATION = MIN_GOAL_SEPARATION_;
            GOAL_HEIGHT_TO_WIDTH_RATIO_MIN = GOAL_HEIGHT_TO_WIDTH_RATIO_MIN_,
                                                    
            // Cosntants for constructing a Goal object.
            THROWOUT_SHORT_GOALS = THROWOUT_SHORT_GOALS_;
            THROWOUT_NARROW_GOALS = THROWOUT_NARROW_GOALS_;
            THROWOUT_ON_ABOVE_KIN_HOR_GOALS = THROWOUT_ON_ABOVE_KIN_HOR_GOALS_;
            THROWOUT_DISTANT_GOALS = THROWOUT_DISTANT_GOALS_;
            MAX_GOAL_DISTANCE = MAX_GOAL_DISTANCE_;
            MIN_GOAL_HEIGHT = MIN_GOAL_HEIGHT_;
            MIN_GOAL_WIDTH = MIN_GOAL_WIDTH_;
            GOAL_WIDTH = GOAL_WIDTH_;
            GOAL_DISTANCE_METHOD = GOAL_DISTANCE_METHOD_;
            EDGE_OF_SCREEN_MARGIN = EDGE_OF_SCREEN_MARGIN_; 
		}
	
		// std::vector<ColourSegment> verticalSegments = ClassifiedImage::matched_vertical_segments[GOAL_COLOUR];
		// std::vector<ColourSegment> horizontalSegments = ClassifiedImage::matched_horizontal_segments[GOAL_COLOUR];
		std::unique_ptr<std::vector<messages::vision::Goal>> GoalDetector_RANSAC::run(const VisionKinematics& visionKinematics, const std::vector<ColourSegment>& horizontalSegments, 
                                                                    const std::vector<ColourSegment>& verticalSegments) {
			std::list<Quad> quads, postCandidates;
			std::pair<bool, Quad> crossbar(false, Quad());
			std::unique_ptr<std::vector<Goal>> posts = std::unique_ptr<std::vector<Goal>>(new std::vector<Goal>);

			std::vector<arma::vec2> startPoints, endPoints;
			std::vector<std::pair<RANSACLine<arma::vec2>, std::vector<arma::vec2>>> ransacResults;
			std::vector<LSFittedLine> startLines, endLines;

			// Get edge points.
			for (const ColourSegment& segment : horizontalSegments) {
				startPoints.push_back(segment.m_start);
				endPoints.push_back(segment.m_end);
			}

			// Use generic RANSAC implementation to find start lines (left edges).
			ransacResults = RANSAC::findMultipleModels<RANSACLine<arma::vec2>, arma::vec2>(startPoints, 
                                                                                            CONSENSUS_THRESHOLD, 
                                                                                            MINIMUM_POINTS, 
                                                                                            MAX_ITERATIONS_PER_FITTING,
                                                                                            MAX_FITTING_ATTEMPTS, 
                                                                                            SELECTION_METHOD);
		
			for (auto& l : ransacResults) {
				startLines.push_back(LSFittedLine(l.second));
			}

			// Use generic RANSAC implementation to find end lines (right edges).
			ransacResults = RANSAC::findMultipleModels<RANSACLine<arma::vec2>, arma::vec2>(endPoints, 
                                                                                            CONSENSUS_THRESHOLD, 
                                                                                            MINIMUM_POINTS, 
                                                                                            MAX_ITERATIONS_PER_FITTING,
                                                                                            MAX_FITTING_ATTEMPTS, 
                                                                                            SELECTION_METHOD);
		
			for (auto& l : ransacResults) {
				endLines.push_back(LSFittedLine(l.second));
			}

			/************************
			 *      DEBUG POINT	    *
			 ************************
			 * startLines			*
			 * endLines			    *
			 ************************/

			// Build candidates out of lines - this finds candidates irrespective of rotation - filtering must be done later.
			quads = buildQuadsFromLines(startLines, endLines, RANSAC_MATCHING_TOLERANCE);

			// Remove posts with invalid aspect ratio : check potential cross bars AND posts.
			removeInvalid(quads);

			// Sort out potential crossbars and vertical posts (posts on too large of a lean will be removed).
			// Edit ANGLE_MARGIN to affect this.
			/*double halfPI = arma::math::pi() * 0.5;
			double lowerAngleThreshold = (ANGLE_MARGIN * halfPI);
			double upperAngleThreshold = halfPI - lowerAngleThreshold;*/
			
			for (const Quad& quad : quads) {

				//TODO!!!!!!!!!!!!!!!!!!!!!!!!!!! - add in filtering with kinematics horizon
/*				double angle = m_kinematicsHorizon.getAngleBetween(Line(quad.getTopCentre(), quad.getBottomCentre()));

				if (angle >= upperAngleThreshold) {*/
				    postCandidates.push_back(quad);
			/*	}
				
				else if (angle <= lowerAngleThreshold) {
				    // Only keep largest crossbar candidate.
				    if (!crossbar.first) {
				        crossbar.first = true;
				        crossbar.second = quad;
				    }
				    
				    else if (crossbar.second.area() < quad.area()) {
				        crossbar.second = quad;
				    }
				}*/
			}

			// Only check upright posts for building candidates.
			mergeClose(postCandidates, 1.5);

			// Generate actual goal from candidate posts.
			if (crossbar.first) {
				// If a cross bar has been found use it to help assign left and right.
				posts = assignGoals(visionKinematics, postCandidates, crossbar.second);
			}
		
			else {
				// No crossbar, just use general method.
				posts = assignGoals(visionKinematics, postCandidates);
			}


			// Improves bottom centre estimate using vertical transitions.
			for (const ColourSegment& segment : verticalSegments) {
				const arma::vec2& point = segment.m_end;
				
				for (Goal& post : *posts) {
				    if ((point[0] <= post.getQuad().getRight()) && 
                            (point[0] >= post.getQuad().getLeft()) && 
                            (point[1] > post.getLocationPixels()[1])) {
				        post.setBase(visionKinematics, point);
				    }
				}
			}

			return std::move(createGoalMessage(posts));
		}

		std::list<Quad> GoalDetector_RANSAC::buildQuadsFromLines(const std::vector<LSFittedLine>& startLines, 
                                                                    const std::vector<LSFittedLine>& endLines, double tolerance) {
			// (must match exactly) 0 <= tolerance <= 1 (any pair will be accepted)
			//
			// LSFittedLine objects contain std::lists of points and can be quite large,
			// therefore it is more efficient to pass by const reference and maintain
			// a std::list of matched end lines than to pass by copy so that the end lines
			// std::list can be shrunk.

			if ((tolerance < 0) || (tolerance > 1)) {
				std::cout << "GoalDetector_RANSAC::buildQuadsFromLines - tolerance must be in [0, 1]" << std::endl;
				tolerance = 1;					// TODO: Pick a better action here? We used to throw.
			}

			std::list<Quad> quads;
			std::vector<bool> used(endLines.size(), false);

			for (const LSFittedLine& startLine : startLines) {
				std::vector<bool> tried(used);						// Consider all used lines tried.
				bool matched = false;

				// Try end lines in order of closeness.
				for (unsigned int i = getClosestUntriedLine(startLine, endLines, tried);
										((i < endLines.size()) && (!matched));
										i = getClosestUntriedLine(startLine, endLines, tried)) {

				    const LSFittedLine& endLine = endLines.at(i);

				    // Check angles.
				    if (startLine.getAngleBetween(endLine) <= (tolerance * arma::math::pi() * 0.5)) {					// Dodgy way (linear with angle between).
				    //if(std::min(a1/a2, a2/a1) <= (1 - tolerance)) {
				        // Get the end points of each line.
				        arma::vec2 sp1, sp2, ep1, ep2;
				        
				        if (startLine.getEndPoints(sp1, sp2) && endLine.getEndPoints(ep1, ep2)) {
				            // Find lengths of line segments.
				            double l1 = arma::norm((sp1 - sp2), 2);
				            double l2 = arma::norm((ep1 - ep2), 2);
				            
				            // Check lengths.
				            if (std::min((l1 / l2), (l2 / l1)) >= (1 - tolerance)) {
				                // Get num points.
				                double n1 = startLine.getNumPoints(), n2 = endLine.getNumPoints();
				                
				                if (std::min((n1 / n2), (n2 / n1)) >= (1 - tolerance)) {
				                    // Check start is to left of end.
				                    if ((0.5 * (sp1[0] + sp2[0])) < (0.5 * (ep1[0] + ep2[0]))) {
				                        // Success
				                        // Order points
				                        if (sp1[1] < sp2[1]) {
				                            arma::vec2 c = sp1; 
				                            sp1 = sp2; 
				                            sp2 = c;
				                        }
				                        
				                        if (ep1[1] < ep2[1]) {
				                            arma::vec2 c = ep1; 
				                            ep1 = ep2; 
				                            ep2 = c;
				                        }
				                        
				                        quads.push_back(Quad(sp1, sp2, ep2, ep1));						// Generate candidate.
				                        used.at(i) = true;												// Remove end line from consideration.
				                        matched = true;
				                    }
				                }
				            }
				        }
				    }
				}			
			}

			return quads;
		}

		unsigned int GoalDetector_RANSAC::getClosestUntriedLine(const LSFittedLine& start,
                                                                const std::vector<LSFittedLine>& endLines, 
                                                                std::vector<bool>& tried) {
			if (endLines.size() != tried.size()) {
				std::cout << "GoalDetector_RANSAC::getClosestUntriedLine - 'endLines' must match 'tried' in size" << std::endl;
				return 0;					// TODO: Pick a better action here? We used to throw.
			}

			unsigned int best = endLines.size();											// For if all have been tried.
			double d_best = std::numeric_limits<double>::max();

			for (unsigned int i = 0; i < endLines.size(); i++) {
				// Check if tried yet.
				if (!tried[i]) {
				    // Check if distance is smallest.
				    double d = start.averageDistanceBetween(endLines[i]);
				    
				    if (d < d_best) {
				        best = i;
				        d_best = d;
				    }
				}
			}

			// Mark as used.
			if (best < endLines.size())
				tried[best] = true;

			return best;
		}

		std::unique_ptr<std::vector<Goal> > GoalDetector_RANSAC::assignGoals(const VisionKinematics& visionKinematics, const std::list<Quad>& candidates, const Quad& crossbar) const {
			if (candidates.size() == 1) {
                Goal goal(visionKinematics);
                
				if (crossbar.getCentre()[0] < candidates.front().getCentre()[0]) {
                    goal = Goal(visionKinematics, messages::vision::Goal::Type::RIGHT, candidates.front());
				}
				
				else {
                    goal = Goal(visionKinematics, messages::vision::Goal::Type::LEFT, candidates.front());
				}

                /*goal.setParameters(THROWOUT_SHORT_GOALS, 
                                   THROWOUT_NARROW_GOALS, 
                                   THROWOUT_ON_ABOVE_KIN_HOR_GOALS, 
                                   THROWOUT_DISTANT_GOALS, 
                                   MAX_GOAL_DISTANCE, 
                                   MIN_GOAL_HEIGHT, 
                                   MIN_GOAL_WIDTH, 
                                   GOAL_WIDTH, 
                                   GOAL_DISTANCE_METHOD,
                                   EDGE_OF_SCREEN_MARGIN);*/
                
                return std::move(  std::unique_ptr<std::vector<Goal>>(  new std::vector<Goal>(1, goal)  )  );
			}
			
            return assignGoals(visionKinematics, candidates);
		}

        std::unique_ptr<std::vector<Goal> > GoalDetector_RANSAC::assignGoals(const VisionKinematics& visionKinematics, const std::list<Quad>& candidates) const {

            std::unique_ptr<std::vector<Goal> > goals = std::unique_ptr<std::vector<Goal>>(new std::vector<Goal>());

            if (candidates.size() == 2) {
                Goal leftPost(visionKinematics), rightPost(visionKinematics);

                //there are exactly two candidates, identify each as left or right
                Quad post1 = candidates.front();
                Quad post2 = candidates.back();

                //calculate separation between candidates
                double pos1 = std::min(post1.getRight(), post2.getRight());      // inside right
                double pos2 = std::max(post1.getLeft(), post2.getLeft());  // inside left

                //only publish if the candidates are far enough apart
                if (std::abs(pos2 - pos1) >= MIN_GOAL_SEPARATION) {
                    //flip if necessary
                    if (post1.getCentre()[0] > post2.getCentre()[0]) {
                        leftPost = Goal(visionKinematics, messages::vision::Goal::Type::LEFT, post2);
                        rightPost = Goal(visionKinematics, messages::vision::Goal::Type::RIGHT, post1);
                    }

                    else {
                        leftPost = Goal(visionKinematics, messages::vision::Goal::Type::LEFT, post1);
                        rightPost = Goal(visionKinematics, messages::vision::Goal::Type::RIGHT, post2);
                    }

                   /* Parameters should be elsewhere 

                   leftPost.setParameters(THROWOUT_SHORT_GOALS, 
                                           THROWOUT_NARROW_GOALS, 
                                           THROWOUT_ON_ABOVE_KIN_HOR_GOALS, 
                                           THROWOUT_DISTANT_GOALS, 
                                           MAX_GOAL_DISTANCE, 
                                           MIN_GOAL_HEIGHT, 
                                           MIN_GOAL_WIDTH, 
                                           GOAL_WIDTH, 
                                           GOAL_DISTANCE_METHOD,
                                           EDGE_OF_SCREEN_MARGIN);
                    rightPost.setParameters(THROWOUT_SHORT_GOALS, 
                                            THROWOUT_NARROW_GOALS, 
                                            THROWOUT_ON_ABOVE_KIN_HOR_GOALS, 
                                            THROWOUT_DISTANT_GOALS, 
                                            MAX_GOAL_DISTANCE, 
                                            MIN_GOAL_HEIGHT, 
                                            MIN_GOAL_WIDTH, 
                                            GOAL_WIDTH, 
                                            GOAL_DISTANCE_METHOD,
                                            EDGE_OF_SCREEN_MARGIN);*/
                    goals->push_back(leftPost);
                    goals->push_back(rightPost);
                }

                else {
                    //should merge
                }
            }

            else {
                //unable to identify which post is which
                //setting all to unknown
                for(const Quad& candidate : candidates) {
                    Goal goal = Goal(visionKinematics, messages::vision::Goal::Type::UNKNOWN, candidate);
                    /*goal.setParameters(THROWOUT_SHORT_GOALS, 
                                        THROWOUT_NARROW_GOALS, 
                                        THROWOUT_ON_ABOVE_KIN_HOR_GOALS, 
                                        THROWOUT_DISTANT_GOALS, 
                                        MAX_GOAL_DISTANCE, 
                                        MIN_GOAL_HEIGHT, 
                                        MIN_GOAL_WIDTH, 
                                        GOAL_WIDTH, 
                                        GOAL_DISTANCE_METHOD,
                                        EDGE_OF_SCREEN_MARGIN);*/
                    goals->push_back(goal);
                }
            }

            return std::move(goals);
        }

		std::vector<arma::vec2> GoalDetector_RANSAC::getEdgePointsFromSegments(const std::vector<ColourSegment> &segments) {
			std::vector<arma::vec2> points;
			
			for (const ColourSegment& segment : segments) {
				points.push_back(segment.m_start);
				points.push_back(segment.m_end);
			}

			return points;
		}
		
        void GoalDetector_RANSAC::mergeClose(std::list<Quad>& posts, double widthMultipleToMerge) {
            std::list<Quad>::iterator a = posts.begin();
            std::list<Quad>::iterator b;
            
            for (std::list<Quad>::iterator a = posts.begin(); a != posts.end(); a++) {
                for (std::list<Quad>::iterator b = (++a)/*Init b as a+1.*/; b != posts.end(); /* Iteration done by for loop */ ) {
                    // If the posts overlap.
                    // Or if their centres are horizontally closer than the largest widths multiplied by widthMultipleToMerge.
                    if (a->overlapsHorizontally(*b) ||
                       std::abs(a->getCentre()[0] - b->getCentre()[0]) <= std::max(a->getAverageWidth(), b->getAverageWidth()) * widthMultipleToMerge) {
                        // Get outer lines.
                        arma::vec2 tl;
                        arma::vec2 tr;
                        arma::vec2 bl;
                        arma::vec2 br;

                        tl << std::min(a->getTopLeft()[0],     b->getTopLeft()[0])     << std::min(a->getTopLeft()[1],     b->getTopLeft()[1]);
                        tr << std::min(a->getTopRight()[0],    b->getTopRight()[0])    << std::min(a->getTopRight()[1],    b->getTopRight()[1]);
                        bl << std::max(a->getBottomLeft()[0],  b->getBottomLeft()[0])  << std::max(a->getBottomLeft()[1],  b->getBottomLeft()[1]);
                        br << std::max(a->getBottomRight()[0], b->getBottomRight()[0]) << std::max(a->getBottomRight()[1], b->getBottomRight()[1]);

                        // Replace original two quads with the new one.
                        a->set(bl, tl, tr, br);
                        b = posts.erase(b);
                    }

                    else {
                        b++;
                    }
                }
            }
        }

        void GoalDetector_RANSAC::removeInvalid(std::list<Quad>& posts) {
            std::list<Quad>::iterator it = posts.begin();

            for (std::list<Quad>::iterator post = posts.begin(); post != posts.end(); /* Iteration done in for loop */ ) {
                // Remove all posts whos' aspect ratios are too low.
                if (post->aspectRatio() < GOAL_HEIGHT_TO_WIDTH_RATIO_MIN) {
                    post = posts.erase(post);
                }

                else {
                    post++;
                }
            }
        }

        std::unique_ptr<std::vector<messages::vision::Goal>> GoalDetector_RANSAC::createGoalMessage(const std::unique_ptr<std::vector<Goal>>& goal_posts){
        	std::unique_ptr<std::vector<messages::vision::Goal>> goal_message = std::unique_ptr<std::vector<messages::vision::Goal>>(new std::vector<messages::vision::Goal>());

        	for (auto& post : *goal_posts){
        		goal_message->push_back(messages::vision::Goal());
        		NUPoint goal_location;
        		switch(post.GOAL_DISTANCE_METHOD){
            		case (WIDTH) :
            			goal_location = post.m_widthLocation;
            			break;
            		case (D2P) :
            			goal_location = post.m_d2pLocation;
            			break;
            		case (AVERAGE) :
            			//TODO: is this useful??
            			goal_location = post.m_widthLocation;
            			goal_location.neckRelativeRadial[0] = (goal_location.neckRelativeRadial[0]+post.m_d2pLocation.neckRelativeRadial[0])/2;
            			break;
            		case (LEAST) :
            			goal_location = post.m_widthLocation;
            			goal_location.neckRelativeRadial[0] = std::min(goal_location.neckRelativeRadial[0],post.m_d2pLocation.neckRelativeRadial[0]);
            			break;
        		}
        		goal_message->back().sphericalFromNeck = goal_location.neckRelativeRadial;
        		//goal_message->back().sphericalError = goal_location.
        		goal_message->back().screenAngular = goal_location.screenAngular;
        		goal_message->back().screenCartesian = goal_location.screenCartesian;
        		goal_message->back().sizeOnScreen[0] = post.m_corners.getAverageWidth();
        		goal_message->back().sizeOnScreen[1] = post.m_corners.getAverageHeight();

        		//goal_message->back().timestamp = goal_location.
        		
        		goal_message->back().type = post.m_goalType;

        		goal_message->back().screen_quad = post.m_corners.getVertices();

        	}

        	return goal_message;
        }

	}
}

