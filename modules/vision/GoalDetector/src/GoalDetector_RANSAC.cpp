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

		using messages::vision::ClassifiedImage::ColourSegment;
		
		GoalDetector_RANSAC::GoalDetector_RANSAC() {
			// Empty constructor.
		}

		void GoalDetector_RANSAC::setParameters(unsigned int minimumPoints, 
												unsigned int maxIterationsPerFitting, 
												double consensusThreshold, 
												unsigned int maxFittingAttempts, 
												double ANGLE_MARGIN_, 
												Horizon& kinematicsHorizon, 
												RANSAC_SELECTION_METHOD RANSACMethod
												double RANSAC_MATCHING_TOLERANCE_) {
			m_minimumPoints = minimumPoints; 										// Minimum points needed to make a line (Min pts to line essentially)
																					// Original value: 25 / VisionConstants::HORIZONTAL_SCANLINE_SPACING;
			m_maxIterationsPerFitting = maxIterationsPerFitting;					// Number of iterations per fitting attempt
			m_consensusThreshold = consensusThreshold;								// Threshold dtermining what constitutes a good fit (Consensus margin)
			m_maxFittingAttempts = maxFittingAttempts;								// Hard limit on number of fitting attempts
			
		    ANGLE_MARGIN = ANGLE_MARGIN_;											// Used for filtering out goal posts which are on too much of a lean.
		    
		    m_kinematicsHorizon = kinematicsHorizon;								// = vbb->getKinematicsHorizon();
		    m_RANSACMethod = RANSACMethod;
		    
			RANSAC_MATCHING_TOLERANCE = RANSAC_MATCHING_TOLERANCE_;
		}
	
		// std::vector<ColourSegment> verticalSegments = ClassifiedImage::matched_vertical_segments[GOAL_COLOUR];
		// std::vector<ColourSegment> horizontalSegments = ClassifiedImage::matched_horizontal_segments[GOAL_COLOUR];
		std::unique_ptr<std::vector<Goal>> GoalDetector_RANSAC::run(const std::vector<ColourSegment>& horizontalSegments, const std::vector<ColourSegment>& verticalSegments) {
			std::list<Quad> quads, postCandidates;
			std::pair<bool, Quad> crossbar(false, Quad());
			std::unique_ptr<std::vector<Goal>> posts = std::unique_ptr<std::vector<Goal>>(new std::vector<Goal>());

			std::vector<arma::vec2> startPoints, endPoints;
			std::vector<pair<RANSACLine<arma::vec2>, std::vector<arma::vec2>>> ransacResults;
			std::vector<LSFittedLine> startLines, endLines;

			// Get edge points.
			for (ColourSegment& segment : horizontalSegments) {
				startPoints.push_back(segment.getStart());
				endPoints.push_back(segment.getEnd());
			}

			// Use generic RANSAC implementation to find start lines (left edges).
			ransacResults = RANSAC::findMultipleModels<RANSACLine<arma::vec2>, arma::vec2>(startPoints, m_consensusThreshold, m_minimumPoints, m_maxIterationsPerFitting, m_maxFittingAttempts, m_RANSACMethod);
		
			for (const auto& line : ransacResults) {
				startLines.push_back(LSFittedLine(line.second));
			}

			// Use generic RANSAC implementation to find end lines (right edges).
			ransacResults = RANSAC::findMultipleModels<RANSACLine<arma::vec2>, arma::vec2>(endPoints, m_consensusThreshold, m_minimumPoints, m_maxIterationsPerFitting, m_maxFittingAttempts, m_RANSACMethod);
		
			for (const auto& line : ransacResults) {
				endLines.push_back(LSFittedLine(line.second));
			}

			/*********************
			 *		DEBUG POINT	*
			 *********************
			 * startLines			*
			 * endLines			*
			 *********************/

			// Build candidates out of lines - this finds candidates irrespective of rotation - filtering must be done later.
			quads = buildQuadsFromLines(startLines, endLines, RANSAC_MATCHING_TOLERANCE);

			// Remove posts with invalid aspect ratio : check potential cross bars AND posts.
			removeInvalid(quads);

			// Sort out potential crossbars and vertical posts (posts on too large of a lean will be removed).
			// Edit ANGLE_MARGIN to affect this.
			double halfPI = datum::pi * 0.5;
			double lowerAngleThreshold = (ANGLE_MARGIN * halfPI);
			double upperAngleThreshold = halfPI - lowerAngleThreshold;
			
			for (const Quad& quad : quads) {
				double angle = m_kinematicsHorizon.getAngleBetween(Line(quad.getTopCentre(), quad.getBottomCentre()));

				if (angle >= upperAngleThreshold) {
				    postCandidates.push_back(quad);
				}
				
				else if (angle <= lowerAngleThreshold) {
				    // Only keep largest crossbar candidate.
				    if (!crossbar.first) {
				        crossbar.first = true;
				        crossbar.second = quad;
				    }
				    
				    else if (crossbar.second.area() < quad.area()) {
				        crossbar.second = quad;
				    }
				}
			}

			// Only check upright posts for building candidates.
			mergeClose(postCandidates, 1.5);

			// Generate actual goal from candidate posts.
			if (crossbar.first) {
				// If a cross bar has been found use it to help assign left and right.
				posts = assignGoals(postCandidates, crossbar.second);
			}
		
			else {
				// No crossbar, just use general method.
				posts = assignGoals(postCandidates);
			}

			// Improves bottom centre estimate using vertical transitions.
			for (const ColourSegment& segment : verticalSegments) {
				const arma::vec2& point = segment.getEnd();
				
				for (Goal& post : posts) {
				    if ((point[0] <= post.getQuad().getRight()) && (point[0] >= post.getQuad().getLeft()) && (point[1] > post.getLocationPixels()[1])) {
				        post.setBase(point);
				    }
				}
			}

			return std::move(posts);
		}

		std::list<Quad> GoalDetector_RANSAC::buildQuadsFromLines(const std::vector<LSFittedLine>& startLines, const std::vector<LSFittedLine>& endLines, double tolerance)
		{
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

			for (const LSFittedLine& s : startLines) {
				std::vector<bool> tried(used);						// Consider all used lines tried.
				bool matched = false;

				// Try end lines in order of closeness.
				for (unsigned int i = getClosestUntriedLine(s, endLines, tried);
										((i < endLines.size()) && (!matched));
										i = getClosestUntriedLine(s, endLines, tried)) {

				    const LSFittedLine& e = endLines.at(i);

				    // Check angles.
				    if (s.getAngleBetween(e) <= (tolerance * datum::pi * 0.5)) {					// Dodgy way (linear with angle between).
				    //if(std::min(a1/a2, a2/a1) <= (1 - tolerance)) {
				        // Get the end points of each line.
				        arma::vec2 sp1, sp2, ep1, ep2;
				        
				        if (s.getEndPoints(sp1, sp2) && e.getEndPoints(ep1, ep2)) {
				            // Find lengths of line segments.
				            double l1 = arma::norm((sp1 - sp2), 2);
				            double l2 = arma::norm((ep1 - ep2), 2);
				            
				            // Check lengths.
				            if (std::min((l1 / l2), (l2 / l1)) >= (1 - tolerance)) {
				                // Get num points.
				                double n1 = s.getNumPoints(), n2 = e.getNumPoints();
				                
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

		unsigned int GoalDetector_RANSAC::getClosestUntriedLine(const LSFittedLine& start, const std::vector<LSFittedLine>& endLines, std::vector<bool>& tried) {
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

		std::vector<arma::vec2> GoalDetector_RANSAC::getEdgePointsFromSegments(const std::vector<ColourSegment> &segments) {
			std::vector<arma::vec2> points;
			
			for (const ColourSegment& s : segments) {
				points.push_back(arma::vec2(s.getStart()));
				points.push_back(arma::vec2(s.getEnd()));
			}

			return points;
		}

		std::unique_ptr<std::vector<Goal>> GoalDetector_RANSAC::assignGoals(const std::list<Quad>& candidates, const Quad& crossbar) const {
			if (candidates.size() == 1) {
				if (crossbar.getCentre()[0] < candidates.front().getCentre()[0]) {
				    return std::unique_ptr<std::vector<Goal>>(new std::vector<Goal>(1, Goal(GOAL_R, candidates.front()));
				}
				
				else {
				    return std::unique_ptr<std::vector<Goal>>(new std::vector<Goal>(1, Goal(GOAL_L, candidates.front()));
				}
			}
			
			else {
				return std::move(assignGoals(candidates));
			}
		}

		std::unique_ptr<std::vector<Goal>> GoalDetector_RANSAC::assignGoals(const std::list<Quad>& candidates) const {
			std::unique_ptr<std::vector<Goal>> goals = std::unique_ptr<std::vector<Goal>>(new std::vector<Goal>());
			
			if (candidates.size() == 2) {
				// There are exactly two candidates, identify each as left or right.
				const Quad& post1 = candidates.front();
				const Quad& post2 = candidates.back();

				// Calculate separation between candidates.
				double pos1 = std::min(post1.getRight(), post2.getRight());			// inside right
				double pos2 = std::max(post1.getLeft(), post2.getLeft());			// inside left

				// Only publish if the candidates are far enough apart.
				if (std::abs(pos2 - pos1) >= VisionConstants::MIN_GOAL_SEPARATION) {
				    // Flip if necessary.
				    if (post1.getCentre()[0] > post2.getCentre()[0]) {
				        goals->push_back(Goal(GOAL_L, post2));
				        goals->push_back(Goal(GOAL_R, post1));
				    }
				    
				    else {
				        goals->push_back(Goal(GOAL_L, post1));
				        goals->push_back(Goal(GOAL_R, post2));
				    }
				}
				
				else {
				    //should merge
				}
			}
			
			else {
				// Unable to identify which post is which.
				// Setting all to unknown.
				for (const Quad& candidate : candidates) {
				    goals->push_back(Goal(GOAL_U, candidate));
				}
			}
			
			return std::move(goals);
		}

		void GoalDetector_RANSAC::removeInvalid(std::list<Quad>& posts) {
			std::list<Quad>::iterator it = posts.begin();
			
			for (std::list<Quad>::iterator it = posts.begin(); it != posts.end(); /* Increment done by if statement */) {
				// Remove all posts whos' aspect ratios are too low.
				if ( it->aspectRatio() < VisionConstants::GOAL_HEIGHT_TO_WIDTH_RATIO_MIN) {
				    it = posts.erase(it);
				}
				
				else {
				    it++;
				}
			}
		}
		
		void GoalDetector::mergeClose(std::list<Quad>& posts, double widthMultipleToMerge) {
			std::list<Quad>::iterator a = posts.begin();
			std::list<Quad>::iterator b;
			
			for (std::list<Quad>::iterator post = posts.begin(); post != posts.end(); post++) {
				for (std::list<Quad>::iterator postCompare = post; postCompare != posts.end();  /* Increment done by if statement */) {
				    // If the posts overlap.
				    // Or if their centres are horizontally closer than the largest widths multiplied by widthMultipleToMerge.
					if (post->overlapsHorizontally(*postCompare) || (std::abs(post->getCentre()[0] - postCompare->getCentre()[0]) <= (std::max(post->getAverageWidth(), postCompare->getAverageWidth()) * widthMultipleToMerge))) {
				        // Get outer lines.
				        arma::vec2 tl = {std::min(post->getTopLeft()[0],		postCompare->getTopLeft()[0]),		std::min(post->getTopLeft()[1],		postCompare->getTopLeft()[1])};
				        arma::vec2 tr = {std::max(post->getTopRight()[0],		postCompare->getTopRight()[0]),		std::min(post->getTopRight()[1],	postCompare->getTopRight()[1])};
				        arma::vec2 bl = {std::min(post->getBottomLeft()[0],		postCompare->getBottomLeft()[0]),	std::max(post->getBottomLeft()[1],	postCompare->getBottomLeft()[1])};
				        arma::vec2 br = {std::max(post->getBottomRight()[0],	postCompare->getBottomRight()[0]),	std::max(post->getBottomRight()[1], postCompare->getBottomRight()[1])};

				        //replace original two quads with the new one
				        post->set(bl, tl, tr, br);
				        postCompare = posts.erase(postCompare);
					}
					
					else {
						postCompare++;
					}
				}
			}
		}

	}
}

