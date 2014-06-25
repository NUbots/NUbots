/*
 * This file is part of NUbots Codebase.
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

#include "GoalDetector.h"

#include "messages/vision/ClassifiedImage.h"
#include "messages/support/Configuration.h"
#include "messages/vision/VisionObjects.h"

#include "utility/math/geometry/Line.h"

#include "utility/math/ransac/RansacLineModel.h"

namespace modules {
namespace vision {

    using utility::math::geometry::Line;
    using utility::math::geometry::Quad;
    using utility::math::geometry::LSFittedLine;

    using utility::math::ransac::RansacLineModel;
    using utility::math::ransac::findMultipleModels;
    using utility::math::ransac::RansacSelectionMethod;

    using messages::vision::ObjectClass;
    using messages::vision::ClassifiedImage;
    using messages::vision::Goal;

    using messages::support::Configuration;


    GoalDetector::GoalDetector(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Trigger<Configuration<GoalDetector>>>([this](const Configuration<GoalDetector>& config) {

                std::string selectionMethod = config["SELECTION_METHOD"].as<std::string>();

                if (selectionMethod.compare("LARGEST_CONSENSUS") == 0) {
                    SELECTION_METHOD = RansacSelectionMethod::LARGEST_CONSENSUS;
                }
                else if (selectionMethod.compare("BEST_FITTING_CONSENSUS") == 0) {
                    SELECTION_METHOD = RansacSelectionMethod::BEST_FITTING_CONSENSUS;
                }
                else {
                    SELECTION_METHOD = RansacSelectionMethod::LARGEST_CONSENSUS;
                }

                MINIMUM_POINTS = config["MINIMUM_POINTS"].as<uint>();
                MAX_ITERATIONS_PER_FITTING = config["MAX_ITERATIONS_PER_FITTING"].as<uint>();
                MAX_FITTING_ATTEMPTS = config["MAX_FITTING_ATTEMPTS"].as<uint>();
                ANGLE_MARGIN = config["ANGLE_MARGIN"].as<double>();
                CONSENSUS_THRESHOLD = config["CONSENSUS_THRESHOLD"].as<double>();
                RANSAC_MATCHING_TOLERANCE = config["RANSAC_MATCHING_TOLERANCE"].as<double>();
                MIN_GOAL_SEPARATION = config["MIN_GOAL_SEPARATION"].as<int>();
                GOAL_HEIGHT_TO_WIDTH_RATIO_MIN = config["GOAL_HEIGHT_TO_WIDTH_RATIO_MIN"].as<float>();
                THROWOUT_SHORT_GOALS = config["THROWOUT_SHORT_GOALS"].as<bool>();
                THROWOUT_NARROW_GOALS = config["THROWOUT_NARROW_GOALS"].as<bool>();
                THROWOUT_ON_ABOVE_KIN_HOR_GOALS = config["THROWOUT_ON_ABOVE_KIN_HOR_GOALS"].as<bool>();
                THROWOUT_DISTANT_GOALS = config["THROWOUT_DISTANT_GOALS"].as<bool>();
                MAX_GOAL_DISTANCE = config["MAX_GOAL_DISTANCE"].as<float>();
                MIN_GOAL_HEIGHT = config["MIN_GOAL_HEIGHT"].as<int>();
                MIN_GOAL_WIDTH = config["MIN_GOAL_WIDTH"].as<int>();
                GOAL_WIDTH = config["GOAL_WIDTH"].as<float>();
                EDGE_OF_SCREEN_MARGIN = config["EDGE_OF_SCREEN_MARGIN"].as<int>();
                D2P_ADAPTIVE_THRESHOLD = config["D2P_ADAPTIVE_THRESHOLD"].as<float>();

        });

        on<Trigger<ClassifiedImage<ObjectClass>>>([this](const ClassifiedImage<ObjectClass>& image) {

            std::vector<Quad> quads, postCandidates;
            std::pair<bool, Quad> crossbar(false, Quad());
            std::vector<Goal> posts;

            std::vector<arma::vec2> startPoints, endPoints;
            std::vector<LSFittedLine> startLines, endLines;
            std::vector<std::pair<RansacLineModel<arma::vec2>, std::vector<arma::vec2>>> ransacResults;

            auto hSegments = image.horizontalSegments.equal_range(ObjectClass::GOAL);
            for(auto it = hSegments.first; it != hSegments.second; ++it) {

                // We throw out points if they are:
                // Less the full quality (subsampled)
                // Do not have a transition on either side (are on an edge)
                if(it->second.subsample == 1
                    && it->second.previous
                    && it->second.next) {

                    startPoints.push_back({ double(it->second.start[0]), double(it->second.start[1]) });
                    endPoints.push_back({ double(it->second.end[0]), double(it->second.end[1]) });
                }
            }

            // Use generic RANSAC implementation to find start lines (left edges).
            ransacResults = findMultipleModels<RansacLineModel<arma::vec2>, arma::vec2>(startPoints,
                                                                                        CONSENSUS_THRESHOLD,
                                                                                        MINIMUM_POINTS,
                                                                                        MAX_ITERATIONS_PER_FITTING,
                                                                                        MAX_FITTING_ATTEMPTS,
                                                                                        SELECTION_METHOD);


            for (auto& l : ransacResults) {
                startLines.push_back(LSFittedLine(l.second));
            }

            // Use generic RANSAC implementation to find end lines (right enddges).
            ransacResults = findMultipleModels<RansacLineModel<arma::vec2>, arma::vec2>(endPoints,
                                                                                        CONSENSUS_THRESHOLD,
                                                                                        MINIMUM_POINTS,
                                                                                        MAX_ITERATIONS_PER_FITTING,
                                                                                        MAX_FITTING_ATTEMPTS,
                                                                                        SELECTION_METHOD);

            for (auto& l : ransacResults) {
                endLines.push_back(LSFittedLine(l.second));
            }

            // Build candidates out of lines - this finds candidates irrespective of rotation - filtering must be done later.
            quads = buildQuadsFromLines(startLines, endLines, RANSAC_MATCHING_TOLERANCE);

            // Remove posts with invalid aspect ratio : check potential cross bars AND posts.
            removeInvalid(quads);

            // Sort out potential crossbars and vertical posts (posts on too large of a lean will be removed).
            // Edit ANGLE_MARGIN to affect this.
            double halfPI = arma::math::pi() * 0.5;
            double lowerAngleThreshold = (ANGLE_MARGIN * halfPI);
            double upperAngleThreshold = halfPI - lowerAngleThreshold;


            for (const Quad& quad : quads) {

                //TODO!!!!!!!!!!!!!!!!!!!!!!!!!!! - add in filtering with kinematics horizon
                //double angle = m_kinematicsHorizon.getAngleBetween(Line(quad.getTopCentre(), quad.getBottomCentre()));
                Line horizontal_line;
                horizontal_line.setLine(0,1,0); //0*x+y=0
                double angle = horizontal_line.getAngleBetween(Line(quad.getTopCentre(), quad.getBottomCentre()));

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

            log("Number of seen goals", postCandidates.size());

//            for(auto& post : postCandidates) {
//                std::cout << post << std::endl;
//            }
//            std::cout << std::endl;

            // Beyond here needs vision kinematics

            // // Generate actual goal from candidate posts.
            // if (crossbar.first) {
            //     // If a cross bar has been found use it to help assign left and right.
            //     posts = assignGoals(visionKinematics, postCandidates, crossbar.second);
            // }
            // else {
            //     // No crossbar, just use general method.
            //     posts = assignGoals(visionKinematics, postCandidates);
            // }

            // // Improves bottom centre estimate using vertical transitions.
            // int numberOfBasesSet = 0;
            // for (const ColourSegment& segment : verticalSegments) {

            //     const arma::vec2& point = segment.m_end;
            //     for (Goal& post : *posts) {
            //         if ((point[0] <= post.getQuad().getRight()) &&
            //                 (point[0] >= post.getQuad().getLeft()) &&
            //                 (point[1] > post.getLocationPixels()[1])) {
            //             post.setBase(visionKinematics, point);
            //             numberOfBasesSet++;
            //         }
            //     }
            // }

            // std::unique_ptr<std::vector<messages::vision::Goal>> finalGoals = std::move(createGoalMessage(posts));

            // return std::move(finalGoals);

        });
    }

std::vector<Quad> GoalDetector::buildQuadsFromLines(const std::vector<LSFittedLine>& startLines,
                                                  const std::vector<LSFittedLine>& endLines, double tolerance) {
    // (must match exactly) 0 <= tolerance <= 1 (any pair will be accepted)
    //
    // LSFittedLine objects contain std::vectors of points and can be quite large,
    // therefore it is more efficient to pass by const reference and maintain
    // a std::vector of matched end lines than to pass by copy so that the end lines
    // std::vector can be shrunk.

    if ((tolerance < 0) || (tolerance > 1)) {
        tolerance = 1;                  // TODO: Pick a better action here? We used to throw.
    }

    std::vector<Quad> quads;
    std::vector<bool> used(endLines.size(), false);

    for (const LSFittedLine& startLine : startLines) {
        std::vector<bool> tried(used);                      // Consider all used lines tried.
        bool matched = false;

        // Try end lines in order of closeness.
        for (unsigned int i = getClosestUntriedLine(startLine, endLines, tried);
                                ((i < endLines.size()) && (!matched));
                                i = getClosestUntriedLine(startLine, endLines, tried)) {

            const LSFittedLine& endLine = endLines.at(i);

            // Check angles.
            if (startLine.getAngleBetween(endLine) <= (tolerance * arma::math::pi() * 0.5)) {                   // Dodgy way (linear with angle between).
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

                                quads.push_back(Quad(sp1, sp2, ep2, ep1));                      // Generate candidate.
                                used.at(i) = true;                                              // Remove end line from consideration.
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

unsigned int GoalDetector::getClosestUntriedLine(const LSFittedLine& start,
                                                        const std::vector<LSFittedLine>& endLines,
                                                        std::vector<bool>& tried) {
    if (endLines.size() != tried.size()) {
        return 0;                   // TODO: Pick a better action here? We used to throw.
    }

    unsigned int best = endLines.size();                                            // For if all have been tried.
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

// std::unique_ptr<std::vector<Goal> > GoalDetector::assignGoals(const VisionKinematics& visionKinematics, const std::vector<Quad>& candidates, const Quad& crossbar) const {
//     if (candidates.size() == 1) {
//         Goal goal(visionKinematics);

//         if (crossbar.getCentre()[0] < candidates.front().getCentre()[0]) {
//             goal = Goal(visionKinematics, messages::vision::Goal::Type::RIGHT, candidates.front());
//         }

//         else {
//             goal = Goal(visionKinematics, messages::vision::Goal::Type::LEFT, candidates.front());
//         }

//         goal.setParameters(THROWOUT_SHORT_GOALS,
//                            THROWOUT_NARROW_GOALS,
//                            THROWOUT_ON_ABOVE_KIN_HOR_GOALS,
//                            THROWOUT_DISTANT_GOALS,
//                            MAX_GOAL_DISTANCE,
//                            MIN_GOAL_HEIGHT,
//                            MIN_GOAL_WIDTH,
//                            GOAL_WIDTH,
//                            GOAL_DISTANCE_METHOD,
//                            EDGE_OF_SCREEN_MARGIN,
//                            D2P_ADAPTIVE_THRESHOLD,
//                            visionKinematics);

//         return std::move(  std::unique_ptr<std::vector<Goal>>(  new std::vector<Goal>(1, goal)  )  );
//     }

//     return assignGoals(visionKinematics, candidates);
// }

// std::unique_ptr<std::vector<Goal> > GoalDetector::assignGoals(const VisionKinematics& visionKinematics, const std::vector<Quad>& candidates) const {

//     std::unique_ptr<std::vector<Goal> > goals = std::unique_ptr<std::vector<Goal>>(new std::vector<Goal>());

//     if (candidates.size() == 2) {
//         Goal leftPost(visionKinematics), rightPost(visionKinematics);

//         //there are exactly two candidates, identify each as left or right
//         Quad post1 = candidates.front();
//         Quad post2 = candidates.back();

//         //calculate separation between candidates
//         double pos1 = std::min(post1.getRight(), post2.getRight());      // inside right
//         double pos2 = std::max(post1.getLeft(), post2.getLeft());  // inside left

//         //only publish if the candidates are far enough apart
//         if (std::abs(pos2 - pos1) >= MIN_GOAL_SEPARATION) {
//             //flip if necessary
//             if (post1.getCentre()[0] > post2.getCentre()[0]) {
//                 leftPost = Goal(visionKinematics, messages::vision::Goal::Type::LEFT, post2);
//                 rightPost = Goal(visionKinematics, messages::vision::Goal::Type::RIGHT, post1);
//             }

//             else {
//                 leftPost = Goal(visionKinematics, messages::vision::Goal::Type::LEFT, post1);
//                 rightPost = Goal(visionKinematics, messages::vision::Goal::Type::RIGHT, post2);
//             }



//            leftPost.setParameters(THROWOUT_SHORT_GOALS,
//                                    THROWOUT_NARROW_GOALS,
//                                    THROWOUT_ON_ABOVE_KIN_HOR_GOALS,
//                                    THROWOUT_DISTANT_GOALS,
//                                    MAX_GOAL_DISTANCE,
//                                    MIN_GOAL_HEIGHT,
//                                    MIN_GOAL_WIDTH,
//                                    GOAL_WIDTH,
//                                    GOAL_DISTANCE_METHOD,
//                                    EDGE_OF_SCREEN_MARGIN,
//                                    D2P_ADAPTIVE_THRESHOLD,
//                                    visionKinematics);
//             rightPost.setParameters(THROWOUT_SHORT_GOALS,
//                                     THROWOUT_NARROW_GOALS,
//                                     THROWOUT_ON_ABOVE_KIN_HOR_GOALS,
//                                     THROWOUT_DISTANT_GOALS,
//                                     MAX_GOAL_DISTANCE,
//                                     MIN_GOAL_HEIGHT,
//                                     MIN_GOAL_WIDTH,
//                                     GOAL_WIDTH,
//                                     GOAL_DISTANCE_METHOD,
//                                     EDGE_OF_SCREEN_MARGIN,
//                                     D2P_ADAPTIVE_THRESHOLD,
//                                     visionKinematics);
//             goals->push_back(leftPost);
//             goals->push_back(rightPost);
//         }

//         else {
//             //should merge
//         }
//     }

//     else {
//         //unable to identify which post is which
//         //setting all to unknown
//         for(const Quad& candidate : candidates) {
//             Goal goal = Goal(visionKinematics, messages::vision::Goal::Type::UNKNOWN, candidate);
//             goal.setParameters(THROWOUT_SHORT_GOALS,
//                                 THROWOUT_NARROW_GOALS,
//                                 THROWOUT_ON_ABOVE_KIN_HOR_GOALS,
//                                 THROWOUT_DISTANT_GOALS,
//                                 MAX_GOAL_DISTANCE,
//                                 MIN_GOAL_HEIGHT,
//                                 MIN_GOAL_WIDTH,
//                                 GOAL_WIDTH,
//                                 GOAL_DISTANCE_METHOD,
//                                 EDGE_OF_SCREEN_MARGIN,
//                                 D2P_ADAPTIVE_THRESHOLD,
//                                 visionKinematics);
//             goals->push_back(goal);
//         }
//     }

//     return std::move(goals);
// }

void GoalDetector::mergeClose(std::vector<Quad>& posts, double widthMultipleToMerge) {
    if(posts.size()<2){
        return;
    }
    for (std::vector<Quad>::iterator a = posts.begin(); a != posts.end(); a++) {
        for (std::vector<Quad>::iterator b = (std::next(a))/*Init b as a+1.*/; b != posts.end(); /* Iteration done inside the 'for' loop */ ) {
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
                tr << std::max(a->getTopRight()[0],    b->getTopRight()[0])    << std::min(a->getTopRight()[1],    b->getTopRight()[1]);
                bl << std::min(a->getBottomLeft()[0],  b->getBottomLeft()[0])  << std::max(a->getBottomLeft()[1],  b->getBottomLeft()[1]);
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

void GoalDetector::removeInvalid(std::vector<Quad>& posts) {
    std::vector<Quad>::iterator it = posts.begin();

    for (std::vector<Quad>::iterator post = posts.begin(); post != posts.end(); /* Iteration done in for loop */ ) {
        // Remove all posts whos' aspect ratios are too low.
        if (post->aspectRatio() < GOAL_HEIGHT_TO_WIDTH_RATIO_MIN || !post->checkCornersValid()) {
            post = posts.erase(post);
        } else {
            post++;
        }
    }
}

}
}

