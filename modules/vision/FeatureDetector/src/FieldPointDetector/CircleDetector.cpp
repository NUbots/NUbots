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

#include "CircleDetector.h"

namespace modules {
    namespace vision {

        CircleDetector::CircleDetector() {
            // Empty constructor.
        }

        void CircleDetector::setParameters(double TOLERANCE_,
                                           unsigned int MINIMUM_POINTS_,
                                           unsigned int ITERATIONS_PER_FITTING_,
                                           double CONSENSUS_MARGIN_,
                                           unsigned int MAX_FITTINGS_,
                                           float CENTRE_CIRCLE_RADIUS_, 
                                           const VisionKinematics& transformer) {
            TOLERANCE = std::max(std::min(TOLERANCE_, 1.0), 0.0);
            MINIMUM_POINTS = MINIMUM_POINTS_;
            ITERATIONS_PER_FITTING = ITERATIONS_PER_FITTING_;
            CONSENSUS_MARGIN = CONSENSUS_MARGIN_;
            MAX_FITTINGS = MAX_FITTINGS_;
            CENTRE_CIRCLE_RADIUS = CENTRE_CIRCLE_RADIUS_;
            m_transformer = transformer;
        }

        bool CircleDetector::run(std::vector<NUPoint>& points, CentreCircle& result) {
            RANSACCircle<NUPoint> candidate;
            std::vector<NUPoint> consensus, remainder;
            double variance;
            unsigned int i = 0;
            bool modelFound = false;

            // Attemp multiple RANSAC fits.

            // Run first iterations.
            modelFound = findModel<RANSACCircle<NUPoint>, NUPoint>(points, 
                                                                   candidate, 
                                                                   consensus, 
                                                                   remainder, 
                                                                   variance, 
                                                                   CONSENSUS_MARGIN, 
                                                                   MINIMUM_POINTS, 
                                                                   ITERATIONS_PER_FITTING, 
                                                                   RANSAC_SELECTION_METHOD::LargestConsensus);

            //continue while models are found but not a final version
            while (modelFound && (i < MAX_FITTINGS)) {
                // Check if the model is good enough.
                if (variance <= (TOLERANCE * candidate.getRadius()) &&
                    candidate.getRadius() <= ((1 + TOLERANCE) * CENTRE_CIRCLE_RADIUS) &&
                    candidate.getRadius() >= ((1 - TOLERANCE) * CENTRE_CIRCLE_RADIUS)) {
                    // Get outer points to determine screen radius.
                    // TODO: Work out how to get image width/height into this submodule.
                    double left = 10; // VisionBlackboard::getInstance()->getImageWidth() - 1;
                    double right = 0;
                    double top = 10; // VisionBlackboard::getInstance()->getImageHeight() - 1;
                    double bottom = 0;

                    for(NUPoint p : consensus) {
                        left = std::min(left, p.screenCartesian[0]);
                        right = std::max(right, p.screenCartesian[0]);
                        top = std::min(top, p.screenCartesian[1]);
                        bottom = std::max(bottom, p.screenCartesian[1]);
                    }

                    NUPoint centre = candidate.getCentre();

                    m_transformer.calculateRepresentationsFromGroundCartesianLocation(centre);

                    arma::vec2 screenSize;
                    screenSize << (right - left) << (bottom - top);
                    result = CentreCircle(centre, candidate.getRadius(), screenSize);
                    points = remainder;

                    // Break out once decent model found.
                    return true;
                }

                else {
                    // Model isn't good enough, reattempt with remainder.
                    modelFound = findModel<RANSACCircle<NUPoint>, NUPoint>(remainder, 
                                                                           candidate, 
                                                                           consensus, 
                                                                           remainder, 
                                                                           variance, 
                                                                           CONSENSUS_MARGIN, 
                                                                           MINIMUM_POINTS, 
                                                                           ITERATIONS_PER_FITTING, 
                                                                           RANSAC_SELECTION_METHOD::LargestConsensus);
                }

                i++;
            }

            return false;
        }

    }
}
