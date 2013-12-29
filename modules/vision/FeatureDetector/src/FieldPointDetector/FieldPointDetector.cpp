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

#include "FieldPointDetector.h"

namespace modules {
    namespace vision {

        using messages::vision::ColourSegment;
        using messages::vision::COLOUR_CLASS;

        FieldPointDetector::FieldPointDetector()  : m_lineDetector(), m_circleDetector(), m_cornerDetector() {
            // Empty constructor.
        }

        void FieldPointDetector::setParameters(bool TRANSFORM_FIRST_,
                                               const VisionKinematics& transformer,
                                    
                                               // CircleDetector.
                                               double CIRCLE_DETECTOR_TOLERANCE_,
                                               unsigned int MINIMUM_POINTS_,
                                               unsigned int ITERATIONS_PER_FITTING_,
                                               double CONSENSUS_MARGIN_,
                                               unsigned int MAX_FITTINGS_,
                                               float CENTRE_CIRCLE_RADIUS_, 
                                    
                                               // CornerDetector.
                                               double CORNER_DETECTOR_TOLERANCE_) {
            TRANSFORM_FIRST = TRANSFORM_FIRST_;

            m_transformer = transformer;

            // CircleDetector.
            CIRCLE_DETECTOR_TOLERANCE = CIRCLE_DETECTOR_TOLERANCE_;
            MINIMUM_POINTS = MINIMUM_POINTS_;
            ITERATIONS_PER_FITTING = ITERATIONS_PER_FITTING_;
            CONSENSUS_MARGIN = CONSENSUS_MARGIN_;
            MAX_FITTINGS = MAX_FITTINGS_;
            CENTRE_CIRCLE_RADIUS = CENTRE_CIRCLE_RADIUS_;

            // CornerDetector.
            CORNER_DETECTOR_TOLERANCE = CORNER_DETECTOR_TOLERANCE_;
        }

        void FieldPointDetector::run(bool findCircles, 
                                    bool findLines, 
                                    bool findCorners,
                                    const std::vector<arma::vec2>& greenHorizon) {
            /// @note At present no detection exists for corners or ellipses in the image plane so
            ///       disabling this flag disables all field point detection methods.
            if (TRANSFORM_FIRST) {
                // Check transforms are valid.
                std::vector<NUPoint> points;
                NUPoint temp;
                CentreCircle circle;
                std::vector<FieldLine> lines;
                std::vector<CornerPoint> corners;

                // Collect all vertical and horizontal line transition centres that exist under the green horizon.
                // TODO: What is vbb->getAllTransistions?
                /*const*/ std::vector<ColourSegment>/*&*/ segments; // = vbb->getAllTransitions(LINE_COLOUR);

                for (const ColourSegment& segment : segments) {
                    temp.screenCartesian = segment.m_centre;

                    if (temp.screenCartesian[1] > greenHorizon.at(temp.screenCartesian[0])[1])
                        points.push_back(temp);
                }

                // Map those points to the ground plane.
                m_transformer.calculateRepresentationsFromPixelLocation(points);

                if (findCircles) {
                    // First attempt to find a centre circle.
                    m_circleDetector.setParameters(CIRCLE_DETECTOR_TOLERANCE,
                                                   MINIMUM_POINTS,
                                                   ITERATIONS_PER_FITTING,
                                                   CONSENSUS_MARGIN,
                                                   MAX_FITTINGS,
                                                   CENTRE_CIRCLE_RADIUS,
                                                   m_transformer);
                    
                    if (m_circleDetector.run(points, circle)) {
                        // Circle found - points are already removed.
                        // TODO: FIX ME!
                        std::move(circle);
                    }
                }

                if (findLines) {
                    // TODO: FIX ME!
                    std::move(m_lineDetector.run(points));
                }

                // Now find corners.
                if (findCorners) {
                    // TODO: FIX ME!
                    m_cornerDetector.setParameters(CORNER_DETECTOR_TOLERANCE);
                    std::move(m_cornerDetector.run(lines));
                }
            }
        }

    }
}
