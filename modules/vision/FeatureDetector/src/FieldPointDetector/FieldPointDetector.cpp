#include "FieldPointDetector.h"

namespace modules {
    namespace vision {

        using messages::vision::ColourSegment;

        FieldPointDetector::FieldPointDetector(/*LineDetector* lineDetector, CircleDetector* circleDetector, CornerDetector* cornerDetector*/) {
            /*
            m_lineDdetector = lineDetector;
            m_circleDdetector = circleDetector;
            m_cornerDdetector = cornerDdetector;
            */
        }

        void FieldPointDetector::setParameters(bool TRANSFORM_FIRST_) {
            TRANSFORM_FIRST = TRANSFORM_FIRST_;
        }

        void FieldPointDetector::run(bool findCircles, 
                                    bool findLines, 
                                    bool findCorners,
                                    const VisionKinematics& transformer, 
                                    const std::vector<arma::vec2>& greenHorizon) const {
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
                const std::vector<ColourSegment>& segments = vbb->getAllTransitions(LINE_COLOUR);

                for (const ColourSegment& segment : segments) {
                    temp.screenCartesian = segment.getCentre();

                    if (temp.screenCartesian[1] > greenHorizon.at(temp.screenCartesian[0])[1])
                        points.push_back(temp);
                }

                // Map those points to the ground plane.
                transformer.calculateRepresentationsFromPixelLocation(points);

                if (findCircles && m_circleDetector) {
                    // First attempt to find a centre circle.
                    if (m_circleDetector->run(points, circle)) {
                        // Circle found - points are already removed.
                        // TODO: FIX ME!
                        std::move(circle);
                    }
                }

                if (findLines && m_lineDetector) {
                    // TODO: FIX ME!
                    std::mmove(m_lineDetector->run(points);)
                }

                // Now find corners.
                if (findCorners && m_cornerDetector) {
                    // TODO: FIX ME!
                    std::move(m_cornerDetector->run(lines));
                }
            }
        }

    }
}
