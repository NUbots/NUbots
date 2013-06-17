#include "fieldpointdetector.h"
#include "Vision/VisionTypes/coloursegment.h"
#include "Vision/VisionTypes/greenhorizon.h"
#include "Vision/visionblackboard.h"
#include <boost/foreach.hpp>
#include "debug.h"
#include "debugverbosityvision.h"

FieldPointDetector::FieldPointDetector(LineDetector *line_detector, CircleDetector *circle_detector, CornerDetector *corner_detector)
{
    m_line_detector = line_detector;
    m_circle_detector = circle_detector;
    m_corner_detector = corner_detector;
}

void FieldPointDetector::run(bool find_circle, bool find_lines, bool find_corners) const
{
    VisionBlackboard* vbb = VisionBlackboard::getInstance();
    const Transformer& transformer = vbb->getTransformer();

    /// @note At present no detection exists for corners or ellipses in the image plane so
    ///       disabling this flag disables all field point detection methods.
    static bool TRANSFORM_FIRST = true;
    if(TRANSFORM_FIRST) {
        //check transforms are valid
        std::vector<NUPoint> points;
        NUPoint temp;
        CentreCircle circle;
        std::vector<FieldLine> lines;
        std::vector<CornerPoint> corners;
        const GreenHorizon& gh = vbb->getGreenHorizon();

        // collect all vertical and horizontal line transition centres that exist under the green horizon
        BOOST_FOREACH(const ColourSegment& s, vbb->getVerticalTransitions(LINE_COLOUR)) {
            temp.screenCartesian = s.getCentre();
            if(gh.isBelowHorizon(temp.screenCartesian))
                points.push_back(temp);
        }
        BOOST_FOREACH(const ColourSegment& s, vbb->getHorizontalTransitions(LINE_COLOUR)) {
            temp.screenCartesian = s.getCentre();
            if(gh.isBelowHorizon(temp.screenCartesian))
                points.push_back(temp);
        }

        #if VISION_FIELDPOINT_VERBOSITY > 1
        std::vector<Point> plotpts;
        BOOST_FOREACH(const NUPoint& g, points) {
            plotpts.push_back(g.screenCartesian);
        }
        DataWrapper::getInstance()->plotCurve("Screencoords", plotpts);
        #endif
        //map those points to the ground plane
        transformer.calculateRepresentationsFromPixelLocation(points);

        #if VISION_FIELDPOINT_VERBOSITY > 1
        plotpts.clear();
        BOOST_FOREACH(const NUPoint& g, points) {
            plotpts.push_back(g.groundCartesian);
        }
        DataWrapper::getInstance()->plotCurve("Groundcoords", plotpts);
        #endif

        if(find_circle && m_circle_detector) {
            //first attempt to find a centre circle
            if(m_circle_detector->run(points, circle)) {
                //circle found - points are already removed
                vbb->addCentreCircle(circle);

                #if VISION_FIELDPOINT_VERBOSITY > 1
                vector<Point> circle_pts;
                double r = circle.getGroundRadius();
                Point c = circle.getLocation().groundCartesian;
                for(double theta = -mathGeneral::PI; theta < mathGeneral::PI; theta+=0.1)
                    circle_pts.push_back(Vector2<double>(r*cos(theta), r*sin(theta)) + c);
                DataWrapper::getInstance()->plotCurve("Centrecircle", circle_pts);
                #endif
            }
        }

        if(find_lines && m_line_detector) {
            lines = m_line_detector->run(points);

            #if VISION_FIELDPOINT_VERBOSITY > 1
            plotpts.clear();
            BOOST_FOREACH(const FieldLine& l, lines) {
                Vector2<NUPoint> ep = l.getEndPoints();
                plotpts.push_back(ep[0].groundCartesian);
                plotpts.push_back(ep[1].groundCartesian);
            }
            DataWrapper::getInstance()->plotLineSegments("Lines", plotpts);
            #endif

            vbb->addLines(lines);
        }

        //now find corners
        if(find_corners && m_corner_detector) {
            corners = m_corner_detector->run(lines);

            #if VISION_FIELDPOINT_VERBOSITY > 1
            plotpts.clear();
            BOOST_FOREACH(CornerPoint& cp, corners) {
                plotpts.push_back(cp.getLocation().groundCartesian);
            }
            DataWrapper::getInstance()->plotCurve("Corners", plotpts);
            #endif

            vbb->addCornerPoints(corners);
        }
    }
}
