#include "fieldpointdetector.h"
#include "Vision/VisionTypes/coloursegment.h"
#include "Vision/VisionTypes/greenhorizon.h"
#include "Vision/visionblackboard.h"
#include <boost/foreach.hpp>

FieldPointDetector::FieldPointDetector(LineDetector *line_detector, CircleDetector *circle_detector, CornerDetector *corner_detector)
{
    m_line_detector = line_detector;
    m_circle_detector = circle_detector;
    m_corner_detector = corner_detector;
}

void FieldPointDetector::run() const
{
    VisionBlackboard* vbb = VisionBlackboard::getInstance();
    const Transformer& transformer = vbb->getTransformer();

    //check transforms are valid
    if(transformer.isScreenToGroundValid()) {
        vector<Point> points;
        Circle circle;
        vector<LSFittedLine> lines;
        vector<FieldLine> field_lines;
        vector<CornerPoint> corners;
        const GreenHorizon& gh = vbb->getGreenHorizon();
        //collect all vertical and horizontal line transition centres that exist under the green horizon
        BOOST_FOREACH(const ColourSegment& s, vbb->getVerticalTransitions(LINE_COLOUR)) {
            const Point& p = s.getCentre();
            if(gh.isBelowHorizon(p))
                points.push_back(p);
        }
        BOOST_FOREACH(const ColourSegment& s, vbb->getHorizontalTransitions(LINE_COLOUR)) {
            const Point& p = s.getCentre();
            if(gh.isBelowHorizon(p))
                points.push_back(p);
        }

        DataWrapper::getInstance()->plot("Screen coords", points);

        //map those points to the ground plane
        points = transformer.screenToGroundCartesian(points);

        DataWrapper::getInstance()->plot("Ground coords", points);

//        vector<Point> temppts;
//        BOOST_FOREACH(Point& p, points) {
//            if(p.abs() < 500)
//                temppts.push_back(p);
//        }
//        DataWrapper::getInstance()->plot(POINTS_PLOT, tempts, "Ground coords");

        if(m_circle_detector){
            //first attempt to find a centre circle
            if(m_circle_detector->run(points, circle)) {
                //circle found - points are already removed
                vbb->addCentreCircle(CentreCircle(circle));

                vector<Point> circle_pts;
                double r = circle.getRadius();
                Point c = circle.getCentre();
                for(double theta = -mathGeneral::PI; theta < mathGeneral::PI; theta+=0.1)
                    circle_pts.push_back(Point(r*cos(theta), r*sin(theta)) + c);
                DataWrapper::getInstance()->plot("Centre circle", circle_pts);
            }
        }

        if(m_line_detector)
            lines = m_line_detector->run(points);

        BOOST_FOREACH(LSFittedLine& relative, lines) {
            //need to map lines back to screen before adding
            LSFittedLine screen;
            field_lines.push_back(FieldLine(screen, relative));
        }

        vbb->addLines(field_lines);

        //now find corners
        if(m_corner_detector)
            corners = m_corner_detector->run(field_lines);

        vbb->addCornerPoints(corners);
    }
}
