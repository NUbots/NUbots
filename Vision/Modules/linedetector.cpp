#include "linedetector.h"
#include "Vision/visionblackboard.h"
#include "boost/foreach.hpp"
#include "Tools/Math/LSFittedLine.h"

LineDetector::LineDetector()
{
    m_method = SAM;
}

LineDetector::LineDetector(METHOD method)
{
    m_method = method;
}

void LineDetector::run()
{
    VisionBlackboard* vbb = VisionBlackboard::getInstance();

    vector<ColourSegment> v_segments = vbb->getVerticalTransitions(VisionFieldObject::LINE_COLOUR);  //get transitions associated with lines
    vector<ColourSegment> h_segments = vbb->getHorizontalTransitions(VisionFieldObject::LINE_COLOUR);
    vector<LSFittedLine> lines;
    vector<LinePoint> points;

    points = getPointsFromSegments(h_segments, v_segments);

    points = pointsUnderGreenHorizon(points, vbb->getGreenHorizon());

    switch(m_method) {
        case SAM:
            lines = m_SAM.run(points, true);
            break;
        case RANSAC:
            break;
    }

//    BOOST_FOREACH(LSFittedLine l, lines) {
//        cout << l->getA() << "x + " << l->getB() << "y = " << l->getC() << " - r2tls: " << l->getr2tls() << " - msd: " << l->getMSD() << " - #points: " << l->numPoints << std::endl;
//    }

    vbb->addLines(lines);
}

vector<LinePoint> LineDetector::getPointsFromSegments(const vector<ColourSegment> &h_segments, const vector<ColourSegment> &v_segments)
{
    vector<LinePoint> points;
    LinePoint pt;
    BOOST_FOREACH(ColourSegment s, h_segments) {
        pt.x = s.getCentre().x;
        pt.y = s.getCentre().y;
        points.push_back(pt);
    }
    BOOST_FOREACH(ColourSegment s, v_segments) {
        pt.x = s.getCentre().x;
        pt.y = s.getCentre().y;
        points.push_back(pt);
    }

    return points;
}

vector<LinePoint> LineDetector::pointsUnderGreenHorizon(const vector<LinePoint>& points, const GreenHorizon& gh)
{
    vector<LinePoint> under;
    BOOST_FOREACH(LinePoint p, points) {
        if(gh.isBelowHorizon(PointType(p.x, p.y))) {
            under.push_back(p);
        }
    }
    return under;
}
