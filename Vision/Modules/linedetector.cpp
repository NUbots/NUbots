#include "linedetector.h"
#include "Vision/visionblackboard.h"
#include "boost/foreach.hpp"
#include "Tools/Math/LSFittedLine.h"

LineDetector::LineDetector()
{
}

void LineDetector::run()
{
    VisionBlackboard* vbb = VisionBlackboard::getInstance();

    vector<ColourSegment> v_segments = vbb->getVerticalTransitions(VisionFieldObject::LINE);  //get transitions associated with lines
    vector<ColourSegment> h_segments = vbb->getHorizontalTransitions(VisionFieldObject::LINE);
    vector<LSFittedLine*> lines;
    vector<LSFittedLine> result;
    vector<LinePoint*> points;

    points = getPointsFromSegments(h_segments, v_segments);

    points = pointsUnderGreenHorizon(points, vbb->getGreenHorizon());

    lines = m_SAM.run(points, true);

    BOOST_FOREACH(LSFittedLine* l, lines) {
        cout << l->getA() << "x + " << l->getB() << "y = " << l->getC() << " - r2tls: " << l->getr2tls() << " - msd: " << l->getMSD() << " - #points: " << l->numPoints;

        result.push_back(*l);
    }

    vbb->addLines(result);
}

vector<LinePoint*> LineDetector::getPointsFromSegments(const vector<ColourSegment> &h_segments, const vector<ColourSegment> &v_segments)
{
    vector<LinePoint*> points;
    BOOST_FOREACH(ColourSegment s, h_segments) {
        points.push_back(new LinePoint(s.getCentre().x, s.getCentre().y));
    }
    BOOST_FOREACH(ColourSegment s, v_segments) {
        points.push_back(new LinePoint(s.getCentre().x, s.getCentre().y));
    }

    return points;
}

vector<LinePoint*> LineDetector::pointsUnderGreenHorizon(const vector<LinePoint*> points, const GreenHorizon& gh)
{
    vector<LinePoint*> under;
    BOOST_FOREACH(LinePoint* p, points) {
        if(gh.isBelowHorizon(PointType(p->x, p->y))) {
            under.push_back(p);
        }
    }
    return under;
}
