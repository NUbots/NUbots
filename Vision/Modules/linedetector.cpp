#include "linedetector.h"
#include "boost/foreach.hpp"
#include "Tools/Math/LSFittedLine.h"

LineDetector::LineDetector()
{
}

vector<LinePoint> LineDetector::getPointsFromSegments(const vector<ColourSegment> &h_segments, const vector<ColourSegment> &v_segments)
{
    vector<LinePoint> points;
    BOOST_FOREACH(ColourSegment s, h_segments) {
        points.push_back(LinePoint(s.getCentre().x,s.getCentre().y));
    }
    BOOST_FOREACH(ColourSegment s, v_segments) {
        points.push_back(LinePoint(s.getCentre().x,s.getCentre().y));
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
