#include "linedetector.h"
#include "boost/foreach.hpp"
#include "Tools/Math/LSFittedLine.h"

LineDetector::LineDetector()
{
}

vector<Point> LineDetector::getPointsFromSegments(const vector<ColourSegment> &h_segments, const vector<ColourSegment> &v_segments)
{
    vector<Point> points;
    BOOST_FOREACH(ColourSegment s, h_segments) {
        points.push_back(Point(s.getCentre().x,s.getCentre().y));
    }
    BOOST_FOREACH(ColourSegment s, v_segments) {
        points.push_back(Point(s.getCentre().x,s.getCentre().y));
    }

    return points;
}

vector<Point> LineDetector::pointsUnderGreenHorizon(const vector<Point>& points, const GreenHorizon& gh)
{
    vector<Point> under;
    BOOST_FOREACH(Point p, points) {
        if(gh.isBelowHorizon(p)) {
            under.push_back(p);
        }
    }
    return under;
}
