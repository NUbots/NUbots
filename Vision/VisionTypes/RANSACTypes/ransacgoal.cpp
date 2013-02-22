#include "ransacgoal.h"
#include <boost/foreach.hpp>

RANSACGoal::RANSACGoal() : s1(0,0), s2(0,0), w1(0), w2(0), width_diff(0), v(0,0), len(0)
{
}

bool RANSACGoal::regenerate(const vector<ColourSegment>& segments)
{
    if(segments.size() == 2) {
        s1 = segments[0].getCentre();
        s2 = segments[1].getCentre();
        w1 = segments[0].getLength();
        w2 = segments[1].getLength();
        width_diff = w2 - w1;
        v = (s2 - s1).normalize();
        len = (s2 - s1).abs();
        l.setLineFromPoints(s1, s2);
        return true;
    }
    else {
        return false;
    }
}

double RANSACGoal::calculateError(ColourSegment c) const
{
    Point p = c.getCentre();
    double d = l.getLinePointDistance(p);
    double w = getInterpolatedWidth(p);

    return d + abs(w - c.getLength());
}

double RANSACGoal::getInterpolatedWidth(Point p) const
{
    return w1 + (p-s1)*v*width_diff/len;
}

void RANSACGoal::fit(const vector<ColourSegment> &segments)
{
    BOOST_FOREACH(const ColourSegment& seg, segments) {
        l.addPoint(seg.getCentre());
    }
    //reconstruct goal from fit data
    Point new_s1, new_s2;
    double new_w1, new_w2;
    l.getEndPoints(new_s1, new_s2);
    new_w1 = getInterpolatedWidth(new_s1);
    new_w2 = getInterpolatedWidth(new_s2);

    s1 = new_s1;
    s2 = new_s2;
    w1 = new_w1;
    w2 = new_w2;
    width_diff = w2 - w1;
    v = (s2 - s1).normalize();
    len = (s2 - s1).abs();
}
