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

        if(s1.y > s2.y) {
            Point p = s1;
            s1 = s2;
            s2 = p;
            double w = w1;
            w1 = w2;
            w2 = w;
        }
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

// WIDTH REGRESSION METHOD
//void RANSACGoal::fit(const vector<ColourSegment> &segments)
//{
//    BOOST_FOREACH(const ColourSegment& seg, segments) {
//        l.addPoint(seg.getCentre());
//    }

//    //reconstruct goal from fit data
//    l.getEndPoints(s1, s2);

//    if(s1.y > s2.y) {
//        Point p = s1;
//        s1 = s2;
//        s2 = p;
//    }
//    v = (s2 - s1).normalize();
//    len = (s2 - s1).abs();

//    //now generate regression for width
//    LSFittedLine width_fuction;
//    BOOST_FOREACH(const ColourSegment& seg, segments) {
//        double t = (seg.getCentre()-s1)*v/len;
//        //width_fuction.addPoint(Vector2<double>(t, seg.getLength()));        // JUST REGRESS WIDTH
//        width_fuction.addPoint(Vector2<double>(t, 2*l.getLinePointDistance(seg.getCentre()) + seg.getLength()));        // REGRESS WIDTH PLUS TWICE CENTRE DISTANCE
//    }

//    Vector2<double> e1, e2;
//    width_fuction.getEndPoints(e1, e2);

//    if(e1.x < e2.x) {   // here x is really t from earlier loop
//        w1 = e1.y;
//        w2 = e2.y;
//    }
//    else {
//        w1 = e2.y;
//        w2 = e1.y;
//    }

//    width_diff = w2 - w1;
//}

// WIDTH EXTRAPOLATION METHOD - FROM ORIGINAL RANSAC POINTS
//void RANSACGoal::fit(const vector<ColourSegment> &segments)
//{
//    BOOST_FOREACH(const ColourSegment& seg, segments) {
//        l.addPoint(seg.getCentre());
//    }
//    //reconstruct goal from fit data
//    Point new_s1, new_s2;
//    double new_w1, new_w2;
//    l.getEndPoints(new_s1, new_s2);

//    if( (s1-new_s1).squareAbs() > (s1-new_s1).squareAbs() ) {
//        Point temp = new_s1;
//        new_s1 = new_s2;
//        new_s1 = temp;
//    }

//    new_w1 = getInterpolatedWidth(new_s1);
//    new_w2 = getInterpolatedWidth(new_s2);

//    s1 = new_s1;
//    s2 = new_s2;
//    w1 = abs(new_w1);
//    w2 = abs(new_w2);
//    if(s1.y > s2.y) {
//        Point p = s1;
//        s1 = s2;
//        s2 = p;
//        double w = w1;
//        w1 = w2;
//        w2 = w;
//    }

//    width_diff = w2 - w1;
//    v = (s2 - s1).normalize();
//    len = (s2 - s1).abs();
//}

// WIDTH EXTRAPOLATION METHOD - FROM TWO MAXIMUM WIDTH POINTS
void RANSACGoal::fit(const vector<ColourSegment> &segments)
{
    if(segments.size() >= 2) {
        std::pair<Point, double> best1(Point(), -1),
                                 best2(Point(), -1);
        BOOST_FOREACH(const ColourSegment& seg, segments) {
            l.addPoint(seg.getCentre());

            double len = seg.getLength();
            if(len > best1.second) {
                best1.second = len;
                best1.first = seg.getCentre();
            }
            else if(len > best2.second) {
                best2.second = len;
                best2.first = seg.getCentre();
            }
        }

        //project maximal width points onto line
        best1.first = l.projectOnto(best1.first);
        best2.first = l.projectOnto(best2.first);

        //reconstruct goal from fit data
        l.getEndPoints(s1, s2);

        //extrapolate maximal width to end points
        Vector2<double> width_interpolation_vector = (best1.first - best2.first).normalize()*( best1.second - best2.second )/(best1.first - best2.first).abs();
        w1 = abs( best2.second + (s1 - best2.first)*width_interpolation_vector );
        w2 = abs( best2.second + (s2 - best2.first)*width_interpolation_vector );

        if(s1.y > s2.y) {
            Point p = s1;
            s1 = s2;
            s2 = p;
            double w = w1;
            w1 = w2;
            w2 = w;
        }

        width_diff = w2 - w1;
        v = (s2 - s1).normalize();
        len = (s2 - s1).abs();
    }
}
