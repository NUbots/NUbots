#include "cornerdetector.h"
#include "Vision/visionblackboard.h"
#include "debug.h"
#include "debugverbosityvision.h"

CornerDetector::CornerDetector(double tolerance)
{
    setTolerance(tolerance);
}

void CornerDetector::setTolerance(double tolerance)
{
#if VISION_FIELDOBJECT_VERBOSITY > 1
    if(tolerance < 0 || tolerance > 1)
        debug << "CornerDetector::setTolerance - invalid tolerance: " << tolerance << " (must be in [0, 1]." << endl;
#endif

    m_tolerance = max(min(tolerance, 1.0), 0.0); //clamp
}

vector<CornerPoint> CornerDetector::run(const vector<FieldLine> &lines) const
{
    vector<FieldLine>::const_iterator it1, it2;
    vector<CornerPoint> results;

    if(lines.size() < 2)
        return results;

    for(it1 = lines.begin(); it1 != lines.end()-1; it1++) {
        Line l1 = it1->getRelativeLineEquation();
        Vector2<Point> l1_pts = it1->getRelativeEndPoints();
        for(it2 = it1+1; it2 < lines.end(); it2++) {
            Line l2 = it2->getRelativeLineEquation();
            Vector2<Point> l2_pts = it2->getRelativeEndPoints();
            if(l1.getAngleBetween(l2) < m_tolerance*mathGeneral::PI*0.5) {
                //nearly perpendicular
                //now build corner from end points
                Point intersection;
                if(l1.getIntersection(l2, intersection)) {
                    CornerPoint::TYPE type = findCorner(l1_pts, l2_pts, intersection, m_tolerance);
                    if(type != CornerPoint::INVALID) {
                        //need screen loc
                        Point screen_loc;
                        if(it1->getScreenLineEquation().getIntersection(it2->getScreenLineEquation(), screen_loc)) {
                            results.push_back(CornerPoint(type, screen_loc, Vector2<float>(intersection.x, intersection.y)));
                        }
                        else {
                            errorlog << "CornerDetector::run - no intersection found for screen lines - transforms are probably not valid, not publishing corner" <<
                                        "\t" << it1->getScreenLineEquation() << "\t" << it2->getScreenLineEquation() << endl;
                        }
                    }
                }
            }
        }
    }

    return results;
}

CornerPoint::TYPE CornerDetector::findCorner(Vector2<Point> ep1, Vector2<Point> ep2, Point intersection, double tolerance) const
{
    Point mid1 = (ep1.x + ep1.y)*0.5,
          mid2 = (ep2.x + ep2.y)*0.5;

    if(tolerance < 0 || tolerance > 1)
        errorlog << "CornerDetector::findCorner called with invalid tolerance: " << tolerance << " (must be in [0, 1]." << endl;

    //compare end points and midpoints to see what is closest to the intersection
    double d1x = (intersection - ep1.x).abs(),
           d1y = (intersection - ep1.y).abs(),
           d1m = (intersection - mid1).abs(),
           d2x = (intersection - ep2.x).abs(),
           d2y = (intersection - ep2.y).abs(),
           d2m = (intersection - mid2).abs();

    double min1 = min(d1m, min(d1x, d1y)),
           min2 = min(d2m, min(d2x, d2y));

    if(min1 < tolerance*(ep1.x - ep1.y).abs() && min2 < tolerance*(ep2.x - ep2.y).abs()) {
        //check distances are within tolerance of the length of the lines
        //perhaps do this later with only 1 line

        if(d1m == min1 && d2m == min2) {
            return CornerPoint::CROSS;
        }
        else if(d1m == min1 || d2m == min2){
            return CornerPoint::T;
        }
        else {
            return CornerPoint::L;
        }
    }
    else {
        return CornerPoint::INVALID;
    }
}
