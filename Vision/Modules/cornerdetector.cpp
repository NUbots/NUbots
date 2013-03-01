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
#if VISION_FIELDPOINT_VERBOSITY > 0
    if(tolerance < 0 || tolerance > 1)
        debug << "CornerDetector::setTolerance - invalid tolerance: " << tolerance << " (must be in [0, 1]." << endl;
#endif

    m_tolerance = max(min(tolerance, 1.0), 0.0); //clamp
}

vector<CornerPoint> CornerDetector::run(const vector<FieldLine> &lines) const
{
    vector<FieldLine>::const_iterator it1, it2;
    vector<CornerPoint> results;

    if(lines.size() < 2) {
        return results;
    }

    if(m_tolerance < 0 || m_tolerance > 1) {
        errorlog << "CornerDetector::run called with invalid tolerance: " << m_tolerance << " (must be in [0, 1]." << endl;
        return results;
    }

    for(it1 = lines.begin(); it1 != lines.end()-1; it1++) {
        Line l1 = it1->getGroundLineEquation();
        Vector2<GroundPoint> l1_pts = it1->getEndPoints();
        for(it2 = it1+1; it2 < lines.end(); it2++) {
            Line l2 = it2->getGroundLineEquation();
            Vector2<GroundPoint> l2_pts = it2->getEndPoints();
            if(l1.getAngleBetween(l2) > (1-m_tolerance)*mathGeneral::PI*0.5) {
                //nearly perpendicular
                //now build corner from end points
                GroundPoint intersection;
                if(l1.getIntersection(l2, intersection.ground)) {
                    CornerPoint::TYPE type = findCorner(l1_pts, l2_pts, intersection, m_tolerance);
                    if(type != CornerPoint::INVALID) {
                        //need screen loc
                        if(it1->getScreenLineEquation().getIntersection(it2->getScreenLineEquation(), intersection.screen)) {

                            /// DEBUG
                            Point mid1 = (l1_pts[0].ground + l1_pts[1].ground)*0.5,
                                  mid2 = (l2_pts[0].ground + l2_pts[1].ground)*0.5;

                            //compare end points and midpoints to see what is closest to the intersection
                            double d1x = (intersection.ground - l1_pts[0].ground).abs(),
                                   d1y = (intersection.ground - l1_pts[1].ground).abs(),
                                   d1m = (intersection.ground - mid1).abs(),
                                   d2x = (intersection.ground - l2_pts[0].ground).abs(),
                                   d2y = (intersection.ground - l2_pts[1].ground).abs(),
                                   d2m = (intersection.ground - mid2).abs();

                            double min1 = min(d1m, min(d1x, d1y)),
                                   min2 = min(d2m, min(d2x, d2y));

                            /// END DEBUG

                            results.push_back(CornerPoint(type, intersection));
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

CornerPoint::TYPE CornerDetector::findCorner(Vector2<GroundPoint> ep1, Vector2<GroundPoint> ep2, GroundPoint intersection, double tolerance) const
{
    Point mid1 = (ep1[0].ground + ep1[1].ground)*0.5,
          mid2 = (ep2[0].ground + ep2[1].ground)*0.5;

    //compare end points and midpoints to see what is closest to the intersection
    double d1x = (intersection.ground - ep1[0].ground).abs(),
           d1y = (intersection.ground - ep1[1].ground).abs(),
           d1m = (intersection.ground - mid1).abs(),
           d2x = (intersection.ground - ep2[0].ground).abs(),
           d2y = (intersection.ground - ep2[1].ground).abs(),
           d2m = (intersection.ground - mid2).abs();

    double min1 = min(d1m, min(d1x, d1y)),
           min2 = min(d2m, min(d2x, d2y));

    //removed distance check
    //if(min1 < tolerance*(ep1[0].ground - ep1[1].ground).abs() && min2 < tolerance*(ep2[0].ground - ep2[1].ground).abs()) {
        //check distances are within tolerance of the length of the lines
        //perhaps do this later with only 1 line

        if(d1m == min1 && d2m == min2) {
            return CornerPoint::X;
        }
        else if(d1m == min1 || d2m == min2){
            return CornerPoint::T;
        }
        else {
            return CornerPoint::L;
        }
    //}
    //else {
    //    return CornerPoint::INVALID;
    //}
}
