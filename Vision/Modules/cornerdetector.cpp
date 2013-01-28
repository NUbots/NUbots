#include "cornerdetector.h"
#include "visionblackboard.h"
#include "Vision/VisionTypes/VisionFieldObjects/fieldline.h"

CornerDetector::CornerDetector()
{
}

vector<CornerPoint> CornerDetector::run() const
{
    const vector<FieldLine>& lines = VisionBlackboard::getInstance()->getLines();
    vector<FieldLine>::const_iterator it1, it2;
    vector<CornerPoint> results;

    double tolerance = 0.05; // 0 <= tol <= 1

    for(it1 = lines.begin(); it1 < lines.end()-1; it1++) {
        Line l1 = it1->getRelativeLineEquation();
        Vector2<Point> l1_pts = it1->getRelativeEndPoints();
        for(it2 = it1+1; it2 < lines.end(); it2++) {
            Line l2 = it2->getRelativeLineEquation();
            Vector2<Point> l2_pts = it2->getRelativeEndPoints();
            if(l1.getAngleBetween(l2) < tolerance*mathGeneral::PI*0.5) {
                //nearly perpendicular
                //now build corner from end points
            }
        }
    }

    return results;
}
