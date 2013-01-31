#include "visionline.h"

VisionLine::VisionLine()
{

}

bool VisionLine::regenerate(const vector<Point>& pts)
{
    if(pts.size() == 2) {
        setLineFromPoints(pts.at(0), pts.at(1));
        return true;
    }
    else {
        return false;
    }
}

double VisionLine::calculateError(Point p) const
{
    return getLinePointDistance(p);
}
