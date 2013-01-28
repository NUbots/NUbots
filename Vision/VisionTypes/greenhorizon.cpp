#include "greenhorizon.h"
#include "debug.h"
#include "debugverbosityvision.h"
#include "Vision/visionblackboard.h"

GreenHorizon::GreenHorizon()
{
}

GreenHorizon::GreenHorizon(const vector<Point>& initial_points)
{
    set(initial_points);
}

void GreenHorizon::set(const vector<Point> &initial_points)
{
    #if VISION_HORIZON_VERBOSITY > 1
        debug << "GreenHorizon::GreenHorizon - Begin" << endl;
    #endif
    VisionBlackboard* vbb = VisionBlackboard::getInstance();  // blackboard instance

    original_points = initial_points;
    interpolated_points.clear();
    int width = vbb->getImageWidth(),
        height = vbb->getImageHeight();

    //unsigned int position, y_new;
    int y_new;
    vector<Point>::const_iterator it_start, it_end;
    it_start = original_points.begin();
    it_end = it_start + 1;
    for (int i = 0; i < width; i++) {
        // consider hull points either side of current x value
        while (i > it_end->x) {
            it_start++;
            it_end++;
        }
        // calculate y value for interpolated point
        y_new = static_cast<float>(it_end->y - it_start->y)/(it_end->x - it_start->x)*(i - it_start->x) + it_start->y;

        #if VISION_HORIZON_VERBOSITY > 2
            debug << "GreenHorizon::set: x: " << i << " y: " << y_new << " it_start: " << *it_start << " it_end: " << *it_end << endl;
        #endif

        if(y_new >= height)
            errorlog << "GreenHorizon::set: " << y_new << " it_start: " << *it_start << " it_end: " << *it_end << endl;
        interpolated_points.push_back(Vector2<double>(i, y_new));
    }
}

int GreenHorizon::getYFromX(int x) const
{
    return interpolated_points.at(x).y;
}

bool GreenHorizon::isBelowHorizon(Point pt) const
{
    return pt.y > interpolated_points.at(pt.x).y;
}

const vector<Point>& GreenHorizon::getOriginalPoints() const
{
    return original_points;
}

const vector<Point>& GreenHorizon::getInterpolatedPoints() const
{
    return interpolated_points;
}

vector<Point> GreenHorizon::getInterpolatedSubset(unsigned int spacing) const
{
    vector<Point> subset;
    for(unsigned int i=0; i<interpolated_points.size(); i+=spacing) {
        subset.push_back(interpolated_points.at(i));
    }
    return subset;
}
