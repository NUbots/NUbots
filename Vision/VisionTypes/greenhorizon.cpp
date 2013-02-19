#include "greenhorizon.h"
#include "debug.h"
#include "debugverbosityvision.h"

GreenHorizon::GreenHorizon()
{
}

GreenHorizon::GreenHorizon(const vector<Point>& initial_points, Point image_size)
{
    set(initial_points, image_size);
}

void GreenHorizon::set(const vector<Point> &initial_points, Point image_size)
{
#if VISION_HORIZON_VERBOSITY > 1
    debug << "GreenHorizon::GreenHorizon - Begin" << std::endl;
#endif

    original_points = initial_points;
    interpolated_points.clear();

    //unsigned int position, y_new;
    int y_new;
    vector<Point>::const_iterator it_start, it_end;

    //generate start/end edge points (if not there)
    if(original_points.front().x > 0) {
        double y = interpolate(original_points.at(0), original_points.at(1), 0);
        //clamp to image vertical bounds
        y = std::max(y, 0.0);
        y = std::min(y, image_size.y);
        original_points.insert(original_points.begin(), Point(0, y));
    }
    if(original_points.back().x < image_size.x - 1) {
        double y = interpolate(original_points.at(original_points.size() - 2),
                               original_points.at(original_points.size() - 1),
                               image_size.x - 1);
        //clamp to image vertical bounds
        y = std::max(y, 0.0);
        y = std::min(y, image_size.y - 1);
        original_points.push_back(Point(image_size.x - 1, y));
    }

    it_start = original_points.begin();
    it_end = it_start + 1;
    for (int x = 0; x < image_size.x; x++) {
        // consider hull points either side of current x value
        while (x > it_end->x) {
            it_start++;
            it_end++;
        }
        // calculate y value for interpolated point
        y_new = static_cast<float>(it_end->y - it_start->y)/(it_end->x - it_start->x)*(x - it_start->x) + it_start->y;

        #if VISION_HORIZON_VERBOSITY > 2
            debug << "GreenHorizon::set: x: " << x << " y: " << y_new << " it_start: " << *it_start << " it_end: " << *it_end << std::endl;
        #endif

        if(y_new >= image_size.y)
            errorlog << "GreenHorizon::set: " << y_new << " it_start: " << *it_start << " it_end: " << *it_end << std::endl;
        interpolated_points.push_back(Point(x, y_new));
    }
}

double GreenHorizon::getYFromX(int x) const
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

double GreenHorizon::interpolate(Point p1, Point p2, double x) const
{
    return p1.y + (p2.y - p1.y) * (x - p1.x) / (p2.x - p1.x);
}

