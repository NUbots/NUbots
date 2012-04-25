/**
*   @name   HorizonInterpolate
*   @file   horizoninterpolate.h
*   @brief  interpolate green horizon points onto a larger number of vertical segments.
*   @author David Budden
*   @date   21/02/2012
*/

#include "horizoninterpolate.h"
#include "debug.h"
/*!
*   Will not interpolate properly if VER_SEGMENTS is smaller than the number of original scanlines
*/
void HorizonInterpolate::interpolate(const unsigned int VER_SEGMENTS)
{
    VisionBlackboard* vbb = VisionBlackboard::getInstance();  // blackboard instance
    const vector<PointType> horizon_points = vbb->getHorizonPoints();

    int width = vbb->getImageWidth();

    vector<PointType> inter_points;
    inter_points.reserve(VER_SEGMENTS);

    int position, y_new;
    vector<PointType>::const_iterator it_start, it_end;
    it_start = horizon_points.begin();
    it_end = horizon_points.begin();
    it_end++;
    for (unsigned int i = 0; i <= VER_SEGMENTS; i++) {
        position = i*(width-1)/VER_SEGMENTS;
        // consider hull points either side of current x value
        while (position > it_end->x) {
            it_start++;
            it_end++;
        }
        // calculate y value for interpolated point
        y_new = static_cast<float>(it_end->y - it_start->y)/(it_end->x - it_start->x)*(position - it_start->x) + it_start->y;
        
        if(y_new >= vbb->getImageHeight())
            errorlog << "interpolate: " << y_new << " it_start: " << *it_start << " it_end: " << *it_end << " pos: " << position << endl;
        inter_points.push_back(PointType(position, y_new));
    }

    // set hull points (overwrite original)
    vbb->setHullPoints(inter_points);
}

