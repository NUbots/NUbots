#ifndef FITELLIPSETHROUGHCIRCLE_H
#define FITELLIPSETHROUGHCIRCLE_H

#include "../Tools/Math/Vector3.h"
#include <vector>
class LinePoint;
class Vision;

class FitEllipseThroughCircle
{
public:
    FitEllipseThroughCircle();
    Vector3<float> DistanceToPoint(LinePoint* point, Vision* vision);
    bool Fit_Ellipse_Through_Circle(std::vector<LinePoint*> centreCirclePoints, Vision* vision);

    float cx;
    float cy;
    float r1;
    float r2;
    float theta;
    float sd;
};

#endif // FITELLIPSETHROUGHCIRCLE_H
