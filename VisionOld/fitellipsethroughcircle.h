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
    Vector3<float> CalculateScreenPosition(std::vector<LinePoint*> centreCirclePoints);
    bool isThisAGoodFit();

    float cx;
    float cy;
    float r1;
    float r2;
    float theta;
    float sd;
    float r;

    //Relative Points:
    float relDistance;
    float relBearing;
    float relElevation;
    float relCx;
    float relCy;
};

#endif // FITELLIPSETHROUGHCIRCLE_H
