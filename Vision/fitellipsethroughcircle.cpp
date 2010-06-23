#include "fitellipsethroughcircle.h"
#include "../Tools/Math/LSFittedLine.h"
#include <vector>
class Vision;
#include "../Tools/Math/Vector3.h"
#include "../Tools/Math/Vector2.h"
#include "CircleFitting.h"
#include "Tools/Math/General.h"
#include "NUPlatform/NUSensors/NUSensorsData.h"
#include "Kinematics/Kinematics.h"
#include "CircleFitting.h"
#include "Vision.h"
//#include <QDebug>
FitEllipseThroughCircle::FitEllipseThroughCircle()
{
    cx = 0;
    cy = 0;
    r1 = 0;
    sd = 0;
}


bool FitEllipseThroughCircle::Fit_Ellipse_Through_Circle(std::vector<LinePoint*> centreCirclePoints, Vision* vision)
{

    CircleFitting circleFitter;
    std::vector < Vector2<int> > points;
    points.reserve(centreCirclePoints.size());

    for(unsigned int i = 0; i < centreCirclePoints.size() ; i++ )
    {
        Vector3<float> relativePoint = DistanceToPoint(centreCirclePoints[i], vision);
        if(relativePoint.x == 100000)
        {
            //Fit_Ellipse(centreCirclePoints);
            return false;
        }
        else
        {
            Vector2<int> tempLinePoint;
            tempLinePoint.x = relativePoint.x * cos(relativePoint.y) * cos (relativePoint.z);
            tempLinePoint.y = relativePoint.x * sin(relativePoint.y) * cos (relativePoint.z);
            points.push_back(tempLinePoint);
            //qDebug() << "CenterCircle through Circle: Point Found: " << tempLinePoint.x << "," <<tempLinePoint.y;
        }
    }
    Circle circ = circleFitter.FitCircleLMA(points);
    LinePoint relativeCentrePoint;
    relativeCentrePoint.x = circ.centreX;
    relativeCentrePoint.y = circ.centreY;

    r1 = circ.radius;
    float reldistance = sqrt(relativeCentrePoint.x * relativeCentrePoint.x  + relativeCentrePoint.y *relativeCentrePoint.y);
    float bearing = atan2(relativeCentrePoint.y,relativeCentrePoint.x);
    //qDebug() << "CenterCircle through Circle: Distance: " << reldistance << "Bearing:" <<bearing << "Radius: " <<r1;
    return true;
}
Vector3<float> FitEllipseThroughCircle::DistanceToPoint(LinePoint* point, Vision* vision)
{
    //USING DISTANCE TO POINT
    //get the center point of the of GOAL Post:
    float D2Pdistance = 100000;
    Vector3<float> relativePoint;

    relativePoint.x = D2Pdistance;
    float bearing = vision->CalculateBearing(point->x);
    float elevation = vision->CalculateElevation(point->y);

    Matrix camera2groundTransform;
    bool isOK = vision->getSensorsData()->getCameraToGroundTransform(camera2groundTransform);
    if(isOK == true)
    {
        Vector3<float> result;
        result = Kinematics::DistanceToPoint(camera2groundTransform, bearing, elevation);
        relativePoint.x = result[0]; //DISTANCE
        relativePoint.y = result[1]; //BEARING
        relativePoint.z = result[2]; //ELEVATION

        #if DEBUG_VISION_VERBOSITY > 6
        debug << "\t\tELLIPISE::Calculated Distance to Point: " << relativePoint.x <<endl;
        #endif
    }
    return relativePoint;
}
/*
void FitEllipseThroughCircle::CalculateScreenPosition(float relativex, float relativey, Vision* vision)
{
    Matrix camera2groundTransform;

    bool isOK = vision->getSensorsData()->getCameraToGroundTransform(camera2groundTransform);
    if(isOK == true)
    {
        Matrix InverseCameraToGroundTransform = InverseMatrix(camera2groundTransform);
        Vector3<float> result;
        result = Kinematics::DistanceToPoint(InverseCameraToGroundTransform, bearing, elevation);
        relativePoint.x = result[0]; //DISTANCE
        relativePoint.y = result[1]; //BEARING
        relativePoint.z = result[2]; //ELEVATION

        #if DEBUG_VISION_VERBOSITY > 6
        debug << "\t\tELLIPISE::Calculated Distance to Point: " << relativePoint.x <<endl;
        #endif
    }
    return;

}
*/
