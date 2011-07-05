#include "fitellipsethroughcircle.h"
#include "../Tools/Math/LSFittedLine.h"
#include <vector>
class Vision;
#include "../Tools/Math/Vector3.h"
#include "../Tools/Math/Vector2.h"
#include "CircleFitting.h"
#include "Tools/Math/General.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Kinematics/Kinematics.h"
#include "Vision.h"

#if TARGET_OS_IS_WINDOWS
    #include <QDebug>
#endif
FitEllipseThroughCircle::FitEllipseThroughCircle()
{
    cx = 0;
    cy = 0;
    r = 0;
    sd = 0;
    
    relDistance = 0;
    relBearing = 0;
    relElevation = 0;
    relCx = 0;
    relCy = 0;
}


bool FitEllipseThroughCircle::Fit_Ellipse_Through_Circle(std::vector<LinePoint*> centreCirclePoints, Vision* vision)
{
    
    //This function assumes points are points that belong on the centre circle:
    
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
            tempLinePoint.x = relativePoint.x * cos(relativePoint.y);// * cos (relativePoint.z);
            tempLinePoint.y = relativePoint.x * sin(relativePoint.y);// * cos (relativePoint.z);
            points.push_back(tempLinePoint);
            //qDebug() << tempLinePoint.x << "," <<tempLinePoint.y;
            //qDebug() << centreCirclePoints[i]->x << "," <<centreCirclePoints[i]->y;
        }
    }
    
    //Perform Circle Fit on Transformed Points:
    Circle circ = circleFitter.FitCircleLMF(points);
    LinePoint relativeCentrePoint;
    relativeCentrePoint.x = circ.centreX;
    relativeCentrePoint.y = circ.centreY;
    relCx = circ.centreX;
    relCy = circ.centreY;
    //qDebug() << "CenterCircle Centre: "<< relativeCentrePoint.x  << " , "<<relativeCentrePoint.y;
    //Assign Relative Centre Circle points:
    r = circ.radius;
    relDistance = sqrt(relativeCentrePoint.x * relativeCentrePoint.x  + relativeCentrePoint.y *relativeCentrePoint.y);

    relBearing =  atan2(relativeCentrePoint.y,relativeCentrePoint.x);
    

    bool result = isThisAGoodFit();

    #if TARGET_OS_IS_WINDOWS
    if(result)
    {
        CalculateScreenPosition(centreCirclePoints);
    }
    #endif

    #if TARGET_OS_IS_WINDOWS
    qDebug() << "\t\tELLIPISEthroughCircle::Calculated Centre Circle: " << relDistance << "cm , "<< relBearing <<" rad. Elevation: " << relElevation <<" Radius: "<< r << "."<< result << endl;
    #endif
    #if DEBUG_VISION_VERBOSITY > 6
        debug << "\t\tELLIPISEthroughCircle::Calculated Centre Circle: " << relDistance << "cm , "<< relBearing <<" rad. Radius: " << r << "."<<endl;
    #endif
    return result;
}
bool FitEllipseThroughCircle::isThisAGoodFit()
{
    //USE KNOWN DIMENSIONS OF THE CIRCLE ON FIELD TO DECIDE IF ITS A GOOD FIT:
    //SPL2010 Rules: Centre Circle Diameter is 1200mm
    //Radius: 600mm = 60cm
    float ActualRadius = 60;
    float PercentageError = 30;
    if((r < ActualRadius*(1+PercentageError/100)) && (r > ActualRadius*(1-PercentageError/100)) )
    {

        sd = fabs(r-ActualRadius);
        #if TARGET_OS_IS_WINDOWS
        qDebug() << "\t\tELLIPISEthroughCircle::Centre Circle: SD :" << sd << "cm." <<endl;
        #endif
        return true;
    }
    else
    {
        return false;
    }
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

    vector<float> ctgvector;
    bool isOK = vision->getSensorsData()->get(NUSensorsData::CameraToGroundTransform, ctgvector);
    if(isOK == true)
    {
        Matrix camera2groundTransform = Matrix4x4fromVector(ctgvector);
        Vector3<float> result;

        result = Kinematics::DistanceToPoint(camera2groundTransform, bearing, elevation);
        relativePoint.x = result[0]; //DISTANCE
        relativePoint.y = result[1]; //BEARING
        relativePoint.z = result[2]; //ELEVATION


        #if DEBUG_VISION_VERBOSITY > 6
        debug << "\t\tELLIPISE::Calculated Distance to Point: " << relativePoint.x <<endl;
        #endif

        //#if TARGET_OS_IS_WINDOWS
        //qDebug() << "\t\t Point On Screen: " << point->x << point->y <<endl;
        //#endif
        //CalculateScreenPosition(relativePoint.x, relativePoint.y, vision);
    }
    return relativePoint;
}

Vector3<float>  FitEllipseThroughCircle::CalculateScreenPosition(std::vector<LinePoint*> centreCirclePoints)
{
    // A Function to obtain the screen position of points, for showing the point on screen
    // Easiest Way: Approximate the height, camera yaw, pitch and perform a trig opperation to obtain the screen position.
    // Easy Way: Average of all the points and use that as the "centre".

    Vector3<float> result;
    float sumX = 0;
    float sumY = 0;
    int minY = 10000;
    int maxY = 0;
    int minX = 10000;
    int maxX = 0;
    for(unsigned int i = 0; i<centreCirclePoints.size(); i++)
    {
        sumX += centreCirclePoints[i]->x;
        sumY += centreCirclePoints[i]->y;
        if( centreCirclePoints[i]->y > maxY)
        {
            maxY = centreCirclePoints[i]->y;
        }
        if( centreCirclePoints[i]->y < minY)
        {
            minY = centreCirclePoints[i]->y;
        }
        if( centreCirclePoints[i]->x > maxX)
        {
            maxX = centreCirclePoints[i]->x;
        }
        if( centreCirclePoints[i]->x < minX)
        {
            minX = centreCirclePoints[i]->x;
        }
    }
    cx = sumX/centreCirclePoints.size();
    cy = sumY/centreCirclePoints.size();
    r1 = maxX-minX;
    r2 = maxY-minY;

    return result;

}

