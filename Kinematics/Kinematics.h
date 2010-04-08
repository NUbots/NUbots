#ifndef H_KINEMATICS_H_DEFINED
#define H_KINEMATICS_H_DEFINED
#include <vector>

class Kinematics
{
    public:
    //! TODO: Implement the below functions.
    //static void TransformPosition(double distance,double bearing,double elevation, double *transformedDistance,double *transformedBearing,double *transformedElevation);
    //static std::vector<float> TransformPosition(double distance,double bearing,double elevation);
    //static float CalculateHeightOfOrigin();
    static std::vector<float> CalculateRightFootPosition(float thetaHipYawPitch, float thetaHipRoll, float thetaHipPitch, float thetaKneePitch, float thetaAnklePitch, float thetaAnkleRoll);
    static std::vector<float> CalculateLeftFootPosition(float thetaHipYawPitch, float thetaHipRoll, float thetaHipPitch, float thetaKneePitch, float thetaAnklePitch, float thetaAnkleRoll);

// Nao measurement constants
    static const float cameraTopOffsetX;
    static const float cameraTopOffsetZ;
    static const float cameraTopOffsetAngle;
    static const float cameraBottomOffsetX;
    static const float cameraBottomOffsetZ;
    static const float cameraBottomOffsetAngle;

    static const float neckOffsetZ;
    static const float hipOffsetY;
    static const float hipOffsetZ;
    static const float thighLength;
    static const float tibiaLength;
    static const float footHeight;
};

#endif

