#ifndef H_KINEMATICS_H_DEFINED
#define H_KINEMATICS_H_DEFINED
#include <vector>
#include <string>
#include "EndEffector.h"
#include "Tools/Math/Vector3.h"
#include "Tools/Math/Rectangle.h"
#include "debug.h"

class Kinematics
{
public:
    enum Effector
    {
        bottomCamera,
        topCamera,
        leftFoot,
        rightFoot,
        numEffectors
    };

    bool LoadModel(const std::string& fileName);
    Matrix CalculateTransform(Effector effectorId, const std::vector<float>& jointValues);

    static Matrix CalculateCamera2GroundTransform(const Matrix& origin2SupportLegTransform, const Matrix& origin2Camera);

    static Vector3<float> DistanceToPoint(const Matrix& Camera2GroundTransform, double angleFromCameraCentreX, double angleFromCameraCentreY);

    static std::vector<float> TransformPosition(const Matrix& Camera2GroundTransform, const std::vector<float>& cameraBasedPosition);
    static Vector3<float> TransformPosition(const Matrix& Camera2GroundTransform, const Vector3<float>& cameraBasedPosition);
    static Matrix TransformPosition(const Matrix& Camera2GroundTransform, const Matrix& cameraBasedPosition);

    static std::vector<float> LookToPoint(const std::vector<float>& pointFieldCoordinates);

    static std::vector<float> ReOrderKneckJoints(const std::vector<float>& joints)
    {
        const unsigned int numRequiredJoints = 2;
        std::vector<float> result(joints.size());
        if(joints.size() >= numRequiredJoints)
        {
            result[0] = joints[1];
            result[1] = joints[0];
        }
        else
        {
            errorlog << "Kinematics::ReOrderKneckJoints - Wrong number of joint values: Expected ";
            errorlog << numRequiredJoints << " Received " << joints.size() << "." << std::endl;
        }
        return result;
    };

    static std::vector<float> ReOrderLegJoints(const std::vector<float>& joints)
    {
        const unsigned int numRequiredJoints = 6;
        std::vector<float> result(joints.size());
        if(joints.size() >= numRequiredJoints)
        {
            result[0] = joints[2];
            result[1] = joints[0];
            result[2] = joints[1];
            result[3] = joints[3];
            result[4] = joints[5];
            result[5] = joints[4];
        }
        else
        {
            errorlog << "Kinematics::ReOrderLegJoints - Wrong number of joint values: Expected ";
            errorlog << numRequiredJoints << " Received " << joints.size() << "." << std::endl;
        }
        return result;
    };

    static std::vector<float> PositionFromTransform(const Matrix& transformMatrix)
    {
        std::vector<float> result(3,0.0f);
        result[0] = transformMatrix[0][3];
        result[1] = transformMatrix[1][3];
        result[2] = transformMatrix[2][3];
        return result;
    }

    static std::vector<float> OrientationFromTransform(const Matrix& transfromMatrix)
    {
        std::vector<float> result(3,0.0f);
        result[0] = asin(-transfromMatrix[2][1]);
        result[1] = atan2(transfromMatrix[2][0], transfromMatrix[2][2]);
        result[2] = atan2(transfromMatrix[0][1], transfromMatrix[1][1]);
        return result;
    }

    std::vector<EndEffector> m_endEffectors;

    Rectangle CalculateFootPosition(const Matrix& supportFootTransformMatrix,const Matrix& theFootTransformMatrix, Effector theFoot);
    double CalculateRelativeFootHeight(const Matrix& supportFootTransformMatrix,const Matrix& theFootTransformMatrix, Effector theFoot);
    float CalculateRadialLegLength(const std::vector<float>& legJoints);
    float CalculateHipPitchAngleForRelYPosition(const std::vector<float>& legJoints, float relYPos);

private:
            
    // Top camera
    float m_cameraTopOffsetZ;
    float m_cameraTopOffsetX;
    float m_cameraTopOffsetAngle;

    // Bottom Camera
    float m_cameraBottomOffsetZ;
    float m_cameraBottomOffsetX;
    float m_cameraBottomOffsetAngle;

    // Neck
    float m_neckOffsetZ;

    // Hips
    float m_hipOffsetY;
    float m_hipOffsetZ;

    // Legs
    float m_thighLength;
    float m_tibiaLength;
    float m_footHeight;

    // Feet
    float m_footInnerWidth;
    float m_footOuterWidth;
    float m_footForwardLength;
    float m_footBackwardLength;
};

#endif

