#ifndef H_KINEMATICS_H_DEFINED
#define H_KINEMATICS_H_DEFINED
#include <vector>
#include <string>
#include "EndEffector.h"

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

    static double DistanceToPoint(const Matrix& Camera2GroundTransform, double angleFromCameraCentreX, double angleFromCameraCentreY);

    static std::vector<float> TransformPosition(const std::vector<float>& cameraBasedPosition);

    static std::vector<float> LookToPoint(const std::vector<float>& pointFieldCoordinates);

    static std::vector<float> ReOrderKneckJoints(const std::vector<float>& joints)
    {
        std::vector<float> result(joints.size());
        result[0] = joints[1];
        result[1] = joints[0];
        return result;
    };

    static std::vector<float> ReOrderLegJoints(const std::vector<float>& joints)
    {
        std::vector<float> result(joints.size());
        result[0] = joints[2];
        result[1] = joints[0];
        result[2] = joints[1];
        result[3] = joints[3];
        result[4] = joints[5];
        result[5] = joints[4];
        return result;
    };
    std::vector<EndEffector> m_endEffectors;
};

#endif

