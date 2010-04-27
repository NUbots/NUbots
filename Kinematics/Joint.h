#ifndef JOINT_H
#define JOINT_H
#include "Tools/Math/TransformMatrices.h"
#include <string>

class Joint
{
public:
    Joint(const TransformMatrices::DHParameters& jointParameters, const std::string& name);
    ~Joint();
    Matrix calculateTransform(double angle);
private:
    std::string jointName;
    TransformMatrices::DHParameters parameters;
    double bufferedAngle;
    Matrix bufferedTransform;
};

#endif // JOINT_H
