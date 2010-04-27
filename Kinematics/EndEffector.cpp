#include "EndEffector.h"
#include "debug.h"

EndEffector::EndEffector()
{
}

Matrix EndEffector::CalculateTransform(std::vector<float> jointValues)
{
    Matrix result(startTransform);
    if(jointValues.size() != joints.size())
    {
        errorlog << "EndEffector::CalculateTransform - Joint values do not match joints." << std::endl;
    }

    return result;
}
