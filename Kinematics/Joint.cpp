#include "Joint.h"
using namespace TransformMatrices;
Joint::Joint(const TransformMatrices::DHParameters& jointParameters, const std::string& name):
        jointName(name), parameters(jointParameters)
{
}


Joint::~Joint()
{

}

Matrix Joint::calculateTransform(double angle)
{
    if(angle != bufferedAngle)
    {
        bufferedTransform = ModifiedDH(parameters, angle);
        bufferedAngle = angle;
    }
    return bufferedTransform;
}
