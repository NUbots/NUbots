#include "Link.h"
using namespace TransformMatrices;
Link::Link(const TransformMatrices::DHParameters& linkParameters, const std::string& name):
        jointName(name), parameters(jointParameters)
{
}


Link::~Link()
{

}

Matrix Link::calculateTransform(double angle)
{
    if(angle != bufferedAngle)
    {
        bufferedTransform = ModifiedDH(parameters, angle);
        bufferedAngle = angle;
    }
    return bufferedTransform;
}
