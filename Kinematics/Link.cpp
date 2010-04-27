#include "Link.h"
using namespace TransformMatrices;
Link::Link(const TransformMatrices::DHParameters& linkParameters, const std::string& name):
        linkName(name), parameters(linkParameters)
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
