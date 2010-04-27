#ifndef LINK_H
#define LINK_H
#include "Tools/Math/TransformMatrices.h"
#include <string>

class Link
{
public:
    Link(const TransformMatrices::DHParameters& linkParameters, const std::string& name);
    ~Link();
    Matrix calculateTransform(double angle);
private:
    std::string linkName;
    TransformMatrices::DHParameters parameters;
    double bufferedAngle;
    Matrix bufferedTransform;
};

#endif // LINK_H
