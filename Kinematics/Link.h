#ifndef LINK_H
#define LINK_H
#include "Tools/Math/TransformMatrices.h"
#include <string>

class Link
{
public:
    Link(const TransformMatrices::DHParameters& linkParameters, const std::string& linkName = std::string("Unknown"));
    ~Link();
    Matrix calculateTransform(double angle);
    std::string Name() {return m_name;};
private:
    std::string m_name;
    TransformMatrices::DHParameters m_parameters;
    double m_bufferedAngle;
    Matrix m_bufferedTransform;
};

#endif // LINK_H
