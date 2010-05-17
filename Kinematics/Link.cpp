#include "Link.h"
using namespace TransformMatrices;
Link::Link(const TransformMatrices::DHParameters& linkParameters, const std::string& linkName):
        m_name(linkName), m_parameters(linkParameters)
{
    m_bufferedAngle = 0.0f;
    m_bufferedTransform = ModifiedDH(m_parameters, m_bufferedAngle);
}


Link::~Link()
{

}

Matrix Link::calculateTransform(double angle)
{
    if(angle != m_bufferedAngle)
    {
        m_bufferedTransform = ModifiedDH(m_parameters, angle);
        m_bufferedAngle = angle;
    }
    return m_bufferedTransform;
}
