#include "EndEffector.h"
#include "debug.h"

EndEffector::EndEffector(const Matrix& startTrans, const std::vector<Link>& endEffectorlinks, const Matrix& endTrans, const std::string& effectorName):
        m_startTransform(startTrans), m_links(endEffectorlinks), m_endTransform(endTrans), m_name(effectorName)
{
}

Matrix EndEffector::CalculateTransform(std::vector<float> jointValues)
{
    Matrix result(m_startTransform);
    if(jointValues.size() != m_links.size())
    {
        errorlog << "EndEffector::CalculateTransform - Joint values do not match links." << std::endl;
    }

    // Make iterators
    std::vector<Link>::const_iterator endLinks = m_links.end();
    std::vector<Link>::iterator currLink = m_links.begin();
    std::vector<float>::iterator currJointAngle = jointValues.begin();
    while (currLink != endLinks)
    {
        result = result * currLink->calculateTransform(*currJointAngle);
        ++currLink;
        ++currJointAngle;
    }
    result = result * m_endTransform;
    return result;
}
