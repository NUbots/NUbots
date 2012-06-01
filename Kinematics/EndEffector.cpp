#include "EndEffector.h"
#include "debug.h"
#include <assert.h>

EndEffector::EndEffector(const Matrix& startTrans, const std::vector<Link>& endEffectorlinks, const Matrix& endTrans, const std::string& effectorName):
        m_startTransform(startTrans), m_links(endEffectorlinks), m_endTransform(endTrans), m_name(effectorName)
{
    m_transforms.resize(m_links.size());
}

void EndEffector::UpdateModel(std::vector<float> jointValues)
{
    Matrix result(m_startTransform);
    if(jointValues.size() != m_links.size())
    {
        errorlog << "EndEffector::CalculateTransform - Joint values do not match links. ";
        errorlog << m_links.size() << " Links but only " << jointValues.size() << " joint values given." << std::endl;
    }
    else
    {
        // Check stuff is ok.
        assert(m_links.size() == jointValues.size());
        assert(m_links.size() == m_transforms.size());
        // Make iterators
        std::vector<Link>::const_iterator endLinks = m_links.end();
        std::vector<Link>::iterator currLink = m_links.begin();
        std::vector<float>::iterator currJointAngle = jointValues.begin();
        std::vector<Matrix>::iterator currTransform = m_transforms.begin();
        // Loop through the joints.
        while (currLink != endLinks)
        {
            result = result * currLink->calculateTransform(*currJointAngle);
            (*currTransform) = result; // Save transform
            ++currLink;
            ++currJointAngle;
            ++currTransform;
        }
    }
}

Matrix EndEffector::EndPosition() const
{
    return m_transforms.back() * m_endTransform;
}

Matrix EndEffector::CalculateTransform(std::vector<float> jointValues)
{
    Matrix result(m_startTransform);
    if(jointValues.size() != m_links.size())
    {
        errorlog << "EndEffector::CalculateTransform - Joint values do not match links. ";
        errorlog << m_links.size() << " Links but only " << jointValues.size() << " joint values given." << std::endl;
    }
    else
    {
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
    }
    result = result * m_endTransform;
    return result;
}
