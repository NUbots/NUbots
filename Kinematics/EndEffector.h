#ifndef ENDEFFECTOR_H
#define ENDEFFECTOR_H
#include <vector>
#include "Tools/Math/Matrix.h"
#include "Link.h"

class EndEffector
{
    Matrix m_startTransform;
    std::vector<Link> m_links;
    Matrix m_endTransform;
    std::string m_name;

public:
    EndEffector(const Matrix& startTrans,
                const std::vector<Link>& endEffectorlinks,
                const Matrix& endTrans,
                const std::string& effectorName = std::string("Unknown"));
    Matrix CalculateTransform(std::vector<float> jointValues);
    std::string Name() {return m_name;};
};

#endif // ENDEFFECTOR_H
