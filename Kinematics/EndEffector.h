#ifndef ENDEFFECTOR_H
#define ENDEFFECTOR_H
#include <vector>
#include "Tools/Math/Matrix.h"
#include "Link.h"

class EndEffector
{
    Matrix m_startTransform;
    std::vector<Link> m_links;
    std::vector<Matrix> m_transforms;
    Matrix m_endTransform;
    std::string m_name;

public:
    EndEffector(const Matrix& startTrans,
                const std::vector<Link>& endEffectorlinks,
                const Matrix& endTrans,
                const std::string& effectorName = std::string("Unknown"));
    Matrix CalculateTransform(std::vector<float> jointValues);
    void UpdateModel(std::vector<float> jointValues);
    Matrix EndPosition() const;
    std::string name() const {return m_name;}
    const std::vector<Link>* links() const {return &m_links;}
};

#endif // ENDEFFECTOR_H
