#ifndef GOALDETECTORRANSACCENTRES_H
#define GOALDETECTORRANSACCENTRES_H

#include "Vision/Modules/goaldetector.h"
#include <vector>

using std::vector;

class GoalDetectorRANSACCentres : public GoalDetector
{
public:
    GoalDetectorRANSACCentres();
    virtual vector<Goal> run();

private:
    unsigned int m_n,
                 m_k,
                 m_max_iterations;
    float m_e;
};
#endif // GOALDETECTORRANSACCENTRES_H
