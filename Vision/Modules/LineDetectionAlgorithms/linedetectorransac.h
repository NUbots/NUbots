#ifndef LINEDETECTORRANSAC_H
#define LINEDETECTORRANSAC_H

#include <vector>
#include "Tools/Math/LSFittedLine.h"
#include "Vision/Modules/linedetector.h"

class LineDetectorRANSAC : public LineDetector
{
public:
    LineDetectorRANSAC();
    void run();

private:
    unsigned int m_n,
                 m_k,
                 m_max_iterations;
    float m_e;
};

#endif // LINEDETECTORRANSAC_H
