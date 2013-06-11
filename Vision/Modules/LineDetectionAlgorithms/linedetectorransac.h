#ifndef LINEDETECTORRANSAC_H
#define LINEDETECTORRANSAC_H

#include <vector>
#include "Vision/Modules/linedetector.h"

class LineDetectorRANSAC : public LineDetector
{
public:
    LineDetectorRANSAC();
    virtual std::vector<FieldLine> run(const std::vector<GroundPoint> &points);

private:
    unsigned int m_n,
                 m_k,
                 m_max_iterations;
    float m_e;
};

#endif // LINEDETECTORRANSAC_H
