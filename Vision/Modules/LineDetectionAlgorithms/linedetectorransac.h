#ifndef LINEDETECTORRANSAC_H
#define LINEDETECTORRANSAC_H

#include <vector>
#include "Vision/Modules/linedetector.h"

class LineDetectorRANSAC : public LineDetector
{
public:
    LineDetectorRANSAC();
    virtual vector<FieldLine> run(const vector<NUPoint> &points);

private:
    unsigned int m_n,
                 m_k,
                 m_max_iterations;
    float m_e;
};

#endif // LINEDETECTORRANSAC_H
