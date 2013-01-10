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
    std::vector<LSFittedLine> fitLines(const std::vector<LinePoint>& line_points);
    bool findLine(std::vector<LinePoint> points, Line& result, std::vector<LinePoint>& consensus, std::vector<LinePoint>& remainder, float& variance);
    Line generateRandomLine(const std::vector<LinePoint>& points) const;

private:
    unsigned int m_n,
                 m_k,
                 m_max_iterations;
    float m_e;
};

#endif // LINEDETECTORRANSAC_H
