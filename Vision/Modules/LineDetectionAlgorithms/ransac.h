#ifndef RANSAC_H
#define RANSAC_H

#include <vector>
#include "Tools/Math/LSFittedLine.h"

class RANSAC
{
public:
    RANSAC();
    std::vector<LSFittedLine> run(const std::vector<LinePoint>& line_points);

private:
    bool findLine(const std::vector<LinePoint>& points, Line& result, std::vector<LinePoint>& consensus, std::vector<LinePoint>& remainder, float& variance);
    Line generateRandomLine(const std::vector<LinePoint>& points) const;
    LSFittedLine fitLine(const std::vector<LinePoint>& points) const;

private:
    unsigned int m_n,
                 m_k,
                 m_max_iterations;
    float m_e;
};

#endif // RANSAC_H
