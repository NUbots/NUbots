#include "ransac.h"

#include <limits>
#include <stdlib.h>
#include <boost/foreach.hpp>

RANSAC::RANSAC()
{
    m_n = 35;
    m_k = 40;
    m_e = 8.0;
    m_max_iterations = 25;
}

std::vector<LSFittedLine> RANSAC::run(const std::vector<LinePoint>& line_points)
{
    float variance;
    Line line;
    LSFittedLine fitted_line;
    std::vector<LSFittedLine> results;
    std::vector<LinePoint> consensus;
    std::vector<LinePoint> remainder;

    bool line_found;
    do {
        line_found = findLine(line_points, line, consensus, remainder, variance);

    }
    while(false);
}

bool RANSAC::findLine(const std::vector<LinePoint>& points, Line& result, std::vector<LinePoint>& consensus, std::vector<LinePoint>& remainder, float& variance)
{
    if (points.size() < m_n) {
        return false;
    }

    // error of best line found so far
    float minerr = std::numeric_limits<float>::max();
    // arrays for storing concensus sets
    bool c1[points.size()];
    bool c2[points.size()];

    bool* best_concensus;
    bool* cur_concensus;
    best_concensus = c1;
    consensus.clear();

    for (unsigned int i = 0; i < m_k; ++i) {
      // randomly select 2 distinct points
        Line l = generateRandomLine(points);
        variance = 0;
        // figure out the variance (sum of distances of points from the line)
        // could use dist() here, but since the denominator is consistent, we
        // save time and implement it again here.
//      float denom = l.getNormaliser();
//      float newe = m_e*denom;
        float newe = m_e*l.getNormaliser();
        unsigned int concensus_size = 0;

        if (c1 == best_concensus)
           cur_concensus = c2;
        else
           cur_concensus = c1;

        for (unsigned int i = 0; i < points.size(); i++) {
            float dist = l.getLinePointDistance(points.at(i));
            if (dist < newe) {
                variance += dist;
                concensus_size++;
                cur_concensus[i] = true;
            }
            else {
                cur_concensus[i] = false;
            }
        }
//      variance /= denom;
        static float k = 0.2;
        variance = k*variance - concensus_size;
        if (variance < minerr && concensus_size >= m_n) {
            minerr = variance;
            result = l;
            best_concensus = cur_concensus;
        }
    }
    variance = variance/(points.size()*m_e);

    if (minerr < std::numeric_limits<float>::max()) {
        for(int i=0; i<points.size(); i++) {
            if(best_concensus[i])
                consensus.push_back(points.at(i));
            else {
                remainder.push_back(points.at(i));
            }
        }
        return true;
    }
    else {
        return false;
    }
}

Line RANSAC::generateRandomLine(const std::vector<LinePoint>& points) const
{
    if(points.size() > 1) {
        LinePoint p1, p2;
        p1 = points.at(rand() % points.size());
        do {
            p2 = points.at(rand() % points.size());
        } while (p1 == p2);
        return Line(p1, p2);
    }
    else {
        return Line();
    }
}
