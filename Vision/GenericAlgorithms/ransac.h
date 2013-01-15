#include <vector>
#include "Tools/Math/LSFittedLine.h"

using std::vector;

namespace RANSAC
{

    vector<LSFittedLine> findMultipleLines(const vector<LinePoint>& line_points,
                                           float e,                     //typical:
                                           unsigned int n,              //typical:
                                           unsigned int k,              //typical:
                                           unsigned int max_iterations);

    bool findLine(std::vector<LinePoint> points,
                  Line& result,
                  vector<LinePoint>& consensus,
                  vector<LinePoint>& remainder,
                  float& variance,
                  float e,
                  unsigned int n,
                  unsigned int k);

    Line generateRandomLine(const vector<LinePoint>& points);
};
