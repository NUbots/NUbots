#include <vector>
#include "Tools/Math/LSFittedLine.h"

using std::vector;

namespace RANSAC
{
    vector<LSFittedLine> findMultipleLines(const vector<Point>& line_points,
                                           float e,                     //typical:
                                           unsigned int n,              //typical:
                                           unsigned int k,              //typical:
                                           unsigned int max_iterations);

    bool findLine(std::vector<Point> points,
                  Line& result,
                  vector<Point>& consensus,
                  vector<Point>& remainder,
                  float& variance,
                  float e,
                  unsigned int n,
                  unsigned int k);

    Line generateRandomLine(const vector<Point>& points);
};
