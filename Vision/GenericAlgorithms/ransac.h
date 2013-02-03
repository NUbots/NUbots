#include <vector>
//#include "Tools/Math/LSFittedLine.h"

using std::vector;
using std::pair;

namespace RANSAC
{
//    vector<LSFittedLine> findMultipleLines(const vector<Point>& line_points,
//                                           float e,                     //typical:
//                                           unsigned int n,              //typical:
//                                           unsigned int k,              //typical:
//                                           unsigned int max_iterations);

//    bool findLine(vector<Point> points,
//                  Line& result,
//                  vector<Point>& consensus,
//                  vector<Point>& remainder,
//                  float& variance,
//                  float e,
//                  unsigned int n,
//                  unsigned int k);

//    Line generateRandomLine(const vector<Point>& points);

    //Model must provide several features
    template<class Model, typename DataPoint>
    vector<pair<Model, vector<DataPoint> > > findMultipleModels(const vector<DataPoint>& line_points,
                                                                float e,
                                                                unsigned int n,
                                                                unsigned int k,
                                                                unsigned int max_iterations);

    template<class Model, typename DataPoint>
    bool findModel(vector<DataPoint> points,
                   Model& result,
                   vector<DataPoint>& consensus,
                   vector<DataPoint>& remainder,
                   float& variance,
                   float e,
                   unsigned int n,
                   unsigned int k);

    template<class Model, typename DataPoint>
    Model generateRandomModel(const vector<DataPoint>& points);
}

#include "ransac.template"
