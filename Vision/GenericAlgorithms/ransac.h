#include <vector>
//#include "Tools/Math/LSFittedLine.h"

using std::vector;
using std::pair;

namespace RANSAC
{
    enum SELECTION_METHOD {
        LargestConsensus,
        BestFittingConsensus
    };

    //Model must provide several features
    template<class Model, typename DataPoint>
    vector<pair<Model, vector<DataPoint> > > findMultipleModels(const vector<DataPoint>& line_points,
                                                                double e,
                                                                unsigned int n,
                                                                unsigned int k,
                                                                unsigned int max_iterations,
                                                                SELECTION_METHOD method);

    template<class Model, typename DataPoint>
    bool findModel(vector<DataPoint> points,
                   Model& result,
                   vector<DataPoint>& consensus,
                   vector<DataPoint>& remainder,
                   double& variance,
                   double e,
                   unsigned int n,
                   unsigned int k,
                   SELECTION_METHOD method);

    template<class Model, typename DataPoint>
    Model generateRandomModel(const vector<DataPoint>& points);
}

#include "ransac.template"
