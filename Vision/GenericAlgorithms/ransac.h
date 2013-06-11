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
    std::vector<pair<Model, std::vector<DataPoint> > > findMultipleModels(const std::vector<DataPoint>& line_points,
                                                                double e,
                                                                unsigned int n,
                                                                unsigned int k,
                                                                unsigned int max_iterations,
                                                                SELECTION_METHOD method);

    template<class Model, typename DataPoint>
    bool findModel(std::vector<DataPoint> points,
                   Model& result,
                   std::vector<DataPoint>& consensus,
                   std::vector<DataPoint>& remainder,
                   double& variance,
                   double e,
                   unsigned int n,
                   unsigned int k,
                   SELECTION_METHOD method);

    template<class Model, typename DataPoint>
    Model generateRandomModel(const std::vector<DataPoint>& points);
}

#include "ransac.template"
