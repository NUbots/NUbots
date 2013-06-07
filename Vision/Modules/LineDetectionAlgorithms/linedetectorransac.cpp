#include "linedetectorransac.h"
#include "Vision/visionblackboard.h"
#include "Vision/visionconstants.h"
#include "Vision/GenericAlgorithms/ransac.h"
#include "Vision/VisionTypes/RANSACTypes/ransacline.h"
#include "debugverbosityvision.h"

#include <limits>
#include <stdlib.h>
#include <boost/foreach.hpp>

LineDetectorRANSAC::LineDetectorRANSAC()
{
    m_n = 15;               //min pts to line essentially
    m_k = 40;               //number of iterations per fitting attempt
    m_e = 4.0;              //consensus margin
    m_max_iterations = 10;  //hard limit on number of lines
}

vector<FieldLine> LineDetectorRANSAC::run(const vector<NUPoint>& points)
{
    vector< pair<RANSACLine<NUPoint>, vector<NUPoint> > > candidates;
    vector<pair<LSFittedLine, LSFittedLine> > linePairs;
    vector<FieldLine> finalLines;

    // find possible line candidates using RANSAC in the ground plane
    candidates = RANSAC::findMultipleModels<RANSACLine<NUPoint>, NUPoint>(points, m_e, m_n, m_k, m_max_iterations, RANSAC::BestFittingConsensus);

    /// @todo perhaps find amount of green along line and remove based on threshold?

    // generate line equations in the image plane
    for(size_t i=0; i<candidates.size(); i++) {
        pair<LSFittedLine, LSFittedLine> lp;
        BOOST_FOREACH(NUPoint& g, candidates.at(i).second) {
            //line pairs are ordered as such : (ground, screen)
            lp.first.addPoint(g.ground);
            lp.second.addPoint(g.screen);
        }
        linePairs.push_back(lp);
    }

    // merge approximately colinear lines
    mergeColinear(linePairs, VisionConstants::RANSAC_MAX_ANGLE_DIFF_TO_MERGE, VisionConstants::RANSAC_MAX_DISTANCE_TO_MERGE);

    // generate FieldLine type from ground and screen equations
    for(size_t i=0; i<linePairs.size(); i++) {
        // line pairs are ordered as such : (ground, screen)
        finalLines.push_back(FieldLine(linePairs.at(i).second, linePairs.at(i).first));
    }

    return finalLines;
}
