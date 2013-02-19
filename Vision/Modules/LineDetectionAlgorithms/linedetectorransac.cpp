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

vector<LSFittedLine> LineDetectorRANSAC::run(const vector<Point> &points)
{
    vector< pair<RANSACLine, vector<Point> > > candidates;
    vector<LSFittedLine> lines;

    candidates = RANSAC::findMultipleModels<RANSACLine, Point>(points, m_e, m_n, m_k, m_max_iterations, RANSAC::BestFittingConsensus);
    for(unsigned int i=0; i<candidates.size(); i++) {
        lines.push_back(LSFittedLine(candidates.at(i).second));
    }

    return mergeColinear(lines, VisionConstants::RANSAC_MAX_ANGLE_DIFF_TO_MERGE, VisionConstants::RANSAC_MAX_DISTANCE_TO_MERGE);
}
