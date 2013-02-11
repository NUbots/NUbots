#include "linedetectorransac.h"
#include "Vision/visionblackboard.h"
#include "Vision/visionconstants.h"
#include "Vision/GenericAlgorithms/ransac.h"
#include "Vision/VisionTypes/RANSACTypes/ransacline.h"

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

vector<LSFittedLine> LineDetectorRANSAC::run()
{
    VisionBlackboard* vbb = VisionBlackboard::getInstance();

    vector<ColourSegment> v_segments = vbb->getVerticalTransitions(LINE_COLOUR);  //get transitions associated with lines
    vector<ColourSegment> h_segments = vbb->getHorizontalTransitions(LINE_COLOUR);
    vector< pair<RANSACLine, vector<Point> > > candidates;
    vector<LSFittedLine> lines;
    vector<Point> points;

    points = getPointsFromSegments(h_segments, v_segments);

    points = pointsUnderGreenHorizon(points, vbb->getGreenHorizon());

    vector<Point> plotpts;
    BOOST_FOREACH(Point& p, points) {
        plotpts.push_back(Point(p.x, vbb->getImageHeight()-p.y));
    }

    DataWrapper::getInstance()->plotPoints(plotpts, "Pre");  // debugging

    candidates = RANSAC::findMultipleModels<RANSACLine, Point>(points, m_e, m_n, m_k, m_max_iterations);
    for(unsigned int i=0; i<candidates.size(); i++) {
        lines.push_back(LSFittedLine(candidates.at(i).second));
    }
//    o << "lines" << endl;
//    o << lines << endl;
//    o << "colinear" << endl;
//    o << mergeColinear(lines, VisionConstants::RANSAC_MAX_ANGLE_DIFF_TO_MERGE, VisionConstants::RANSAC_MAX_DISTANCE_TO_MERGE) << endl;

    //debugging
    if(vbb->getTransformer().isScreenToGroundValid())
        points = vbb->getTransformer().screenToGroundCartesian(points);

    DataWrapper::getInstance()->plotPoints(points, "Post");  // debugging

//    //use generic ransac implementation to fine lines
//    candidates = RANSAC::findMultipleModels<VisionLine, Point>(points, m_e, m_n, m_k, m_max_iterations);
//    lines.clear();
//    for(unsigned int i=0; i<candidates.size(); i++) {
//        lines.push_back(LSFittedLine(candidates.at(i).second));
//    }

//    o << "lines" << endl;
//    o << lines << endl;
//    o << "colinear" << mergeColinear(lines, VisionConstants::RANSAC_MAX_ANGLE_DIFF_TO_MERGE, VisionConstants::RANSAC_MAX_DISTANCE_TO_MERGE) << endl;

//    o.close();

    return mergeColinear(lines, VisionConstants::RANSAC_MAX_ANGLE_DIFF_TO_MERGE, VisionConstants::RANSAC_MAX_DISTANCE_TO_MERGE);
}
