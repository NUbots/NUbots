#include "linedetectorransac.h"
#include "Vision/visionblackboard.h"
#include "Vision/visionconstants.h"
#include "Vision/GenericAlgorithms/ransac.h"

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

void LineDetectorRANSAC::run()
{
    VisionBlackboard* vbb = VisionBlackboard::getInstance();

    vector<ColourSegment> v_segments = vbb->getVerticalTransitions(VisionFieldObject::LINE_COLOUR);  //get transitions associated with lines
    vector<ColourSegment> h_segments = vbb->getHorizontalTransitions(VisionFieldObject::LINE_COLOUR);
    vector<LSFittedLine> lines;
    vector<LinePoint> points;
    vector<LSFittedLine>::iterator l_it;

    points = getPointsFromSegments(h_segments, v_segments);

    points = pointsUnderGreenHorizon(points, vbb->getGreenHorizon());

    //use generic ransac implementation to fine lines
    lines = RANSAC::findMultipleLines(points, m_e, m_n, m_k, m_max_iterations);

    for(l_it = lines.begin(); l_it<lines.end(); l_it++) {
        FieldLine l(*l_it);
        vbb->addLine(l);
    }
}
