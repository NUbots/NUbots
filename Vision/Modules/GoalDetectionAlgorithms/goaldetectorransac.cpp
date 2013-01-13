#include "goaldetectorransac.h"
#include "Vision/visionblackboard.h"
#include "Vision/visionconstants.h"
#include "Vision/GenericAlgorithms/ransac.h"

#include <limits>
#include <stdlib.h>
#include <boost/foreach.hpp>

//for debugging only
#include "Infrastructure/NUImage/ColorModelConversions.h"

float test_e;

GoalDetectorRANSAC::GoalDetectorRANSAC()
{
    m_n = 8;               //min pts to line essentially
    m_k = 40;               //number of iterations per fitting attempt
    m_e = 8.0;              //consensus margin
    m_max_iterations = 6;  //hard limit on number of fitting attempts
}

void updateE(int big_e, void * object) {
    test_e = big_e/20.0;
}

void GoalDetectorRANSAC::run()
{
//    // ---- debugging
//    char c = 0;
//    cv::namedWindow("GoalRansac");
//    cv::createTrackbar("n", "GoalRansac", &m_n, 40);
//    cv::createTrackbar("k", "GoalRansac", &m_k, 100);
//    cv::createTrackbar("max_it", "GoalRansac", &m_max_iterations, 20);
//    cv::createTrackbar("e", "GoalRansac", 0, 100, updateE);
//    while (c != 27){
//        //---- end debugging
        VisionBlackboard* vbb = VisionBlackboard::getInstance();

        //get transitions associated with goals
        vector<ColourSegment> v_segments = vbb->getVerticalTransitions(VisionFieldObject::GOAL_Y_COLOUR);
        vector<ColourSegment> h_segments = vbb->getHorizontalTransitions(VisionFieldObject::GOAL_Y_COLOUR);
        vector<LSFittedLine> lines;
        vector<LinePoint> h_points;
        vector<LinePoint> v_points;
        vector<LSFittedLine>::iterator l_it;

        h_points = getEdgePointsFromSegments(h_segments);
        v_points = getEdgePointsFromSegments(v_segments);

        //use generic ransac implementation to fine lines
//        lines = RANSAC::findMultipleLines(h_points, m_e, m_n, m_k, m_max_iterations);

        h_points.insert(h_points.end(), v_points.begin(), v_points.end());
        lines = RANSAC::findMultipleLines(h_points, m_e, m_n, m_k, m_max_iterations);



        //MAKE GOALS FROM LINES


//        // ---- debugging
//        const NUImage& img = vbb->getOriginalImage();
//        cv::Mat mat(img.getHeight(), img.getWidth(), CV_8UC3);
//        unsigned char* ptr,
//                r, g, b;

//        for(int y=0; y<img.getHeight(); y++) {
//            ptr = mat.ptr<unsigned char>(y);
//            for(int x=0; x<img.getWidth(); x++) {
//                ColorModelConversions::fromYCbCrToRGB(img(x,y).y, img(x,y).cb, img(x,y).cr, r, g, b);
//                ptr[3*x]   = b;
//                ptr[3*x+1] = g;
//                ptr[3*x+2] = r;
//            }
//        }

//        for(l_it = lines.begin(); l_it<lines.end(); l_it++) {
//            FieldLine l(*l_it);
//            l.render(mat);
//        }
//        cv::imshow("GoalRansac", mat);
//        c = cv::waitKey(1);
//        //---- end debugging
//    }

}

vector<LinePoint> GoalDetectorRANSAC::getEdgePointsFromSegments(const vector<ColourSegment> &segments)
{
    vector<LinePoint> points;
    BOOST_FOREACH(ColourSegment s, segments) {
        points.push_back(LinePoint(s.getStart().x, s.getStart().y));
        points.push_back(LinePoint(s.getEnd().x, s.getEnd().y));
    }

    return points;
}
