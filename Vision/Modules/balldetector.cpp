#include "balldetector.h"
#include "Vision/visionconstants.h"
#include "debug.h"
#include "debugverbosityvision.h"

////for stat
//#include <boost/foreach.hpp>
//#include <boost/accumulators/accumulators.hpp>
//#include <boost/accumulators/statistics/stats.hpp>
//#include <boost/accumulators/statistics/mean.hpp>
//#include <boost/accumulators/statistics/variance.hpp>

//using namespace boost::accumulators;

BallDetector::BallDetector() {}

BallDetector::~BallDetector() {}

// BROKEN
//vector<Ball> BallDetector::run()
//{
//    VisionBlackboard* vbb = VisionBlackboard::getInstance();
//    NUImage img = vbb->getOriginalImage();
//    const LookUpTable& lut = vbb->getLUT();
//    // BEGIN BALL DETECTION -----------------------------------------------------------------

//    std::vector<ColourSegment> v_segments = vbb->getVerticalTransitions(BALL_COLOUR);
//    std::vector<ColourSegment> h_segments = vbb->getHorizontalTransitions(BALL_COLOUR);
//    std::vector<Point> edges;
//    std::vector<Ball> balls; //will only ever hold one

//    appendEdgesFromSegments(h_segments, edges);
//    appendEdgesFromSegments(v_segments, edges);

//    #if VISION_BALL_VERBOSITY > 1
//        debug << "BallDetector::detectBall() - number of vertical ball segments: " << v_segments.size() << std::endl;
//        debug << "BallDetector::detectBall() - number of horizontal ball segments: " << h_segments.size() << std::endl;
//    #endif

//    const GreenHorizon& green_horizon = vbb->getGreenHorizon();

//    #if VISION_BALL_VERBOSITY > 1
//    int debug_edges_size = edges.size();
//    #endif

//    // Throw out points above the horizon
//    std::vector<Point>::iterator it;
//    it = edges.begin();
//    while (it < edges.end()) {
//        if(green_horizon.isBelowHorizon(*it))
//            it++;   //move to next edge
//        else
//            it = edges.erase(it);
//    }

//    #if VISION_BALL_VERBOSITY > 1
//    debug << "BallDetector::detectBall() - " << debug_edges_size - edges.size() << " ball segments above green horizon." << std::endl;
//    #endif

//    if (edges.size() > 0) {
//        int height = img.getHeight();
//        int width = img.getWidth();

//        // Arithmetic mean
//        Point avg, stddev;
//        Vector2<accumulator_set<double, stats<tag::mean, tag::variance> > > acc;

//        BOOST_FOREACH(Point& p, edges) {
//            acc.x(p.x);
//            acc.y(p.y);
//        }

//        avg.x = std::mean(acc.x);
//        avg.y = std::mean(acc.y);
//        stddev.x = std::sqrt(variance(acc.x));
//        stddev.y = std::sqrt(variance(acc.y));

//        // Statistical throw-out
//        it = edges.begin();
//        while (it < edges.end()) {
//            if (std::abs(it->x - avg.x) > stddev.x || std::abs(it->y - avg.y) > stddev.y)
//                it = edges.erase(it);
//            else
//                it++;
//        }

//        // Geometric mean
//        Vector2<long double> pos(1.0, 1.0);

//        BOOST_FOREACH(Point& p, edges) {
//            pos.x *= std::pow(p.x, 1.0/edges.size());
//            pos.y *= std::pow(p.y, 1.0/edges.size());
//        }

//        pos.x = std::min(pos.x, width - 1.0L);
//        pos.y = std::min(pos.y, height - 1.0L);

//        // Find ball centre (not occluded)
//        int top = pos.y,
//            bottom = pos.y,
//            left = pos.x,
//            right = pos.x;
//        int not_orange_count = 0;

//        // FIND BALL CENTRE (single iteration approach; doesn't deal great with occlusion)
//        for(top = pos.y; top > 0 && not_orange_count <= VisionConstants::BALL_ORANGE_TOLERANCE; top--) {
//            if (getColourFromIndex( lut.classifyPixel( img((int)pos.x, top) ) ) != orange) {
//                not_orange_count++;
//            }
//            else {
//                not_orange_count = 0;
//            }
//        }
//        top += not_orange_count;

//        not_orange_count = 0;
//        for(bottom = pos.y; bottom < height && not_orange_count <= VisionConstants::BALL_ORANGE_TOLERANCE; bottom++) {
//            if (getColourFromIndex( lut.classifyPixel( img((int)pos.x, bottom) ) ) != orange)
//                not_orange_count++;
//            else
//                not_orange_count = 0;
//            bottom ++;
//        }
//        bottom -= not_orange_count;
//        not_orange_count = 0;

//        while (left > 0 && not_orange_count <= VisionConstants::BALL_ORANGE_TOLERANCE) {
//            if (getColourFromIndex( lut.classifyPixel( img(left, (int)pos.y) ) ) != orange) {
//                not_orange_count++;
//            }
//            else {
//                not_orange_count = 0;
//            }
//            left --;
//        }
//        left += not_orange_count;
//        not_orange_count = 0;

//        while (right < width && not_orange_count <= VisionConstants::BALL_ORANGE_TOLERANCE) {
//            if (getColourFromIndex( lut.classifyPixel( img(right, (int)pos.y) ) ) != orange)
//                not_orange_count++;
//            else
//                not_orange_count = 0;
//            right ++;
//        }
//        right -= not_orange_count;

//        // CHECK IF POINT IS ON EDGE OF BALL (OR OCCLUDED)
//        // OCCLUSION CHECK / COMPENSATION
//        bool top_edge = false,
//             bottom_edge = false,
//             left_edge = false,
//             right_edge = false;

//        for (int i = left; i > left - VisionConstants::BALL_EDGE_THRESHOLD && i >= 0; i--) {
//            if (getColourFromIndex(lut.classifyPixel(img(i, (int)pos.y))) == green) {
//                left_edge = true;
//                break;
//            }
//        }
//        for (int i = right; i < right + VisionConstants::BALL_EDGE_THRESHOLD && i < width; i++) {
//            if (getColourFromIndex(lut.classifyPixel(img(i, (int)pos.y))) == green) {
//                right_edge = true;
//                break;
//            }
//        }
//        for (int i = bottom; i < bottom + VisionConstants::BALL_EDGE_THRESHOLD && i < height; i++) {
//            if (getColourFromIndex(lut.classifyPixel(img((int)pos.x, i))) == green) {
//                bottom_edge = true;
//                break;
//            }
//        }
//        for (int i = top; i > top - VisionConstants::BALL_EDGE_THRESHOLD && i >= 0; i--) {
//            if (getColourFromIndex(lut.classifyPixel(img((int)pos.x, i))) == green) {
//                top_edge = true;
//                break;
//            }
//        }
//        top_edge = true;

//        // DETERMINE CENTRE
//        Point center;

//        if (left_edge && right_edge && top_edge && !bottom_edge)        // only bottom occluded
//            center = Point((right+left)/2, min((top+(top+right-left))/2, height-1));
//        else if (left_edge && right_edge && !top_edge && bottom_edge)   // only top occluded
//            center = Point((right+left)/2, max((bottom+(bottom-right+left))/2, 0));
//        else if (left_edge && !right_edge && top_edge && bottom_edge)   // only right occluded
//            center = Point(min((left+(left+bottom-top))/2, width-1),(top+bottom)/2);
//        else if (!left_edge && right_edge && top_edge && bottom_edge)   // only left occluded
//            center = Point(max((right+(right-bottom+top))/2, 0),(top+bottom)/2);
//        else
//            center = Point((right+left)/2,(top+bottom)/2);

//        // CHECK FOR SUCCESS
//        if (center != Point(1, 1) && bottom > top && right > left) {
//            // expensive check - only do if we want to
//            if(VisionConstants::BALL_MIN_PERCENT_ORANGE > 0) {
//                // CHECK FOR PIXEL DENSITY
//                int count = 0;

//                double min_dimension = std::min(right-left, bottom-top);

//                int box_left = std::max(center.x - min_dimension/2, 0.0);
//                int box_right = std::min(center.x + min_dimension/2, width-1.0);
//                int box_top = std::max(center.y - min_dimension/2, 0.0);
//                int box_bottom = std::min(center.y + min_dimension/2, height-1.0);

//                //std::cout << box_left << ", " << box_right << ", " << box_top << ", " << box_bottom << std::endl;

//                for (int i = box_left; i < box_right; i++) {
//                    for (int j = box_top; j < box_bottom; j++) {
//                        if (getColourFromIndex(lut.classifyPixel(img(i, j))) == orange)
//                            count++;
//                    }
//                }
//                //std::cout << "PERCENT ORANGE: " << float(count)/((min*2)*(min*2)) << std::endl;

//                if (count/(min_dimension*min_dimension) >= VisionConstants::BALL_MIN_PERCENT_ORANGE) {
//                    balls.push_back(Ball(center, max((right-left), (bottom-top))));
//                }
//                else {
//                    //std::cout << "BALL THROWN OUT ON RATIO" << std::endl;
//                    #if VISION_BALL_VERBOSITY > 1
//                        debug << "BallDetector::detectBall - ball thrown out on percentage contained orange" << std::endl;
//                    #endif
//                }
//            }
//            else {
//                balls.push_back(Ball(center, max((right-left), (bottom-top))));
//            }
//        }
//        else {
//            #if VISION_BALL_VERBOSITY > 1
//                debug << "BallDetector::detectBall - (1,1) ball thrown out" << std::endl;
//            #endif
//        }
//    }

//    return balls;
//}


vector<Ball> BallDetector::run()
{
    VisionBlackboard* vbb = VisionBlackboard::getInstance();
    NUImage img = vbb->getOriginalImage();
    const LookUpTable& lut = vbb->getLUT();
    // BEGIN BALL DETECTION -----------------------------------------------------------------

    std::vector<ColourSegment> v_segments = vbb->getVerticalTransitions(BALL_COLOUR);
    std::vector<ColourSegment> h_segments = vbb->getHorizontalTransitions(BALL_COLOUR);
    std::vector<Point> edges;
    std::vector<Ball> balls; //will only ever hold one

    appendEdgesFromSegments(h_segments, edges);
    appendEdgesFromSegments(v_segments, edges);

    #if VISION_BALL_VERBOSITY > 1
        debug << "BallDetector::detectBall() - number of vertical ball segments: " << v_segments.size() << std::endl;
        debug << "BallDetector::detectBall() - number of horizontal ball segments: " << h_segments.size() << std::endl;
    #endif

    const GreenHorizon& green_horizon = vbb->getGreenHorizon();

    // Throw out points above the horizon
    std::vector<Point>::iterator it;
    it = edges.begin();
    while (it < edges.end()) {
        if(green_horizon.isBelowHorizon(*it)) {
            it++;   //move to next edge
        }
        else {
            it = edges.erase(it);
        }
    }

    #if VISION_BALL_VERBOSITY > 1
    debug << "BallDetector::detectBall() - " << edges.size() << " ball segments above green horizon." << std::endl;
    #endif

    if (edges.size() > 0) {
        // Arithmetic mean
        int x_mean = 0,
            y_mean = 0;
        for (unsigned int i = 0; i < edges.size(); i++) {
            x_mean += edges.at(i).x;
            y_mean += edges.at(i).y;
        }
        x_mean /= edges.size();
        y_mean /= edges.size();

        // Standard deviation
        int x_dev = 0,
            y_dev = 0;
        for (unsigned int i = 0; i < edges.size(); i++) {
            x_dev += abs(edges.at(i).x - x_mean);
            y_dev += abs(edges.at(i).y - y_mean);
        }
        x_dev /= edges.size();
        y_dev /= edges.size();

        //std::cout << edges.size() << std::endl;


        // Statistical throw-out
        it = edges.begin();
        while (it < edges.end()) {
            if (abs(it->x - x_mean) > x_dev || abs(it->y - y_mean) > y_dev)
                it = edges.erase(it);
            else
                it++;
        }

        // Geometric mean
        long double x_pos = 1,
               y_pos = 1;
        for (unsigned int i = 0; i < edges.size(); i++) {
            int x = edges.at(i).x,
                y = edges.at(i).y;

            x_pos *= pow(x, 1.0/edges.size());
            y_pos *= pow(y, 1.0/edges.size());
        }
        //x_pos = pow(x_pos, 1.0/edges.size());
        //y_pos = pow(y_pos, 1.0/edges.size());

        if (y_pos >= img.getHeight())
            y_pos = img.getHeight()-1;
        if (x_pos >= img.getWidth())
            x_pos = img.getWidth()-1;

        // Find ball centre (not occluded)        
        int top = y_pos,
            bottom = y_pos,
            left = x_pos,
            right = x_pos;
        int not_orange_count = 0;

        bool top_edge = false,
             bottom_edge = false,
             left_edge = false,
             right_edge = false;        

        // FIND BALL CENTRE (single iteration approach; doesn't deal great with occlusion)

        while (top > 0 && not_orange_count <= VisionConstants::BALL_ORANGE_TOLERANCE) {
            if (getColourFromIndex(lut.classifyPixel(img((int)x_pos, top))) != orange) {
                not_orange_count++;
            }
            else {
                not_orange_count = 0;
            }
            top --;
        }
        top += not_orange_count;
        not_orange_count = 0;

        while (bottom < img.getHeight() && not_orange_count <= VisionConstants::BALL_ORANGE_TOLERANCE) {
            if (getColourFromIndex(lut.classifyPixel(img((int)x_pos, bottom))) != orange) {
                not_orange_count++;
            }
            else {
                not_orange_count = 0;
            }
            bottom ++;
        }
        bottom -= not_orange_count;
        not_orange_count = 0;

        while (left > 0 && not_orange_count <= VisionConstants::BALL_ORANGE_TOLERANCE) {
            if (getColourFromIndex(lut.classifyPixel(img(left, (int)y_pos))) != orange) {
                not_orange_count++;
            }
            else {
                not_orange_count = 0;
            }
            left --;
        }
        left += not_orange_count;
        not_orange_count = 0;

        while (right < img.getWidth() && not_orange_count <= VisionConstants::BALL_ORANGE_TOLERANCE) {
            if (getColourFromIndex(lut.classifyPixel(img(right, (int)y_pos))) != orange) {
                not_orange_count++;
            }
            else {
                not_orange_count = 0;
            }
            right ++;
        }
        right -= not_orange_count;
        
        //RUNNING POINT FOR OLD BALL FITTING CODE
//        if (false) {
//            CircleFitting fitter = CircleFitting();
            
//            //make a std::vector of edge points
//            std::vector< Vector2<int> > edge_points;
//            Vector2<int> pt_top,pt_bottom,pt_left,pt_right;
//            pt_left.y = pt_right.y = y_pos;
//            pt_left.x = left;
//            pt_right.x = right;
//            pt_top.x = pt_bottom.x = x_pos;
//            pt_top.y = top;
//            pt_bottom.y = bottom;
//            edge_points.push_back(pt_top);
//            edge_points.push_back(pt_bottom);
//            edge_points.push_back(pt_left);
//            edge_points.push_back(pt_right);
            
//            //fit circle and recover data
//            Circle fitted_ball = fitter.FitCircleLMF(edge_points);
//            if (fitted_ball.isDefined) {
//                Point centre = Point((int)fitted_ball.centreX,(int)fitted_ball.centreY);
//                Ball newball(centre, (float)fitted_ball.radius);
//                vbb->addBall(newball);
//            }
//        }

        // CHECK IF POINT IS ON EDGE OF BALL (OR OCCLUDED)
        // OCCLUSION CHECK / COMPENSATION

        for (int i = left; i > left - VisionConstants::BALL_EDGE_THRESHOLD; i--) {
            if (i <= 0)
                break;
            else if (getColourFromIndex(lut.classifyPixel(img(i, (int)y_pos))) == green) {
                left_edge = true;
                break;
            }
        }
        for (int i = right; i < right + VisionConstants::BALL_EDGE_THRESHOLD; i++) {
            if (i >= img.getWidth()-1)
                break;
            else if (getColourFromIndex(lut.classifyPixel(img(i, (int)y_pos))) == green) {
                right_edge = true;
                break;
            }
        }
        for (int i = bottom; i < bottom + VisionConstants::BALL_EDGE_THRESHOLD; i++) {
            if (i >= img.getHeight()-1)
                break;
            else if (getColourFromIndex(lut.classifyPixel(img((int)x_pos, i))) == green) {
                bottom_edge = true;
                break;
            }
        }
        for (int i = top; i > top - VisionConstants::BALL_EDGE_THRESHOLD; i--) {
            if (i <= 0)
                break;
            else if (getColourFromIndex(lut.classifyPixel(img((int)x_pos, i))) == green) {
                top_edge = true;
                break;
            }
        }
        top_edge = true;

        Point center;
        if (left_edge == true && right_edge == true && top_edge == true && bottom_edge == true) {
            center = Point((right+left)/2,(top+bottom)/2);
        }
        else if (left_edge == true && right_edge == true) {
            if (top_edge == true && bottom_edge == false) {
                center = Point((right+left)/2, std::min((top+(top+right-left))/2, img.getHeight()-1));
            }
            else if (top_edge == false && bottom_edge == true) {
                center = Point((right+left)/2, std::max((bottom+(bottom-right+left))/2, 0));
            }
            else {
                center = Point((right+left)/2,(top+bottom)/2);
            }
        }
        else if (top_edge == true && bottom_edge == true) {
            if (left_edge == true && right_edge == false) {
                center = Point(std::min((left+(left+bottom-top))/2, img.getWidth()-1),(top+bottom)/2);
            }
            else if (left_edge == false && right_edge == true) {
                center = Point(std::max((right+(right-bottom+top))/2, 0),(top+bottom)/2);
            }
            else {
                center = Point((right+left)/2,(top+bottom)/2);
            }
        }
        else {
            center = Point((right+left)/2,(top+bottom)/2);
        }

        if (!(center.x ==1 and center.y==1) && bottom-top > 0 && right-left > 0) {

            // CHECK FOR PIXEL DENSITY
            int count = 0;

            int min = std::min(right-left, bottom-top);
            min /= 2;

            int box_left = std::max(center.x - min, 0.0);
            int box_right = std::min(center.x + min, img.getWidth()-1.0);
            int box_top = std::max(center.y - min, 0.0);
            int box_bottom = std::min(center.y + min, img.getHeight()-1.0);

            //std::cout << box_left << ", " << box_right << ", " << box_top << ", " << box_bottom << std::endl;

            for (int i = box_left; i < box_right; i++) {
                for (int j = box_top; j < box_bottom; j++) {
                    if (getColourFromIndex(lut.classifyPixel(img(i, j))) == orange)
                        count++;
                }
            }
            //std::cout << "PERCENT ORANGE: " << float(count)/((min*2)*(min*2)) << std::endl;

            if (count/((min*2.0)*(min*2.0)) >= VisionConstants::BALL_MIN_PERCENT_ORANGE) {
                balls.push_back(Ball(center, std::max((right-left), (bottom-top))));
            }
            else {
                //std::cout << "BALL THROWN OUT ON RATIO" << std::endl;
                #if VISION_BALL_VERBOSITY > 1
                    debug << "BallDetector::detectBall - ball thrown out on percentage contained orange" << std::endl;
                #endif
            }
        }
        else {
            #if VISION_BALL_VERBOSITY > 1
                debug << "BallDetector::detectBall - (1,1) ball thrown out" << std::endl;
            #endif
        }
    }

    return balls;
}

void BallDetector::appendEdgesFromSegments(const std::vector<ColourSegment> &segments, std::vector< Point > &pointList)
{
    std::vector<ColourSegment>::const_iterator it;
    for(it = segments.begin(); it < segments.end(); it++) {
        pointList.push_back(it->getStart());
        pointList.push_back(it->getEnd());
    }
}
