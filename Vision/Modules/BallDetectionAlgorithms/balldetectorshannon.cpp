#include "balldetectorshannon.h"
#include "Vision/visionconstants.h"
#include "Vision/visionblackboard.h"
#include "debug.h"
#include "debugverbosityvision.h"

////for stat
//#include <boost/foreach.hpp>
//#include <boost/accumulators/accumulators.hpp>
//#include <boost/accumulators/statistics/stats.hpp>
//#include <boost/accumulators/statistics/mean.hpp>
//#include <boost/accumulators/statistics/variance.hpp>

//using namespace boost::accumulators;

BallDetectorShannon::BallDetectorShannon() {}

BallDetectorShannon::~BallDetectorShannon() {}

// BROKEN
std::vector<Ball> BallDetectorShannon::run()
{
    VisionBlackboard* vbb = VisionBlackboard::getInstance();
    NUImage img = vbb->getOriginalImage();
    const LookUpTable& lut = vbb->getLUT();
    const GreenHorizon& green_horizon = vbb->getGreenHorizon();
    // BEGIN BALL DETECTION -----------------------------------------------------------------

    std::vector<ColourSegment> v_segments = vbb->getVerticalTransitions(BALL_COLOUR);
    std::vector<ColourSegment> h_segments = vbb->getHorizontalTransitions(BALL_COLOUR);
    std::list<Point> edges;
    std::vector<Ball> balls; //will only ever hold one

    appendEdgesFromSegments(h_segments, edges, green_horizon);
    appendEdgesFromSegments(v_segments, edges, green_horizon);

    #if VISION_BALL_VERBOSITY > 1
    debug << "BallDetectorShannon::detectBall() - number of vertical ball segments: " << v_segments.size() << std::endl;
    debug << "BallDetectorShannon::detectBall() - number of horizontal ball segments: " << h_segments.size() << std::endl;
    debug << "BallDetectorShannon::detectBall() - ball segments above green horizon: " << edges.size() << std::endl;
    #endif

    int height = img.getHeight();
    int width = img.getWidth();

    // Arithmetic mean
    Point avg, stddev;
    Vector2<accumulator_set<double, stats<tag::mean, tag::variance> > > acc;

    for(const Point& p : edges) {
        acc.x(p.x);
        acc.y(p.y);
    }

    avg.x = mean(acc.x);
    avg.y = mean(acc.y);
    stddev.x = std::sqrt(variance(acc.x));
    stddev.y = std::sqrt(variance(acc.y));

    #if VISION_BALL_VERBOSITY > 1
    debug << "BallDetectorShannon::detectBall() - arithmetic mean: " << avg << " stddev: " << stddev << std::endl;
    #endif

    std::list<Point>::iterator it;
    // Statistical throw-out
    it = edges.begin();
    while (it != edges.end()) {
        if (std::abs(it->x - avg.x) > stddev.x || std::abs(it->y - avg.y) > stddev.y)
            it = edges.erase(it);
        else
            it++;
    }

    #if VISION_BALL_VERBOSITY > 1
    debug << "BallDetectorShannon::detectBall() - segments after throwout: " << edges.size() << std::endl;
    #endif

    if(!edges.empty())
    {
        // Geometric mean
        Vector2<long double> pos(1.0, 1.0);
        long double root_order = 1.0 / edges.size();

        for(const Point& p : edges) {
            pos.x *= std::pow(p.x, root_order);
            pos.y *= std::pow(p.y, root_order);
        }

        pos.x = std::min(pos.x, width - 1.0L);
        pos.y = std::min(pos.y, height - 1.0L);

        #if VISION_BALL_VERBOSITY > 1
        debug << "BallDetectorShannon::detectBall() - geometric mean: " << pos << std::endl;
        #endif

        // Find ball centre (not occluded)
        int top = pos.y,
            bottom = pos.y,
            left = pos.x,
            right = pos.x;
        int not_orange_count = 0;

        // FIND BALL CENTRE (single iteration approach; doesn't deal great with occlusion)
        for(top = pos.y; top > 0 && not_orange_count <= VisionConstants::BALL_ORANGE_TOLERANCE; top--) {
            if (getColourFromIndex( lut.classifyPixel( img((int)pos.x, top) ) ) != orange) {
                not_orange_count++;
            }
            else {
                not_orange_count = 0;
            }
        }
        top += not_orange_count;

        not_orange_count = 0;
        for(bottom = pos.y; bottom < height && not_orange_count <= VisionConstants::BALL_ORANGE_TOLERANCE; bottom++) {
            if (getColourFromIndex( lut.classifyPixel( img((int)pos.x, bottom) ) ) != orange)
                not_orange_count++;
            else
                not_orange_count = 0;
        }
        bottom -= not_orange_count;

        not_orange_count = 0;
        for(left = pos.x; left > 0 && not_orange_count <= VisionConstants::BALL_ORANGE_TOLERANCE; left--) {
            if (getColourFromIndex( lut.classifyPixel( img(left, (int)pos.y) ) ) != orange) {
                not_orange_count++;
            }
            else {
                not_orange_count = 0;
            }
        }
        left += not_orange_count;

        not_orange_count = 0;
        for(right = pos.x; right < width && not_orange_count <= VisionConstants::BALL_ORANGE_TOLERANCE; right++) {
            if (getColourFromIndex( lut.classifyPixel( img(right, (int)pos.y) ) ) != orange)
                not_orange_count++;
            else
                not_orange_count = 0;
        }
        right -= not_orange_count;

        #if VISION_BALL_VERBOSITY > 1
        debug << "BallDetectorShannon::detectBall() - \n\ttop: " << top << " bottom: " << bottom << " left: " << left << " right: " << right << std::endl;
        #endif

        // CHECK IF POINT IS ON EDGE OF BALL (OR OCCLUDED)
        // OCCLUSION CHECK / COMPENSATION
        bool top_edge = false,
             bottom_edge = false,
             left_edge = false,
             right_edge = false;

        for (int i = left; i > left - VisionConstants::BALL_EDGE_THRESHOLD && i >= 0; i--) {
            if (getColourFromIndex(lut.classifyPixel(img(i, (int)pos.y))) == green) {
                left_edge = true;
                break;
            }
        }
        for (int i = right; i < right + VisionConstants::BALL_EDGE_THRESHOLD && i < width; i++) {
            if (getColourFromIndex(lut.classifyPixel(img(i, (int)pos.y))) == green) {
                right_edge = true;
                break;
            }
        }
        for (int i = bottom; i < bottom + VisionConstants::BALL_EDGE_THRESHOLD && i < height; i++) {
            if (getColourFromIndex(lut.classifyPixel(img((int)pos.x, i))) == green) {
                bottom_edge = true;
                break;
            }
        }
        for (int i = top; i > top - VisionConstants::BALL_EDGE_THRESHOLD && i >= 0; i--) {
            if (getColourFromIndex(lut.classifyPixel(img((int)pos.x, i))) == green) {
                top_edge = true;
                break;
            }
        }
        top_edge = true;

        // DETERMINE CENTRE
        Point center;

        if (left_edge && right_edge && top_edge && !bottom_edge)        // only bottom occluded
            center = Point((right+left)/2, std::min((top+(top+right-left))/2, height-1));
        else if (left_edge && right_edge && !top_edge && bottom_edge)   // only top occluded
            center = Point((right+left)/2, std::max((bottom+(bottom-right+left))/2, 0));
        else if (left_edge && !right_edge && top_edge && bottom_edge)   // only right occluded
            center = Point(std::min((left+(left+bottom-top))/2, width-1),(top+bottom)/2);
        else if (!left_edge && right_edge && top_edge && bottom_edge)   // only left occluded
            center = Point(std::max((right+(right-bottom+top))/2, 0),(top+bottom)/2);
        else
            center = Point((right+left)/2,(top+bottom)/2);

        // CHECK FOR SUCCESS
        if (center != Point(1, 1) && bottom > top && right > left) {
            // expensive check - only do if we want to
            if(VisionConstants::BALL_MIN_PERCENT_ORANGE > 0) {
                // CHECK FOR PIXEL DENSITY
                int count = 0;

                double min_dimension = std::min(right-left, bottom-top);

                int box_left = std::max(center.x - min_dimension/2, 0.0);
                int box_right = std::min(center.x + min_dimension/2, width-1.0);
                int box_top = std::max(center.y - min_dimension/2, 0.0);
                int box_bottom = std::min(center.y + min_dimension/2, height-1.0);

                //std::cout << box_left << ", " << box_right << ", " << box_top << ", " << box_bottom << std::endl;

                for (int i = box_left; i < box_right; i++) {
                    for (int j = box_top; j < box_bottom; j++) {
                        if (getColourFromIndex(lut.classifyPixel(img(i, j))) == orange)
                            count++;
                    }
                }
                //std::cout << "PERCENT ORANGE: " << float(count)/((min*2)*(min*2)) << std::endl;

                if (count/(min_dimension*min_dimension) >= VisionConstants::BALL_MIN_PERCENT_ORANGE) {
                    balls.push_back(Ball(center, std::max((right-left), (bottom-top))));
                }
                else {
                    //std::cout << "BALL THROWN OUT ON RATIO" << std::endl;
                    #if VISION_BALL_VERBOSITY > 1
                        debug << "BallDetectorShannon::detectBall - ball thrown out on percentage contained orange" << std::endl;
                    #endif
                }
            }
            else {
                balls.push_back(Ball(center, std::max((right-left), (bottom-top))));
            }
        }
        else {
            #if VISION_BALL_VERBOSITY > 1
                debug << "BallDetectorShannon::detectBall - (1,1) ball thrown out" << std::endl;
            #endif
        }
    }

    return balls;
}

void BallDetectorShannon::appendEdgesFromSegments(const std::vector<ColourSegment> &segments, std::list< Point > &pointList, const GreenHorizon& gh)
{
    std::vector<ColourSegment>::const_iterator it;
    for(it = segments.begin(); it < segments.end(); it++) {
        Point start = it->getStart();
        Point end = it->getEnd();
        if(gh.isBelowHorizon(start))
            pointList.push_back(start);
        if(gh.isBelowHorizon(end))
            pointList.push_back(end);
    }
}
