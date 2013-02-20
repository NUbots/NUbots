#include "circledetector.h"
#include "Vision/visionblackboard.h"
#include "Vision/VisionTypes/coloursegment.h"
#include "Vision/VisionTypes/greenhorizon.h"
#include "Vision/VisionTypes/RANSACTypes/ransaccircle.h"
#include "Vision/GenericAlgorithms/ransac.h"

CircleDetector::CircleDetector(double tolerance, unsigned int n, unsigned int k, double e)
{
    m_n = n;               //min pts to circle essentially
    m_k = k;               //number of iterations per fitting attempt
    m_e = e;              //consensus margin
    setTolerance(tolerance);
}

void CircleDetector::setTolerance(double tolerance)
{
#if VISION_FIELDOBJECT_VERBOSITY > 1
    if(tolerance < 0 || tolerance > 1)
        debug << "CircleDetector::setTolerance - invalid tolerance: " << tolerance << " (must be in [0, 1]." << endl;
#endif

    m_tolerance = max(min(tolerance, 1.0), 0.0); //clamp
}

bool CircleDetector::run(vector<Point> &points, CentreCircle &result)
{
    RANSACCircle candidate;
    vector<Point> consensus, remainder;
    double variance;
    //attemp single RANSAC fit
    if(RANSAC::findModel<RANSACCircle, Point>(points, candidate, consensus, remainder, variance, m_e, m_n, m_k, RANSAC::LargestConsensus)) {
        //now check if the model is good enough
        if(variance <= m_tolerance*candidate.getRadius()) {

            //get outer points to determine screen radius
            double left = VisionBlackboard::getInstance()->getImageWidth() - 1,
                   right = 0,
                   top = VisionBlackboard::getInstance()->getImageHeight() - 1,
                   bottom = 0;
            BOOST_FOREACH(Point& p, consensus) {
                left = min(left, p.screen.x);
                right = max(right, p.screen.x);
                top = min(top, p.screen.y);
                bottom = max(bottom, p.screen.y);
            }

            result = CentreCircle(candidate.getCentre(), candidate.getRadius(), Vector2<double>(right-left, bottom-top));
            points = remainder;
            return true;
        }
    }
    return false;
}

