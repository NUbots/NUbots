#include "circledetector.h"
#include "Vision/visionblackboard.h"
#include "Vision/visionconstants.h"
#include "Vision/VisionTypes/coloursegment.h"
#include "Vision/VisionTypes/greenhorizon.h"
#include "Vision/VisionTypes/RANSACTypes/ransaccircle.h"
#include "Vision/GenericAlgorithms/ransac.h"
#include "debug.h"
#include "debugverbosityvision.h"

CircleDetector::CircleDetector(double tolerance, unsigned int n, unsigned int k, double e, unsigned int max_iterations)
{
    m_n = n;               //min pts to circle essentially
    m_k = k;               //number of iterations per fitting attempt
    m_e = e;              //consensus margin
    m_max_iterations = max_iterations;
    setTolerance(tolerance);
}

void CircleDetector::setTolerance(double tolerance)
{
#if VISION_FIELDPOINT_VERBOSITY > 0
    if(tolerance < 0 || tolerance > 1)
        debug << "CircleDetector::setTolerance - invalid tolerance: " << tolerance << " (must be in [0, 1]." << endl;
#endif

    m_tolerance = max(min(tolerance, 1.0), 0.0); //clamp
}

bool CircleDetector::run(vector<GroundPoint> &points, CentreCircle &result)
{
    RANSACCircle<GroundPoint> candidate;
    vector<GroundPoint> consensus, remainder;
    double variance;
    unsigned int i=0;
    bool modelfound = false;

    // attemp multiple RANSAC fits

    // run first iterations
    modelfound = RANSAC::findModel<RANSACCircle<GroundPoint>, GroundPoint>(points, candidate, consensus, remainder, variance, m_e, m_n, m_k, RANSAC::LargestConsensus);

    //continue while models are found but not a final version
    while(modelfound && i < m_max_iterations) {
        // check if the model is good enough
        if(variance <= m_tolerance*candidate.getRadius() &&
            candidate.getRadius() <= (1 + m_tolerance)*VisionConstants::CENTRE_CIRCLE_RADIUS &&
            candidate.getRadius() >= (1 - m_tolerance)*VisionConstants::CENTRE_CIRCLE_RADIUS)
        {
            // get outer points to determine screen radius
            double left = VisionBlackboard::getInstance()->getImageWidth() - 1,
                   right = 0,
                   top = VisionBlackboard::getInstance()->getImageHeight() - 1,
                   bottom = 0;
            BOOST_FOREACH(const GroundPoint& p, consensus) {
                left = min(left, p.screen.x);
                right = max(right, p.screen.x);
                top = min(top, p.screen.y);
                bottom = max(bottom, p.screen.y);
            }

            result = CentreCircle(candidate.getCentre(), candidate.getRadius(), Vector2<double>(right-left, bottom-top));
            points = remainder;
            return true;    // break out once decent model found
        }
        else {
            // model isn't good enough, reattempt with remainder
            modelfound = RANSAC::findModel<RANSACCircle<GroundPoint>, GroundPoint>(remainder, candidate, consensus, remainder, variance, m_e, m_n, m_k, RANSAC::LargestConsensus);
        }

        i++;
    }
    return false;
}

