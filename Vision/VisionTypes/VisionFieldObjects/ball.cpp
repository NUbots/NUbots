#include "ball.h"
#include "Vision/visionblackboard.h"
#include "Vision/visionconstants.h"

#include "debug.h"
#include "debugverbosityvision.h"

#include "Kinematics/Kinematics.h"
#include "Tools/Math/Matrix.h"
#include "Tools/Math/General.h"

Ball::Ball()
{
    m_id = BALL;
    m_diameter = 0;
    valid = calculatePositions();
    valid = valid && check();
}

Ball::Ball(Point centre, double diameter)
{
    m_id = BALL;
    double top = centre.y - diameter*0.5,
           bottom = centre.y + diameter*0.5,
           left = centre.x - diameter*0.5,
           right = centre.x + diameter*0.5;
    Point top_pt = Point((right-left)*0.5, top);
    Point bottom_pt = Point((right-left)*0.5, bottom);
    Point right_pt = Point(right, (bottom-top)*0.5);
    Point left_pt = Point(left, (bottom-top)*0.5);

    //    if(VisionConstants::DO_RADIAL_CORRECTION) {
    //        VisionBlackboard* vbb = VisionBlackboard::getInstance();
    //        top_pt = vbb->correctDistortion(top_pt);
    //        bottom_pt = vbb->correctDistortion(bottom_pt);
    //        right_pt = vbb->correctDistortion(right_pt);
    //        left_pt = vbb->correctDistortion(left_pt);
    //        centre_pt = vbb->correctDistortion(centre_pt);
    //    }
        
    m_diameter = std::max(bottom_pt.y - top_pt.y, right_pt.x - left_pt.x);
    m_location.screenCartesian = centre;
    m_size_on_screen = Vector2<double>(m_diameter, m_diameter);
    valid = calculatePositions();
    valid = valid && check();
}

float Ball::getRadius() const
{
    return m_diameter*0.5;
}

bool Ball::addToExternalFieldObjects(FieldObjects *fieldobjects, float timestamp) const
{
    #if VISION_BALL_VERBOSITY > 1
        debug << "Ball::addToExternalFieldObjects:" << std::endl;
        debug << *this << std::endl;
    #endif
    if(valid) {
        //add ball to mobileFieldObjects
        //std::cout << m_transformed_spherical_pos.x << " " << m_transformed_spherical_pos.y << " " << m_transformed_spherical_pos.z << std::endl;
        fieldobjects->mobileFieldObjects[FieldObjects::FO_BALL].UpdateVisualObject(Vector3<float>(m_location.neckRelativeRadial.x, m_location.neckRelativeRadial.y, m_location.neckRelativeRadial.z),
                                                                                   Vector3<float>(m_spherical_error.x, m_spherical_error.y, m_spherical_error.z),
                                                                                   Vector2<float>(m_location.screenAngular.x, m_location.screenAngular.y),
                                                                                   Vector2<int>(m_location.screenCartesian.x,m_location.screenCartesian.y),
                                                                                   Vector2<int>(m_size_on_screen.x,m_size_on_screen.y),
                                                                                   timestamp);
        #if VISION_BALL_VERBOSITY > 1
            debug << "Ball::addToExternalFieldObjects: valid" << std::endl;
        #endif
        return true;
    }
    else {
        #if VISION_BALL_VERBOSITY > 1
            debug << "Ball::addToExternalFieldObjects: invalid" << std::endl;
        #endif
        return false;
    }
}

bool Ball::check() const
{
    //various throwouts here

    //throwout for below horizon
    if(VisionConstants::THROWOUT_ON_ABOVE_KIN_HOR_BALL and
       not VisionBlackboard::getInstance()->getKinematicsHorizon().IsBelowHorizon(m_location.screenCartesian.x, m_location.screenCartesian.y)) {
        errorlog << "Ball::check() - Ball above horizon: should not occur" << std::endl;
        #if VISION_BALL_VERBOSITY > 1
            debug << "Ball::check - Ball thrown out: above kinematics horizon" << std::endl;
        #endif
        return false;
    }
    
    //Distance discrepency throwout - if width method says ball is a lot closer than d2p (by specified value) then discard
//    if(VisionConstants::THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BALL and
//            std::abs(width_dist - d2p) > VisionConstants::MAX_DISTANCE_METHOD_DISCREPENCY_BALL) {
//        #if VISION_BALL_VERBOSITY > 1
//        debug << "Ball::check - Ball thrown out: width distance too much smaller than d2p" << std::endl;
//            debug << "\td2p: " << d2p << " width_dist: " << width_dist << " MAX_DISTANCE_METHOD_DISCREPENCY_BALL: " << VisionConstants::MAX_DISTANCE_METHOD_DISCREPENCY_BALL << std::endl;
//        #endif
//        return false;
//    }

    //throw out if ball is too small
    if(VisionConstants::THROWOUT_SMALL_BALLS and 
        m_diameter < VisionConstants::MIN_BALL_DIAMETER_PIXELS) {
        #if VISION_BALL_VERBOSITY > 1
            debug << "Ball::check - Ball thrown out: too small" << std::endl;
            debug << "\tdiameter: " << m_diameter << " MIN_BALL_DIAMETER_PIXELS: " << VisionConstants::MIN_BALL_DIAMETER_PIXELS << std::endl;
        #endif
        return false;
    }
    
    //throw out if ball is too far away
    if(VisionConstants::THROWOUT_DISTANT_BALLS and 
        m_location.neckRelativeRadial.x > VisionConstants::MAX_BALL_DISTANCE) {
        #if VISION_BALL_VERBOSITY > 1
            debug << "Ball::check - Ball thrown out: too far away" << std::endl;
            debug << "\td2p: " << m_location.relativeRadial.x << " MAX_BALL_DISTANCE: " << VisionConstants::MAX_BALL_DISTANCE << std::endl;
        #endif
        return false;
    }
    
    //all checks passed, keep ball
    return true;
}

double Ball::findScreenError(VisionFieldObject *other) const
{
    Ball* b = dynamic_cast<Ball*>(other);
    return ( m_location.screenCartesian - b->m_location.screenCartesian ).abs() + ( m_size_on_screen - b->m_size_on_screen ).abs();
}

double Ball::findGroundError(VisionFieldObject *other) const
{
    Ball* b = dynamic_cast<Ball*>(other);
    return ( m_location.groundCartesian - b->m_location.groundCartesian ).abs();
}

bool Ball::calculatePositions()
{
    const Transformer& tran = VisionBlackboard::getInstance()->getTransformer();
    //To the bottom of the Goal Post
    NUPoint d2p_loc, width_loc;
    d2p_loc.screenCartesian = m_location.screenCartesian;
    width_loc.screenCartesian = m_location.screenCartesian;

    double width_dist = VisionConstants::BALL_WIDTH*tran.getCameraDistanceInPixels()/m_size_on_screen.x;

    tran.calculateRepresentationsFromPixelLocation(d2p_loc);
    tran.calculateRepresentationsFromPixelLocation(width_loc, true, width_dist);

    switch(VisionConstants::BALL_DISTANCE_METHOD) {
        case D2P:
            m_location = d2p_loc;
            break;
        case Width:
            m_location = width_loc;
            break;
        case Average:
            //average distances
            m_location.screenCartesian = (d2p_loc.screenCartesian + width_loc.screenCartesian) * 0.5;
            m_location.neckRelativeRadial = (d2p_loc.neckRelativeRadial + width_loc.neckRelativeRadial) * 0.5;
            m_location.screenAngular = (d2p_loc.screenAngular + width_loc.screenAngular) * 0.5;
            m_location.groundCartesian = (d2p_loc.groundCartesian + width_loc.groundCartesian) * 0.5;
            break;
        case Least:
            m_location = (d2p_loc.neckRelativeRadial.x < width_loc.neckRelativeRadial.x ? d2p_loc : width_loc);
            break;
    }
    #if VISION_BALL_VERBOSITY > 2
        debug << "Ball::calculatePositions: " << m_location << std::endl;
    #endif

    return m_location.neckRelativeRadial.x > 0;
}


/*!
*   @brief Calculates the distance using the set METHOD and the provided coordinate angles.
*   @param bearing The angle about the z axis.
*   @param elevation The angle about the y axis.
*/
//double Ball::distanceToBall(double bearing, double elevation) {
//    const Transformer& tran = VisionBlackboard::getInstance()->getTransformer();
//    //reset distance values
//    bool d2pvalid = false;
//    d2p = 0;
//    width_dist = 0;
//    double result = 0;
//    //get distance to point from base

//    d2pvalid = tran.isDistanceToPointValid();
//    if(d2pvalid)
//        d2p = tran.distanceToPoint(bearing, elevation);

//    #if VISION_BALL_VERBOSITY > 1
//        if(!d2pvalid)
//            debug << "Ball::distanceToGoal: d2p invalid - combination methods will only return width_dist" << std::endl;
//    #endif
//    //get distance from width
//    width_dist = VisionConstants::BALL_WIDTH*tran.getCameraDistanceInPixels()/m_size_on_screen.x;

//    #if VISION_BALL_VERBOSITY > 1
//        debug << "Ball::distanceToGoal: bearing: " << bearing << " elevation: " << elevation << std::endl;
//        debug << "Ball::distanceToGoal: d2p: " << d2p << std::endl;
//        debug << "Ball::distanceToGoal: m_size_on_screen.x: " << m_size_on_screen.x << std::endl;
//        debug << "Ball::distanceToGoal: width_dist: " << width_dist << std::endl;
//        debug << "Ball::distanceToGoal: Method: " << getDistanceMethodName(VisionConstants::BALL_DISTANCE_METHOD) << std::endl;
//    #endif
//    switch(VisionConstants::BALL_DISTANCE_METHOD) {
//    case D2P:
//        distance_valid = d2pvalid && d2p > 0;
//        result = d2p;
//        break;
//    case Width:
//        distance_valid = true;
//        result = width_dist;
//        break;
//    case Average:
//        //average distances
//        distance_valid = d2pvalid && d2p > 0;
//        result = (d2p + width_dist) * 0.5;
//        break;
//    case Least:
//        distance_valid = d2pvalid && d2p > 0;
//        result = (distance_valid ? std::min(d2p, width_dist) : width_dist);
//        break;
//    }

//    return result;
//}

std::ostream& operator<< (std::ostream& output, const Ball& b)
{
    output << "Ball " << std::endl;
    output << "\tpixelloc: [" << b.m_location.screenCartesian.x << ", " << b.m_location.screenCartesian.y << "]" << std::endl;
    output << " angularloc: [" << b.m_location.screenAngular.x << ", " << b.m_location.screenAngular.y << "]" << std::endl;
    output << "\trelative field coords: [" << b.m_location.neckRelativeRadial.x << ", " << b.m_location.neckRelativeRadial.y << ", " << b.m_location.neckRelativeRadial.z << "]" << std::endl;
    output << "\tspherical error: [" << b.m_spherical_error.x << ", " << b.m_spherical_error.y << "]" << std::endl;
    output << "\tsize on screen: [" << b.m_size_on_screen.x << ", " << b.m_size_on_screen.y << "]";
    return output;
}

std::ostream& operator<< (std::ostream& output, const std::vector<Ball>& b)
{
    for (size_t i=0; i<b.size(); i++)
        output << b[i] << std::endl;
    return output;
}
