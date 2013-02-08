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
    m_location_pixels.x = 0;
    m_location_pixels.y = 0;
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
    Vector2<double> top_pt = Vector2<double>((right-left)*0.5, top);
    Vector2<double> bottom_pt = Vector2<double>((right-left)*0.5, bottom);
    Vector2<double> right_pt = Vector2<double>(right, (bottom-top)*0.5);
    Vector2<double> left_pt = Vector2<double>(left, (bottom-top)*0.5);
    Vector2<double> centre_pt = Vector2<double>(centre.x, centre.y);

    //    if(VisionConstants::DO_RADIAL_CORRECTION) {
    //        VisionBlackboard* vbb = VisionBlackboard::getInstance();
    //        top_pt = vbb->correctDistortion(top_pt);
    //        bottom_pt = vbb->correctDistortion(bottom_pt);
    //        right_pt = vbb->correctDistortion(right_pt);
    //        left_pt = vbb->correctDistortion(left_pt);
    //        centre_pt = vbb->correctDistortion(centre_pt);
    //    }
        
    m_diameter = mathGeneral::roundNumberToInt(max(bottom_pt.y - top_pt.y, right_pt.x - left_pt.x));
    m_location_pixels.x = mathGeneral::roundNumberToInt(centre_pt.x);
    m_location_pixels.y = mathGeneral::roundNumberToInt(centre_pt.y);
    m_size_on_screen = Vector2<double>(m_diameter, m_diameter);
    valid = calculatePositions();
    //valid = valid && check();
    valid = check();
}

float Ball::getRadius() const
{
    return m_diameter*0.5;
}

bool Ball::addToExternalFieldObjects(FieldObjects *fieldobjects, float timestamp) const
{
    #if VISION_FIELDOBJECT_VERBOSITY > 1
        debug << "Ball::addToExternalFieldObjects:" << endl;
        debug << *this << endl;
    #endif
    if(valid) {
        //add ball to mobileFieldObjects
        //cout << m_transformed_spherical_pos.x << " " << m_transformed_spherical_pos.y << " " << m_transformed_spherical_pos.z << endl;
        fieldobjects->mobileFieldObjects[FieldObjects::FO_BALL].UpdateVisualObject(m_spherical_position,
                                                                                   m_spherical_error,
                                                                                   m_location_angular,
                                                                                   Vector2<int>(m_location_pixels.x,m_location_pixels.y),
                                                                                   Vector2<int>(m_size_on_screen.x,m_size_on_screen.y),
                                                                                   timestamp);
        #if VISION_FIELDOBJECT_VERBOSITY > 1
            debug << "Ball::addToExternalFieldObjects: valid" << endl;
        #endif
        return true;
    }
    else {
        #if VISION_FIELDOBJECT_VERBOSITY > 1
            debug << "Ball::addToExternalFieldObjects: invalid" << endl;
        #endif
        return false;
    }
}

bool Ball::check() const
{
    //various throwouts here

    if(!distance_valid) {
        #if VISION_FIELDOBJECT_VERBOSITY > 1
            debug << "Ball::check - Ball thrown out: distance invalid" << endl;
        #endif
        return false;
    }

    //throwout for below horizon
    if(VisionConstants::THROWOUT_ON_ABOVE_KIN_HOR_BALL and
       not VisionBlackboard::getInstance()->getKinematicsHorizon().IsBelowHorizon(m_location_pixels.x, m_location_pixels.y)) {
        errorlog << "Ball::check() - Ball above horizon: should not occur" << endl;
        #if VISION_FIELDOBJECT_VERBOSITY > 1
            debug << "Ball::check - Ball thrown out: above kinematics horizon" << endl;
        #endif
        return false;
    }
    
    //Distance discrepency throwout - if width method says ball is a lot closer than d2p (by specified value) then discard
    if(VisionConstants::THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BALL and
            abs(width_dist - d2p) > VisionConstants::MAX_DISTANCE_METHOD_DISCREPENCY_BALL) {
        #if VISION_FIELDOBJECT_VERBOSITY > 1
        debug << "Ball::check - Ball thrown out: width distance too much smaller than d2p" << endl;
            debug << "\td2p: " << d2p << " width_dist: " << width_dist << " MAX_DISTANCE_METHOD_DISCREPENCY_BALL: " << VisionConstants::MAX_DISTANCE_METHOD_DISCREPENCY_BALL << endl;
        #endif
        return false;
    }

    //throw out if ball is too small
    if(VisionConstants::THROWOUT_SMALL_BALLS and 
        m_diameter < VisionConstants::MIN_BALL_DIAMETER_PIXELS) {
        #if VISION_FIELDOBJECT_VERBOSITY > 1
            debug << "Ball::check - Ball thrown out: too small" << endl;
            debug << "\tdiameter: " << m_diameter << " MIN_BALL_DIAMETER_PIXELS: " << VisionConstants::MIN_BALL_DIAMETER_PIXELS << endl;
        #endif
        return false;
    }
    
    //throw out if ball is too far away
    if(VisionConstants::THROWOUT_DISTANT_BALLS and 
        m_spherical_position.x > VisionConstants::MAX_BALL_DISTANCE) {
        #if VISION_FIELDOBJECT_VERBOSITY > 1
            debug << "Ball::check - Ball thrown out: too far away" << endl;
            debug << "\td2p: " << m_spherical_position.x << " MAX_BALL_DISTANCE: " << VisionConstants::MAX_BALL_DISTANCE << endl;
        #endif
        return false;
    }
    
    //all checks passed, keep ball
    return true;
}

bool Ball::calculatePositions()
{
    const Transformer& tran = VisionBlackboard::getInstance()->getTransformer();
    //To the bottom of the Goal Post.
    Point radial = tran.screenToRadial2D(m_location_pixels);
    m_location_angular = Vector2<float>(radial.x, radial.y);
    double dist = distanceToBall(radial.x, radial.y);

    m_spherical_position.x = dist;
    m_spherical_position.y = radial.x;
    m_spherical_position.z = radial.y;
    //m_spherical_error - not calculated

    #if VISION_FIELDOBJECT_VERBOSITY > 2
        debug << "Goal::calculatePositions: ";
        debug << d2p << " " << width_dist << " " << m_spherical_position.x << endl;
    #endif

    return distance_valid && dist > 0;
}


/*!
*   @brief Calculates the distance using the set METHOD and the provided coordinate angles.
*   @param bearing The angle about the z axis.
*   @param elevation The angle about the y axis.
*/
double Ball::distanceToBall(double bearing, double elevation) {
    const Transformer& tran = VisionBlackboard::getInstance()->getTransformer();
    //reset distance values
    bool d2pvalid = false;
    d2p = 0;
    width_dist = 0;
    //get distance to point from base

    d2pvalid = tran.isDistanceToPointValid();
    if(d2pvalid)
        d2p = tran.distanceToPoint(bearing, elevation);

    #if VISION_FIELDOBJECT_VERBOSITY > 1
        if(!d2pvalid)
            debug << "Ball::distanceToGoal: d2p invalid - combination methods will only return width_dist" << endl;
    #endif
    //get distance from width
    width_dist = VisionConstants::BALL_WIDTH*tran.getCameraDistanceInPixels()/m_size_on_screen.x;

    #if VISION_FIELDOBJECT_VERBOSITY > 1
        debug << "Ball::distanceToGoal: bearing: " << bearing << " elevation: " << elevation << endl;
        debug << "Ball::distanceToGoal: d2p: " << d2p << endl;
        debug << "Ball::distanceToGoal: m_size_on_screen.x: " << m_size_on_screen.x << endl;
        debug << "Ball::distanceToGoal: width_dist: " << width_dist << endl;
        debug << "Ball::distanceToGoal: Method: " << VisionConstants::getDistanceMethodName(VisionConstants::BALL_DISTANCE_METHOD) << endl;
    #endif
    switch(VisionConstants::BALL_DISTANCE_METHOD) {
    case VisionConstants::D2P:
        distance_valid = d2pvalid && d2p > 0;
        return d2p;
    case VisionConstants::Width:
        distance_valid = true;
        return width_dist;
    case VisionConstants::Average:
        //average distances
        distance_valid = d2pvalid && d2p > 0;
        return (d2p + width_dist) * 0.5;
    case VisionConstants::Least:
        distance_valid = d2pvalid && d2p > 0;
        if(distance_valid)
            return min(d2p, width_dist);
        else
            return width_dist;
    }
}

//void Ball::render(cv::Mat &mat) const
//{
//    cv::circle(mat, cv::Point2i(m_location_pixels.x, m_location_pixels.y), m_diameter*0.5, cv::Scalar(0, 125, 255), 2);
//}

ostream& operator<< (ostream& output, const Ball& b)
{
    output << "Ball " << endl;
    output << "\tpixelloc: [" << b.m_location_pixels.x << ", " << b.m_location_pixels.y << "]" << endl;
    output << " angularloc: [" << b.m_location_angular.x << ", " << b.m_location_angular.y << "]" << endl;
    output << "\trelative field coords: [" << b.m_spherical_position.x << ", " << b.m_spherical_position.y << ", " << b.m_spherical_position.z << "]" << endl;
    output << "\tspherical error: [" << b.m_spherical_error.x << ", " << b.m_spherical_error.y << "]" << endl;
    output << "\tsize on screen: [" << b.m_size_on_screen.x << ", " << b.m_size_on_screen.y << "]";
    return output;
}

ostream& operator<< (ostream& output, const vector<Ball>& b)
{
    for (size_t i=0; i<b.size(); i++)
        output << b[i] << endl;
    return output;
}
