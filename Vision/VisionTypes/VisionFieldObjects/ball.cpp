#include "ball.h"
#include "Vision/visionblackboard.h"
#include "Vision/visionconstants.h"

#include "debug.h"
#include "debugverbosityvision.h"

#include "Kinematics/Kinematics.h"
#include "Tools/Math/Matrix.h"

Ball::Ball()
{
    m_radius = 0;
    m_location_pixels.x = 0;
    m_location_pixels.y = 0;
    valid = calculatePositions();
    valid = valid && check();
}

Ball::Ball(PointType centre, float radius)
{
    int top = centre.y - radius,
        bottom = centre.y + radius,
        left = centre.x - radius,
        right = centre.x + radius;
    Vector2<float> top_pt = Vector2<float>((right-left)*0.5, top);
    Vector2<float> bottom_pt = Vector2<float>((right-left)*0.5, bottom);
    Vector2<float> right_pt = Vector2<float>(right, (bottom-top)*0.5);
    Vector2<float> left_pt = Vector2<float>(left, (bottom-top)*0.5);
    Vector2<float> centre_pt = Vector2<float>(centre.x, centre.y);
//    if(VisionConstants::DO_RADIAL_CORRECTION) {
//        top_pt = correctDistortion(top_pt);
//        bottom_pt = correctDistortion(bottom_pt);
//        right_pt = correctDistortion(right_pt);
//        left_pt = correctDistortion(left_pt);
//        centre_pt = correctDistortion(centre_pt);
//    }
        
    m_radius = max(bottom_pt.y - top_pt.y, right_pt.x - left_pt.x)*0.5;
    m_location_pixels.x = mathGeneral::roundNumberToInt(centre_pt.x);
    m_location_pixels.y = mathGeneral::roundNumberToInt(centre_pt.y);
    m_size_on_screen = Vector2<int>(m_radius*2, m_radius*2);
    valid = calculatePositions();
    valid = valid && check();
}

float Ball::getRadius() const
{
    return m_radius;
}

Vector3<float> Ball::getRelativeFieldCoords() const
{
    return m_spherical_position;
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
        fieldobjects->mobileFieldObjects[FieldObjects::FO_BALL].UpdateVisualObject(m_transformed_spherical_pos,
                                                                        m_spherical_error,
                                                                        m_location_angular,
                                                                        m_location_pixels,
                                                                        m_size_on_screen,
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
            width_dist + VisionConstants::MAX_DISTANCE_METHOD_DISCREPENCY_BALL < d2p) {
        #if VISION_FIELDOBJECT_VERBOSITY > 1
        debug << "Ball::check - Ball thrown out: width distance too much smaller than d2p" << endl;
            debug << "\td2p: " << d2p << " width_dist: " << width_dist << " MAX_DISTANCE_METHOD_DISCREPENCY_BALL: " << VisionConstants::MAX_DISTANCE_METHOD_DISCREPENCY_BALL << endl;
        #endif
        return false;
    }

    //throw out if ball is too small
    if(VisionConstants::THROWOUT_SMALL_BALLS and 
        m_radius*2 < VisionConstants::MIN_BALL_DIAMETER_PIXELS) {
        #if VISION_FIELDOBJECT_VERBOSITY > 1
            debug << "Ball::check - Ball thrown out: too small" << endl;
            debug << "\tdiameter: " << m_radius*2 << " MIN_BALL_DIAMETER_PIXELS: " << VisionConstants::MIN_BALL_DIAMETER_PIXELS << endl;
        #endif
        return false;
    }
    
    //throw out if ball is too far away
    if(VisionConstants::THROWOUT_DISTANT_BALLS and 
        m_transformed_spherical_pos.x > VisionConstants::MAX_BALL_DISTANCE) {
        #if VISION_FIELDOBJECT_VERBOSITY > 1
            debug << "Ball::check - Ball thrown out: too far away" << endl;
            debug << "\td2p: " << m_transformed_spherical_pos.x << " MAX_BALL_DISTANCE: " << VisionConstants::MAX_BALL_DISTANCE << endl;
        #endif
        return false;
    }
    
    //all checks passed, keep ball
    return true;
}

bool Ball::calculatePositions()
{
    VisionBlackboard* vbb = VisionBlackboard::getInstance();
    //To the bottom of the Goal Post.
    bool transform_valid;
    float elevation;
    float bearing = (float)vbb->calculateBearing(m_location_pixels.x);
    if(VisionConstants::BALL_DISTANCE_POSITION_BOTTOM) {
        elevation = (float)vbb->calculateElevation(m_location_pixels.y + m_radius);
    }
    else {
        elevation = (float)vbb->calculateElevation(m_location_pixels.y);
    }
    
    float distance = distanceToBall(bearing, elevation);
    
    #if VISION_FIELDOBJECT_VERBOSITY > 1
        debug << "Ball::calculatePositions() distance: " << distance << endl;
    #endif
    //! @todo implement 
    m_spherical_position[0] = distance; //distance calculated in distanceToBall() by given METHOD
    m_spherical_position[1] = bearing; //bearing
    m_spherical_position[2] = elevation; //elevation
    
    m_location_angular = Vector2<float>(bearing, elevation);

    if(vbb->isCameraToGroundValid()) {
        transform_valid = true;
        Matrix cameraToGroundTransform = Matrix4x4fromVector(vbb->getCameraToGroundVector());
        m_transformed_spherical_pos = Kinematics::TransformPosition(cameraToGroundTransform,m_spherical_position);
    }
    else {
        transform_valid = false;
        m_transformed_spherical_pos = Vector3<float>(0,0,0);
        #if VISION_FIELDOBJECT_VERBOSITY > 1
            debug << "Ball::calculatePositions: Kinematics CTG transform invalid - will not push ball" << endl;
        #endif
    }
    #if VISION_FIELDOBJECT_VERBOSITY > 2
        debug << "Ball::calculatePositions: ";
        debug << d2p << " " << width_dist << " " << distance << " " << m_transformed_spherical_pos.x << endl;
    #endif

    return transform_valid;
}


/*!
*   @brief Calculates the distance using the set METHOD and the provided coordinate angles.
*   @param bearing The angle about the z axis.
*   @param elevation The angle about the y axis.
*/
float Ball::distanceToBall(float bearing, float elevation) {
    VisionBlackboard* vbb = VisionBlackboard::getInstance();
    //reset distance values
    bool d2pvalid = false;
    d2p = 0;
    width_dist = 0;
    //get distance to point from base
//    if(vbb->isCameraToGroundValid())
//    {
//        Matrix camera2groundTransform = Matrix4x4fromVector(vbb->getCameraToGroundVector());
//        Vector3<float> result = Kinematics::DistanceToPoint(camera2groundTransform, bearing, elevation);
//        d2p = result[0];
//    }
    d2pvalid = vbb->distanceToPoint(bearing, elevation, d2p);
    
    #if VISION_FIELDOBJECT_VERBOSITY > 1
        if(!d2pvalid)
            debug << "Ball::distanceToBall: d2p invalid - combination methods will only return width_dist" << endl;
    #endif
        
    //get distance from width
    width_dist = VisionConstants::BALL_WIDTH*vbb->getCameraDistanceInPixels()/m_size_on_screen.x;

    #if VISION_FIELDOBJECT_VERBOSITY > 1
        debug << "Ball::distanceToBall: bearing: " << bearing << " elevation: " << elevation << endl;
        debug << "Ball::distanceToBall: d2p: " << d2p << endl;
        debug << "Ball::distanceToBall: m_size_on_screen.x: " << m_size_on_screen.x << endl;
        debug << "Ball::distanceToBall: width_dist: " << width_dist << endl;
    #endif
    switch(VisionConstants::BALL_DISTANCE_METHOD) {
    case VisionConstants::D2P:
        #if VISION_FIELDOBJECT_VERBOSITY > 1
            debug << "Ball::distanceToBall: Method: Combo" << endl;
        #endif
        distance_valid = d2pvalid;
        return d2p;
    case VisionConstants::Width:
        #if VISION_FIELDOBJECT_VERBOSITY > 1
            debug << "Ball::distanceToBall: Method: Width" << endl;
        #endif
        distance_valid = true;
        return width_dist;
    case VisionConstants::Average:
        #if VISION_FIELDOBJECT_VERBOSITY > 1
            debug << "Ball::distanceToBall: Method: Average" << endl;
        #endif
        //average distances
        distance_valid = true;
        if(d2pvalid) {
            return (d2p + width_dist) * 0.5;
        }
        else {
            return width_dist;
        }
    case VisionConstants::Least:
        #if VISION_FIELDOBJECT_VERBOSITY > 1
            debug << "Ball::distanceToBall: Method: Least" << endl;
        #endif
        distance_valid = true;
        if(d2pvalid) {
            return min(d2p, width_dist);
        }
        else {
            return width_dist;
        }
    }
}

/*! @brief Stream insertion operator for a single ColourSegment.
 *      The segment is terminated by a newline.
 */
ostream& operator<< (ostream& output, const Ball& b)
{
    output << "Ball " << endl;
    output << "\tpixelloc: [" << b.m_location_pixels.x << ", " << b.m_location_pixels.y << "]" << endl;
    output << " angularloc: [" << b.m_location_angular.x << ", " << b.m_location_angular.y << "]" << endl;
    output << "\trelative field coords: [" << b.m_spherical_position.x << ", " << b.m_spherical_position.y << ", " << b.m_spherical_position.z << "]" << endl;
    output << "\ttransformed field coords: [" << b.m_transformed_spherical_pos.x << ", " << b.m_transformed_spherical_pos.y << ", " << b.m_transformed_spherical_pos.z << "]" << endl;
    output << "\tspherical error: [" << b.m_spherical_error.x << ", " << b.m_spherical_error.y << "]" << endl;
    output << "\tsize on screen: [" << b.m_size_on_screen.x << ", " << b.m_size_on_screen.y << "]";
    return output;
}

/*! @brief Stream insertion operator for a vector of ColourSegments.
 *      Each segment is terminated by a newline.
 *  @relates ColourSegment
 */
ostream& operator<< (ostream& output, const vector<Ball>& b)
{
    for (size_t i=0; i<b.size(); i++)
        output << b[i] << endl;
    return output;
}
