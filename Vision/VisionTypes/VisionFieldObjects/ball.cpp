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
    m_radius = 0;
    m_location_pixels.x = 0;
    m_location_pixels.y = 0;
    calculatePositions();
    check();
}

Ball::Ball(const PointType& centre, float radius)
{
    m_radius = radius;
    m_location_pixels.x = centre.x;
    m_location_pixels.y = centre.y;
    m_size_on_screen = Vector2<int>(radius*2, radius*2);
    calculatePositions();
    check();
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

void Ball::check()
{
    //various throwouts here
    //throwout for below horizon
    if(VisionBlackboard::getInstance()->getKinematicsHorizon().IsBelowHorizon(m_location_pixels.x, m_location_pixels.y)) {
        //throwout for distances not agreeing
        if(abs(d2p-width_dist) <= VisionConstants::MAX_DISTANCE_DISCREPENCY_BALL) {
            valid = true;
        }
        else {
            valid = false;
            #if VISION_FIELDOBJECT_VERBOSITY > 1
                debug << "Ball::check - Ball thrown out: distances don't agree" << endl;
                debug << "\td2p: " << d2p << " width_dist: " << width_dist << " MAX_DISTANCE_DISCREPENCY_BALL: " << VisionConstants::MAX_DISTANCE_DISCREPENCY_BALL << endl;
            #endif
        }
    }
    else {
        valid = false;
        errorlog << "Ball::check() - Ball below horizon: should not occur" << endl;
        #if VISION_FIELDOBJECT_VERBOSITY > 1
            debug << "Ball::check - Ball thrown out: above kinematics horizon" << endl;
        #endif
    }
}

void Ball::calculatePositions()
{
    VisionBlackboard* vbb = VisionBlackboard::getInstance();
    //To the bottom of the Goal Post.
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
    //m_spherical_error - not calculated
    
//    if(vbb->isCameraTransformValid()) {        
//        Matrix cameraTransform = Matrix4x4fromVector(vbb->getCameraTransformVector());
//        m_transformed_spherical_pos = Kinematics::TransformPosition(cameraTransform,m_spherical_position);
//    }
    if(vbb->isCameraToGroundValid()) {        
        Matrix cameraToGroundTransform = Matrix4x4fromVector(vbb->getCameraToGroundVector());
        
        //debugging
        m_transformed_spherical_pos = Kinematics::TransformPosition(cameraToGroundTransform,Vector3<float>(d2p, bearing, elevation));
        cout << " " << m_transformed_spherical_pos.x;
        m_transformed_spherical_pos = Kinematics::TransformPosition(cameraToGroundTransform,Vector3<float>(width_dist, bearing, elevation));
        cout << " " << m_transformed_spherical_pos.x << endl;
        
        //real
        m_transformed_spherical_pos = Kinematics::TransformPosition(cameraToGroundTransform,m_spherical_position);
        
    }
    else {
        m_transformed_spherical_pos = Vector3<float>(0,0,0);
    }
    #if VISION_FIELDOBJECT_VERBOSITY > 2
        debug << "Ball::calculatePositions: ";
        debug << d2p << " " << width_dist << " " << distance << " " << m_transformed_spherical_pos.x << endl;
    #endif
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
    if(vbb->isCameraHeightValid() && vbb->isCameraPitchValid()) {
        float cam_height = vbb->getCameraHeight(),
              cam_pitch = vbb->getCameraPitch(),
              theta = 0;    //resultant angle inclusive of body pitch, camera pitch and pixel elevation
        if(VisionConstants::D2P_INCLUDE_BODY_PITCH) {
            if(vbb->isBodyPitchValid()) {
                d2pvalid = true;
                float body_pitch = vbb->getBodyPitch();
                theta = mathGeneral::PI*0.5 - cam_pitch + elevation - body_pitch;
                d2p = cam_height / cos(theta);
            }
        }
        else {
            d2pvalid = true;
            
            theta = mathGeneral::PI*0.5 - cam_pitch + elevation;
            //d2p = cam_height * tan(theta);
            d2p = cam_height / cos(theta);
        }
//        cout << "Ball::distanceToBall: x, y: ";
//        cout << m_location_pixels.x << " " << m_location_pixels.y << endl;
//        cout << "Ball::distanceToBall: bearing, elevation, cam_height, cam_pitch: ";
//        cout << bearing << " " << elevation << " " << cam_height << " " << cam_pitch << endl;
//        cout << "Ball::distanceToBall: new d2p: ";
    }
    
    #if VISION_FIELDOBJECT_VERBOSITY > 1
        if(!d2pvalid)
            debug << "Ball::distanceToBall: d2p invalid - most methods will only return width_dist" << endl;
    #endif
        
    //get distance from width
    width_dist = VisionConstants::BALL_WIDTH*vbb->getCameraDistanceInPixels()/m_size_on_screen.x;

    #if VISION_FIELDOBJECT_VERBOSITY > 1
        debug << "Ball::distanceToBall: bearing: " << bearing << " elevation: " << elevation << endl;
        debug << "Ball::distanceToBall: d2p: " << d2p << endl;
        debug << "Ball::distanceToBall: m_size_on_screen.x: " << m_size_on_screen.x << endl;
        debug << "Ball::distanceToBall: width_dist: " << width_dist << endl;
    #endif
    cout << "Ball::distanceToBall: d2p: " << d2p << " width_dist: " << width_dist;
    switch(VisionConstants::BALL_DISTANCE_METHOD) {
    case VisionConstants::D2P:
        #if VISION_FIELDOBJECT_VERBOSITY > 1
            debug << "Ball::distanceToBall: Method: Combo" << endl;
        #endif
        return d2p;
    case VisionConstants::Width:
        #if VISION_FIELDOBJECT_VERBOSITY > 1
            debug << "Ball::distanceToBall: Method: Width" << endl;
        #endif
        return width_dist;
    case VisionConstants::Average:
        #if VISION_FIELDOBJECT_VERBOSITY > 1
            debug << "Ball::distanceToBall: Method: Average" << endl;
        #endif
        //average distances
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
