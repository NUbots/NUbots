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
    calculatePositions();
}

Ball::Ball(const PointType& centre, float radius)
{
    m_radius = radius;
    m_location_pixels.x = centre.x;
    m_location_pixels.y = centre.y;
    m_size_on_screen = Vector2<int>(radius*2, radius*2);
    calculatePositions();
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
    
    //add ball to mobileFieldObjects
    fieldobjects->mobileFieldObjects[FieldObjects::FO_BALL].UpdateVisualObject(m_transformed_spherical_pos,
                                                                    m_spherical_error,
                                                                    m_location_angular,
                                                                    m_location_pixels,
                                                                    m_size_on_screen,
                                                                    timestamp);
    return true;
}

void Ball::calculatePositions()
{
    VisionBlackboard* vbb = VisionBlackboard::getInstance();
    //To the bottom of the Goal Post.
    float bearing = (float)vbb->calculateBearing(m_location_pixels.x);
    float elevation = (float)vbb->calculateElevation(m_location_pixels.y);
    
    float distance = distanceToBall(bearing, elevation);
    
    #if VISION_FIELDOBJECT_VERBOSITY > 1
        debug << "Ball::calculatePositions() distance: " << distance << endl;
    #endif
    //! @todo implement 
    m_spherical_position[0] = distance; //dist
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
        m_transformed_spherical_pos = Kinematics::TransformPosition(cameraToGroundTransform,m_spherical_position);
    }
    else {
        m_transformed_spherical_pos = Vector3<float>(0,0,0);
    }
}


/*!
*   @brief Calculates the distance using the set METHOD and the provided coordinate angles.
*   @param bearing The angle about the z axis.
*   @param elevation The angle about the y axis.
*/
float Ball::distanceToBall(float bearing, float elevation) const {
    VisionBlackboard* vbb = VisionBlackboard::getInstance();
    float distance = 0;
    switch(METHOD) {
    case D2P:    
        if(vbb->isCameraToGroundValid())
        {
            Matrix camera2groundTransform = Matrix4x4fromVector(vbb->getCameraToGroundVector());
            Vector3<float> result = Kinematics::DistanceToPoint(camera2groundTransform, bearing, elevation);
            distance = result[0];
        }
        #if VISION_FIELDOBJECT_VERBOSITY > 1
            debug << "Ball::distanceToBall: Method: D2P" << endl;
            debug << "Ball::distanceToBall: bearing: " << bearing << " elevation: " << elevation << endl;
            debug << "Ball::distanceToBall distance: " << distance << endl;
        #endif
        break;
    case Width:
        distance = VisionConstants::BALL_WIDTH*vbb->getCameraDistanceInPixels()/m_size_on_screen.x;
        #if VISION_FIELDOBJECT_VERBOSITY > 1
            debug << "Ball::distanceToBall: Method: Width" << endl;
            debug << "Ball::distanceToBall: m_size_on_screen.x: " << m_size_on_screen.x << endl;
            debug << "Ball::distanceToBall distance: " << distance << endl;
        #endif
        break;
    }
    
    return distance;
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
