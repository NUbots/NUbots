#include "obstacle.h"
#include "Vision/visionblackboard.h"

#include "debug.h"
#include "debugverbosityvision.h"

#include "Kinematics/Kinematics.h"
#include "Tools/Math/Matrix.h"

Obstacle::Obstacle(const PointType &position, int width, int height)
{
    m_size_on_screen = Vector2<int>(width, height);
    m_bottom_centre = Vector2<int>(position.x, position.y);
    //CALCULATE DISTANCE AND BEARING VALS
    valid = calculatePositions();
    valid = valid && check();
}


Vector3<float> Obstacle::getRelativeFieldCoords() const
{
    return m_spherical_position;
}

bool Obstacle::addToExternalFieldObjects(FieldObjects *fieldobjects, float timestamp) const
{
#if VISION_FIELDOBJECT_VERBOSITY > 1
    debug << "Obstacle::addToExternalFieldObjects" << endl;
    debug << "    " << *this << endl;
#endif
if(valid) {
    #if VISION_FIELDOBJECT_VERBOSITY > 1
        debug << "Obstacle::addToExternalFieldObjects - valid" << endl;
    #endif
    AmbiguousObject newAmbObj = AmbiguousObject(FieldObjects::FO_OBSTACLE, "Unknown Obstacle");
    //newAmbObj.addPossibleObjectID(FieldObjects::FO_BLUE_ROBOT_UNKNOWN);
    newAmbObj.UpdateVisualObject(m_transformed_spherical_pos,
                                 m_spherical_error,
                                 m_location_angular,
                                 m_bottom_centre,
                                 m_size_on_screen,
                                 timestamp);
    fieldobjects->ambiguousFieldObjects.push_back(newAmbObj);
    return true;
}
else {
    #if VISION_FIELDOBJECT_VERBOSITY > 1
        debug << "Obstacle::addToExternalFieldObjects - invalid" << endl;
    #endif
    return false;
}
}

bool Obstacle::check() const
{
    //! @todo Do a check based on width and d2p consistency
    if(!distance_valid) {
        #if VISION_FIELDOBJECT_VERBOSITY > 1
            debug << "Obstacle::check - Obstacle thrown out: distance invalid" << endl;
        #endif
        return false;
    }

    //all checks passed
    return true;
}

bool Obstacle::calculatePositions()
{
    VisionBlackboard* vbb = VisionBlackboard::getInstance();
    //To the bottom of the Goal Post.
    bool transform_valid;
    float bearing = (float)vbb->calculateBearing(m_bottom_centre.x);
    float elevation = (float)vbb->calculateElevation(m_bottom_centre.y);

    float distance = distanceToObstacle(bearing, elevation);

    //Camera to Ground to calculate distance
    
    m_spherical_position[0] = distance;//distance
    m_spherical_position[1] = bearing;
    m_spherical_position[2] = elevation;
    
    m_location_angular = Vector2<float>(bearing, elevation);
    //m_spherical_error - not calculated
    
    //Camera Transform to transform position
    if(vbb->isCameraTransformValid())
    {        
        Matrix cameraTransform = Matrix4x4fromVector(vbb->getCameraTransformVector());
        m_transformed_spherical_pos = Kinematics::TransformPosition(cameraTransform,m_spherical_position);
        transform_valid = true;
    }
    else {
        transform_valid = false;
        m_transformed_spherical_pos = Vector3<float>(0,0,0);
        #if VISION_FIELDOBJECT_VERBOSITY > 1
            debug << "Obstacle::calculatePositions: Kinematics CTG transform invalid - will not push obstacle" << endl;
        #endif
    }

    #if VISION_FIELDOBJECT_VERBOSITY > 2
        debug << "Obstacle::calculatePositions: ";
        debug << d2p << " " << distance << " " << m_transformed_spherical_pos.x << endl;
    #endif

    return transform_valid;
}

/*!
*   @brief Calculates the distance using the set METHOD and the provided coordinate angles.
*   @param bearing The angle about the z axis.
*   @param elevation The angle about the y axis.
*/
float Obstacle::distanceToObstacle(float bearing, float elevation) {
    VisionBlackboard* vbb = VisionBlackboard::getInstance();
    //reset distance values
    d2p = 0;
    distance_valid = vbb->distanceToPoint(bearing, elevation, d2p);

    #if VISION_FIELDOBJECT_VERBOSITY > 1
        if(!distance_valid)
            debug << "Obstacle::distanceToGoal: d2p invalid - will not push obstacle" << endl;
        debug << "Goal::distanceToGoal: bearing: " << bearing << " elevation: " << elevation << endl;
        debug << "Goal::distanceToGoal: d2p: " << d2p << endl;
    #endif
    return d2p;
}

/*! @brief Stream insertion operator for a single ColourSegment.
 *      The segment is terminated by a newline.
 */
ostream& operator<< (ostream& output, const Obstacle& o)
{
    output << "Obstacle - pixelloc: [" << o.getLocationPixels().x << ", " << o.getLocationPixels().y << "]";
    output << " angularloc: [" << o.getLocationAngular().x << ", " << o.getLocationAngular().y << "]";
    output << " relative field coords: [" << o.getRelativeFieldCoords().x << ", " << o.getRelativeFieldCoords().y << ", " << o.getRelativeFieldCoords().z << "]";
    return output;
}

/*! @brief Stream insertion operator for a vector of ColourSegments.
 *      Each segment is terminated by a newline.
 *  @relates ColourSegment
 */
ostream& operator<< (ostream& output, const vector<Obstacle>& o)
{
    for (size_t i=0; i<o.size(); i++)
        output << o[i] << endl;
    return output;
}
