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
    calculatePositions();
}


Vector3<float> Obstacle::getRelativeFieldCoords() const
{
    return m_spherical_position;
}

bool Obstacle::addToExternalFieldObjects(FieldObjects *fieldobjects, float timestamp) const
{
    //! @todo implement robot mapping
    #if VISION_FIELDOBJECT_VERBOSITY > 1
        debug << "Goal::addToExternalFieldObjects - m_id: Obstacle" << endl;
    #endif
    AmbiguousObject newAmbObj = AmbiguousObject(FieldObjects::FO_OBSTACLE, "Unknown Yellow Post");
    //newAmbObj.addPossibleObjectID(FieldObjects::FO_BLUE_ROBOT_UNKNOWN);
    newAmbObj.UpdateVisualObject(m_transformed_spherical_pos,
                                 m_spherical_error,
                                 m_location_angular,
                                 m_bottom_centre,
                                 m_size_on_screen,
                                 timestamp);
    fieldobjects->ambiguousFieldObjects.push_back(newAmbObj);
}

void Obstacle::calculatePositions()
{
    VisionBlackboard* vbb = VisionBlackboard::getInstance();
    //To the bottom of the Goal Post.
    float distance;
    float bearing = (float)vbb->calculateBearing(m_bottom_centre.x);
    float elevation = (float)vbb->calculateElevation(m_bottom_centre.y);

    //Camera to Ground to calculate distance
    
    if(vbb->isCameraToGroundValid())
    {
        Matrix camera2groundTransform = Matrix4x4fromVector(vbb->getCameraToGroundVector());
        Vector3<float> result;
        result = Kinematics::DistanceToPoint(camera2groundTransform, bearing, elevation);
        distance = result[0];
        //bearing = result[1];
        //elevation = result[2];
    }
    
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
    }
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
