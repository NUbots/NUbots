#include "goal.h"
#include "Vision/visionblackboard.h"
#include "Vision/visionconstants.h"

#include "debug.h"
#include "debugverbosityvision.h"

#include "Kinematics/Kinematics.h"
#include "Tools/Math/Matrix.h"

string Goal::getIDName(GoalID id)
{
    switch(id) {
    case YellowLeftGoal:        return "YellowLeftGoal";
    case YellowRightGoal:       return "YellowRightGoal";
    case YellowUnknownGoal:     return "YellowUnknownGoal";
    case BlueLeftGoal:          return "BlueLeftGoal";
    case BlueRightGoal:         return "BlueRightGoal";
    case BlueUnknownGoal:       return "BlueUnknownGoal";
    case InvalidGoal:           return "InvalidGoal";
    default:                    return "InvalidGoal";
    }
}

Goal::Goal(GoalID id, const Quad &corners)
{
    m_id = id;
    m_corners = corners;
    //SET WIDTH
    m_size_on_screen = Vector2<int>(corners.getWidth(), corners.getHeight());
    m_bottom_centre = corners.getBottomCentre();
    m_location_pixels = corners.getCentre();
    //CALCULATE DISTANCE AND BEARING VALS
    calculatePositions();
}

const Quad& Goal::getQuad() const
{
    return m_corners;
}

Goal::GoalID Goal::getID() const
{
    return m_id;
}

/*!
*   @brief Returns the position in spherical coordinates.
*   @return m_spherical_position The spherical coordinates for this goal.
*/
Vector3<float> Goal::getRelativeFieldCoords() const
{
    return m_spherical_position;
}

/*!
*   @brief Updates the external field objects with this goal.
*   @param fieldobjects A pointer to the external field objects.
*   @param timestamp The current timestamp to apply to the field objects.
*
*   This method uses the id of the goal to determine where to place it, it also
*   includes no checks before placing them, and nor should it. For example, if this is 
*   called on multiple YellowLeftGoals then localisation will only see the last one.
*/
bool Goal::addToExternalFieldObjects(FieldObjects *fieldobjects, float timestamp) const
{
    #if VISION_FIELDOBJECT_VERBOSITY > 1
        debug << "Goal::addToExternalFieldObjects - m_id: " << getIDName(m_id) << endl;
        debug << "    " << *this << endl;
    #endif
    AmbiguousObject newAmbObj;
    FieldObjects::StationaryFieldObjectID stat_id;
    bool stationary = false;

    switch(m_id) {
    case YellowLeftGoal:
        stat_id = FieldObjects::FO_YELLOW_LEFT_GOALPOST;
        stationary = true;
        break;
    case YellowRightGoal:
        stat_id = FieldObjects::FO_YELLOW_RIGHT_GOALPOST;
        stationary = true;
        break;
    case BlueLeftGoal:
        stat_id = FieldObjects::FO_BLUE_LEFT_GOALPOST;
        stationary = true;
        break;
    case BlueRightGoal:
        stat_id = FieldObjects::FO_BLUE_RIGHT_GOALPOST;
        stationary = true;
        break;
    case YellowUnknownGoal:
        newAmbObj = AmbiguousObject(FieldObjects::FO_YELLOW_GOALPOST_UNKNOWN, "Unknown Yellow Post");
        newAmbObj.addPossibleObjectID(FieldObjects::FO_YELLOW_LEFT_GOALPOST);
        newAmbObj.addPossibleObjectID(FieldObjects::FO_YELLOW_RIGHT_GOALPOST);
        stationary = false;
        break;
    case BlueUnknownGoal:
        newAmbObj = AmbiguousObject(FieldObjects::FO_BLUE_GOALPOST_UNKNOWN, "Unknown Blue Post");
        newAmbObj.addPossibleObjectID(FieldObjects::FO_BLUE_LEFT_GOALPOST);
        newAmbObj.addPossibleObjectID(FieldObjects::FO_BLUE_RIGHT_GOALPOST);
        stationary = false;
        break;
    default:
        //invalid object - do not push to fieldobjects
        errorlog << "Goal::addToExternalFieldObjects - attempt to add invalid Goal object" << endl;
        #if VISION_FIELDOBJECT_VERBOSITY > 1
            debug << "Goal::addToExternalFieldObjects - attempt to add invalid Goal object" << endl;
        #endif
        return false;
    }
    
    if(stationary) {
        //add post to stationaryFieldObjects
        fieldobjects->stationaryFieldObjects[stat_id].UpdateVisualObject(m_transformed_spherical_pos,
                                                                        m_spherical_error,
                                                                        m_location_angular,
                                                                        m_location_pixels,
                                                                        m_size_on_screen,
                                                                        timestamp);
    }
    else {
        //update ambiguous goal post and add it to ambiguousFieldObjects
        newAmbObj.UpdateVisualObject(m_transformed_spherical_pos,
                                     m_spherical_error,
                                     m_location_angular,
                                     m_location_pixels,
                                     m_size_on_screen,
                                     timestamp);
        fieldobjects->ambiguousFieldObjects.push_back(newAmbObj);
    }
    
    return true;
}

/*!
*   @brief Updates the spherical position, angular location and transformed spherical position.
*   
*   This method uses the camera transform and as such if it was not valid when retrieved from the wrapper
*   this will leave m_transformed_spherical_position at all zeros.
*/
void Goal::calculatePositions()
{
    VisionBlackboard* vbb = VisionBlackboard::getInstance();
    //To the bottom of the Goal Post.
    float bearing = (float)vbb->calculateBearing(m_bottom_centre.x);
    float elevation = (float)vbb->calculateElevation(m_bottom_centre.y);
    
    float distance = distanceToGoal(bearing, elevation);

    //debug << "Goal::calculatePositions() distance: " << distance << endl;
    
    m_spherical_position[0] = distance;//distance
    m_spherical_position[1] = bearing;
    m_spherical_position[2] = elevation;
    
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
float Goal::distanceToGoal(float bearing, float elevation) const {
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
        break;
    case Width:
        //debug << "Goal::distanceToGoal: m_size_on_screen.x: " << m_size_on_screen.x << endl;
        distance = VisionConstants::GOAL_WIDTH*vbb->getCameraDistanceInPixels()/m_size_on_screen.x;
        //debug << "Goal::distanceToGoal distance: " << distance << endl;
        break;
    }
    
    return distance;
}

/*! @brief Stream insertion operator for a single ColourSegment.
 *      The segment is terminated by a newline.
 */
ostream& operator<< (ostream& output, const Goal& g)
{
    output << "Goal - " << Goal::getIDName(g.m_id) << endl;
    output << "\tpixelloc: [" << g.m_location_pixels.x << ", " << g.m_location_pixels.y << "]" << endl;
    output << " angularloc: [" << g.m_location_angular.x << ", " << g.m_location_angular.y << "]" << endl;
    output << "\trelative field coords: [" << g.m_spherical_position.x << ", " << g.m_spherical_position.y << ", " << g.m_spherical_position.z << "]" << endl;
    output << "\ttransformed field coords: [" << g.m_transformed_spherical_pos.x << ", " << g.m_transformed_spherical_pos.y << ", " << g.m_transformed_spherical_pos.z << "]" << endl;
    output << "\tspherical error: [" << g.m_spherical_error.x << ", " << g.m_spherical_error.y << "]" << endl;
    output << "\tsize on screen: [" << g.m_size_on_screen.x << ", " << g.m_size_on_screen.y << "]";
    return output;
}

/*! @brief Stream insertion operator for a vector of ColourSegments.
 *      Each segment is terminated by a newline.
 *  @relates ColourSegment
 */
ostream& operator<< (ostream& output, const vector<Goal>& g)
{
    for (size_t i=0; i<g.size(); i++)
        output << g[i] << endl;
    return output;
}
