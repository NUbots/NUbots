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

    m_size_on_screen = Vector2<int>(corners.getWidth(), corners.getHeight());
    m_bottom_centre = corners.getBottomCentre();

//    if(VisionConstants::DO_RADIAL_CORRECTION) {
//        VisionBlackboard* vbb = VisionBlackboard::getInstance();
//        Vector2<float> corr_bottom_centre = vbb->correctDistortion(Vector2<float>(m_bottom_centre.x, m_bottom_centre.y));
//        m_bottom_centre.x = mathGeneral::roundNumberToInt(corr_bottom_centre.x);
//        m_bottom_centre.y = mathGeneral::roundNumberToInt(corr_bottom_centre.y);
//    }

    m_location_pixels = corners.getCentre();
    //CALCULATE DISTANCE AND BEARING VALS
    valid = calculatePositions();
    valid = valid && check();
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
    if(valid) {
        #if VISION_FIELDOBJECT_VERBOSITY > 1
            debug << "Goal::addToExternalFieldObjects - valid" << endl;
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
    else {
        #if VISION_FIELDOBJECT_VERBOSITY > 1
            debug << "Goal::addToExternalFieldObjects - invalid" << endl;
        #endif
        return false;
    }
}

bool Goal::check() const
{
    //various throwouts here

    if(!distance_valid) {
        #if VISION_FIELDOBJECT_VERBOSITY > 1
            debug << "Goal::check - Goal thrown out: distance invalid" << endl;
        #endif
        return false;
    }

    //throwout for base below horizon
    if(VisionConstants::THROWOUT_ON_ABOVE_KIN_HOR_GOALS and
       not VisionBlackboard::getInstance()->getKinematicsHorizon().IsBelowHorizon(m_bottom_centre.x, m_bottom_centre.y)) {
        #if VISION_FIELDOBJECT_VERBOSITY > 1
            debug << "Goal::check - Goal thrown out: base above kinematics horizon" << endl;
        #endif
        return false;
    }
    
    //Distance discrepency throwout - if width method says goal is a lot closer than d2p (by specified value) then discard
    if(VisionConstants::THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_GOALS and
            width_dist + VisionConstants::MAX_DISTANCE_METHOD_DISCREPENCY_GOALS < d2p) {
        #if VISION_FIELDOBJECT_VERBOSITY > 1
        debug << "Goal::check - Goal thrown out: width distance too much smaller than d2p" << endl;
            debug << "\td2p: " << d2p << " width_dist: " << width_dist << " MAX_DISTANCE_METHOD_DISCREPENCY_GOALS: " << VisionConstants::MAX_DISTANCE_METHOD_DISCREPENCY_GOALS << endl;
        #endif
        return false;
    }
    
    //throw out if goal is too far away
    if(VisionConstants::THROWOUT_DISTANT_GOALS and 
        m_transformed_spherical_pos.x > VisionConstants::MAX_GOAL_DISTANCE) {
        #if VISION_FIELDOBJECT_VERBOSITY > 1
            debug << "Goal::check - Goal thrown out: too far away" << endl;
            debug << "\td2p: " << m_transformed_spherical_pos.x << " MAX_GOAL_DISTANCE: " << VisionConstants::MAX_GOAL_DISTANCE << endl;
        #endif
        return false;
    }
    
    //all checks passed, keep goalpost
    return true;
}

/*!
*   @brief Updates the spherical position, angular location and transformed spherical position.
*   
*   This method uses the camera transform and as such if it was not valid when retrieved from the wrapper
*   this will leave m_transformed_spherical_position at all zeros.
*/
bool Goal::calculatePositions()
{
    VisionBlackboard* vbb = VisionBlackboard::getInstance();
    //To the bottom of the Goal Post.
    bool transform_valid;
    float bearing = (float)vbb->calculateBearing(m_bottom_centre.x);
    float elevation = (float)vbb->calculateElevation(m_bottom_centre.y);
    
    float distance = distanceToGoal(bearing, elevation);

    if(distance <= 0) {
        //object behind us - ignore it
        m_spherical_position = Vector3<float>(0,0,0);//distance
        m_location_angular = Vector2<float>(0,0);
        m_transformed_spherical_pos = Vector3<float>(0,0,0);
        #if VISION_FIELDOBJECT_VERBOSITY > 1
            debug << "Goal::calculateDistances - Goal thrown out: negative distance" << endl;
            debug << "\td2p: " << distance << endl;
        #endif
        return false;
    }

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
        transform_valid = true;
    }
    else {
        transform_valid = false;
        m_transformed_spherical_pos = Vector3<float>(0,0,0);
        #if VISION_FIELDOBJECT_VERBOSITY > 1
            debug << "Goal::calculatePositions: Kinematics CTG transform invalid - will not push goal" << endl;
        #endif
    }
    
    #if VISION_FIELDOBJECT_VERBOSITY > 2
        debug << "Goal::calculatePositions: ";
        debug << d2p << " " << width_dist << " " << distance << " " << m_transformed_spherical_pos.x << endl;
    #endif

    return transform_valid;
}

/*!
*   @brief Calculates the distance using the set METHOD and the provided coordinate angles.
*   @param bearing The angle about the z axis.
*   @param elevation The angle about the y axis.
*/
float Goal::distanceToGoal(float bearing, float elevation) {
    VisionBlackboard* vbb = VisionBlackboard::getInstance();
    //reset distance values
    bool d2pvalid = false;
    d2p = 0,
    width_dist = 0;
    //get distance to point from base
//    if(vbb->isCameraToGroundValid())
//    {
//        d2pvalid = true;
//        Matrix camera2groundTransform = Matrix4x4fromVector(vbb->getCameraToGroundVector());
//        Vector3<float> result = Kinematics::DistanceToPoint(camera2groundTransform, bearing, elevation);
//        d2p = result[0];
//    }
    d2pvalid = vbb->distanceToPoint(bearing, elevation, d2p);

    #if VISION_FIELDOBJECT_VERBOSITY > 1
        if(!d2pvalid)
            debug << "Goal::distanceToGoal: d2p invalid - combination methods will only return width_dist" << endl;
    #endif
    //get distance from width
    width_dist = VisionConstants::GOAL_WIDTH*vbb->getCameraDistanceInPixels()/m_size_on_screen.x;

    float HACKWIDTH = 50;
    float HACKPIXELS = 3;
    //HACK FOR GOALS AT BASE OF IMAGE
    if(m_size_on_screen.x > HACKWIDTH and (vbb->getImageHeight() - m_location_pixels.y) < HACKPIXELS) {
        #if VISION_FIELDOBJECT_VERBOSITY > 1
            debug << "Goal::distanceToGoal: Goal wide and cutoff at bottom so used width_dist" << endl;
        #endif
        return width_dist;
    }

    #if VISION_FIELDOBJECT_VERBOSITY > 1
        debug << "Goal::distanceToGoal: bearing: " << bearing << " elevation: " << elevation << endl;
        debug << "Goal::distanceToGoal: d2p: " << d2p << endl;
        debug << "Goal::distanceToGoal: m_size_on_screen.x: " << m_size_on_screen.x << endl;
        debug << "Goal::distanceToGoal: width_dist: " << width_dist << endl;
    #endif
    switch(VisionConstants::GOAL_DISTANCE_METHOD) {
    case VisionConstants::D2P:
        #if VISION_FIELDOBJECT_VERBOSITY > 1
            debug << "Goal::distanceToGoal: Method: D2P" << endl;
        #endif
        distance_valid = d2pvalid;
        return d2p;
    case VisionConstants::Width:
        #if VISION_FIELDOBJECT_VERBOSITY > 1
            debug << "Goal::distanceToGoal: Method: Width" << endl;
        #endif
        distance_valid = true;
        return width_dist;
    case VisionConstants::Average:
        #if VISION_FIELDOBJECT_VERBOSITY > 1
            debug << "Goal::distanceToGoal: Method: Average" << endl;
        #endif
        //average distances
        distance_valid = true;
        if(d2pvalid)
            return (d2p + width_dist) * 0.5;
        else
            return width_dist;
    case VisionConstants::Least:
        #if VISION_FIELDOBJECT_VERBOSITY > 1
            debug << "Goal::distanceToGoal: Method: Least" << endl;
        #endif
        distance_valid = true;
        if(d2pvalid)
            return min(d2p, width_dist);
        else
            return width_dist;
    }
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
