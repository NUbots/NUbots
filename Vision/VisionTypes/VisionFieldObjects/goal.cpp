#include "goal.h"
#include "Vision/visionblackboard.h"
#include "Vision/visionconstants.h"

#include "debug.h"
#include "debugverbosityvision.h"

#include "Kinematics/Kinematics.h"
#include "Tools/Math/Matrix.h"

string Goal::getIDName(ID id)
{
    switch(id) {
    case YellowLeft:        return "YellowLeft";
    case YellowRight:       return "YellowRight";
    case YellowUnknown:     return "YellowUnknown";
    case BlueLeft:          return "BlueLeft";
    case BlueRight:         return "BlueRight";
    case BlueUnknown:       return "BlueUnknown";
    case Invalid:           return "Invalid";
    }
}

Goal::Goal(ID id, const Quad &corners)
{
    m_id = id;
    m_corners = corners;
    //SET WIDTH
    m_size_on_screen = Vector2<int>(corners.getWidth(), corners.getHeight());
    m_bottom_centre = corners.getBottomCentre();
    m_centre = corners.getCentre();
    //CALCULATE DISTANCE AND BEARING VALS
    calculatePositions();
}


Vector3<float> Goal::getRelativeFieldCoords() const
{
    return m_spherical_position;
}

bool Goal::addToExternalFieldObjects(FieldObjects *fieldobjects, float timestamp) const
{
    #if VISION_FIELDOBJECT_VERBOSITY > 1
        debug << "Goal::addToExternalFieldObjects - m_id: " << getIDName(m_id) << endl;
    #endif
    AmbiguousObject newAmbObj;

    switch(m_id) {
    case YellowLeft:
        
        break;
    case YellowRight:
        
        break;
    case YellowUnknown:
        newAmbObj = AmbiguousObject(FieldObjects::FO_YELLOW_GOALPOST_UNKNOWN, "Unknown Yellow Post");
        newAmbObj.addPossibleObjectID(FieldObjects::FO_YELLOW_LEFT_GOALPOST);
        newAmbObj.addPossibleObjectID(FieldObjects::FO_YELLOW_RIGHT_GOALPOST);
        newAmbObj.UpdateVisualObject(m_transformed_spherical_pos,
                                     m_spherical_error,
                                     m_location_angular,
                                     m_centre,
                                     m_size_on_screen,
                                     timestamp);
        fieldobjects->ambiguousFieldObjects.push_back(newAmbObj);
        break;
    case BlueLeft:
        
        break;
    case BlueRight:
        
        break;
    case BlueUnknown:
        newAmbObj = AmbiguousObject(FieldObjects::FO_BLUE_GOALPOST_UNKNOWN, "Unknown Blue Post");
        newAmbObj.addPossibleObjectID(FieldObjects::FO_BLUE_LEFT_GOALPOST);
        newAmbObj.addPossibleObjectID(FieldObjects::FO_BLUE_RIGHT_GOALPOST);
        newAmbObj.UpdateVisualObject(m_transformed_spherical_pos,
                                     m_spherical_error,
                                     m_location_angular,
                                     m_centre,
                                     m_size_on_screen,
                                     timestamp);
        fieldobjects->ambiguousFieldObjects.push_back(newAmbObj);
        break;
    default:
        //invalid object - do not push to fieldobjects
        errorlog << "Goal::addToExternalFieldObjects - attempt to add invalid Goal object" << endl;
        #if VISION_FIELDOBJECT_VERBOSITY > 1
            debug << "Goal::addToExternalFieldObjects - attempt to add invalid Goal object" << endl;
        #endif
        return false;
    }
}

void Goal::calculatePositions()
{
    VisionBlackboard* vbb = VisionBlackboard::getInstance();
    //To the bottom of the Goal Post.
    float distance;
    float bearing = (float)vbb->calculateBearing(m_bottom_centre.x);
    float elevation = (float)vbb->calculateElevation(m_bottom_centre.y);

    m_spherical_position[0] = distance;//distance
    m_spherical_position[1] = bearing;
    m_spherical_position[2] = elevation;
    
    m_location_angular = Vector2<float>(bearing, elevation);
    //m_spherical_error - not calculated
    
    
    if(vbb->isCameraTransformValid())
    {        
        Matrix cameraTransform = Matrix4x4fromVector(vbb->getCameraTransformVector());
        m_transformed_spherical_pos = Kinematics::TransformPosition(cameraTransform,m_spherical_position);
    }
}

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
        distance = VisionConstants::GOAL_WIDTH*vbb->getCameraDistanceInPixels()/m_size_on_screen.x;
        break;
    }
    
    return distance;
}
