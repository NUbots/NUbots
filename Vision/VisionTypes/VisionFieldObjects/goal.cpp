#include "goal.h"

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
}


void Goal::getRelativeFieldCoords(vector<float>& coords) const
{
    
}

bool Goal::addToExternalFieldObjects(FieldObjects *fieldobjects) const
{
    #if VISION_FIELDOBJECT_VERBOSITY > 1
        debug << "Goal::addToExternalFieldObjects - m_id: " << getIDName(m_id) << endl;
    #endif
    
    Vector2<int> viewPosition;
    Vector2<int> sizeOnScreen;
    Vector3<float> sphericalError;
    Vector3<float> sphericalPosition = CalculateSphericalPosition(GoalPost, vision);
    viewPosition.x = GoalPost->getCentreX();
    viewPosition.y = GoalPost->getCentreY();
    Vector3 <float> transformedSphericalPosition;
    Vector2<float> screenPositionAngle(sphericalPosition[1], sphericalPosition[2]);
    
    vector<float> ctvector;
    bool isOK = vision->getSensorsData()->get(NUSensorsData::CameraTransform, ctvector);
    if(isOK == true)
    {
        Matrix cameraTransform = Matrix4x4fromVector(ctvector);
        transformedSphericalPosition = Kinematics::TransformPosition(cameraTransform,sphericalPosition);
    }

    sizeOnScreen.x = GoalPost->width();
    sizeOnScreen.y = GoalPost->height();     
        
    switch(m_id) {
    case YellowLeft:
        
        break;
    case YellowRight:
        
        break;
    case YellowUnknown:
        AmbiguousObject newAmbObj = AmbiguousObject(FieldObjects::FO_YELLOW_GOALPOST_UNKNOWN, "Unknown Yellow Post");
        newAmbObj.addPossibleObjectID(FieldObjects::FO_YELLOW_LEFT_GOALPOST);
        newAmbObj.addPossibleObjectID(FieldObjects::FO_YELLOW_RIGHT_GOALPOST);
        newAmbObj.UpdateVisualObject(   transformedSphericalPosition,
                                        sphericalError,
                                        screenPositionAngle,
                                        viewPosition,
                                        sizeOnScreen,
                                        vision->m_timestamp);
        if(transformedSphericalPosition[2] < 0.00)
            AllObjects->ambiguousFieldObjects.push_back(newAmbObj);
        break;
    case BlueLeft:
        
        break;
    case BlueRight:
        
        break;
    case BlueUnknown:
        AmbiguousObject newAmbObj = AmbiguousObject(FieldObjects::FO_BLUE_GOALPOST_UNKNOWN, "Unknown Blue Post");
        newAmbObj.addPossibleObjectID(FieldObjects::FO_BLUE_LEFT_GOALPOST);
        newAmbObj.addPossibleObjectID(FieldObjects::FO_BLUE_RIGHT_GOALPOST);
        newAmbObj.UpdateVisualObject(   transformedSphericalPosition,
                                        sphericalError,
                                        screenPositionAngle,
                                        viewPosition,
                                        sizeOnScreen,
                                        vision->m_timestamp);
        if(transformedSphericalPosition[2] < 0.00)
            AllObjects->ambiguousFieldObjects.push_back(newAmbObj);
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
