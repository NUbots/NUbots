#include "visionfieldobject.h"
#include "Vision/visionconstants.h"

//std::map<VisionFieldObject::VFO_ID, VisionID::EXTERNAL_FIELD_OBJECT_ID> createMap()
//{
//  std::map<VisionFieldObject::VFO_ID, VisionID::EXTERNAL_FIELD_OBJECT_ID> m;
//  m[VisionFieldObject::BALL]            = VisionID::BALL;
//  m[VisionFieldObject::GOAL_Y]          = VisionID::GOAL_Y;
//  m[VisionFieldObject::GOAL_B]          = VisionID::GOAL_B;
//  m[VisionFieldObject::LINE]            = VisionID::LINE;
//  m[VisionFieldObject::CORNER]          = VisionID::CORNER;
//  m[VisionFieldObject::CENTRE_CIRCLE]   = VisionID::CENTRE_CIRCLE;
//  m[VisionFieldObject::OBSTACLE]        = VisionID::OBSTACLE;
//  m[VisionFieldObject::UNKNOWN]         = VisionID::UNKNOWN;
//  return m;
//}

//static std::map<VisionFieldObject::VFO_ID, VisionID::EXTERNAL_FIELD_OBJECT_ID> id_map = createMap();

string VisionFieldObject::getVFOName(VFO_ID id)
{
    switch(id) {
    case BALL:          return "BALL";
    case GOAL_Y:        return "GOAL_Y";
    case GOAL_B:        return "GOAL_B";
    case LINE:          return "LINE";
    case CORNER:        return "CORNER";
    case CENTRE_CIRCLE: return "CENTRE_CIRCLE";
    case OBSTACLE:      return "OBSTACLE";
    case UNKNOWN:       return "UNKNOWN";
    }
}

VisionFieldObject::VFO_ID VisionFieldObject::getVFOFromName(const string &name)
{
    if(name.compare("BALL") == 0)
        return BALL;
    else if(name.compare("GOAL_Y") == 0)
        return GOAL_Y;
    else if(name.compare("GOAL_B") == 0)
        return GOAL_B;
    else if(name.compare("LINE") == 0)
        return LINE;
    else if(name.compare("CORNER") == 0)
        return CORNER;
    else if(name.compare("CENTRE_CIRCLE") == 0)
        return CENTRE_CIRCLE;
    else if(name.compare("OBSTACLE") == 0)
        return OBSTACLE;
    else
        return UNKNOWN;
}

Vector2<float> VisionFieldObject::correctDistortion(const Vector2<float>& pt)
{
    Vector2<float> centre_relative = pt - Vector2<float>(160,120);
    float r2 = centre_relative.x*centre_relative.x + centre_relative.y*centre_relative.y;
    float corr_factor = 1 + VisionConstants::RADIAL_CORRECTION_COEFFICIENT*r2;
    Vector2<float> result = centre_relative* corr_factor;
    return Vector2<float>(result.x, result.y) + Vector2<float>(160,120);  //need to correct the additive
}

VisionFieldObject::VisionFieldObject()
{
    
}

const Vector2<int>& VisionFieldObject::getLocationPixels() const
{
    return m_location_pixels;
}
const Vector2<float>& VisionFieldObject::getLocationAngular() const
{
    return m_location_angular;
}
