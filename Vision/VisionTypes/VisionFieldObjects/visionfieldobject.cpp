#include "visionfieldobject.h"
#include "Vision/visionconstants.h"

#include "beacon.h"
#include "goal.h"
#include "ball.h"
#include "fieldline.h"
#include "obstacle.h"

VisionFieldObject::VisionFieldObject()
{
    
}

const Point &VisionFieldObject::getLocationPixels() const
{
    return m_location_pixels;
}
const Vector2<float>& VisionFieldObject::getLocationAngular() const
{
    return m_location_angular;
}
