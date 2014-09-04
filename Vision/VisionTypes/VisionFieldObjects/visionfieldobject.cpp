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

Vector2<double> VisionFieldObject::getLocationPixels() const
{
    return m_location.screenCartesian;
}
Vector2<double> VisionFieldObject::getLocationAngular() const
{
    return m_location.screenAngular;
}
