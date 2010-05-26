#include "StationaryObject.h"

StationaryObject::StationaryObject(const Vector2<float>& initialFieldLocation, int id, const std::string& initName):
        Object(id, initName),
        fieldLocation(initialFieldLocation)
{

}

StationaryObject::StationaryObject(float x, float y, int id, const std::string& initName):
        Object(id, initName),
        fieldLocation(x,y)
{

}

StationaryObject::StationaryObject(const StationaryObject& otherObject):
        Object(otherObject.getID(), otherObject.getName()),
        fieldLocation(otherObject.getFieldLocation())
{

}

StationaryObject::~StationaryObject()
{

}


