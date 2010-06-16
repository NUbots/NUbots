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

std::ostream& operator<< (std::ostream& output, const StationaryObject& p_stat)
{
    output << *static_cast<const Object*>(&p_stat);
    output << p_stat.fieldLocation.x << ' ' << p_stat.fieldLocation.y << ' ' ;
    return output;
}

std::istream& operator>> (std::istream& input, StationaryObject& p_stat)
{
    input >> *static_cast<Object*>(&p_stat);
    input >> p_stat.fieldLocation.x;
    input >> p_stat.fieldLocation.y;
    return input;
}

