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

    output.write(reinterpret_cast<const char*>(&p_stat.fieldLocation.x), sizeof(p_stat.fieldLocation.x));
    output.write(reinterpret_cast<const char*>(&p_stat.fieldLocation.y), sizeof(p_stat.fieldLocation.y));

    return output;
}

std::istream& operator>> (std::istream& input, StationaryObject& p_stat)
{
    input >> *static_cast<Object*>(&p_stat);

    input.read(reinterpret_cast<char*>(&p_stat.fieldLocation.x), sizeof(p_stat.fieldLocation.x));
    input.read(reinterpret_cast<char*>(&p_stat.fieldLocation.y), sizeof(p_stat.fieldLocation.y));

    return input;
}

