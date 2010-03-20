#include "WorldModelShareObject.h"

WorldModelShareObject::WorldModelShareObject()
{
}

WorldModelShareObject::WorldModelShareObject(double initX, double initY, double initSRXX, double initSRXY, double initSRYY):
        fieldLocation(initX,initY),
        SRXX(initSRXX),
        SRXY(initSRXY),
        SRYY(initSRYY)
{
}

WorldModelShareObject::WorldModelShareObject(const Vector2<float>& initLocation, double initSRXX, double initSRXY, double initSRYY):
        fieldLocation(initLocation),
        SRXX(initSRXX),
        SRXY(initSRXY),
        SRYY(initSRYY)
{
}

WorldModelShareObject::WorldModelShareObject(const WorldModelShareObject& srcObj):
        fieldLocation(srcObj.fieldLocation),
        SRXX(srcObj.SRXX),
        SRXY(srcObj.SRXY),
        SRYY(srcObj.SRYY)
{
}
