#ifndef WORLDMODELSHAREOBJECT_H
#define WORLDMODELSHAREOBJECT_H

#include "Tools/Math/Vector2.h"

class WorldModelShareObject
{
public:
    WorldModelShareObject();
    WorldModelShareObject(double initX, double initY, double initSRXX, double initSRXY, double initSRYY);
    WorldModelShareObject(const Vector2<float>& initLocation, double initSRXX, double initSRXY, double initSRYY);
    WorldModelShareObject(const WorldModelShareObject& srcObj);
    int seen;
    Vector2<float> fieldLocation;
    float SRXX; //For New LOCWM SharedBall
    float SRXY; //For New LOCWM SharedBall
    float SRYY; //For New LOCWM SharedBall
};

#endif // WORLDMODELSHAREOBJECT_H
