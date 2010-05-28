#include "Self.h"
#include "StationaryObject.h"
#include "Tools/Math/General.h"
using namespace mathGeneral;
Self::Self()
{
	WorldModelLocation[0] = 0;
	WorldModelLocation[1] = 0;
	WorldModelLocation[2] = 0;
}
Self::~Self()
{
}


Self::Self(float wmX, float wmY)
{
	WorldModelLocation[0] = wmX;
        WorldModelLocation[1] = wmY;
        WorldModelLocation[2] = 0;
}
void Self::updateLocationOfSelf(float wmX, float wmY, float heading)
{
	WorldModelLocation[0] = wmX;
        WorldModelLocation[1] = wmY;
        WorldModelLocation[2] = heading;
}

std::vector<float> Self::CalculateDifferenceFromFieldState(const std::vector<float> targetState)
{
    float selfX = WorldModelLocation[0];
    float selfY = WorldModelLocation[1];
    float selfHeading = WorldModelLocation[2];

    float targetX = targetState[0];
    float targetY = targetState[1];
    float targetHeading = targetState[2];

    float diffX = targetX - selfX;
    float diffY = targetY - selfY;
    if( (diffX == 0) && (diffY == 0)) diffY = 0.0001;
    float positionHeading = atan2(diffY, diffX);

    float distance = sqrt( diffX * diffX + diffY * diffY );
    float bearing = normaliseAngle(positionHeading - selfHeading);
    float headingDifference = normaliseAngle(targetHeading - selfHeading);

    std::vector<float> result(3,0.0f);
    result[0] = distance;
    result[1] = bearing;
    result[2] = headingDifference;

    return result;
}

float Self::CalculateDistanceToStationaryObject(const StationaryObject& theObject)
{
    float selfX = WorldModelLocation[0];
    float selfY = WorldModelLocation[1];
    float diffX = theObject.X() - selfX;
    float diffY = theObject.Y() - selfY;
    float distance = sqrt( diffX * diffX + diffY * diffY );
    return distance;
}

float Self::CalculateBearingToStationaryObject(const StationaryObject& theObject)
{
    float selfX = WorldModelLocation[0];
    float selfY = WorldModelLocation[1];
    float selfHeading = WorldModelLocation[2];
    float diffX = theObject.X() - selfX;
    float diffY = theObject.Y() - selfY;
    float positionHeading = atan2(diffY, diffX);
    float bearing = normaliseAngle(positionHeading - selfHeading);
    return bearing;
}
