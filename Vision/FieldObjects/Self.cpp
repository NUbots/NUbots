#include "Self.h"
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
	WorldModelLocation[0] = wmY;
	WorldModelLocation[0] = 0;
}
void Self::updateLocationOfSelf(float wmX, float wmY, float heading)
{
	WorldModelLocation[0] = wmX;
	WorldModelLocation[0] = wmY;
	WorldModelLocation[0] = heading;
}