
#ifndef CORNERPOINT_H
#define CORNERPOINT_H

#include "../Tools/Math/LSFittedLine.h"
class CornerPoint {
	public:
		  double PosX, PosY;
		  bool Valid;
		  LSFittedLine Line[2];
		  unsigned short CornerType, FieldObjectID;
		  short Orientation, Direction;
                  bool isObtuse;
		  double visBearing;
		  double visDistance;
		  double visElevation;
};

#endif
