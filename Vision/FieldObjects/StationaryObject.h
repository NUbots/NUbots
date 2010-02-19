#include "Object.h"
#include "../../Tools/Math/Vector2.h"
#include "../../Tools/Math/Vector3.h"

class StationaryObject: public Object{
	private:
		Vector2<float> AbsoluteLocation;
		Vector3<float> WorldModelRelativeLocation;
	public:
		StationaryObject();
		~StationaryObject();
		StationaryObject(Vector2<float> newAbsoluteLocation);
		StationaryObject(float x, float y);
		
		void updateWorldModelRelativeLocation(Vector3<float> newWMRelLoc);
		void updateWorldModelRelativeVariables(float distance, float bearing, float elevation);
		//Access:
		Vector2<float> getAbsoluteLocation(){return AbsoluteLocation;}
		//ShortCuts:
		float X(){return AbsoluteLocation[0];}
		float Y(){return AbsoluteLocation[1];}
		float wmDistance(){return WorldModelRelativeLocation[0];}
		float wmBearing(){return WorldModelRelativeLocation[1];}
		float wmElevation(){return WorldModelRelativeLocation[2];}	
};

