#include "Object.h"
#include "../../Tools/Math/Vector2.h"
#include "../../Tools/Math/Vector3.h"

class StationaryObject: public Object{
	private:
                int ID;
                Vector2<float> AbsoluteLocation;
		Vector3<float> WorldModelRelativeLocation;
	public:
                StationaryObject();
                StationaryObject(int id, Vector2<float> newAbsoluteLocation);
                StationaryObject(int id, float x, float y);
                StationaryObject(const StationaryObject& otherObject);
		~StationaryObject();

		void updateWorldModelRelativeLocation(Vector3<float> newWMRelLoc);
		void updateWorldModelRelativeVariables(float distance, float bearing, float elevation);
		//Access:
                Vector2<float> getAbsoluteLocation() const {return AbsoluteLocation;}
		//ShortCuts:
                int getID() const {return ID;};
                float X()const {return AbsoluteLocation.x;}
                float Y() const {return AbsoluteLocation.y;}
                float wmDistance() const {return WorldModelRelativeLocation.x;}
                float wmBearing() const {return WorldModelRelativeLocation.y;}
                float wmElevation() const {return WorldModelRelativeLocation.z;}
};

