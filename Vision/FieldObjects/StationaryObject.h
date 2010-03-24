#include "Object.h"
#include "../../Tools/Math/Vector2.h"
#include "../../Tools/Math/Vector3.h"
#include <string>

class StationaryObject: public Object{
	private:
                int ID;
                Vector2<float> fieldLocation;
                Vector3<float> estimatedRelativeLocation;
	public:
                StationaryObject(const Vector2<float>& initialFieldLocation, int id = -1, const std::string& initName = "Unknown");
                StationaryObject(float x = 0, float y = 0, int id = -1, const std::string& initName = "Unknown");
                StationaryObject(const StationaryObject& otherObject);
		~StationaryObject();

                void updateEstimatedRelativeLocation(const Vector3<float>& newWMRelLoc);
                void updateEstimatedRelativeVariables(float distance, float bearing, float elevation);
		//Access:
                Vector2<float> getFieldLocation() const {return fieldLocation;}
		//ShortCuts:
                float X() const {return fieldLocation.x;}
                float Y() const {return fieldLocation.y;}
                float estimatedDistance() const {return estimatedRelativeLocation.x;}
                float estimatedBearing() const {return estimatedRelativeLocation.y;}
                float estimatedElevation() const {return estimatedRelativeLocation.z;}
};

