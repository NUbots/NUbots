#include "Object.h"
#include "../../Tools/Math/Vector2.h"
#include "../../Tools/Math/Vector3.h"
#include <string>

class StationaryObject: public Object{
	private:
                int ID;
                Vector2<float> fieldLocation;

	public:
                StationaryObject(const Vector2<float>& initialFieldLocation, int id = -1, const std::string& initName = "Unknown");
                StationaryObject(float x = 0, float y = 0, int id = -1, const std::string& initName = "Unknown");
                StationaryObject(const StationaryObject& otherObject);
		~StationaryObject();


		//Access:
                Vector2<float> getFieldLocation() const {return fieldLocation;}
		//ShortCuts:
                float X() const {return fieldLocation.x;}
                float Y() const {return fieldLocation.y;}

};

