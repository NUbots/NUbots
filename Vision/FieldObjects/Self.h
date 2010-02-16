
#include "../../Tools/Math/Vector3.h"

class Self {
	private:
		Vector3<float> WorldModelLocation;
		Vector3<float> WorldModelLocationError;
	
	public:
		Self();
		~Self();
		Self(float x, float y);
                void updateLocationOfSelf(float wmX, float wmY, float heading);
		float wmX(){return WorldModelLocation[0];}
		float wmY(){return WorldModelLocation[1];}
		float Heading(){return WorldModelLocation[2];}
		
};
