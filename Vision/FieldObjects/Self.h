#ifndef FIELDOBJECTS_SELF_H
#define FIELDOBJECTS_SELF_H

#include "../../Tools/Math/Vector3.h"
#include <vector>
class StationaryObject;
class MobileObject;

class Self {
	private:
		Vector3<float> WorldModelLocation;
		Vector3<float> WorldModelLocationError;
	
	public:
		Self();
		~Self();
		Self(float x, float y);
        void updateLocationOfSelf(float wmX, float wmY, float heading, float sdX, float sdY, float sdHeading);
		float wmX(){return WorldModelLocation[0];}
		float wmY(){return WorldModelLocation[1];}
		float Heading(){return WorldModelLocation[2];}
        float sdX() {return WorldModelLocationError[0];}
        float sdY() {return WorldModelLocationError[1];}
        float sdHeading() {return WorldModelLocationError[2];}
        bool lost();
    
        std::vector<float> CalculateDifferenceFromFieldState(const std::vector<float> desiredState);
        float CalculateDistanceToStationaryObject(const StationaryObject& theObject);
        float CalculateBearingToStationaryObject(const StationaryObject& theObject);
        std::vector<float> CalculateClosestInterceptToMobileObject(const MobileObject& theObject);
        float CalculateYInterceptOfMobileObject(const MobileObject& theObject);
		float CalculateXInterceptOfMobileObject(const MobileObject& theObject);
};

#endif