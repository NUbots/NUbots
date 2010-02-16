#include "Object.h"
#include "../../Tools/Math/Vector2.h"
#include "../../Tools/Math/Vector3.h"

class MobileObject : public Object{
	private:
		Vector2<float> AbsoluteLocation;
		Vector2<float> AbsoluteLocationError;
		Vector2<float> Velocity;
		Vector2<float> VelocityError;
		Vector3<float> WorldModelRelativeLocation;
		Vector3<float> WorldModelRelativeLocationError;
	public:
		MobileObject();
		~MobileObject();
		MobileObject(Vector2<float> newAbsoluteLocation);
		MobileObject(float x, float y);
		void updateAbsoluteLocation(Vector2<float> newAbsoluteLocation);
		void updateAbsoluteLocationError(Vector2<float> newAbsoluteLocationError);
		void updateObjectLocation(float x, float y, float sdx, float sdy);
		
		void updateVelocity(Vector2<float> newVelocity);
		void updateVelocityError(Vector2<float> newVelocityError);
		void updateObjectVelocities(float velX, float velY, float sdVelX, float sdVelY);
	
		void updateWorldModelRelativeLocation(Vector3<float> newWMRelLoc);
		void updateWorldModelRelativeLocationError(Vector3<float> newWNRelLocError);
		void updateWorldModelRelativeVaribles(float wmDistance, float wmBearing, float wmElevation, float sdWmDistance, float sdWmBearing, float sdWmElevation );

		//Access:
                Vector2<float> getAbsoluteLocation(){return AbsoluteLocation;}
                Vector2<float> getAbsoluteLocationError(){return AbsoluteLocationError;}
                Vector2<float> getVelocity(){return Velocity;}
                Vector2<float> getVelocityError(){return VelocityError;}
		Vector3<float> getWorldModelRelativeLocation(){return WorldModelRelativeLocation;}
		
		//ShortCuts (single variableAccess):
		float X(){return AbsoluteLocation[0];}
		float Y(){return AbsoluteLocation[1];}
		float sdX(){return AbsoluteLocationError[0];}
		float sdY(){return AbsoluteLocationError[1];}
		float velX(){return Velocity[0];}
		float velY(){return Velocity[1];}
		float sdVelX(){return VelocityError[0];}
		float sdVelY(){return VelocityError[1];}
		float wmDistance(){return WorldModelRelativeLocation[0];}
		float wmBearing(){return WorldModelRelativeLocation[1];}
		float wmElevation(){return WorldModelRelativeLocation[2];}
                float sdwmDistance(){return WorldModelRelativeLocationError[0];}
                float sdwmBearing(){return WorldModelRelativeLocationError[1];}
                float sdwmElevation(){return WorldModelRelativeLocationError[2];}
		
};
