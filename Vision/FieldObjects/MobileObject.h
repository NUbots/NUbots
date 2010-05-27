#ifndef MOBILEOBJECTS_H
#define MOBILEOBJECTS_H


#include "Object.h"
#include "../../Tools/Math/Vector2.h"
#include "../../Tools/Math/Vector3.h"

class MobileObject : public Object{
	private:
                Vector2<float> estimatedFieldLocation;
                Vector2<float> estimatedFieldLocationError;
                Vector2<float> estimatedVelocity;
                Vector2<float> estimatedVelocityError;
	public:
		~MobileObject();
                MobileObject(int initID = -1, const std::string& initName = "Unknown");
                MobileObject(const Vector2<float>& newAbsoluteLocation, int initID = -1, const std::string& initName = "Unknown");
                MobileObject(const MobileObject& srcObj);
                void updateAbsoluteLocation(const Vector2<float>& newAbsoluteLocation);
                void updateAbsoluteLocationError(const Vector2<float>& newAbsoluteLocationError);
		void updateObjectLocation(float x, float y, float sdx, float sdy);
		
                void updateVelocity(const Vector2<float>& newVelocity);
                void updateVelocityError(const Vector2<float>& newVelocityError);
		void updateObjectVelocities(float velX, float velY, float sdVelX, float sdVelY);

		//Access:
                Vector2<float> getEstimatedFieldLocation() const {return estimatedFieldLocation;}
                Vector2<float> getEstimatedFieldLocationError() const {return estimatedFieldLocationError;}
                Vector2<float> getEstimatedVelocity() const {return estimatedVelocity;}
                Vector2<float> getEstimatedVelocityError() const {return estimatedVelocityError;}
		
		//ShortCuts (single variableAccess):
                float X() const {return estimatedFieldLocation.x;}
                float Y() const {return estimatedFieldLocation.y;}
                float sdX() const {return estimatedFieldLocationError.x;}
                float sdY() const {return estimatedFieldLocationError.y;}
                float velX() const {return estimatedVelocity.x;}
                float velY() const {return estimatedVelocity.y;}
                float sdVelX() const {return estimatedVelocityError.x;}
                float sdVelY() const {return estimatedVelocityError.y;}		
};
#endif



