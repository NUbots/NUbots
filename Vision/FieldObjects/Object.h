#ifndef FIELDOBJECT_H
#define FIELDOBJECT_H

#include "../../Tools/Math/Vector2.h"
#include "../../Tools/Math/Vector3.h"
#include <string>

class Object
{
    private:
        int ID;
        std::string name;
        Vector3<float> measuredRelativePosition;
        Vector3<float> relativeMeasurementError;
        
        // Vision Parameters:
        bool isVisible;                     // true if the object was seen in this image, false otherwise
        Vector2<int> imagePosition;         // Position on Screen (centre of object)
        Vector2<int> sizeOnScreen;          // (x,y) = (width, height) with ImagePosition at the center.
        float timeLastSeen;                 // The time in ms the object was last seen
        float timeSinceLastSeen;            // The time in ms since the object was last seen
        float timeSeen;                     // The consecutive time in ms the object has been seen
        float previousFrameTimestamp;       // The previous frame's timestamp (I use this to increment the timeSeen)




    public:
        Object(int initID = -1, const std::string& initName = "Unknown");
        ~Object();

        void preProcess(const float timestamp);

        void UpdateVisualObject(    const Vector3<float>& newMeasured,
                                    const Vector3<float>& newMeasuredError,
                                    const Vector2<int>& newImagePosition,
                                    const Vector2<int>& newSizeOnScreen,
                                    const float timestamp);

        void postProcess(const float timestamp);

        int getID() const {return ID;};
        std::string getName() const {return name;};

        Vector3<float> getMeasuredRelativeLocation() const {return measuredRelativePosition;}
        //void setRelativeLocation(Vector3<float> newSphericalPosition);
	//void setRelativeLocationVariables(float distance, float bearing, float elevation);
    
        Vector3<float> getRelativeMeasurementError() const {return relativeMeasurementError;}
        //void setRelativeLocationError(Vector3<float> newSphericalError);

        //void setViewPosition(Vector2<int> newViewPosition);
	//void setViewPositionVariables(int x, int y);
        Vector2<int> getImagePosition() const {return imagePosition;}
        //For COPY of whole object:
        void CopyObject(const Object& sourceObject);

        //Access vision variables:
        bool isObjectVisible() const {return isVisible;}
        float TimeLastSeen() const {return timeLastSeen;}
        float TimeSinceLastSeen() const {return timeSinceLastSeen;}
        float TimeSeen() const {return timeSeen;}
        float measuredDistance() const {return measuredRelativePosition.x;}
        float measuredBearing() const {return measuredRelativePosition.y;}
        float measuredElevation() const {return measuredRelativePosition.z;}
        int ScreenX() const {return imagePosition.x;}
        int ScreenY() const {return imagePosition.y;}
        int getObjectWidth() const {return sizeOnScreen.x;}
        int getObjectHeight() const {return sizeOnScreen.y;}

};

#endif // FIELDOBJECT_H
