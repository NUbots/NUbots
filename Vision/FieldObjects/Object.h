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
        //Vision Parameters:
        bool isVisible;
        Vector2<int> imagePosition;
        int numberOfTimesSeen;		// Number Of Times this objects been seen 
        int framesSinceLastSeen;	// Number of frames since we last saw this object
        float timeLastSeen;         // The time in ms the object was last seen
        int framesSeen;			// Number of consecutive frames seen from this object


    public:
        Object(int initID = -1, const std::string& initName = "Unknown");
        ~Object();

        void UpdateVisualObject(    const Vector3<float>& newMeasured,
                                    const Vector3<float>& newMeasuredError,
                                    const Vector2<int>& newImagePosition,
                                    const float timestamp);

        void ResetFrame();

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
        int FramesSeen() const {return framesSeen;}
        int NumberOfTimesSeen() const {return numberOfTimesSeen;}
        int FrameSinceLastSeen() const {return framesSinceLastSeen;}
        float TimeLastSeen() const {return timeLastSeen;}
        float measuredDistance() const {return measuredRelativePosition.x;}
        float measuredBearing() const {return measuredRelativePosition.y;}
        float measuredElevation() const {return measuredRelativePosition.z;}
        int ScreenX() const {return imagePosition.x;}
        int ScreenY() const {return imagePosition.y;}

};

#endif // FIELDOBJECT_H
