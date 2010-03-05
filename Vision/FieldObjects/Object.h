#ifndef FIELDOBJECT_H
#define FIELDOBJECT_H

#include "../../Tools/Math/Vector2.h"
#include "../../Tools/Math/Vector3.h"

class Object
{
    private:
        Vector3<float> sphericalPosition;
        Vector3<float> sphericalError;
        //Vision Parameters:
        bool isVisible;
        Vector2<int> viewPosition;
        int numberOfTimesSeen;		// Number Of Times this objects been seen 
        int framesSinceLastSeen;	// Number of frames since we last saw this object
        int framesSeen;			// Number of consecutive frames seen from this object


    public:
        Object();
        ~Object();

        void UpdateVisualObject( Vector3<float> newSpherical,
                                Vector3<float> newSphericalError,
                                Vector2<int> newViewPosition);

        void ResetFrame();


        Vector3<float> getRelativeLocation(){return sphericalPosition;}
        //void setRelativeLocation(Vector3<float> newSphericalPosition);
	//void setRelativeLocationVariables(float distance, float bearing, float elevation);
    
        Vector3<float> getRelativeLocationError(){return sphericalPosition;}
        //void setRelativeLocationError(Vector3<float> newSphericalError);

        //void setViewPosition(Vector2<int> newViewPosition);
	//void setViewPositionVariables(int x, int y);
        Vector2<int> getViewPosition(){return viewPosition;}
        //For COPY of whole object:
        void CopyObject(Object sourceObject);

        //Access vision variables:
        bool isObjectVisible(){return isVisible;}
        int FramesSeen(){return framesSeen;}
        int NumberOfTimesSeen(){return numberOfTimesSeen;}
        int FrameSinceLastSeen(){return framesSinceLastSeen;}
        float Distance(){return sphericalPosition[0];}
        float Bearing(){return sphericalPosition[1];}
        float Elevation(){return sphericalPosition[2];}
	int ScreenX(){return viewPosition[0];}
	int ScreenY(){return viewPosition[1];}

};

#endif // FIELDOBJECT_H
