/** @file visionfieldobject.h
*   @class VisionFieldObject
*   @author Shannon Fenn: shannon.fenn@uon.edu.au
*   @brief Abstract parent class for internal representation of field objects.
*/

#ifndef VISIONFIELDOBJECT_H
#define VISIONFIELDOBJECT_H

#include "Vision/basicvisiontypes.h"
#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include <vector>

using namespace std;

class VisionFieldObject
{
public:
    //! VFO_ID enum and associated string conversion methods
    enum VFO_ID {
        BALL,
        GOAL_Y,
        GOAL_B,
        LINE,
        CORNER,
        CENTRE_CIRCLE,
        OBSTACLE,
        UNKNOWN
    };

    static string getVFOName(VFO_ID id);
    static VFO_ID getVFOFromName(const string& name);
    
public:
    VisionFieldObject();
    
    const PointType& getLocationPixels() const;
    const vector<float>& getLocationAngular() const;
    virtual void getRelativeFieldCoords(vector<float>& coords) const = 0;
    virtual bool addToExternalFieldObjects(FieldObjects* fieldobjects) const = 0;
    

private:
    PointType m_location_pixels;        //! @variable The pixel location of the object on the screen.
    vector<float> m_location_angular;   //! @variable The angular location of the object relative to the screen centre.
    float confidence;   //! unused
    float error;        //! unused
};

#endif // VISIONFIELDOBJECT_H
