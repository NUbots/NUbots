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
#include "Tools/Math/Vector3.h"

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
    
    const Vector2<int>& getLocationPixels() const;
    const Vector2<float>& getLocationAngular() const;
    virtual Vector3<float> getRelativeFieldCoords() const = 0;
    virtual bool addToExternalFieldObjects(FieldObjects* fieldobjects, float timestamp) const = 0;
    
private:
    virtual void calculatePositions() = 0;

protected:
    Vector2<int> m_location_pixels;            //! @variable The pixel location of the object on the screen.
    Vector2<float> m_location_angular;       //! @variable The angular location of the object relative to the screen centre.
    Vector3<float> m_spherical_position;    //! @variable The position (distance, bearing, elevation) of the object relative to the robot
    float confidence;   //! unused
    float error;        //! unused
    Vector2<int> m_size_on_screen;
    Vector3<float> m_spherical_error;
    Vector3 <float> m_transformed_spherical_pos;
};

#endif // VISIONFIELDOBJECT_H
