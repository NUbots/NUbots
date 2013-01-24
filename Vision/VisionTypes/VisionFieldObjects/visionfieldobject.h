/** @file visionfieldobject.h
*   @class VisionFieldObject
*   @author Shannon Fenn: shannon.fenn@uon.edu.au
*   @brief Abstract parent class for internal representation of field objects.
*/

#ifndef VISIONFIELDOBJECT_H
#define VISIONFIELDOBJECT_H

#include "Vision/basicvisiontypes.h"

#include "Vision/VisionTypes/Interfaces/publishable.h"
#include "Vision/VisionTypes/Interfaces/optimisable.h"
#include "Vision/VisionTypes/Interfaces/renderable.h"

#include "Infrastructure/NUBlackboard.h"
#include "Tools/Math/Vector3.h"
#include "Tools/Math/Vector2.h"

#include <vector>

using namespace std;
using namespace Vision;

typedef Vector2<double> Point;

class VisionFieldObject : public Publishable, public Optimisable, public Renderable
{
public:
    VisionFieldObject();
    virtual ~VisionFieldObject() {}

    VFO_ID getID() const {return m_id;}
    string getName() const {return getVFOName(m_id);}
    
    //! @brief returns the screen location in pixels (relative to the top left).
    const Point& getLocationPixels() const;
    //! @brief returns the angular screen location (relative to the image centre) in radians.
    const Vector2<float>& getLocationAngular() const;
    //! @brief returns the screen size in pixels.
    const Vector2<double>& getScreenSize() const { return m_size_on_screen; }
    //! @brief returns the field position relative to the robot.
    virtual Vector3<float> getRelativeFieldCoords() const = 0;

protected:
    Point m_location_pixels;         //! @variable The pixel location of the object on the screen.
    Vector2<double> m_size_on_screen;          //! @variable The width and height on screen in pixels.

    VFO_ID m_id;
    Vector2<float> m_location_angular;      //! @variable The angular location of the object relative to the screen centre.
    Vector3<float> m_spherical_position;    //! @variable The position (distance, bearing, elevation) of the object relative to the robots camera.
    float m_confidence;   //! unused
    float m_error;        //! unused
    Vector3<float> m_spherical_error;       //! @variable The error in each of the spherical dimensions.
    Vector3 <float> m_transformed_spherical_pos;    //! @variable The transformed location (relative to the centre of the feet) in cm.
    bool valid;                             //! @variable Whether the object is valid.
    bool distance_valid;                    //! @variable Whether the distance is valid.
};

#endif // VISIONFIELDOBJECT_H
