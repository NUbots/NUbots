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

    //! @brief converts a VisionFieldObject Id into a string.
    static string getVFOName(VFO_ID id);
    //! @brief converts a string into a VisionFieldObject Id.
    static VFO_ID getVFOFromName(const string& name);
    
public:
    VisionFieldObject();

    //! @brief returns the screen location in pixels (relative to the top left).
    const Vector2<int>& getLocationPixels() const;
    //! @brief returns the angular screen location (relative to the image centre) in radians.
    const Vector2<float>& getLocationAngular() const;
    //! @brief returns the field position relative to the robot.
    virtual Vector3<float> getRelativeFieldCoords() const = 0;
    /*!
      @brief pushes the object to the external field objects.
      @param fieldobjects a pointer to the global list of field objects.
      @param timestamp the image timestamp.
      @return the success of the operation.
      */
    virtual bool addToExternalFieldObjects(FieldObjects* fieldobjects, float timestamp) const = 0;
    //! @brief applies a series of checks to decide if the object is valid.
    virtual bool check() const = 0;

protected:
    Vector2<int> m_location_pixels;         //! @variable The pixel location of the object on the screen.
    Vector2<float> m_location_angular;      //! @variable The angular location of the object relative to the screen centre.
    Vector3<float> m_spherical_position;    //! @variable The position (distance, bearing, elevation) of the object relative to the robots camera.
    float m_confidence;   //! unused
    float m_error;        //! unused
    Vector2<int> m_size_on_screen;          //! @variable The width and height on screen in pixels.
    Vector3<float> m_spherical_error;       //! @variable The error in each of the spherical dimensions.
    Vector3 <float> m_transformed_spherical_pos;    //! @variable The transformed location (relative to the centre of the feet) in cm.
    bool valid;                             //! @variable Whether the object is valid.
    bool distance_valid;                    //! @variable Whether the distance is valid.
};

#endif // VISIONFIELDOBJECT_H
