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
#include "Tools/Math/Vector2.h"

#include <vector>

using namespace std;

class VisionFieldObject
{
public:
    //! VFO_ID enum and associated string conversion methods
    enum VFO_ID {
        BALL = 0,
        GOAL_Y_L=1,
        GOAL_Y_R=2,
        GOAL_Y_U=3,
        GOAL_B_L=4,
        GOAL_B_R=5,
        GOAL_B_U=6,
        BEACON_Y=7,
        BEACON_B=8,
        BEACON_U=9,
        FIELDLINE=10,
        OBSTACLE=11,
        INVALID=12
    };
    
    enum COLOUR_CLASS {
        BALL_COLOUR,
        GOAL_Y_COLOUR,
        GOAL_B_COLOUR,
        LINE_COLOUR,
        UNKNOWN_COLOUR
    };

    //! @brief converts a VisionFieldObject Id into a string.
    static string getVFOName(VFO_ID id);
    //! @brief converts a string into a VisionFieldObject Id.
    static VFO_ID getVFOFromName(const string& name);
    //! @brief returns the first VisionFieldObject Id.
    static VFO_ID getVFOFromNum(int n);
    
    //! @brief converts a colour class into a string.
    static string getColourClassName(COLOUR_CLASS id);
    //! @brief converts a string into a colour class.
    static COLOUR_CLASS getColourClassFromName(const string& name);
    
    static bool isGoal(VFO_ID id) { return id >= VisionFieldObject::GOAL_Y_L && id <= VisionFieldObject::GOAL_B_U;}
    static bool isBlueGoal(VFO_ID id) {return id >= VisionFieldObject::GOAL_B_L && id <= VisionFieldObject::GOAL_B_U;}
    static bool isYellowGoal(VFO_ID id) {return id >= VisionFieldObject::GOAL_Y_L && id <= VisionFieldObject::GOAL_Y_U;}
    static bool isBeacon(VFO_ID id) {return id >= VisionFieldObject::BEACON_Y && id <= VisionFieldObject::BEACON_U;}
    
public:
    VisionFieldObject();

    VFO_ID getID() const {return m_id;}
    string getName() const {return getVFOName(m_id);}
    
    //! @brief returns the screen location in pixels (relative to the top left).
    const Vector2<int>& getLocationPixels() const;
    //! @brief returns the angular screen location (relative to the image centre) in radians.
    const Vector2<float>& getLocationAngular() const;
    //! @brief returns the screen size in pixels.
    const Vector2<int>& getScreenSize() const { return m_size_on_screen; }
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
    
    //! @brief Stream output for labelling purposes
    virtual void printLabel(ostream& out) const = 0;
    
    //! @brief Calculation of error for optimisation
    virtual double findError(const Vector2<double>& measured) const {return sqrt( pow(m_location_pixels.x - measured.x,2) + pow(m_location_pixels.y - measured.y,2));}
    
    virtual void render(cv::Mat& mat) const = 0;
    
public:
    Vector2<int> m_location_pixels;         //! @variable The pixel location of the object on the screen.
    Vector2<int> m_size_on_screen;          //! @variable The width and height on screen in pixels.

protected:
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
