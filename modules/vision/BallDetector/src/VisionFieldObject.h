/** @file visionfieldobject.h
 *   @class VisionFieldObject
 *   @author Shannon Fenn: shannon.fenn@uon.edu.au
 *   @brief Abstract parent class for internal representation of field objects.
 */

#ifndef VISIONFIELDOBJECT_H
#define VISIONFIELDOBJECT_H

#include <nuclear>
#include <armadillo>
#include <vector>
#include <string>

class VisionFieldObject /*: public Publishable, public Printable*/ {
public:
	VisionFieldObject();
    virtual ~VisionFieldObject();

    VFO_ID getID() const;

    std::string getName() const;

    bool isValid() const;

    const NUPoint& getLocation() const;

    //! @brief returns the screen location in pixels (relative to the top left).
    arma::vec2 getLocationPixels() const;

    //! @brief returns the angular screen location (relative to the camera) in radians.
    arma::vec2 getLocationAngular() const;

    //! @brief returns the screen size in pixels.
    arma::vec2 getScreenSize() const;

    //! @brief returns the field position relative to the robot.
    virtual arma::vec3 getRelativeFieldCoords() const;

	virtual double findScreenError(VisionFieldObject* other) const = 0;
	virtual double findGroundError(VisionFieldObject* other) const = 0;

protected:
    NUPoint m_location;                         //! @variable The location of the object (includes screen, radial and ground position).
    arma::vec2 m_sizeOnScreen;                  //! @variable The width and height on screen in pixels.

    VFO_ID m_id;
    float m_confidence;                         //! unused
    float m_error;                              //! unused
    arma::vec3 m_sphericalError;                //! @variable The error in each of the spherical dimensions.
    bool valid;                                 //! @variable Whether the object is valid.
    //bool distance_valid;                      //! @variable Whether the distance is valid.
};

#endif // VISIONFIELDOBJECT_H
