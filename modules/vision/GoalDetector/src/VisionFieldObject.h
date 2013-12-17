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

        class VFOMap : public std::map<VFO_ID, std::pair<int, std::string> > {
        public:
            VFOMap() {
                (*this)[BALL]           = std::pair<int, std::string>(0, "BALL");
                (*this)[FIELDLINE]      = std::pair<int, std::string>(1, "FIELDLINE");
                (*this)[CORNER]         = std::pair<int, std::string>(2, "CORNER");
                (*this)[CENTRE_CIRCLE]  = std::pair<int, std::string>(3, "CENTRE_CIRCLE");
                (*this)[OBSTACLE]       = std::pair<int, std::string>(4, "OBSTACLE");
                (*this)[GOAL_L]         = std::pair<int, std::string>(5, "GOAL_L");
                (*this)[GOAL_R]         = std::pair<int, std::string>(6, "GOAL_R");
                (*this)[GOAL_U]         = std::pair<int, std::string>(7, "GOAL_U");
                (*this)[GOAL_Y_L]       = std::pair<int, std::string>(5, "GOAL_Y_L");
                (*this)[GOAL_Y_R]       = std::pair<int, std::string>(6, "GOAL_Y_R");
                (*this)[GOAL_Y_U]       = std::pair<int, std::string>(7, "GOAL_Y_U");
                (*this)[GOAL_B_L]       = std::pair<int, std::string>(5, "GOAL_B_L");
                (*this)[GOAL_B_R]       = std::pair<int, std::string>(6, "GOAL_B_R");
                (*this)[GOAL_B_U]       = std::pair<int, std::string>(7, "GOAL_B_U");
            }
        };

        VFOMap vfomap;

     enum DistanceMethod {
        Width,
        D2P,
        Average,
        Least
    };

    //! VFO_ID enum and associated std::string conversion methods
    enum VFO_ID {
        BALL            = 0,
        FIELDLINE       = 1,
        CORNER          = 2,
        CENTRE_CIRCLE   = 3,
        OBSTACLE        = 4,
        GOAL_L          = 5,
        GOAL_R          = 6,
        GOAL_U          = 7,
        GOAL_Y_L        = 8,
        GOAL_Y_R        = 9,
        GOAL_Y_U        = 10,
        GOAL_B_L        = 11,
        GOAL_B_R        = 12,
        GOAL_B_U        = 13,
        INVALID         = 14
    };


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

    //! @brief returns whether the given id maps to a goal
    bool isGoal(VFO_ID id) const;

    //! @brief converts a VisionFieldObject Id into a string.
    std::string VFOName(VFO_ID id);

    //! @brief converts a string into a VisionFieldObject Id.
    VFO_ID VFOFromName(const std::string &name);

    //! @brief converts an int into a VisionFieldObject Id.
    VFO_ID VFOFromInt(int n);

    //! @brief converts a VisionFieldObject Id into an int.
    int intFromVFO(VFO_ID id);

    int numVFOIDs();

    DistanceMethod getDistanceMethodFromName(std::string name);
    std::string getDistanceMethodName(DistanceMethod method);
    std::string getLineMethodName(LineDetectionMethod method);
    std::string getGoalMethodName(GoalDetectionMethod method);    

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
