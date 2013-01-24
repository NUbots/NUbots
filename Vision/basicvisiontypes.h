#ifndef BASICVISIONTYPES_H
#define BASICVISIONTYPES_H

#include "Tools/Math/Vector2.h"

typedef Vector2<double> Point;

namespace Vision {
    enum ScanDirection {
        VERTICAL,
        HORIZONTAL
    };

    enum COLOUR_CLASS {
        BALL_COLOUR,
        GOAL_COLOUR,
//        GOAL_Y_COLOUR,
//        GOAL_B_COLOUR,
        LINE_COLOUR,
        UNKNOWN_COLOUR
    };

    //! VFO_ID enum and associated string conversion methods
    enum VFO_ID {
        BALL        = 0,
        GOAL_L      = 1,
        GOAL_R      = 2,
        GOAL_U      = 3,
        FIELDLINE   = 4,
        OBSTACLE    = 5,
        INVALID     = 6
//        GOAL_Y_L=1,
//        GOAL_Y_R=2,
//        GOAL_Y_U=3,
//        GOAL_B_L=4,
//        GOAL_B_R=5,
//        GOAL_B_U=6,
//        BEACON_Y=7,
//        BEACON_B=8,
//        BEACON_U=9,
//        FIELDLINE=10,
//        OBSTACLE=11,
//        INVALID=12
    };

    //! @brief returns whether the given id maps to a goal
    inline bool isGoal(VFO_ID id) { return id >= GOAL_L && id <= GOAL_U;}
    //inline bool isBlueGoal(VFO_ID id) {return id >= GOAL_B_L && id <= GOAL_B_U;}
    //inline bool isYellowGoal(VFO_ID id) {return id >= GOAL_Y_L && id <= GOAL_Y_U;}
    //inline bool isBeacon(VFO_ID id) {return id >= BEACON_Y && id <= BEACON_U;}

    //! @brief converts a VisionFieldObject Id into a string.
    std::string getVFOName(VFO_ID id);

    //! @brief converts a string into a VisionFieldObject Id.
    VFO_ID getVFOFromName(const std::string &name);

    //! @brief converts an int into a VisionFieldObject Id.
    VFO_ID getVFOFromNum(int n);

    //! @brief converts a VisionFieldObject Id into an int.
    int getVFONum(VFO_ID id);

    //! @brief Less than operator for VisionFieldObject IDs.
    inline bool operator <(VFO_ID id0, VFO_ID id1)
    {
        return getVFONum(id0) < getVFONum(id1);
    }

    //! @brief converts a colour class into a string.
    std::string getColourClassName(COLOUR_CLASS id);

    //! @brief converts a string into a colour class.
    COLOUR_CLASS getColourClassFromName(const std::string& name);

}

#endif // BASICVISIONTYPES_H
