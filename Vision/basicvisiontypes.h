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

    enum DEBUG_ID {
        DBID_IMAGE              = 0,
        DBID_CLASSED_IMAGE      = 1,
        DBID_H_SCANS            = 2,
        DBID_V_SCANS            = 3,
        DBID_SEGMENTS           = 4,
        DBID_MATCHED_SEGMENTS   = 5,
        DBID_HORIZON            = 6,
        DBID_GREENHORIZON_SCANS = 7,
        DBID_GREENHORIZON_FINAL = 8,
        DBID_OBJECT_POINTS      = 9,
        DBID_FILTERED_SEGMENTS  = 10,
        DBID_GOALS              = 11,
        DBID_BALLS              = 12,
        DBID_OBSTACLES          = 13,
        DBID_LINES              = 14,
        DBID_GOAL_LINES_START   = 15,
        DBID_GOAL_LINES_END     = 16,
        DBID_INVALID           = 17
//        DBID_BEACONS            = 12,
//        DBID_BALLS              = 13,
//        DBID_OBSTACLES          = 14,
//        DBID_LINES              = 15,
//        DBID_GOAL_LINES_START   = 16,
//        DBID_GOAL_LINES_END     = 17,
//        NUMBER_OF_IDS           = 18
    };

    inline std::string getDebugIDName(DEBUG_ID id) {
        switch(id) {
        case DBID_IMAGE:
            return "DBID_IMAGE";
        case DBID_H_SCANS:
            return "DBID_H_SCANS";
        case DBID_V_SCANS:
            return "DBID_V_SCANS";
        case DBID_SEGMENTS:
            return "DBID_SEGMENTS";
        case DBID_MATCHED_SEGMENTS:
            return "DBID_MATCHED_SEGMENTS";
        case DBID_HORIZON:
            return "DBID_HORIZON";
        case DBID_GREENHORIZON_SCANS:
            return "DBID_GREENHORIZON_SCANS";
        case DBID_GREENHORIZON_FINAL:
            return "DBID_GREENHORIZON_FINAL";
        case DBID_OBJECT_POINTS:
            return "DBID_OBJECT_POINTS";
        case DBID_FILTERED_SEGMENTS:
            return "DBID_FILTERED_SEGMENTS";
        case DBID_GOALS:
            return "DBID_GOALS";
        //case DBID_BEACONS:
        //    return "DBID_BEACONS";
        case DBID_BALLS:
            return "DBID_BALLS";
        case DBID_OBSTACLES:
            return "DBID_OBSTACLES";
        case DBID_GOAL_LINES_START:
            return "DBID_GOAL_LINES_START";
        case DBID_GOAL_LINES_END:
            return "DBID_GOAL_LINES_END";
        default:
            return "NOT VALID";
        }
    }

    inline DEBUG_ID getDebugIDFromInt(int id) {
        switch(id) {
        case 0: return DBID_IMAGE;
        case 1: return DBID_H_SCANS;
        case 2: return DBID_V_SCANS;
        case 3: return DBID_SEGMENTS;
        case 4: return DBID_MATCHED_SEGMENTS;
        case 5: return DBID_HORIZON;
        case 6: return DBID_GREENHORIZON_SCANS;
        case 7: return DBID_GREENHORIZON_FINAL;
        case 8: return DBID_OBJECT_POINTS;
        case 9: return DBID_FILTERED_SEGMENTS;
        case 10: return DBID_GOALS;
        case 11: return DBID_BALLS;
        case 12: return DBID_OBSTACLES;
        case 13: return DBID_GOAL_LINES_START;
        case 14: return DBID_GOAL_LINES_END;
        default: return DBID_INVALID;
        }
    }

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
