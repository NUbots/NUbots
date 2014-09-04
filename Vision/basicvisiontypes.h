#ifndef BASICVISIONTYPES_H
#define BASICVISIONTYPES_H

#include "../Tools/Math/Vector2.h"

//#include "Vision/VisionTypes/nupoint.h"
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
        TEAM_CYAN_COLOUR,
        TEAM_MAGENTA_COLOUR,
        UNKNOWN_COLOUR
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

    enum DEBUG_ID {
        DBID_IMAGE                  = 0,
        DBID_CLASSED_IMAGE          = 1,
        DBID_H_SCANS                = 2,
        DBID_V_SCANS                = 3,
        DBID_SEGMENTS               = 4,
        DBID_FILTERED_SEGMENTS      = 5,
        DBID_MATCHED_SEGMENTS       = 6,
        DBID_HORIZON                = 7,
        DBID_GREENHORIZON_SCANS     = 8,
        DBID_GREENHORIZON_THROWN    = 9,
        DBID_GREENHORIZON_FINAL     = 10,
        DBID_OBSTACLE_POINTS          = 11,
        DBID_GOALS                  = 12,
        DBID_BALLS                  = 13,
        DBID_OBSTACLES              = 14,
        DBID_LINES                  = 15,
        DBID_CENTRE_CIRCLES         = 16,
        DBID_CORNERS                = 17,
        DBID_GOAL_LINES_START       = 18,
        DBID_GOAL_LINES_END         = 19,
        DBID_GOALS_HIST             = 21,
        DBID_GOALS_RANSAC_EDGES     = 22
//        DBID_BEACONS            = 12,
//        DBID_BALLS              = 13,
//        DBID_OBSTACLES          = 14,
//        DBID_LINES              = 15,
//        DBID_GOAL_LINES_START   = 16,
//        DBID_GOAL_LINES_END     = 17,
//        NUMBER_OF_IDS           = 18
    };

    enum DEBUG_PLOT_ID {
        POINTS_PLOT,
        LINES_PLOT
    };

    enum DistanceMethod {
        Width,
        D2P,
        Average,
        Least
    };

    enum GoalDetectionMethod {
        HIST,
        RANSAC_G
    };

    enum LineDetectionMethod {
        SAM,
        RANSAC
    };

    std::string debugIDName(DEBUG_ID id);

    DEBUG_ID debugIDFromInt(int id);

    int intFromeDebugID(DEBUG_ID id);

    int numDebugIDs();

    //! @brief returns whether the given id maps to a goal
    inline bool isGoal(VFO_ID id) { return id >= GOAL_L && id <= GOAL_U;}
    //inline bool isBlueGoal(VFO_ID id) {return id >= GOAL_B_L && id <= GOAL_B_U;}
    //inline bool isYellowGoal(VFO_ID id) {return id >= GOAL_Y_L && id <= GOAL_Y_U;}
    //inline bool isBeacon(VFO_ID id) {return id >= BEACON_Y && id <= BEACON_U;}

    //! @brief converts a VisionFieldObject Id into a string.
    std::string VFOName(VFO_ID id);

    //! @brief converts a string into a VisionFieldObject Id.
    VFO_ID VFOFromName(const std::string &name);

    //! @brief converts an int into a VisionFieldObject Id.
    VFO_ID VFOFromInt(int n);

    //! @brief converts a VisionFieldObject Id into an int.
    int intFromVFO(VFO_ID id);

    int numVFOIDs();

    //! @brief converts a colour class into a string.
    std::string getColourClassName(COLOUR_CLASS id);

    //! @brief converts a string into a colour class.
    COLOUR_CLASS getColourClassFromName(const std::string& name);

    DistanceMethod getDistanceMethodFromName(std::string name);

    std::string getDistanceMethodName(DistanceMethod method);

    LineDetectionMethod getLineMethodFromName(std::string name);

    std::string getLineMethodName(LineDetectionMethod method);

    GoalDetectionMethod getGoalMethodFromName(std::string name);

    std::string getGoalMethodName(GoalDetectionMethod method);
}

#endif // BASICVISIONTYPES_H
