#ifndef DEBUGVERBOSITYVISION_H
#define DEBUGVERBOSITYVISION_H

//#include "Autoconfig/debugverbosityvision.h"

//! NOT ALL IMPLEMENTED YET

/*! 0 - off
*   1 - low     - typically only initialisation info
*   2 - medium  - minimal regularly changing info
*   3 - high    - full available debug info
*
*   Note: 2 and 3 make for increasing debug file size with runtime
*   Note: Do not implement level 1 to print more than once.
*/

#define DEBUG_VISION_VERBOSITY_ON
#ifdef DEBUG_VISION_VERBOSITY_ON
    #define VISION_CONTROLFLOW_VERBOSITY 0
    #define VISION_SCANLINE_VERBOSITY 0
    #define VISION_BLACKBOARD_VERBOSITY 0
    #define VISION_CONTROLLER_VERBOSITY 0
    #define VISION_HORIZON_VERBOSITY 0
    #define VISION_WRAPPER_VERBOSITY 1
    #define VISION_FILTER_VERBOSITY 1
    #define VISION_TRANSFORM_VERBOSITY 0
    #define VISION_OBSTACLE_VERBOSITY 0
    #define VISION_GOAL_VERBOSITY 3
    #define VISION_BALL_VERBOSITY 0
    #define VISION_FIELDPOINT_VERBOSITY 2
#else
    #define VISION_CONTROLFLOW_VERBOSITY 0
    #define VISION_SCANLINE_VERBOSITY 0
    #define VISION_BLACKBOARD_VERBOSITY 0
    #define VISION_CONTROLLER_VERBOSITY 0
    #define VISION_HORIZON_VERBOSITY 0
    #define VISION_WRAPPER_VERBOSITY 0
    #define VISION_FILTER_VERBOSITY 0
    #define VISION_TRANSFORM_VERBOSITY 0
    #define VISION_OBSTACLE_VERBOSITY 0
    #define VISION_GOAL_VERBOSITY 0
    #define VISION_BALL_VERBOSITY 0
    #define VISION_FIELDPOINT_VERBOSITY 0
#endif

#endif // DEBUGVERBOSITYVISION_H
