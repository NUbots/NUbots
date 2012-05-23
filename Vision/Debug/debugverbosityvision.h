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

//#define DEBUG_VISION_VERBOSITY_ON
#ifdef DEBUG_VISION_VERBOSITY_ON
    #define VISION_SCAN_VERBOSITY       2
    #define VISION_BLACKBOARD_VERBOSITY 2
    #define VISION_CONTROLLER_VERBOSITY 2
    #define VISION_HORIZON_VERBOSITY    2
    #define VISION_WRAPPER_VERBOSITY    2
    #define VISION_LUT_VERBOSITY        2
    #define VISION_FILTER_VERBOSITY     2
#else
    #define VISION_SCAN_VERBOSITY       0
    #define VISION_BLACKBOARD_VERBOSITY 0
    #define VISION_CONTROLLER_VERBOSITY 0
    #define VISION_HORIZON_VERBOSITY    0
    #define VISION_WRAPPER_VERBOSITY    0
    #define VISION_LUT_VERBOSITY        0
    #define VISION_FILTER_VERBOSITY     0
#endif

#endif // DEBUGVERBOSITYVISION_H
