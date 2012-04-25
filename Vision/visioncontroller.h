/**
*       @name   VisionController
*       @file   visioncontroller.h
*       @brief  Controller for vision blackboard.
*       @author Shannon Fenn
*       @author David Budden
*       @date   02/03/12
*
*/

#ifndef VISIONCONTROLLER_H
#define VISIONCONTROLLER_H

#include "visionblackboard.h"
#include "VisionWrapper/datawrappercurrent.h"
#include "VisionTools/lookuptable.h"
#include "Modules/greenhorizonch.h"
#include "Modules/horizoninterpolate.h"
#include "Modules/objectdetectionch.h"
#include "Modules/scanlines.h"
#include "Modules/segmentfilter.h"

class VisionController
{
public:
    /**
    *   @brief return unique instance of blackboard - lazy initialisation.
    */
    static VisionController* getInstance();

    /**
    *   @brief Runs the vision system for a single frame.
    *   @return A status indication of the execution of the frame.
    */
    int runFrame();

    /**
    *   @brief Runs the vision system until a keypress is given.
    *   @return A status indication of the execution.
    */
    int run();
private:
    //! @brief Private constructor for controller.
    VisionController();

    /** @brief Private destructor.
    *
    *   To be called implicitly at program close or by the controller as the controller is live for the
    *   whole program in absence of errors.
    */
    ~VisionController();

private:
//! SELF
    static VisionController* instance;           //! @variable Singleton instance

//! VARIABLES
    //VisionWrapper* wrapper;             //! @variable Reference to singleton Wrapper for vision system
    VisionBlackboard* blackboard;            //! @variable Reference to singleton Blackboard for vision system
    SegmentFilter segment_filter;       //! @variable Segment filter object for pre-classification filtering

};

#endif // VISIONCONTROLLER_H
