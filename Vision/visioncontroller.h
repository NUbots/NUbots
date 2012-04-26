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

#include "Infrastructure/Jobs/JobList.h"

#include "Vision/visionblackboard.h"
#include "Vision/VisionWrapper/datawrappercurrent.h"
#include "Vision/VisionTools/lookuptable.h"
#include "Vision/Modules/greenhorizonch.h"
#include "Vision/Modules/horizoninterpolate.h"
#include "Vision/Modules/objectdetectionch.h"
#include "Vision/Modules/scanlines.h"
#include "Vision/Modules/segmentfilter.h"

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
    
    /**
    *   @brief Gets the current camera settings from the blackboard and returns them
    *   @return The current camera settings
    */
    CameraSettings getCurrentCameraSettings() const;
    
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
    VisionBlackboard* m_blackboard;            //! @variable Reference to singleton Blackboard for vision system
    SegmentFilter segment_filter;       //! @variable Segment filter object for pre-classification filtering
    
    

};

#endif // VISIONCONTROLLER_H
