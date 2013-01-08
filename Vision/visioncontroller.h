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


#include "Vision/visionblackboard.h"
#include "Vision/VisionWrapper/datawrappercurrent.h"
#include "Vision/Modules/segmentfilter.h"
#include "Vision/Modules/linedetector.h"
#include "Vision/Modules/goaldetector.h"

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
    int runFrame(bool lookForBall, bool lookForLandmarks);

//    /**
//    *   @brief Runs the vision system until a keypress is given.
//    *   @return A status indication of the execution.
//    */
//    int run();
    
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
    DataWrapper* m_data_wrapper;               //! @variable Reference to singleton Wrapper for vision system
    VisionBlackboard* m_blackboard;     //! @variable Reference to singleton Blackboard for vision system
    SegmentFilter m_segment_filter;       //! @variable Segment filter object for pre-classification filtering

    LineDetector* m_line_detector_sam;
    LineDetector* m_line_detector_ransac;
    GoalDetector* m_goal_detector_hist;
};

#endif // VISIONCONTROLLER_H
