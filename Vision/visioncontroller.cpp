#include "visioncontroller.h"
#include "debug.h"
#include "debugverbosityvision.h"
#include <time.h>

//#include "Infrastructure/Jobs/JobList.h"

#include "Vision/VisionTools/lookuptable.h"
#include "Vision/Modules/greenhorizonch.h"
#include "Vision/Modules/objectdetectionch.h"
#include "Vision/Modules/scanlines.h"
#include "Vision/Modules/goaldetection.h"
#include "Vision/Modules/balldetection.h"
//robocup hacks
#include "Vision/Modules/robocuphacks.h"


VisionController* VisionController::instance = 0;

VisionController::VisionController()
{
    m_blackboard = VisionBlackboard::getInstance();
    m_data_wrapper = DataWrapper::getInstance();
}

VisionController::~VisionController()
{
}

VisionController* VisionController::getInstance()
{
    if(instance == 0)
        instance = new VisionController();
    return instance;
}

int VisionController::runFrame(bool lookForBall, bool lookForLandmarks)
{
    #if VISION_CONTROLLER_VERBOSITY > 1
        debug << "VisionController::runFrame() - Begin" << endl;
        debug << "VisionController::runFrame() - Update VisionBlackboard" << endl;
    #endif
    //force blackboard to update from wrapper
    m_blackboard->update();

    #if VISION_CONTROLLER_VERBOSITY > 1
        debug << "VisionController::runFrame() - Run Modules" << endl;
    #endif
    //run modules    
    GreenHorizonCH::calculateHorizon();
#if VISION_CONTROLLER_VERBOSITY > 2
    debug << "VisionController::runFrame() - calculateHorizon done" << endl;
#endif
    //HorizonInterpolate::interpolate(ScanLines::HORIZONTAL_SCANLINES);

    ObjectDetectionCH::detectObjects();
#if VISION_CONTROLLER_VERBOSITY > 2
    debug << "VisionController::runFrame() - detectObjects done" << endl;
#endif
    ScanLines::generateScanLines();
#if VISION_CONTROLLER_VERBOSITY > 2
    debug << "VisionController::runFrame() - generateScanLines done" << endl;
#endif
    ScanLines::classifyHorizontalScanLines();
#if VISION_CONTROLLER_VERBOSITY > 2
    debug << "VisionController::runFrame() - classifyHorizontalScanLines done" << endl;
#endif
    ScanLines::classifyVerticalScanLines();
#if VISION_CONTROLLER_VERBOSITY > 2
    debug << "VisionController::runFrame() - classifyVerticalScanLines done" << endl;
#endif
    m_segment_filter.run();
#if VISION_CONTROLLER_VERBOSITY > 2
    debug << "VisionController::runFrame() - segment filter done" << endl;
#endif
    if(lookForLandmarks)
        GoalDetection::detectGoals();
#if VISION_CONTROLLER_VERBOSITY > 2
    if(lookForLandmarks)
        debug << "VisionController::runFrame() - goal detection done - looking for landmarks" << endl;
    else
        debug << "VisionController::runFrame() - goal detection done - not looking for landmarks" << endl;
#endif
    if(lookForBall)
        BallDetection::detectBall();
#if VISION_CONTROLLER_VERBOSITY > 2
    if(lookForBall)
        debug << "VisionController::runFrame() - ball detection done - looking for ball" << endl;
    else
        debug << "VisionController::runFrame() - ball detection done - not looking for ball" << endl;
#endif
    
    #if VISION_CONTROLLER_VERBOSITY > 1
        debug << "VisionController::runFrame() - Publish Results" << endl;
    #endif

    //RobocupHacks::beaconGoalHack();
    //RobocupHacks::ballGoalHack();

    //force blackboard to publish results through wrapper
    m_blackboard->publish();
    //publish debug information as well
    m_blackboard->debugPublish();
    
    #if VISION_CONTROLLER_VERBOSITY > 1
        debug << "VisionController::runFrame() - Finish" << endl;
    #endif
    return 0;
}

//int VisionController::run()
//{
//    char c=0;

//    while(c!=27) {
//        DataWrapper::getInstance()->updateFrame();
//        instance->runFrame();
//        c = waitKey();
//    }

//    return 0;
//}

CameraSettings VisionController::getCurrentCameraSettings() const
{
    return m_blackboard->getCameraSettings();
}
