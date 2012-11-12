#include "visioncontroller.h"
#include "debug.h"
#include "debugverbosityvision.h"
#include <time.h>

//#include "Infrastructure/Jobs/JobList.h"

#include "Tools/Profiling/Profiler.h"

#include "Vision/VisionTools/lookuptable.h"
#include "Vision/Modules/greenhorizonch.h"
#include "Vision/Modules/objectdetectionch.h"
#include "Vision/Modules/scanlines.h"
#include "Vision/Modules/goaldetection.h"
#include "Vision/Modules/balldetection.h"
//robocup hacks
#include "Vision/visionconstants.h"
#include "Vision/Modules/LineDetectionAlgorithms/splitandmerge.h"
#include "Vision/Modules/LineDetectionAlgorithms/ransac.h"


VisionController* VisionController::instance = 0;

VisionController::VisionController()
{
    m_blackboard = VisionBlackboard::getInstance();
    m_data_wrapper = DataWrapper::getInstance();
    switch(VisionConstants::LINE_METHOD) {
    case VisionConstants::RANSAC:
        m_line_detector = new RANSAC();
        break;
    case VisionConstants::SAM:
        m_line_detector = new SplitAndMerge();
        break;
    }
}

VisionController::~VisionController()
{
    delete m_line_detector;
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
    #endif
    //force blackboard to update from wrapper
    m_blackboard->update();
    #if VISION_CONTROLLER_VERBOSITY > 1
        debug << "VisionController::runFrame() - VisionBlackboard updated" << endl;
    #endif

    //! HORIZON

    GreenHorizonCH::calculateHorizon();
    #if VISION_CONTROLLER_VERBOSITY > 2
        debug << "VisionController::runFrame() - calculateHorizon done" << endl;
    #endif

    //! PRE-DETECTION PROCESSING

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

    //! DETECTION MODULES

    if(lookForLandmarks) {
        GoalDetection::detectGoals();
        #if VISION_CONTROLLER_VERBOSITY > 2
            debug << "VisionController::runFrame() - goal detection done" << endl;
        #endif

        //PROFILING
        #ifdef VISION_PROFILER_ON
        static Profiler prof("lines");
        prof.start();
        #endif
        m_line_detector->run();
        #ifdef VISION_PROFILER_ON
        prof.stop();
        debug << prof << endl;
        #endif
        #if VISION_CONTROLLER_VERBOSITY > 2
            debug << "VisionController::runFrame() - line detection done" << endl;
        #endif
    }
    else {
        #if VISION_CONTROLLER_VERBOSITY > 2
            debug << "VisionController::runFrame() - not looking for landmarks" << endl;
        #endif
    }

    if(lookForBall) {
        BallDetection::detectBall();
        #if VISION_CONTROLLER_VERBOSITY > 2
            debug << "VisionController::runFrame() - ball detection done" << endl;
        #endif
    }
    else {
        #if VISION_CONTROLLER_VERBOSITY > 2
            debug << "VisionController::runFrame() - not looking for ball" << endl;
        #endif
    }

    ObjectDetectionCH::detectObjects();
    #if VISION_CONTROLLER_VERBOSITY > 2
        debug << "VisionController::runFrame() - detectObjects done" << endl;
    #endif

    //RobocupHacks::beaconGoalHack();
    //RobocupHacks::ballGoalHack();

    //force blackboard to publish results through wrapper
    m_blackboard->publish();
    #if VISION_CONTROLLER_VERBOSITY > 1
        debug << "VisionController::runFrame() - Results published" << endl;
    #endif
    //publish debug information as well
    m_blackboard->debugPublish();
    #if VISION_CONTROLLER_VERBOSITY > 1
        debug << "VisionController::runFrame() - Debugging info published" << endl;
    #endif
    
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
