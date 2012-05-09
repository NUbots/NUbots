#include "visioncontroller.h"
#include "debug.h"
#include "debugverbosityvision.h"
#include <time.h>

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

int VisionController::runFrame()
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
    debug << "calculateHorizon done" << endl;
    HorizonInterpolate::interpolate(32);
    debug << "interpolate done" << endl;
    ObjectDetectionCH::detectObjects();
    debug << "detectObjects done" << endl;
    ScanLines::generateScanLines();
    debug << "generateScanLines done" << endl;
    ScanLines::classifyHorizontalScanLines();
    debug << "classifyHorizontalScanLines done" << endl;
    ScanLines::classifyVerticalScanLines();
    debug << "classifyVerticalScanLines done" << endl;
    m_segment_filter.run();
    debug << "segment filter done" << endl;
    
    #if VISION_CONTROLLER_VERBOSITY > 1
        debug << "VisionController::runFrame() - Publish Results" << endl;
    #endif
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
