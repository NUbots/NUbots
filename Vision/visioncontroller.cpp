#include "visioncontroller.h"
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
    //force wrapper to update
    m_data_wrapper->updateFrame();
    //force blackboard to update from wrapper
    m_blackboard->update();

    //run modules
    GreenHorizonCH::calculateHorizon();
    HorizonInterpolate::interpolate(32);
    ObjectDetectionCH::detectObjects();
    ScanLines::generateScanLines();
    ScanLines::classifyHorizontalScanLines();
    ScanLines::classifyVerticalScanLines();
    m_segment_filter.run();
    
    //force blackboard to publish results through wrapper
    m_blackboard->publish();
    //publish debug information as well
    m_blackboard->debugPublish();

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
