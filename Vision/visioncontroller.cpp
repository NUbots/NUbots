#include "visioncontroller.h"
#include <time.h>

VisionController* VisionController::instance = 0;

VisionController::VisionController()
{
    m_blackboard = VisionBlackboard::getInstance();
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
    m_blackboard->update();

    GreenHorizonCH::calculateHorizon();
    HorizonInterpolate::interpolate(32);
    ObjectDetectionCH::detectObjects();
    ScanLines::generateScanLines();
    ScanLines::classifyHorizontalScanLines();
    ScanLines::classifyVerticalScanLines();
    segment_filter.run();
    
    m_blackboard->publish();
    m_blackboard->debugPublish();

    return 0;
}

int VisionController::run()
{
    char c=0;

    while(c!=27) {
        DataWrapper::getInstance()->updateFrame();
        instance->runFrame();
        c = waitKey();
    }

    return 0;
}

CameraSettings VisionController::getCurrentCameraSettings() const
{
    return m_blackboard->getCameraSettings();
}
