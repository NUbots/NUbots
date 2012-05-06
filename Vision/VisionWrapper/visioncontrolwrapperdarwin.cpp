#include "visioncontrolwrapperdarwin.h"

#include "nubotdataconfig.h"
#include "Infrastructure/Jobs/VisionJobs/SaveImagesJob.h"

VisionControlWrapper* VisionControlWrapper::instance = 0;

VisionControlWrapper* VisionControlWrapper::getInstance()
{
    if(!instance)
        instance = new VisionControlWrapper();
    return instance;
}

VisionControlWrapper::VisionControlWrapper()
{
    controller = VisionController::getInstance();
    data_wrapper = DataWrapper::getInstance();
}

int VisionControlWrapper::runFrame()
{
    return controller->runFrame();
}

void VisionControlWrapper::process(JobList* jobs)
{
    data_wrapper->process(jobs);
}

void VisionControlWrapper::saveAnImage() const
{
    data_wrapper->saveAnImage();
}
