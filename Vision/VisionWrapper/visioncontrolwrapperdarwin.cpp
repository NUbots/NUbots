#include "visioncontrolwrapperdarwin.h"

#include "nubotdataconfig.h"
#include "debug.h"
#include "debugverbosityvision.h"
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
    #if VISION_WRAPPER_VERBOSITY > 1
        debug << "VisionControlWrapper::runFrame() - Begin" << endl;
    #endif
    //force data wrapper to update
    if(!data_wrapper->updateFrame()) {
        #if VISION_WRAPPER_VERBOSITY > 1
            debug << "VisionControlWrapper::runFrame() - updateFrame() failed" << endl;
        #endif
        return -1;  //failure - do not run vision
    }
    #if VISION_WRAPPER_VERBOSITY > 1
        debug << "VisionControlWrapper::runFrame() - updateFrame() succeeded" << endl;
    #endif
        
    int result = controller->runFrame(Blackboard->lookForBall, Blackboard->lookForLandmarks); //run vision on the frame
    
    data_wrapper->postProcess();    //post process all the field objects
    
    #if VISION_WRAPPER_VERBOSITY > 1
        debug << "VisionControlWrapper::runFrame() - Finish" << endl;
    #endif
    return result;
}

void VisionControlWrapper::process(JobList* jobs)
{
    #if VISION_WRAPPER_VERBOSITY > 1
        debug << "VisionControlWrapper::process():" << endl;
    #endif
    data_wrapper->process(jobs);
}

void VisionControlWrapper::saveAnImage() const
{
    #if VISION_WRAPPER_VERBOSITY > 1
        debug << "VisionControlWrapper::saveAnImage():" << endl;
    #endif
    data_wrapper->saveAnImage();
}
