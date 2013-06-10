#include "visioncontrolwrapperdarwin.h"

#include "nubotdataconfig.h"
#include "debug.h"
#include "debugverbosityvision.h"
#include "Infrastructure/Jobs/VisionJobs/SaveImagesJob.h"
#include "Vision/Threads/SaveImagesThread.h"

VisionControlWrapper* VisionControlWrapper::instance = 0;

VisionControlWrapper* VisionControlWrapper::getInstance()
{
    if(!instance)
        instance = new VisionControlWrapper();
    return instance;
}

VisionControlWrapper::VisionControlWrapper()
{
    data_wrapper = DataWrapper::getInstance();
    m_saveimages_thread = new SaveImagesThread(this);
}

int VisionControlWrapper::runFrame()
{
    static unsigned int frame = 0;
    #if VISION_WRAPPER_VERBOSITY > 1
        debug << "VisionControlWrapper::runFrame(): - frame " << frame << endl;
    #endif
    frame = (frame + 1) % 10000;
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

    if(data_wrapper->isSavingImages)
    {
        #if DEBUG_VISION_VERBOSITY > 1
            debug << "Vision::starting the save images loop." << endl;
        #endif
        m_saveimages_thread->signal();
    }

    int result = controller.runFrame(Blackboard->lookForBall, Blackboard->lookForGoals, Blackboard->lookForFieldPoints, Blackboard->lookForObstacles); //run vision on the frame
    
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
