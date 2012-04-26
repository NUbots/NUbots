#include "visioncontrolwrapperdarwin.h"

#include "Autoconfig/nubotdataconfig.h"
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
}

int VisionControlWrapper::runFrame()
{
    return controller->runFrame();
}

void VisionControlWrapper::process(JobList* jobs)
{
    #if DEBUG_VISION_VERBOSITY > 1
        debug  << "VisionControlWrapper::Process - Begin" << endl;
    #endif
    static list<Job*>::iterator it;     // the iterator over the motion jobs
    
    for (it = jobs->vision_begin(); it != jobs->vision_end();)
    {
        if ((*it)->getID() == Job::VISION_SAVE_IMAGES)
        {   
            #if DEBUG_VISION_VERBOSITY > 1
                debug << "VisionControlWrapper::process(): Processing a save images job." << endl;
            #endif
            static SaveImagesJob* job;
            job = (SaveImagesJob*) (*it);
            if(isSavingImages != job->saving())
            {
                if(job->saving() == true)
                {
                    currentSettings = controller->getCurrentCameraSettings();
                    if (!imagefile.is_open())
                        imagefile.open((string(DATA_DIR) + string("image.strm")).c_str());
                    if (!sensorfile.is_open())
                        sensorfile.open((string(DATA_DIR) + string("sensor.strm")).c_str());
                    m_actions->add(NUActionatorsData::Sound, m_sensor_data->CurrentTime, NUSounds::START_SAVING_IMAGES);
                }
                else
                {
                    imagefile.flush();
                    sensorfile.flush();

                    ChangeCameraSettingsJob* newJob  = new ChangeCameraSettingsJob(currentSettings);
                    jobs->addCameraJob(newJob);
                    m_actions->add(NUActionatorsData::Sound, m_sensor_data->CurrentTime, NUSounds::STOP_SAVING_IMAGES);
                }
            }
            isSavingImages = job->saving();
            isSavingImagesWithVaryingSettings = job->varyCameraSettings();
            it = jobs->removeVisionJob(it);
        }
        else 
        {
            ++it;
        }
    }
}
