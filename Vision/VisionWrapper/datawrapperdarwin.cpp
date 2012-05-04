#include "datawrapperdarwin.h"

#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/Jobs/CameraJobs/ChangeCameraSettingsJob.h"

DataWrapper* DataWrapper::instance = 0;

string DataWrapper::getIDName(DATA_ID id) {
    switch(id) {
    case DID_IMAGE:
        return "DID_IMAGE";
    case DID_CLASSED_IMAGE:
        return "DID_CLASSED_IMAGE";
    default:
        return "NOT VALID";
    }
}

string DataWrapper::getIDName(DEBUG_ID id) {
    switch(id) {
    case DBID_IMAGE:
        return "DBID_IMAGE";
    case DBID_H_SCANS:
        return "DBID_H_SCANS";
    case DBID_V_SCANS:
        return "DBID_V_SCANS";
    case DBID_SEGMENTS:
        return "DBID_SEGMENTS";
    case DBID_TRANSITIONS:
        return "DBID_TRANSITIONS";
    case DBID_HORIZON:
        return "DBID_HORIZON";
    case DBID_GREENHORIZON_SCANS:
        return "DBID_GREENHORIZON_SCANS";
    case DBID_GREENHORIZON_FINAL:
        return "DBID_GREENHORIZON_FINAL";
    case DBID_OBJECT_POINTS:
        return "DBID_OBJECT_POINTS";
    case DBID_FILTERED_SEGMENTS:
        return "DBID_FILTERED_SEGMENTS";
    default:
        return "NOT VALID";
    }
}

DataWrapper::DataWrapper()
{
    numFramesDropped = 0;
    numFramesProcessed = 0;
    numSavedImages = 0;
}

DataWrapper::~DataWrapper()
{
}

DataWrapper* DataWrapper::getInstance()
{
    if (instance == 0)
        instance = new DataWrapper();
    return instance;
}

/**
*   @brief Fetches the next frame from the webcam.
*/
NUImage* DataWrapper::getFrame()
{
    return Blackboard->Image;
}

/*! @brief Retrieves the horizon data and builds a Horizon and returns it.
*   @return m_kinematics_horizon A reference to the kinematics horizon line.
*   @note This method has a chance of retrieving an invalid line, in this case
*           the old line is returned with the "exists" flag set to false.
*/
const Horizon& DataWrapper::getKinematicsHorizon()
{
    if(m_sensor_data->getHorizon(m_horizon_coefficients)) {
        m_kinematics_horizon.setLine(m_horizon_coefficients.at(0), m_horizon_coefficients.at(1), m_horizon_coefficients.at(2));
        m_kinematics_horizon.exists = true;
    }
    else {
        m_kinematics_horizon.exists = false;
    }
    
    return m_kinematics_horizon;
}

/*! @brief Retrieves the camera transform vectord returns it.
*   @param ctgvector A reference to a float vector to fill.
*   @return valid Whether the retrieved values are valid or not.
*/
bool DataWrapper::getCTGVector(vector<float>& ctgvector)
{
    return m_sensor_data->get(NUSensorsData::CameraToGroundTransform, ctgvector);
}

const LookUpTable& DataWrapper::getLUT() const
{
    //! @todo Implement + Comment
    return m_LUT;
}
    
//! PUBLISH METHODS
void DataWrapper::publish(DATA_ID id, const Mat& img)
{
    //! @todo Implement + Comment
}

//void DataWrapper::publish(DATA_ID id, vector<VisionFieldObject> data)
//{
//    //! @todo Implement + Comment
//}

void DataWrapper::debugRefresh()
{
    //! @todo Implement + Comment
}

bool DataWrapper::debugPublish(DEBUG_ID id, const vector<PointType> data_points, const Scalar& colour)
{
    //! @todo better debug printing + Comment
    
}

bool DataWrapper::debugPublish(DEBUG_ID id, const vector<PointType> data_points, const vector<Scalar>& colours)
{
    //! @todo better debug printing + Comment
}

bool DataWrapper::debugPublish(DEBUG_ID id, const Mat& img)
{
    //! @todo better debug printing + Comment
}

void DataWrapper::updateFrame()
{
    //! @todo Finish implementing & Comment
    m_actions = Blackboard->Actions;
    m_sensor_data = Blackboard->Sensors;
    
    if (m_current_frame != NULL and Blackboard->Image->GetTimestamp() - m_timestamp > 40)
        numFramesDropped++;
    numFramesProcessed++;
    m_current_frame = Blackboard->Image;
    m_timestamp = Blackboard->Image->GetTimestamp();
}

bool DataWrapper::loadLUTFromFile(const string& fileName)
{
    //! @todo Implement + Comment
}

void DataWrapper::process(JobList* jobs)
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

void DataWrapper::saveAnImage() const
{
    #if DEBUG_VISION_VERBOSITY > 1
        debug << "VisionControlWrapper::SaveAnImage(). Starting..." << endl;
    #endif

    if (!imagefile.is_open())
        imagefile.open((string(DATA_DIR) + string("image.strm")).c_str());
    if (!sensorfile.is_open())
        sensorfile.open((string(DATA_DIR) + string("sensor.strm")).c_str());

    if (imagefile.is_open() and numSavedImages < 2500)
    {
        if(sensorfile.is_open())
        {
            sensorfile << (*m_sensor_data) << flush;
        }
        NUImage buffer;
        buffer.cloneExisting(*m_current_frame);
        imagefile << buffer;
        numSavedImages++;
        
        if (isSavingImagesWithVaryingSettings)
        {
            CameraSettings tempCameraSettings = currentImage->getCameraSettings();
            if (numSavedImages % 10 == 0 )
            {
                tempCameraSettings.p_exposure.set(currentSettings.p_exposure.get() - 0);
            }
            else if (numSavedImages % 10 == 1 )
            {
                tempCameraSettings.p_exposure.set(currentSettings.p_exposure.get() - 50);
            }
            else if (numSavedImages % 10 == 2 )
            {
                tempCameraSettings.p_exposure.set(currentSettings.p_exposure.get() - 25);
            }
            else if (numSavedImages % 10 == 3 )
            {
                tempCameraSettings.p_exposure.set(currentSettings.p_exposure.get() - 0);
            }
            else if (numSavedImages % 10 == 4 )
            {
                tempCameraSettings.p_exposure.set(currentSettings.p_exposure.get() + 25);
            }
            else if (numSavedImages % 10 == 5 )
            {
                tempCameraSettings.p_exposure.set(currentSettings.p_exposure.get() + 50);
            }
            else if (numSavedImages % 10 == 6 )
            {
                tempCameraSettings.p_exposure.set(currentSettings.p_exposure.get() + 100);
            }
            else if (numSavedImages % 10 == 7 )
            {
                tempCameraSettings.p_exposure.set(currentSettings.p_exposure.get() + 150);
            }
            else if (numSavedImages % 10 == 8 )
            {
                tempCameraSettings.p_exposure.set(currentSettings.p_exposure.get() + 200);
            }
            else if (numSavedImages % 10 == 9 )
            {
                tempCameraSettings.p_exposure.set(currentSettings.p_exposure.get() + 300);
            }
            
            //Set the Camera Setttings using Jobs:
            ChangeCameraSettingsJob* newJob = new ChangeCameraSettingsJob(tempCameraSettings);
            Blackboard->Jobs->addCameraJob(newJob);
            //m_camera->setSettings(tempCameraSettings);
        }
    }
    #if DEBUG_VISION_VERBOSITY > 1
        debug << "Vision::SaveAnImage(). Finished" << endl;
    #endif
}
