#include "datawrapperdarwin.h"

#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/Jobs/CameraJobs/ChangeCameraSettingsJob.h"
#include "nubotdataconfig.h"
#include "debug.h"
#include "debugverbosityvision.h"

#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "NUPlatform/NUActionators/NUSounds.h"

#include "Vision/VisionTypes/coloursegment.h"
#include "Vision/basicvisiontypes.h"

#include <boost/foreach.hpp>

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
    case DBID_GOALS:
        return "DBID_GOALS";
    case DBID_BEACONS:
        return "DBID_BEACONS";
    case DBID_BALLS:
        return "DBID_BALLS";
    case DBID_OBSTACLES:
        return "DBID_OBSTACLES";
    default:
        return "NOT VALID";
    }
}

DataWrapper::DataWrapper()
{
    numFramesDropped = 0;
    numFramesProcessed = 0;
    numSavedImages = 0;
    loadLUTFromFile(string(DATA_DIR) + string("default.lut"));
    Blackboard->lookForBall = true; //initialise
    Blackboard->lookForLandmarks = true; //initialise
    isSavingImages = false;
    isSavingImagesWithVaryingSettings = false;
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
*   @brief Fetches the next frame from the shared Blackboard.
*/
NUImage* DataWrapper::getFrame()
{
    return current_frame;
}

/*! @brief Retrieves the horizon data and builds a Horizon and returns it.
*   @return m_kinematics_horizon A reference to the kinematics horizon line.
*   @note This method has a chance of retrieving an invalid line, in this case
*           the old line is returned with the "exists" flag set to false.
*/
const Horizon& DataWrapper::getKinematicsHorizon()
{
    #if VISION_WRAPPER_VERBOSITY > 1
        debug << "DataWrapper::getKinematicsHorizon() - Begin" << endl;
    #endif
    if(sensor_data->getHorizon(m_horizon_coefficients)) {
        #if VISION_WRAPPER_VERBOSITY > 1
            debug << "DataWrapper::getKinematicsHorizon() - success" << endl;
        #endif
        m_kinematics_horizon.setLine(m_horizon_coefficients.at(0), m_horizon_coefficients.at(1), m_horizon_coefficients.at(2));
        m_kinematics_horizon.exists = true;
    }
    else {
        #if VISION_WRAPPER_VERBOSITY > 1
            debug << "DataWrapper::getKinematicsHorizon() - failed" << endl;
        #endif
        m_kinematics_horizon.setLineFromPoints(Point(0, current_frame->getHeight()), Point(current_frame->getWidth(), current_frame->getHeight()));
        m_kinematics_horizon.exists = false;
    }
    
    return m_kinematics_horizon;
}

/*! @brief Retrieves the camera to ground vector returns it.
*   @param ctgvector A reference to a float vector to fill.
*   @return valid Whether the retrieved values are valid or not.
*/
bool DataWrapper::getCTGVector(vector<float>& ctgvector)
{
    #if VISION_WRAPPER_VERBOSITY > 1
        debug << "DataWrapper::getCTGVector()" << endl;
    #endif
    return sensor_data->get(NUSensorsData::CameraToGroundTransform, ctgvector);
}

/*! @brief Retrieves the camera transform vector returns it.
*   @param ctgvector A reference to a float vector to fill.
*   @return valid Whether the retrieved values are valid or not.
*/
bool DataWrapper::getCTVector(vector<float>& ctvector)
{
    #if VISION_WRAPPER_VERBOSITY > 1
        debug << "DataWrapper::getCTVector()" << endl;
    #endif
    return sensor_data->get(NUSensorsData::CameraTransform, ctvector);
}

/*! @brief Retrieves the camera height returns it.
*   @param height A reference to a float to change.
*   @return valid Whether the retrieved value is valid or not.
*/
bool DataWrapper::getCameraHeight(float& height)
{
    return sensor_data->getCameraHeight(height);
}

/*! @brief Retrieves the camera pitch returns it.
*   @param pitch A reference to a float to change.
*   @return valid Whether the retrieved value is valid or not.
*/
bool DataWrapper::getCameraPitch(float& pitch)
{
    return sensor_data->getPosition(NUSensorsData::HeadPitch, pitch);
}

/*! @brief Retrieves the body pitch returns it.
*   @param pitch A reference to a float to change.
*   @return valid Whether the retrieved value is valid or not.
*/
bool DataWrapper::getBodyPitch(float& pitch)
{
    vector<float> orientation;
    bool valid = sensor_data->get(NUSensorsData::Orientation, orientation);
    if(valid && orientation.size() > 2) {
        pitch = orientation.at(1);
        return true;
    }
    else {
        pitch = 0;
        return false;
    }
}

/*! @brief Returns a reference to the stored Lookup Table
*   @return mLUT A reference to the current LUT
*/
const LookUpTable& DataWrapper::getLUT() const
{
    return m_LUT;
}
    
//! PUBLISH METHODS


void DataWrapper::publish(const vector<const VisionFieldObject*> &visual_objects)
{
    //! @todo Implement + Comment
    
    for(int i=0; i<visual_objects.size(); i++) {
        visual_objects.at(i)->addToExternalFieldObjects(field_objects, m_timestamp);
    }
}

void DataWrapper::publish(const VisionFieldObject* visual_object)
{
    //! @todo Implement + Comment
    visual_object->addToExternalFieldObjects(field_objects, m_timestamp);
}

void DataWrapper::debugRefresh()
{
    //! @todo Implement + Comment
}

bool DataWrapper::debugPublish(vector<Ball> data) {
    #if VISION_WRAPPER_VERBOSITY > 1
        if(data.empty()) {
            debug << "DataWrapper::debugPublish - empty vector DEBUG_ID = " << getIDName(DBID_BALLS) << endl;
            return false;
        }
        BOOST_FOREACH(Ball ball, data) {
            debug << "DataWrapper::debugPublish - Ball = " << ball << endl;
        }
    #endif
    return true; 
}

bool DataWrapper::debugPublish(vector<Beacon> data) {  
    #if VISION_WRAPPER_VERBOSITY > 1
        if(data.empty()) {
            debug << "DataWrapper::debugPublish - empty vector DEBUG_ID = " << getIDName(DBID_BEACONS) << endl;
            return false;
        }
        BOOST_FOREACH(Beacon beacon, data) {
            debug << "DataWrapper::debugPublish - Beacon = " << beacon << endl;
        }
    #endif
    return true;    
}

bool DataWrapper::debugPublish(vector<Goal> data) {
    #if VISION_WRAPPER_VERBOSITY > 1
        if(data.empty()) {
            debug << "DataWrapper::debugPublish - empty vector DEBUG_ID = " << getIDName(DBID_GOALS) << endl;
            return false;
        }
        BOOST_FOREACH(Goal post, data) {
            debug << "DataWrapper::debugPublish - Goal = " << post << endl;
        }
    #endif
    return true;    
}

bool DataWrapper::debugPublish(vector<Obstacle> data) {
    #if VISION_WRAPPER_VERBOSITY > 1
        if(data.empty()) {
            debug << "DataWrapper::debugPublish - empty vector DEBUG_ID = " << getIDName(DBID_OBSTACLES) << endl;
            return false;
        }
        BOOST_FOREACH(Obstacle obst, data) {
            debug << "DataWrapper::debugPublish - Obstacle = " << obst << endl;
        }
    #endif
    return true;  
}

bool DataWrapper::debugPublish(DEBUG_ID id, const vector<PointType>& data_points)
{
    //! @todo better debug printing + Comment
    vector<PointType>::const_iterator it;

    #if VISION_WRAPPER_VERBOSITY > 1
        debug << "DataWrapper::debugPublish - " << endl;
        if(data_points.empty()) {
            debug << "\tempty vector DEBUG_ID = " << getIDName(id) << endl;
            return false;
        }
    #endif

    #if VISION_WRAPPER_VERBOSITY > 2
        debug << "\t" << id << endl;
        debug << "\t" << data_points << endl;
    #endif

    return true;
}

bool DataWrapper::debugPublish(DEBUG_ID id, const SegmentedRegion& region)
{
    //! @todo better debug printing + Comment
    
    #if VISION_WRAPPER_VERBOSITY > 1
        if(region.getSegments().empty()) {
            errorlog << "DataWrapper::debugPublish - empty vector DEBUG_ID = " << getIDName(id) << endl;
            return false;
        }
    #endif
        
    #if VISION_WRAPPER_VERBOSITY > 2
        BOOST_FOREACH(const vector<ColourSegment>& line, region.getSegments()) {
            if(region.getDirection() == VisionID::HORIZONTAL)
                debug << "y: " << line.front().getStart().y << endl;
            else
                debug << "x: " << line.front().getStart().x << endl;
            BOOST_FOREACH(const ColourSegment& seg, line) {
                debug << "\t" << seg;
            }
        }
    #endif

    return true;
}

bool DataWrapper::debugPublish(DEBUG_ID id, const cv::Mat& img)
{
    //! @todo better debug printing + Comment
}

/*! @brief Updates the held information ready for a new frame.
*   Gets copies of the actions and sensors pointers from the blackboard and
*   gets a new image from the blackboard. Updates framecounts.
*   @return Whether the fetched data is valid.
*/
bool DataWrapper::updateFrame()
{
    #if VISION_WRAPPER_VERBOSITY > 1
        debug << "DataWrapper::updateFrame() - Begin" << endl;
    #endif
    //! @todo Finish implementing & Comment
    actions = Blackboard->Actions;
    sensor_data = Blackboard->Sensors;
    field_objects = Blackboard->Objects;
    
    if (current_frame != NULL and Blackboard->Image->GetTimestamp() - m_timestamp > 40)
        numFramesDropped++;
    numFramesProcessed++;
    current_frame = Blackboard->Image;
    
    if (current_frame == NULL || sensor_data == NULL || actions == NULL || field_objects == NULL)
    {
        #if VISION_WRAPPER_VERBOSITY > 1
            debug << "DataWrapper::updateFrame(): null reference from BB" << endl;
        #endif
        // keep object times updated.
        if(field_objects && sensor_data)
        {
            field_objects->preProcess(sensor_data->GetTimestamp());
            field_objects->postProcess(sensor_data->GetTimestamp());
        }
        return false;
    }
    m_timestamp = current_frame->GetTimestamp();
    //succesful
    field_objects->preProcess(m_timestamp);
    return true;
}

/**
*   @brief Post processes field objects with image timestamp.
*/
void DataWrapper::postProcess()
{
    if (current_frame != NULL && field_objects != NULL)
    {
        field_objects->postProcess(current_frame->GetTimestamp());
    }
}

/**
*   @brief loads the colour look up table
*   @param filename The filename for the LUT stored on disk
*   @note Taken from original vision system
*/
bool DataWrapper::loadLUTFromFile(const string& fileName)
{
    #if VISION_WRAPPER_VERBOSITY > 1
        debug << "DataWrapper::loadLUTFromFile() - " << fileName << endl;
    #endif
    return m_LUT.loadLUTFromFile(fileName);
}

/**
*   @brief Processes saving images jobs.
*   @param jobs The current JobList
*   @note Taken from original vision system
*/
void DataWrapper::process(JobList* jobs)
{
    #if VISION_WRAPPER_VERBOSITY > 1
        debug  << "DataWrapper::Process - Begin" << endl;
    #endif
    static list<Job*>::iterator it;     // the iterator over the motion jobs
    
    for (it = jobs->vision_begin(); it != jobs->vision_end();)
    {
        if ((*it)->getID() == Job::VISION_SAVE_IMAGES)
        {   
            #if VISION_WRAPPER_VERBOSITY > 1
                debug << "DataWrapper::process(): Processing a save images job." << endl;
            #endif
            static SaveImagesJob* job;
            job = (SaveImagesJob*) (*it);
            if(isSavingImages != job->saving()) {
                //if the job changes the saving images state
                if(job->saving() == true) {
                    //we weren't saving and now we've started
                    currentSettings = current_frame->getCameraSettings();
                    if (!imagefile.is_open())
                        imagefile.open((string(DATA_DIR) + string("image.strm")).c_str());
                    if (!sensorfile.is_open())
                        sensorfile.open((string(DATA_DIR) + string("sensor.strm")).c_str());
                    actions->add(NUActionatorsData::Sound, sensor_data->CurrentTime, NUSounds::START_SAVING_IMAGES);
                }
                else {
                    //we were saving and now we've finished
                    imagefile.flush();
                    sensorfile.flush();

                    ChangeCameraSettingsJob* newJob  = new ChangeCameraSettingsJob(currentSettings);
                    jobs->addCameraJob(newJob);
                    actions->add(NUActionatorsData::Sound, sensor_data->CurrentTime, NUSounds::STOP_SAVING_IMAGES);
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

/**
*   @brief Saves an image and the current sensor data to the associated streams.
*   @note Taken from original vision system
*/
void DataWrapper::saveAnImage()
{
    #if VISION_WRAPPER_VERBOSITY > 1
        debug << "DataWrapper::SaveAnImage(). Starting..." << endl;
    #endif

    if (!imagefile.is_open())
        imagefile.open((string(DATA_DIR) + string("image.strm")).c_str());
    if (!sensorfile.is_open())
        sensorfile.open((string(DATA_DIR) + string("sensor.strm")).c_str());

    if (imagefile.is_open() and numSavedImages < 2500)
    {
        if(sensorfile.is_open())
        {
            sensorfile << (*sensor_data) << flush;
        }
        NUImage buffer;
        buffer.cloneExisting(*current_frame);
        imagefile << buffer;
        numSavedImages++;
        
        if (isSavingImagesWithVaryingSettings)
        {
            CameraSettings tempCameraSettings = current_frame->getCameraSettings();
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
    #if VISION_WRAPPER_VERBOSITY > 1
        debug << "DataWrapper::SaveAnImage(). Finished" << endl;
    #endif
}
