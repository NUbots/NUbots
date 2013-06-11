#include "datawrapperdarwin.h"

#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/Jobs/CameraJobs/ChangeCameraSettingsJob.h"
#include "nubotdataconfig.h"
#include "debug.h"
#include "debugverbosityvision.h"

#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "NUPlatform/NUActionators/NUSounds.h"
#include "Kinematics/Kinematics.h"

#include "Vision/VisionTypes/coloursegment.h"
#include "Vision/basicvisiontypes.h"
#include "Vision/visionconstants.h"

#include <boost/foreach.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/accumulators/statistics/count.hpp>

using namespace boost::accumulators;

DataWrapper* DataWrapper::instance = 0;

DataWrapper::DataWrapper()
{
    numFramesDropped = 0;
    numFramesProcessed = 0;
    numSavedImages = 0;
    loadLUTFromFile(string(DATA_DIR) + string("default.lut"));
    Blackboard->lookForBall = true; //initialise
    Blackboard->lookForGoals = true; //initialise
    isSavingImages = false;
    isSavingImagesWithVaryingSettings = false;

    VisionConstants::loadFromFile(string(CONFIG_DIR) + string("VisionOptions.cfg"));

    string sen_calib_name = string(CONFIG_DIR) + string("SensorCalibration.cfg");

    debug << "opening sensor calibration config: " << sen_calib_name << endl;
    if( ! m_sensor_calibration.ReadSettings(sen_calib_name)) {
        errorlog << "DataWrapper::DataWrapper() - failed to load sensor calibration: " << sen_calib_name << ". Using default values." << endl;
        m_sensor_calibration = SensorCalibration();
    }
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

//! @brief Retrieves the camera height returns it.
float DataWrapper::getCameraHeight() const
{
    return m_camera_height;
}

//! @brief Retrieves the camera pitch returns it.
float DataWrapper::getHeadPitch() const
{
    return m_head_pitch;
}

//! @brief Retrieves the camera yaw returns it.
float DataWrapper::getHeadYaw() const
{
    return m_head_yaw;
}

//! @brief Retrieves the body pitch returns it.
Vector3<float> DataWrapper::getOrientation() const
{
    return m_orientation;
}

//! @brief Returns the neck position snapshot.
Vector3<double> DataWrapper::getNeckPosition() const
{
    return m_neck_position;
}

Vector2<double> DataWrapper::getCameraFOV() const
{
    return Vector2<double>(camera_data->m_horizontalFov, camera_data->m_verticalFov);
}

SensorCalibration DataWrapper::getSensorCalibration() const
{
    return m_sensor_calibration;
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

void DataWrapper::debugPublish(vector<Ball> data) {
    #if VISION_WRAPPER_VERBOSITY > 1
        debug << "DataWrapper::debugPublish - DEBUG_ID = " << getIDName(DBID_BALLS) << endl;
        BOOST_FOREACH(Ball ball, data) {
            debug << "DataWrapper::debugPublish - Ball = " << ball << endl;
        }
    #endif
}

//void DataWrapper::debugPublish(vector<Beacon> data) {
//    #if VISION_WRAPPER_VERBOSITY > 1
//        debug << "DataWrapper::debugPublish - DEBUG_ID = " << getIDName(DBID_BEACONS) << endl;
//        BOOST_FOREACH(Beacon beacon, data) {
//            debug << "DataWrapper::debugPublish - Beacon = " << beacon << endl;
//        }
//    #endif
//}

void DataWrapper::debugPublish(vector<Goal> data) {
//    static accumulator_set<double, stats<tag::mean, tag::variance> > acc_d2p;
//    static accumulator_set<double, stats<tag::mean, tag::variance> > acc_w;
//    static int i=0;
//    if(data.size() > 2) {
//        cout << "3 or more posts found" << endl;
//    }
//    else if(data.size() == 2) {
//        if(data[0].getID() == GOAL_R) {
//            acc_d2p( data[0].getLocation().relativeRadial.x );
//            acc_w( data[0].width_dist);
//            i++;
//        }
//        else if(data[1].getID() == GOAL_R) {
//            acc_d2p( data[1].getLocation().relativeRadial.x );
//            acc_w( data[1].width_dist);
//            i++;
//        }
//        else {
//            cout << "two goals but no right one" << endl;
//        }
//    }
//    else if(data.size() == 1) {
//        acc_d2p( data.front().getLocation().relativeRadial.x );
//        acc_w( data.front().width_dist);
//        i++;
//    }
//    else {
//        cout << "No goals" << endl;
//    }

//    if(i >= 30) {
//        cout << "d2p: (" << mean(acc_d2p) << ", " << sqrt(variance(acc_d2p)) << ") width: " << "(" << mean(acc_w) << ", " << sqrt(variance(acc_w)) << ")" << endl;
//        acc_d2p = accumulator_set<double, stats<tag::mean, tag::variance> >();
//        acc_w = accumulator_set<double, stats<tag::mean, tag::variance> >();
//        i=0;
//    }
    #if VISION_WRAPPER_VERBOSITY > 1
        debug << "DataWrapper::debugPublish - DEBUG_ID = " << getIDName(DBID_GOALS) << endl;
        BOOST_FOREACH(Goal post, data) {
            debug << "DataWrapper::debugPublish - Goal = " << post << endl;
        }
    #endif
}

void DataWrapper::debugPublish(vector<Obstacle> data) {
    #if VISION_WRAPPER_VERBOSITY > 1
        debug << "DataWrapper::debugPublish - DEBUG_ID = " << getIDName(DBID_OBSTACLES) << endl;
        BOOST_FOREACH(Obstacle obst, data) {
            debug << "DataWrapper::debugPublish - Obstacle = " << obst << endl;
        }
    #endif
}

void DataWrapper::debugPublish(const vector<FieldLine> &data)
{
    #if VISION_WRAPPER_VERBOSITY > 2
        debug << "DataWrapper::debugPublish - DEBUG_ID = " << getIDName(id) << endl;
        BOOST_FOREACH(const FieldLine& l, data) {
            debug << "DataWrapper::debugPublish - Line = ";
            l.printLabel(debug);
            debug << endl;
        }
    #endif
}

void DataWrapper::debugPublish(const vector<CentreCircle> &data)
{
    #if VISION_WRAPPER_VERBOSITY > 2
        debug << "DataWrapper::debugPublish - DEBUG_ID = " << getIDName(id) << endl;
        BOOST_FOREACH(const CentreCircle& c, data) {
            debug << "DataWrapper::debugPublish - CentreCircle = ";
            debug << c << endl;
        }
    #endif
}

void DataWrapper::debugPublish(const vector<CornerPoint> &data)
{
    #if VISION_WRAPPER_VERBOSITY > 2
        debug << "DataWrapper::debugPublish - DEBUG_ID = " << getIDName(id) << endl;
        BOOST_FOREACH(const CornerPoint& c, data) {
            debug << "DataWrapper::debugPublish - CornerPoint = ";
            debug << c << endl;
        }
    #endif
}

void DataWrapper::debugPublish(DEBUG_ID id, const vector<Point> &data_points)
{
    //! @todo better debug printing + Comment
    #if VISION_WRAPPER_VERBOSITY > 2
        debug << "DataWrapper::debugPublish - DEBUG_ID = " << getIDName(id) << endl;
        debug << "\t" << id << endl;
        debug << "\t" << data_points << endl;
    #endif
}

void DataWrapper::debugPublish(DEBUG_ID id, const SegmentedRegion& region)
{
    //! @todo better debug printing + Comment
    switch(id) {
    case HORIZONTAL:
        Blackboard->horizontalScans = region;
        break;
    case VERTICAL:
        Blackboard->verticalScans = region;
        break;
    }



    #if VISION_WRAPPER_VERBOSITY > 2
        debug << "DataWrapper::debugPublish - DEBUG_ID = " << getIDName(id) << endl;
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
}

void DataWrapper::debugPublish(DEBUG_ID id, const vector<LSFittedLine> &data)
{
    //! @todo better debug printing + Comment
    #if VISION_WRAPPER_VERBOSITY > 2
        debug << "DataWrapper::debugPublish - DEBUG_ID = " << getIDName(id) << endl;
        BOOST_FOREACH(const LSFittedLine& line, data) {
            debug << "\t" << line << endl;
        }
    #endif
}

void DataWrapper::plotCurve(string name, vector<Vector2<double> > pts)
{
#if VISION_WRAPPER_VERBOSITY > 2
    debug << "DataWrapper::plotCurve " << name << " " << pts << endl;
#endif
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
    camera_data = Blackboard->CameraSpecs;
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
#if VISION_WRAPPER_VERBOSITY > 1
    debug << "Frames dropped: " << numFramesDropped << endl;
#endif

    vector<float> orientation(3, 0);

    //update kinematics snapshot
    if(!sensor_data->getCameraHeight(m_camera_height))
        errorlog << "DataWrapperDarwin - updateFrame() - failed to get camera height from NUSensorsData" << endl;
    if(!sensor_data->getPosition(NUSensorsData::HeadPitch, m_head_pitch))
        errorlog << "DataWrapperDarwin - updateFrame() - failed to get head pitch from NUSensorsData" << endl;
    if(!sensor_data->getPosition(NUSensorsData::HeadYaw, m_head_yaw))
        errorlog << "DataWrapperDarwin - updateFrame() - failed to get head yaw from NUSensorsData" << endl;
    if(!sensor_data->getOrientation(orientation))
        errorlog << "DataWrapperDarwin - updateFrame() - failed to get orientation from NUSensorsData" << endl;

    vector<float> left, right;
    if(sensor_data->get(NUSensorsData::LLegTransform, left) and sensor_data->get(NUSensorsData::RLegTransform, right))
    {
        m_neck_position = Kinematics::CalculateNeckPosition(Matrix4x4fromVector(left), Matrix4x4fromVector(right), m_sensor_calibration.m_neck_position_offset);
    }
    else
    {
        errorlog << "DataWrapperDarwin - updateFrame() - failed to get left or right leg transforms from NUSensorsData" << endl;
        // Default in case kinemtaics not available. Base height of darwin.
        m_neck_position = Vector3<double>(0.0, 0.0, 39.22);
    }

    return current_frame->getWidth() > 0 && current_frame->getHeight() > 0;
}

/**
*   @brief Post processes field objects with image timestamp.
*/
void DataWrapper::postProcess()
{
    if (current_frame != NULL && field_objects != NULL)
    {
        field_objects->postProcess(m_timestamp);
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
            //sensorfile << sensor_data_copy << flush;
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
