#include <boost/foreach.hpp>
#include "datawrappernuview.h"
#include "Infrastructure/NUImage/ColorModelConversions.h"
#include "debug.h"
#include "debugverbosityvision.h"
#include "nubotdataconfig.h"

#include "Vision/VisionTypes/coloursegment.h"
#include "Infrastructure/Jobs/JobList.h"
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

void getPointsAndColoursFromSegments(const vector< vector<ColourSegment> >& segments, vector<Scalar>& colours, vector<PointType>& pts)
{
    unsigned char r, g, b;
    
    BOOST_FOREACH(const vector<ColourSegment>& line, segments) {
        BOOST_FOREACH(const ColourSegment& seg, line) {
            ClassIndex::getColourAsRGB(seg.getColour(), r, g, b);
            pts.push_back(seg.getStart());
            pts.push_back(seg.getEnd());
            colours.push_back(Scalar(b,g,r));
        }
    }
}

DataWrapper::DataWrapper()
{
    numFramesDropped = numFramesProcessed = 0;
    display_callback = NULL;
}

DataWrapper::~DataWrapper()
{
}

DataWrapper* DataWrapper::getInstance()
{
    if(!instance)
        instance = new DataWrapper();
    return instance;
}

//void DataWrapper::setCallBack(virtualNUbot* virtual_nubot)
//{
//    m_virtual_nubot = virtual_nubot;
//}

/**
*   @brief Fetches the next frame from the webcam.
*/
const NUImage* DataWrapper::getFrame()
{
    return m_current_image;
}

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

bool DataWrapper::getCameraHeight(float& height)
{
    return sensor_data->getCameraHeight(height);
}

bool DataWrapper::getCameraPitch(float& pitch)
{
    return sensor_data->getPosition(NUSensorsData::HeadPitch, pitch);
}

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
        m_kinematics_horizon.setLineFromPoints(Point(0, 0), Point(m_current_image->getWidth(), 0));

        m_kinematics_horizon.exists = false;
    }
    
    return m_kinematics_horizon;
}

////! @brief Returns image width.
//unsigned int DataWrapper::getImageWidth()
//{
//    return m_image_width;
//}

////! @brief Returns image height.
//unsigned int DataWrapper::getImageHeight()
//{
//    return m_image_height;
//}

//! @brief Returns camera settings.
CameraSettings DataWrapper::getCameraSettings()
{
    return CameraSettings();
}

const LookUpTable& DataWrapper::getLUT() const
{
    return LUT;
}

//! Outputs supply data to the appropriate external interface
void DataWrapper::publish(DATA_ID id, const Mat &img)
{
    
}

void DataWrapper::publish(const vector<const VisionFieldObject*> &visual_objects)
{
    for(int i=0; i<visual_objects.size(); i++) {
        visual_objects.at(i)->addToExternalFieldObjects(field_objects, m_timestamp);
    }
}

void DataWrapper::publish(const VisionFieldObject* visual_object)
{
    cout << m_timestamp << endl;
    visual_object->addToExternalFieldObjects(field_objects, m_timestamp);
}

//! Outputs debug data to the appropriate external interface
void DataWrapper::debugRefresh()
{
    
}

bool DataWrapper::debugPublish(vector<Ball> data) {
    return false;
}

bool DataWrapper::debugPublish(vector<Beacon> data) {
    return false;
}

bool DataWrapper::debugPublish(vector<Goal> data) {
    return false;
}

bool DataWrapper::debugPublish(vector<Obstacle> data) {
    
    return false;
}

bool DataWrapper::debugPublish(DEBUG_ID id, const vector<PointType>& data_points)
{
    #if VISION_WRAPPER_VERBOSITY > 1
        if(data_points.empty()) {
            debug << "DataWrapper::debugPublish - empty vector DEBUG_ID = " << getIDName(id) << endl;
            return false;
        }
    #endif

    #if VISION_WRAPPER_VERBOSITY > 2
        debug << id << endl;
        debug << colour[0] << "," << colour[1] << "," << colour[2] << "," << colour[3] << "\t";
        debug << data_points << endl;
    #endif

    switch(id) {
    case DBID_H_SCANS:
        errorlog << "DataWrapper::debugPublish - DBID_H_SCANS printing not implemented" << endl;
//        BOOST_FOREACH(const PointType& pt, data_points) {
//            line(img, cv::Point2i(0, pt.y), cv::Point2i(img.cols, pt.y), Scalar(127,127,127), 1);
//        }
        return false;
    case DBID_V_SCANS:
        errorlog << "DataWrapper::debugPublish - DBID_V_SCANS printing not implemented" << endl;
//        BOOST_FOREACH(const PointType& pt, data_points) {
//            line(img, cv::Point2i(pt.x, pt.y), cv::Point2i(pt.x, img.rows), Scalar(127,127,127), 1);
//        }
        return false;
    case DBID_TRANSITIONS:
        errorlog << "DataWrapper::debugPublish - DBID_TRANSITIONS printing not implemented" << endl;
//        BOOST_FOREACH(const PointType& pt, data_points) {
//            circle(img, cv::Point2i(pt.x, pt.y), 1, Scalar(255,255,0), 4);
//        }
        return false;
    case DBID_HORIZON:
        errorlog << "DataWrapper::debugPublish - DBID_HORIZON printing not implemented" << endl;
        return false;
    case DBID_GREENHORIZON_SCANS:
        if(display_callback != NULL)
            (*display_callback)(data_points, GLDisplay::greenHorizonScanPoints);
        else
            errorlog << "DataWrapper::debugPublish - null callback pointer" << endl;
        break;
    case DBID_GREENHORIZON_FINAL:
        if(display_callback != NULL)
            (*display_callback)(data_points, GLDisplay::greenHorizonPoints);
        else
            errorlog << "DataWrapper::debugPublish - null callback pointer" << endl;
        break;
    case DBID_OBJECT_POINTS:
        errorlog << "DataWrapper::debugPublish - DBID_OBJECT_POINTS printing not implemented" << endl;
//        BOOST_FOREACH(const PointType& pt, data_points) {
//            circle(img, cv::Point2i(pt.x, pt.y), 1, Scalar(0,0,255), 4);
//        }
        return false;
    default:
        errorlog << "DataWrapper::debugPublish - Called with invalid id" << endl;
        return false;
    }

    return true;
}

//! Outputs debug data to the appropriate external interface
bool DataWrapper::debugPublish(DEBUG_ID id, const SegmentedRegion& region)
{
    vector<PointType> data_points;
    vector<Scalar> colours;
    vector<PointType>::const_iterator it;
    vector<Scalar>::const_iterator c_it;

    getPointsAndColoursFromSegments(region.getSegments(), colours, data_points);

    if(data_points.empty() || colours.empty()) {
        errorlog << "DataWrapper::debugPublish - empty vector DEBUG_ID = " << getIDName(id) << endl;
        return false;
    }
#if VISION_WRAPPER_VERBOSITY > 2
    debug << id << endl;
    debug << colours.front()[0] << "," << colours.front()[1] << "," << colours.front()[2] << "," << colours.front()[3] << "\t";
    debug << data_points << endl;
#endif

    switch(id) {
    case DBID_SEGMENTS:
        errorlog << "DataWrapper::debugPublish - DBID_SEGMENTS printing not implemented" << endl;
//        c_it = colours.begin();
//        for (it = data_points.begin(); it < data_points.end()-1; it+=2) {
//            //draws a line between each consecutive pair of points of the corresponding colour
//            line(img, cv::Point2i(it->x, it->y), cv::Point2i((it+1)->x, (it+1)->y), *c_it, 1);
//            c_it++;
//        }
        return false;
    case DBID_FILTERED_SEGMENTS:
        errorlog << "DataWrapper::debugPublish - DBID_FILTERED_SEGMENTS printing not implemented" << endl;
//        c_it = colours.begin();
//        for (it = data_points.begin(); it < data_points.end()-1; it+=2) {
//            //draws a line between each consecutive pair of points of the corresponding colour
//            line(img, cv::Point2i(it->x, it->y), cv::Point2i((it+1)->x, (it+1)->y), *c_it, 1);
//            c_it++;
//        }
        return false;
    default:
        errorlog << "DataWrapper::debugPublish - Called with invalid id" << endl;
        return false;
    }

    return true;
}

bool DataWrapper::debugPublish(DEBUG_ID id, const Mat &img)
{
    return false;
}

bool DataWrapper::updateFrame()
{
    if (m_current_image == NULL || sensor_data == NULL || actions == NULL || field_objects == NULL)
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
    m_timestamp = m_current_image->GetTimestamp();
    cout << "pre: " << m_current_image->GetTimestamp() << endl;
    field_objects->preProcess(m_current_image->GetTimestamp());
    return true;
}

void DataWrapper::postProcess()
{
    if (m_current_image != NULL && field_objects != NULL)
    {
        cout << "post: " << m_current_image->GetTimestamp() << endl;
        field_objects->postProcess(m_current_image->GetTimestamp());
    }
}

/**
*   @brief loads the colour look up table
*   @param filename The filename for the LUT stored on disk
*   @note Taken from original vision system
*/
bool DataWrapper::loadLUTFromFile(const string& fileName)
{
    return LUT.loadLUTFromFile(fileName);
}

void DataWrapper::setRawImage(const NUImage* image)
{
    m_current_image = image;
}

void DataWrapper::setSensorData(NUSensorsData* sensors)
{
    sensor_data = sensors;
}

void DataWrapper::setFieldObjects(FieldObjects *fieldObjects)
{
    field_objects = fieldObjects;
}

void DataWrapper::setLUT(unsigned char *vals)
{
    LUT.set(vals);
}

void DataWrapper::classifyImage(ClassifiedImage &target) const
{
    if(m_current_image != NULL) {
        int width = m_current_image->getWidth();
        int height = m_current_image->getHeight();

        target.setImageDimensions(width,height);
        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                target.image[y][x] = LUT.classifyPixel((*m_current_image)(x,y));
            }
        }
    }
}

void DataWrapper::classifyPreviewImage(ClassifiedImage &target,unsigned char* temp_vals) const
{
    int width = m_current_image->getWidth();
    int height = m_current_image->getHeight();

    target.setImageDimensions(width,height);
    LookUpTable tempLUT(temp_vals);
    //qDebug() << "Begin Loop:";
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            target.image[y][x] = tempLUT.classifyPixel((*m_current_image)(x,y));
        }
    }
}

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
        buffer.cloneExisting(*m_current_image);
        imagefile << buffer;
        numSavedImages++;

        if (isSavingImagesWithVaryingSettings)
        {
            CameraSettings tempCameraSettings = m_current_image->getCameraSettings();
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
