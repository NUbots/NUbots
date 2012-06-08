#include <boost/foreach.hpp>
#include "datawrappernuview.h"
#include "Infrastructure/NUImage/ColorModelConversions.h"
#include "debug.h"
#include "nubotdataconfig.h"

#include "Vision/VisionTypes/coloursegment.h"

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
    m_current_image = m_camera->grabNewImage();
    LUTname = string(getenv("HOME")) +  string("/_nubot/default.lut");

    if(!loadLUTFromFile(LUTname)){
        errorlog << "DataWrapper::DataWrapper() - failed to load LUT: " << LUTname << endl;
    }

    numFramesDropped = numFramesProcessed = 0;

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

/**
*   @brief Fetches the next frame from the webcam.
*/
NUImage* DataWrapper::getFrame()
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
        m_kinematics_horizon.setLineFromPoints(Point(0, current_frame->getHeight()), Point(current_frame->getWidth(), current_frame->getHeight()));
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

}

void DataWrapper::publish(const VisionFieldObject* visual_object)
{
    
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
    
    return false;
}

//! Outputs debug data to the appropriate external interface
bool DataWrapper::debugPublish(DEBUG_ID id, const SegmentedRegion& region)
{
    return false;
}

bool DataWrapper::debugPublish(DEBUG_ID id, const Mat &img)
{
    return false;
}

bool DataWrapper::updateFrame()
{
    return true;
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

void DataWrapper::setRawImage(NUImage* image)
{
    m_current_image = image;
}

void DataWrapper::setSensorData(NUSensorsData* NUSensorsData)
{
    sensor_data = NUSensorsData;
}

void DataWrapper::setFieldObjects(FieldObjects *fieldObjects)
{
    field_objects = fieldObjects;
}
