#include "datawrapperdarwin.h"

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

NUImage* DataWrapper::getFrame()
{
    return Blackboard->Image;
}

bool DataWrapper::getCTGVector(vector<float> &ctgvector)
{
    bool isOK = Blackboard->Sensors->get(NUSensorsData::CameraToGroundTransform, ctgvector);
    return isOK;
}
    
/*! @brief Retrieves the horizon data and builds a Horizon and returns it.
*   @return m_kinematics_horizon A reference to the kinematics horizon line.
*   @note This method has a chance of retrieving an invalid line, in this case
*           the old line is returned with the "exists" flag set to false.
*/
const Horizon& DataWrapper::getKinematicsHorizon() const
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
    return LUT;
}
    
//! PUBLISH METHODS
void DataWrapper::publish(DATA_ID id, const Mat& img)
{
    //! @todo Implement + Comment
}

void DataWrapper::publish(DATA_ID id, vector<VisionFieldObject> data)
{
    //! @todo Implement + Comment
}

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
    //! @todo Implement + Comment
}

bool DataWrapper::loadLUTFromFile(const string& fileName)
{
    //! @todo Implement + Comment
}
