#include "datawrapperdarwin.h"

DataWrapper* DataWrapper::instance = 0;

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

    
//! @brief Returns a reference to the kinematics horizon line.
const Line& DataWrapper::getKinematicsHorizon() const
{
    
}

    //! @brief Generates spoofed camera transform vector.
    bool getCTGVector(vector<float>& ctgvector);    //for transforms

    CameraSettings getCameraSettings();

    const LookUpTable& getLUT() const;
        
    //! PUBLISH METHODS
    void publish(DATA_ID id, const Mat& img);
    //void publish(DATA_ID id, vector<VisionObject> data);

    void debugRefresh();
    bool debugPublish(DEBUG_ID id, const vector<PointType> data_points, const Scalar& colour);
    bool debugPublish(DEBUG_ID id, const vector<PointType> data_points, const vector<Scalar>& colours);
    bool debugPublish(DEBUG_ID id, const Mat& img);


    DataWrapper();
    ~DataWrapper();
    
    void updateFrame();
    bool loadLUTFromFile(const string& fileName);

