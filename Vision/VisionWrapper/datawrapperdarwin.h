#ifndef DATAWRAPPERDARWIN_H
#define DATAWRAPPERDARWIN_H

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Kinematics/Horizon.h"

#include "Vision/VisionTools/lookuptable.h"

using namespace std;
using namespace cv;


class DataWrapper
{
    friend class VisionController;
public:
    enum DATA_ID {
        DID_IMAGE,
        DID_CLASSED_IMAGE
    };
    
    enum DEBUG_ID {
        DBID_IMAGE=0,
        DBID_H_SCANS=1,
        DBID_V_SCANS=2,
        DBID_SEGMENTS=3,
        DBID_TRANSITIONS=4,
        DBID_HORIZON=5,
        DBID_GREENHORIZON_SCANS=6,
        DBID_GREENHORIZON_FINAL=7,
        DBID_OBJECT_POINTS=8,
        DBID_FILTERED_SEGMENTS=9,
        NUMBER_OF_IDS=10
    };

    static string getIDName(DEBUG_ID id);
    static string getIDName(DATA_ID id);
    static DataWrapper* getInstance();

    /**
    *   @brief Fetches the next frame from the webcam.
    */
    NUImage* getFrame();

    bool getCTGVector(vector<float>& ctgvector);    //for transforms
    
    //! @brief Returns a reference to the kinematics horizon line.
    const Horizon& getKinematicsHorizon();

    //! @brief Generates spoofed camera transform vector.
    bool getCTGVector(vector<float>& ctgvector);    //for transforms

    const LookUpTable& getLUT() const;
        
    //! PUBLISH METHODS
    void publish(DATA_ID id, const Mat& img);
    //void publish(DATA_ID id, vector<VisionObject> data);

    void debugRefresh();
    bool debugPublish(DEBUG_ID id, const vector<PointType> data_points, const Scalar& colour);
    bool debugPublish(DEBUG_ID id, const vector<PointType> data_points, const vector<Scalar>& colours);
    bool debugPublish(DEBUG_ID id, const Mat& img);

private:
    DataWrapper();
    ~DataWrapper();
    
    void updateFrame();
    bool loadLUTFromFile(const string& fileName);
    
    
private:
    static DataWrapper* instance;

    NUImage* m_current_frame;
    NUSensorsData* m_sensor_data;
    LookUpTable m_LUT;
    
    vector<float> m_horizon_coefficients;
    Horizon m_kinematics_horizon;
};

#endif // DATAWRAPPERDARWIN_H
