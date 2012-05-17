#ifndef DATAWRAPPERDARWIN_H
#define DATAWRAPPERDARWIN_H

#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/Jobs/VisionJobs/SaveImagesJob.h"
#include "Kinematics/Horizon.h"

#include "Vision/VisionTools/lookuptable.h"
#include "Vision/basicvisiontypes.h"
#include "Vision/VisionTypes/segmentedregion.h"
#include "Vision/VisionTypes/VisionFieldObjects/visionfieldobject.h"
#include "Vision/VisionTypes/VisionFieldObjects/ball.h"
#include "Vision/VisionTypes/VisionFieldObjects/beacon.h"
#include "Vision/VisionTypes/VisionFieldObjects/goal.h"
#include "Vision/VisionTypes/VisionFieldObjects/obstacle.h"

using namespace std;
using namespace cv;

class NUSensorsData;
class NUActionatorsData;

class DataWrapper
{
    friend class VisionController;
    friend class VisionControlWrapper;
    
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

public:
    //! Data access interface
    
    NUImage* getFrame();

    bool getCTGVector(vector<float>& ctgvector);    //for transforms
    bool getCTVector(vector<float>& ctvector);    //for transforms
    
    //! @brief Returns a reference to the kinematics horizon line.
    const Horizon& getKinematicsHorizon();

    const LookUpTable& getLUT() const;
        
    //! Data publish interface
    void publish(const vector<VisionFieldObject*>& visual_objects);
    //void publish(DATA_ID id, vector<VisionObject> data);

    void debugRefresh();
    bool debugPublish(vector<Ball> data);
    bool debugPublish(vector<Beacon> data);
    bool debugPublish(vector<Goal> data);
    bool debugPublish(vector<Obstacle> data);
    bool debugPublish(DEBUG_ID id, const vector<PointType>& data_points);
    bool debugPublish(DEBUG_ID id, const SegmentedRegion& region);
    bool debugPublish(DEBUG_ID id, const Mat& img);

    //! Control interface       
private:    
    bool updateFrame();
    bool loadLUTFromFile(const string& fileName);
    int getNumFramesDropped() const {return numFramesDropped;}      //! @brief Returns the number of dropped frames since start.
    int getNumFramesProcessed() const {return numFramesProcessed;}  //! @brief Returns the number of processed frames since start.
    void process(JobList* jobs);
    
    //! Vision Save Images Interface
    void saveAnImage();
    
private:
    DataWrapper();
    ~DataWrapper();
    
    static DataWrapper* instance;

    LookUpTable m_LUT;
    
    vector<float> m_horizon_coefficients;
    Horizon m_kinematics_horizon;
    
    //! Frame info
    double m_timestamp;
    int numFramesDropped;
    int numFramesProcessed;
    
    //! SavingImages:
    bool isSavingImages;
    bool isSavingImagesWithVaryingSettings;
    int numSavedImages;
    ofstream imagefile;
    ofstream sensorfile;
    CameraSettings currentSettings;
    
    //! Shared data objects
    NUImage* current_frame;
    NUSensorsData* sensor_data;             //! pointer to shared sensor data
    NUActionatorsData* actions;             //! pointer to shared actionators data
    FieldObjects* field_objects;            //! pointer to shared fieldobject data
};

#endif // DATAWRAPPERDARWIN_H
