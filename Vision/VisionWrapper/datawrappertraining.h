#ifndef DATAWRAPPERTRAINING_H
#define DATAWRAPPERTRAINING_H

#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>

#include "Kinematics/Horizon.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"

#include "Vision/basicvisiontypes.h"
#include "Vision/VisionTypes/segmentedregion.h"
#include "Vision/VisionTools/pccamera.h"
#include "Vision/VisionTools/lookuptable.h"
#include "Vision/VisionTypes/VisionFieldObjects/ball.h"
#include "Vision/VisionTypes/VisionFieldObjects/beacon.h"
#include "Vision/VisionTypes/VisionFieldObjects/goal.h"
#include "Vision/VisionTypes/VisionFieldObjects/obstacle.h"
#include "Tools/Math/LSFittedLine.h"

#define GROUP_NAME "/home/shannon/Images/paper"
#define GROUP_EXT ".png"

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
        DBID_MATCHED_SEGMENTS=4,
        DBID_HORIZON=5,
        DBID_GREENHORIZON_SCANS=6,
        DBID_GREENHORIZON_FINAL=7,
        DBID_OBJECT_POINTS=8,
        DBID_FILTERED_SEGMENTS=9,
        DBID_GOALS=10,
        DBID_BEACONS=11,
        DBID_BALLS=12,
        DBID_OBSTACLES=13,
        DBID_LINES=14,
        NUMBER_OF_IDS=15
    };

    static string getIDName(DEBUG_ID id);
    static string getIDName(DATA_ID id);

    static DataWrapper* getInstance();

    //! RETRIEVAL METHODS
    NUImage* getFrame();

    bool getCTGVector(vector<float>& ctgvector);    //for transforms
    bool getCTVector(vector<float>& ctvector);    //for transforms
    bool getCameraHeight(float& height);            //for transforms
    bool getCameraPitch(float& pitch);              //for transforms
    bool getBodyPitch(float& pitch);

    //! @brief Generates spoofed horizon line.
    const Horizon& getKinematicsHorizon();

    CameraSettings getCameraSettings();

    const LookUpTable& getLUT() const;

    //! PUBLISH METHODS
    void publish(const vector<const VisionFieldObject*> &visual_objects);
    void publish(const VisionFieldObject* visual_object);

    void debugRefresh() {}
    bool debugPublish(const vector<Ball>& data);
    bool debugPublish(const vector<Beacon>& data);
    bool debugPublish(const vector<Goal>& data);
    bool debugPublish(const vector<Obstacle>& data);
    bool debugPublish(const vector<LSFittedLine>& data);
    bool debugPublish(DEBUG_ID id, const vector<PointType>& data_points) {return id<NUMBER_OF_IDS && !data_points.empty();}
    bool debugPublish(DEBUG_ID id, const SegmentedRegion& region) {return id<NUMBER_OF_IDS && !region.empty();}
    bool debugPublish(DEBUG_ID id, const cv::Mat& img) {return id<NUMBER_OF_IDS && !img.empty();}


private:
    DataWrapper();
    ~DataWrapper();
    bool updateFrame();
    int getNumFramesDropped() const {return numFramesDropped;}      //! @brief Returns the number of dropped frames since start.
    int getNumFramesProcessed() const {return numFramesProcessed;}  //! @brief Returns the number of processed frames since start.

    void resetHistory();
    void printHistory(ostream& out);
    bool setStream(const string& filename);
    bool loadLUTFromFile(const string& fileName);
    void resetStream();

private:
    static DataWrapper* instance;

    NUImage* m_current_image;

    string LUTname;
    LookUpTable LUT;

    Horizon kinematics_horizon;

    //! Used when reading from strm
    string image_stream_name;
    ifstream imagestrm;
    string sensor_stream_name;
    ifstream sensorstrm;

    //! Used for debugging
    int debug_window_num;
    map<DEBUG_ID, pair<string, cv::Mat>* > debug_map;
    pair<string, cv::Mat>* debug_windows;

    //! Used for displaying results
    string results_window_name;
    cv::Mat results_img;

    //! Frame info
    double m_timestamp;
    int numFramesDropped;
    int numFramesProcessed;

    //! Detection info
    vector<Ball> ball_detections;
    vector<Goal> goal_detections;
    vector<Beacon> beacon_detections;
    vector<Obstacle> obstacle_detections;
    vector<LSFittedLine> line_detections;
    vector<vector<Ball> > ball_detection_history;
    vector<vector<Goal> > goal_detection_history;
    vector<vector<Beacon> > beacon_detection_history;
    vector<vector<Obstacle> > obstacle_detection_history;
    vector<vector<LSFittedLine> > line_detection_history;

    //vector<VisionFieldObject*> detections;
    //vector< vector<VisionFieldObject*> > detection_history;

    NUSensorsData* m_sensor_data;
};

#endif // DATAWRAPPERTRAINING_H
