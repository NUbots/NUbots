#ifndef DATAWRAPPERPC_H
#define DATAWRAPPERPC_H

#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "Kinematics/Horizon.h"

#include "Vision/basicvisiontypes.h"
#include "Vision/VisionTypes/segmentedregion.h"
#include "Vision/VisionTools/pccamera.h"
#include "Vision/VisionTools/lookuptable.h"
#include "Vision/VisionTypes/VisionFieldObjects/ball.h"
#include "Vision/VisionTypes/VisionFieldObjects/beacon.h"
#include "Vision/VisionTypes/VisionFieldObjects/goal.h"
#include "Vision/VisionTypes/VisionFieldObjects/obstacle.h"

#define GROUP_NAME "/home/shannon/Images/paper"
#define GROUP_EXT ".png"

using namespace std;
using namespace cv;

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
        DBID_GOALS=10,
        DBID_BEACONS=11,
        DBID_BALLS=12,
        DBID_OBSTACLES=13,
        NUMBER_OF_IDS=14
    };

    static string getIDName(DEBUG_ID id);
    static string getIDName(DATA_ID id);

    static DataWrapper* getInstance();

    //! RETRIEVAL METHODS
    NUImage* getFrame();

    bool getCTGVector(vector<float>& ctgvector);    //for transforms
    bool getCTVector(vector<float>& ctvector);    //for transforms
    
    //! @brief Generates spoofed horizon line.
    const Horizon& getKinematicsHorizon();

    CameraSettings getCameraSettings();

    const LookUpTable& getLUT() const;
        
    //! PUBLISH METHODS
    void publish(DATA_ID id, const Mat& img);

    void debugRefresh();
    bool debugPublish(vector<Ball> data);
    bool debugPublish(vector<Beacon> data);
    bool debugPublish(vector<Goal> data);
    bool debugPublish(vector<Obstacle> data);
    bool debugPublish(DEBUG_ID id, const vector<PointType>& data_points);
    bool debugPublish(DEBUG_ID id, const SegmentedRegion& region);
    bool debugPublish(DEBUG_ID id, const Mat& img);
    
    
private:
    DataWrapper();
    ~DataWrapper();
    //void startImageFileGroup(string filename);
    bool updateFrame();
    bool loadLUTFromFile(const string& fileName);
    int getNumFramesDropped() const {return numFramesDropped;}      //! @brief Returns the number of dropped frames since start.
    int getNumFramesProcessed() const {return numFramesProcessed;}  //! @brief Returns the number of processed frames since start.
    
    void ycrcb2ycbcr(Mat* img_ycrcb);
    void generateImageFromMat(Mat& frame);

private:
    enum INPUT_METHOD {
        CAMERA,
        STREAM,
        FILE
    };
    
private:
    static const INPUT_METHOD METHOD = STREAM;  //CAMERA, STREAM, FILE

    static DataWrapper* instance;

    NUImage* m_current_image;

    string LUTname;
    LookUpTable LUT;

    Horizon kinematics_horizon;

    PCCamera* m_camera;          //! Used when streaming from camera

    //! Used when reading from strm
    string streamname;
    ifstream imagestrm;

    //! Used when reading from file
    unsigned char* m_yuyv_buffer;
    VideoCapture* capture;
    Mat m_current_image_cv;
    int num_images, cur_image;
    
    //! Used for debugging
    int debug_window_num;
    map<DEBUG_ID, pair<string, Mat>* > debug_map;
    pair<string, Mat>* debug_windows;

//    int id_window_map[NUMBER_OF_IDS];
//
//    string* debug_window_name;  //for array
//    Mat* debug_img;             //for array
    
    //! Used for displaying results
    string results_window_name;
    Mat results_img;
    
    //! Frame info
    double m_timestamp;
    int numFramesDropped;
    int numFramesProcessed;

    
    //NUSensorsData* m_sensor_data;

//! ONLY FOR DEBUGGING - DO NOT RELY ON THE FOLLOWING METHODS OR VARIABLES BEING PRESENT
public:
    void show(Mat* img);
};

#endif // DATAWRAPPERPC_H
