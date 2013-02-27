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
#include "Vision/VisionTypes/VisionFieldObjects/fieldline.h"

using namespace std;
using namespace vision;
using cv::Mat;
using cv::VideoCapture;
using cv::Scalar;
using cv::namedWindow;
using cv::Vec3b;

class DataWrapper
{
    friend class VisionController;
    friend class VisionControlWrapper;

public:
    enum PIN_MAP {
        GPIO_BALL  = 23,
        GPIO_BGOAL = 18,
        GPIO_YGOAL = 22,
        GPIO_LINE  = 17
    };

    static string getIDName(DEBUG_ID id);

    static DataWrapper* getInstance(bool disp_on=false, bool cam=true);

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

    void debugRefresh();
    bool debugPublish(const vector<Ball>& data);
    bool debugPublish(const vector<Beacon>& data);
    bool debugPublish(const vector<Goal>& data);
    bool debugPublish(const vector<Obstacle>& data);
    bool debugPublish(const vector<FieldLine>& data);
    bool debugPublish(DEBUG_ID id, const vector<PointType>& data_points);
    bool debugPublish(DEBUG_ID id, const SegmentedRegion& region);
    bool debugPublish(DEBUG_ID id);
    bool debugPublish(DEBUG_ID id, const NUImage *const img);
    
    
private:private:
    enum INPUT_METHOD {
        CAMERA,
        STREAM
    };


    DataWrapper(bool disp_on=false, bool cam=true);
    ~DataWrapper();
    //void startImageFileGroup(string filename);
    bool updateFrame();
    bool loadLUTFromFile(const string& fileName);
    int getNumFramesDropped() const {return numFramesDropped;}      //! @brief Returns the number of dropped frames since start.
    int getNumFramesProcessed() const {return numFramesProcessed;}  //! @brief Returns the number of processed frames since start.

private:
    INPUT_METHOD m_input_method;

    bool m_writing;
    ofstream out_stream;

    string streamname;
    ifstream imagestrm;

    static DataWrapper* instance;

    NUImage* m_current_image;

    bool m_display_on;

    string configname;

    string LUTname;
    LookUpTable LUT;

    Horizon kinematics_horizon;

    PCCamera* m_camera;          //! Used when streaming from camera

    //! Used for displaying results
    string results_window_name;
    Mat results_img;
    
    //! Frame info
    double m_timestamp;
    int numFramesDropped;
    int numFramesProcessed;

    bool m_gpio;
};

#endif // DATAWRAPPERPC_H
