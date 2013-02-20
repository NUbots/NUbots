#ifndef DATAWRAPPERQT_H
#define DATAWRAPPERQT_H

#include <iostream>
#include <fstream>

#include "Kinematics/Horizon.h"

#ifdef TARGET_OS_IS_WINDOWS
#include "NUPlatform/Platforms/Generic/Cameras/NUOpenCVCamera.h"
#else
#include "Vision/VisionTools/pccamera.h"
#endif

#include "Vision/basicvisiontypes.h"
#include "Vision/VisionTypes/segmentedregion.h"
#include "Vision/VisionTools/lookuptable.h"
#include "Vision/VisionTypes/VisionFieldObjects/ball.h"
#include "Vision/VisionTypes/VisionFieldObjects/beacon.h"
#include "Vision/VisionTypes/VisionFieldObjects/goal.h"
#include "Vision/VisionTypes/VisionFieldObjects/obstacle.h"
#include "Vision/VisionTypes/VisionFieldObjects/fieldline.h"
#include "Tools/Math/LSFittedLine.h"

#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "NUPlatform/NUCamera/NUCameraData.h"

#include "mainwindow.h"

using std::vector;
using std::string;

using namespace Vision;

class DataWrapper
{
    friend class VisionController;
    friend class VisionControlWrapper;

public:
    static DataWrapper* getInstance();

    //! RETRIEVAL METHODS
    NUImage* getFrame();

    bool getCTGVector(vector<float>& ctgvector);    //for transforms
    bool getCTVector(vector<float>& ctvector);    //for transforms
    bool getCameraHeight(float& height);            //for transforms
    bool getCameraPitch(float& pitch);              //for transforms
    bool getBodyPitch(float& pitch);
    Vector2<double> getCameraFOV() const {return Vector2<double>(m_camspecs.m_horizontalFov, m_camspecs.m_verticalFov);}

    //! @brief Generates spoofed horizon line.
    const Horizon& getKinematicsHorizon();

    CameraSettings getCameraSettings();

    const LookUpTable& getLUT() const;

    //! PUBLISH METHODS
    void publish(const vector<const VisionFieldObject*> &visual_objects);
    void publish(const VisionFieldObject* visual_object);

    void debugRefresh();
    void debugPublish(const vector<Ball>& data);
//    bool debugPublish(const vector<Beacon>& data);
    void debugPublish(const vector<Goal>& data);
    void debugPublish(const vector<Obstacle>& data);
    void debugPublish(const vector<FieldLine>& data);
    void debugPublish(DEBUG_ID id, const vector< Vector2<double> >& data_points);
    void debugPublish(DEBUG_ID id, const vector<Point>& data_points);
    void debugPublish(DEBUG_ID id, const SegmentedRegion& region);
    void debugPublish(DEBUG_ID id);
    void debugPublish(DEBUG_ID id, const NUImage *const img);
    void debugPublish(DEBUG_ID id, const vector<LSFittedLine> &data);

    void plot(string name, vector< Vector2<double> > pts);


private:
    DataWrapper(MainWindow* ui);
    ~DataWrapper();
    bool updateFrame();
    bool loadLUTFromFile(const string& fileName);
    int getNumFramesDropped() const {return numFramesDropped;}      //! @brief Returns the number of dropped frames since start.
    int getNumFramesProcessed() const {return numFramesProcessed;}  //! @brief Returns the number of processed frames since start.

private:
    enum INPUT_METHOD {
        CAMERA,
        STREAM
    };

private:
    static DataWrapper* instance;

    bool ok;

    MainWindow* gui;

    INPUT_METHOD m_method;  //CAMERA, STREAM

    NUImage m_current_image;
    NUSensorsData m_sensor_data;

    string configname;

    string LUTname;
    LookUpTable LUT;

    Horizon kinematics_horizon;

    NUCamera* m_camera;
    NUCameraData m_camspecs;

    //! Used when reading from strm
    string streamname;
    ifstream imagestrm;
    bool using_sensors;
    string sensorstreamname;
    ifstream sensorstrm;

    //! Frame info
    double m_timestamp;
    int numFramesDropped;
    int numFramesProcessed;
};

#endif // DATAWRAPPERQT_H
