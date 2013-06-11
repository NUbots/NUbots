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
#include "Vision/VisionTypes/VisionFieldObjects/centrecircle.h"
#include "Vision/VisionTypes/VisionFieldObjects/cornerpoint.h"

#include "Vision/VisionTypes/RANSACTypes/ransacgoal.h"

#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "NUPlatform/NUCamera/NUCameraData.h"

#include "mainwindow.h"

///DEBUG
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>

using namespace boost::accumulators;
//END DEBUG

using std::vector;
using std::string;

using namespace Vision;

class DataWrapper
{
    friend class VisionController;
    friend class VisionControlWrapper;

private:
    enum INPUT_METHOD {
        CAMERA,
        STREAM
    };

public:
    static DataWrapper* getInstance();

    //! RETRIEVAL METHODS
    NUImage* getFrame();

    bool getCTGVector(vector<float>& ctgvector);    //for transforms
    bool getCTVector(vector<float>& ctvector);    //for transforms
    bool getCameraHeight(float& height);            //for transforms
    bool getCameraPitch(float& pitch);              //for transforms
    bool getCameraYaw(float& yaw);                  //for transforms
    bool getBodyPitch(float& pitch);
    Vector2<double> getCameraFOV() const;

    //! @brief Generates spoofed horizon line.
    const Horizon& getKinematicsHorizon();

    CameraSettings getCameraSettings();

    const LookUpTable& getLUT() const;

    //! PUBLISH METHODS
    void publish(const std::vector<const VisionFieldObject*> &visual_objects);
    void publish(const VisionFieldObject* visual_object);

    void debugPublish(const std::vector<Ball>& data);
//    bool debugPublish(const std::vector<Beacon>& data);
    void debugPublish(const std::vector<Goal>& data);
    //void debugPublish(int i, const std::vector<Goal>& d);
    void debugPublish(const std::vector<Obstacle>& data);
    void debugPublish(const std::vector<FieldLine>& data);
    void debugPublish(const std::vector<CentreCircle>& data);
    void debugPublish(const std::vector<CornerPoint>& data);
    void debugPublish(DEBUG_ID id, const std::vector<Point>& data_points);
    void debugPublish(DEBUG_ID id, const std::vector<GroundPoint>& data_points);
    void debugPublish(DEBUG_ID id, const SegmentedRegion& region);
    void debugPublish(DEBUG_ID id);
    void debugPublish(DEBUG_ID id, const NUImage *const img);
    void debugPublish(DEBUG_ID id, const std::vector<LSFittedLine>& data);
    void debugPublish(DEBUG_ID id, const std::vector<Goal>& data);

    void plotCurve(std::string name, std::vector< Point > pts);
    void plotLineSegments(std::string name, std::vector< Point > pts);
    void plotHistogram(std::string name, const Histogram1D& hist, Colour colour = yellow);


private:
    DataWrapper(MainWindow* ui, bool ok, INPUT_METHOD method, std::string istrm, std::string sstrm, std::string cfg, std::string lname);
    ~DataWrapper();
    bool updateFrame();
    bool loadLUTFromFile(const std::string& fileName);
    int getNumFramesDropped() const {return numFramesDropped;}      //! @brief Returns the number of dropped frames since start.
    int getNumFramesProcessed() const {return numFramesProcessed;}  //! @brief Returns the number of processed frames since start.

private:
    static DataWrapper* instance;

    bool m_ok;

    //DEBUG
    accumulator_set<double, stats<tag::mean, tag::variance> > acc_hist, acc_r1, acc_r2;
    std::pair<double, double> ratio_hist, ratio_r1, ratio_r2;
    //END DEBUG

    MainWindow* gui;

    INPUT_METHOD m_method;  //CAMERA, STREAM

    NUImage m_current_image;
    NUSensorsData m_sensor_data;

    std::string configname;

    std::string LUTname;
    LookUpTable LUT;

    Horizon kinematics_horizon;

    NUCamera* m_camera;
    NUCameraData m_camspecs;

    //! Used when reading from strm
    std::string streamname;
    ifstream imagestrm;
    bool using_sensors;
    std::string sensorstreamname;
    ifstream sensorstrm;

    //! Frame info
    int numFramesDropped;
    int numFramesProcessed;

    //debug, remove later
    int true_num_posts;
};

#endif // DATAWRAPPERQT_H
