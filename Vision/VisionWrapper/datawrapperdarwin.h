#ifndef DATAWRAPPERDARWIN_H
#define DATAWRAPPERDARWIN_H

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "NUPlatform/Platforms/Darwin/DarwinCamera.h"
#include "visioncontroller.h"

#include "Infrastructure/NUSensorsData/NUSensorsData.h"

using namespace std;
using namespace cv;


class DataWrapper
{
    friend class VisionController;
public:
    static DataWrapper* getInstance();

    /**
    *   @brief Fetches the next frame from the webcam.
    */
    NUImage* getFrame();

    bool getCTGVector(vector<float>& ctgvector);    //for transforms
    
    //! @brief Returns a reference to the kinematics horizon line.
    const Line& getKinematicsHorizon();

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



private:
    DataWrapper();
    ~DataWrapper();
    
    void updateFrame();
    bool loadLUTFromFile(const string& fileName);

    
    
private:
    static DataWrapper* instance;

    NUImage* m_current_frame;
    NUSensorsData* m_sensor_data;
};

#endif // DATAWRAPPERDARWIN_H
