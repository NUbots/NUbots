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
//#include "Vision/VisionTypes/VisionFieldObjects/beacon.h"
#include "Vision/VisionTypes/VisionFieldObjects/goal.h"
#include "Vision/VisionTypes/VisionFieldObjects/obstacle.h"
#include "Vision/VisionTypes/VisionFieldObjects/fieldline.h"

using namespace std;

class NUSensorsData;
class NUActionatorsData;

class DataWrapper
{
    friend class VisionController;
    friend class VisionControlWrapper;
    
public:
    static DataWrapper* getInstance();

public:
    //! Data access interface
    
    NUImage* getFrame();
    bool getCTGVector(vector<float>& ctgvector);    //for transforms
    bool getCTVector(vector<float>& ctvector);      //for transforms
    bool getCameraHeight(float& height);            //for transforms
    bool getCameraPitch(float& pitch);              //for transforms
    bool getBodyPitch(float& pitch);
    Vector2<double> getCameraFOV() const {return Vector2<double>(camera_data->m_horizontalFov, camera_data->m_verticalFov);}
    
    //! @brief Returns a reference to the kinematics horizon line.
    const Horizon& getKinematicsHorizon();

    const LookUpTable& getLUT() const;
        
    //! Data publish interface
    void publish(const vector<const VisionFieldObject*>& visual_objects);
    void publish(const VisionFieldObject* visual_object);
    //void publish(DATA_ID id, vector<VisionObject> data);

    void debugRefresh();
    void debugPublish(vector<Ball> data);
    //void debugPublish(vector<Beacon> data);
    void debugPublish(vector<Goal> data);
    void debugPublish(DEBUG_ID id, const vector<Goal>& data);
    void debugPublish(int i, const vector<Goal>& d);
    void debugPublish(vector<Obstacle> data);
    void debugPublish(const vector<FieldLine>& data);
    void debugPublish(DEBUG_ID id, const vector<Point>& data_points);
    void debugPublish(DEBUG_ID id, const SegmentedRegion& region);
    void debugPublish(DEBUG_ID id) {}
    void debugPublish(DEBUG_ID id, const NUImage *const img) {}
    void debugPublish(DEBUG_ID id, const vector<LSFittedLine> &data);

    void plot(string name, vector< Vector2<double> > pts);

    //! Control interface       
private:    
    bool updateFrame();
    void postProcess();
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
    NUCameraData* camera_data;
    //NUSensorsData sensor_data_copy;
    NUActionatorsData* actions;             //! pointer to shared actionators data
    FieldObjects* field_objects;            //! pointer to shared fieldobject data
};

#endif // DATAWRAPPERDARWIN_H
