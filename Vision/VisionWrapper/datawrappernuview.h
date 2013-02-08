#ifndef VISIONDATAWRAPPERNUVIEW_H
#define VISIONDATAWRAPPERNUVIEW_H


#include <iostream>
#include <fstream>

#include "Kinematics/Horizon.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
//#include "Infrastructure/Jobs/JobList.h"
//#include "Infrastructure/Jobs/VisionJobs/SaveImagesJob.h"

#include "Vision/basicvisiontypes.h"
#include "Vision/VisionTypes/segmentedregion.h"
#include "Vision/VisionTools/pccamera.h"
#include "Vision/VisionTools/lookuptable.h"
#include "Vision/VisionTypes/VisionFieldObjects/ball.h"
//#include "Vision/VisionTypes/VisionFieldObjects/beacon.h"
#include "Vision/VisionTypes/VisionFieldObjects/goal.h"
#include "Vision/VisionTypes/VisionFieldObjects/obstacle.h"
#include "Vision/VisionTypes/VisionFieldObjects/fieldline.h"
#include "Infrastructure/NUImage/ClassifiedImage.h"

//for virtualNUbot/Qt
#include "GLDisplay.h"
#include <QObject>

using namespace std;
//using namespace cv;

class virtualNUbot;

class DataWrapper : public QObject
{
    friend class VisionController;
    friend class VisionControlWrapper;
    friend class virtualNUbot;

public:
    static DataWrapper* getInstance();

    //! RETRIEVAL METHODS
    const NUImage* getFrame();

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
    void debugPublish(vector<Ball> data);
    //bool debugPublish(vector<Beacon> data);
    void debugPublish(vector<Goal> data);
    void debugPublish(vector<Obstacle> data);
    void debugPublish(const vector<FieldLine>& data);
    void debugPublish(DEBUG_ID id, const vector<Point>& data_points);
    void debugPublish(DEBUG_ID id, const SegmentedRegion& region);
    void debugPublish(DEBUG_ID id, const NUImage *const img);
    void debugPublish(DEBUG_ID id, const vector<LSFittedLine> &data);

    void plotPoints(const vector<Point>& pts, string name);
    
private:
    DataWrapper();
    ~DataWrapper();
    //void startImageFileGroup(string filename);
    bool updateFrame();
    void postProcess();
    bool loadLUTFromFile(const string& fileName);
    int getNumFramesDropped() const {return numFramesDropped;}      //! @brief Returns the number of dropped frames since start.
    int getNumFramesProcessed() const {return numFramesProcessed;}  //! @brief Returns the number of processed frames since start.
    void saveAnImage();
    
    //for virtualnubot
    void setRawImage(const NUImage *image);
    void setSensorData(NUSensorsData* sensors);
    void setFieldObjects(FieldObjects* fieldObjects);
    void setLUT(unsigned char* vals);
    void classifyImage(ClassifiedImage &target) const;
    void classifyPreviewImage(ClassifiedImage &target,unsigned char* temp_vals) const;
    
signals:
    void pointsUpdated(vector<Point> pts, GLDisplay::display);
    void segmentsUpdated(SegmentedRegion region, GLDisplay::display);
    void linesUpdated(vector<LSFittedLine> lines, GLDisplay::display);
    void plotUpdated(vector<Point> pts, GLDisplay::display);
    
private:

    static DataWrapper* instance;

    const NUImage* m_current_image;

    string LUTname;
    LookUpTable LUT;

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
    NUSensorsData* sensor_data;             //! pointer to shared sensor data
    NUActionatorsData* actions;             //! pointer to shared actionators data
    FieldObjects* field_objects;            //! pointer to shared fieldobject data

};

#endif // VISIONDATAWRAPPERNUVIEW_H
