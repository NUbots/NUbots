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
#include "Vision/VisionTypes/VisionFieldObjects/fieldline.h"
#include "Tools/Math/LSFittedLine.h"

#define GROUP_NAME "/home/shannon/Images/paper"
#define GROUP_EXT ".png"

class DataWrapper
{
    friend class VisionController;
    friend class VisionControlWrapper;

public:

    //! Represents types of debugging information
    enum DEBUG_ID {
        DBID_IMAGE              =0,
        DID_CLASSED_IMAGE       =1,
        DBID_H_SCANS            =2,
        DBID_V_SCANS            =3,
        DBID_SEGMENTS           =4,
        DBID_MATCHED_SEGMENTS   =5,
        DBID_HORIZON            =6,
        DBID_GREENHORIZON_SCANS =7,
        DBID_GREENHORIZON_FINAL =8,
        DBID_OBJECT_POINTS      =9,
        DBID_FILTERED_SEGMENTS  =10,
        DBID_GOALS              =11,
        DBID_BEACONS            =12,
        DBID_BALLS              =13,
        DBID_OBSTACLES          =14,
        DBID_LINES              =15,
        NUMBER_OF_IDS           =16
    };

    static string getIDName(DEBUG_ID id);

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
    //bool debugPublish(const vector<Beacon>& data);
    bool debugPublish(const vector<Goal>& data);
    bool debugPublish(const vector<Obstacle>& data);
    bool debugPublish(const vector<FieldLine>& data);
    bool debugPublish(DEBUG_ID id, const vector<PointType>& data_points) {return id<NUMBER_OF_IDS && !data_points.empty();}
    bool debugPublish(DEBUG_ID id, const SegmentedRegion& region) {return id<NUMBER_OF_IDS && !region.empty();}
    bool debugPublish(DEBUG_ID id, const NUImage* const img) {return id<NUMBER_OF_IDS;}


private:
    DataWrapper();
    ~DataWrapper();
    bool updateFrame();
    void updateFrame(NUImage& img);
    int getNumFramesProcessed() const {return numFramesProcessed;}  //! @brief Returns the number of processed frames since start.

    void resetHistory();
    void resetDetections();
    void printHistory(ostream& out);
    bool setImageStream(const string& filename);
    bool loadLUTFromFile(const string& filename);
    void resetStream();
    
    void printLabels(ostream& out) const;
    bool readLabels(istream& in, vector< vector<VisionFieldObject*> >& labels) const;
    bool readLabels(istream& in, vector< vector< pair<VisionFieldObject::VFO_ID, Vector2<double> > > >& labels) const;

    bool renderFrame(cv::Mat &mat, bool lines_only=false);
private:
    static DataWrapper* instance;   //! @var static singleton instance

    NUImage* m_current_image;       //! @var The current image pointer

    string LUTname;                 //! @var look up table filename
    LookUpTable LUT;                //! @var look up table

    Horizon kinematics_horizon;     //! @var the kinematics horizon - represents "level"

    string image_stream_name;       //! @var filename for the image stream
    ifstream imagestrm;             //! @var image stream

    int numFramesProcessed;         //! @var the number of frames processed so far

    //! Detection info
    vector<Ball> ball_detections;           //! @var balls detected in this frame
    vector<Goal> goal_detections;           //! @var goals
    vector<Beacon> beacon_detections;       //! @var beacons
    vector<Obstacle> obstacle_detections;   //! @var obstacles
    vector<FieldLine> line_detections;      //! @var lines

    vector<vector<Ball> > ball_detection_history;           //! @var balls detected in each frame so far
    vector<vector<Goal> > goal_detection_history;           //! @var goals
    vector<vector<Beacon> > beacon_detection_history;       //! @var beacons
    vector<vector<Obstacle> > obstacle_detection_history;   //! @var obstacles
    vector<vector<FieldLine> > line_detection_history;      //! @var lines

    vector<const VisionFieldObject*> detections;    //! @var all field objects detected in this frame
};

#endif // DATAWRAPPERTRAINING_H
