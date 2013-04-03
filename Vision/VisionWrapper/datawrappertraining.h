#ifndef DATAWRAPPERTRAINING_H
#define DATAWRAPPERTRAINING_H

#include <iostream>
#include <fstream>

#include "Kinematics/Horizon.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "NUPlatform/NUCamera/NUCameraData.h"

#include "Vision/basicvisiontypes.h"
#include "Vision/VisionTypes/segmentedregion.h"
#include "Vision/VisionTools/pccamera.h"
#include "Vision/VisionTools/lookuptable.h"
#include "Vision/VisionTools/transformer.h"
#include "Vision/VisionTypes/groundpoint.h"
#include "Vision/VisionTypes/histogram1d.h"
#include "Vision/VisionTypes/VisionFieldObjects/ball.h"
#include "Vision/VisionTypes/VisionFieldObjects/goal.h"
#include "Vision/VisionTypes/VisionFieldObjects/obstacle.h"
#include "Vision/VisionTypes/VisionFieldObjects/fieldline.h"
#include "Vision/VisionTypes/VisionFieldObjects/centrecircle.h"
#include "Vision/VisionTypes/VisionFieldObjects/cornerpoint.h"
#include "Tools/Math/LSFittedLine.h"

#include <QImage>

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
    bool getCameraYaw(float& yaw);
    bool getBodyPitch(float& pitch);
    Vector2<double> getCameraFOV() const;

    //! @brief Generates spoofed horizon line.
    const Horizon& getKinematicsHorizon();

    CameraSettings getCameraSettings();

    const LookUpTable& getLUT() const;

    //! PUBLISH METHODS
    void publish(const vector<const VisionFieldObject*> &visual_objects);
    void publish(const VisionFieldObject* visual_object);

    void debugPublish(const vector<Ball>& data);
    void debugPublish(const vector<Goal>& data);
    void debugPublish(const vector<Obstacle>& data);
    void debugPublish(const vector<FieldLine>& data);
    void debugPublish(const vector<CentreCircle>& data);
    void debugPublish(const vector<CornerPoint>& data);
    bool debugPublish(DEBUG_ID id, const vector<Point>& data_points) {return false;}
    bool debugPublish(DEBUG_ID id, const SegmentedRegion& region) {return false;}
    bool debugPublish(DEBUG_ID id, const NUImage* const img) {return false;}
    void debugPublish(DEBUG_ID id);
    void debugPublish(DEBUG_ID id, const vector<LSFittedLine>& data) {}
    void debugPublish(DEBUG_ID id, const vector<Goal>& data) {}


    void plotCurve(string name, vector< Point > pts) {}
    void plotLineSegments(string name, vector< Point > pts) {}
    void plotHistogram(string name, const Histogram1D& hist, Colour colour = yellow) {}

private:
    DataWrapper();
    ~DataWrapper();
    bool updateFrame();
    void updateFrame(NUImage& img, NUSensorsData& sensors);
    int getNumFramesProcessed() const {return numFramesProcessed;}  //! @brief Returns the number of processed frames since start.

    void resetHistory();
    void resetDetections();
    void printHistory(ostream& out);
    bool setImageStream(const string& filename);
    bool setSensorStream(const string &filename);
    bool loadLUTFromFile(const string& filename);
    void resetStream();
    
    void printLabels(ostream& out) const;
    bool readLabels(istream& in, vector< vector<VisionFieldObject*> >& labels) const;
    //bool readLabels(istream& in, vector< vector< pair<VFO_ID, Vector2<double> > > >& labels) const;

    bool renderFrame(QImage &img, bool lines_only=false);
private:
    static DataWrapper* instance;   //! @var static singleton instance

    bool valid;

    NUImage m_current_image;       //! @var The current image pointer
    NUSensorsData m_current_sensors;

    string LUTname;                 //! @var look up table filename
    LookUpTable LUT;                //! @var look up table

    Horizon kinematics_horizon;     //! @var the kinematics horizon - represents "level"

    string image_stream_name;       //! @var filename for the image stream
    ifstream imagestrm;             //! @var image stream
    string sensor_stream_name;
    ifstream sensorstrm;

    int numFramesProcessed;         //! @var the number of frames processed so far

    NUCameraData m_camspecs;
    Transformer transformer;

    //! Detection info
    vector<Ball> ball_detections;           //! @var balls detected in this frame
    vector<Goal> goal_detections;           //! @var goals
    vector<Obstacle> obstacle_detections;   //! @var obstacles
    vector<FieldLine> line_detections;      //! @var lines
    vector<CentreCircle> centrecircle_detections;      //! @var lines
    vector<CornerPoint> corner_detections;      //! @var lines

    vector<vector<Ball> > ball_detection_history;           //! @var balls detected in each frame so far
    vector<vector<Goal> > goal_detection_history;           //! @var goals
    vector<vector<Obstacle> > obstacle_detection_history;   //! @var obstacles
    vector<vector<FieldLine> > line_detection_history;      //! @var lines
    vector<vector<CentreCircle> > centrecircle_detection_history;      //! @var lines
    vector<vector<CornerPoint> > corner_detection_history;      //! @var lines

    vector<const VisionFieldObject*> detections;    //! @var all field objects detected in this frame
};

#endif // DATAWRAPPERTRAINING_H
