/**
*       @name   VisionBlackboard
*       @file   VisionBlackboard.h
*       @brief  Singleton blackboard for vision system
*       @author Shannon Fenn
*       @author David Budden
*       @date   14/02/12
*
*/

#include "Kinematics/Horizon.h"
#include "NUPlatform/NUCamera/NUCameraData.h"
#include "Tools/Math/Vector2.h"

#include "VisionWrapper/datawrappercurrent.h"
#include "VisionTools/lookuptable.h"
#include "basicvisiontypes.h"
#include "VisionTypes/coloursegment.h"
#include "VisionTypes/segmentedregion.h"
#include "VisionTypes/transition.h"
#include "VisionTypes/VisionFieldObjects/visionfieldobject.h"
#include "VisionTypes/VisionFieldObjects/ball.h"
#include "VisionTypes/VisionFieldObjects/goal.h"
#include "VisionTypes/VisionFieldObjects/beacon.h"
#include "VisionTypes/VisionFieldObjects/obstacle.h"
#include "VisionTypes/greenhorizon.h"

#ifndef VISIONBLACKBOARD_H
#define VISIONBLACKBOARD_H

using namespace std;

class VisionWrapper;

class VisionBlackboard
{
    friend class VisionController;

public:

    static VisionBlackboard* getInstance();

    //MUTATORS
    void setGreenHullPoints(const vector<PointType>& points);
    void setGreenHorizonScanPoints(const vector<PointType>& points);

    void setHorizontalScanlines(const vector<unsigned int>& scanlines);
    void setHorizontalSegments(const vector<vector<ColourSegment> >& segmented_scanlines);
    void setVerticalSegments(const vector<vector<ColourSegment> >& segmented_scanlines);
    void setHorizontalFilteredSegments(const vector<vector<ColourSegment> >& segmented_scanlines);
    void setVerticalFilteredSegments(const vector<vector<ColourSegment> >& segmented_scanlines);

    void setHorizontalTransitions(VisionFieldObject::VFO_ID vfo_id, const vector<Transition>& transitions);
    void setVerticalTransitions(VisionFieldObject::VFO_ID vfo_id, const vector<Transition>& transitions);
    void setHorizontalTransitionsMap(const map<VisionFieldObject::VFO_ID, vector<Transition> >& t_map);
    void setVerticalTransitionsMap(const map<VisionFieldObject::VFO_ID, vector<Transition> >& t_map);

    void setObjectPoints(const vector<PointType>& points);
    
    void addGoal(const Goal& newgoal);
    void addBeacon(const Beacon& newbeacon);
    void addBall(const Ball& newball);
    void addObstacle(const Obstacle& newobstacle);


    //ACCESSORS
//    const Mat* getOriginalImageMat() const;
    const NUImage& getOriginalImage() const;

    const GreenHorizon& getGreenHorizon() const;
    const vector<PointType>& getGreenHorizonScanPoints() const;

    const vector<unsigned int>& getHorizontalScanlines() const;
    
    const SegmentedRegion& getHorizontalSegmentedRegion() const;
    const SegmentedRegion& getVerticalSegmentedRegion() const;
    const SegmentedRegion& getHorizontalFilteredRegion() const;
    const SegmentedRegion& getVerticalFilteredRegion() const;

    const vector<Transition>& getHorizontalTransitions(VisionFieldObject::VFO_ID vfo_id);
    const vector<Transition>& getVerticalTransitions(VisionFieldObject::VFO_ID vfo_id);
    const map<VisionFieldObject::VFO_ID, vector<Transition> >& getHorizontalTransitionsMap() const;
    const map<VisionFieldObject::VFO_ID, vector<Transition> >& getVerticalTransitionsMap() const;
    
    const Horizon& getKinematicsHorizon() const;
    bool isCameraToGroundValid() const;
    const vector<float>& getCameraToGroundVector() const;
    bool isCameraTransformValid() const;
    const vector<float>& getCameraTransformVector() const;
    bool isCameraPitchValid() const;
    float getCameraPitch() const;
    bool isCameraHeightValid() const;
    float getCameraHeight() const;
    bool isBodyPitchValid() const;
    float getBodyPitch() const;

    const vector<PointType>& getObjectPoints() const;

    const LookUpTable& getLUT() const;
    
    
    Vector2<float> correctDistortion(const Vector2<float>& pt);
    double calculateBearing(double x) const;
    double calculateElevation(double y) const; 

    int getImageWidth() const;
    int getImageHeight() const;
    
    double getCameraDistanceInPixels() const;
    bool distanceToPoint(float bearing, float elevation, float& distance) const;

private:

    VisionBlackboard();
    ~VisionBlackboard();

    void updateLUT();
    void calculateFOVAndCamDist();
    void update();
    void publish() const;
    void debugPublish() const;
    
    void checkKinematicsHorizon();


    CameraSettings getCameraSettings() const;

private:
//! SELF
    static VisionBlackboard* instance;           //! @variable Singleton instance.

//! VARIABLES

    //! Image Data
    DataWrapper* wrapper;
//    Mat* original_image_cv;                 //! @variable Opencv mat for storing the original image 3 channels.
//    Mat* original_image_cv_4ch;             //! @variable Opencv mat for storing the original image 4 channels.
    const NUImage* original_image;                  //! @variable Image for storing the original image.
    
    NUCameraData m_camera_specs;
    
    Vector2<double> m_FOV;
    double effective_camera_dist_pixels;
    
    LookUpTable LUT;

    //vector<VFieldObject*> VFO_list;   //! @variable Vector of Vision Field Objects    
    
    //! Green Horizon data
    //vector<PointType> green_horizon_points;      //! @variable Vector of points forming the green horizon.
    GreenHorizon green_horizon;                  //! @variable The green horizon.
    vector<PointType> green_horizon_scan_points; //! @variable Vector of points used in green horizon scanning.
    
    //! Object data
    vector<PointType> object_points;   //! @variable Vector of points indicating potential objects.
    
    //! Kinematics Data
    Horizon kinematics_horizon; //! @variable Line defining kinematics horizon.
    vector<float> ctgvector;    //! @variable The camera to ground vector (for d2p).
    bool ctgvalid;              //! @variable Whether the ctgvector is valid.
    vector<float> ctvector;     //! @variable The camera transform vector.
    bool ctvalid;               //! @variable Whether the ctvector is valid.
    float camera_pitch;         //! @variable The camera pitch angle.
    bool camera_pitch_valid;    //! @variable Whether the camera pitch is valid.
    float camera_height;        //! @variable The height of the camera from the ground.
    bool camera_height_valid;   //! @variable Whether the camera height is valid.
    float body_pitch;           //! @variable The body pitch angle.
    bool body_pitch_valid;      //! @variable Whether the body pitch is valid.

    //! Scanline/Segmentation data
    vector<unsigned int> horizontal_scanlines;         //! @variable Vector of unsigned ints representing heights of horizontal scan lines.
    SegmentedRegion horizontal_segmented_scanlines;     //! @variable The segmented horizontal scanlines.
    SegmentedRegion vertical_segmented_scanlines;       //! @variable The segmented vertical scanlines.
    SegmentedRegion horizontal_filtered_segments;     //! @variable The filtered segmented horizontal scanlines.
    SegmentedRegion vertical_filtered_segments;       //! @variable The filtered segmented vertical scanlines.

    //! Transitions
    map<VisionFieldObject::VFO_ID, vector<Transition> > mapped_horizontal_transitions;
    map<VisionFieldObject::VFO_ID, vector<Transition> > mapped_vertical_transitions;
    //vector<Transition> horizontal_transitions;  //! @variable The transition rule matches in the horizontal segments.
    //vector<Transition> vertical_transitions;    //! @variable The transition rule matches in the vertical segments.
    
    vector<const VisionFieldObject*> m_vfos;
    vector<Goal> m_goals;
    vector<Beacon> m_beacons;
    vector<Ball> m_balls;
    vector<Obstacle> m_obstacles;
    
};

#endif // VISIONBLACKBOARD_H
