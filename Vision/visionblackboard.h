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
#include "VisionTools/transformer.h"
#include "basicvisiontypes.h"
#include "VisionTypes/coloursegment.h"
#include "VisionTypes/segmentedregion.h"
#include "VisionTypes/VisionFieldObjects/visionfieldobject.h"
#include "VisionTypes/VisionFieldObjects/ball.h"
#include "VisionTypes/VisionFieldObjects/goal.h"
#include "VisionTypes/VisionFieldObjects/beacon.h"
#include "VisionTypes/VisionFieldObjects/obstacle.h"
#include "VisionTypes/VisionFieldObjects/fieldline.h"
#include "VisionTypes/greenhorizon.h"

#ifndef VISIONBLACKBOARD_H
#define VISIONBLACKBOARD_H

using namespace std;
using namespace Vision;

class VisionWrapper;

class VisionBlackboard
{
    friend class VisionController;

public:

    static VisionBlackboard* getInstance();

    //MUTATORS
    void setGreenHullPoints(const vector<Vector2<double> >& points);
    void setGreenHorizonScanPoints(const vector<Point> &points);

    void setHorizontalScanlines(const vector<int> &scanlines);
    void setHorizontalSegments(const vector<vector<ColourSegment> >& segmented_scanlines);
    void setVerticalSegments(const vector<vector<ColourSegment> >& segmented_scanlines);
    void setHorizontalFilteredSegments(const vector<vector<ColourSegment> >& segmented_scanlines);
    void setVerticalFilteredSegments(const vector<vector<ColourSegment> >& segmented_scanlines);

    void setHorizontalTransitions(COLOUR_CLASS colour_class, const vector<ColourSegment>& transitions);
    void setVerticalTransitions(COLOUR_CLASS colour_class, const vector<ColourSegment>& transitions);
    void setHorizontalTransitionsMap(const map<COLOUR_CLASS, vector<ColourSegment> >& t_map);
    void setVerticalTransitionsMap(const map<COLOUR_CLASS, vector<ColourSegment> >& t_map);

    void setObstaclePoints(const vector<Point> &points);
    
    void addGoal(const Goal& newgoal);
    //void addBeacon(const Beacon& newbeacon);
    void addBall(const Ball& newball);
    void addObstacle(const Obstacle& newobstacle);
    void addLine(const FieldLine& newline);

    void addGoals(const vector<Goal>& newgoals);
    //void addBeacons(const vector<Beacon>& newbeacons);
    void addBalls(const vector<Ball>& newballs);
    void addObstacles(const vector<Obstacle>& newobstacles);
    void addLines(const vector<FieldLine>& newlines);


    //ACCESSORS
//    const Mat* getOriginalImageMat() const;
    const NUImage& getOriginalImage() const;

    const GreenHorizon& getGreenHorizon() const;
    const vector<Vector2<double> >& getGreenHorizonScanPoints() const;

    const vector<int> &getHorizontalScanlines() const;
    
    const SegmentedRegion& getHorizontalSegmentedRegion() const;
    const SegmentedRegion& getVerticalSegmentedRegion() const;
    const SegmentedRegion& getHorizontalFilteredRegion() const;
    const SegmentedRegion& getVerticalFilteredRegion() const;

    const vector<ColourSegment>& getHorizontalTransitions(COLOUR_CLASS colour_class);
    const vector<ColourSegment>& getVerticalTransitions(COLOUR_CLASS colour_class);
    const map<COLOUR_CLASS, vector<ColourSegment> >& getHorizontalTransitionsMap() const;
    const map<COLOUR_CLASS, vector<ColourSegment> >& getVerticalTransitionsMap() const;
    
    const Horizon& getKinematicsHorizon() const;
    const Transformer& getTransformer() const;

//    bool isCameraPitchValid() const;
//    float getCameraPitch() const;
//    bool isCameraHeightValid() const;
//    float getCameraHeight() const;
//    bool isBodyPitchValid() const;
//    float getBodyPitch() const;

    vector<Ball>& getBalls();
    vector<Goal>& getGoals();
    //vector<Beacon>& getBeacons();
    vector<Obstacle>& getObstacles();
    vector<FieldLine>& getLines();

    const vector<Vector2<double> >& getObstaclePoints() const;

    const LookUpTable& getLUT() const;

    int getImageWidth() const;
    int getImageHeight() const;
    Vector2<double> getFOV() const;
    
    double getCameraDistanceInPixels() const;

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

    LookUpTable LUT;
    
    //! Green Horizon data
    GreenHorizon m_green_horizon;   //! @variable The green horizon.
    vector<Point> gh_scan_points;   //! @variable The location of initial green (used to form GH and find obstacles)
    vector<Point> obstacle_points;    //! @variable The list of points used by obstacle detection

    Transformer m_transformer;

    //! Object data
    //vector<Vector2<double> > object_points;   //! @variable Vector of points indicating potential objects.
    
    //! Kinematics Data
    Horizon kinematics_horizon; //! @variable Line defining kinematics horizon.

    //! Scanline/Segmentation data
    vector<int> horizontal_scanlines;                   //! @variable Vector of unsigned ints representing heights of horizontal scan lines.
    SegmentedRegion horizontal_segmented_scanlines;     //! @variable The segmented horizontal scanlines.
    SegmentedRegion vertical_segmented_scanlines;       //! @variable The segmented vertical scanlines.
    SegmentedRegion horizontal_filtered_segments;       //! @variable The filtered segmented horizontal scanlines.
    SegmentedRegion vertical_filtered_segments;         //! @variable The filtered segmented vertical scanlines.

    //! Transitions
    map<COLOUR_CLASS, vector<ColourSegment> > matched_horizontal_segments;
    map<COLOUR_CLASS, vector<ColourSegment> > matched_vertical_segments;
    //vector<Transition> horizontal_transitions;  //! @variable The transition rule matches in the horizontal segments.
    //vector<Transition> vertical_transitions;    //! @variable The transition rule matches in the vertical segments.
    
    vector<const VisionFieldObject*> m_vfos;
    vector<Goal> m_goals;
    //vector<Beacon> m_beacons;
    vector<Ball> m_balls;
    vector<Obstacle> m_obstacles;
    vector<FieldLine> m_lines;
    
};

#endif // VISIONBLACKBOARD_H
