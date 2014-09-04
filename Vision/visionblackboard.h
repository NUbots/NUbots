/**
*       @name   VisionBlackboard
*       @file   VisionBlackboard.h
*       @brief  Singleton blackboard for vision system
*       @author Shannon Fenn
*       @author David Budden
*       @date   14/02/12
*
*/

#include <map>

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
#include "VisionTypes/VisionFieldObjects/centrecircle.h"
#include "VisionTypes/VisionFieldObjects/cornerpoint.h"
#include "VisionTypes/greenhorizon.h"

#ifndef VISIONBLACKBOARD_H
#define VISIONBLACKBOARD_H

using std::map;
using std::vector;
using namespace Vision;

class VisionWrapper;

class VisionBlackboard
{
    friend class VisionController;
    friend class RobocupHacks;

public:

    static VisionBlackboard* getInstance();

    //MUTATORS
    void setGreenHullPoints(const std::vector<Vector2<double> >& points);
    void setGreenHorizonScanPoints(const std::vector< Vector2<double> >& points);

    void setHorizontalScanlines(const std::vector<int> &scanlines);
    void setHorizontalSegments(const std::vector<std::vector<ColourSegment> >& segmented_scanlines);
    void setVerticalSegments(const std::vector<std::vector<ColourSegment> >& segmented_scanlines);
    void setHorizontalFilteredSegments(const std::vector<std::vector<ColourSegment> >& segmented_scanlines);
    void setVerticalFilteredSegments(const std::vector<std::vector<ColourSegment> >& segmented_scanlines);

    void setHorizontalTransitions(COLOUR_CLASS colour_class, const std::vector<ColourSegment>& transitions);
    void setVerticalTransitions(COLOUR_CLASS colour_class, const std::vector<ColourSegment>& transitions);
    void setHorizontalTransitionsMap(const map<COLOUR_CLASS, std::vector<ColourSegment> >& t_map);
    void setVerticalTransitionsMap(const map<COLOUR_CLASS, std::vector<ColourSegment> >& t_map);

    void setObstaclePoints(const std::vector<Point> &points);
    
    void addGoal(const Goal& newgoal);
    void addBall(const Ball& newball);
    void addObstacle(const Obstacle& newobstacle);
    void addLine(const FieldLine& newline);
    void addCentreCircle(const CentreCircle& newcircle);
    void addCornerPoint(const CornerPoint& newcorner);

    void addGoals(const std::vector<Goal>& newgoals);
    void addBalls(const std::vector<Ball>& newballs);
    void addObstacles(const std::vector<Obstacle>& newobstacles);
    void addLines(const std::vector<FieldLine>& newlines);
    void addCentreCircles(const std::vector<CentreCircle>& newcircles);
    void addCornerPoints(const std::vector<CornerPoint>& newcorners);


    //ACCESSORS
    const NUImage& getOriginalImage() const;

    const GreenHorizon& getGreenHorizon() const;
    const std::vector<Vector2<double> >& getGreenHorizonScanPoints() const;

    const std::vector<int> &getHorizontalScanlines() const;
    
    const SegmentedRegion& getHorizontalSegmentedRegion() const;
    const SegmentedRegion& getVerticalSegmentedRegion() const;
    const SegmentedRegion& getHorizontalFilteredRegion() const;
    const SegmentedRegion& getVerticalFilteredRegion() const;

    const std::vector<ColourSegment>& getHorizontalTransitions(COLOUR_CLASS colour_class);
    const std::vector<ColourSegment>& getVerticalTransitions(COLOUR_CLASS colour_class);
    std::vector<ColourSegment> getAllTransitions(COLOUR_CLASS colour_class);
    const map<COLOUR_CLASS, std::vector<ColourSegment> >& getHorizontalTransitionsMap() const;
    const map<COLOUR_CLASS, std::vector<ColourSegment> >& getVerticalTransitionsMap() const;
    
    const Horizon& getKinematicsHorizon() const;
    const Transformer& getTransformer() const;

    const std::vector<Ball>& getBalls();
    const std::vector<Goal>& getGoals();
    const std::vector<Obstacle>& getObstacles();
    const std::vector<FieldLine>& getLines();
    const std::vector<CentreCircle>& getCentreCircles();
    const std::vector<CornerPoint>& getCorners();

    const std::vector<Point>& getObstaclePoints() const;

    const LookUpTable& getLUT() const;

    int getImageWidth() const;
    int getImageHeight() const;
    Vector2<double> getFOV() const;
    
    double getCameraDistanceInPixels() const;

private:
    VisionBlackboard();
    ~VisionBlackboard();

    void calculateFOVAndCamDist();
    void update();
    void publish() const;
    void debugPublish() const;
    
    void checkKinematicsHorizon();

    CameraSettings getCameraSettings() const;

private:
//! SELF
    static VisionBlackboard* instance;           //! @variable Singleton instance.

    //! Image Data
    DataWrapper* wrapper;
    const NUImage* original_image;                  //! @variable Image for storing the original image.

    LookUpTable LUT;
    Transformer m_transformer;
    
    //! Green Horizon data
    GreenHorizon m_green_horizon;   //! @variable The green horizon.
    std::vector<Point> gh_scan_points;   //! @variable The location of initial green (used to form GH and find obstacles)
    std::vector<Point> obstacle_points;    //! @variable The std::list of points used by obstacle detection
    
    //! Kinematics Data
    Horizon kinematics_horizon; //! @variable Line defining kinematics horizon.

    //! Scanline/Segmentation data
    std::vector<int> horizontal_scanlines;                   //! @variable Vector of unsigned ints representing heights of horizontal scan lines.
    SegmentedRegion horizontal_segmented_scanlines;     //! @variable The segmented horizontal scanlines.
    SegmentedRegion vertical_segmented_scanlines;       //! @variable The segmented vertical scanlines.
    SegmentedRegion horizontal_filtered_segments;       //! @variable The filtered segmented horizontal scanlines.
    SegmentedRegion vertical_filtered_segments;         //! @variable The filtered segmented vertical scanlines.

    //! Transitions
    std::map<COLOUR_CLASS, std::vector<ColourSegment> > matched_horizontal_segments;
    std::map<COLOUR_CLASS, std::vector<ColourSegment> > matched_vertical_segments;
    
    std::vector<const VisionFieldObject*> m_vfos;
    std::vector<Goal> m_goals;
    std::vector<Ball> m_balls;
    std::vector<Obstacle> m_obstacles;
    std::vector<FieldLine> m_lines;
    std::vector<CentreCircle> m_centre_circles;
    std::vector<CornerPoint> m_corner_points;
    
};

#endif // VISIONBLACKBOARD_H
