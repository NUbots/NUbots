/**
*       @name   VisionBlackboard
*       @file   VisionBlackboard.h
*       @brief  Singleton blackboard for vision system
*       @author Shannon Fenn
*       @author David Budden
*       @date   14/02/12
*
*/

#include "Tools/Math/Line.h"

#include "VisionWrapper/datawrappercurrent.h"
#include "VisionTools/lookuptable.h"

#include "basicvisiontypes.h"
#include "VisionTypes/coloursegment.h"
#include "VisionTypes/segmentedregion.h"
#include "VisionTypes/transition.h"
#include "VisionTypes/visionfieldobject.h"

#ifndef VISIONBLACKBOARD_H
#define VISIONBLACKBOARD_H

using namespace cv;
using namespace std;

class VisionWrapper;

class VisionBlackboard
{
    friend class VisionController;

public:

    static VisionBlackboard* getInstance();

    //MUTATORS
    void setHullPoints(const vector<PointType>& points);
    void setHorizonScanPoints(const vector<PointType>& points);

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


    //ACCESSORS
//    const Mat* getOriginalImageMat() const;
    const NUImage& getOriginalImage() const;

    const vector<PointType>& getHorizonPoints() const;
    const vector<PointType>& getHorizonScanPoints() const;

    const vector<unsigned int>& getHorizontalScanlines() const;
    
    const SegmentedRegion& getHorizontalSegmentedRegion() const;
    const SegmentedRegion& getVerticalSegmentedRegion() const;
    const SegmentedRegion& getHorizontalFilteredRegion() const;
    const SegmentedRegion& getVerticalFilteredRegion() const;

    const vector<Transition>& getHorizontalTransitions(VisionFieldObject::VFO_ID vfo_id);
    const vector<Transition>& getVerticalTransitions(VisionFieldObject::VFO_ID vfo_id);
    const map<VisionFieldObject::VFO_ID, vector<Transition> >& getHorizontalTransitionsMap() const;
    const map<VisionFieldObject::VFO_ID, vector<Transition> >& getVerticalTransitionsMap() const;
    
    const Line& getKinematicsHorizon() const;

    const vector<PointType>& getObjectPoints() const;


    const LookUpTable& getLUT() const;


    unsigned int getHorizonYFromX(unsigned int x) const;

    int getImageWidth() const;
    int getImageHeight() const;

private:

    VisionBlackboard();
    ~VisionBlackboard();

    void updateLUT();
    void update();
    void publish() const;
    void debugPublish() const;


    CameraSettings getCameraSettings() const;

private:
//! SELF
    static VisionBlackboard* instance;           //! @variable Singleton instance

//! VARIABLES

    //! Image Data
    DataWrapper* wrapper;
//    Mat* original_image_cv;                 //! @variable Opencv mat for storing the original image 3 channels
//    Mat* original_image_cv_4ch;             //! @variable Opencv mat for storing the original image 4 channels
    NUImage* original_image;                  //! @variable Image for storing the original image

    LookUpTable LUT;

    //vector<VFieldObject*> VFO_list;   //! @variable Vector of Vision Field Objects    
    
    //! Green Horizon data
    vector<PointType> horizon_points;      //! @variable Vector of points forming the green horizon
    vector<PointType> horizon_scan_points; //! @variable Vector of points used in green horizon scanning
    
    //! Object data
    vector<PointType> object_points;   //! @variable Vector of points indicating potential objects
    
    //! Kinematics Data
    Line kinematics_horizon;    //! @variable Line defining kinematics horizon
    vector<float> ctgvector;    //! @variable The camera to ground vector (for d2p)
    bool ctgvalid;              //! @variable Whether the ctgvector is valid

    //! Scanline/Segmentation data    
    vector<unsigned int> horizontal_scanlines;         //! @variable Vector of unsigned ints representing heights of horizontal scan lines
    SegmentedRegion horizontal_segmented_scanlines;     //! @variable The segmented horizontal scanlines
    SegmentedRegion vertical_segmented_scanlines;       //! @variable The segmented vertical scanlines
    SegmentedRegion horizontal_filtered_segments;     //! @variable The filtered segmented horizontal scanlines
    SegmentedRegion vertical_filtered_segments;       //! @variable The filtered segmented vertical scanlines

    //! Transitions
    map<VisionFieldObject::VFO_ID, vector<Transition> > mapped_horizontal_transitions;
    map<VisionFieldObject::VFO_ID, vector<Transition> > mapped_vertical_transitions;
    //vector<Transition> horizontal_transitions;  //! @variable The transition rule matches in the horizontal segments
    //vector<Transition> vertical_transitions;    //! @variable The transition rule matches in the vertical segments
};

#endif // VISIONBLACKBOARD_H
