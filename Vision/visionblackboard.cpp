#include "visionblackboard.h"
#include "VisionWrapper/datawrappercurrent.h"
#include "debug.h"

#include <algorithm>
#include <boost/foreach.hpp>

VisionBlackboard* VisionBlackboard::instance = 0;

void getPointsAndColoursFromSegments(const vector< vector<ColourSegment> >& segments, vector<Scalar>& colours, vector<PointType>& pts)
{
    unsigned char r, g, b;
    
    BOOST_FOREACH(const vector<ColourSegment>& line, segments) {
        BOOST_FOREACH(const ColourSegment& seg, line) {
            ClassIndex::getColourAsRGB(seg.getColour(), r, g, b);
            pts.push_back(seg.getStart());
            pts.push_back(seg.getEnd());
            colours.push_back(Scalar(b,g,r));
        }
    }
}

//! @brief Private constructor for blackboard.
VisionBlackboard::VisionBlackboard()
{
    wrapper = DataWrapper::getInstance();

    LUT = wrapper->getLUT();
    
    //Get Image
    original_image = wrapper->getFrame();

    //get Camera to ground vector
    ctgvalid = wrapper->getCTGVector(ctgvector);
}

/** @brief Private destructor.
*
*   To be called implicitly at program close or by the controller as the VBB is live for the
*   whole program in absence of errors.
*/
VisionBlackboard::~VisionBlackboard()
{
    //delete original_image_cv_4ch;
    //delete original_image_cv;
}

/**
*   @brief return unique instance of blackboard - lazy initialisation.
*/
VisionBlackboard* VisionBlackboard::getInstance()
{
    if (!instance)
        instance = new VisionBlackboard();
    return instance;
}

/**
*   @brief sets the greenhorizon hull point set.
*   @param points A vector of pixel locations for the GH hull.
*
*   Clears the previous list of point pointers and copies the new list.
*/
void VisionBlackboard::setHullPoints(const vector<PointType>& points)
{
    horizon_points = points;
}

/**
*   @brief sets the green horizon scan point set.
*   @param points A vector of pixel locations for the green horizon scan.
*
*   Clears the previous list of point pointers and copies the new list.
*/
void VisionBlackboard::setHorizonScanPoints(const vector<PointType>& points)
{
    horizon_scan_points = points;
}

/**
*   @brief sets the object point set.
*   @param points A vector of pixel locations for objects.
*
*   Clears the previous list of point pointers and copies the new list.
*/
void VisionBlackboard::setObjectPoints(const vector<PointType>& points)
{
    object_points = points;
}

/**
*   @brief sets the horizontal scan lines.
*   @param horizontal_scanlines A vector of unsigned ints defining horizontal scanlines.
*
*   Clears the previous list of point pointers and copies the new list.
*/
void VisionBlackboard::setHorizontalScanlines(const vector<unsigned int>& scanlines)
{
    horizontal_scanlines = scanlines;
}

/**
*   @brief sets the horizontal segments.
*   @param segmented_scanlines A vector of vectors of colour segments.
*/
void VisionBlackboard::setHorizontalSegments(const vector<vector<ColourSegment> >& segmented_scanlines)
{
    horizontal_segmented_scanlines.set(segmented_scanlines, HORIZONTAL);
}

/**
*   @brief sets the vertical segments.
*   @param segmented_scanlines A vector of vectors of colour segments.
*/
void VisionBlackboard::setVerticalSegments(const vector<vector<ColourSegment> >& segmented_scanlines)
{
    vertical_segmented_scanlines.set(segmented_scanlines, VERTICAL);
}

/**
*   @brief sets the filtered horizontal segments.
*   @param segmented_scanlines A vector of vectors of colour segments.
*/
void VisionBlackboard::setHorizontalFilteredSegments(const vector<vector<ColourSegment> >& segmented_scanlines)
{
    horizontal_filtered_segments.set(segmented_scanlines, HORIZONTAL);
}

/**
*   @brief sets the filtered vertical segments.
*   @param segmented_scanlines A vector of vectors of colour segments.
*/
void VisionBlackboard::setVerticalFilteredSegments(const vector<vector<ColourSegment> >& segmented_scanlines)
{
    vertical_filtered_segments.set(segmented_scanlines, VERTICAL);
}

/**
*   @brief sets the vertical transition rule matches for the given vision field object.
*   @param vfo_if The identifier of the field object
*   @param transitions A vector of transitions that matched the horizontal rules.
*/
void VisionBlackboard::setHorizontalTransitions(VisionFieldObject::VFO_ID vfo_id, const vector<Transition> &transitions)
{
    mapped_horizontal_transitions[vfo_id] = transitions;
}

/**
*   @brief sets the vertical transition rule matches for the given vision field object.
*   @param vfo_if The identifier of the field object
*   @param transitions A vector of transitions that matched the vertical rules.
*/
void VisionBlackboard::setVerticalTransitions(VisionFieldObject::VFO_ID vfo_id, const vector<Transition> &transitions)
{
    mapped_vertical_transitions[vfo_id] = transitions;
}

/**
*   @brief sets the horizontal transition rule matches for all vision field objects.
*   @param t_map A map from VFO_IDs to transitions that matched the horizontal rules.
*/
void VisionBlackboard::setHorizontalTransitionsMap(const map<VisionFieldObject::VFO_ID, vector<Transition> > &t_map)
{
    mapped_horizontal_transitions = t_map;
}

/**
*   @brief sets the vertical transition rule matches for all vision field objects.
*   @param t_map A map from VFO_IDs to transitions that matched the vertical rules.
*/
void VisionBlackboard::setVerticalTransitionsMap(const map<VisionFieldObject::VFO_ID, vector<Transition> > &t_map)
{
    mapped_vertical_transitions = t_map;
}

/**
*   @brief returns the kinematics horizon data set.
*   @return A vector of values defining horizon line.
*/
const vector<PointType>& VisionBlackboard::getHorizonPoints() const
{
    return horizon_points;
}

/**
*   @brief returns the green horizon scan point set.
*   @return points A vector of pixel locations for the green horizon scan.
*/
const vector<PointType>& VisionBlackboard::getHorizonScanPoints() const
{
    return horizon_scan_points;
}

/**
*   @brief returns the object point set.
*   @return points A vector of pixel locations for objects.
*/
const vector<PointType>& VisionBlackboard::getObjectPoints() const
{
    return object_points;
}

/**
*   @brief returns a pointer to the Lookup Table.
*   @return A pointer to the LUT.
*/
const LookUpTable& VisionBlackboard::getLUT() const
{
    return LUT;
}

/**
*   @brief returns the kinematics horizon line.
*   @return kinematics_horizon A Line defining the kinematics horizon.
*/
const Line& VisionBlackboard::getKinematicsHorizon() const
{
    return kinematics_horizon;
}

/**
*   @brief returns the set of heights for horizontal scan lines.
*   @return horizontal_scanlines A vector of unsigned ints defining horizontal scanlines.
*/
const vector<unsigned int>& VisionBlackboard::getHorizontalScanlines() const
{
    return horizontal_scanlines;
}

/**
*   @brief returns the classified horizontal segments.
*   @return horizontal_segmented_scanlines A SegmentedRegion - essentially a vector of vectors of colour segments.
*/
const SegmentedRegion& VisionBlackboard::getHorizontalSegmentedRegion() const
{
    return horizontal_segmented_scanlines;
}

/**
*   @brief returns the classified vertical segments
*   @return vertical_segmented_scanlines A SegmentedRegion - essentially a vector of vectors of colour segments.
*/
const SegmentedRegion& VisionBlackboard::getVerticalSegmentedRegion() const
{
    return vertical_segmented_scanlines;
}

/**
*   @brief returns the filtered horizontal segments.
*   @return horizontal_filtered_segments A SegmentedRegion - essentially a vector of vectors of colour segments.
*/
const SegmentedRegion& VisionBlackboard::getHorizontalFilteredRegion() const
{
    return horizontal_filtered_segments;
}

/**
*   @brief returns the filtered vertical segments
*   @return vertical_filtered_segments A SegmentedRegion - essentially a vector of vectors of colour segments.
*/
const SegmentedRegion& VisionBlackboard::getVerticalFilteredRegion() const
{
    return vertical_filtered_segments;
}

/**
*   @brief returns the horizontal transition rule matches for the given VFO
*   @param vfo_if The identifier of the field object
*   @return horizontal_transitions The horizontal transition rule matches
*
*   @note This method cannot be const as an element is insert by the [] operator
*   in the case that this does not find a mapping (using the default constructor). This
*   is good as there is no need to worry about manually inserting a vector for each field object
*   or doing any checks in this method for missing mappings.
*/
const vector<Transition>& VisionBlackboard::getHorizontalTransitions(VisionFieldObject::VFO_ID vfo_id)
{
    return mapped_horizontal_transitions[vfo_id];
}

/**
*   @brief returns the vertical transition rule matches for the given VFO
*   @param vfo_if The identifier of the field object
*   @return vertical_transitions The vertical transition rule matches
*
*   @note This method cannot be const as an element is insert by the [] operator
*   in the case that this does not find a mapping (using the default constructor). This
*   is good as there is no need to worry about manually inserting a vector for each field object
*   or doing any checks in this method for missing mappings.
*/
const vector<Transition>& VisionBlackboard::getVerticalTransitions(VisionFieldObject::VFO_ID vfo_id)
{
    return mapped_vertical_transitions[vfo_id];
}

/**
*   @brief returns the horizontal transition rule matches for all VFOs
*   @return horizontal_transitions The horizontal transition rule matches for all VFOs
*/
const map<VisionFieldObject::VFO_ID, vector<Transition> >& VisionBlackboard::getHorizontalTransitionsMap() const
{
    return mapped_horizontal_transitions;
}

/**
*   @brief returns the vertical transition rule matches for all VFOs
*   @return vertical_transitions The vertical transition rule matches for all VFOs
*/
const map<VisionFieldObject::VFO_ID, vector<Transition> >& VisionBlackboard::getVerticalTransitionsMap() const
{
    return mapped_vertical_transitions;
}

/**
*   @brief returns a pointer to the Image wrapping the current image buffer.
*   @return A pointer to the Image wrapping the current image buffer.
*/
const NUImage& VisionBlackboard::getOriginalImage() const
{
    return *original_image;
}

/**
*   @brief returns the height of the image in pixels.
*   @return Image height in pixels.
*/
int VisionBlackboard::getImageWidth() const
{
    return original_image->getWidth();
}

/**
*   @brief returns the height of the image in pixels.
*   @return Image height in pixels.
*/
int VisionBlackboard::getImageHeight() const
{
    return original_image->getHeight();
}

/*! @brief Retrieves camera settings from the wrapper and returns them.
*   @return camera settings
*/
CameraSettings VisionBlackboard::getCameraSettings() const
{
    return wrapper->getCameraSettings();
}

/**
*   @brief Retrieves a new LUT from the wrapper.
*/
void VisionBlackboard::updateLUT()
{
    LUT = wrapper->getLUT();
}

/**
*   @brief Retrieves a new image and sensor data from the wrapper.
*/
void VisionBlackboard::update()
{
    //Get updated kinematics data
    kinematics_horizon = wrapper->getKinematicsHorizon();
    //kinematics_horizon_points = wrapper->getKinematicsHorizon();
    ctgvalid = wrapper->getCTGVector(ctgvector);
}

/**
*   @brief Publishes results to wrapper.
*/
void VisionBlackboard::publish() const
{
    Mat classed;
    LUT.classifyImage(*original_image, classed);
    wrapper->publish(DataWrapper::DID_CLASSED_IMAGE, classed);
}

/**
*   @brief Publishes debug information to wrapper.
*/
void VisionBlackboard::debugPublish() const
{
    vector<Scalar> colours;
    vector<PointType> pts;

#if VISION_BLACKBOARD_VERBOSITY > 1
    debug << "VisionBlackboard::debugPublish - " << endl;
    debug << "kinematics_horizon_points: " << kinematics_horizon_points.size() << endl;
    debug << "horizon_scan_points: " << horizon_scan_points.size() << endl;
    debug << "horizon_points: " << horizon_points.size() << endl;
    debug << "object_points: " << object_points.size() << endl;
    debug << "horizontal_scanlines: " << horizontal_scanlines.size() << endl;
    debug << "horizon_points: " << horizon_points.size() << endl;
    debug << "horizontal_segmented_scanlines: " << horizontal_segmented_scanlines.getSegments().size() << endl;
    debug << "vertical_segmented_scanlines: " << vertical_segmented_scanlines.getSegments().size() << endl;
    debug << "horizontal_filtered_segments: " << horizontal_segmented_scanlines.getSegments().size() << endl;
    debug << "vertical_filtered_segments: " << vertical_segmented_scanlines.getSegments().size() << endl;
    debug << "horizontal_transitions: " << horizontal_transitions.size() << endl;
    debug << "vertical_transitions: " << vertical_transitions.size() << endl;
    debug << vertical_transitions << endl;
#endif

    wrapper->debugRefresh();
//    wrapper->debugPublish(DataWrapper::DBID_IMAGE, *original_image_cv_4ch);
    
    //horizon
    pts.clear();
    float A = kinematics_horizon.getA(), B = kinematics_horizon.getB(), C = kinematics_horizon.getC();
    if(B!=0) {
        pts.push_back(PointType(0, -1*C/B));
        pts.push_back(PointType(original_image->getHeight(), -1*A/B*original_image->getWidth() - C/B));
        wrapper->debugPublish(DataWrapper::DBID_HORIZON, pts, Scalar(0,255,255));
    }
    else {
        errorlog << "VisionBlackboard::publishDebug - vertical horizon!" << endl;
    }
    
    //horizon scans
    wrapper->debugPublish(DataWrapper::DBID_GREENHORIZON_SCANS, horizon_scan_points, Scalar(127,127,127));
    
    //horizon points
    wrapper->debugPublish(DataWrapper::DBID_GREENHORIZON_FINAL, horizon_points, Scalar(255,0,255));
    
    //object points
    wrapper->debugPublish(DataWrapper::DBID_OBJECT_POINTS, object_points, Scalar(0,0,255));
    
    //horizontal scans
    pts.clear();
    for(unsigned int i=0; i<horizontal_scanlines.size(); i++) {
        pts.push_back(PointType(0, horizontal_scanlines.at(i)));
    }
    wrapper->debugPublish(DataWrapper::DBID_H_SCANS, pts, Scalar(127,127,127));
    
    //vertical scans
    wrapper->debugPublish(DataWrapper::DBID_V_SCANS, horizon_points, Scalar(127,127,127));
    
    //horizontal segments
    colours.clear();
    pts.clear();
    getPointsAndColoursFromSegments(horizontal_segmented_scanlines.getSegments(), colours, pts);
    wrapper->debugPublish(DataWrapper::DBID_SEGMENTS, pts, colours);
    
    //vertical segments
    colours.clear();
    pts.clear();
    getPointsAndColoursFromSegments(vertical_segmented_scanlines.getSegments(), colours, pts);
    wrapper->debugPublish(DataWrapper::DBID_SEGMENTS, pts, colours);
    
    //horizontal filtered segments
    colours.clear();
    pts.clear();
    getPointsAndColoursFromSegments(horizontal_filtered_segments.getSegments(), colours, pts);
    wrapper->debugPublish(DataWrapper::DBID_FILTERED_SEGMENTS, pts, colours);
    
    //vertical filtered segments
    colours.clear();
    pts.clear();
    getPointsAndColoursFromSegments(vertical_filtered_segments.getSegments(), colours, pts);
    wrapper->debugPublish(DataWrapper::DBID_FILTERED_SEGMENTS, pts, colours);
    
    //horizontal transitions
    map<VisionFieldObject::VFO_ID, vector<Transition> >::const_iterator it;
    vector<Transition> v_t;
    pts.clear();
    for(it=mapped_horizontal_transitions.begin(); it!=mapped_horizontal_transitions.end(); it++) {
        v_t = it->second;
        BOOST_FOREACH(const Transition& t, v_t) {
            pts.push_back(t.getLocation());
        }
    }
    wrapper->debugPublish(DataWrapper::DBID_TRANSITIONS, pts, Scalar(255,255,0));
    
    //vertical transitions
    pts.clear();
    for(it=mapped_vertical_transitions.begin(); it!=mapped_vertical_transitions.end(); it++) {
        v_t = it->second;
        BOOST_FOREACH(const Transition& t, v_t) {
            pts.push_back(t.getLocation());
        }
    }
    wrapper->debugPublish(DataWrapper::DBID_TRANSITIONS, pts, Scalar(0,255,255));
}
