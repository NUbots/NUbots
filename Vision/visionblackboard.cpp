#include "visionblackboard.h"
#include "VisionWrapper/datawrappercurrent.h"
#include "debug.h"
#include "debugverbosityvision.h"
#include "nubotdataconfig.h"
#include "visionconstants.h"

#include <algorithm>
#include <boost/foreach.hpp>
#include "Tools/Math/General.h"

VisionBlackboard* VisionBlackboard::instance = 0;

//! @brief Private constructor for blackboard.
VisionBlackboard::VisionBlackboard()
{
    wrapper = DataWrapper::getInstance();

    LUT = wrapper->getLUT();
    
    //Get Image
    original_image = wrapper->getFrame();

    //get Camera to ground vector
    //ctgvalid = wrapper->getCTGVector(ctgvector);
    
    m_camera_specs = NUCameraData(string(CONFIG_DIR) + string("CameraSpecs.cfg"));

    VisionConstants::loadFromFile(string(CONFIG_DIR) + string("VisionOptions.cfg"));
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

void VisionBlackboard::addGoal(const Goal& newgoal) 
{
    m_goals.push_back(newgoal);
    //m_vfos.push_back(static_cast<const VisionFieldObject*>(&(m_goals.back())));
}

void VisionBlackboard::addBeacon(const Beacon& newbeacon)
{
    m_beacons.push_back(newbeacon);
    //m_vfos.push_back(static_cast<const VisionFieldObject*>(&(m_beacons.back())));
}

void VisionBlackboard::addBall(const Ball& newball)
{
    m_balls.push_back(newball);
    //m_vfos.push_back(static_cast<const VisionFieldObject*>(&(m_balls.back())));
}

void VisionBlackboard::addObstacle(const Obstacle& newobstacle)
{
    m_obstacles.push_back(newobstacle);
    //m_vfos.push_back(static_cast<const VisionFieldObject*>(&(m_obstacles.back())));
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

double VisionBlackboard::calculateBearing(double x) const {
    return atan( (original_image->getWidth()*0.5-x)  * (tan(m_FOV.x*0.5)) / (original_image->getWidth()*0.5) );
}


double VisionBlackboard::calculateElevation(double y) const {
    return atan( (original_image->getHeight()*0.5-y) * (tan(m_FOV.y*0.5)) / (original_image->getHeight()*0.5) );
}

/**
*   @brief returns the kinematics horizon line.
*   @return kinematics_horizon A Line defining the kinematics horizon.
*/
const Horizon& VisionBlackboard::getKinematicsHorizon() const
{
    return kinematics_horizon;
}

bool VisionBlackboard::isCameraToGroundValid() const
{
    return ctgvalid;
}

const vector<float>& VisionBlackboard::getCameraToGroundVector() const
{
    return ctgvector;
}

bool VisionBlackboard::isCameraTransformValid() const
{
    return ctvalid;
}

const vector<float>& VisionBlackboard::getCameraTransformVector() const
{
    return ctvector;
}

bool VisionBlackboard::isCameraPitchValid() const
{
    return camera_pitch_valid;
}

float VisionBlackboard::getCameraPitch() const 
{
    return camera_pitch;
}

bool VisionBlackboard::isCameraHeightValid() const
{
    return camera_height_valid;
}

float VisionBlackboard::getCameraHeight() const
{
    return camera_height;
}

bool VisionBlackboard::isBodyPitchValid() const
{
    return body_pitch_valid;
}

float VisionBlackboard::getBodyPitch() const 
{
    return body_pitch;
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

double VisionBlackboard::getCameraDistanceInPixels() const
{
    return effective_camera_dist_pixels;
}

bool VisionBlackboard::distanceToPoint(float bearing, float elevation, float& distance) const
{
    float theta = 0;
    if(camera_height_valid && camera_pitch_valid) {
        //resultant angle inclusive of body pitch, camera pitch, pixel elevation and angle correction factor
        theta = mathGeneral::PI*0.5 - camera_pitch + elevation + VisionConstants::D2P_ANGLE_CORRECTION;    
        if(VisionConstants::D2P_INCLUDE_BODY_PITCH) {
            if(body_pitch_valid) {
                distance = camera_height / cos(theta - body_pitch) / cos(bearing);
                return true;
            }
            else {
                distance = 0;
                return false;
            }
        }
        else {
            distance = camera_height / cos(theta) / cos(bearing);
            return true;
        }
    }
    else {
        return false;
    }
}

/*! @brief Retrieves camera settings from the wrapper and returns them.
*   @return camera settings
*/
CameraSettings VisionBlackboard::getCameraSettings() const
{
    return  original_image->getCameraSettings();
}

/**
*   @brief Retrieves a new LUT from the wrapper.
*/
void VisionBlackboard::updateLUT()
{
    LUT = wrapper->getLUT();
}

void VisionBlackboard::calculateFOVAndCamDist()
{
    #if VISION_BLACKBOARD_VERBOSITY > 1
        debug << "VisionBlackboard::calculateFOVAndCamDist - Begin" << endl;
    #endif  
    
    m_FOV = Vector2<double>(m_camera_specs.m_horizontalFov, m_camera_specs.m_verticalFov);
    effective_camera_dist_pixels = (0.5*original_image->getWidth())/tan(0.5*m_FOV.x);
    
    #if VISION_BLACKBOARD_VERBOSITY > 1
        debug << "VisionBlackboard::calculateFOVAndCamDist - End" << endl;
    #endif  
}

/**
*   @brief Retrieves a new image and sensor data from the wrapper.
*   This method instructs the wrapper to update itself, then grabs the new information 
*   from the wrapper.
*/
void VisionBlackboard::update()
{
    #if VISION_BLACKBOARD_VERBOSITY > 1
        debug << "VisionBlackboard::update() - Begin" << endl;
    #endif
        
    //update sensor data copies
    ctgvalid = wrapper->getCTGVector(ctgvector);
    ctvalid = wrapper->getCTVector(ctvector);
    camera_pitch_valid = wrapper->getCameraPitch(camera_pitch);
    camera_height_valid = wrapper->getCameraHeight(camera_height);
    body_pitch_valid = wrapper->getBodyPitch(body_pitch);
    //get new image pointer
    original_image = wrapper->getFrame();
    
    //Get updated kinematics data
    kinematics_horizon = wrapper->getKinematicsHorizon();
    checkHorizon();
    
    //calculate the field of view and effective camera distance
    calculateFOVAndCamDist();
    #if VISION_BLACKBOARD_VERBOSITY > 1
        debug << "VisionBlackboard::update() - Finish" << endl;
    #endif
        
    //clear out result vectors
    m_balls.clear();
    m_beacons.clear();
    m_goals.clear();
    m_obstacles.clear();
    m_vfos.clear();
}

/**
*   @brief Publishes results to wrapper.
*/
void VisionBlackboard::publish() const
{
    #if VISION_BLACKBOARD_VERBOSITY > 1
        debug << "VisionBlackboard::publish() - Begin" << endl;
    #endif
    //wrapper->publish(m_vfos);
    unsigned int i;
    for(i=0; i<m_balls.size(); i++) {
        wrapper->publish(static_cast<const VisionFieldObject*>(&m_balls.at(i)));
    }
    for(i=0; i<m_beacons.size(); i++) {
        wrapper->publish(static_cast<const VisionFieldObject*>(&m_beacons.at(i)));
    }
    for(i=0; i<m_goals.size(); i++) {
        wrapper->publish(static_cast<const VisionFieldObject*>(&m_goals.at(i)));
    }
    for(i=0; i<m_obstacles.size(); i++) {
        wrapper->publish(static_cast<const VisionFieldObject*>(&m_obstacles.at(i)));
    }
    #if VISION_BLACKBOARD_VERBOSITY > 1
        debug << "VisionBlackboard::publish() - End" << endl;
    #endif
}

/**
*   @brief Publishes debug information to wrapper.
*/
void VisionBlackboard::debugPublish() const
{
    #if VISION_BLACKBOARD_VERBOSITY > 1
        debug << "VisionBlackboard::debugPublish() - Begin" << endl;
    #endif
    vector<PointType> pts;
    map<VisionFieldObject::VFO_ID, vector<Transition> >::const_iterator it;
    vector<Transition> v_t;

#if VISION_BLACKBOARD_VERBOSITY > 1
    debug << "VisionBlackboard::debugPublish - " << endl;
    debug << "kinematics_horizon: " << kinematics_horizon.getA() << " " << kinematics_horizon.getB() << " " << kinematics_horizon.getC() << endl;
    debug << "horizon_scan_points: " << horizon_scan_points.size() << endl;
    debug << "horizon_points: " << horizon_points.size() << endl;
    debug << "object_points: " << object_points.size() << endl;
    debug << "horizontal_scanlines: " << horizontal_scanlines.size() << endl;
    debug << "horizon_points: " << horizon_points.size() << endl;
    debug << "horizontal_segmented_scanlines: " << horizontal_segmented_scanlines.getSegments().size() << endl;
    debug << "vertical_segmented_scanlines: " << vertical_segmented_scanlines.getSegments().size() << endl;
    debug << "horizontal_filtered_segments: " << horizontal_segmented_scanlines.getSegments().size() << endl;
    debug << "vertical_filtered_segments: " << vertical_segmented_scanlines.getSegments().size() << endl;
    int size=0;
    for(it=mapped_horizontal_transitions.begin(); it!=mapped_horizontal_transitions.end(); it++) {
        size += it->second.size();
    }
    debug << "mapped_horizontal_transitions: " << size << endl;
    size=0;
    for(it=mapped_vertical_transitions.begin(); it!=mapped_vertical_transitions.end(); it++) {
        size += it->second.size();
    }
    debug << "mapped_vertical_transitions: " << size << endl;
#endif

    wrapper->debugRefresh();
//    wrapper->debugPublish(DataWrapper::DBID_IMAGE, *original_image_cv_4ch);
    
    //horizon
    pts.clear();
    float A = kinematics_horizon.getA(), B = kinematics_horizon.getB(), C = kinematics_horizon.getC();
    if(B!=0) {
        pts.push_back(PointType(0, -1*C/B));
        pts.push_back(PointType(original_image->getHeight(), -1*A/B*original_image->getWidth() - C/B));
        wrapper->debugPublish(DataWrapper::DBID_HORIZON, pts);
    }
    else {
        errorlog << "VisionBlackboard::publishDebug - vertical horizon!" << endl;
    }
    
    //horizon scans
    wrapper->debugPublish(DataWrapper::DBID_GREENHORIZON_SCANS, horizon_scan_points);
    
    //horizon points
    wrapper->debugPublish(DataWrapper::DBID_GREENHORIZON_FINAL, horizon_points);
    
    //object points
    wrapper->debugPublish(DataWrapper::DBID_OBJECT_POINTS, object_points);
    
    //field objects
    wrapper->debugPublish(m_goals);
    wrapper->debugPublish(m_beacons);
    wrapper->debugPublish(m_balls);
    wrapper->debugPublish(m_obstacles);
    
    //horizontal scans
    pts.clear();
    for(unsigned int i=0; i<horizontal_scanlines.size(); i++) {
        pts.push_back(PointType(0, horizontal_scanlines.at(i)));
    }
    wrapper->debugPublish(DataWrapper::DBID_H_SCANS, pts);
    
    //vertical scans
    wrapper->debugPublish(DataWrapper::DBID_V_SCANS, horizon_points);
    
    //horizontal segments
    wrapper->debugPublish(DataWrapper::DBID_SEGMENTS, horizontal_segmented_scanlines);
    
    //vertical segments
    wrapper->debugPublish(DataWrapper::DBID_SEGMENTS, vertical_segmented_scanlines);
    
    //horizontal filtered segments
    wrapper->debugPublish(DataWrapper::DBID_FILTERED_SEGMENTS, horizontal_filtered_segments);
    
    //vertical filtered segments
    wrapper->debugPublish(DataWrapper::DBID_FILTERED_SEGMENTS, vertical_filtered_segments);
    
    //horizontal transitions
    pts.clear();
    for(it=mapped_horizontal_transitions.begin(); it!=mapped_horizontal_transitions.end(); it++) {
        v_t = it->second;
        BOOST_FOREACH(const Transition& t, v_t) {
            pts.push_back(t.getLocation());
        }
    }
    wrapper->debugPublish(DataWrapper::DBID_TRANSITIONS, pts);
    
    //vertical transitions
    pts.clear();
    for(it=mapped_vertical_transitions.begin(); it!=mapped_vertical_transitions.end(); it++) {
        v_t = it->second;
        BOOST_FOREACH(const Transition& t, v_t) {
            pts.push_back(t.getLocation());
        }
    }
    wrapper->debugPublish(DataWrapper::DBID_TRANSITIONS, pts);
}

void VisionBlackboard::checkHorizon()
{
    #if VISION_BLACKBOARD_VERBOSITY > 1
        debug << "VisionBlackboard::checkHorizon() - Begin." << endl;
    #endif
    int width = original_image->getWidth(),
        height = original_image->getHeight();
    
    if(kinematics_horizon.exists) {
        if(kinematics_horizon.isVertical()) {
            //vertical horizon
            #if VISION_BLACKBOARD_VERBOSITY > 1
                debug << "VisionBlackboard::checkHorizon() - Vertical Horizon, clamping to top." << endl;
            #endif
            //kinematics_horizon.setLineFromPoints(Point(0, height), Point(width, height));
        }
        else {
            if(kinematics_horizon.findYFromX(0) < 0 || kinematics_horizon.findYFromX(0) > height) {
                //left point off screen
                #if VISION_BLACKBOARD_VERBOSITY > 1
                    debug << "VisionBlackboard::checkHorizon() - Left kinematics horizon point off screen." << endl;
                #endif
            }
            if(kinematics_horizon.findYFromX(width) < 0 || kinematics_horizon.findYFromX(width) > height) {
                //right point off screen
                #if VISION_BLACKBOARD_VERBOSITY > 1
                    debug << "VisionBlackboard::checkHorizon() - Right kinematics horizon point off screen." << endl;
                #endif
            }
        }
    }
    else {
        #if VISION_BLACKBOARD_VERBOSITY > 1
            debug << "VisionBlackboard::checkHorizon() - Horizon non-existant." << endl;
        #endif
    }
}
