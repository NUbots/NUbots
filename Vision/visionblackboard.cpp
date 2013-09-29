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
    //Get Image
    original_image = NULL;

    VisionConstants::loadFromFile(std::string(CONFIG_DIR) + std::string("VisionOptions.cfg"));
}

/** @brief Private destructor.
*
*   To be called implicitly at program close or by the controller as the VBB is live for the
*   whole program in absence of errors.
*/
VisionBlackboard::~VisionBlackboard()
{
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
void VisionBlackboard::setGreenHullPoints(const std::vector<Point>& points)
{
    m_green_horizon.set(points, Point(original_image->getWidth(), original_image->getHeight()));
}

/**
*   @brief sets the green horizon scan point set.
*   @param points A vector of pixel locations for the green horizon scan.
*
*   Clears the previous list of point pointers and copies the new list.
*/
void VisionBlackboard::setGreenHorizonScanPoints(const std::vector<Point>& points)
{
    gh_scan_points = points;
}

/**
*   @brief sets the obstacle point set.
*   @param points A vector of pixel locations for potential obstacles.
*
*   Clears the previous list of point pointers and copies the new list.
*/
void VisionBlackboard::setObstaclePoints(const std::vector<Point>& points)
{
    obstacle_points = points;
}

/**
  * Adds a goal to the list of results.
  * @param newgoal The goal to add.
  */
void VisionBlackboard::addGoal(const Goal& newgoal) 
{
    m_goals.push_back(newgoal);
    //m_vfos.push_back(static_cast<const VisionFieldObject*>(&(m_goals.back())));
}

///**
//  * Adds a beacon to the list of results.
//  * @param newbeacon The beacon to add.
//  */
//void VisionBlackboard::addBeacon(const Beacon& newbeacon)
//{
//    m_beacons.push_back(newbeacon);
//    //m_vfos.push_back(static_cast<const VisionFieldObject*>(&(m_beacons.back())));
//}

/**
  * Adds a ball to the list of results.
  * @param newball The ball to add.
  */
void VisionBlackboard::addBall(const Ball& newball)
{
    m_balls.push_back(newball);
    //m_vfos.push_back(static_cast<const VisionFieldObject*>(&(m_balls.back())));
}

/**
  * Adds an obstacle to the list of results.
  * @param newobstacle The obstacle to add.
  */
void VisionBlackboard::addObstacle(const Obstacle& newobstacle)
{
    m_obstacles.push_back(newobstacle);
    //m_vfos.push_back(static_cast<const VisionFieldObject*>(&(m_obstacles.back())));
}

/**
  * Adds a lines to the current set.
  * @param newline The line to add.
  */
void VisionBlackboard::addLine(const FieldLine& newline)
{
    m_lines.push_back(newline);
}

/**
  * Adds a centre circle to the current set.
  * @param newCircle The circle to add.
  */
void VisionBlackboard::addCentreCircle(const CentreCircle &newcircle)
{
    m_centre_circles.push_back(newcircle);
}

/**
  * Adds a corner to the current set.
  * @param newCorner The corner point to add.
  */
void VisionBlackboard::addCornerPoint(const CornerPoint &newcorner)
{
    m_corner_points.push_back(newcorner);
}

void VisionBlackboard::addGoals(const std::vector<Goal>& newgoals)
{
    if(!newgoals.empty())
        m_goals.insert(m_goals.end(), newgoals.begin(), newgoals.end());
}

//void VisionBlackboard::addBeacons(const vector<Beacon>& newbeacons)
//{
//    m_beacons.insert(m_beacons.end(), newbeacons.begin(), newbeacons.end());
//}

void VisionBlackboard::addBalls(const std::vector<Ball>& newballs)
{
    if(!newballs.empty())
        m_balls.insert(m_balls.end(), newballs.begin(), newballs.end());
}
void VisionBlackboard::addObstacles(const std::vector<Obstacle>& newobstacles)
{
    if(!newobstacles.empty())
        m_obstacles.insert(m_obstacles.end(), newobstacles.begin(), newobstacles.end());
}

void VisionBlackboard::addLines(const std::vector<FieldLine>& newlines)
{
    if(!newlines.empty())
        m_lines.insert(m_lines.end(), newlines.begin(), newlines.end());
}

void VisionBlackboard::addCentreCircles(const std::vector<CentreCircle> &newcircles)
{
    if(!newcircles.empty())
        m_centre_circles.insert(m_centre_circles.end(), newcircles.begin(), newcircles.end());
}

void VisionBlackboard::addCornerPoints(const std::vector<CornerPoint> &newcorners)
{
    if(!newcorners.empty())
        m_corner_points.insert(m_corner_points.end(), newcorners.begin(), newcorners.end());
}

/**
*   @brief sets the horizontal scan lines.
*   @param horizontal_scanlines A vector of unsigned ints defining horizontal scanlines.
*
*   Clears the previous list of point pointers and copies the new list.
*/
void VisionBlackboard::setHorizontalScanlines(const std::vector<int>& scanlines)
{
    horizontal_scanlines = scanlines;
}

/**
*   @brief sets the horizontal segments.
*   @param segmented_scanlines A vector of vectors of colour segments.
*/
void VisionBlackboard::setHorizontalSegments(const std::vector<std::vector<ColourSegment> >& segmented_scanlines)
{
    horizontal_segmented_scanlines.set(segmented_scanlines, HORIZONTAL);
}

/**
*   @brief sets the vertical segments.
*   @param segmented_scanlines A vector of vectors of colour segments.
*/
void VisionBlackboard::setVerticalSegments(const std::vector<std::vector<ColourSegment> >& segmented_scanlines)
{
    vertical_segmented_scanlines.set(segmented_scanlines, VERTICAL);
}

/**
*   @brief sets the filtered horizontal segments.
*   @param segmented_scanlines A vector of vectors of colour segments.
*/
void VisionBlackboard::setHorizontalFilteredSegments(const std::vector<std::vector<ColourSegment> >& segmented_scanlines)
{
    horizontal_filtered_segments.set(segmented_scanlines, HORIZONTAL);
}

/**
*   @brief sets the filtered vertical segments.
*   @param segmented_scanlines A vector of vectors of colour segments.
*/
void VisionBlackboard::setVerticalFilteredSegments(const std::vector<std::vector<ColourSegment> >& segmented_scanlines)
{
    vertical_filtered_segments.set(segmented_scanlines, VERTICAL);
}

/**
*   @brief sets the vertical transition rule matches for the given vision field object.
*   @param vfo_if The identifier of the field object
*   @param transitions A vector of transitions that matched the horizontal rules.
*/
void VisionBlackboard::setHorizontalTransitions(COLOUR_CLASS colour_class, const std::vector<ColourSegment> &transitions)
{
    matched_horizontal_segments[colour_class] = transitions;
}

/**
*   @brief sets the vertical transition rule matches for the given vision field object.
*   @param vfo_if The identifier of the field object
*   @param transitions A vector of transitions that matched the vertical rules.
*/
void VisionBlackboard::setVerticalTransitions(COLOUR_CLASS colour_class, const std::vector<ColourSegment> &transitions)
{
    matched_vertical_segments[colour_class] = transitions;
}

/**
*   @brief sets the horizontal transition rule matches for all vision field objects.
*   @param t_map A map from COLOUR_CLASSs to transitions that matched the horizontal rules.
*/
void VisionBlackboard::setHorizontalTransitionsMap(const map<COLOUR_CLASS, std::vector<ColourSegment> > &t_map)
{
    matched_horizontal_segments = t_map;
}

/**
*   @brief sets the vertical transition rule matches for all vision field objects.
*   @param t_map A map from COLOUR_CLASSs to transitions that matched the vertical rules.
*/
void VisionBlackboard::setVerticalTransitionsMap(const map<COLOUR_CLASS, std::vector<ColourSegment> > &t_map)
{
    matched_vertical_segments = t_map;
}

/**
*   @brief returns the green horizon.
*/
const GreenHorizon& VisionBlackboard::getGreenHorizon() const
{
    return m_green_horizon;
}

/**
*   @brief returns the object point set.
*   @return points A vector of pixel locations for objects.
*/
const std::vector<Point>& VisionBlackboard::getObstaclePoints() const
{
    return obstacle_points;
}

/// @brief returns a pointer to the Lookup Table.
const LookUpTable& VisionBlackboard::getLUT() const
{
    return wrapper->getLUT();
}

/// @brief returns the kinematics horizon line.
const Horizon& VisionBlackboard::getKinematicsHorizon() const
{
    return kinematics_horizon;
}

/// @brief returns the transformations object.
const Transformer& VisionBlackboard::getTransformer() const
{
    return m_transformer;
}

//! Returns the list of found balls.
const std::vector<Ball>& VisionBlackboard::getBalls()
{
    return m_balls;
}

//! Returns the list of found goals.
const std::vector<Goal>& VisionBlackboard::getGoals()
{
    return m_goals;
}

////! Returns the list of found beacons.
//const std::vector<Beacon>& VisionBlackboard::getBeacons()
//{
//    return m_beacons;
//}

//! Returns the list of found obstacles.
const std::vector<Obstacle>& VisionBlackboard::getObstacles()
{
    return m_obstacles;
}

//! Returns the list of found lines.
const std::vector<FieldLine>& VisionBlackboard::getLines()
{
    return m_lines;
}

const std::vector<CentreCircle>& VisionBlackboard::getCentreCircles()
{
    return m_centre_circles;
}

const std::vector<CornerPoint>& VisionBlackboard::getCorners()
{
    return m_corner_points;
}

/**
*   @brief returns the set of heights for horizontal scan lines.
*   @return horizontal_scanlines A vector of unsigned ints defining horizontal scanlines.
*/
const std::vector<int>& VisionBlackboard::getHorizontalScanlines() const
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
*   @return horizontal_segments The horizontal transition rule matches
*
*   @note This method cannot be const as an element is accessed by the [] operator
*   in the case that this does not find a mapping (using the default constructor). This
*   is good as there is no need to worry about manually inserting a vector for each field object
*   or doing any checks in this method for missing mappings.
*/
const std::vector<ColourSegment> &VisionBlackboard::getHorizontalTransitions(COLOUR_CLASS colour_class)
{
    return matched_horizontal_segments[colour_class];
}

/**
*   @brief returns the vertical transition rule matches for the given VFO
*   @param vfo_if The identifier of the field object
*   @return vertical_segments The vertical transition rule matches
*
*   @note This method cannot be const as an element is accessed by the [] operator
*   in the case that this does not find a mapping (using the default constructor). This
*   is good as there is no need to worry about manually inserting a vector for each field object
*   or doing any checks in this method for missing mappings.
*/
const std::vector<ColourSegment> &VisionBlackboard::getVerticalTransitions(COLOUR_CLASS colour_class)
{
    return matched_vertical_segments[colour_class];
}

/**
*   @brief returns the horizontal and vertical transition rule matches for the given VFO
*   @param vfo_if The identifier of the field object
*   @return vertical_segments The horizontal and vertical transition rule matches
*
*   @note This method cannot be const as an element is accessed by the [] operator
*   in the case that this does not find a mapping (using the default constructor). This
*   is good as there is no need to worry about manually inserting a vector for each field object
*   or doing any checks in this method for missing mappings.
*/
std::vector<ColourSegment> VisionBlackboard::getAllTransitions(COLOUR_CLASS colour_class)
{
    std::vector<ColourSegment> segments = matched_horizontal_segments[colour_class];
    segments.insert(segments.end(), matched_vertical_segments[colour_class].begin(), matched_vertical_segments[colour_class].end());
    return segments;
}

/**
*   @brief returns the horizontal transition rule matches for all VFOs
*   @return horizontal_segments The horizontal transition rule matches for all VFOs
*/
const map<COLOUR_CLASS, std::vector<ColourSegment> > &VisionBlackboard::getHorizontalTransitionsMap() const
{
    return matched_horizontal_segments;
}

/**
*   @brief returns the vertical transition rule matches for all VFOs
*   @return vertical_segments The vertical transition rule matches for all VFOs
*/
const map<COLOUR_CLASS, std::vector<ColourSegment> > &VisionBlackboard::getVerticalTransitionsMap() const
{
    return matched_vertical_segments;
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

/// @brief returns the horizontal and vertical field of view of the camera
Vector2<double> VisionBlackboard::getFOV() const
{
    return m_transformer.getFOV();
}

/*! @brief Retrieves camera settings from the wrapper and returns them.
*   @return camera settings
*/
CameraSettings VisionBlackboard::getCameraSettings() const
{
    return  original_image->getCameraSettings();
}

/**
*   @brief Retrieves a new image and sensor data from the wrapper.
*   This method instructs the wrapper to update itself, then grabs the new information 
*   from the wrapper.
*/
void VisionBlackboard::update()
{
#if VISION_CONTROLFLOW_VERBOSITY > 0
    debug << "VisionBlackboard::update() - Begin" << std::endl;
#endif

    wrapper = DataWrapper::getInstance();

    //get new image pointer
    original_image = wrapper->getFrame();

    //WARNING The following warning may not be triggered properly
    if(original_image == NULL) {
        std::cout << "VisionBlackboard::update() - WARNING - Camera Image pointer is null - Camera may be disconnected or faulty." << std::endl;
        errorlog << "VisionBlackboard::update() - WARNING - Camera Image pointer is null - Camera may be disconnected or faulty." << std::endl;
    }
    else if(original_image->getHeight()==0 or original_image->getWidth()==0 ) {
        std::cout << "VisionBlackboard::update() - WARNING - Image height or width is zero - Camera may be disconnected or faulty." << std::endl;
        errorlog << "VisionBlackboard::update() - WARNING - Image height or width is zero - Camera may be disconnected or faulty." << std::endl;
    }

    //get data copies from wrapper
    float head_pitch = wrapper->getHeadPitch();
    float head_yaw = wrapper->getHeadYaw();
    Vector3<float> orientation = wrapper->getOrientation();
    Vector3<double> neck_position = wrapper->getNeckPosition();

    //setup transformer
    m_transformer.setSensors(head_pitch, head_yaw, orientation.x, orientation.y, neck_position);
    m_transformer.setCamParams(Vector2<double>(original_image->getWidth(), original_image->getHeight()),
                               wrapper->getCameraFOV());
    m_transformer.setCalibration(SensorCalibration());

    //setup horizon
    kinematics_horizon = wrapper->getKinematicsHorizon();
    checkKinematicsHorizon();
        
    //clear intermediates
    matched_horizontal_segments.clear();
    matched_vertical_segments.clear();

    //clear out result vectors
    m_balls.clear();
    //m_beacons.clear();
    m_goals.clear();
    m_obstacles.clear();
    m_lines.clear();
    m_corner_points.clear();
    m_centre_circles.clear();
    m_vfos.clear();

#if VISION_CONTROLFLOW_VERBOSITY > 0
    debug << "VisionBlackboard::update() - Finish" << std::endl;
#endif
}

/**
*   @brief Publishes results to wrapper.
*/
void VisionBlackboard::publish() const
{
    #if VISION_CONTROLFLOW_VERBOSITY > 0
        debug << "VisionBlackboard::publish() - Begin" << std::endl;
    #endif
    //wrapper->publish(m_vfos);
    unsigned int i;
    for(i=0; i<m_balls.size(); i++) {
        wrapper->publish(static_cast<const VisionFieldObject*>(&m_balls.at(i)));
    }
//    for(i=0; i<m_beacons.size(); i++) {
//        wrapper->publish(static_cast<const VisionFieldObject*>(&m_beacons.at(i)));
//    }
    for(i=0; i<m_goals.size(); i++) {
        wrapper->publish(static_cast<const VisionFieldObject*>(&m_goals.at(i)));
    }
    for(i=0; i<m_obstacles.size(); i++) {
        wrapper->publish(static_cast<const VisionFieldObject*>(&m_obstacles.at(i)));
    }
    for(i=0; i<m_lines.size(); i++) {
        wrapper->publish(static_cast<const VisionFieldObject*>(&m_lines.at(i)));
    }
    for(i=0; i<m_centre_circles.size(); i++) {
        wrapper->publish(static_cast<const VisionFieldObject*>(&m_centre_circles.at(i)));
    }
    for(i=0; i<m_corner_points.size(); i++) {
        wrapper->publish(static_cast<const VisionFieldObject*>(&m_corner_points.at(i)));
    }
    #if VISION_CONTROLFLOW_VERBOSITY > 0
        debug << "VisionBlackboard::publish() - End" << std::endl;
    #endif
}

/**
*   @brief Publishes debug information to wrapper.
*/
void VisionBlackboard::debugPublish() const
{
    #if VISION_CONTROLFLOW_VERBOSITY > 0
        debug << "VisionBlackboard::debugPublish() - Begin" << std::endl;
    #endif
    std::vector<Vector2<double> > pts;
    map<COLOUR_CLASS, std::vector<ColourSegment> >::const_iterator it;
    std::vector<ColourSegment> v_s;

#if VISION_BLACKBOARD_VERBOSITY > 1
    debug << "VisionBlackboard::debugPublish - " << std::endl;
    debug << "kinematics_horizon: " << kinematics_horizon.getA() << " " << kinematics_horizon.getB() << " " << kinematics_horizon.getC() << std::endl;
    debug << "horizon_scan_points: " << m_green_horizon.getOriginalPoints().size() << std::endl;
    debug << "object_points: " << obstacle_points.size() << std::endl;
    debug << "horizontal_scanlines: " << horizontal_scanlines.size() << std::endl;
    debug << "horizontal_segmented_scanlines: " << horizontal_segmented_scanlines.getSegments().size() << std::endl;
    debug << "vertical_segmented_scanlines: " << vertical_segmented_scanlines.getSegments().size() << std::endl;
    debug << "horizontal_filtered_segments: " << horizontal_segmented_scanlines.getSegments().size() << std::endl;
    debug << "vertical_filtered_segments: " << vertical_segmented_scanlines.getSegments().size() << std::endl;
    int size=0;
    for(it=matched_horizontal_segments.begin(); it!=matched_horizontal_segments.end(); it++) {
        size += it->second.size();
    }
    debug << "matched_horizontal_segments: " << size << std::endl;
    size=0;
    for(it=matched_vertical_segments.begin(); it!=matched_vertical_segments.end(); it++) {
        size += it->second.size();
    }
    debug << "matched_vertical_segments: " << size << std::endl;
#endif

    wrapper->debugPublish(DBID_IMAGE, original_image);
    wrapper->debugPublish(DBID_CLASSED_IMAGE);
    
    //horizon
    pts.clear();
    if(!kinematics_horizon.isVertical()) {
        pts.push_back(Vector2<double>(0, kinematics_horizon.findYFromX(0)));
        pts.push_back(Vector2<double>(original_image->getWidth(), kinematics_horizon.findYFromX(original_image->getWidth())));
        wrapper->debugPublish(DBID_HORIZON, pts);
    }
    else {
        errorlog << "VisionBlackboard::publishDebug - vertical horizon!" << std::endl;
    }

    //horizon scans
    wrapper->debugPublish(DBID_GREENHORIZON_SCANS, gh_scan_points);
    
    //horizon points
    wrapper->debugPublish(DBID_GREENHORIZON_FINAL, m_green_horizon.getOriginalPoints());
    
    //object points
    wrapper->debugPublish(DBID_OBSTACLE_POINTS, obstacle_points);
    
    //field objects
    wrapper->debugPublish(m_goals);
    //wrapper->debugPublish(m_beacons);
    wrapper->debugPublish(m_balls);
    wrapper->debugPublish(m_obstacles);
    wrapper->debugPublish(m_lines);
    wrapper->debugPublish(m_centre_circles);
    wrapper->debugPublish(m_corner_points);
    
    //horizontal scans
    pts.clear();
    for(unsigned int i=0; i<horizontal_scanlines.size(); i++) {
        pts.push_back(Vector2<double>(0, horizontal_scanlines.at(i)));
    }
    wrapper->debugPublish(DBID_H_SCANS, pts);
    
    //vertical scans
    wrapper->debugPublish(DBID_V_SCANS, m_green_horizon.getInterpolatedSubset(VisionConstants::VERTICAL_SCANLINE_SPACING));
    
    //horizontal segments
    wrapper->debugPublish(DBID_SEGMENTS, horizontal_segmented_scanlines);
    
    //vertical segments
    wrapper->debugPublish(DBID_SEGMENTS, vertical_segmented_scanlines);
    
    //horizontal filtered segments
    wrapper->debugPublish(DBID_FILTERED_SEGMENTS, horizontal_filtered_segments);
    
    //vertical filtered segments
    wrapper->debugPublish(DBID_FILTERED_SEGMENTS, vertical_filtered_segments);
    
    //horizontal transitions
    pts.clear();
    for(it=matched_horizontal_segments.begin(); it!=matched_horizontal_segments.end(); it++) {
        v_s = it->second;
        BOOST_FOREACH(const ColourSegment& s, v_s) {
            if(s.getColour() == white) {
                pts.push_back(Point(s.getCentre().x, s.getCentre().y));
            }
            else {
                pts.push_back(Point(s.getStart().x, s.getStart().y));
                pts.push_back(Point(s.getEnd().x, s.getEnd().y));
            }
        }
    }
    wrapper->debugPublish(DBID_MATCHED_SEGMENTS, pts);
    
    //vertical transitions
    pts.clear();
    for(it=matched_vertical_segments.begin(); it!=matched_vertical_segments.end(); it++) {
        v_s = it->second;
        BOOST_FOREACH(const ColourSegment& s, v_s) {
            if(s.getColour() == white) {
                pts.push_back(Point(s.getCentre().x, s.getCentre().y));
            }
            else {
                pts.push_back(Point(s.getStart().x, s.getStart().y));
                pts.push_back(Point(s.getEnd().x, s.getEnd().y));
            }
        }
    }
    wrapper->debugPublish(DBID_MATCHED_SEGMENTS, pts);
}

//! Checks the kinematics horizon is within the image bounds and resets it if not.
void VisionBlackboard::checkKinematicsHorizon()
{
    #if VISION_CONTROLFLOW_VERBOSITY > 0
        debug << "VisionBlackboard::checkHorizon() - Begin." << std::endl;
    #endif
    int width = original_image->getWidth(),
        height = original_image->getHeight();
    
    if(kinematics_horizon.exists) {
        if(kinematics_horizon.isVertical()) {
            //vertical horizon
            #if VISION_BLACKBOARD_VERBOSITY > 1
                debug << "VisionBlackboard::checkHorizon() - Vertical Horizon, clamping to top." << std::endl;
            #endif
            kinematics_horizon.setLineFromPoints(Point(0, 0), Point(width, 0));
        }
        else {
            //check the base points are below the horizon
            if(kinematics_horizon.IsBelowHorizon(0, height) && kinematics_horizon.IsBelowHorizon(width, height)) {
                #if VISION_BLACKBOARD_VERBOSITY > 1
                    debug << "VisionBlackboard::checkHorizon() - Image bottom corners are not below horizon - clamping to top." << std::endl;
                #endif
                kinematics_horizon.setLineFromPoints(Point(0, 0), Point(width, 0));
            }
//            if(kinematics_horizon.findYFromX(0) < 0 || kinematics_horizon.findYFromX(0) > height) {
//                //left point off screen
//                #if VISION_BLACKBOARD_VERBOSITY > 1
//                    debug << "VisionBlackboard::checkHorizon() - Left kinematics horizon point off screen." << std::endl;
//                #endif
//            }
//            if(kinematics_horizon.findYFromX(width) < 0 || kinematics_horizon.findYFromX(width) > height) {
//                //right point off screen
//                #if VISION_BLACKBOARD_VERBOSITY > 1
//                    debug << "VisionBlackboard::checkHorizon() - Right kinematics horizon point off screen." << std::endl;
//                #endif
//            }
        }
    }
    else {
        #if VISION_BLACKBOARD_VERBOSITY > 1
            debug << "VisionBlackboard::checkHorizon() - Horizon non-existant." << std::endl;
        #endif
    }
}
