#include "obstacle.h"
#include "Vision/visionblackboard.h"
#include "Vision/visionconstants.h"

#include "debug.h"
#include "debugverbosityvision.h"

#include "Kinematics/Kinematics.h"
#include "Tools/Math/Matrix.h"

Obstacle::Obstacle(const Point& position, double width, double height)
{
    m_id = OBSTACLE;
    m_location_pixels = position;
    m_size_on_screen = Vector2<double>(width, height);
//    if(VisionConstants::DO_RADIAL_CORRECTION) {
//        VisionBlackboard* vbb = VisionBlackboard::getInstance();
//        Vector2<float> bottomcentre = Vector2<float>(position.x, position.y);

//        bottomcentre = vbb->correctDistortion(bottomcentre);

//        m_bottom_centre = Vector2<int>(bottomcentre.x, bottomcentre.y);
//    }

    //CALCULATE DISTANCE AND BEARING VALS
    valid = calculatePositions();
    //valid = valid && check();
    valid = check();
}

bool Obstacle::addToExternalFieldObjects(FieldObjects *fieldobjects, float timestamp) const
{
#if VISION_FIELDOBJECT_VERBOSITY > 1
    debug << "Obstacle::addToExternalFieldObjects" << endl;
    debug << "    " << *this << endl;
#endif
if(valid) {
    #if VISION_FIELDOBJECT_VERBOSITY > 1
        debug << "Obstacle::addToExternalFieldObjects - valid" << endl;
    #endif
    VisionBlackboard* vbb = VisionBlackboard::getInstance();
    AmbiguousObject newAmbObj = AmbiguousObject(FieldObjects::FO_OBSTACLE, "Unknown Obstacle");
    //newAmbObj.addPossibleObjectID(FieldObjects::FO_BLUE_ROBOT_UNKNOWN);
    newAmbObj.UpdateVisualObject(m_spherical_position,
                                 m_spherical_error,
                                 m_location_angular,
                                 Vector2<int>(m_location_pixels.x,m_location_pixels.y),
                                 Vector2<int>(m_size_on_screen.x,m_size_on_screen.y),
                                 timestamp);
    newAmbObj.arc_width = m_size_on_screen.x * vbb->getFOV().x / vbb->getImageWidth();
    fieldobjects->ambiguousFieldObjects.push_back(newAmbObj);
    return true;
}
else {
    #if VISION_FIELDOBJECT_VERBOSITY > 1
        debug << "Obstacle::addToExternalFieldObjects - invalid" << endl;
    #endif
    return false;
}
}

bool Obstacle::check() const
{
    //! @todo Do a check based on width and d2p consistency
//    if(!distance_valid) {
//        #if VISION_FIELDOBJECT_VERBOSITY > 1
//            debug << "Obstacle::check - Obstacle thrown out: distance invalid" << endl;
//        #endif
//        return false;
//    }

    //all checks passed
    return true;
}

bool Obstacle::calculatePositions()
{
    const Transformer& transformer = VisionBlackboard::getInstance()->getTransformer();
    //To the bottom of the Goal Post.
    Point radial = transformer.screenToRadial2D(m_location_pixels);

    distance_valid = transformer.isDistanceToPointValid();
    if(distance_valid)
        d2p = transformer.distanceToPoint(radial.x, radial.y);
    
    m_spherical_position.x = d2p;       //distance
    m_spherical_position.y = radial.x;  //bearing
    m_spherical_position.z = radial.y;  //elevation
    
    m_location_angular = Vector2<float>(radial.x, radial.y);
    //m_spherical_error - not calculated

    #if VISION_FIELDOBJECT_VERBOSITY > 2
        debug << "Obstacle::calculatePositions: " << m_spherical_position << endl;
    #endif

    return distance_valid && d2p > 0;
}

void Obstacle::render(cv::Mat &mat) const
{
    Vector2<double> half = m_size_on_screen*0.5;
    cv::line(mat, cv::Point2i(m_location_pixels.x-half.x, m_location_pixels.y-m_size_on_screen.y), cv::Point2i(m_location_pixels.x-half.x, m_location_pixels.y), cv::Scalar(255, 255, 0));
    cv::line(mat, cv::Point2i(m_location_pixels.x-half.x, m_location_pixels.y), cv::Point2i(m_location_pixels.x+half.x, m_location_pixels.y), cv::Scalar(255, 255, 0));
    cv::line(mat, cv::Point2i(m_location_pixels.x+half.x, m_location_pixels.y), cv::Point2i(m_location_pixels.x+half.x, m_location_pixels.y-m_size_on_screen.y), cv::Scalar(255, 255, 0));
}

/*! @brief Stream insertion operator for a single ColourSegment.
 *      The segment is terminated by a newline.
 */
ostream& operator<< (ostream& output, const Obstacle& o)
{
    output << "Obstacle - pixelloc: [" << o.getLocationPixels().x << ", " << o.getLocationPixels().y << "]";
    output << " angularloc: [" << o.getLocationAngular().x << ", " << o.getLocationAngular().y << "]";
    output << " relative field coords: [" << o.getRelativeFieldCoords().x << ", " << o.getRelativeFieldCoords().y << ", " << o.getRelativeFieldCoords().z << "]";
    return output;
}

/*! @brief Stream insertion operator for a vector of ColourSegments.
 *      Each segment is terminated by a newline.
 *  @relates ColourSegment
 */
ostream& operator<< (ostream& output, const vector<Obstacle>& o)
{
    for (size_t i=0; i<o.size(); i++)
        output << o[i] << endl;
    return output;
}
