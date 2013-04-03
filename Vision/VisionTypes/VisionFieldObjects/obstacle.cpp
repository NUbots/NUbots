#include "obstacle.h"
#include "Vision/visionblackboard.h"
#include "Vision/visionconstants.h"

#include "debug.h"
#include "debugverbosityvision.h"

#include "Kinematics/Kinematics.h"
#include "Tools/Math/Matrix.h"

Obstacle::Obstacle(Point position, double width, double height)
{
    m_id = OBSTACLE;
    m_location.screen = position;
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
#if VISION_OBSTACLE_VERBOSITY > 1
    debug << "Obstacle::addToExternalFieldObjects" << endl;
    debug << "    " << *this << endl;
#endif
if(valid) {
    #if VISION_OBSTACLE_VERBOSITY > 1
        debug << "Obstacle::addToExternalFieldObjects - valid" << endl;
    #endif
    VisionBlackboard* vbb = VisionBlackboard::getInstance();
    AmbiguousObject newAmbObj = AmbiguousObject(FieldObjects::FO_OBSTACLE, "Unknown Obstacle");
    //newAmbObj.addPossibleObjectID(FieldObjects::FO_BLUE_ROBOT_UNKNOWN);
    newAmbObj.UpdateVisualObject(Vector3<float>(m_location.relativeRadial.x, m_location.relativeRadial.y, m_location.relativeRadial.z),
                                 Vector3<float>(m_spherical_error.x, m_spherical_error.y, m_spherical_error.z),
                                 Vector2<float>(m_location.angular.x, m_location.angular.y),
                                 Vector2<int>(m_location.screen.x,m_location.screen.y),
                                 Vector2<int>(m_size_on_screen.x,m_size_on_screen.y),
                                 timestamp);

    newAmbObj.arc_width = m_arc_width;

    fieldobjects->ambiguousFieldObjects.push_back(newAmbObj);
    return true;
}
else {
    #if VISION_OBSTACLE_VERBOSITY > 1
        debug << "Obstacle::addToExternalFieldObjects - invalid" << endl;
    #endif
    return false;
}
}

bool Obstacle::check() const
{
    //! @todo Do a check based on width and d2p consistency
//    if(!distance_valid) {
//        #if VISION_OBSTACLE_VERBOSITY > 1
//            debug << "Obstacle::check - Obstacle thrown out: distance invalid" << endl;
//        #endif
//        return false;
//    }

    //all checks passed
    return true;
}

double Obstacle::findScreenError(VisionFieldObject* other) const
{
    Obstacle* o = dynamic_cast<Obstacle*>(other);
    return ( m_location.screen - o->m_location.screen ).abs() + ( m_size_on_screen - o->m_size_on_screen ).abs();
}

double Obstacle::findGroundError(VisionFieldObject* other) const
{
    Obstacle* o = dynamic_cast<Obstacle*>(other);
    double w = 2 * m_location.relativeRadial.x * tan(m_arc_width*0.5);              // w/2 = d * tan(theta/2)
    double w_o = 2 * o->m_location.relativeRadial.x * tan(o->m_arc_width*0.5);

    return ( m_location.ground - o->m_location.ground ).abs() + abs( w - w_o );
}


bool Obstacle::calculatePositions()
{
    const Transformer& transformer = VisionBlackboard::getInstance()->getTransformer();
    //To the bottom of the Goal Post.
    transformer.screenToRadial2D(m_location);

    distance_valid = transformer.isDistanceToPointValid();
    if(distance_valid) {
        d2p = transformer.distanceToPoint(m_location.angular.x, m_location.angular.y);
        transformer.screenToRadial3D(m_location, d2p);
    }

    // find arc width
    GroundPoint gp1, gp2;
    gp1 = transformer.screenToRadial2D(m_location.screen - Point(m_size_on_screen.x, 0));
    gp2 = transformer.screenToRadial2D(m_location.screen + Point(m_size_on_screen.x, 0));

    m_arc_width = abs( gp1.angular.x - gp2.angular.x );

    //m_spherical_error - not calculated

    #if VISION_OBSTACLE_VERBOSITY > 2
        debug << "Obstacle::calculatePositions: " << m_location.relativeRadial << endl;
    #endif

    return distance_valid && d2p > 0;
}

//void Obstacle::render(cv::Mat &mat) const
//{
//    Vector2<double> half = m_size_on_screen*0.5;
//    cv::line(mat, cv::Point2i(m_location_pixels.x-half.x, m_location_pixels.y-m_size_on_screen.y), cv::Point2i(m_location_pixels.x-half.x, m_location_pixels.y), cv::Scalar(255, 255, 0));
//    cv::line(mat, cv::Point2i(m_location_pixels.x-half.x, m_location_pixels.y), cv::Point2i(m_location_pixels.x+half.x, m_location_pixels.y), cv::Scalar(255, 255, 0));
//    cv::line(mat, cv::Point2i(m_location_pixels.x+half.x, m_location_pixels.y), cv::Point2i(m_location_pixels.x+half.x, m_location_pixels.y-m_size_on_screen.y), cv::Scalar(255, 255, 0));
//}

/*! @brief Stream insertion operator for a single ColourSegment.
 *      The segment is terminated by a newline.
 */
ostream& operator<< (ostream& output, const Obstacle& o)
{
    output << "Obstacle" << endl;
    output << "\tpixelloc: " << o.m_location.screen << endl;
    output << "\tangularloc: " << o.m_location.angular << endl;
    output << "\trelative field coords: " << o.m_location.relativeRadial << endl;
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
