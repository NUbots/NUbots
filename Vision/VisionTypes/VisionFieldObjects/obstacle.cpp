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
    m_location.screenCartesian = position;
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
    debug << "Obstacle::addToExternalFieldObjects" << std::endl;
    debug << "    " << *this << std::endl;
#endif
if(valid) {
    #if VISION_OBSTACLE_VERBOSITY > 1
        debug << "Obstacle::addToExternalFieldObjects - valid" << std::endl;
    #endif
    AmbiguousObject newAmbObj = AmbiguousObject(FieldObjects::FO_OBSTACLE, "Unknown Obstacle");
    //newAmbObj.addPossibleObjectID(FieldObjects::FO_BLUE_ROBOT_UNKNOWN);
    newAmbObj.UpdateVisualObject(Vector3<float>(m_location.neckRelativeRadial.x, m_location.neckRelativeRadial.y, m_location.neckRelativeRadial.z),
                                 Vector3<float>(m_spherical_error.x, m_spherical_error.y, m_spherical_error.z),
                                 Vector2<float>(m_location.screenAngular.x, m_location.screenAngular.y),
                                 Vector2<int>(m_location.screenCartesian.x,m_location.screenCartesian.y),
                                 Vector2<int>(m_size_on_screen.x,m_size_on_screen.y),
                                 timestamp);

    newAmbObj.arc_width = m_arc_width;

    fieldobjects->ambiguousFieldObjects.push_back(newAmbObj);
    return true;
}
else {
    #if VISION_OBSTACLE_VERBOSITY > 1
        debug << "Obstacle::addToExternalFieldObjects - invalid" << std::endl;
    #endif
    return false;
}
}

bool Obstacle::check() const
{
    //! @todo Do a check based on width and d2p consistency
//    if(!distance_valid) {
//        #if VISION_OBSTACLE_VERBOSITY > 1
//            debug << "Obstacle::check - Obstacle thrown out: distance invalid" << std::endl;
//        #endif
//        return false;
//    }

    //all checks passed
    return true;
}

double Obstacle::findScreenError(VisionFieldObject* other) const
{
    Obstacle* o = dynamic_cast<Obstacle*>(other);
    return ( m_location.screenCartesian - o->m_location.screenCartesian ).abs() + ( m_size_on_screen - o->m_size_on_screen ).abs();
}

double Obstacle::findGroundError(VisionFieldObject* other) const
{
    Obstacle* o = dynamic_cast<Obstacle*>(other);
    double w = 2 * m_location.neckRelativeRadial.x * tan(m_arc_width*0.5);              // w/2 = d * tan(theta/2)
    double w_o = 2 * o->m_location.neckRelativeRadial.x * tan(o->m_arc_width*0.5);

    return ( m_location.groundCartesian - o->m_location.groundCartesian ).abs() + abs( w - w_o );
}


bool Obstacle::calculatePositions()
{
    const Transformer& transformer = VisionBlackboard::getInstance()->getTransformer();
    //To the bottom of the Goal Post.
    transformer.calculateRepresentations(m_location);

    // find arc width
    NUPoint gp1, gp2;
    gp1.screenCartesian = m_location.screenCartesian - Point(m_size_on_screen.x, 0);
    gp2.screenCartesian = m_location.screenCartesian + Point(m_size_on_screen.x, 0);
    transformer.calculateRepresentations(gp1);
    transformer.calculateRepresentations(gp2);

    m_arc_width = abs( gp1.screenAngular.x - gp2.screenAngular.x );

    #if VISION_OBSTACLE_VERBOSITY > 2
        debug << "Obstacle::calculatePositions: " << m_location << std::endl;
    #endif

    return m_location.neckRelativeRadial.x > 0;
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
std::ostream& operator<< (std::ostream& output, const Obstacle& o)
{
    output << "Obstacle" << std::endl;
    output << "\tpixelloc: " << o.m_location.screenCartesian << std::endl;
    output << "\tangularloc: " << o.m_location.screenAngular << std::endl;
    output << "\trelative field coords: " << o.m_location.neckRelativeRadial << std::endl;
    return output;
}

/*! @brief Stream insertion operator for a vector of ColourSegments.
 *      Each segment is terminated by a newline.
 *  @relates ColourSegment
 */
std::ostream& operator<< (std::ostream& output, const std::vector<Obstacle>& o)
{
    for (size_t i=0; i<o.size(); i++)
        output << o[i] << std::endl;
    return output;
}
