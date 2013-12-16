#include "Obstacle.h"

using messages::vision;

Obstacle::Obstacle(arma::vec2 position, double width, double height, COLOUR_CLASS colour) {
    m_id = OBSTACLE;
    m_location.screenCartesian = position;
    m_sizeOnScreen = arma::vec2({width, height});
    m_colour = colour;

//    if(VisionConstants::DO_RADIAL_CORRECTION) {
//        VisionBlackboard* vbb = VisionBlackboard::getInstance();
//        Vector2<float> bottomcentre = Vector2<float>(position.x, position.y);

//        bottomcentre = vbb->correctDistortion(bottomcentre);

//        m_bottom_centre = Vector2<int>(bottomcentre.x, bottomcentre.y);
//    }

    //CALCULATE DISTANCE AND BEARING VALS
    valid = (calculatePositions() && check());

//    valid = calculatePositions();
//    valid = check();
}

bool Obstacle::addToExternalFieldObjects(FieldObjects *fieldobjects, float timestamp) const {
    if(valid) {
        AmbiguousObject newAmbObj = AmbiguousObject(FieldObjects::FO_OBSTACLE, "Unknown Obstacle");

        //newAmbObj.addPossibleObjectID(FieldObjects::FO_BLUE_ROBOT_UNKNOWN);
        newAmbObj.UpdateVisualObject(m_location.neckRelativeRadial, m_sphericalError,
                                     m_location.screenAngular, m_location.screenCartesian, m_sizeOnScreen,
                                     timestamp);

        newAmbObj.arcWidth = m_arcWidth;
        fieldobjects->ambiguousFieldObjects.push_back(newAmbObj);

        return true;
    }

    else {
        return false;
    }
}

bool Obstacle::check() const {
    //! TODO: Do a check based on width and d2p consistency
//    if(!distance_valid) {
//        #if VISION_OBSTACLE_VERBOSITY > 1
//            debug << "Obstacle::check - Obstacle thrown out: distance invalid" << std::endl;
//        #endif
//        return false;
//    }

    // All checks passed.
    return true;
}

double Obstacle::findScreenError(VisionFieldObject* other) const {
    Obstacle* o = dynamic_cast<Obstacle*>(other);

    return (arma::norm(m_location.screenCartesian - o->m_location.screenCartesian, 2) + arma::norm(m_size_on_screen - o->m_size_on_screen, 2));
}

double Obstacle::findGroundError(VisionFieldObject* other) const {
    Obstacle* o = dynamic_cast<Obstacle*>(other);
    double w = 2 * m_location.neckRelativeRadial[0] * tan(m_arcWidth * 0.5);              // w/2 = d * tan(theta/2)
    double w_o = 2 * o->m_location.neckRelativeRadial[0] * tan(o->m_arcWidth * 0.5);

    return (arma::norm(m_location.groundCartesian - o->m_location.groundCartesian, 2)) + abs(w - w_o);
}

bool Obstacle::calculatePositions() {
    const Transformer& transformer = VisionBlackboard::getInstance()->getTransformer();

    //To the bottom of the Goal Post.
    transformer.calculateRepresentationsFromPixelLocation(m_location);

    // find arc width
    NUPoint gp1, gp2;

    gp1.screenCartesian = m_location.screenCartesian - arma::vec2({m_sizeOnScreen[0], 0});
    gp2.screenCartesian = m_location.screenCartesian + arma::vec2({m_sizeOnScreen[0], 0});

    transformer.calculateRepresentationsFromPixelLocation(gp1);
    transformer.calculateRepresentationsFromPixelLocation(gp2);

    m_arcWidth = std::abs(gp1.screenAngular[0] - gp2.screenAngular[0]);

    return (m_location.neckRelativeRadial[0] > 0);
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
std::ostream& operator<< (std::ostream& output, const Obstacle& obstacle) {
    output << "Obstacle" << std::endl;
    output << "\tpixelloc: " << obstacle.m_location.screenCartesian << std::endl;
    output << "\tangularloc: " << obstacle.m_location.screenAngular << std::endl;
    output << "\trelative field coords: " << obstacle.m_location.neckRelativeRadial << std::endl;

    return output;
}

/*! @brief Stream insertion operator for a vector of ColourSegments.
 *      Each segment is terminated by a newline.
 *  @relates ColourSegment
 */
std::ostream& operator<< (std::ostream& output, const std::vector<Obstacle>& obstacles) {
    for (const auto& obstacle ; obstacles) {
        output << obstacle << std::endl;
    }

    return output;
}
