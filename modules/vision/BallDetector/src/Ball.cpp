#include "Ball.h"

Ball::Ball() {
    m_id = BALL;
    m_diameter = 0;
    valid = (calculatePositions() && check());
}

Ball::Ball(arma::vec2 centre, double diameter) {
    m_id = BALL;

    double top = centre[1] - (diameter * 0.5);
    double bottom = centre[1] + (diameter * 0.5);
    double left = centre[0] - (diameter * 0.5);
    double right = centre[0] + (diameter * 0.5);

    arma::vec2 topPoint = arma::vec2({(right - left) * 0.5, top});
    arma::vec2 bottomPoint = arma::vec2({(right - left) * 0.5, bottom});
    arma::vec2 rightPoint = arma::vec2({right, (bottom - top) * 0.5});
    arma::vec2 leftPoint = arma::vec2({left, (bottom - top) * 0.5)};

    //    if(VisionConstants::DO_RADIAL_CORRECTION) {
    //        VisionBlackboard* vbb = VisionBlackboard::getInstance();
    //        top_pt = vbb->correctDistortion(top_pt);
    //        bottom_pt = vbb->correctDistortion(bottom_pt);
    //        right_pt = vbb->correctDistortion(right_pt);
    //        left_pt = vbb->correctDistortion(left_pt);
    //        centre_pt = vbb->correctDistortion(centre_pt);
    //    }
        
    m_diameter = std::max(bottomPoint[1] - topPoint[1], rightPoint[0] - leftPoint[0]);
    m_location.screenCartesian = centre;
    m_sizeOnScreen = arma::vec2({m_diameter, m_diameter});
    valid = (calculatePositions() && check());
}

float Ball::getRadius() const {
    return (m_diameter * 0.5);
}

bool Ball::addToExternalFieldObjects(FieldObjects *fieldobjects, float timestamp) const {
    if (valid) {
        // Add ball to mobileFieldObjects.
        fieldobjects->mobileFieldObjects[FieldObjects::FO_BALL].UpdateVisualObject(m_location.neckRelativeRadial, m_sphericalError,
                                                                                   m_location.screenAngular, m_location.screenCartesian, m_sizeOnScreen,
                                                                                   timestamp);

        return true;
    }

    else {
        return false;
    }
}

bool Ball::check() const {
    // Various throwouts here.

    // Throwout for below horizon.
    if(VisionConstants::THROWOUT_ON_ABOVE_KIN_HOR_BALL && !(VisionBlackboard::getInstance()->getKinematicsHorizon().IsBelowHorizon(m_location.screenCartesian[0], m_location.screenCartesian[1]))) {
        std::cout << "Ball::check() - Ball above horizon: should not occur" << std::endl;

        return false;
    }
    
    //Distance discrepency throwout - if width method says ball is a lot closer than d2p (by specified value) then discard
//    if(VisionConstants::THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BALL and
//            std::abs(width_dist - d2p) > VisionConstants::MAX_DISTANCE_METHOD_DISCREPENCY_BALL) {
//        #if VISION_BALL_VERBOSITY > 1
//        debug << "Ball::check - Ball thrown out: width distance too much smaller than d2p" << std::endl;
//            debug << "\td2p: " << d2p << " width_dist: " << width_dist << " MAX_DISTANCE_METHOD_DISCREPENCY_BALL: " << VisionConstants::MAX_DISTANCE_METHOD_DISCREPENCY_BALL << std::endl;
//        #endif
//        return false;
//    }

    // Throw out if ball is too small.
    if (VisionConstants::THROWOUT_SMALL_BALLS && (m_diameter < VisionConstants::MIN_BALL_DIAMETER_PIXELS)) {
        return false;
    }
    
    // Throw out if ball is too far away.
    if (VisionConstants::THROWOUT_DISTANT_BALLS && (m_location.neckRelativeRadial[0] > VisionConstants::MAX_BALL_DISTANCE)) {
        return false;
    }
    
    // All checks passed, keep ball.
    return true;
}

double Ball::findScreenError(VisionFieldObject *other) const {
    Ball* b = dynamic_cast<Ball*>(other);

    return ((arma::norm(m_location.screenCartesian - b->m_location.screenCartesian, 2) + amra::norm(m_sizeOnScreen - b->m_sizeOnScreen, 2));
}

double Ball::findGroundError(VisionFieldObject *other) const {
    Ball* b = dynamic_cast<Ball*>(other);

    return arma::norm(m_location.groundCartesian - b->m_location.groundCartesian, 2);
}

bool Ball::calculatePositions() {
    const Transformer& transformer = VisionBlackboard::getInstance()->getTransformer();

    // To the bottom of the Goal Post.
    NUPoint d2pLocation, widthLocation;

    d2pLocation.screenCartesian = m_location.screenCartesian;
    widthLocation.screenCartesian = m_location.screenCartesian;

    double widthDistance = ((VisionConstants::BALL_WIDTH * transformer.getCameraDistanceInPixels()) / (m_sizeOnScreen[0]));

    transformer.calculateRepresentationsFromPixelLocation(d2pLocation);
    transformer.calculateRepresentationsFromPixelLocation(widthLocation, true, widthDistance);

    switch(VisionConstants::BALL_DISTANCE_METHOD) {
        case D2P: {
            m_location = d2pLocation;

            break;
        }

        case Width: {
            m_location = widthLocation;

            break;
        }

        case Average: {
            //average distances
            m_location.screenCartesian      = (d2pLocation.screenCartesian + widthLocation.screenCartesian) * 0.5;
            m_location.neckRelativeRadial   = (d2pLocation.neckRelativeRadial + widthLocation.neckRelativeRadial) * 0.5;
            m_location.screenAngular        = (d2pLocation.screenAngular + widthLocation.screenAngular) * 0.5;
            m_location.groundCartesian      = (d2pLocation.groundCartesian + widthLocation.groundCartesian) * 0.5;

            break;
        }

        case Least: {
            m_location = ((d2pLocation.neckRelativeRadial[0] < widthLocation.neckRelativeRadial[0]) ? d2pLocation : widthLocation);

            break;
        }
    }

    return (m_location.neckRelativeRadial[0] > 0);
}


/*!
*   @brief Calculates the distance using the set METHOD and the provided coordinate angles.
*   @param bearing The angle about the z axis.
*   @param elevation The angle about the y axis.
*/
//double Ball::distanceToBall(double bearing, double elevation) {
//    const Transformer& tran = VisionBlackboard::getInstance()->getTransformer();
//    //reset distance values
//    bool d2pvalid = false;
//    d2p = 0;
//    width_dist = 0;
//    double result = 0;
//    //get distance to point from base

//    d2pvalid = tran.isDistanceToPointValid();
//    if(d2pvalid)
//        d2p = tran.distanceToPoint(bearing, elevation);

//    #if VISION_BALL_VERBOSITY > 1
//        if(!d2pvalid)
//            debug << "Ball::distanceToGoal: d2p invalid - combination methods will only return width_dist" << std::endl;
//    #endif
//    //get distance from width
//    width_dist = VisionConstants::BALL_WIDTH*tran.getCameraDistanceInPixels()/m_sizeOnScreen[0];

//    #if VISION_BALL_VERBOSITY > 1
//        debug << "Ball::distanceToGoal: bearing: " << bearing << " elevation: " << elevation << std::endl;
//        debug << "Ball::distanceToGoal: d2p: " << d2p << std::endl;
//        debug << "Ball::distanceToGoal: m_sizeOnScreen[0]: " << m_sizeOnScreen[0] << std::endl;
//        debug << "Ball::distanceToGoal: width_dist: " << width_dist << std::endl;
//        debug << "Ball::distanceToGoal: Method: " << getDistanceMethodName(VisionConstants::BALL_DISTANCE_METHOD) << std::endl;
//    #endif
//    switch(VisionConstants::BALL_DISTANCE_METHOD) {
//    case D2P:
//        distance_valid = d2pvalid && d2p > 0;
//        result = d2p;
//        break;
//    case Width:
//        distance_valid = true;
//        result = width_dist;
//        break;
//    case Average:
//        //average distances
//        distance_valid = d2pvalid && d2p > 0;
//        result = (d2p + width_dist) * 0.5;
//        break;
//    case Least:
//        distance_valid = d2pvalid && d2p > 0;
//        result = (distance_valid ? std::min(d2p, width_dist) : width_dist);
//        break;
//    }

//    return result;
//}

std::ostream& operator<< (std::ostream& output, const Ball& b) {
    output << "Ball " << std::endl;
    output << "\tpixelloc: [" << b.m_location.screenCartesian[0] << ", " << b.m_location.screenCartesian[1] << "]" << std::endl;
    output << " angularloc: [" << b.m_location.screenAngular[0] << ", " << b.m_location.screenAngular[1] << "]" << std::endl;
    output << "\trelative field coords: [" << b.m_location.neckRelativeRadial[0] << ", " << b.m_location.neckRelativeRadial[1] << ", " << b.m_location.neckRelativeRadial[2] << "]" << std::endl;
    output << "\tspherical error: [" << b.m_spherical_error[0] << ", " << b.m_spherical_error[1] << "]" << std::endl;
    output << "\tsize on screen: [" << b.m_sizeOnScreen[0] << ", " << b.m_sizeOnScreen[1] << "]";

    return output;
}

std::ostream& operator<< (std::ostream& output, const std::vector<Ball>& b) {
    for (const auto& ball : b) {
        output << ball << std::endl;
    }

    return output;
}
