#include "goal.h"
#include "Vision/visionblackboard.h"
#include "Vision/visionconstants.h"

#include "debug.h"
#include "debugverbosityvision.h"

#include "Kinematics/Kinematics.h"
#include "Tools/Math/Matrix.h"

Goal::Goal(VFO_ID id, const Quad &corners)
{
    m_id = id;
    m_corners = corners;

    m_location.screen = corners.getBottomCentre();
    m_size_on_screen = Vector2<double>(corners.getAverageWidth(), corners.getAverageHeight());

//    if(VisionConstants::DO_RADIAL_CORRECTION) {
//        VisionBlackboard* vbb = VisionBlackboard::getInstance();
//        Vector2<float> corr_bottom_centre = vbb->correctDistortion(Vector2<float>(m_bottom_centre.x, m_bottom_centre.y));
//        m_bottom_centre.x = mathGeneral::roundNumberToInt(corr_bottom_centre.x);
//        m_bottom_centre.y = mathGeneral::roundNumberToInt(corr_bottom_centre.y);
//    }

    //CALCULATE DISTANCE AND BEARING VALS
    valid = calculatePositions();
    //valid = valid && check();
    valid = check();
}

void Goal::setBase(Point base)
{
    m_location.screen = base;

    valid = calculatePositions();
    //valid = valid && check();
    valid = check();
}

const Quad& Goal::getQuad() const
{
    return m_corners;
}

/*!
*   @brief Updates the external field objects with this goal.
*   @param fieldobjects A pointer to the external field objects.
*   @param timestamp The current timestamp to apply to the field objects.
*
*   This method uses the id of the goal to determine where to place it, it also
*   includes no checks before placing them, and nor should it. For example, if this is 
*   called on multiple YellowLeftGoals then localisation will only see the last one.
*/
bool Goal::addToExternalFieldObjects(FieldObjects *fieldobjects, float timestamp) const
{
    #if VISION_GOAL_VERBOSITY > 1
        debug << "Goal::addToExternalFieldObjects - m_id: " << VFOName(m_id) << endl;
        debug << "    " << *this << endl;
    #endif
    if(valid) {
        #if VISION_GOAL_VERBOSITY > 1
            debug << "Goal::addToExternalFieldObjects - valid" << endl;
        #endif
        AmbiguousObject newAmbObj;

        switch(m_id) {
        case GOAL_L:
            newAmbObj = AmbiguousObject(FieldObjects::FO_YELLOW_GOALPOST_UNKNOWN, "Left Yellow Post");
            newAmbObj.addPossibleObjectID(FieldObjects::FO_YELLOW_LEFT_GOALPOST);
            newAmbObj.addPossibleObjectID(FieldObjects::FO_BLUE_LEFT_GOALPOST);
            break;
        case GOAL_R:
            newAmbObj = AmbiguousObject(FieldObjects::FO_YELLOW_GOALPOST_UNKNOWN, "Right Yellow Post");
            newAmbObj.addPossibleObjectID(FieldObjects::FO_YELLOW_RIGHT_GOALPOST);
            newAmbObj.addPossibleObjectID(FieldObjects::FO_BLUE_RIGHT_GOALPOST);
            break;
        case GOAL_U:
            newAmbObj = AmbiguousObject(FieldObjects::FO_YELLOW_GOALPOST_UNKNOWN, "Unknown Yellow Post");
            newAmbObj.addPossibleObjectID(FieldObjects::FO_YELLOW_LEFT_GOALPOST);
            newAmbObj.addPossibleObjectID(FieldObjects::FO_YELLOW_RIGHT_GOALPOST);
            newAmbObj.addPossibleObjectID(FieldObjects::FO_BLUE_LEFT_GOALPOST);
            newAmbObj.addPossibleObjectID(FieldObjects::FO_BLUE_RIGHT_GOALPOST);
            break;
//        case GOAL_Y_L:
//            stat_id = FieldObjects::FO_YELLOW_LEFT_GOALPOST;
//            stationary = true;
//            break;
//        case GOAL_Y_R:
//            stat_id = FieldObjects::FO_YELLOW_RIGHT_GOALPOST;
//            stationary = true;
//            break;
//        case GOAL_Y_U:
//            newAmbObj = AmbiguousObject(FieldObjects::FO_YELLOW_GOALPOST_UNKNOWN, "Unknown Yellow Post");
//            newAmbObj.addPossibleObjectID(FieldObjects::FO_YELLOW_LEFT_GOALPOST);
//            newAmbObj.addPossibleObjectID(FieldObjects::FO_YELLOW_RIGHT_GOALPOST);
//            stationary = false;
//            break;
//        case GOAL_B_L:
//            stat_id = FieldObjects::FO_BLUE_LEFT_GOALPOST;
//            stationary = true;
//            break;
//        case GOAL_B_R:
//            stat_id = FieldObjects::FO_BLUE_RIGHT_GOALPOST;
//            stationary = true;
//            break;
//        case GOAL_B_U:
//            newAmbObj = AmbiguousObject(FieldObjects::FO_BLUE_GOALPOST_UNKNOWN, "Unknown Blue Post");
//            newAmbObj.addPossibleObjectID(FieldObjects::FO_BLUE_LEFT_GOALPOST);
//            newAmbObj.addPossibleObjectID(FieldObjects::FO_BLUE_RIGHT_GOALPOST);
//            stationary = false;
//            break;
        default:
            //invalid object - do not push to fieldobjects
            errorlog << "Goal::addToExternalFieldObjects - attempt to add invalid Goal object id: " << VFOName(m_id) << endl;
            #if VISION_GOAL_VERBOSITY > 1
                debug << "Goal::addToExternalFieldObjects - attempt to add invalid Goal object id: " << VFOName(m_id) << endl;
            #endif
            return false;
        }

        //update ambiguous goal post and add it to ambiguousFieldObjects
        newAmbObj.UpdateVisualObject(Vector3<float>(m_location.relativeRadial.x, m_location.relativeRadial.y, m_location.relativeRadial.z),
                                     Vector3<float>(m_spherical_error.x, m_spherical_error.y, m_spherical_error.z),
                                     Vector2<float>(m_location.angular.x, m_location.angular.y),
                                     Vector2<int>(m_location.screen.x,m_location.screen.y),
                                     Vector2<int>(m_size_on_screen.x,m_size_on_screen.y),
                                     timestamp);
        fieldobjects->ambiguousFieldObjects.push_back(newAmbObj);
        return true;
    }
    else {
        #if VISION_GOAL_VERBOSITY > 1
            debug << "Goal::addToExternalFieldObjects - invalid" << endl;
        #endif
        return false;
    }
}

bool Goal::check() const
{
    //various throwouts here

//    if(!distance_valid) {
//        #if VISION_GOAL_VERBOSITY > 1
//            debug << "Goal::check - Goal thrown out: distance invalid" << endl;
//        #endif
//        return false;
//    }

    if(VisionConstants::THROWOUT_SHORT_GOALS) {
        if(m_corners.getAverageHeight() <= VisionConstants::MIN_GOAL_HEIGHT) {
            #if VISION_GOAL_VERBOSITY > 1
                debug << "Goal::check - Goal thrown out: less than 20pix high" << endl;
            #endif
            return false;
        }
    }
    
    if(VisionConstants::THROWOUT_NARROW_GOALS) {
        if(m_corners.getAverageWidth() <= VisionConstants::MIN_GOAL_WIDTH) {
            #if VISION_GOAL_VERBOSITY > 1
                debug << "Goal::check - Goal thrown out: less than " << VisionConstants::MIN_GOAL_WIDTH << "pix high" << endl;
            #endif
            return false;
        }
    }

    //throwout for base below horizon
    if(VisionConstants::THROWOUT_ON_ABOVE_KIN_HOR_GOALS and
       not VisionBlackboard::getInstance()->getKinematicsHorizon().IsBelowHorizon(m_location.screen.x, m_location.screen.y)) {
        #if VISION_GOAL_VERBOSITY > 1
            debug << "Goal::check - Goal thrown out: base above kinematics horizon" << endl;
        #endif
        return false;
    }
    
    //Distance discrepency throwout - if width method says goal is a lot closer than d2p (by specified value) then discard
    if(VisionConstants::THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_GOALS and
            width_dist + VisionConstants::MAX_DISTANCE_METHOD_DISCREPENCY_GOALS < d2p) {
        #if VISION_GOAL_VERBOSITY > 1
        debug << "Goal::check - Goal thrown out: width distance too much smaller than d2p" << endl;
            debug << "\td2p: " << d2p << " width_dist: " << width_dist << " MAX_DISTANCE_METHOD_DISCREPENCY_GOALS: " << VisionConstants::MAX_DISTANCE_METHOD_DISCREPENCY_GOALS << endl;
        #endif
        return false;
    }
    
    //throw out if goal is too far away
    if(VisionConstants::THROWOUT_DISTANT_GOALS and 
        m_location.relativeRadial.x > VisionConstants::MAX_GOAL_DISTANCE) {
        #if VISION_GOAL_VERBOSITY > 1
            debug << "Goal::check - Goal thrown out: too far away" << endl;
            debug << "\td2p: " << m_location.relativeRadial.x << " MAX_GOAL_DISTANCE: " << VisionConstants::MAX_GOAL_DISTANCE << endl;
        #endif
        return false;
    }
    
    //all checks passed, keep goalpost
    return true;
}

/*!
*   @brief Updates the spherical position, angular location and transformed spherical position.
*   
*   This method uses the camera transform and as such if it was not valid when retrieved from the wrapper
*   this will leave m_transformed_spherical_position at all zeros.
*/
bool Goal::calculatePositions()
{
    const Transformer& tran = VisionBlackboard::getInstance()->getTransformer();
    //To the bottom of the Goal Post.
    tran.screenToRadial2D(m_location);
    double dist = distanceToGoal(m_location.angular.x, m_location.angular.y);

    tran.screenToRadial3D(m_location, dist);
    //m_spherical_error - not calculated

    #if VISION_GOAL_VERBOSITY > 2
        debug << "Goal::calculatePositions: ";
        debug << d2p << " " << width_dist << " " << m_location.relativeRadial.x << endl;
    #endif

    return distance_valid && dist > 0;
}

/*!
*   @brief Calculates the distance using the set METHOD and the provided coordinate angles.
*   @param bearing The angle about the z axis.
*   @param elevation The angle about the y axis.
*/
double Goal::distanceToGoal(double bearing, double elevation)
{
    const Transformer& tran = VisionBlackboard::getInstance()->getTransformer();
    //reset distance values
    bool d2pvalid = false;
    double result = d2p = width_dist = 0;
    //get distance to point from base
//    if(vbb->isCameraToGroundValid())
//    {
//        d2pvalid = true;
//        Matrix camera2groundTransform = Matrix4x4fromVector(vbb->getCameraToGroundVector());
//        Vector3<float> result = Kinematics::DistanceToPoint(camera2groundTransform, bearing, elevation);
//        d2p = result[0];
//    }
    d2pvalid = tran.isDistanceToPointValid();
    if(d2pvalid)
        d2p = tran.distanceToPoint(bearing, elevation);

    #if VISION_GOAL_VERBOSITY > 1
        if(!d2pvalid)
            debug << "Goal::distanceToGoal: d2p invalid - combination methods will only return width_dist" << endl;
    #endif
    //get distance from width
    width_dist = VisionConstants::GOAL_WIDTH*tran.getCameraDistanceInPixels()/m_size_on_screen.x;

    float HACKWIDTH = 50;
    float HACKPIXELS = 3;
    //HACK FOR GOALS AT BASE OF IMAGE
    if(m_size_on_screen.x > HACKWIDTH and (VisionBlackboard::getInstance()->getImageHeight() - m_location.screen.y) < HACKPIXELS) {
        #if VISION_GOAL_VERBOSITY > 1
            debug << "Goal::distanceToGoal: Goal wide and cutoff at bottom so used width_dist" << endl;
        #endif
        return width_dist;
    }

    #if VISION_GOAL_VERBOSITY > 1
        debug << "Goal::distanceToGoal: bearing: " << bearing << " elevation: " << elevation << endl;
        debug << "Goal::distanceToGoal: d2p: " << d2p << endl;
        debug << "Goal::distanceToGoal: m_size_on_screen.x: " << m_size_on_screen.x << endl;
        debug << "Goal::distanceToGoal: width_dist: " << width_dist << endl;
    #endif
    switch(VisionConstants::GOAL_DISTANCE_METHOD) {
    case D2P:
        #if VISION_GOAL_VERBOSITY > 1
            debug << "Goal::distanceToGoal: Method: D2P" << endl;
        #endif
        distance_valid = d2pvalid && d2p > 0;
        result = d2p;
        break;
    case Width:
        #if VISION_GOAL_VERBOSITY > 1
            debug << "Goal::distanceToGoal: Method: Width" << endl;
        #endif
        distance_valid = true;
        result = width_dist;
        break;
    case Average:
        #if VISION_GOAL_VERBOSITY > 1
            debug << "Goal::distanceToGoal: Method: Average" << endl;
        #endif
        //average distances
        distance_valid = d2pvalid && d2p > 0;
        result = (d2p + width_dist) * 0.5;
        break;
    case Least:
        #if VISION_GOAL_VERBOSITY > 1
            debug << "Goal::distanceToGoal: Method: Least" << endl;
        #endif
        distance_valid = d2pvalid && d2p > 0;
        result = (distance_valid ? min(d2p, width_dist) : width_dist);
        break;
    }

    return result;
}

/*! @brief Stream insertion operator for a single ColourSegment.
 *      The segment is terminated by a newline.
 */
ostream& operator<< (ostream& output, const Goal& g)
{
    output << "Goal - " << VFOName(g.m_id) << endl;
    output << "\tpixelloc: " << g.m_location.screen << endl;
    output << "\tangularloc: " << g.m_location.angular << endl;
    output << "\trelative field coords: " << g.m_location.relativeRadial << endl;
    output << "\tspherical error: " << g.m_spherical_error << endl;
    output << "\tsize on screen: " << g.m_size_on_screen;
    return output;
}

/*! @brief Stream insertion operator for a vector of ColourSegments.
 *      Each segment is terminated by a newline.
 *  @relates ColourSegment
 */
ostream& operator<< (ostream& output, const vector<Goal>& g)
{
    for (size_t i=0; i<g.size(); i++)
        output << g[i] << endl;
    return output;
}
