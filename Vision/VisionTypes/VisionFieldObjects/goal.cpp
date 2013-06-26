#include "goal.h"
#include "Vision/visionblackboard.h"
#include "Vision/visionconstants.h"

#include "debug.h"
#include "debugverbosityvision.h"

#include "Kinematics/Kinematics.h"
#include "Tools/Math/Matrix.h"

Goal::Goal(VFO_ID id, const Quad &corners, bool known)
{
    m_id = id;
    m_corners = corners;
    m_known = known;

    m_location.screenCartesian = corners.getBottomCentre();
    m_size_on_screen = Vector2<double>(corners.getAverageWidth(), corners.getAverageHeight());

//    if(VisionConstants::DO_RADIAL_CORRECTION) {
//        VisionBlackboard* vbb = VisionBlackboard::getInstance();
//        Vector2<float> corr_bottom_centre = vbb->correctDistortion(Vector2<float>(m_bottom_centre.x, m_bottom_centre.y));
//        m_bottom_centre.x = mathGeneral::roundNumberToInt(corr_bottom_centre.x);
//        m_bottom_centre.y = mathGeneral::roundNumberToInt(corr_bottom_centre.y);
//    }

    //CALCULATE DISTANCE AND BEARING VALS
    valid = calculatePositions();
    valid = valid && check();
}

void Goal::setBase(Point base)
{
    Line l_line(m_corners.getBottomLeft(), m_corners.getTopLeft());
    Line r_line(m_corners.getBottomRight(), m_corners.getTopRight());

    Point l_point = l_line.projectOnto(base);
    Point r_point = r_line.projectOnto(base);

    m_location.screenCartesian = Point( (l_point.x + r_point.x)*0.5, base.y );

    valid = calculatePositions();
    valid = valid && check();
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
        debug << "Goal::addToExternalFieldObjects - m_id: " << VFOName(m_id) << std::endl;
        debug << "    " << *this << std::endl;
    #endif
    if(valid) {
        #if VISION_GOAL_VERBOSITY > 1
            debug << "Goal::addToExternalFieldObjects - valid" << std::endl;
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
        case GOAL_B_L:
            fieldobjects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST].UpdateVisualObject(Vector3<float>(m_location.neckRelativeRadial.x, m_location.neckRelativeRadial.y, m_location.neckRelativeRadial.z),
                                                                                                         Vector3<float>(m_spherical_error.x, m_spherical_error.y, m_spherical_error.z),
                                                                                                         Vector2<float>(m_location.screenAngular.x, m_location.screenAngular.y),
                                                                                                         Vector2<int>(m_location.screenCartesian.x,m_location.screenCartesian.y),
                                                                                                         Vector2<int>(m_size_on_screen.x,m_size_on_screen.y),
                                                                                                         timestamp);
            return true;
        case GOAL_B_R:
            fieldobjects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST].UpdateVisualObject(Vector3<float>(m_location.neckRelativeRadial.x, m_location.neckRelativeRadial.y, m_location.neckRelativeRadial.z),
                                                                                                         Vector3<float>(m_spherical_error.x, m_spherical_error.y, m_spherical_error.z),
                                                                                                         Vector2<float>(m_location.screenAngular.x, m_location.screenAngular.y),
                                                                                                         Vector2<int>(m_location.screenCartesian.x,m_location.screenCartesian.y),
                                                                                                         Vector2<int>(m_size_on_screen.x,m_size_on_screen.y),
                                                                                                          timestamp);
            return true;
        case GOAL_Y_L:
            fieldobjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST].UpdateVisualObject(Vector3<float>(m_location.neckRelativeRadial.x, m_location.neckRelativeRadial.y, m_location.neckRelativeRadial.z),
                                                                                                         Vector3<float>(m_spherical_error.x, m_spherical_error.y, m_spherical_error.z),
                                                                                                         Vector2<float>(m_location.screenAngular.x, m_location.screenAngular.y),
                                                                                                         Vector2<int>(m_location.screenCartesian.x,m_location.screenCartesian.y),
                                                                                                         Vector2<int>(m_size_on_screen.x,m_size_on_screen.y),
                                                                                                           timestamp);
            return true;
        case GOAL_Y_R:
            fieldobjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST].UpdateVisualObject(Vector3<float>(m_location.neckRelativeRadial.x, m_location.neckRelativeRadial.y, m_location.neckRelativeRadial.z),
                                                                                                         Vector3<float>(m_spherical_error.x, m_spherical_error.y, m_spherical_error.z),
                                                                                                         Vector2<float>(m_location.screenAngular.x, m_location.screenAngular.y),
                                                                                                         Vector2<int>(m_location.screenCartesian.x,m_location.screenCartesian.y),
                                                                                                         Vector2<int>(m_size_on_screen.x,m_size_on_screen.y),
                                                                                                         timestamp);
            return true;
        default:
            //invalid object - do not push to fieldobjects
            errorlog << "Goal::addToExternalFieldObjects - attempt to add invalid Goal object id: " << VFOName(m_id) << std::endl;
            #if VISION_GOAL_VERBOSITY > 1
                debug << "Goal::addToExternalFieldObjects - attempt to add invalid Goal object id: " << VFOName(m_id) << std::endl;
            #endif
            return false;
        }

        //update ambiguous goal post and add it to ambiguousFieldObjects
        newAmbObj.UpdateVisualObject(Vector3<float>(m_location.neckRelativeRadial.x, m_location.neckRelativeRadial.y, m_location.neckRelativeRadial.z),
                                     Vector3<float>(m_spherical_error.x, m_spherical_error.y, m_spherical_error.z),
                                     Vector2<float>(m_location.screenAngular.x, m_location.screenAngular.y),
                                     Vector2<int>(m_location.screenCartesian.x,m_location.screenCartesian.y),
                                     Vector2<int>(m_size_on_screen.x,m_size_on_screen.y),
                                     timestamp);
        fieldobjects->ambiguousFieldObjects.push_back(newAmbObj);
        return true;
    }
    else {
        #if VISION_GOAL_VERBOSITY > 1
            debug << "Goal::addToExternalFieldObjects - invalid" << std::endl;
        #endif
        return false;
    }
}

bool Goal::check() const
{
    //various throwouts here
    if(VisionConstants::THROWOUT_SHORT_GOALS) {
        if(m_corners.getAverageHeight() <= VisionConstants::MIN_GOAL_HEIGHT) {
            #if VISION_GOAL_VERBOSITY > 1
                debug << "Goal::check - Goal thrown out: less than 20pix high" << std::endl;
            #endif
            return false;
        }
    }
    
    if(VisionConstants::THROWOUT_NARROW_GOALS) {
        if(m_corners.getAverageWidth() <= VisionConstants::MIN_GOAL_WIDTH) {
            #if VISION_GOAL_VERBOSITY > 1
                debug << "Goal::check - Goal thrown out: less than " << VisionConstants::MIN_GOAL_WIDTH << "pix high" << std::endl;
            #endif
            return false;
        }
    }

    //throwout for base below horizon
    if(VisionConstants::THROWOUT_ON_ABOVE_KIN_HOR_GOALS and
       not VisionBlackboard::getInstance()->getKinematicsHorizon().IsBelowHorizon(m_location.screenCartesian.x, m_location.screenCartesian.y)) {
        #if VISION_GOAL_VERBOSITY > 1
            debug << "Goal::check - Goal thrown out: base above kinematics horizon" << std::endl;
        #endif
        return false;
    }

    //throw out if goal is too far away
    if(VisionConstants::THROWOUT_DISTANT_GOALS and 
        m_location.neckRelativeRadial.x > VisionConstants::MAX_GOAL_DISTANCE) {
        #if VISION_GOAL_VERBOSITY > 1
            debug << "Goal::check - Goal thrown out: too far away" << std::endl;
            debug << "\td2p: " << m_location.neckRelativeRadial.x << " MAX_GOAL_DISTANCE: " << VisionConstants::MAX_GOAL_DISTANCE << std::endl;
        #endif
        return false;
    }
    
    //all checks passed, keep goalpost
    return true;
}

double Goal::findScreenError(VisionFieldObject* other) const
{
    Goal* g = dynamic_cast<Goal*>(other);
    return ( m_location.screenCartesian - g->m_location.screenCartesian ).abs() + ( m_size_on_screen - g->m_size_on_screen ).abs();
}

double Goal::findGroundError(VisionFieldObject* other) const
{
    Goal* g = dynamic_cast<Goal*>(other);
    return ( m_location.groundCartesian - g->m_location.groundCartesian ).abs();
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
    int img_width = VisionBlackboard::getInstance()->getImageWidth();
    int img_height = VisionBlackboard::getInstance()->getImageHeight();

    d2p_loc.screenCartesian = m_location.screenCartesian;
    width_loc.screenCartesian = m_location.screenCartesian;
    height_loc.screenCartesian = m_location.screenCartesian;

    //get distance from width
    width_dist = VisionConstants::GOAL_WIDTH*tran.getCameraDistanceInPixels()/m_size_on_screen.x;
    height_dist = VisionConstants::GOAL_HEIGHT*tran.getCameraDistanceInPixels()/m_size_on_screen.y;

    // D2P
    tran.calculateRepresentationsFromPixelLocation(d2p_loc);
    // WIDTH
    tran.calculateRepresentationsFromPixelLocation(width_loc, true, width_dist);
    // HEIGHT
    tran.calculateRepresentationsFromPixelLocation(height_loc, true, height_dist);


    // check if we are off the edge of the screen by a certain margin
    int EDGE_OF_SCREEN_MARGIN = 5;
    off_top = m_location.screenCartesian.y - m_size_on_screen.y < EDGE_OF_SCREEN_MARGIN;
    off_bottom = m_location.screenCartesian.y >= img_height - EDGE_OF_SCREEN_MARGIN;
    off_side = m_location.screenCartesian.x - 0.5 * m_size_on_screen.x <= EDGE_OF_SCREEN_MARGIN ||
                    m_location.screenCartesian.x + 0.5 * m_size_on_screen.x >= img_width - EDGE_OF_SCREEN_MARGIN;

    if(off_bottom && off_side)
    {
        // we can't tell distance to these goals
        m_location.neckRelativeRadial = Vector3<double>();
        return false;
    }
    else if(off_bottom || off_top)
    {
        // we can only use width
        m_location = width_loc;
    }
    else if(off_side)
    {
        // we can only use d2p
        m_location.neckRelativeRadial = Vector3<double>();
        return false;
        //m_location = d2p_loc;
    }
    else
    {
        // use method of choice
        switch(VisionConstants::GOAL_DISTANCE_METHOD) {
            case D2P:
                m_location = d2p_loc;
                break;
            case Width:
                m_location = width_loc;
                break;
            case Average:
                //average distances
                m_location.screenCartesian = (d2p_loc.screenCartesian + width_loc.screenCartesian) * 0.5;
                m_location.neckRelativeRadial = (d2p_loc.neckRelativeRadial + width_loc.neckRelativeRadial) * 0.5;
                m_location.screenAngular = (d2p_loc.screenAngular + width_loc.screenAngular) * 0.5;
                m_location.groundCartesian = (d2p_loc.groundCartesian + width_loc.groundCartesian) * 0.5;
                break;
            case Least:
                m_location = (d2p_loc.neckRelativeRadial.x < width_loc.neckRelativeRadial.x ? d2p_loc : width_loc);
                break;
        }
    }

    #if VISION_GOAL_VERBOSITY > 2
        debug << "Goal::calculatePositions: " << m_location << std::endl;
    #endif

    return m_location.neckRelativeRadial.x > 0;
}

/*! @brief Stream insertion operator for a single ColourSegment.
 *      The segment is terminated by a newline.
 */
std::ostream& operator<< (std::ostream& output, const Goal& g)
{
    output << "Goal - " << VFOName(g.m_id) << std::endl;
    output << "\tpixelloc: " << g.m_location.screenCartesian << std::endl;
    output << "\tangularloc: " << g.m_location.screenAngular << std::endl;
    output << "\trelative field coords: " << g.m_location.neckRelativeRadial << std::endl;
    output << "\tspherical error: " << g.m_spherical_error << std::endl;
    output << "\tsize on screen: " << g.m_size_on_screen;
    return output;
}

/*! @brief Stream insertion operator for a std::vector of ColourSegments.
 *      Each segment is terminated by a newline.
 *  @relates ColourSegment
 */
std::ostream& operator<< (std::ostream& output, const std::vector<Goal>& g)
{
    for (size_t i=0; i<g.size(); i++)
        output << g[i] << std::endl;
    return output;
}
