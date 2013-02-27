//#include "beacon.h"
//#include "debug.h"
//#include "debugverbosityvision.h"

//#include "Vision/visionconstants.h"
//#include "Vision/visionblackboard.h"

//#include "Kinematics/Kinematics.h"
//#include "Tools/Math/Matrix.h"

//Beacon::Beacon(VFO_ID id, const Quad &corners)
//{
//    m_id = id;
//    m_corners = corners;
    
//    //    if(VisionConstants::DO_RADIAL_CORRECTION) {
//    //        VisionBlackboard* vbb = VisionBlackboard::getInstance();
//    //        Vector2<float> corr_bottom_centre = vbb->correctDistortion(Vector2<float>(m_bottom_centre.x, m_bottom_centre.y));
//    //        m_bottom_centre.x = mathGeneral::roundNumberToInt(corr_bottom_centre.x);
//    //        m_bottom_centre.y = mathGeneral::roundNumberToInt(corr_bottom_centre.y);
//    //    }

//    m_size_on_screen = Vector2<int>(corners.getWidth(), corners.getHeight());
//    m_location_pixels = corners.getBottomCentre();
//    //CALCULATE DISTANCE AND BEARING VALS
//    valid = calculatePositions();

//    //valid = valid && check(); //this must be last
//    valid = check();
//}

//const Quad& Beacon::getQuad() const
//{
//    return m_corners;
//}

//bool Beacon::addToExternalFieldObjects(FieldObjects *fieldobjects, float timestamp) const
//{
//    #if VISION_FIELDOBJECT_VERBOSITY > 1
//        debug << "Beacon::addToExternalFieldObjects - m_id: " << VFOName(m_id) << endl;
//        debug << "    " << *this << endl;
//    #endif
        
//    if(valid) {
//        #if VISION_FIELDOBJECT_VERBOSITY > 1
//            debug << "Beacon::addToExternalFieldObjects - valid" << endl;
//        #endif
//        AmbiguousObject newAmbObj;
//        FieldObjects::StationaryFieldObjectID stat_id;
//        bool stationary = false;

//        switch(m_id) {
//        case BEACON_Y:
//            stat_id = FieldObjects::FO_YELLOW_BEACON;
//            stationary = true;
//            break;
//        case BEACON_B:
//            stat_id = FieldObjects::FO_BLUE_BEACON;
//            stationary = true;
//            break;
//        case BEACON_U:
//            newAmbObj = AmbiguousObject(FieldObjects::FO_BEACON_UNKNOWN, "Unknown Beacon");
//            newAmbObj.addPossibleObjectID(FieldObjects::FO_YELLOW_BEACON);
//            newAmbObj.addPossibleObjectID(FieldObjects::FO_BLUE_BEACON);
//            stationary = false;
//            break;
//        default:
//            //invalid object - do not push to fieldobjects
//            errorlog << "Beacon::addToExternalFieldObjects - attempt to add invalid Beacon object" << endl;
//            #if VISION_FIELDOBJECT_VERBOSITY > 1
//                debug << "Beacon::addToExternalFieldObjects - attempt to add invalid Beacon object" << endl;
//            #endif
//            return false;
//        }

//        if(stationary) {
//            //add Beacon to stationaryFieldObjects
//            fieldobjects->stationaryFieldObjects[stat_id].UpdateVisualObject(m_transformed_spherical_pos,
//                                                                            m_spherical_error,
//                                                                            m_location_angular,
//                                                                            m_location_pixels,
//                                                                            m_size_on_screen,
//                                                                            timestamp);
//        }
//        else {
//            //update ambiguous Beacon and add it to ambiguousFieldObjects
//            newAmbObj.UpdateVisualObject(m_transformed_spherical_pos,
//                                         m_spherical_error,
//                                         m_location_angular,
//                                         m_location_pixels,
//                                         m_size_on_screen,
//                                         timestamp);
//            fieldobjects->ambiguousFieldObjects.push_back(newAmbObj);
//        }

//        return true;
//    }
//    else {
//        #if VISION_FIELDOBJECT_VERBOSITY > 1
//            debug << "Beacon::addToExternalFieldObjects - invalid" << endl;
//        #endif
//        return false;
//    }
//}

//bool Beacon::check() const
//{
////    if(!distance_valid) {
////        #if VISION_FIELDOBJECT_VERBOSITY > 1
////            debug << "Beacon::check - Beacon thrown out: distance invalid" << endl;
////        #endif
////        return false;
////    }

//    //throwout for base below horizon
//    if(VisionConstants::THROWOUT_ON_ABOVE_KIN_HOR_BEACONS and
//       not VisionBlackboard::getInstance()->getKinematicsHorizon().IsBelowHorizon(m_location_pixels.x, m_location_pixels.y)) {
//        #if VISION_FIELDOBJECT_VERBOSITY > 1
//            debug << "Beacon::check - Beacon thrown out: base above kinematics horizon" << endl;
//        #endif
//        return false;
//    }

//    //Distance discrepency throwout - if width method says Beacon is a lot closer than d2p (by specified value) then discard
//    if(VisionConstants::THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BEACONS and
//            width_dist + VisionConstants::MAX_DISTANCE_METHOD_DISCREPENCY_BEACONS < d2p) {
//        #if VISION_FIELDOBJECT_VERBOSITY > 1
//        debug << "Beacon::check - Beacon thrown out: width distance too much smaller than d2p" << endl;
//            debug << "\td2p: " << d2p << " width_dist: " << width_dist << " MAX_DISTANCE_METHOD_DISCREPENCY_BEACONS: " << VisionConstants::MAX_DISTANCE_METHOD_DISCREPENCY_BEACONS << endl;
//        #endif
//        return false;
//    }

//    //throw out if Beacon is too far away
//    if(VisionConstants::THROWOUT_DISTANT_BEACONS and
//        m_transformed_spherical_pos.x > VisionConstants::MAX_BEACON_DISTANCE) {
//        #if VISION_FIELDOBJECT_VERBOSITY > 1
//            debug << "Beacon::check - Beacon thrown out: too far away" << endl;
//            debug << "\td2p: " << m_transformed_spherical_pos.x << " MAX_BEACON_DISTANCE: " << VisionConstants::MAX_BEACON_DISTANCE << endl;
//        #endif
//        return false;
//    }

//    //all checks passed
//    return true;
//}

//void Beacon::setUnknown()
//{
//    m_id = BEACON_U;
//}

//bool Beacon::calculatePositions()
//{
//    VisionBlackboard* vbb = VisionBlackboard::getInstance();
//    //To the bottom of the Goal Post.
//    bool transform_valid;
//    float bearing = (float)vbb->calculateBearing(m_location_pixels.x);
//    float elevation = (float)vbb->calculateElevation(m_location_pixels.y);
    
//    float distance = distanceToBeacon(bearing, elevation);

//    if(distance <= 0) {
//        //object behind us - ignore it
//        m_spherical_position = Vector3<float>(0,0,0);//distance
//        m_location_angular = Vector2<float>(0,0);
//        m_transformed_spherical_pos = Vector3<float>(0,0,0);
//        return false;
//    }
//    //debug << "Goal::calculatePositions() distance: " << distance << endl;
    
//    m_spherical_position[0] = distance;//distance
//    m_spherical_position[1] = bearing;
//    m_spherical_position[2] = elevation;
    
//    m_location_angular = Vector2<float>(bearing, elevation);
//    //m_spherical_error - not calculated
        
////    if(vbb->isCameraTransformValid()) {
////        Matrix cameraTransform = Matrix4x4fromVector(vbb->getCameraTransformVector());
////        m_transformed_spherical_pos = Kinematics::TransformPosition(cameraTransform,m_spherical_position);
////    }
//    if(vbb->isCameraToGroundValid()) {
//        Matrix cameraToGroundTransform = Matrix4x4fromVector(vbb->getCameraToGroundVector());
//        m_transformed_spherical_pos = Kinematics::TransformPosition(cameraToGroundTransform,m_spherical_position);
//        transform_valid = true;
//    }
//    else {
//        transform_valid = false;
//        m_transformed_spherical_pos = Vector3<float>(0,0,0);
//        #if VISION_FIELDOBJECT_VERBOSITY > 1
//            debug << "Beacon::calculatePositions: Kinematics CTG transform invalid - will not push beacon" << endl;
//        #endif
//    }
    
//    #if VISION_FIELDOBJECT_VERBOSITY > 2
//        debug << "Beacon::calculatePositions: ";
//        debug << d2p << " " << width_dist << " " << distance << " " << m_transformed_spherical_pos.x << endl;
//    #endif

//    return transform_valid;
//}

///*!
//*   @brief Calculates the distance using the set METHOD and the provided coordinate angles.
//*   @param bearing The angle about the z axis.
//*   @param elevation The angle about the y axis.
//*/
//float Beacon::distanceToBeacon(float bearing, float elevation) {
//    VisionBlackboard* vbb = VisionBlackboard::getInstance();
//    //reset distance values
//    bool d2pvalid = false;
//    d2p = 0,
//    width_dist = 0;
//    //get distance to point from base
////    if(vbb->isCameraToGroundValid())
////    {
////        d2pvalid = true;
////        Matrix camera2groundTransform = Matrix4x4fromVector(vbb->getCameraToGroundVector());
////        Vector3<float> result = Kinematics::DistanceToPoint(camera2groundTransform, bearing, elevation);
////        d2p = result[0];
////    }
//    d2pvalid = vbb->distanceToPoint(bearing, elevation, d2p);

//    #if VISION_FIELDOBJECT_VERBOSITY > 1
//        if(!d2pvalid)
//            debug << "Beacon::distanceToBeacon: d2p invalid - combination methods will only return width_dist" << endl;
//    #endif
//    //get distance from width
//    width_dist = VisionConstants::BEACON_WIDTH*vbb->getCameraDistanceInPixels()/m_size_on_screen.x;

//    #if VISION_FIELDOBJECT_VERBOSITY > 1
//        debug << "Beacon::distanceToBeacon: bearing: " << bearing << " elevation: " << elevation << endl;
//        debug << "Beacon::distanceToBeacon: d2p: " << d2p << endl;
//        debug << "Beacon::distanceToBeacon: m_size_on_screen.x: " << m_size_on_screen.x << endl;
//        debug << "Beacon::distanceToBeacon: width_dist: " << width_dist << endl;
//    #endif
//    switch(VisionConstants::BEACON_DISTANCE_METHOD) {
//    case VisionConstants::D2P:
//        #if VISION_FIELDOBJECT_VERBOSITY > 1
//            debug << "Beacon::distanceToBeacon: Method: D2P" << endl;
//        #endif
//        distance_valid = d2pvalid;
//        return d2p;
//    case VisionConstants::Width:
//        #if VISION_FIELDOBJECT_VERBOSITY > 1
//            debug << "Beacon::distanceToBeacon: Method: Width" << endl;
//        #endif
//        distance_valid = true;
//        return width_dist;
//    case VisionConstants::Average:
//        #if VISION_FIELDOBJECT_VERBOSITY > 1
//            debug << "Beacon::distanceToBeacon: Method: Average" << endl;
//        #endif
//        //average distances
//        distance_valid = true;
//        if(d2pvalid)
//            return (d2p + width_dist) * 0.5;
//        else
//            return width_dist;
//    case VisionConstants::Least:
//        #if VISION_FIELDOBJECT_VERBOSITY > 1
//            debug << "Beacon::distanceToBeacon: Method: Least" << endl;
//        #endif
//        distance_valid = true;
//        if(d2pvalid)
//            return min(d2p, width_dist);
//        else
//            return width_dist;
//    }
//}

//void Beacon::render(cv::Mat &mat) const
//{
//    cv::Rect r(m_location_pixels.x - 0.5*m_size_on_screen.x, m_location_pixels.y-m_size_on_screen.y, m_size_on_screen.x, m_size_on_screen.y);
//    switch(m_id) {
//    case BEACON_Y:
//        cv::rectangle(mat, r, cv::Scalar(0, 255, 255));
//        break;
//    case BEACON_B:
//        cv::rectangle(mat, r, cv::Scalar(255, 0, 0));
//        break;
//    case BEACON_U:
//        cv::rectangle(mat, r, cv::Scalar(255, 255, 255));
//        break;
//    }

//}

///*! @brief Stream insertion operator for a single ColourSegment.
// *      The segment is terminated by a newline.
// */
//ostream& operator<< (ostream& output, const Beacon& b)
//{
//    output << "Beacon - pixelloc: [" << b.getLocationPixels().x << ", " << b.getLocationPixels().y << "]";
//    output << "\tpixelloc: [" << b.m_location_pixels.x << ", " << b.m_location_pixels.y << "]" << endl;
//    output << " angularloc: [" << b.m_location_angular.x << ", " << b.m_location_angular.y << "]" << endl;
//    output << "\trelative field coords: [" << b.m_spherical_position.x << ", " << b.m_spherical_position.y << ", " << b.m_spherical_position.z << "]" << endl;
//    output << "\ttransformed field coords: [" << b.m_transformed_spherical_pos.x << ", " << b.m_transformed_spherical_pos.y << ", " << b.m_transformed_spherical_pos.z << "]" << endl;
//    output << "\tspherical error: [" << b.m_spherical_error.x << ", " << b.m_spherical_error.y << "]" << endl;
//    output << "\tsize on screen: [" << b.m_size_on_screen.x << ", " << b.m_size_on_screen.y << "]";
//    return output;
//}

///*! @brief Stream insertion operator for a vector of ColourSegments.
// *      Each segment is terminated by a newline.
// *  @relates ColourSegment
// */
//ostream& operator<< (ostream& output, const vector<Beacon>& b)
//{
//    for (size_t i=0; i<b.size(); i++)
//        output << b[i] << endl;
//    return output;
//}
