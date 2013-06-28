#include "visionconstants.h"

#include "nubotdataconfig.h"
#include "debug.h"
#include "debugverbosityvision.h"

#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/GameInformation/GameInformation.h"
#include "Tools/Math/General.h"

#include <fstream>
#include <boost/algorithm/string.hpp>

//HACK FOR RC2013
int VisionConstants::WHITE_SIDE_IS_BLUE;
bool VisionConstants::NON_WHITE_SIDE_CHECK;
int VisionConstants::UPPER_WHITE_THRESHOLD;
int VisionConstants::LOWER_WHITE_THRESHOLD;
//! Distortion Correction
bool VisionConstants::DO_RADIAL_CORRECTION;
float VisionConstants::RADIAL_CORRECTION_COEFFICIENT;
//! Goal filtering constants
bool VisionConstants::THROWOUT_ON_ABOVE_KIN_HOR_GOALS;
bool VisionConstants::THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_GOALS;
float VisionConstants::MAX_DISTANCE_METHOD_DISCREPENCY_GOALS;
bool VisionConstants::THROWOUT_DISTANT_GOALS;
float VisionConstants::MAX_GOAL_DISTANCE;
bool VisionConstants::THROWOUT_INSIGNIFICANT_GOALS;
int VisionConstants::MIN_TRANSITIONS_FOR_SIGNIFICANCE_GOALS;
bool VisionConstants::THROWOUT_NARROW_GOALS;
int VisionConstants::MIN_GOAL_WIDTH;
bool VisionConstants::THROWOUT_SHORT_GOALS;
int VisionConstants::MIN_GOAL_HEIGHT;
float VisionConstants::GOAL_HEIGHT_TO_WIDTH_RATIO_MIN;
int VisionConstants::GOAL_MAX_OBJECTS;
int VisionConstants::GOAL_BINS;
int VisionConstants::GOAL_MIN_THRESHOLD;
float VisionConstants::GOAL_SDEV_THRESHOLD;
float VisionConstants::GOAL_RANSAC_MATCHING_TOLERANCE;
////! Beacon filtering constants
//bool VisionConstants::THROWOUT_ON_ABOVE_KIN_HOR_BEACONS;
//bool VisionConstants::THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BEACONS;
//float VisionConstants::MAX_DISTANCE_METHOD_DISCREPENCY_BEACONS;
//bool VisionConstants::THROWOUT_DISTANT_BEACONS;
//float VisionConstants::MAX_BEACON_DISTANCE;
//bool VisionConstants::THROWOUT_INSIGNIFICANT_BEACONS;
//int VisionConstants::MIN_TRANSITIONS_FOR_SIGNIFICANCE_BEACONS;
//! Ball filtering constants
bool VisionConstants::THROWOUT_ON_ABOVE_KIN_HOR_BALL;
bool VisionConstants::THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BALL;
float VisionConstants::MAX_DISTANCE_METHOD_DISCREPENCY_BALL;
bool VisionConstants::THROWOUT_SMALL_BALLS;
float VisionConstants::MIN_BALL_DIAMETER_PIXELS;
bool VisionConstants::THROWOUT_INSIGNIFICANT_BALLS;
int VisionConstants::MIN_TRANSITIONS_FOR_SIGNIFICANCE_BALL;
bool VisionConstants::THROWOUT_DISTANT_BALLS;
float VisionConstants::MAX_BALL_DISTANCE;
//! Distance calculation options
bool VisionConstants::D2P_INCLUDE_BODY_PITCH;
bool VisionConstants::BALL_DISTANCE_POSITION_BOTTOM;
//! Distance method options
DistanceMethod VisionConstants::BALL_DISTANCE_METHOD;
DistanceMethod VisionConstants::GOAL_DISTANCE_METHOD;
//DistanceMethod VisionConstants::BEACON_DISTANCE_METHOD;
LineDetectionMethod VisionConstants::LINE_METHOD;
//! Field-object detection constants
int VisionConstants::BALL_EDGE_THRESHOLD;
int VisionConstants::BALL_ORANGE_TOLERANCE;
float VisionConstants::BALL_MIN_PERCENT_ORANGE;
float VisionConstants::GOAL_MIN_PERCENT_YELLOW;
float VisionConstants::GOAL_MIN_PERCENT_BLUE;
//float VisionConstants::BEACON_MIN_PERCENT_YELLOW;
//float VisionConstants::BEACON_MIN_PERCENT_BLUE;
int VisionConstants::MIN_GOAL_SEPARATION;
//! Obstacle detection constants
int VisionConstants::MIN_DISTANCE_FROM_HORIZON;
int VisionConstants::MIN_CONSECUTIVE_POINTS;
//! Field dimension constants
float VisionConstants::GOAL_WIDTH;
float VisionConstants::GOAL_HEIGHT;
float VisionConstants::DISTANCE_BETWEEN_POSTS;
float VisionConstants::BALL_WIDTH;
float VisionConstants::CENTRE_CIRCLE_RADIUS;
//float VisionConstants::BEACON_WIDTH;
//! ScanLine options
unsigned int VisionConstants::HORIZONTAL_SCANLINE_SPACING;
unsigned int VisionConstants::VERTICAL_SCANLINE_SPACING;
unsigned int VisionConstants::GREEN_HORIZON_SCAN_SPACING;
unsigned int VisionConstants::GREEN_HORIZON_MIN_GREEN_PIXELS;
float VisionConstants::GREEN_HORIZON_UPPER_THRESHOLD_MULT;
//! Split and Merge constants
unsigned int VisionConstants::SAM_MAX_LINES;
float VisionConstants::SAM_SPLIT_DISTANCE;
unsigned int VisionConstants::SAM_MIN_POINTS_OVER;
unsigned int VisionConstants::SAM_MIN_POINTS_TO_LINE;
float VisionConstants::SAM_MAX_ANGLE_DIFF_TO_MERGE;
float VisionConstants::SAM_MAX_DISTANCE_TO_MERGE;
unsigned int VisionConstants::SAM_MIN_POINTS_TO_LINE_FINAL;
float VisionConstants::SAM_MIN_LINE_R2_FIT;
float VisionConstants::SAM_MAX_LINE_MSD;
bool VisionConstants::SAM_CLEAR_SMALL;
bool VisionConstants::SAM_CLEAR_DIRTY;
//! RANSAC constants
float VisionConstants::RANSAC_MAX_ANGLE_DIFF_TO_MERGE; //
float VisionConstants::RANSAC_MAX_DISTANCE_TO_MERGE; //

VisionConstants::VisionConstants()
{
}

/*! @brief Loads vision constants and options from the given file.
  * @param filename The name of the file (located in the Config directory).
  */
void VisionConstants::loadFromFile(std::string filename) 
{
    WHITE_SIDE_IS_BLUE = -1;
    NON_WHITE_SIDE_CHECK = false;
    UPPER_WHITE_THRESHOLD = 1000;
    LOWER_WHITE_THRESHOLD = 100;
    GOAL_WIDTH = 10;
    GOAL_HEIGHT = 90;
    DISTANCE_BETWEEN_POSTS = 160;
    BALL_WIDTH = 6.5;
    CENTRE_CIRCLE_RADIUS = 60;

    HORIZONTAL_SCANLINE_SPACING = 5; //defaults in case of bad file
    VERTICAL_SCANLINE_SPACING = 5;
    GREEN_HORIZON_SCAN_SPACING = 11;
    GREEN_HORIZON_MIN_GREEN_PIXELS = 5;
    GREEN_HORIZON_UPPER_THRESHOLD_MULT = 2.0;
    GOAL_HEIGHT_TO_WIDTH_RATIO_MIN = 1.5,
    MIN_GOAL_SEPARATION = 20;
    SAM_MAX_LINES = 100;
    SAM_SPLIT_DISTANCE = 1.0;
    SAM_MIN_POINTS_OVER = 2;
    SAM_MIN_POINTS_TO_LINE = 3;
    SAM_MAX_ANGLE_DIFF_TO_MERGE = 0.1;
    SAM_MAX_DISTANCE_TO_MERGE = 10;
    SAM_MIN_POINTS_TO_LINE_FINAL = 5;
    SAM_MIN_LINE_R2_FIT = 0.95;
    SAM_MAX_LINE_MSD = 1;
    SAM_CLEAR_SMALL = true;
    SAM_CLEAR_DIRTY = true;
    LINE_METHOD = RANSAC;
    GOAL_MAX_OBJECTS = 8;
    GOAL_BINS = 20;
    GOAL_MIN_THRESHOLD = 1;
    GOAL_SDEV_THRESHOLD = 0.75;
    GOAL_RANSAC_MATCHING_TOLERANCE = 0.2;
    RANSAC_MAX_ANGLE_DIFF_TO_MERGE = SAM_MAX_ANGLE_DIFF_TO_MERGE; //
    RANSAC_MAX_DISTANCE_TO_MERGE = SAM_MAX_DISTANCE_TO_MERGE; //

    std::ifstream in(filename.c_str());
    if(!in.is_open())
        errorlog << "VisionConstants::loadFromFile failed to load: " << filename << std::endl;
    std::string name;
    std::string sval;
    while(in.good()) {
        getline(in, name, ':');
        boost::trim(name);
        boost::to_upper(name);
        if(name.compare("WHITE_SIDE_IS_BLUE") == 0) {
            in >> WHITE_SIDE_IS_BLUE;
        }
        else if(name.compare("NON_WHITE_SIDE_CHECK") == 0) {
            in >> NON_WHITE_SIDE_CHECK;
        }
        else if(name.compare("DO_RADIAL_CORRECTION") == 0) {
            in >> DO_RADIAL_CORRECTION;
        }
        else if(name.compare("UPPER_WHITE_THRESHOLD") == 0) {
            in >> UPPER_WHITE_THRESHOLD;
        }
        else if(name.compare("LOWER_WHITE_THRESHOLD") == 0) {
            in >> LOWER_WHITE_THRESHOLD;
        }
        else if(name.compare("RADIAL_CORRECTION_COEFFICIENT") == 0) {
            in >> RADIAL_CORRECTION_COEFFICIENT;
        }
        else if(name.compare("THROWOUT_ON_ABOVE_KIN_HOR_GOALS") == 0) {
            in >> THROWOUT_ON_ABOVE_KIN_HOR_GOALS;
        }
        else if(name.compare("THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_GOALS") == 0) {
            in >> THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_GOALS;
        }
        else if(name.compare("THROWOUT_DISTANT_GOALS") == 0) {
            in >> THROWOUT_DISTANT_GOALS;
        }
        else if(name.compare("MAX_DISTANCE_METHOD_DISCREPENCY_GOALS") == 0) {
            in >> MAX_DISTANCE_METHOD_DISCREPENCY_GOALS;
        }
        else if(name.compare("MAX_GOAL_DISTANCE") == 0) {
            in >> MAX_GOAL_DISTANCE;
        }
        else if(name.compare("THROWOUT_ON_ABOVE_KIN_HOR_BALL") == 0) {
            in >> THROWOUT_ON_ABOVE_KIN_HOR_BALL;
        }
        else if(name.compare("THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BALL") == 0) {
            in >> THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BALL;
        }
        else if(name.compare("THROWOUT_SMALL_BALLS") == 0) {
            in >> THROWOUT_SMALL_BALLS;
        }
        else if(name.compare("MAX_DISTANCE_METHOD_DISCREPENCY_BALL") == 0) {
            in >> MAX_DISTANCE_METHOD_DISCREPENCY_BALL;
        }
        else if(name.compare("MIN_BALL_DIAMETER_PIXELS") == 0) {
            in >> MIN_BALL_DIAMETER_PIXELS;
        }
        else if(name.compare("THROWOUT_DISTANT_BALLS") == 0) {
            in >> THROWOUT_DISTANT_BALLS;
        }
        else if(name.compare("MAX_BALL_DISTANCE") == 0) {
            in >> MAX_BALL_DISTANCE;
        }
        else if(name.compare("D2P_INCLUDE_BODY_PITCH") == 0) {
            in >> D2P_INCLUDE_BODY_PITCH;
        }
        else if(name.compare("BALL_DISTANCE_POSITION_BOTTOM") == 0) {
            in >> BALL_DISTANCE_POSITION_BOTTOM;
        }
        else if(name.compare("BALL_DISTANCE_METHOD") == 0) {
            in >> sval;
            boost::trim(sval);
            boost::to_upper(sval);
            BALL_DISTANCE_METHOD = getDistanceMethodFromName(sval);
        }
        else if(name.compare("GOAL_DISTANCE_METHOD") == 0) {
            in >> sval;
            boost::trim(sval);
            boost::to_upper(sval);
            GOAL_DISTANCE_METHOD = getDistanceMethodFromName(sval);
        }
        else if(name.compare("LINE_METHOD") == 0) {
            in >> sval;
            boost::trim(sval);
            boost::to_upper(sval);
            LINE_METHOD = getLineMethodFromName(sval);
        }
        else if(name.compare("BALL_EDGE_THRESHOLD") == 0) {
            in >> BALL_EDGE_THRESHOLD;
        }
        else if(name.compare("BALL_ORANGE_TOLERANCE") == 0) {
            in >> BALL_ORANGE_TOLERANCE;
        }
        else if(name.compare("BALL_MIN_PERCENT_ORANGE") == 0) {
            in >> BALL_MIN_PERCENT_ORANGE;
        }
        else if(name.compare("GOAL_MIN_PERCENT_YELLOW") == 0) {
            in >> GOAL_MIN_PERCENT_YELLOW;
        }
        else if(name.compare("GOAL_MIN_PERCENT_BLUE") == 0) {
            in >> GOAL_MIN_PERCENT_BLUE;
        }
        else if(name.compare("MIN_DISTANCE_FROM_HORIZON") == 0) {
            in >> MIN_DISTANCE_FROM_HORIZON;
        }
        else if(name.compare("MIN_CONSECUTIVE_POINTS") == 0) {
            in >> MIN_CONSECUTIVE_POINTS;
        }
        else if(name.compare("GOAL_WIDTH") == 0) {
            in >> GOAL_WIDTH;
        }
        else if(name.compare("GOAL_HEIGHT") == 0) {
            in >> GOAL_HEIGHT;
        }
        else if(name.compare("DISTANCE_BETWEEN_POSTS") == 0) {
            in >> DISTANCE_BETWEEN_POSTS;
        }
        else if(name.compare("BALL_WIDTH") == 0) {
            in >> BALL_WIDTH;
        }
        else if(name.compare("CENTRE_CIRCLE_RADIUS") == 0) {
            in >> CENTRE_CIRCLE_RADIUS;
        }
        else if(name.compare("THROWOUT_INSIGNIFICANT_GOALS") == 0) {
            in >> THROWOUT_INSIGNIFICANT_GOALS;
        }
        else if(name.compare("MIN_TRANSITIONS_FOR_SIGNIFICANCE_GOALS") == 0) {
            in >> MIN_TRANSITIONS_FOR_SIGNIFICANCE_GOALS;
        }
        else if(name.compare("THROWOUT_INSIGNIFICANT_BALLS") == 0) {
            in >> THROWOUT_INSIGNIFICANT_BALLS;
        }
        else if(name.compare("MIN_TRANSITIONS_FOR_SIGNIFICANCE_BALL") == 0) {
            in >> MIN_TRANSITIONS_FOR_SIGNIFICANCE_BALL;
        }
        else if(name.compare("HORIZONTAL_SCANLINE_SPACING") == 0) {
            in >> HORIZONTAL_SCANLINE_SPACING;
        }
        else if(name.compare("VERTICAL_SCANLINE_SPACING") == 0) {
            in >> VERTICAL_SCANLINE_SPACING;
        }
        else if(name.compare("GREEN_HORIZON_SCAN_SPACING") == 0) {
            in >> GREEN_HORIZON_SCAN_SPACING;
        }
        else if(name.compare("GREEN_HORIZON_MIN_GREEN_PIXELS") == 0) {
            in >> GREEN_HORIZON_MIN_GREEN_PIXELS;
        }
        else if(name.compare("GREEN_HORIZON_UPPER_THRESHOLD_MULT") == 0) {
            in >> GREEN_HORIZON_UPPER_THRESHOLD_MULT;
        }
        else if(name.compare("THROWOUT_NARROW_GOALS") == 0) {
            in >> THROWOUT_NARROW_GOALS;
        }
        else if(name.compare("MIN_GOAL_WIDTH") == 0) {
            in >> MIN_GOAL_WIDTH;
        }
        else if(name.compare("THROWOUT_SHORT_GOALS") == 0) {
            in >> THROWOUT_SHORT_GOALS;
        }
        else if(name.compare("MIN_GOAL_HEIGHT") == 0) {
            in >> MIN_GOAL_HEIGHT;
        }
        else if(name.compare("GOAL_HEIGHT_TO_WIDTH_RATIO_MIN") == 0) {
            in >> GOAL_HEIGHT_TO_WIDTH_RATIO_MIN;
        }
        else if(name.compare("MIN_GOAL_SEPARATION") == 0) {
            in >> MIN_GOAL_SEPARATION;
        }
        else if(name.compare("SAM_MAX_LINES") == 0) {
            in >> SAM_MAX_LINES;
        }
        else if(name.compare("SAM_SPLIT_DISTANCE") == 0) {
            in >> SAM_SPLIT_DISTANCE;
        }
        else if(name.compare("SAM_MIN_POINTS_OVER") == 0) {
            in >> SAM_MIN_POINTS_OVER;
        }
        else if(name.compare("SAM_MIN_POINTS_TO_LINE") == 0) {
            in >> SAM_MIN_POINTS_TO_LINE;
        }
        else if(name.compare("SAM_MAX_ANGLE_DIFF_TO_MERGE") == 0) {
            in >> SAM_MAX_ANGLE_DIFF_TO_MERGE;
        }
        else if(name.compare("SAM_MAX_DISTANCE_TO_MERGE") == 0) {
            in >> SAM_MAX_DISTANCE_TO_MERGE;
        }
        else if(name.compare("SAM_MIN_POINTS_TO_LINE_FINAL") == 0) {
            in >> SAM_MIN_POINTS_TO_LINE_FINAL;
        }
        else if(name.compare("SAM_MIN_LINE_R2_FIT") == 0) {
            in >> SAM_MIN_LINE_R2_FIT;
        }
        else if(name.compare("SAM_MAX_LINE_MSD") == 0) {
            in >> SAM_MAX_LINE_MSD;
        }
        else if(name.compare("SAM_CLEAR_SMALL") == 0) {
            in >> SAM_CLEAR_SMALL;
        }
        else if(name.compare("SAM_CLEAR_DIRTY") == 0) {
            in >> SAM_CLEAR_DIRTY;
        }
        else if(name.compare("RANSAC_MAX_ANGLE_DIFF_TO_MERGE") == 0) {
            in >> RANSAC_MAX_ANGLE_DIFF_TO_MERGE;
        }
        else if(name.compare("RANSAC_MAX_DISTANCE_TO_MERGE") == 0) {
            in >> RANSAC_MAX_DISTANCE_TO_MERGE;
        }
        else if(name.compare("GOAL_MAX_OBJECTS") == 0) {
            in >> GOAL_MAX_OBJECTS;
        }
        else if(name.compare("GOAL_BINS") == 0) {
            in >> GOAL_BINS;
        }
        else if(name.compare("GOAL_MIN_THRESHOLD") == 0) {
            in >> GOAL_MIN_THRESHOLD;
        }
        else if(name.compare("GOAL_SDEV_THRESHOLD") == 0) {
            in >> GOAL_SDEV_THRESHOLD;
        }
        else if(name.compare("GOAL_RANSAC_MATCHING_TOLERANCE") == 0) {
            in >> GOAL_RANSAC_MATCHING_TOLERANCE;
        }
        else {
            errorlog << "VisionConstants::loadFromFile - unknown constant: " << name << std::endl;
        }
        
        // ignore the rest of the line
        in.ignore(128, '\n');
        //force eofbit in the case of last rule
        in.peek();
    }
    in.close();
    
    //debug << "VisionConstants::loadFromFile-" << std::endl;
    //print(debug);
}

bool VisionConstants::setParameter(std::string name, bool val)
{
    if(name.compare("DO_RADIAL_CORRECTION") == 0) {
        DO_RADIAL_CORRECTION = val;
    }
    else if(name.compare("THROWOUT_ON_ABOVE_KIN_HOR_GOALS") == 0) {
        THROWOUT_ON_ABOVE_KIN_HOR_GOALS = val;
    }
    else if(name.compare("THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_GOALS") == 0) {
        THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_GOALS = val;
    }
    else if(name.compare("THROWOUT_DISTANT_GOALS") == 0) {
        THROWOUT_DISTANT_GOALS = val;
    }
//    else if(name.compare("THROWOUT_ON_ABOVE_KIN_HOR_BEACONS") == 0) {
//        THROWOUT_ON_ABOVE_KIN_HOR_BEACONS = val;
//    }
//    else if(name.compare("THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BEACONS") == 0) {
//        THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BEACONS = val;
//    }
//    else if(name.compare("THROWOUT_DISTANT_BEACONS") == 0) {
//        THROWOUT_DISTANT_BEACONS = val;
//    }
    else if(name.compare("THROWOUT_ON_ABOVE_KIN_HOR_BALL") == 0) {
        THROWOUT_ON_ABOVE_KIN_HOR_BALL = val;
    }
    else if(name.compare("THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BALL") == 0) {
        THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BALL = val;
    }
    else if(name.compare("THROWOUT_SMALL_BALLS") == 0) {
        THROWOUT_SMALL_BALLS = val;
    }
    else if(name.compare("THROWOUT_DISTANT_BALLS") == 0) {
        THROWOUT_DISTANT_BALLS = val;
    }
    else if(name.compare("D2P_INCLUDE_BODY_PITCH") == 0) {
        D2P_INCLUDE_BODY_PITCH = val;
    }
    else if(name.compare("THROWOUT_INSIGNIFICANT_GOALS") == 0) {
        THROWOUT_INSIGNIFICANT_GOALS = val;
    }
//    else if(name.compare("THROWOUT_INSIGNIFICANT_BEACONS") == 0) {
//        THROWOUT_INSIGNIFICANT_BEACONS = val;
//    }
    else if(name.compare("THROWOUT_INSIGNIFICANT_BALLS") == 0) {
        THROWOUT_INSIGNIFICANT_BALLS = val;
    }
    else if(name.compare("THROWOUT_NARROW_GOALS") == 0) {
        THROWOUT_NARROW_GOALS = val;
    }
    else if(name.compare("THROWOUT_SHORT_GOALS") == 0) {
        THROWOUT_SHORT_GOALS = val;
    }
    else if(name.compare("SAM_CLEAR_SMALL") == 0) {
        SAM_CLEAR_SMALL = val;
    }
    else if(name.compare("SAM_CLEAR_DIRTY") == 0) {
        SAM_CLEAR_DIRTY = val;
    }
    else if(name.compare("BALL_DISTANCE_POSITION_BOTTOM") == 0) {
        BALL_DISTANCE_POSITION_BOTTOM = val;
    }
    else {
        return false;
    }
    return true;
}

bool VisionConstants::setParameter(std::string name, int val)
{
    if(name.compare("BALL_EDGE_THRESHOLD") == 0) {
        BALL_EDGE_THRESHOLD = val;
    }
    else if(name.compare("BALL_ORANGE_TOLERANCE") == 0) {
        BALL_ORANGE_TOLERANCE = val;
    }
    else if(name.compare("MIN_DISTANCE_FROM_HORIZON") == 0) {
        MIN_DISTANCE_FROM_HORIZON = val;
    }
    else if(name.compare("MIN_CONSECUTIVE_POINTS") == 0) {
        MIN_CONSECUTIVE_POINTS = val;
    }
    else if(name.compare("MIN_TRANSITIONS_FOR_SIGNIFICANCE_GOALS") == 0) {
        MIN_TRANSITIONS_FOR_SIGNIFICANCE_GOALS = val;
    }
//    else if(name.compare("MIN_TRANSITIONS_FOR_SIGNIFICANCE_BEACONS") == 0) {
//        MIN_TRANSITIONS_FOR_SIGNIFICANCE_BEACONS = val;
//    }
    else if(name.compare("MIN_TRANSITIONS_FOR_SIGNIFICANCE_BALL") == 0) {
        MIN_TRANSITIONS_FOR_SIGNIFICANCE_BALL = val;
    }
    else if(name.compare("MIN_GOAL_WIDTH") == 0) {
        MIN_GOAL_WIDTH = val;
    }
    else if(name.compare("MIN_GOAL_HEIGHT") == 0) {
        MIN_GOAL_HEIGHT = val;
    }
    else if(name.compare("MIN_GOAL_SEPARATION") == 0) {
        MIN_GOAL_SEPARATION = val;
    }
    else if(name.compare("GOAL_MAX_OBJECTS") == 0) {
        GOAL_MAX_OBJECTS = val;
    }
    else if(name.compare("GOAL_BINS") == 0) {
        GOAL_BINS = val;
    }
    else if(name.compare("GOAL_MIN_THRESHOLD") == 0) {
        GOAL_MIN_THRESHOLD = val;
    }
    else {
        return false;
    }
    return true;
}


bool VisionConstants::setParameter(std::string name, unsigned int val)
{
    if(name.compare("HORIZONTAL_SCANLINE_SPACING") == 0) {
        HORIZONTAL_SCANLINE_SPACING = val;
    }
    else if(name.compare("VERTICAL_SCANLINE_SPACING") == 0) {
        VERTICAL_SCANLINE_SPACING = val;
    }
    else if(name.compare("GREEN_HORIZON_SCAN_SPACING") == 0) {
        GREEN_HORIZON_SCAN_SPACING = val;
    }
    else if(name.compare("GREEN_HORIZON_MIN_GREEN_PIXELS") == 0) {
        GREEN_HORIZON_MIN_GREEN_PIXELS = val;
    }
    else if(name.compare("SAM_MAX_LINES") == 0) {
        SAM_MAX_LINES = val;
    }
    else if(name.compare("SAM_MIN_POINTS_OVER") == 0) {
        SAM_MIN_POINTS_OVER = val;
    }
    else if(name.compare("SAM_MIN_POINTS_TO_LINE") == 0) {
        SAM_MIN_POINTS_TO_LINE = val;
    }
    else if(name.compare("SAM_MIN_POINTS_TO_LINE_FINAL") == 0) {
        SAM_MIN_POINTS_TO_LINE_FINAL = val;
    }
    else {
        return false;
    }
    return true;
}

bool VisionConstants::setParameter(std::string name, float val)
{
    if(name.compare("RADIAL_CORRECTION_COEFFICIENT") == 0) {
        RADIAL_CORRECTION_COEFFICIENT = val;
    }
    else if(name.compare("MAX_DISTANCE_METHOD_DISCREPENCY_GOALS") == 0) {
        MAX_DISTANCE_METHOD_DISCREPENCY_GOALS = val;
    }
    else if(name.compare("MAX_GOAL_DISTANCE") == 0) {
        MAX_GOAL_DISTANCE = val;
    }
//    else if(name.compare("MAX_DISTANCE_METHOD_DISCREPENCY_BEACONS") == 0) {
//        MAX_DISTANCE_METHOD_DISCREPENCY_BEACONS = val;
//    }
//    else if(name.compare("MAX_BEACON_DISTANCE") == 0) {
//        MAX_BEACON_DISTANCE = val;
//    }
    else if(name.compare("MAX_DISTANCE_METHOD_DISCREPENCY_BALL") == 0) {
        MAX_DISTANCE_METHOD_DISCREPENCY_BALL = val;
    }
    else if(name.compare("MIN_BALL_DIAMETER_PIXELS") == 0) {
        MIN_BALL_DIAMETER_PIXELS = val;
    }
    else if(name.compare("MAX_BALL_DISTANCE") == 0) {
        MAX_BALL_DISTANCE = val;
    }
    else if(name.compare("BALL_MIN_PERCENT_ORANGE") == 0) {
        BALL_MIN_PERCENT_ORANGE = val;
    }
    else if(name.compare("GOAL_MIN_PERCENT_YELLOW") == 0) {
        GOAL_MIN_PERCENT_YELLOW = val;
    }
    else if(name.compare("GOAL_MIN_PERCENT_BLUE") == 0) {
        GOAL_MIN_PERCENT_BLUE = val;
    }
//    else if(name.compare("BEACON_MIN_PERCENT_YELLOW") == 0) {
//        BEACON_MIN_PERCENT_YELLOW = val;
//    }
//    else if(name.compare("BEACON_MIN_PERCENT_BLUE") == 0) {
//        BEACON_MIN_PERCENT_BLUE = val;
//    }
    else if(name.compare("GOAL_WIDTH") == 0) {
        GOAL_WIDTH = val;
    }
    else if(name.compare("GOAL_HEIGHT") == 0) {
        GOAL_HEIGHT = val;
    }
    else if(name.compare("BALL_WIDTH") == 0) {
        BALL_WIDTH = val;
    }
    else if(name.compare("CENTRE_CIRCLE_RADIUS") == 0) {
        CENTRE_CIRCLE_RADIUS = val;
    }
//    else if(name.compare("BEACON_WIDTH") == 0) {
//        BEACON_WIDTH = val;
//    }
    else if(name.compare("DISTANCE_BETWEEN_POSTS") == 0) {
        DISTANCE_BETWEEN_POSTS = val;
    }
    else if(name.compare("GREEN_HORIZON_UPPER_THRESHOLD_MULT") == 0) {
        GREEN_HORIZON_UPPER_THRESHOLD_MULT = val;
    }
    else if(name.compare("GOAL_HEIGHT_TO_WIDTH_RATIO_MIN") == 0) {
        GOAL_HEIGHT_TO_WIDTH_RATIO_MIN = val;
    }
    else if(name.compare("SAM_SPLIT_DISTANCE") == 0) {
        SAM_SPLIT_DISTANCE = val;
    }
    else if(name.compare("SAM_MAX_ANGLE_DIFF_TO_MERGE") == 0) {
        SAM_MAX_ANGLE_DIFF_TO_MERGE = val;
    }
    else if(name.compare("SAM_MAX_DISTANCE_TO_MERGE") == 0) {
        SAM_MAX_DISTANCE_TO_MERGE = val;
    }
    else if(name.compare("SAM_MIN_LINE_R2_FIT") == 0) {
        SAM_MIN_LINE_R2_FIT = val;
    }
    else if(name.compare("SAM_MAX_LINE_MSD") == 0) {
        SAM_MAX_LINE_MSD = val;
    }
    else if(name.compare("RANSAC_MAX_ANGLE_DIFF_TO_MERGE") == 0) {
        RANSAC_MAX_ANGLE_DIFF_TO_MERGE = val;
    }
    else if(name.compare("RANSAC_MAX_DISTANCE_TO_MERGE") == 0) {
        RANSAC_MAX_DISTANCE_TO_MERGE = val;
    }
    else if(name.compare("GOAL_SDEV_THRESHOLD") == 0) {
        GOAL_SDEV_THRESHOLD = val;
    }
    else if(name.compare("GOAL_RANSAC_MATCHING_TOLERANCE") == 0) {
        GOAL_RANSAC_MATCHING_TOLERANCE = val;
    }
    else {
        return false;
    }
    return true;
}

bool VisionConstants::setParameter(std::string name, DistanceMethod val)
{
    if(name.compare("BALL_DISTANCE_METHOD") == 0) {
        BALL_DISTANCE_METHOD = val;
    }
    else if(name.compare("GOAL_DISTANCE_METHOD") == 0) {
        GOAL_DISTANCE_METHOD = val;
    }
//    else if(name.compare("BEACON_DISTANCE_METHOD") == 0) {
//        BEACON_DISTANCE_METHOD = val;
//    }
    else {
        return false;
    }
    return true;
}

void VisionConstants::print(std::ostream& out)
{
    out << "WHITE_SIDE_IS_BLUE: " << WHITE_SIDE_IS_BLUE << std::endl;
    out << "NON_WHITE_SIDE_CHECK: " << NON_WHITE_SIDE_CHECK << std::endl;
    out << "UPPER_WHITE_THRESHOLD: " << UPPER_WHITE_THRESHOLD << std::endl;
    out << "LOWER_WHITE_THRESHOLD: " << LOWER_WHITE_THRESHOLD << std::endl;

    out << "DO_RADIAL_CORRECTION: " << DO_RADIAL_CORRECTION << std::endl;
    out << "RADIAL_CORRECTION_COEFFICIENT: " << RADIAL_CORRECTION_COEFFICIENT << std::endl;

    out << "THROWOUT_ON_ABOVE_KIN_HOR_GOALS: " << THROWOUT_ON_ABOVE_KIN_HOR_GOALS << std::endl;
    out << "THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_GOALS: " << THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_GOALS << std::endl;
    out << "MAX_DISTANCE_METHOD_DISCREPENCY_GOALS: " << MAX_DISTANCE_METHOD_DISCREPENCY_GOALS << std::endl;
    out << "THROWOUT_DISTANT_GOALS: " << THROWOUT_DISTANT_GOALS << std::endl;
    out << "MAX_GOAL_DISTANCE: " << MAX_GOAL_DISTANCE << std::endl;
    out << "THROWOUT_INSIGNIFICANT_GOALS: " << THROWOUT_INSIGNIFICANT_GOALS << std::endl;
    out << "MIN_TRANSITIONS_FOR_SIGNIFICANCE_GOALS: " << MIN_TRANSITIONS_FOR_SIGNIFICANCE_GOALS << std::endl;
    out << "THROWOUT_NARROW_GOALS: " << THROWOUT_NARROW_GOALS << std::endl;
    out << "MIN_GOAL_WIDTH: " << MIN_GOAL_WIDTH << std::endl;
    out << "THROWOUT_SHORT_GOALS: " << THROWOUT_SHORT_GOALS << std::endl;
    out << "MIN_GOAL_HEIGHT: " << MIN_GOAL_HEIGHT << std::endl;
    out << "GOAL_HEIGHT_TO_WIDTH_RATIO_MIN: " << GOAL_HEIGHT_TO_WIDTH_RATIO_MIN << std::endl;

    out << "GOAL_MAX_OBJECTS: " << GOAL_MAX_OBJECTS << std::endl;
    out << "GOAL_BINS: " << GOAL_BINS << std::endl;
    out << "GOAL_MIN_THRESHOLD: " << GOAL_MIN_THRESHOLD << std::endl;
    out << "GOAL_SDEV_THRESHOLD: " << GOAL_SDEV_THRESHOLD << std::endl;
    out << "GOAL_RANSAC_MATCHING_TOLERANCE: " << GOAL_RANSAC_MATCHING_TOLERANCE << std::endl;

    out << "THROWOUT_ON_ABOVE_KIN_HOR_BALL: " << THROWOUT_ON_ABOVE_KIN_HOR_BALL << std::endl;
    out << "THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BALL: " << THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BALL << std::endl;
    out << "MAX_DISTANCE_METHOD_DISCREPENCY_BALL: " << MAX_DISTANCE_METHOD_DISCREPENCY_BALL << std::endl;
    out << "THROWOUT_SMALL_BALLS: " << THROWOUT_SMALL_BALLS << std::endl;
    out << "MIN_BALL_DIAMETER_PIXELS: " << MIN_BALL_DIAMETER_PIXELS << std::endl;
    out << "THROWOUT_INSIGNIFICANT_BALLS: " << THROWOUT_INSIGNIFICANT_BALLS << std::endl;
    out << "MIN_TRANSITIONS_FOR_SIGNIFICANCE_BALL: " << MIN_TRANSITIONS_FOR_SIGNIFICANCE_BALL << std::endl;

    out << "D2P_INCLUDE_BODY_PITCH: " << D2P_INCLUDE_BODY_PITCH << std::endl;
    out << "BALL_DISTANCE_POSITION_BOTTOM: " << BALL_DISTANCE_POSITION_BOTTOM << std::endl;

    out << "BALL_DISTANCE_METHOD: " << getDistanceMethodName(BALL_DISTANCE_METHOD) << std::endl;
    out << "GOAL_DISTANCE_METHOD: " << getDistanceMethodName(GOAL_DISTANCE_METHOD) << std::endl;

    out << "BALL_EDGE_THRESHOLD: " << BALL_EDGE_THRESHOLD << std::endl;
    out << "BALL_ORANGE_TOLERANCE: " << BALL_ORANGE_TOLERANCE << std::endl;
    out << "BALL_MIN_PERCENT_ORANGE: " << BALL_MIN_PERCENT_ORANGE << std::endl;
    out << "GOAL_MIN_PERCENT_YELLOW: " << GOAL_MIN_PERCENT_YELLOW << std::endl;
    out << "GOAL_MIN_PERCENT_BLUE: " << GOAL_MIN_PERCENT_BLUE << std::endl;
    out << "MIN_GOAL_SEPARATION: " << MIN_GOAL_SEPARATION << std::endl;

    out << "MIN_DISTANCE_FROM_HORIZON: " << MIN_DISTANCE_FROM_HORIZON << std::endl;
    out << "MIN_CONSECUTIVE_POINTS: " << MIN_CONSECUTIVE_POINTS << std::endl;

    out << "GOAL_WIDTH: " << GOAL_WIDTH << std::endl;
    out << "GOAL_HEIGHT: " << GOAL_HEIGHT << std::endl;
    out << "DISTANCE_BETWEEN_POSTS: " << DISTANCE_BETWEEN_POSTS << std::endl;
    out << "BALL_WIDTH: " << BALL_WIDTH << std::endl;
    out << "CENTRE_CIRCLE_RADIUS: " << CENTRE_CIRCLE_RADIUS << std::endl;

    out << "HORIZONTAL_SCANLINE_SPACING: " << HORIZONTAL_SCANLINE_SPACING << std::endl;
    out << "VERTICAL_SCANLINE_SPACING: " << VERTICAL_SCANLINE_SPACING << std::endl;
    out << "GREEN_HORIZON_SCAN_SPACING: " << GREEN_HORIZON_SCAN_SPACING << std::endl;
    out << "GREEN_HORIZON_MIN_GREEN_PIXELS: " << GREEN_HORIZON_MIN_GREEN_PIXELS << std::endl;
    out << "GREEN_HORIZON_UPPER_THRESHOLD_MULT: " << GREEN_HORIZON_UPPER_THRESHOLD_MULT << std::endl;

    out << "SAM_MAX_LINES: " << SAM_MAX_LINES << std::endl;
    out << "SAM_SPLIT_DISTANCE: " << SAM_SPLIT_DISTANCE << std::endl;
    out << "SAM_MIN_POINTS_OVER: " << SAM_MIN_POINTS_OVER << std::endl;
    out << "SAM_MIN_POINTS_TO_LINE: " << SAM_MIN_POINTS_TO_LINE << std::endl;
    out << "SAM_MAX_ANGLE_DIFF_TO_MERGE: " << SAM_MAX_ANGLE_DIFF_TO_MERGE << std::endl;
    out << "SAM_MAX_DISTANCE_TO_MERGE: " << SAM_MAX_DISTANCE_TO_MERGE << std::endl;
    out << "SAM_MIN_POINTS_TO_LINE_FINAL: " << SAM_MIN_POINTS_TO_LINE_FINAL << std::endl;
    out << "SAM_MIN_LINE_R2_FIT: " << SAM_MIN_LINE_R2_FIT << std::endl;
    out << "SAM_MAX_LINE_MSD: " << SAM_MAX_LINE_MSD << std::endl;
    out << "SAM_CLEAR_SMALL: " << SAM_CLEAR_SMALL << std::endl;
    out << "SAM_CLEAR_DIRTY: " << SAM_CLEAR_DIRTY << std::endl;

    out << "RANSAC_MAX_ANGLE_DIFF_TO_MERGE: " << RANSAC_MAX_ANGLE_DIFF_TO_MERGE << std::endl;
    out << "RANSAC_MAX_DISTANCE_TO_MERGE: " << RANSAC_MAX_DISTANCE_TO_MERGE << std::endl;

}

void VisionConstants::setFlags(bool val)
{
    DO_RADIAL_CORRECTION = val;
    THROWOUT_ON_ABOVE_KIN_HOR_GOALS = val;
    THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_GOALS = val;
    THROWOUT_DISTANT_GOALS = val;
    THROWOUT_INSIGNIFICANT_GOALS = val;
    THROWOUT_NARROW_GOALS = val;
    THROWOUT_SHORT_GOALS = val;

    THROWOUT_ON_ABOVE_KIN_HOR_BALL = val;
    THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BALL = val;
    THROWOUT_SMALL_BALLS = val;
    THROWOUT_INSIGNIFICANT_BALLS = val;
    THROWOUT_DISTANT_BALLS = val;

    D2P_INCLUDE_BODY_PITCH = val;

    SAM_CLEAR_SMALL = val;
    SAM_CLEAR_DIRTY = val;

}

std::vector<Parameter> VisionConstants::getAllOptimisable()
{
    std::vector<Parameter> params;
    //! Goal filtering constants
    params.push_back(Parameter("MIN_TRANSITIONS_FOR_SIGNIFICANCE_GOALS", MIN_TRANSITIONS_FOR_SIGNIFICANCE_GOALS, 1, 500));
    params.push_back(Parameter("MIN_GOAL_WIDTH", MIN_GOAL_WIDTH, 0, 320));
    params.push_back(Parameter("MIN_GOAL_HEIGHT", MIN_GOAL_HEIGHT, 0, 240));

    params.push_back(Parameter("GOAL_HEIGHT_TO_WIDTH_RATIO_MIN", GOAL_HEIGHT_TO_WIDTH_RATIO_MIN, 0, 50));

    params.push_back(Parameter("GOAL_MAX_OBJECTS", GOAL_MAX_OBJECTS, 1, 50));
    params.push_back(Parameter("GOAL_BINS", GOAL_BINS, 5, 320));
    params.push_back(Parameter("GOAL_MIN_THRESHOLD", GOAL_MIN_THRESHOLD, 1, 50));
    params.push_back(Parameter("GOAL_SDEV_THRESHOLD", GOAL_SDEV_THRESHOLD, 0, 3));

    //! Ball filtering constants
    params.push_back(Parameter("MIN_BALL_DIAMETER_PIXELS", MIN_BALL_DIAMETER_PIXELS, 1, 100));
    params.push_back(Parameter("MIN_TRANSITIONS_FOR_SIGNIFICANCE_BALL", MIN_TRANSITIONS_FOR_SIGNIFICANCE_BALL, 1, 500));
    //! Field-object detection constants
    params.push_back(Parameter("BALL_EDGE_THRESHOLD", BALL_EDGE_THRESHOLD, 0, 50));
    params.push_back(Parameter("BALL_ORANGE_TOLERANCE", BALL_ORANGE_TOLERANCE, 0, 50));
    params.push_back(Parameter("BALL_MIN_PERCENT_ORANGE", BALL_MIN_PERCENT_ORANGE, 0, 1));
    params.push_back(Parameter("GOAL_MIN_PERCENT_YELLOW", GOAL_MIN_PERCENT_YELLOW, 0, 1));
    params.push_back(Parameter("GOAL_MIN_PERCENT_BLUE", GOAL_MIN_PERCENT_BLUE, 0, 1));
    params.push_back(Parameter("MIN_GOAL_SEPARATION", MIN_GOAL_SEPARATION, 0, 320));
    //! Obstacle detection constants
    params.push_back(Parameter("MIN_DISTANCE_FROM_HORIZON", MIN_DISTANCE_FROM_HORIZON, 0, 240));
    params.push_back(Parameter("MIN_CONSECUTIVE_POINTS", MIN_CONSECUTIVE_POINTS, 0, 50));
    params.push_back(Parameter("GREEN_HORIZON_MIN_GREEN_PIXELS", GREEN_HORIZON_MIN_GREEN_PIXELS, 1, 50));
    params.push_back(Parameter("GREEN_HORIZON_UPPER_THRESHOLD_MULT", GREEN_HORIZON_UPPER_THRESHOLD_MULT, 0, 20));
    //! Split and Merge constants
    params.push_back(Parameter("SAM_SPLIT_DISTANCE", SAM_SPLIT_DISTANCE, 0, 320));
    params.push_back(Parameter("SAM_MIN_POINTS_OVER", SAM_MIN_POINTS_OVER, 1, 500));
    params.push_back(Parameter("SAM_MIN_POINTS_TO_LINE", SAM_MIN_POINTS_TO_LINE, 2, 50));
    params.push_back(Parameter("SAM_MAX_ANGLE_DIFF_TO_MERGE", SAM_MAX_ANGLE_DIFF_TO_MERGE, 0, mathGeneral::PI*0.25));
    params.push_back(Parameter("SAM_MAX_DISTANCE_TO_MERGE", SAM_MAX_DISTANCE_TO_MERGE, 0, 150));
    params.push_back(Parameter("SAM_MIN_POINTS_TO_LINE_FINAL", SAM_MIN_POINTS_TO_LINE_FINAL, 2, 50));
    params.push_back(Parameter("SAM_MIN_LINE_R2_FIT", SAM_MIN_LINE_R2_FIT, 0, 1));
    params.push_back(Parameter("SAM_MAX_LINE_MSD", SAM_MAX_LINE_MSD, 0, 150));
    //! ScanLine options
    params.push_back(Parameter("HORIZONTAL_SCANLINE_SPACING", HORIZONTAL_SCANLINE_SPACING, 1, 50));
    params.push_back(Parameter("VERTICAL_SCANLINE_SPACING", VERTICAL_SCANLINE_SPACING, 1, 50));
    params.push_back(Parameter("GREEN_HORIZON_SCAN_SPACING", GREEN_HORIZON_SCAN_SPACING, 1, 50));

    return params;
}

std::vector<Parameter> VisionConstants::getBallParams()
{
    std::vector<Parameter> params;
    params.push_back(Parameter("MIN_BALL_DIAMETER_PIXELS", MIN_BALL_DIAMETER_PIXELS, 1, 100));
    params.push_back(Parameter("MIN_TRANSITIONS_FOR_SIGNIFICANCE_BALL", MIN_TRANSITIONS_FOR_SIGNIFICANCE_BALL, 1, 500));
    params.push_back(Parameter("BALL_EDGE_THRESHOLD", BALL_EDGE_THRESHOLD, 0, 50));
    params.push_back(Parameter("BALL_ORANGE_TOLERANCE", BALL_ORANGE_TOLERANCE, 0, 50));
    params.push_back(Parameter("BALL_MIN_PERCENT_ORANGE", BALL_MIN_PERCENT_ORANGE, 0, 1));

    return params;
}

std::vector<Parameter> VisionConstants::getGoalParams()
{
    std::vector<Parameter> params;
    //! Goal filtering constants
    params.push_back(Parameter("MIN_TRANSITIONS_FOR_SIGNIFICANCE_GOALS", MIN_TRANSITIONS_FOR_SIGNIFICANCE_GOALS, 1, 500));
    params.push_back(Parameter("MIN_GOAL_WIDTH", MIN_GOAL_WIDTH, 0, 320));
    params.push_back(Parameter("MIN_GOAL_HEIGHT", MIN_GOAL_HEIGHT, 0, 240));

    params.push_back(Parameter("GOAL_HEIGHT_TO_WIDTH_RATIO_MIN", GOAL_HEIGHT_TO_WIDTH_RATIO_MIN, 0, 50));

    params.push_back(Parameter("GOAL_MAX_OBJECTS", GOAL_MAX_OBJECTS, 1, 50));
    params.push_back(Parameter("GOAL_BINS", GOAL_BINS, 5, 320));
    params.push_back(Parameter("GOAL_MIN_THRESHOLD", GOAL_MIN_THRESHOLD, 1, 50));
    params.push_back(Parameter("GOAL_SDEV_THRESHOLD", GOAL_SDEV_THRESHOLD, 0, 3));
    //! Beacon filtering constants
    //! Ball filtering constants
    //! Field-object detection constants
    params.push_back(Parameter("GOAL_MIN_PERCENT_YELLOW", GOAL_MIN_PERCENT_YELLOW, 0, 1));
    params.push_back(Parameter("GOAL_MIN_PERCENT_BLUE", GOAL_MIN_PERCENT_BLUE, 0, 1));
    params.push_back(Parameter("MIN_GOAL_SEPARATION", MIN_GOAL_SEPARATION, 0, 320));

    return params;
}

std::vector<Parameter> VisionConstants::getObstacleParams()
{
    std::vector<Parameter> params;
    //! Obstacle detection constants
    params.push_back(Parameter("MIN_DISTANCE_FROM_HORIZON", MIN_DISTANCE_FROM_HORIZON, 0, 240));
    params.push_back(Parameter("MIN_CONSECUTIVE_POINTS", MIN_CONSECUTIVE_POINTS, 0, 50));
    return params;
}

std::vector<Parameter> VisionConstants::getLineParams()
{
    std::vector<Parameter> params;
    //! Split and Merge constants
    params.push_back(Parameter("SAM_SPLIT_DISTANCE", SAM_SPLIT_DISTANCE, 0, 320));
    params.push_back(Parameter("SAM_MIN_POINTS_OVER", SAM_MIN_POINTS_OVER, 1, 500));
    params.push_back(Parameter("SAM_MIN_POINTS_TO_LINE", SAM_MIN_POINTS_TO_LINE, 2, 50));
    params.push_back(Parameter("SAM_MAX_ANGLE_DIFF_TO_MERGE", SAM_MAX_ANGLE_DIFF_TO_MERGE, 0, mathGeneral::PI*0.25));
    params.push_back(Parameter("SAM_MAX_DISTANCE_TO_MERGE", SAM_MAX_DISTANCE_TO_MERGE, 0, 150));
    params.push_back(Parameter("SAM_MIN_POINTS_TO_LINE_FINAL", SAM_MIN_POINTS_TO_LINE_FINAL, 2, 50));
    params.push_back(Parameter("SAM_MIN_LINE_R2_FIT", SAM_MIN_LINE_R2_FIT, 0, 1));
    params.push_back(Parameter("SAM_MAX_LINE_MSD", SAM_MAX_LINE_MSD, 0, 150));
    return params;
}

std::vector<Parameter> VisionConstants::getGeneralParams()
{
    std::vector<Parameter> params;
    params.push_back(Parameter("GREEN_HORIZON_MIN_GREEN_PIXELS", GREEN_HORIZON_MIN_GREEN_PIXELS, 1, 50));
    params.push_back(Parameter("GREEN_HORIZON_UPPER_THRESHOLD_MULT", GREEN_HORIZON_UPPER_THRESHOLD_MULT, 0, 20));

    //! ScanLine options
    params.push_back(Parameter("HORIZONTAL_SCANLINE_SPACING", HORIZONTAL_SCANLINE_SPACING, 1, 50));
    params.push_back(Parameter("VERTICAL_SCANLINE_SPACING", VERTICAL_SCANLINE_SPACING, 1, 50));
    params.push_back(Parameter("GREEN_HORIZON_SCAN_SPACING", GREEN_HORIZON_SCAN_SPACING, 1, 50));
    return params;
}

bool VisionConstants::setAllOptimisable(const std::vector<float>& params)
{
    if(params.size() != 29) {
        return false; //not a valid size
    }
    MIN_TRANSITIONS_FOR_SIGNIFICANCE_GOALS = params.at(0);
    MIN_GOAL_WIDTH = params.at(1);
    MIN_GOAL_HEIGHT = params.at(2);
    GOAL_HEIGHT_TO_WIDTH_RATIO_MIN = params.at(3);

    GOAL_MAX_OBJECTS = params.at(4);
    GOAL_BINS = params.at(5);
    GOAL_MIN_THRESHOLD = params.at(6);
    GOAL_SDEV_THRESHOLD = params.at(7);

    MIN_TRANSITIONS_FOR_SIGNIFICANCE_BALL = params.at(8);
    BALL_EDGE_THRESHOLD = params.at(9);
    BALL_ORANGE_TOLERANCE = params.at(10);
    BALL_MIN_PERCENT_ORANGE = params.at(11);
    GOAL_MIN_PERCENT_YELLOW = params.at(12);
    GOAL_MIN_PERCENT_BLUE = params.at(13);
    MIN_DISTANCE_FROM_HORIZON = params.at(14);
    MIN_CONSECUTIVE_POINTS = params.at(15);
    GREEN_HORIZON_MIN_GREEN_PIXELS = params.at(16);
    GREEN_HORIZON_UPPER_THRESHOLD_MULT = params.at(17);
    SAM_SPLIT_DISTANCE = params.at(18);
    SAM_MIN_POINTS_OVER = params.at(19);
    SAM_MIN_POINTS_TO_LINE = params.at(20);
    SAM_MAX_ANGLE_DIFF_TO_MERGE = params.at(21);
    SAM_MAX_DISTANCE_TO_MERGE = params.at(22);
    SAM_MIN_POINTS_TO_LINE_FINAL = params.at(23);
    SAM_MIN_LINE_R2_FIT = params.at(24);
    SAM_MAX_LINE_MSD = params.at(25);
    HORIZONTAL_SCANLINE_SPACING = params.at(26);
    VERTICAL_SCANLINE_SPACING = params.at(27);
    GREEN_HORIZON_SCAN_SPACING = params.at(28);
    return true;
}

bool VisionConstants::setBallParams(const std::vector<float>& params)
{
    if(params.size() != 5) {
        return false; //not a valid size
    }
    MIN_BALL_DIAMETER_PIXELS = params.at(0);
    MIN_TRANSITIONS_FOR_SIGNIFICANCE_BALL = params.at(1);
    BALL_EDGE_THRESHOLD = params.at(2);
    BALL_ORANGE_TOLERANCE = params.at(3);
    BALL_MIN_PERCENT_ORANGE = params.at(4);
    return true;
}

bool VisionConstants::setGoalParams(const std::vector<float>& params)
{
    if(params.size() != 11) {
        return false; //not a valid size
    }
    MIN_TRANSITIONS_FOR_SIGNIFICANCE_GOALS = params.at(0);
    MIN_GOAL_WIDTH = params.at(1);
    MIN_GOAL_HEIGHT = params.at(2);
    GOAL_HEIGHT_TO_WIDTH_RATIO_MIN = params.at(3);

    GOAL_MAX_OBJECTS = params.at(4);
    GOAL_BINS = params.at(5);
    GOAL_MIN_THRESHOLD = params.at(6);
    GOAL_SDEV_THRESHOLD = params.at(7);

    GOAL_MIN_PERCENT_YELLOW = params.at(8);
    GOAL_MIN_PERCENT_BLUE = params.at(9);
    MIN_GOAL_SEPARATION = params.at(10);
    return true;
}

bool VisionConstants::setObstacleParams(const std::vector<float>& params)
{
    if(params.size() != 2) {
        return false; //not a valid size
    }
    MIN_DISTANCE_FROM_HORIZON = params.at(0);
    MIN_CONSECUTIVE_POINTS = params.at(1);
    return true;
}

bool VisionConstants::setLineParams(const std::vector<float>& params)
{
    if(params.size() != 8) {
        return false; //not a valid size
    }
    SAM_SPLIT_DISTANCE = params.at(0);
    SAM_MIN_POINTS_OVER = params.at(1);
    SAM_MIN_POINTS_TO_LINE = params.at(2);
    SAM_MAX_ANGLE_DIFF_TO_MERGE = params.at(3);
    SAM_MAX_DISTANCE_TO_MERGE = params.at(4);
    SAM_MIN_POINTS_TO_LINE_FINAL = params.at(5);
    SAM_MIN_LINE_R2_FIT = params.at(6);
    SAM_MAX_LINE_MSD = params.at(7);
    return true;
}

bool VisionConstants::setGeneralParams(const std::vector<float>& params)
{
    if(params.size() != 5) {
        return false; //not a valid size
    }
    GREEN_HORIZON_MIN_GREEN_PIXELS = params.at(0);
    GREEN_HORIZON_UPPER_THRESHOLD_MULT = params.at(1);
    //! ScanLine options
    HORIZONTAL_SCANLINE_SPACING = params.at(2);
    VERTICAL_SCANLINE_SPACING = params.at(3);
    GREEN_HORIZON_SCAN_SPACING = params.at(4);
    return true;
}
