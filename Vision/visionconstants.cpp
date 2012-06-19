#include "visionconstants.h"

#include "nubotdataconfig.h"
#include "debug.h"
#include "debugverbosityvision.h"

#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/GameInformation/GameInformation.h"

#include <fstream>
#include <boost/algorithm/string.hpp>

// Distortion Correction
bool VisionConstants::DO_RADIAL_CORRECTION;
float VisionConstants::RADIAL_CORRECTION_COEFFICIENT;
// Goal filtering constants
bool VisionConstants::THROWOUT_ON_ABOVE_KIN_HOR_GOALS;
bool VisionConstants::THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_GOALS;
float VisionConstants::MAX_DISTANCE_METHOD_DISCREPENCY_GOALS;
bool VisionConstants::THROWOUT_DISTANT_GOALS;
float VisionConstants::MAX_GOAL_DISTANCE;
bool VisionConstants::THROWOUT_INSIGNIFICANT_GOALS;
int VisionConstants::MIN_TRANSITIONS_FOR_SIGNIFICANCE_GOALS;
bool VisionConstants::THROWOUT_NARROW_GOALS;
int VisionConstants::MIN_GOAL_WIDTH;
float VisionConstants::GOAL_EDGE_RATIO;
// Beacon filtering constants
bool VisionConstants::THROWOUT_ON_ABOVE_KIN_HOR_BEACONS;
bool VisionConstants::THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BEACONS;
float VisionConstants::MAX_DISTANCE_METHOD_DISCREPENCY_BEACONS;
bool VisionConstants::THROWOUT_DISTANT_BEACONS;
float VisionConstants::MAX_BEACON_DISTANCE;
bool VisionConstants::THROWOUT_INSIGNIFICANT_BEACONS;
int VisionConstants::MIN_TRANSITIONS_FOR_SIGNIFICANCE_BEACONS;
// Ball filtering constants
bool VisionConstants::THROWOUT_ON_ABOVE_KIN_HOR_BALL;
bool VisionConstants::THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BALL;
float VisionConstants::MAX_DISTANCE_METHOD_DISCREPENCY_BALL;
bool VisionConstants::THROWOUT_SMALL_BALLS;
float VisionConstants::MIN_BALL_DIAMETER_PIXELS;
bool VisionConstants::THROWOUT_INSIGNIFICANT_BALLS;
int VisionConstants::MIN_TRANSITIONS_FOR_SIGNIFICANCE_BALL;
bool VisionConstants::THROWOUT_DISTANT_BALLS;
float VisionConstants::MAX_BALL_DISTANCE;
// Distance calculation options
bool VisionConstants::D2P_INCLUDE_BODY_PITCH;
float VisionConstants::D2P_ANGLE_CORRECTION;
bool VisionConstants::BALL_DISTANCE_POSITION_BOTTOM;
//Distance method options
VisionConstants::DistanceMethod VisionConstants::BALL_DISTANCE_METHOD;
VisionConstants::DistanceMethod VisionConstants::GOAL_DISTANCE_METHOD;
VisionConstants::DistanceMethod VisionConstants::BEACON_DISTANCE_METHOD;
//Field-object detection constants
int VisionConstants::BALL_EDGE_THRESHOLD;
int VisionConstants::BALL_ORANGE_TOLERANCE;
float VisionConstants::BALL_MIN_PERCENT_ORANGE;
float VisionConstants::GOAL_MIN_PERCENT_YELLOW;
float VisionConstants::GOAL_MIN_PERCENT_BLUE;
float VisionConstants::BEACON_MIN_PERCENT_YELLOW;
float VisionConstants::BEACON_MIN_PERCENT_BLUE;
int VisionConstants::MIN_GOAL_SEPARATION;
// Obstacle detection constants
int VisionConstants::MIN_DISTANCE_FROM_HORIZON;
int VisionConstants::MIN_CONSECUTIVE_POINTS;
// Field dimension constants
float VisionConstants::GOAL_WIDTH;
float VisionConstants::DISTANCE_BETWEEN_POSTS;
float VisionConstants::BALL_WIDTH;
float VisionConstants::BEACON_WIDTH;
// ScanLine options
unsigned int VisionConstants::HORIZONTAL_SCANLINE_SPACING;
unsigned int VisionConstants::VERTICAL_SCANLINE_SPACING;
unsigned int VisionConstants::GREEN_HORIZON_SCAN_SPACING;
unsigned int VisionConstants::GREEN_HORIZON_MIN_GREEN_PIXELS;
float VisionConstants::GREEN_HORIZON_LOWER_THRESHOLD_MULT;
float VisionConstants::GREEN_HORIZON_UPPER_THRESHOLD_MULT;

VisionConstants::VisionConstants()
{
}

/*! @brief Loads vision constants and options from the given file.
  * @param filename The name of the file (located in the Config directory).
  */
void VisionConstants::loadFromFile(std::string filename) 
{
    HORIZONTAL_SCANLINE_SPACING = 5; //defaults in case of bad file
    VERTICAL_SCANLINE_SPACING = 5;
    GREEN_HORIZON_SCAN_SPACING = 11;
    GREEN_HORIZON_MIN_GREEN_PIXELS = 5;
    GREEN_HORIZON_LOWER_THRESHOLD_MULT = 1;
    GREEN_HORIZON_UPPER_THRESHOLD_MULT = 2.5;
    std::ifstream in(filename.c_str());
    std::string name;
    int         ival;
    float       fval;
    bool        bval;
    std::string sval;
    while(in.good()) {
        getline(in, name, ':');
        boost::trim(name);
        boost::to_upper(name);
        if(name.compare("DO_RADIAL_CORRECTION") == 0) {
            in >> bval;
            DO_RADIAL_CORRECTION = bval;
        }
        else if(name.compare("RADIAL_CORRECTION_COEFFICIENT") == 0) {
            in >> fval;
            RADIAL_CORRECTION_COEFFICIENT = fval;
        }
        else if(name.compare("THROWOUT_ON_ABOVE_KIN_HOR_GOALS") == 0) {
            in >> bval;
            THROWOUT_ON_ABOVE_KIN_HOR_GOALS = bval;
        }
        else if(name.compare("THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_GOALS") == 0) {
            in >> bval;
            THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_GOALS = bval;
        }
        else if(name.compare("THROWOUT_DISTANT_GOALS") == 0) {
            in >> bval;
            THROWOUT_DISTANT_GOALS = bval;
        }
        else if(name.compare("MAX_DISTANCE_METHOD_DISCREPENCY_GOALS") == 0) {
            in >> fval;
            MAX_DISTANCE_METHOD_DISCREPENCY_GOALS = fval;
        }
        else if(name.compare("MAX_GOAL_DISTANCE") == 0) {
            in >> fval;
            MAX_GOAL_DISTANCE = fval;
        }
        else if(name.compare("THROWOUT_ON_ABOVE_KIN_HOR_BEACONS") == 0) {
            in >> bval;
            THROWOUT_ON_ABOVE_KIN_HOR_BEACONS = bval;
        }
        else if(name.compare("THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BEACONS") == 0) {
            in >> bval;
            THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BEACONS = bval;
        }
        else if(name.compare("THROWOUT_DISTANT_BEACONS") == 0) {
            in >> bval;
            THROWOUT_DISTANT_BEACONS = bval;
        }
        else if(name.compare("MAX_DISTANCE_METHOD_DISCREPENCY_BEACONS") == 0) {
            in >> fval;
            MAX_DISTANCE_METHOD_DISCREPENCY_BEACONS = fval;
        }
        else if(name.compare("MAX_BEACON_DISTANCE") == 0) {
            in >> fval;
            MAX_BEACON_DISTANCE = fval;
        }
        else if(name.compare("THROWOUT_ON_ABOVE_KIN_HOR_BALL") == 0) {
            in >> bval;
            THROWOUT_ON_ABOVE_KIN_HOR_BALL = bval;
        }
        else if(name.compare("THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BALL") == 0) {
            in >> bval;
            THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BALL = bval;
        }
        else if(name.compare("THROWOUT_SMALL_BALLS") == 0) {
            in >> bval;
            THROWOUT_SMALL_BALLS = bval;
        }
        else if(name.compare("MAX_DISTANCE_METHOD_DISCREPENCY_BALL") == 0) {
            in >> fval;
            MAX_DISTANCE_METHOD_DISCREPENCY_BALL = fval;
        }
        else if(name.compare("MIN_BALL_DIAMETER_PIXELS") == 0) {
            in >> fval;
            MIN_BALL_DIAMETER_PIXELS = fval;
        }
        else if(name.compare("THROWOUT_DISTANT_BALLS") == 0) {
            in >> bval;
            THROWOUT_DISTANT_BALLS = bval;
        }
        else if(name.compare("MAX_BALL_DISTANCE") == 0) {
            in >> fval;
            MAX_BALL_DISTANCE = fval;
        }
        else if(name.compare("D2P_INCLUDE_BODY_PITCH") == 0) {
            in >> bval;
            D2P_INCLUDE_BODY_PITCH = bval;
        }
        else if(name.compare("D2P_ANGLE_CORRECTION") == 0) {
            int config_player;
            in >> config_player;
            if(config_player == Blackboard->GameInfo->getPlayerNumber()) {
                in >> fval;
                D2P_ANGLE_CORRECTION = fval;
            }
        }
        else if(name.compare("BALL_DISTANCE_POSITION_BOTTOM") == 0) {
            in >> bval;
            BALL_DISTANCE_POSITION_BOTTOM = bval;
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
        else if(name.compare("BEACON_DISTANCE_METHOD") == 0) {
            in >> sval;
            boost::trim(sval);
            boost::to_upper(sval);
            BEACON_DISTANCE_METHOD = getDistanceMethodFromName(sval);
        }
        else if(name.compare("BALL_EDGE_THRESHOLD") == 0) {
            in >> ival;
            BALL_EDGE_THRESHOLD = ival;
        }
        else if(name.compare("BALL_ORANGE_TOLERANCE") == 0) {
            in >> ival;
            BALL_ORANGE_TOLERANCE = ival;
        }
        else if(name.compare("BALL_MIN_PERCENT_ORANGE") == 0) {
            in >> fval;
            BALL_MIN_PERCENT_ORANGE = fval;
        }
        else if(name.compare("GOAL_MIN_PERCENT_YELLOW") == 0) {
            in >> fval;
            GOAL_MIN_PERCENT_YELLOW = fval;
        }
        else if(name.compare("GOAL_MIN_PERCENT_BLUE") == 0) {
            in >> fval;
            GOAL_MIN_PERCENT_BLUE = fval;
        }
        else if(name.compare("BEACON_MIN_PERCENT_YELLOW") == 0) {
            in >> fval;
            BEACON_MIN_PERCENT_YELLOW = fval;
        }
        else if(name.compare("BEACON_MIN_PERCENT_BLUE") == 0) {
            in >> fval;
            BEACON_MIN_PERCENT_BLUE = fval;
        }
        else if(name.compare("MIN_DISTANCE_FROM_HORIZON") == 0) {
            in >> ival;
            MIN_DISTANCE_FROM_HORIZON = ival;
        }
        else if(name.compare("MIN_CONSECUTIVE_POINTS") == 0) {
            in >> ival;
            MIN_CONSECUTIVE_POINTS = ival;
        }
        else if(name.compare("GOAL_WIDTH") == 0) {
            in >> fval;
            GOAL_WIDTH = fval;
        }
        else if(name.compare("DISTANCE_BETWEEN_POSTS") == 0) {
            in >> fval;
            DISTANCE_BETWEEN_POSTS = fval;
        }
        else if(name.compare("BALL_WIDTH") == 0) {
            in >> fval;
            BALL_WIDTH = fval;
        }
        else if(name.compare("BEACON_WIDTH") == 0) {
            in >> fval;
            BEACON_WIDTH = fval;
        }
        else if(name.compare("THROWOUT_INSIGNIFICANT_GOALS") == 0) {
            in >> bval;
            THROWOUT_INSIGNIFICANT_GOALS = bval;
        }
        else if(name.compare("MIN_TRANSITIONS_FOR_SIGNIFICANCE_GOALS") == 0) {
            in >> ival;
            MIN_TRANSITIONS_FOR_SIGNIFICANCE_GOALS = ival;
        }
        else if(name.compare("THROWOUT_INSIGNIFICANT_BEACONS") == 0) {
            in >> bval;
            THROWOUT_INSIGNIFICANT_BEACONS = bval;
        }
        else if(name.compare("MIN_TRANSITIONS_FOR_SIGNIFICANCE_BEACONS") == 0) {
            in >> ival;
            MIN_TRANSITIONS_FOR_SIGNIFICANCE_BEACONS = ival;
        }
        else if(name.compare("THROWOUT_INSIGNIFICANT_BALLS") == 0) {
            in >> bval;
            THROWOUT_INSIGNIFICANT_BALLS = bval;
        }
        else if(name.compare("MIN_TRANSITIONS_FOR_SIGNIFICANCE_BALL") == 0) {
            in >> ival;
            MIN_TRANSITIONS_FOR_SIGNIFICANCE_BALL = ival;
        }
        else if(name.compare("HORIZONTAL_SCANLINE_SPACING") == 0) {
            in >> ival;
            HORIZONTAL_SCANLINE_SPACING = ival;
        }
        else if(name.compare("VERTICAL_SCANLINE_SPACING") == 0) {
            in >> ival;
            VERTICAL_SCANLINE_SPACING = ival;
        }
        else if(name.compare("GREEN_HORIZON_SCAN_SPACING") == 0) {
            in >> ival;
            GREEN_HORIZON_SCAN_SPACING = ival;
        }
        else if(name.compare("GREEN_HORIZON_MIN_GREEN_PIXELS") == 0) {
            in >> ival;
            GREEN_HORIZON_MIN_GREEN_PIXELS = ival;
        }
        else if(name.compare("GREEN_HORIZON_LOWER_THRESHOLD_MULT") == 0) {
            in >> fval;
            GREEN_HORIZON_LOWER_THRESHOLD_MULT = fval;
        }
        else if(name.compare("GREEN_HORIZON_UPPER_THRESHOLD_MULT") == 0) {
            in >> fval;
            GREEN_HORIZON_UPPER_THRESHOLD_MULT = fval;
        }
        else if(name.compare("THROWOUT_NARROW_GOALS") == 0) {
            in >> bval;
            THROWOUT_NARROW_GOALS = bval;
        }
        else if(name.compare("MIN_GOAL_WIDTH") == 0) {
            in >> ival;
            MIN_GOAL_WIDTH = ival;
        }
        else if(name.compare("GOAL_EDGE_RATIO") == 0) {
            in >> fval;
            GOAL_EDGE_RATIO = fval;
        }
        else if(name.compare("MIN_GOAL_SEPARATION") == 0) {
            in >> ival;
            MIN_GOAL_SEPARATION = ival;
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
    
    debug << "VisionConstants::loadFromFile-" << std::endl;
    
    
    debug << "\tDO_RADIAL_CORRECTION: " << DO_RADIAL_CORRECTION << std::endl;
    debug << "\tRADIAL_CORRECTION_COEFFICIENT: " << RADIAL_CORRECTION_COEFFICIENT << std::endl;

    debug << "\tTHROWOUT_ON_ABOVE_KIN_HOR_GOALS: " << THROWOUT_ON_ABOVE_KIN_HOR_GOALS << std::endl;
    debug << "\tTHROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_GOALS: " << THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_GOALS << std::endl;
    debug << "\tMAX_DISTANCE_METHOD_DISCREPENCY_GOALS: " << MAX_DISTANCE_METHOD_DISCREPENCY_GOALS << std::endl;
    debug << "\tTHROWOUT_DISTANT_GOALS: " << THROWOUT_DISTANT_GOALS << std::endl;
    debug << "\tMAX_GOAL_DISTANCE: " << MAX_GOAL_DISTANCE << std::endl;
    debug << "\tTHROWOUT_INSIGNIFICANT_GOALS: " << THROWOUT_INSIGNIFICANT_GOALS << std::endl;
    debug << "\tMIN_TRANSITIONS_FOR_SIGNIFICANCE_GOALS: " << MIN_TRANSITIONS_FOR_SIGNIFICANCE_GOALS << std::endl;
    debug << "\tTHROWOUT_NARROW_GOALS: " << THROWOUT_NARROW_GOALS << std::endl;
    debug << "\tMIN_GOAL_WIDTH: " << MIN_GOAL_WIDTH << std::endl;
    debug << "\tGOAL_EDGE_RATIO: " << GOAL_EDGE_RATIO << std::endl;

    debug << "\tTHROWOUT_ON_ABOVE_KIN_HOR_BEACONS: " << THROWOUT_ON_ABOVE_KIN_HOR_BEACONS << std::endl;
    debug << "\tTHROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BEACONS: " << THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BEACONS << std::endl;
    debug << "\tMAX_DISTANCE_METHOD_DISCREPENCY_BEACONS: " << MAX_DISTANCE_METHOD_DISCREPENCY_BEACONS << std::endl;
    debug << "\tTHROWOUT_DISTANT_BEACONS: " << THROWOUT_DISTANT_BEACONS << std::endl;
    debug << "\tMAX_BEACON_DISTANCE: " << MAX_BEACON_DISTANCE << std::endl;
    debug << "\tTHROWOUT_INSIGNIFICANT_BEACONS: " << THROWOUT_INSIGNIFICANT_BEACONS << std::endl;
    debug << "\tMIN_TRANSITIONS_FOR_SIGNIFICANCE_BEACONS: " << MIN_TRANSITIONS_FOR_SIGNIFICANCE_BEACONS << std::endl;

    debug << "\tTHROWOUT_ON_ABOVE_KIN_HOR_BALL: " << THROWOUT_ON_ABOVE_KIN_HOR_BALL << std::endl;
    debug << "\tTHROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BALL: " << THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BALL << std::endl;
    debug << "\tMAX_DISTANCE_METHOD_DISCREPENCY_BALL: " << MAX_DISTANCE_METHOD_DISCREPENCY_BALL << std::endl;
    debug << "\tTHROWOUT_SMALL_BALLS: " << THROWOUT_SMALL_BALLS << std::endl;
    debug << "\tMIN_BALL_DIAMETER_PIXELS: " << MIN_BALL_DIAMETER_PIXELS << std::endl;
    debug << "\tTHROWOUT_INSIGNIFICANT_BALLS: " << THROWOUT_INSIGNIFICANT_BALLS << std::endl;
    debug << "\tMIN_TRANSITIONS_FOR_SIGNIFICANCE_BALL: " << MIN_TRANSITIONS_FOR_SIGNIFICANCE_BALL << std::endl;

    debug << "\tD2P_INCLUDE_BODY_PITCH: " << D2P_INCLUDE_BODY_PITCH << std::endl;
    debug << "\tD2P_ANGLE_CORRECTION: " << D2P_ANGLE_CORRECTION << std::endl;
    debug << "\tBALL_DISTANCE_POSITION_BOTTOM: " << BALL_DISTANCE_POSITION_BOTTOM << std::endl;

    debug << "\tBALL_DISTANCE_METHOD: " << getDistanceMethodName(BALL_DISTANCE_METHOD) << std::endl;
    debug << "\tGOAL_DISTANCE_METHOD: " << getDistanceMethodName(GOAL_DISTANCE_METHOD) << std::endl;
    debug << "\tBEACON_DISTANCE_METHOD: " << getDistanceMethodName(BEACON_DISTANCE_METHOD) << std::endl;

    debug << "\tBALL_EDGE_THRESHOLD: " << BALL_EDGE_THRESHOLD << std::endl;
    debug << "\tBALL_ORANGE_TOLERANCE: " << BALL_ORANGE_TOLERANCE << std::endl;
    debug << "\tBALL_MIN_PERCENT_ORANGE: " << BALL_MIN_PERCENT_ORANGE << std::endl;
    debug << "\tGOAL_MIN_PERCENT_YELLOW: " << GOAL_MIN_PERCENT_YELLOW << std::endl;
    debug << "\tGOAL_MIN_PERCENT_BLUE: " << GOAL_MIN_PERCENT_BLUE << std::endl;
    debug << "\tBEACON_MIN_PERCENT_YELLOW: " << BEACON_MIN_PERCENT_YELLOW << std::endl;
    debug << "\tBEACON_MIN_PERCENT_BLUE: " << BEACON_MIN_PERCENT_BLUE << std::endl;
    debug << "\tMIN_GOAL_SEPARATION: " << MIN_GOAL_SEPARATION << std::endl;

    debug << "\tMIN_DISTANCE_FROM_HORIZON: " << MIN_DISTANCE_FROM_HORIZON << std::endl;
    debug << "\tMIN_CONSECUTIVE_POINTS: " << MIN_CONSECUTIVE_POINTS << std::endl;

    debug << "\tGOAL_WIDTH: " << GOAL_WIDTH << std::endl;
    debug << "\tDISTANCE_BETWEEN_POSTS: " << DISTANCE_BETWEEN_POSTS << std::endl;
    debug << "\tBALL_WIDTH: " << BALL_WIDTH << std::endl;
    debug << "\tBEACON_WIDTH: " << BEACON_WIDTH << std::endl;

    debug << "\tHORIZONTAL_SCANLINE_SPACING: " << HORIZONTAL_SCANLINE_SPACING << std::endl;
    debug << "\tVERTICAL_SCANLINE_SPACING: " << VERTICAL_SCANLINE_SPACING << std::endl;
    debug << "\tGREEN_HORIZON_SCAN_SPACING: " << GREEN_HORIZON_SCAN_SPACING << std::endl;
    debug << "\tGREEN_HORIZON_MIN_GREEN_PIXELS: " << GREEN_HORIZON_MIN_GREEN_PIXELS << std::endl;
    debug << "\tGREEN_HORIZON_LOWER_THRESHOLD_MULT: " << GREEN_HORIZON_LOWER_THRESHOLD_MULT << std::endl;
    debug << "\tGREEN_HORIZON_UPPER_THRESHOLD_MULT: " << GREEN_HORIZON_UPPER_THRESHOLD_MULT << std::endl;

}

VisionConstants::DistanceMethod VisionConstants::getDistanceMethodFromName(std::string name)
{
    if(name.compare("WIDTH") == 0)
        return Width;
    else if(name.compare("D2P") == 0)
        return D2P;
    else if(name.compare("LEAST") == 0)
        return Least;
    else if(name.compare("AVERAGE") == 0)
        return Average;

    //no match - return default
    #ifdef DEBUG_VISION_VERBOSITY_ON
        debug << "VisionConstants::getDistanceMethodFromName - unmatched method name: " << name << " used D2P instead" << std::endl;
    #endif
    return D2P; //default
}

std::string VisionConstants::getDistanceMethodName(VisionConstants::DistanceMethod method)
{
    switch(method) {
    case Width:     return "WIDTH";
    case D2P:       return "D2P";
    case Average:   return "AVERAGE";
    case Least:     return "LEAST";
    }
}
