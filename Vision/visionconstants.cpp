#include "visionconstants.h"

#include "nubotdataconfig.h"
#include "debug.h"
#include "debugverbosityvision.h"

#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/GameInformation/GameInformation.h"
#include "Tools/Math/General.h"

#include <fstream>
#include <boost/algorithm/string.hpp>

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
float VisionConstants::GOAL_HEIGHT_TO_WIDTH_RATIO_LOW;
float VisionConstants::GOAL_HEIGHT_TO_WIDTH_RATIO_HIGH;
//! Beacon filtering constants
bool VisionConstants::THROWOUT_ON_ABOVE_KIN_HOR_BEACONS;
bool VisionConstants::THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BEACONS;
float VisionConstants::MAX_DISTANCE_METHOD_DISCREPENCY_BEACONS;
bool VisionConstants::THROWOUT_DISTANT_BEACONS;
float VisionConstants::MAX_BEACON_DISTANCE;
bool VisionConstants::THROWOUT_INSIGNIFICANT_BEACONS;
int VisionConstants::MIN_TRANSITIONS_FOR_SIGNIFICANCE_BEACONS;
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
float VisionConstants::D2P_ANGLE_CORRECTION;
bool VisionConstants::BALL_DISTANCE_POSITION_BOTTOM;
//! Distance method options
VisionConstants::DistanceMethod VisionConstants::BALL_DISTANCE_METHOD;
VisionConstants::DistanceMethod VisionConstants::GOAL_DISTANCE_METHOD;
VisionConstants::DistanceMethod VisionConstants::BEACON_DISTANCE_METHOD;
//! Field-object detection constants
int VisionConstants::BALL_EDGE_THRESHOLD;
int VisionConstants::BALL_ORANGE_TOLERANCE;
float VisionConstants::BALL_MIN_PERCENT_ORANGE;
float VisionConstants::GOAL_MIN_PERCENT_YELLOW;
float VisionConstants::GOAL_MIN_PERCENT_BLUE;
float VisionConstants::BEACON_MIN_PERCENT_YELLOW;
float VisionConstants::BEACON_MIN_PERCENT_BLUE;
int VisionConstants::MIN_GOAL_SEPARATION;
//! Obstacle detection constants
int VisionConstants::MIN_DISTANCE_FROM_HORIZON;
int VisionConstants::MIN_CONSECUTIVE_POINTS;
//! Field dimension constants
float VisionConstants::GOAL_WIDTH;
float VisionConstants::DISTANCE_BETWEEN_POSTS;
float VisionConstants::BALL_WIDTH;
float VisionConstants::BEACON_WIDTH;
//! ScanLine options
unsigned int VisionConstants::HORIZONTAL_SCANLINE_SPACING;
unsigned int VisionConstants::VERTICAL_SCANLINE_SPACING;
unsigned int VisionConstants::GREEN_HORIZON_SCAN_SPACING;
unsigned int VisionConstants::GREEN_HORIZON_MIN_GREEN_PIXELS;
float VisionConstants::GREEN_HORIZON_LOWER_THRESHOLD_MULT;
float VisionConstants::GREEN_HORIZON_UPPER_THRESHOLD_MULT;
//! Split and Merge constants
unsigned int VisionConstants::SAM_MAX_POINTS;
unsigned int VisionConstants::SAM_MAX_LINES;
float VisionConstants::SAM_SPLIT_DISTANCE;
unsigned int VisionConstants::SAM_MIN_POINTS_OVER;
unsigned int VisionConstants::SAM_MIN_POINTS_TO_LINE;
unsigned int VisionConstants::SAM_SPLIT_NOISE_ITERATIONS;
float VisionConstants::SAM_MAX_ANGLE_DIFF_TO_MERGE;
float VisionConstants::SAM_MAX_DISTANCE_TO_MERGE;
unsigned int VisionConstants::SAM_MIN_POINTS_TO_LINE_FINAL;
float VisionConstants::SAM_MIN_LINE_R2_FIT;
float VisionConstants::SAM_MAX_LINE_MSD;
bool VisionConstants::SAM_CLEAR_SMALL;
bool VisionConstants::SAM_CLEAR_DIRTY;

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
    GOAL_HEIGHT_TO_WIDTH_RATIO_LOW = 3,
    GOAL_HEIGHT_TO_WIDTH_RATIO_HIGH = 15;
    MIN_GOAL_SEPARATION = 20;
    SAM_MAX_POINTS = 500;
    SAM_MAX_LINES = 15;
    SAM_SPLIT_DISTANCE = 1.0;
    SAM_MIN_POINTS_OVER = 2;
    SAM_MIN_POINTS_TO_LINE = 3;
    SAM_SPLIT_NOISE_ITERATIONS = 2;
    SAM_MAX_ANGLE_DIFF_TO_MERGE = 0.1;
    SAM_MAX_DISTANCE_TO_MERGE = 10;
    SAM_MIN_POINTS_TO_LINE_FINAL = 5;
    SAM_MIN_LINE_R2_FIT = 0.95;
    SAM_MAX_LINE_MSD = 1;
    SAM_CLEAR_SMALL = true;
    SAM_CLEAR_DIRTY = true;

    std::ifstream in(filename.c_str());
    std::string name;
    std::string sval;
    while(in.good()) {
        getline(in, name, ':');
        boost::trim(name);
        boost::to_upper(name);
        if(name.compare("DO_RADIAL_CORRECTION") == 0) {
            in >> DO_RADIAL_CORRECTION;
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
        else if(name.compare("THROWOUT_ON_ABOVE_KIN_HOR_BEACONS") == 0) {
            in >> THROWOUT_ON_ABOVE_KIN_HOR_BEACONS;
        }
        else if(name.compare("THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BEACONS") == 0) {
            in >> THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BEACONS;
        }
        else if(name.compare("THROWOUT_DISTANT_BEACONS") == 0) {
            in >> THROWOUT_DISTANT_BEACONS;
        }
        else if(name.compare("MAX_DISTANCE_METHOD_DISCREPENCY_BEACONS") == 0) {
            in >> MAX_DISTANCE_METHOD_DISCREPENCY_BEACONS;
        }
        else if(name.compare("MAX_BEACON_DISTANCE") == 0) {
            in >> MAX_BEACON_DISTANCE;
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
        else if(name.compare("D2P_ANGLE_CORRECTION") == 0) {
            int config_player;
            in >> config_player;
            #if (defined TARGET_IS_PC || defined TARGET_IS_TRAINING)
                in >> D2P_ANGLE_CORRECTION;
                D2P_ANGLE_CORRECTION = 0;   //discard the value
            #else
                if(config_player == Blackboard->GameInfo->getPlayerNumber()) {
                    in >> D2P_ANGLE_CORRECTION;
                }
            #endif
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
        else if(name.compare("BEACON_DISTANCE_METHOD") == 0) {
            in >> sval;
            boost::trim(sval);
            boost::to_upper(sval);
            BEACON_DISTANCE_METHOD = getDistanceMethodFromName(sval);
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
        else if(name.compare("BEACON_MIN_PERCENT_YELLOW") == 0) {
            in >> BEACON_MIN_PERCENT_YELLOW;
        }
        else if(name.compare("BEACON_MIN_PERCENT_BLUE") == 0) {
            in >> BEACON_MIN_PERCENT_BLUE;
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
        else if(name.compare("DISTANCE_BETWEEN_POSTS") == 0) {
            in >> DISTANCE_BETWEEN_POSTS;
        }
        else if(name.compare("BALL_WIDTH") == 0) {
            in >> BALL_WIDTH;
        }
        else if(name.compare("BEACON_WIDTH") == 0) {
            in >> BEACON_WIDTH;
        }
        else if(name.compare("THROWOUT_INSIGNIFICANT_GOALS") == 0) {
            in >> THROWOUT_INSIGNIFICANT_GOALS;
        }
        else if(name.compare("MIN_TRANSITIONS_FOR_SIGNIFICANCE_GOALS") == 0) {
            in >> MIN_TRANSITIONS_FOR_SIGNIFICANCE_GOALS;
        }
        else if(name.compare("THROWOUT_INSIGNIFICANT_BEACONS") == 0) {
            in >> THROWOUT_INSIGNIFICANT_BEACONS;
        }
        else if(name.compare("MIN_TRANSITIONS_FOR_SIGNIFICANCE_BEACONS") == 0) {
            in >> MIN_TRANSITIONS_FOR_SIGNIFICANCE_BEACONS;
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
        else if(name.compare("GREEN_HORIZON_LOWER_THRESHOLD_MULT") == 0) {
            in >> GREEN_HORIZON_LOWER_THRESHOLD_MULT;
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
        else if(name.compare("GOAL_HEIGHT_TO_WIDTH_RATIO_LOW") == 0) {
            in >> GOAL_HEIGHT_TO_WIDTH_RATIO_LOW;
        }
        else if(name.compare("GOAL_HEIGHT_TO_WIDTH_RATIO_HIGH") == 0) {
            in >> GOAL_HEIGHT_TO_WIDTH_RATIO_HIGH;
        }
        else if(name.compare("MIN_GOAL_SEPARATION") == 0) {
            in >> MIN_GOAL_SEPARATION;
        }
        else if(name.compare("SAM_MAX_LINES") == 0) {
            in >> SAM_MAX_POINTS;
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
        else if(name.compare("SAM_SPLIT_NOISE_ITERATIONS") == 0) {
            in >> SAM_SPLIT_NOISE_ITERATIONS;
        }
        else if(name.compare("SAM_MAX_ANGLE_DIFF_TO_MERGE") == 0) {
            in >> SAM_MAX_ANGLE_DIFF_TO_MERGE;
        }
        else if(name.compare("SAM_MAX_INTERCEPT_DIFF_TO_MERGE") == 0) {
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
    debug << "\tTHROWOUT_SHORT_GOALS: " << THROWOUT_SHORT_GOALS << std::endl;
    debug << "\tMIN_GOAL_HEIGHT: " << MIN_GOAL_HEIGHT << std::endl;
    debug << "\tGOAL_HEIGHT_TO_WIDTH_RATIO_LOW: " << GOAL_HEIGHT_TO_WIDTH_RATIO_LOW << std::endl;
    debug << "\tGOAL_HEIGHT_TO_WIDTH_RATIO_HIGH: " << GOAL_HEIGHT_TO_WIDTH_RATIO_HIGH << std::endl;
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

    debug << "\tSAM_MAX_POINTS: " << SAM_MAX_POINTS << std::endl;
    debug << "\tSAM_MAX_LINES: " << SAM_MAX_LINES << std::endl;
    debug << "\tSAM_SPLIT_DISTANCE: " << SAM_SPLIT_DISTANCE << std::endl;
    debug << "\tSAM_MIN_POINTS_OVER: " << SAM_MIN_POINTS_OVER << std::endl;
    debug << "\tSAM_MIN_POINTS_TO_LINE: " << SAM_MIN_POINTS_TO_LINE << std::endl;
    debug << "\tSAM_SPLIT_NOISE_ITERATIONS: " << SAM_SPLIT_NOISE_ITERATIONS << std::endl;
    debug << "\tSAM_MAX_ANGLE_DIFF_TO_MERGE: " << SAM_MAX_ANGLE_DIFF_TO_MERGE << std::endl;
    debug << "\tSAM_MAX_DISTANCE_TO_MERGE: " << SAM_MAX_DISTANCE_TO_MERGE << std::endl;
    debug << "\tSAM_MIN_POINTS_TO_LINE_FINAL: " << SAM_MIN_POINTS_TO_LINE_FINAL << std::endl;
    debug << "\tSAM_MIN_LINE_R2_FIT: " << SAM_MIN_LINE_R2_FIT << std::endl;
    debug << "\tSAM_MAX_LINE_MSD: " << SAM_MAX_LINE_MSD << std::endl;
    debug << "\tSAM_CLEAR_SMALL: " << SAM_CLEAR_SMALL << std::endl;
    debug << "\tSAM_CLEAR_DIRTY: " << SAM_CLEAR_DIRTY << std::endl;

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

    THROWOUT_ON_ABOVE_KIN_HOR_BEACONS = val;
    THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BEACONS = val;
    THROWOUT_DISTANT_BEACONS = val;
    THROWOUT_INSIGNIFICANT_BEACONS = val;

    THROWOUT_ON_ABOVE_KIN_HOR_BALL = val;
    THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BALL = val;
    THROWOUT_SMALL_BALLS = val;
    THROWOUT_INSIGNIFICANT_BALLS = val;
    THROWOUT_DISTANT_BALLS = val;

    D2P_INCLUDE_BODY_PITCH = val;

    SAM_CLEAR_SMALL = val;
    SAM_CLEAR_DIRTY = val;

}

vector<Parameter> VisionConstants::getAllOptimisable()
{
    vector<Parameter> params;
    //! Goal filtering constants
    params.push_back(Parameter("MIN_TRANSITIONS_FOR_SIGNIFICANCE_GOALS", MIN_TRANSITIONS_FOR_SIGNIFICANCE_GOALS, 1, 500));
    params.push_back(Parameter("MIN_GOAL_WIDTH", MIN_GOAL_WIDTH, 0, 320));
    params.push_back(Parameter("MIN_GOAL_HEIGHT", MIN_GOAL_HEIGHT, 0, 240));

    params.push_back(Parameter("GOAL_HEIGHT_TO_WIDTH_RATIO_LOW", GOAL_HEIGHT_TO_WIDTH_RATIO_LOW, 0, 240));
    params.push_back(Parameter("GOAL_HEIGHT_TO_WIDTH_RATIO_HIGH", GOAL_HEIGHT_TO_WIDTH_RATIO_HIGH, 0, 240));
    //! Beacon filtering constants
    params.push_back(Parameter("MIN_TRANSITIONS_FOR_SIGNIFICANCE_BEACONS", MIN_TRANSITIONS_FOR_SIGNIFICANCE_BEACONS, 1, 500));
    //! Ball filtering constants
    params.push_back(Parameter("MIN_BALL_DIAMETER_PIXELS", MIN_BALL_DIAMETER_PIXELS, 1, 320));
    params.push_back(Parameter("MIN_TRANSITIONS_FOR_SIGNIFICANCE_BALL", MIN_TRANSITIONS_FOR_SIGNIFICANCE_BALL, 1, 500));
    //! Field-object detection constants
    params.push_back(Parameter("BALL_EDGE_THRESHOLD", BALL_EDGE_THRESHOLD, 0, 50));
    params.push_back(Parameter("BALL_ORANGE_TOLERANCE", BALL_ORANGE_TOLERANCE, 0, 50));
    params.push_back(Parameter("BALL_MIN_PERCENT_ORANGE", BALL_MIN_PERCENT_ORANGE, 0, 1));
    params.push_back(Parameter("GOAL_MIN_PERCENT_YELLOW", GOAL_MIN_PERCENT_YELLOW, 0, 1));
    params.push_back(Parameter("GOAL_MIN_PERCENT_BLUE", GOAL_MIN_PERCENT_BLUE, 0, 1));
    params.push_back(Parameter("BEACON_MIN_PERCENT_YELLOW", BEACON_MIN_PERCENT_YELLOW, 0, 1));
    params.push_back(Parameter("BEACON_MIN_PERCENT_BLUE", BEACON_MIN_PERCENT_BLUE, 0, 1));
    params.push_back(Parameter("MIN_GOAL_SEPARATION", MIN_GOAL_SEPARATION, 0, 320));
    //! Obstacle detection constants
    params.push_back(Parameter("MIN_DISTANCE_FROM_HORIZON", MIN_DISTANCE_FROM_HORIZON, 0, 240));
    params.push_back(Parameter("MIN_CONSECUTIVE_POINTS", MIN_CONSECUTIVE_POINTS, 0, 50));
    //! ScanLine options
    //params.push_back(Parameter(HORIZONTAL_SCANLINE_SPACING;
    //params.push_back(Parameter(VERTICAL_SCANLINE_SPACING;
    //params.push_back(Parameter(GREEN_HORIZON_SCAN_SPACING;
    params.push_back(Parameter("GREEN_HORIZON_MIN_GREEN_PIXELS", GREEN_HORIZON_MIN_GREEN_PIXELS, 1, 50));
    params.push_back(Parameter("GREEN_HORIZON_LOWER_THRESHOLD_MULT", GREEN_HORIZON_LOWER_THRESHOLD_MULT, 0, 20));
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
