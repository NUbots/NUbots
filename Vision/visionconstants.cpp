#include "visionconstants.h"

#include "nubotdataconfig.h"
#include "debug.h"
#include "debugverbosityvision.h"

#include <fstream>
#include <boost/algorithm/string.hpp>
//#include <iostream>

bool VisionConstants::THROWOUT_ON_ABOVE_KIN_HOR_GOALS;
bool VisionConstants::THROWOUT_ON_ABOVE_KIN_HOR_BALL;
bool VisionConstants::THROWOUT_ON_DISTANCE_DISCREPENCY_GOALS;
bool VisionConstants::THROWOUT_ON_DISTANCE_DISCREPENCY_BALL;
float VisionConstants::MAX_DISTANCE_DISCREPENCY_GOALS; 
float VisionConstants::MAX_DISTANCE_DISCREPENCY_BALL;     

bool VisionConstants::D2P_INCLUDE_BODY_PITCH;
bool VisionConstants::BALL_DISTANCE_POSITION_BOTTOM;
VisionConstants::BallDistanceMethod VisionConstants::BALL_DISTANCE_METHOD;

int VisionConstants::BALL_EDGE_THRESHOLD;
int VisionConstants::BALL_ORANGE_TOLERANCE;

float VisionConstants::GOAL_WIDTH;
float VisionConstants::DISTANCE_BETWEEN_POSTS;
float VisionConstants::BALL_WIDTH;

VisionConstants::VisionConstants()
{
}

/*! @brief Loads vision constants and options from the given file.
  * @param filename The name of the file (located in the Config directory).
  */
void VisionConstants::loadFromFile(std::string filename) 
{
    std::ifstream in(filename.c_str());
    std::string name;
    int         ival;
    float       fval;
    bool        bval;
    std::string sval;
    while(in.good()) {
        getline(in, name, ':');
        if(name.compare("THROWOUT_ON_ABOVE_KIN_HOR_GOALS") == 0) {
            in >> bval;
            THROWOUT_ON_ABOVE_KIN_HOR_GOALS = bval;
        }
        else if(name.compare("THROWOUT_ON_ABOVE_KIN_HOR_BALL") == 0) {
            in >> bval;
            THROWOUT_ON_ABOVE_KIN_HOR_BALL = bval;
        }
        else if(name.compare("THROWOUT_ON_DISTANCE_DISCREPENCY_GOALS") == 0) {
            in >> bval;
            THROWOUT_ON_DISTANCE_DISCREPENCY_GOALS = bval;
        }
        else if(name.compare("THROWOUT_ON_DISTANCE_DISCREPENCY_BALL") == 0) {
            in >> bval;
            THROWOUT_ON_DISTANCE_DISCREPENCY_BALL = bval;
        }
        else if(name.compare("MAX_DISTANCE_DISCREPENCY_GOALS") == 0) {
            in >> fval;
            MAX_DISTANCE_DISCREPENCY_GOALS = fval;
        }
        else if(name.compare("MAX_DISTANCE_DISCREPENCY_BALL") == 0) {
            in >> fval;
            MAX_DISTANCE_DISCREPENCY_BALL = fval;
        }
        else if(name.compare("BALL_EDGE_THRESHOLD") == 0) {
            in >> ival;
            BALL_EDGE_THRESHOLD = ival;
        }
        else if(name.compare("BALL_ORANGE_TOLERANCE") == 0) {
            in >> ival;
            BALL_ORANGE_TOLERANCE = ival;
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
        else if(name.compare("BALL_DISTANCE_POSITION_BOTTOM") == 0) {
            in >> bval;
            BALL_DISTANCE_POSITION_BOTTOM = bval;
        }
        else if(name.compare("D2P_INCLUDE_BODY_PITCH") == 0) {
            in >> bval;
            D2P_INCLUDE_BODY_PITCH = bval;
        }
        else if(name.compare("BALL_DISTANCE_METHOD") == 0) {
            in >> sval;
            boost::trim(sval);
            BALL_DISTANCE_METHOD = getBallMethodFromName(sval);
        }
        else {
            errorlog << "VisionConstants::loadFromFile - unknown constant: " << name << std::endl;
        }
        
        // ignore the rest of the line
        in.ignore(128, '\n');
        //force eofbit in the case of last rule
        in.peek();
    }
    
    debug << "VisionConstants::loadFromFile-" << std::endl;
    debug << "\tTHROWOUT_ON_ABOVE_KIN_HOR_GOALS: " << THROWOUT_ON_ABOVE_KIN_HOR_GOALS << std::endl;
    debug << "\tTHROWOUT_ON_ABOVE_KIN_HOR_BALL: " << THROWOUT_ON_ABOVE_KIN_HOR_BALL << std::endl;
    debug << "\tTHROWOUT_ON_DISTANCE_DISCREPENCY_GOALS: " << THROWOUT_ON_DISTANCE_DISCREPENCY_GOALS << std::endl;
    debug << "\tTHROWOUT_ON_DISTANCE_DISCREPENCY_BALL: " << THROWOUT_ON_DISTANCE_DISCREPENCY_BALL << std::endl;
    debug << "\tMAX_DISTANCE_DISCREPENCY_GOALS: " << MAX_DISTANCE_DISCREPENCY_GOALS << std::endl; 
    debug << "\tMAX_DISTANCE_DISCREPENCY_BALL: " << MAX_DISTANCE_DISCREPENCY_BALL << std::endl;     
    debug << "\tBALL_EDGE_THRESHOLD: " << BALL_EDGE_THRESHOLD << std::endl;
    debug << "\tBALL_ORANGE_TOLERANCE: " << BALL_ORANGE_TOLERANCE << std::endl;
    debug << "\tGOAL_WIDTH: " << GOAL_WIDTH << std::endl;
    debug << "\tDISTANCE_BETWEEN_POSTS: " << DISTANCE_BETWEEN_POSTS << std::endl;
    debug << "\tBALL_WIDTH: " << BALL_WIDTH << std::endl;
    debug << "\tBALL_DISTANCE_POSITION_BOTTOM: " << BALL_DISTANCE_POSITION_BOTTOM << std::endl;
    debug << "\tD2P_INCLUDE_BODY_PITCH: " << D2P_INCLUDE_BODY_PITCH << std::endl;
    debug << "\tBALL_DISTANCE_METHOD: " << getBallMethodName(BALL_DISTANCE_METHOD) << std::endl;
    
    in.close();
}


VisionConstants::BallDistanceMethod VisionConstants::getBallMethodFromName(std::string name)
{
    if(name.compare("Width") == 0) {
        return Width;
    }
    else if(name.compare("D2P") == 0) {
        return D2P;
    }
    else if(name.compare("Least") == 0) {
        return Least;
    }
    else if(name.compare("Average") == 0) {
        return Average;
    }
    else {
        #ifdef DEBUG_VISION_VERBOSITY_ON
            debug << "VisionConstants::getBallMethodFromName - unmatched method name: " << name << " used Least instead" << std::endl;
        #endif
        return Least; //default 
    }
}

std::string VisionConstants::getBallMethodName(VisionConstants::BallDistanceMethod method)
{
    switch(method) {
    case Width:     return "Width";
    case D2P:       return "D2P";
    case Average:   return "Average";
    case Least:     return "Least";
    }
}
