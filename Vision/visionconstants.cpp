#include "visionconstants.h"

int VisionConstants::BALL_EDGE_THRESHOLD;
int VisionConstants::BALL_ORANGE_TOLERANCE;
float VisionConstants::GOAL_WIDTH;
float VisionConstants::DISTANCE_BETWEEN_POSTS;
float VisionConstants::BALL_WIDTH;

VisionConstants::VisionConstants()
{
}

void VisionConstants::loadFromFile(std::string filename) 
{
    //! @todo implement properly
    GOAL_WIDTH = 11; //lab = 11cm, official=10cm
    DISTANCE_BETWEEN_POSTS = 140;
    BALL_EDGE_THRESHOLD = 15;
    BALL_ORANGE_TOLERANCE = 25;
    BALL_WIDTH = 6.5;
}
