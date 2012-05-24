#include "visionconstants.h"

unsigned int VisionConstants::GOAL_WIDTH;
unsigned int VisionConstants::DISTANCE_BETWEEN_POSTS;
int VisionConstants::BALL_EDGE_THRESHOLD;
int VisionConstants::BALL_ORANGE_TOLERANCE;

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
}
