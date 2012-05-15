#include "visionconstants.h"

unsigned int VisionConstants::GOAL_WIDTH;
unsigned int VisionConstants::DISTANCE_BETWEEN_POSTS;

VisionConstants::VisionConstants()
{
}

void VisionConstants::loadFromFile(std::string filename) 
{
    //! @todo implement properly
    GOAL_WIDTH = 11; //lab = 11cm, official=10cm
    DISTANCE_BETWEEN_POSTS = 140;
}
