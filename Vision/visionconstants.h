#ifndef VISIONCONSTANTS_H
#define VISIONCONSTANTS_H

#include <string>

class VisionConstants
{
public:

    static int BALL_EDGE_THRESHOLD;
    static int BALL_ORANGE_TOLERANCE;
    //! For the Goals
    static float GOAL_WIDTH;
    static float DISTANCE_BETWEEN_POSTS;
    
    static void loadFromFile(std::string filename);
    
private:
    VisionConstants();  //so noone can make an object of this type
};

#endif // VISIONCONSTANTS_H
