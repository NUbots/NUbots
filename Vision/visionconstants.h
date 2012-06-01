#ifndef VISIONCONSTANTS_H
#define VISIONCONSTANTS_H

#include <string>

class VisionConstants
{
public:

    static int BALL_EDGE_THRESHOLD;
    static int BALL_ORANGE_TOLERANCE;
    //! Field Constants
    static float GOAL_WIDTH;
    static float DISTANCE_BETWEEN_POSTS;
    static float BALL_WIDTH;
    
    static void loadFromFile(std::string filename);
    
private:
    VisionConstants();  //so noone can make an object of this type
};

#endif // VISIONCONSTANTS_H
