#ifndef VISIONCONSTANTS_H
#define VISIONCONSTANTS_H

#include <string>

class VisionConstants
{
public:

    //! Distortion Correction
    static bool DO_RADIAL_CORRECTION;
    static float RADIAL_CORRECTION_COEFFICIENT;
    
    //! Goal filtering constants
    static bool THROWOUT_ON_ABOVE_KIN_HOR_GOALS;
    static bool THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_GOALS;
    static bool THROWOUT_DISTANT_GOALS;
    static float MAX_DISTANCE_METHOD_DISCREPENCY_GOALS; //! The maximum allowed discrepency between the d2p and width distance measures for goal posts
    static float MAX_GOAL_DISTANCE;

    //! Ball filtering constants
    static bool THROWOUT_ON_ABOVE_KIN_HOR_BALL;
    static bool THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BALL;
    static bool THROWOUT_SMALL_BALLS;
    static float MAX_DISTANCE_METHOD_DISCREPENCY_BALL;  //! The maximum allowed discrepency between the d2p and width distance measures for the ball
    static float MIN_BALL_DIAMETER_PIXELS;

    //! Distance calculation options
    static bool D2P_INCLUDE_BODY_PITCH;      //! If this is true then the d2p for the ball is calculated from its base, else from its centre
    static bool BALL_DISTANCE_POSITION_BOTTOM;      //! If this is true then the d2p for the ball is calculated from its base, else from its centre
    enum BallDistanceMethod {
        Width,
        D2P,
        Average,
        Least
    };
    static BallDistanceMethod getBallMethodFromName(std::string name);
    static std::string getBallMethodName(BallDistanceMethod method);
    
    static BallDistanceMethod BALL_DISTANCE_METHOD;
    
    //! Field-object detection constants
    static int BALL_EDGE_THRESHOLD;         //! 
    static int BALL_ORANGE_TOLERANCE;       //! 
    //! Field dimension constants
    static float GOAL_WIDTH;                //! The physical width of the goal posts in cm
    static float DISTANCE_BETWEEN_POSTS;    //! The physical distance between the posts in cm
    static float BALL_WIDTH;                //! The physical width of the ball in cm
    
    static void loadFromFile(std::string filename);
    
private:
    VisionConstants();  //so noone can make an object of this type
};

#endif // VISIONCONSTANTS_H
