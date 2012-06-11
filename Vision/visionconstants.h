#ifndef VISIONCONSTANTS_H
#define VISIONCONSTANTS_H

#include <string>

class VisionConstants
{
public:

    enum DistanceMethod {
        Width,
        D2P,
        Average,
        Least
    };

    //! Distortion Correction
    static bool DO_RADIAL_CORRECTION;
    static float RADIAL_CORRECTION_COEFFICIENT;
    
    //! Goal filtering constants
    static bool THROWOUT_ON_ABOVE_KIN_HOR_GOALS;
    static bool THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_GOALS;
    static float MAX_DISTANCE_METHOD_DISCREPENCY_GOALS; //! The maximum allowed discrepency between the d2p and width distance measures for goal posts
    static bool THROWOUT_DISTANT_GOALS;
    static float MAX_GOAL_DISTANCE;
    static bool THROWOUT_INSIGNIFICANT_GOALS;
    static int MIN_TRANSITIONS_FOR_SIGNIFICANCE_GOALS;

    //! Beacon filtering constants
    static bool THROWOUT_ON_ABOVE_KIN_HOR_BEACONS;
    static bool THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BEACONS;
    static float MAX_DISTANCE_METHOD_DISCREPENCY_BEACONS; //! The maximum allowed discrepency between the d2p and width distance measures for beacons
    static bool THROWOUT_DISTANT_BEACONS;
    static float MAX_BEACON_DISTANCE;
    static bool THROWOUT_INSIGNIFICANT_BEACONS;
    static int MIN_TRANSITIONS_FOR_SIGNIFICANCE_BEACONS;

    //! Ball filtering constants
    static bool THROWOUT_ON_ABOVE_KIN_HOR_BALL;
    static bool THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BALL;
    static float MAX_DISTANCE_METHOD_DISCREPENCY_BALL;  //! The maximum allowed discrepency between the d2p and width distance measures for the ball
    static bool THROWOUT_SMALL_BALLS;
    static float MIN_BALL_DIAMETER_PIXELS;
    static bool THROWOUT_INSIGNIFICANT_BALLS;
    static int MIN_TRANSITIONS_FOR_SIGNIFICANCE_BALL;
    static bool THROWOUT_DISTANT_BALLS;
    static float MAX_BALL_DISTANCE;

    //! Distance calculation options
    static bool D2P_INCLUDE_BODY_PITCH;      //! If this is true then the d2p for the ball is calculated from its base, else from its centre
    static float D2P_ANGLE_CORRECTION;      //! If this is true then the d2p for the ball is calculated from its base, else from its centre
    static bool BALL_DISTANCE_POSITION_BOTTOM;      //! If this is true then the d2p for the ball is calculated from its base, else from its centre

    //! Distance method options
    static DistanceMethod BALL_DISTANCE_METHOD;
    static DistanceMethod GOAL_DISTANCE_METHOD;
    static DistanceMethod BEACON_DISTANCE_METHOD;
    
    //! Field-object detection constants
    static int BALL_EDGE_THRESHOLD;         //! Dave?
    static int BALL_ORANGE_TOLERANCE;       //! Dave?
    static float BALL_MIN_PERCENT_ORANGE;   //! Dave?
    static float GOAL_MIN_PERCENT_YELLOW;   //! Dave?
    static float GOAL_MIN_PERCENT_BLUE;     //! Dave?
    static float BEACON_MIN_PERCENT_YELLOW; //! Dave?
    static float BEACON_MIN_PERCENT_BLUE;   //! Dave?

    //! Obstacle detection constants
    static int MIN_DISTANCE_FROM_HORIZON;   //! Dave?
    static int MIN_CONSECUTIVE_POINTS;      //! Dave?

    //! Field dimension constants
    static float GOAL_WIDTH;                //! The physical width of the goal posts in cm
    static float DISTANCE_BETWEEN_POSTS;    //! The physical distance between the posts in cm
    static float BALL_WIDTH;                //! The physical width of the ball in cm
    static float BEACON_WIDTH;              //! The physical width of the beacons in cm
    
    //! ScanLine options
    static unsigned int HORIZONTAL_SCANLINE_SPACING;
    static unsigned int VERTICAL_SCANLINE_SPACING;
    static unsigned int GREEN_HORIZON_SCAN_SPACING;
    static unsigned int GREEN_HORIZON_MIN_GREEN_PIXELS;
    static float GREEN_HORIZON_LOWER_THRESHOLD_MULT;
    static float GREEN_HORIZON_UPPER_THRESHOLD_MULT;

    // static methods
    static DistanceMethod getDistanceMethodFromName(std::string name);
    static std::string getDistanceMethodName(DistanceMethod method);

    static void loadFromFile(std::string filename);
    
private:
    VisionConstants();  //so noone can make an object of this type
};

#endif // VISIONCONSTANTS_H
