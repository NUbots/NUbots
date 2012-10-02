#ifndef VISIONCONSTANTS_H
#define VISIONCONSTANTS_H

#include <string>
#include "Tools/Optimisation/Parameter.h"

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
    static bool DO_RADIAL_CORRECTION;           //! Whether to perform radial distortion correction.
    static float RADIAL_CORRECTION_COEFFICIENT; //! The radial distortion correction coefficient.
    
    //! Goal filtering constants
    static bool THROWOUT_ON_ABOVE_KIN_HOR_GOALS;    //! Whether to throw out goals whose base is above the kinematics horizon.
    static bool THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_GOALS;  //! Whether to throw out goals when the distance methods disagree.
    static float MAX_DISTANCE_METHOD_DISCREPENCY_GOALS;         //! The maximum allowed discrepency between the d2p and width distance measures for goal posts
    static bool THROWOUT_DISTANT_GOALS; //! Whether to throw out goals too far away.
    static float MAX_GOAL_DISTANCE;     //! How far away a goal has to been to be ignored.
    static bool THROWOUT_INSIGNIFICANT_GOALS;           //! Whether to throw out goals with too few transitions.
    static int MIN_TRANSITIONS_FOR_SIGNIFICANCE_GOALS;  //! The minimum number of transitions to keep a goal.
    static bool THROWOUT_NARROW_GOALS;  //! Whether to throw out goals that are too narrow.
    static int MIN_GOAL_WIDTH;          //! The minimum width of a goal.
    static bool THROWOUT_SHORT_GOALS;  //! Whether to throw out goals that are too short.
    static int MIN_GOAL_HEIGHT;          //! The minimum height of a goal.
    static float GOAL_HEIGHT_TO_WIDTH_RATIO_LOW;
    static float GOAL_HEIGHT_TO_WIDTH_RATIO_HIGH;

    //! Beacon filtering constants
    static bool THROWOUT_ON_ABOVE_KIN_HOR_BEACONS;  //! Whether to throw out beacons whose base is above the kinematics horizon.
    static bool THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BEACONS;    //! Whether to throw out beacons when the distance methods disagree.
    static float MAX_DISTANCE_METHOD_DISCREPENCY_BEACONS;           //! The maximum allowed discrepency between the d2p and width distance measures for beacons
    static bool THROWOUT_DISTANT_BEACONS;   //! Whether to throw out beacons too far away.
    static float MAX_BEACON_DISTANCE;       //! How far away a beacon has to been to be ignored.
    static bool THROWOUT_INSIGNIFICANT_BEACONS; //! Whether to throw out beacons with too few transitions.
    static int MIN_TRANSITIONS_FOR_SIGNIFICANCE_BEACONS;    //! The minimum number of transitions to keep a beacon.

    //! Ball filtering constants
    static bool THROWOUT_ON_ABOVE_KIN_HOR_BALL; //! Whether to throw out a ball whose base is above the kinematics horizon.
    static bool THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BALL;   //! Whether to throw out a ball when the distance methods disagree.
    static float MAX_DISTANCE_METHOD_DISCREPENCY_BALL;          //! The maximum allowed discrepency between the d2p and width distance measures for the ball
    static bool THROWOUT_SMALL_BALLS;       //! Whether to throw out balls that are too small.
    static float MIN_BALL_DIAMETER_PIXELS;  //! Minimum size for a ball.
    static bool THROWOUT_INSIGNIFICANT_BALLS;           //! Whether to throw out ball with too few transitions.
    static int MIN_TRANSITIONS_FOR_SIGNIFICANCE_BALL;   //! The minimum number of transitions to keep a ball.
    static bool THROWOUT_DISTANT_BALLS; //! Whether to throw out balls that are too far away.
    static float MAX_BALL_DISTANCE;     //! The maximum distance for a ball.

    //! Distance calculation options
    static bool D2P_INCLUDE_BODY_PITCH;      //! If this is true then the d2p for the ball is calculated from its base, else from its centre
    static float D2P_ANGLE_CORRECTION;      //! If this is true then the d2p for the ball is calculated from its base, else from its centre
    static bool BALL_DISTANCE_POSITION_BOTTOM;      //! If this is true then the d2p for the ball is calculated from its base, else from its centre

    //! Distance method options
    static DistanceMethod BALL_DISTANCE_METHOD;     //! The preferred method for calculating the distance to the ball
    static DistanceMethod GOAL_DISTANCE_METHOD;     //! The preferred method for calculating the distance to the goals
    static DistanceMethod BEACON_DISTANCE_METHOD;   //! The preferred method for calculating the distance to the beacons
    
    //! Field-object detection constants
    static int BALL_EDGE_THRESHOLD;         //! Dave?
    static int BALL_ORANGE_TOLERANCE;       //! Dave?
    static float BALL_MIN_PERCENT_ORANGE;   //! Dave?
    static float GOAL_MIN_PERCENT_YELLOW;   //! Dave?
    static float GOAL_MIN_PERCENT_BLUE;     //! Dave?
    static float BEACON_MIN_PERCENT_YELLOW; //! Dave?
    static float BEACON_MIN_PERCENT_BLUE;   //! Dave?
    static int MIN_GOAL_SEPARATION;

    //! Obstacle detection constants
    static int MIN_DISTANCE_FROM_HORIZON;   //! Dave?
    static int MIN_CONSECUTIVE_POINTS;      //! Dave?

    //! Field dimension constants
    static float GOAL_WIDTH;                //! The physical width of the goal posts in cm
    static float DISTANCE_BETWEEN_POSTS;    //! The physical distance between the posts in cm
    static float BALL_WIDTH;                //! The physical width of the ball in cm
    static float BEACON_WIDTH;              //! The physical width of the beacons in cm
    
    //! ScanLine options
    static unsigned int HORIZONTAL_SCANLINE_SPACING;    //! The spacing between horizontal scans.
    static unsigned int VERTICAL_SCANLINE_SPACING;      //! The spacing between vertical scans.
    static unsigned int GREEN_HORIZON_SCAN_SPACING;     //! The spacing between scans used to locate the GH.
    static unsigned int GREEN_HORIZON_MIN_GREEN_PIXELS; //! Dave?
    static float GREEN_HORIZON_LOWER_THRESHOLD_MULT;    //! Dave?
    static float GREEN_HORIZON_UPPER_THRESHOLD_MULT;    //! Dave?

    //! Split and Merge constants
    //maximum field objects rules
    static unsigned int SAM_MAX_POINTS; //500
    static unsigned int SAM_MAX_LINES; //15
    //splitting rules
    static float SAM_SPLIT_DISTANCE; //1.0
    static unsigned int SAM_MIN_POINTS_OVER; //2
    static unsigned int SAM_MIN_POINTS_TO_LINE; //3
    //Noise splitting rules
    static unsigned int SAM_SPLIT_NOISE_ITERATIONS; //1
    //merging rules
    static float SAM_MAX_ANGLE_DIFF_TO_MERGE; //
    static float SAM_MAX_DISTANCE_TO_MERGE; //
    //Line keeping rules
    static unsigned int SAM_MIN_POINTS_TO_LINE_FINAL; //5
    static float SAM_MIN_LINE_R2_FIT; //0.90
    static float SAM_MAX_LINE_MSD; //50 set at constructor
    //clearing options
    static bool SAM_CLEAR_SMALL;
    static bool SAM_CLEAR_DIRTY;


    // static methods
    static DistanceMethod getDistanceMethodFromName(std::string name);  //! Converts a string to a distance method.
    static std::string getDistanceMethodName(DistanceMethod method);    //! Converts a distance method to a string name.

    static void loadFromFile(std::string filename); //! Loads the constants from a file
    static void print(ostream& out);

    static void setFlags(bool val=true);

    static vector<Parameter> getAllOptimisable();
    static vector<Parameter> getBallParams();
    static vector<Parameter> getGoalBeaconParams();
    static vector<Parameter> getObstacleParams();
    static vector<Parameter> getLineParams();
    static vector<Parameter> getGeneralParams();


    static bool setAllOptimisable(const vector<float>& params);
    static bool setBallParams(const vector<float>& params);
    static bool setGoalBeaconParams(const vector<float>& params);
    static bool setObstacleParams(const vector<float>& params);
    static bool setLineParams(const vector<float>& params);
    static bool setGeneralParams(const vector<float>& params);
    
private:
    VisionConstants();  //so noone can make an object of this type
};

#endif // VISIONCONSTANTS_H
