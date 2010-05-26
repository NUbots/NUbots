#ifndef _KF_h_DEFINED
#define _KF_h_DEFINED

#include <math.h>
#include "Tools/Math/Matrix.h"
#include "odometryMotionModel.h"
enum KfUpdateResult
{
    KF_OUTLIER = 0,
    KF_OK = 1,
};

using namespace std;

class KF {
    public:
  	    // Constructor
        KF();
    
        // Functions

        // Update functions
        void timeUpdate(double odometeryForward, double odometeryLeft, double odometeryTurn);
        KfUpdateResult odometeryUpdate(double odom_X, double odom_Y, double odom_Theta, double R_X, double R_Y, double R_Theta);
        KfUpdateResult ballmeas(double Ballmeas, double theta_Ballmeas);
        KfUpdateResult fieldObjectmeas(double distance, double bearing,double objX,double objY, double distanceErrorOffset, double distanceErrorRelative, double bearingError);
	    void linear2MeasurementUpdate(double Y1,double Y2, double SR11, double SR12, double SR22, int index1, int index2);

        // Data retrieval
        double sd(int Xi);
        double variance(int Xi);
        double getState(int stateID);
        Matrix GetBallSR();
        double getDistanceToPosition(double posX, double posY);
        double getBearingToPosition(double posX, double posY);

        // Utility
        void init();
        void Reset();
        bool clipState(int stateIndex, double minValue, double maxValue);


        // Variables

        // Multiple Models - Model state Description.
        double alpha;
        bool isActive;
        bool toBeActivated;

        Matrix updateUncertainties; // Update Uncertainty. (A matrix)
        Matrix stateEstimates; // State estimates. (Xhat Matrix)
        Matrix stateStandardDeviations; // Standard Deviation Matrix. (S Matrix)

        int nStates; // Number of states. (Constant)
        Matrix sqrtOfTestWeightings; // Square root of W (Constant)
        Matrix sqrtOfProcessNoise; // Square root of Process Noise (Q matrix). (Constant)
        Matrix sqrtOfProcessNoiseReset; // Square root of Q when resetting. (Conastant) 
	Matrix sigmaPoints;
	
	
	Matrix srukfCovX;  // Original covariance mat
	Matrix srukfSx;    // Square root of Covariance
	Matrix srukfSq;    // State noise square root covariance
	Matrix srukfSr;    // Measurement noise square root covariance
	
	

        double frameRate; // Constant from init on.
	// Motion Model
	OdometryMotionModel odom_Model;
        // Tuning Values (Constants) -- Values assigned in KF.cpp
        static const float c_Kappa;
        static const float c_ballDecayRate;
        static const float c_threshold2;

        // Ball distance measurement error weightings (Constant) -- Values assigned in KF.cpp
        static const float c_R_ball_theta;
        static const float c_R_ball_range_offset;
        static const float c_R_ball_range_relative;
	
	void measureLocalization(double x,double y,double theta);
	void performFiltering(double odometeryForward, double odometeryLeft, double odometeryTurn);
};

#endif
