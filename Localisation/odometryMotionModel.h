
#ifndef ODOMETRYMOTIONMODEL_H
#define ODOMETRYMOTIONMODEL_H

#include "pose2d.h"
#include <math.h>
#include "probabilityUtils.h"

class OdometryMotionModel{
protected:

		
		/// angle of line of motion
		double dRot1;
		
		/// net angle of rotation
		double dRot2;
		
		/// translational distance moved
		double dTrans;
		
		/// Simple Probability utilities object for sampling
		ProbabilityUtils d_myProb;
public:
	
	/// error in radians per meter
	double d_tErr1;
		
		/// error in metres per meter
	double d_tErr2;
		
		/// error in radians per radian
	double rErr1;
		
		/// error in meters per radian
	double rErr2;
	
		double newPose[3];
		
    

	
	
		/// Constructor
		OdometryMotionModel();
		
		/// Overloaded constructor
		OdometryMotionModel(double terr1, double terr2, double rerr1, double rerr2);
		
		/// function to generate control input u(t) from odometry poses at time t and (t-1)
		void prepareForSampling(Pose2D odoState, Pose2D odoStatePrev);
		void prepareForSampling(Pose2D diffState);
		
		/// function to draw sample from given poseafter application of control input ut
		Pose2D getSample(Pose2D state);
		
		/// pdf of motion model's distribution
		double getProbabilityOfSample(Pose2D newState,Pose2D diffOdom, Pose2D oldState);
		
		/// function to keep angles between -PI and PI
		double LimitTheta(double theta);
		
		double* getNextSigma(Pose2D diffOdom, Pose2D oldSigma);
		
		///Destructor
		~OdometryMotionModel();
		

};

#endif
