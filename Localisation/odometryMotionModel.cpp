/***************************************************************************
File Name: 	odometrymotionmodel.cpp
PURPOSE: 	Odometry motion Model which can be tweaked for any robot
		base by adjusting the standard Deviations for the errors
 ***************************************************************************/
#include "odometryMotionModel.h"
#include "probabilityUtils.h"

#include <iostream>
using namespace std;


/**
*  
* Function Name:	OdometryMotionModel
* Author:		Shashank Bhatia
* Description: 		Constructor
* 			
* Date: 		
* input: 		errors			
* Output:		initialization of Noise parameters 
*/
OdometryMotionModel::OdometryMotionModel(double terr1, double terr2, double rerr1, double rerr2)
{
	d_tErr1 = terr1;
	d_tErr2 = terr2;
	rErr1 = rerr1;
	rErr2 = rerr2;

        muXx     = terr1;
        muXy     = terr2;

        muYy     = rerr1;
        muYx     = rerr2;
	
}



/**
*  
* Function Name:	prepareForSampling
* Author:		Shashank Bhatia
* Description:  	Prepares The Motion model control input based on odometry state
* Date: 		
* input: 		old odometry state and new odometry state			
* Output:		initialization of standard deviations and generating control input ut 
* @param 		odoState 
* @param 		odoStatePrev 
*/
void OdometryMotionModel::prepareForSampling(Pose2D odoState, Pose2D odoStatePrev)
{

// 	odoState.Theta = LimitTheta(odoState.Theta);
// 	odoStatePrev.Theta = LimitTheta(odoStatePrev.Theta);
	
	/*
	dRot1 = atan2((odoState.Y - odoStatePrev.Y), (odoState.X - odoStatePrev.X)) - thetaDeg;

	
	dTrans =fabs(sqrt(pow(odoState.Y - odoStatePrev.Y, 2) + pow(odoState.X - odoStatePrev.X, 2)));*/
}


void OdometryMotionModel::prepareForSampling(Pose2D diffState)
{

// 	odoState.Theta = LimitTheta(odoState.Theta);
// 	odoStatePrev.Theta = LimitTheta(odoStatePrev.Theta);
	double thetaDeg = (diffState.Theta*180)/M_PI;
	dRot1 = atan2((diffState.Y), (diffState.X)) - thetaDeg;
// 	cout<<"\nThetaDeg = "<<thetaDeg<<"  dRot1 deg = "<<dRot1;
	dRot1 = (dRot1 * M_PI) / 180;
	
// 	cout<<"  dRot1 Rad = "<<dRot1;
	dTrans =fabs(sqrt(pow(diffState.Y, 2) + pow(diffState.X, 2)));

	dRot2 = diffState.Theta - dRot1;
}



/**
*  
* Function Name:	getSample
* Author:		Shashank Bhatia
* Description:  	Draws Sample from Model after applying input ut
* Date: 		
* input: 		Current pose			
* Output:		generating a sample x(t+1) from x(t) and u(t)
* @param 		state
* @return		pose1
*/
Pose2D OdometryMotionModel::getSample(Pose2D state)
{

// 	state.Theta = LimitTheta(state.Theta);
	double dHatRot1 = dRot1 - d_myProb.randomGaussian(0.00,(rErr1 * (dRot1) + d_tErr1 * dTrans));
	double dHatTrans = dTrans - d_myProb.randomGaussian(0.00,(d_tErr2 * dTrans + rErr2 * ((dRot1) + (dRot2))));
	double dHatRot2 = dRot2 - d_myProb.randomGaussian(0.00,(rErr1 * (dRot2) + d_tErr1 * dTrans));

	double x = state.X + dHatTrans * cos(state.Theta + dHatRot1);
	double y = state.Y + dHatTrans * sin(state.Theta + dHatRot1);
	double theta =  state.Theta + dHatRot1 + dHatRot2;
// 	theta = LimitTheta(theta);// limiter

	Pose2D pose1(x, y, theta);
	return pose1;
}



/**
*  
* Function Name:	getProbabilityOfSample
* Author:		Shashank Bhatia
* Description:  	Calculating probability of new sample under conditions p(x(t+1)|x(t),u(t))
			where u(t) is calculated by oldOdom and newOdom
* Date: 		
* input: 		old state, odometry data treated as control input, new state			
* Output:		calculating p(x(t+1)|x(t),u(t))
* @param newState 
* @param newOdom 
* @param oldOdom 
* @param oldState 
* @return probability of sample
*/
double OdometryMotionModel::getProbabilityOfSample(Pose2D newState,Pose2D diffOdom, Pose2D oldState)
{


	prepareForSampling(diffOdom);
	
	double dHatRot1,dHatRot2,dHatTrans;
	dHatRot1 = dHatRot2 = dHatTrans = 0.000001;
	dHatRot1 = atan2(newState.Y - oldState.Y, newState.X - oldState.X) - oldState.Theta;
	
	dHatTrans = sqrt(pow(newState.X - oldState.X, 2) + pow(newState.Y - oldState.Y, 2));
	dHatRot2 = newState.Theta - oldState.Theta - dHatRot1;
	
	double p1_max = d_myProb.ProbabilityOfValInVariance(0.00, (rErr1 *(dHatRot1) + d_tErr1 * dHatTrans));
	double p2_max = d_myProb.ProbabilityOfValInVariance(0.00,
			(d_tErr2 * dHatTrans +
					rErr2 * (((dHatRot1) + ((dHatRot2))))));
	double p3_max = d_myProb.ProbabilityOfValInVariance(0.00, (rErr1 * (dHatRot2) + d_tErr1 * dHatTrans));
	
/*	double p1 = d_myProb.ProbabilityOfValInVariance((dRot1 - dHatRot1), (rErr1 *fabs(dHatRot1) + d_tErr1 * dHatTrans))/p1_max;
	
	double p2 = d_myProb.ProbabilityOfValInVariance((dTrans - dHatTrans),(d_tErr2 * dHatTrans + rErr2 * fabs(((dHatRot1) + ((dHatRot2))))))/p2_max;
	
	double p3 = d_myProb.ProbabilityOfValInVariance((dRot2 - dHatRot2), (rErr1 * fabs(dHatRot2) + d_tErr1 * dHatTrans))/p3_max*/;
	
	
	double p1 = d_myProb.ProbabilityOfValInVariance( 0.00 , (rErr1 *fabs(dRot1) + d_tErr1 * dTrans));
	
	double p2 = d_myProb.ProbabilityOfValInVariance( 0.00 , (d_tErr2 * dTrans + rErr2 * fabs(((dRot1) + ((dHatRot2))))));
	
	double p3 = d_myProb.ProbabilityOfValInVariance( 0.00 , (rErr1 * fabs(dRot2) + d_tErr1 * dTrans));
	
	
	if (p1 == 0 || p2 == 0 || p3 == 0 || !double(p1) || !double(p2) || !double(p3))
	{
		cout<<"Problem with weights: "<< endl;
		cout<<" values of p1,p2,p3 respectively :"<<p1<<","<<p2<<","<<p3;
		return 0;
	}
	if(!p1*p2*p3)
	{
		cout<<"Problem with weights: "<< endl;
		cout<<" values of p1,p2,p3 respectively :"<<p1<<","<<p2<<","<<p3;
		return 0;	  
	}
	return (p1 * p2 * p3);


}



/**
 * 
 * @param diffOdom 
 * @param oldSigma 
 * @return 
 */
double* OdometryMotionModel::getNextSigma(Pose2D diffOdom, Pose2D oldSigma)
{
	
//	----------- works 
	double t = sqrt ( (diffOdom.X*diffOdom.X) + (diffOdom.Y*diffOdom.Y) );
	

	
	double muTheta = rErr1 * t + rErr2 * fabs(diffOdom.Theta);
	
// 	double varX     = 0.005;
// 	double varY     = 0.02;
// 	double varTheta = muTheta/10;
	
	
	double varX     = d_tErr1 * t   + rErr1 * fabs(diffOdom.Theta);
        double varY     =    muYy * fabs(diffOdom.Theta);
	double varTheta = rErr1 * t + rErr2 * fabs(diffOdom.Theta);
	      
 	
 	double tempX = d_myProb.randomGaussian( diffOdom.X*muXx + diffOdom.Y*muXy , diffOdom.X*varX);
        double tempY = d_myProb.randomGaussian( diffOdom.Y*muYy + diffOdom.X*muYx , diffOdom.Y*varY );
	      
	double tempTheta = d_myProb.randomGaussian( muTheta , varTheta); 
	       
	
	newPose[0] = oldSigma.X + diffOdom.X*cos(oldSigma.Theta + (diffOdom.Theta/2)) - diffOdom.Y*sin(oldSigma.Theta + (diffOdom.Theta/2)) + tempX;

	newPose[1] = oldSigma.Y + diffOdom.X*sin(oldSigma.Theta + (diffOdom.Theta/2)) + diffOdom.Y*cos(oldSigma.Theta + (diffOdom.Theta/2)) + tempY;
	
	newPose[2] = oldSigma.Theta + diffOdom.Theta ;//+ tempTheta

	return newPose;
	//----------------------------------------*/
// 	double muD_t = 0.1;
// 	double muD_r = 0.01;
// 	double muT_t = 0.1;
// 	double muT_r = 0.01;
// 	double muC_t = 0.1;
// 	double muC_r = 0.01;
// 	double varC_t = 0.05;
// 	double varD_t = 0.05;
// 	double varT_t = 0.05;
// 	double varC_r = 0.05;
// 	double varD_r = 0.05;
// 	double varT_r = 0.05;
// 	
// 	double t = sqrt ( (diffOdom.X*diffOdom.X) + (diffOdom.Y*diffOdom.Y) ) ;  // lateral movement
// 	double r = diffOdom.Theta;                                               // Horizontal movement
// 
// 	double C = d_myProb.randomGaussian( (t*muC_t + r*muC_r) ,  (t*t*varC_t + r*r*varC_r) );
// 	double D = d_myProb.randomGaussian( (t*muD_t + r*muD_r) ,  (t*t*varD_t + r*r*varD_r) );
// 	double E = d_myProb.randomGaussian( (t*muT_t + r*muT_r) ,  (t*t*varT_t + r*r*varT_r) );
// 	
// 	newPose[0] = oldSigma.X + D * cos(oldSigma.Theta + (E/2)) + C * cos(oldSigma.Theta + ((E+M_PI)/2)) ;
// 	
// 	newPose[1] = oldSigma.Y + D * sin(oldSigma.Theta + (E/2)) + C * sin(oldSigma.Theta + ((E+M_PI)/2)) ;
// 		
// 	newPose[2] = oldSigma.Theta + E;
	
	
	
	
// 	// ---------------------- Original
// 	double varX = d_tErr1 * sqrt ( (diffOdom.X*diffOdom.X) + (diffOdom.Y*diffOdom.Y) ) + rErr1 * fabs(diffOdom.Theta);
// 	double varY = d_tErr2 * sqrt ( (diffOdom.X*diffOdom.X) + (diffOdom.Y*diffOdom.Y) ) + rErr2 * fabs(diffOdom.Theta);
// 	double varTheta = rErr1 *  sqrt ( (diffOdom.X*diffOdom.X) + (diffOdom.Y*diffOdom.Y) )  + rErr2 * fabs(diffOdom.Theta);
// 	      
// 	double tempX = d_myProb.randomGaussian( 0.0 , varX);
// 	double tempY = d_myProb.randomGaussian( 0.0 , varY);
// 	      
// 	double tempTheta = d_myProb.randomGaussian( 0.0 , varTheta); 
// 	       
// 	newPose[0] = oldSigma.X + diffOdom.X*cos(oldSigma.Theta + (diffOdom.Theta/2)) - diffOdom.Y*sin(oldSigma.Theta + (diffOdom.Theta/2)) + tempX;
// 
// 	newPose[1] = oldSigma.Y + diffOdom.X*sin(oldSigma.Theta + (diffOdom.Theta/2)) + diffOdom.Y*cos(oldSigma.Theta + (diffOdom.Theta/2)) + tempY;
// 	
// 	newPose[2] = oldSigma.Theta + diffOdom.Theta; /*+ tempTheta;*/
// 
//  	return newPose;
	
	
	
	
	
	
	
	
	      
// 	      double varX = 10*d_tErr1 * sqrt ( (diffOdom.X*diffOdom.X) + (diffOdom.Y*diffOdom.Y) ) + d_tErr2 * fabs(diffOdom.Theta);
// 	      double varY = rErr1 * sqrt ( (diffOdom.X*diffOdom.X) + (diffOdom.Y*diffOdom.Y) ) + d_tErr2 * fabs(diffOdom.Theta);
// 	      double varTheta = /*rErr1 *  sqrt ( (diffOdom.X*diffOdom.X) + (diffOdom.Y*diffOdom.Y) )  +*/ rErr2 * fabs(diffOdom.Theta);
// 	      
// 	      double tempX = d_myProb.randomGaussian( 0.0 , varX);
// 	      double tempY = d_myProb.randomGaussian( 0.0 , varY);
// 	      
// 	      double tempTheta = d_myProb.randomGaussian( 0.0 , varTheta); 
// 	       
// 	      newPose[0] = oldSigma.X + diffOdom.X*cos(oldSigma.Theta + (diffOdom.Theta/2)) - diffOdom.Y*sin(oldSigma.Theta + (diffOdom.Theta/2)) + tempX;
// 
// 	      newPose[1] = oldSigma.Y + diffOdom.X*sin(oldSigma.Theta + (diffOdom.Theta/2)) + diffOdom.Y*cos(oldSigma.Theta + (diffOdom.Theta/2)) + tempY;
// 	
// 	      newPose[2] = oldSigma.Theta + diffOdom.Theta + tempTheta;
// 
// 	      return newPose;
	
}







/**
*  
* Function Name:	LimitTheta
* Author:		Shashank Bhatia
* Description:  	Keeps Theta between -PI and PI
* Date: 		
* input: 		angle value
* Output:		Normalized Angle
* 
* @param theta 
* @return theta
*/
double OdometryMotionModel::LimitTheta(double theta)
{
	while (theta >= M_PI)
	{
		theta -= (M_PI * 2);
	}
	while (theta <= -M_PI) 
	{
		theta += (M_PI * 2);
	}

	if( (theta >= -M_PI) &&  (theta <= M_PI))
		return theta;
	else 
		return 0;
}


/**
*  
* Function Name:	OdometryMotionModel
* Author:		Shashank Bhatia
* Description: 		Destructor
* 			
* Date: 		
* input: 		NA
* Output:		NA
*/
OdometryMotionModel::~OdometryMotionModel()
{

}
