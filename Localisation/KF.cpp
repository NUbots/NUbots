/**
7 states calculation
Xhat[0][0]=x
Xhat[1][0]=y
Xhat[2][0]=orientation
Xhat[3][0]=ballX
Xhat[4][0]=ballY
Xhat[5][0]=ballVeocityX
Xhat[6][0]=ballVeocityY
**/

#include "KF.h"
#include "Tools/Math/Matrix.h"
#include "Tools/Math/General.h"
#include <iostream>
#include "debug.h"
#include "odometryMotionModel.h"
#include "pose2d.h"
#include <sstream>
#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/GameInformation/GameInformation.h"

#define VARIANCE_REST_THRESHOLD 800

using namespace mathGeneral;

// Tuning Values (Constants)
const float KF::c_Kappa = 1.0f; // weight used in w matrix. (Constant)
const float KF::c_ballDecayRate = 0.985f; // Ball weighting 0.985 => speed halves every 1.5 seconds. (Constant)
const float KF::c_threshold2 = 15.0f; // Threshold for outlier rejection. Magic Number. (Constant)

const float KF::c_outlierLikelyhood = 1e-3;

// Ball distance measurement error weightings (Constant)
const float KF::c_R_ball_theta = 0.0001f;
const float KF::c_R_ball_range_offset = 25.0f; // (5cm)^2
const float KF::c_R_ball_range_relative = 0.0025f; // 5% of range added.


double muXx     = 0.07;
double muXy     = 0.00005;

double muYy     = 0.00005;
double muYx     = 0.000005;
// odom_Model(0.0005,0.0001,0.000005,0.0008)
KF::KF():odom_Model(0.07,0.00005,0.00005,0.000005)
{
    /**************************Initialization**************************/
	/*This is where values can be adjusted*/
  m_alpha = 1.0; // Accuracy of model (0.0 -> 1.0)
  m_isActive = false; // Model currently in use.
  m_toBeActivated = false; // Model to be in use.

// Update Uncertainty
  updateUncertainties = Matrix(7, 7, true);
  updateUncertainties[5][5] = c_ballDecayRate; // Ball velocity x
  updateUncertainties[6][6] = c_ballDecayRate; // Ball velocity y
  updateUncertainties[3][5] = 1.0f/30.0f; // [ballX][ballXvelocity]
  updateUncertainties[4][6] = 1.0f/30.0f; // [ballY][ballYvelocity]

  init();									//Initialisation of Xhat and S

// Process Noise - Matrix Square Root of Q
  sqrtOfProcessNoise = Matrix(7,7,true);
  sqrtOfProcessNoise[0][0] = 0.2; // Robot X coord.
  sqrtOfProcessNoise[1][1] = 0.2; // Robot Y coord.
  sqrtOfProcessNoise[2][2] = 0.005; // Robot Theta. 0.00001
  sqrtOfProcessNoise[3][3] = 5.0; // Ball X.
  sqrtOfProcessNoise[4][4] = 5.0; // Ball Y.
  sqrtOfProcessNoise[5][5] = 5.6569; // Ball X Velocity.
  sqrtOfProcessNoise[6][6] = 5.6569; // Ball Y Velocity.

//  sqrtOfProcessNoise = Matrix(7,7,true);
//  sqrtOfProcessNoise[0][0] = 6.0; // Robot X coord.
//  sqrtOfProcessNoise[1][1] = 6.0; // Robot Y coord.
//  sqrtOfProcessNoise[2][2] = 0.15; // Robot Theta. 0.00001
//  sqrtOfProcessNoise[3][3] = 15.0; // Ball X.
//  sqrtOfProcessNoise[4][4] = 15.0; // Ball Y.
//  sqrtOfProcessNoise[5][5] = 30.0; // Ball X Velocity.
//  sqrtOfProcessNoise[6][6] = 30.0; // Ball Y Velocity.


	//RHM 7/7/08: Extra matrix for resetting
  // Process noise for used for reset.
//  sqrtOfProcessNoiseReset = Matrix(7,7,false);
//  sqrtOfProcessNoiseReset[0][0] = 50.0; // extra 50cm sd when kidnapped?
//  sqrtOfProcessNoiseReset[1][1] = 50.0; // extra 50cm sd when kidnapped?
  //sqrtOfProcessNoiseReset[2][2] = 0.25; // extra 15deg shift when kidnapped? 0.25
//  sqrtOfProcessNoiseReset[2][2] = 0.5; // extra 15deg shift when kidnapped? 0.25
//  sqrtOfProcessNoiseReset[3][3] = 20.0; // ball itself shouldn't have moved much?
//  sqrtOfProcessNoiseReset[4][4] = 20.0; // just being cautious
	
  sqrtOfProcessNoiseReset = Matrix(7,7,false);
  sqrtOfProcessNoiseReset[0][0] = 150.0; // extra 50cm sd when kidnapped?
  sqrtOfProcessNoiseReset[1][1] = 100.0; // extra 50cm sd when kidnapped?
  //sqrtOfProcessNoiseReset[2][2] = 0.25; // extra 15deg shift when kidnapped? 0.25
  sqrtOfProcessNoiseReset[2][2] = 2.0; // extra 15deg shift when kidnapped? 0.25
  sqrtOfProcessNoiseReset[3][3] = 20.0; // ball itself shouldn't have moved much?
  sqrtOfProcessNoiseReset[4][4] = 20.0; // just being cautious


  nStates = stateEstimates.getm(); // number of states.


  const unsigned int numSigmaPoints = 2*nStates+1;

  // Create square root of W matrix
  sqrtOfTestWeightings = Matrix(1,numSigmaPoints,false);
  sqrtOfTestWeightings[0][0] = sqrt(c_Kappa/(nStates+c_Kappa));
  double outerWeighting = sqrt(1.0/(2*(nStates+c_Kappa)));
  for(int i=1; i <= 2*nStates; i++)
  {
    sqrtOfTestWeightings[0][i] = (outerWeighting);
  }
  
  return;
}


void KF::init(){
  // Initial state estimates
    stateEstimates= Matrix(7,1,true);
    stateEstimates[2][0]=3+3.1416/2.0; // 0 for all values but robot bearing = 3.
  // S = Standard deviation matrix.
  // Initial Uncertainty
    stateStandardDeviations = Matrix(7,7,false);
    stateStandardDeviations[0][0] = 150; // 100 cm
    stateStandardDeviations[1][1] = 100; // 150 cm
    stateStandardDeviations[2][2] = 2;   // 2 radians
    stateStandardDeviations[3][3] = 150; // 100 cm
    stateStandardDeviations[4][4] = 100; // 150 cm
    stateStandardDeviations[5][5] = 10;  // 10 cm/s
    stateStandardDeviations[6][6] = 10;  // 10 cm/s
    
}

unsigned int KF::id() const
{
    return m_id;
}
unsigned int KF::parentId() const
{
    return m_parentId;
}

bool KF::active() const
{
    return m_isActive;
}

void KF::setActive(bool active)
{
    m_isActive = active;
}

double KF::alpha() const
{
    return m_alpha;
}

void KF::setAlpha(double new_alpha)
{
    m_alpha = new_alpha;
    return;
}

unsigned int KF::GenerateId()
{
    static unsigned int s_next_id = 0;
    return s_next_id++;
}

unsigned int KF::spawnFromModel(const KF& parent)
{
    m_parentId = parent.id();
    m_id = KF::GenerateId();
    //! @TODO: add copy model lines. (not sure what is required yet).
    return id();
}

void KF::performFiltering(double odom_X, double odom_Y, double odom_Theta)
{
	
	
// 	std::cout<<"Cov Matrix : " <<srukfCovX<<std::endl;
	// Step 1 : Calculate sqaure root of Covariance
	
	//-----------------------------------------------------------------------------------------------
	
	
	
	
	// Step 2 : Calculate new sigma points based on previous covariance
	double sigmaAngleMax = 2.5;               // required for normalising Angle
	
        Matrix sigmaPoints = CalculateSigmaPoints();
        const unsigned int numSigmaPoints = sigmaPoints.getn();
	//-----------------------------------------------------------------------------------------------
	
	
	// Step 3: Update the state estimate - Pass all sigma points through motion model
	
	Pose2D oldPose, diffOdom;
	double *newPose;

        for (int i = 0 ; i < numSigmaPoints; i++)
	{
		oldPose.X = sigmaPoints[0][i];
		oldPose.Y = sigmaPoints[1][i];
		oldPose.Theta = sigmaPoints[2][i];
   
		diffOdom.X = odom_X;
		diffOdom.Y = odom_Y;
		diffOdom.Theta = odom_Theta;
   
		newPose = odom_Model.getNextSigma(diffOdom,oldPose);

		sigmaPoints[0][i] = *newPose;
		sigmaPoints[1][i] = *(newPose+1);
		sigmaPoints[2][i] = *(newPose+2); 
	}
	//-----------------------------------------------------------------------------------------------
	
	
	
	
	// Step 4: Calculate new state based on propagated sigma points and the weightings of the sigmaPoints
	Matrix newStateEstimates(stateEstimates.getm(),stateEstimates.getn(), false);
	
        for(int i=0; i < numSigmaPoints; i++)
	{
		// Eqn 20
		newStateEstimates = newStateEstimates + sqrtOfTestWeightings[0][i]*sqrtOfTestWeightings[0][i]*sigmaPoints.getCol(i);
	}
	//-----------------------------------------------------------------------------------------------
	
	// Step 5: Calculate measurement error and then find new srukfSx
	Matrix newSrukfSx(stateStandardDeviations.getm(),stateStandardDeviations.getn(), false);
	Matrix Mx(sigmaPoints.getm(),sigmaPoints.getn(), false);
  	
        for(int i=0; i < numSigmaPoints; i++)
	{
		// Eqn 21 part
		Mx.setCol(i, sqrtOfTestWeightings[0][i] * (sigmaPoints.getCol(i) - newStateEstimates));      // Error matrix	
 	}
	
//        const float muXx = 0.5;
//        const float muXy = 0.0;     // don't know why but it looks like the covariance's should be 0
//        const float muYy = 0.5;
//        const float muYx = 0.0;
//        const float muTt = 0.5;
//        const float muXt = 0.003;
//        const float muYt = 0.003;
//        const float varX = muXx*fabs(diffOdom.X) + muYx*fabs(diffOdom.Y);
//        const float varY = muYy*fabs(diffOdom.Y) + muXy*fabs(diffOdom.X);
//        const float varTheta = muTt*fabs(diffOdom.Theta) + muXt*fabs(diffOdom.X) + muYt*fabs(diffOdom.Y);
//        Matrix motionNoise(7,7,false);
//        motionNoise[KF::selfX][KF::selfX] = sqrt(varX);
//        motionNoise[KF::selfY][KF::selfY] = sqrt(varY);
//        motionNoise[KF::selfTheta][KF::selfTheta] = sqrt(varTheta);

	// Eqn 23 originally it should be a Mx*Mz

        stateStandardDeviations = HT(Mx);
//        stateStandardDeviations = HT(horzcat(Mx,motionNoise));
        stateEstimates = newStateEstimates;

//        std::cout << "Odometry: (" << odom_X << "," << odom_Y << "," << odom_Theta << ")" << std::endl;
//        std::cout << "Odometry Variance: (" << varX << "," << varY << "," << varTheta << ")" << std::endl;
//        std::cout << HT(Mx) << std::endl;
//        std::cout << stateStandardDeviations << std::endl;
/*	return KF_OK;*/
}



void KF::timeUpdate(double deltaTime){
    double deltaTimeSeconds = deltaTime / 1000;
    //-----------------------Update for ball velocity
    stateEstimates[3][0] = stateEstimates[3][0] + stateEstimates[5][0]*deltaTimeSeconds; // Update ball x position by ball x velocity.
    stateEstimates[4][0] = stateEstimates[4][0] + stateEstimates[6][0]*deltaTimeSeconds; // Update ball y position by ball y velocity.
    stateEstimates[5][0] = c_ballDecayRate*stateEstimates[5][0]; // Reduce ball x velocity assuming deceleration
    stateEstimates[6][0] = c_ballDecayRate*stateEstimates[6][0]; // Reduce ball y velocity assuming deceleration

    updateUncertainties[3][5] = deltaTimeSeconds; // [ballX][ballXvelocity]
    updateUncertainties[4][6] = deltaTimeSeconds; // [ballY][ballYvelocity]

    // Householder transform. Unscented KF algorithm. Takes a while.
    stateStandardDeviations=HT(horzcat(updateUncertainties*stateStandardDeviations, sqrtOfProcessNoise));
    stateEstimates[2][0] = normaliseAngle(stateEstimates[2][0]); // unwrap the robots angle to keep within -pi < theta < pi.
    return;
}


void KF::timeUpdate(float odom_x, float odom_y, float odom_theta, double deltaTime)
{
    double deltaTimeSeconds = deltaTime / 1000;
    //-----------------------Update for ball velocity
    stateEstimates[KF::ballX][0] = stateEstimates[3][0] + stateEstimates[5][0]*deltaTimeSeconds; // Update ball x position by ball x velocity.
    stateEstimates[KF::ballY][0] = stateEstimates[4][0] + stateEstimates[6][0]*deltaTimeSeconds; // Update ball y position by ball y velocity.
    stateEstimates[KF::ballXVelocity][0] = c_ballDecayRate*stateEstimates[5][0]; // Reduce ball x velocity assuming deceleration
    stateEstimates[KF::ballYVelocity][0] = c_ballDecayRate*stateEstimates[6][0]; // Reduce ball y velocity assuming deceleration

    float mid_theta = stateEstimates[KF::selfTheta][0] + 0.5*odom_theta;
    stateEstimates[KF::selfX][0] = stateEstimates[KF::selfX][0] + odom_x*cos(mid_theta) - odom_y*sin(mid_theta);
    stateEstimates[KF::selfY][0] = stateEstimates[KF::selfY][0] + odom_x*sin(mid_theta) - odom_y*cos(mid_theta);

// Householder transform. Unscented KF algorithm. Takes a while.
    stateStandardDeviations=HT(horzcat(updateUncertainties*stateStandardDeviations, sqrtOfProcessNoise));
    stateEstimates[2][0] = normaliseAngle(stateEstimates[2][0]); // unwrap the robots angle to keep within -pi < theta < pi.
}



// RHM 7/7/08: Additional function for resetting
void KF::Reset(){
	// Add extra uncertainty
	stateStandardDeviations = HT(horzcat(stateStandardDeviations, sqrtOfProcessNoiseReset));    
}



KfUpdateResult KF::odometeryUpdate(double odom_X, double odom_Y, double odom_Theta, double R_X, double R_Y, double R_Theta)
{
	
// 	std::cout << "Calculating sigma points." << std::endl;
  // Unscented KF Stuff.
	Matrix yBar;                                  	//reset
	Matrix Py;
	Matrix Pxy=Matrix(7, 2, false);                    //Pxy=[0;0;0];
        Matrix scriptX = CalculateSigmaPoints();
        const unsigned int numSigmaPoints = scriptX.getn();

	//----------------------------------------------------------------
// 	std::cout << "Running motion model." << std::endl;
	Pose2D oldPose, diffOdom;
        double *newPose;

        Matrix sigmaPoints = scriptX;

        for (int i = 0 ; i < scriptX.getn(); i++)
	{
		oldPose.X = scriptX[0][i];
		oldPose.Y = scriptX[1][i];
		oldPose.Theta = scriptX[2][i];
   
		diffOdom.X = odom_X;
		diffOdom.Y = odom_Y;
		diffOdom.Theta = odom_Theta;
   
		newPose = odom_Model.getNextSigma(diffOdom,oldPose);

                sigmaPoints[0][i] = *newPose;
		sigmaPoints[1][i] = *(newPose+1);
		sigmaPoints[2][i] = *(newPose+2);
// 		std::cout<<"\nOld Sigma = [ "<<oldPose.X<<", "<<oldPose.Y<<", "<<oldPose.Theta<<" ]"<<"\tNew Sigma = [ "<<sigmaPoints[0][i]<<", "<<sigmaPoints[1][i]<<", "<<sigmaPoints[2][i]<<" ]";
   
   
	}

  // RUN MOTION MODEL HERE -> motionModel(MX,lambda)
//   std::cout << "Calculating new mean and variance." << std::endl;
    
  // Update Mean
	Matrix newStateEstimates(stateEstimates.getm(),stateEstimates.getn(), false);
	Matrix newCovariance(stateStandardDeviations.getm(),stateStandardDeviations.getn(), false);

//   std::cout << "Calculating Mean." << std::endl;
        for(int i=0; i < numSigmaPoints; i++){
		newStateEstimates = newStateEstimates + sqrtOfTestWeightings[0][i]*sqrtOfTestWeightings[0][i]*sigmaPoints.getCol(i);
	}
	std::cout<<"New Mean    = ["<<newStateEstimates[0][0]<<", "<<newStateEstimates[1][0]<<", "<<newStateEstimates[1][0]<<" ]"<<std::endl;
// std::cout << "Calculating Covariance." << std::endl;
	Matrix temp;
  // Update Covariance
        for(int i=0; i < numSigmaPoints; i++){
		temp = sigmaPoints.getCol(i) - newStateEstimates;
		newCovariance = newCovariance + sqrtOfTestWeightings[0][i]*sqrtOfTestWeightings[0][i]*temp*temp.transp();
	}
//   std::cout << "Updating current state." << std::endl;
	stateEstimates = newStateEstimates;
//   newCovariance.print();
	stateStandardDeviations = cholesky(newCovariance);
//   std::cout<<"\nNew current State = ["<<stateStandardDeviations[0][0]<<", "<<stateStandardDeviations[1][0]<<", "<<stateStandardDeviations[1][0]<<" ]";
	return KF_OK; 
}

KfUpdateResult KF::ballmeas(double Ballmeas, double theta_Ballmeas){
  double ballX_rel = Ballmeas * cos(theta_Ballmeas);
  double ballY_rel = Ballmeas * sin(theta_Ballmeas);
  // RHM 8/7/08:
  double R_range = c_R_ball_range_offset + c_R_ball_range_relative*pow(Ballmeas,2);
  double R_bearing = c_R_ball_theta;
    
  // Calculate update uncertainties - S_ball_rel & R_ball_rel.
  Matrix S_ball_rel = Matrix(2,2,false);
  S_ball_rel[0][0] = cos(theta_Ballmeas) * sqrt(R_range);
  S_ball_rel[0][1] = -sin(theta_Ballmeas) * Ballmeas * sqrt(R_bearing);
  S_ball_rel[1][0] = sin(theta_Ballmeas) * sqrt(R_range);
  S_ball_rel[1][1] = cos(theta_Ballmeas) * Ballmeas * sqrt(R_bearing);

  Matrix R_ball_rel = S_ball_rel * S_ball_rel.transp();  // R = S^2

  Matrix yBar;                                  	//reset
  Matrix Py;
  Matrix Pxy = Matrix(7,2,false);                    //Pxy=[0;0;0];

  Matrix scriptX = CalculateSigmaPoints();
  const unsigned int numSigmaPoints = scriptX.getn();

  Matrix scriptY = Matrix(2, numSigmaPoints, false);
  Matrix temp = Matrix(2,1,false);
  for(int i = 0; i < numSigmaPoints; i++){
    temp[0][0] = (scriptX[3][i] - scriptX[0][i]) * cos(scriptX[2][i]) + (scriptX[4][i] - scriptX[1][i]) * sin(scriptX[2][i]);
    temp[1][0] = -(scriptX[3][i] - scriptX[0][i]) * sin(scriptX[2][i]) + (scriptX[4][i] - scriptX[1][i]) * cos(scriptX[2][i]);
    scriptY.setCol(i,temp.getCol(0));
  }
    
  Matrix Mx = Matrix(scriptX.getm(), numSigmaPoints, false);
  Matrix My = Matrix(scriptY.getm(), numSigmaPoints, false);
  for(int i = 0; i < numSigmaPoints; i++){
    Mx.setCol(i, sqrtOfTestWeightings[0][i] * scriptX.getCol(i));
    My.setCol(i, sqrtOfTestWeightings[0][i] * scriptY.getCol(i));
  }                                      

  Matrix M1 = sqrtOfTestWeightings;
  yBar = My * M1.transp(); // Predicted Measurement
  Py = (My - yBar * M1) * (My - yBar * M1).transp();
  Pxy = (Mx - stateEstimates * M1) * (My -yBar * M1).transp();
    
  Matrix K = Pxy * Invert22(Py + R_ball_rel);   // Kalman Filter Gain.

  Matrix y = Matrix(2,1,false); // Measurement.
  y[0][0] = ballX_rel;
  y[1][0] = ballY_rel;
	
	//RHM: (3) Outlier rejection.
	// (i) Compute
  // double innovation2 = (yBar-temp).transp()*Invert22(Py+R_ball_rel)*(yBar-temp);
	double innovation2 = convDble((yBar - y).transp()*Invert22(Py + R_ball_rel)*(yBar - y));
	// (ii) Now compare this with a 'magic number' (threshold2) for outliers, and also vision
  // object confidence, something like if ((innovation2> threshold2) && (visionObject.confidence is small) )
  if (innovation2 > c_threshold2){
	//std::cout<<"not update"<<std::endl;
   	return KF_OUTLIER; // Outlier, so chuck it all out...
  }

  stateStandardDeviations = HT( horzcat(Mx - stateEstimates*M1 - K*My + K*yBar* M1, K*S_ball_rel) ); // Update uncertainties.
  stateEstimates = stateEstimates - K * (yBar - y); // Update States.
    
  return KF_OK; // It all worked OK!
}


/*
// RHM 7/7/08: Change for resetting (return int)
KfUpdateResult KF::fieldObjectmeas(double distance,double bearing,double objX, double objY, double distanceErrorOffset, double distanceErrorRelative, double bearingError){
  double objX_rel = distance * cos(bearing);
  double objY_rel = distance * sin(bearing);

  double R_range = distanceErrorOffset + distanceErrorRelative * pow(distance, 2);
  double R_bearing = bearingError;
  //if(not_goal && INGORE_RANGE) R_range= 22500;	//150^2

  // Calculate update uncertainties - S_obj_rel & R_obj_rel
  Matrix S_obj_rel = Matrix(2,2,false);
  S_obj_rel[0][0] = cos(bearing) * sqrt(R_range);
  S_obj_rel[0][1] = -sin(bearing) * distance * sqrt(R_bearing);
  S_obj_rel[1][0] = sin(bearing) * sqrt(R_range);
  S_obj_rel[1][1] = cos(bearing) * distance * sqrt(R_bearing);

  Matrix R_obj_rel = S_obj_rel * S_obj_rel.transp(); // R = S^2

  // Unscented KF Stuff.
  Matrix yBar;                                  	//reset
  Matrix Py;
  Matrix Pxy=Matrix(7, 2, false);                    //Pxy=[0;0;0];
  Matrix scriptX = CalculateSigmaPoints();
  const unsigned int numSigmaPoints = scriptX.getn();
	//----------------------------------------------------------------
  Matrix scriptY = Matrix(2, numSigmaPoints, false);
  Matrix temp = Matrix(2, 1, false);
 
  double dX,dY,Cc,Ss;
 
  for(int i = 0; i < numSigmaPoints; i++){
 	  dX = objX-scriptX[0][i];
 	  dY = objY-scriptX[1][i];
 	  Cc = cos(scriptX[2][i]);
 	  Ss = sin(scriptX[2][i]);       
    temp[0][0] = dX * Cc + dY * Ss;
    temp[1][0] = -dX * Ss + dY * Cc; 
    scriptY.setCol(i, temp.getCol(0));
  }
  Matrix Mx = Matrix(scriptX.getm(), numSigmaPoints, false);
  Matrix My = Matrix(scriptY.getm(), numSigmaPoints, false);
  for(int i = 0; i < numSigmaPoints; i++){
    Mx.setCol(i, sqrtOfTestWeightings[0][i] * scriptX.getCol(i));
    My.setCol(i, sqrtOfTestWeightings[0][i] * scriptY.getCol(i));
  }
     
  Matrix M1 = sqrtOfTestWeightings;
  yBar = My * M1.transp(); // Predicted Measurement.
  Py = (My - yBar * M1) * (My - yBar * M1).transp();
  Pxy = (Mx - stateEstimates * M1) * (My - yBar * M1).transp();

  Matrix K = Pxy * Invert22(Py + R_obj_rel); // K = Kalman filter gain.

  Matrix y = Matrix(2,1,false); // Measurement. I terms of relative (x,y).
  y[0][0] = objX_rel;
  y[1][0] = objY_rel;
  //end of standard ukf stuff
	//RHM: 20/06/08 Outlier rejection.
  double innovation2 = convDble((yBar - y).transp() * Invert22(Py + R_obj_rel) * (yBar - y));

  // Update Alpha
  double innovation2measError = convDble((yBar - y).transp() * Invert22(R_obj_rel) * (yBar - y));
  m_alpha *= 1 / (1 + innovation2measError);
  //alpha *= CalculateAlphaWeighting(yBar - y,Py+R_obj_rel,c_outlierLikelyhood);


  // SB: 27/06/2011 
  // Changing the outcome of obtaining an outlier during the game playing state// 
  // During the game, if an outlier is detected, it will increase the variance of current model
  // If there are multiple outliers, and the variance goes beyond a certain level, the models are reset
  // However the new position of models are uniformly spread around the current location within a square boundary
  if (innovation2 > c_threshold2)
  {	
	  #ifdef PLAYING_STATE_RESETTING 
		if(Blackboard->GameInfo->getCurrentState() == STATE_PLAYING ) // if an outlier is detected during play
		{
			// Increase the variance of this model by 10%
			std::cout<<"\nIncreasing uncertainty by 10%";
			    stateStandardDeviations[0][0] += stateStandardDeviations[0][0]*0.1;
				stateStandardDeviations[1][1] += stateStandardDeviations[1][1]*0.1;
				stateStandardDeviations[2][2] += stateStandardDeviations[2][2]*0.1;
				stateStandardDeviations[3][3] += stateStandardDeviations[3][3]*0.1;
				stateStandardDeviations[4][4] += stateStandardDeviations[4][4]*0.1;
				stateStandardDeviations[5][5] += stateStandardDeviations[5][5]*0.1;
				stateStandardDeviations[6][6] += stateStandardDeviations[6][6]*0.1;
			
		}
	  #endif
		return KF_OUTLIER;
	}
  if (innovation2 < 0){
	  //std::cout<<"**********************************"<<std::endl;
	  //Py.print();
	  //R_obj_rel.print();
	  //std::cout<<"**********************************"<<std::endl;
  }

  stateStandardDeviations = HT( horzcat(Mx - stateEstimates*M1 - K*My + K*yBar*M1, K*S_obj_rel) );
  stateEstimates = stateEstimates - K*(yBar - y);
  return KF_OK;   
}



*/

KfUpdateResult KF::MultiFieldObs(const Matrix& locations, const Matrix& measurements, const Matrix& R_Measurement)
{
        unsigned int numObs = measurements.getm();
        Matrix R_obj_rel(R_Measurement); // R = S^2
        Matrix S_obj_rel(cholesky(R_obj_rel)); // R = S^2

        // Unscented KF Stuff.
        Matrix yBar;                                  	//reset
        Matrix Py;
        Matrix Pxy=Matrix(7, 2*numObs, false);                    //Pxy=[0;0;0];
        Matrix scriptX = CalculateSigmaPoints();
        const unsigned int numSigmaPoints = scriptX.getn();
              //----------------------------------------------------------------
        Matrix scriptY = Matrix(numObs, numSigmaPoints, false);
        Matrix temp = Matrix(numObs, 1, false);

        double dX,dY,Cc,Ss;

        for(int i = 0; i < numSigmaPoints; i++)
        {
            for(unsigned int j=0; j < numObs; j+=2)
            {
                dX = locations[j][0]-scriptX[0][i];
                dY = locations[j+1][0]-scriptX[1][i];
                temp[j][0] = sqrt(dX*dX + dY*dY);
                temp[j+1][0] = normaliseAngle(atan2(dY,dX) - scriptX[2][i]);
            }
            scriptY.setCol(i, temp.getCol(0));
        }
        Matrix Mx = Matrix(scriptX.getm(), numSigmaPoints, false);
        Matrix My = Matrix(scriptY.getm(), numSigmaPoints, false);
        for(int i = 0; i < numSigmaPoints; i++){
          Mx.setCol(i, sqrtOfTestWeightings[0][i] * scriptX.getCol(i));
          My.setCol(i, sqrtOfTestWeightings[0][i] * scriptY.getCol(i));
        }

        Matrix M1 = sqrtOfTestWeightings;
        yBar = My * M1.transp(); // Predicted Measurement.
        Py = (My - yBar * M1) * (My - yBar * M1).transp();
        Pxy = (Mx - stateEstimates * M1) * (My - yBar * M1).transp();

        Matrix invPyRObj = InverseMatrix(Py + R_obj_rel);

        Matrix K = Pxy * invPyRObj; // K = Kalman filter gain.

        Matrix y = Matrix(measurements); // Measurement. I terms of relative (x,y).

        //end of standard ukf stuff
              //RHM: 20/06/08 Outlier rejection.
        Matrix yDiffTemp = (yBar - y);

        for(unsigned int i = 0; i < yDiffTemp.getm(); i+=2)
        {
            Matrix inv(2,2,false);
            inv[0][0] = invPyRObj[i][i];
            inv[0][1] = invPyRObj[i][i+1];
            inv[1][0] = invPyRObj[i+1][i];
            inv[1][1] = invPyRObj[i+1][i+1];

            Matrix diff(2,1,false);
            diff[0][0] = yDiffTemp[i][0];
            diff[1][0] = yDiffTemp[i+1][0];
            double innovation2 = convDble(diff.transp() * inv * diff);
            if(innovation2 > c_threshold2)
            {
                return KF_OUTLIER;
            }
        }


        // Update Alpha
        double innovation2measError = convDble((yBar - y).transp() * InverseMatrix(R_obj_rel) * (yBar - y));
        m_alpha *= 1 / (1 + innovation2measError);
        //alpha *= CalculateAlphaWeighting(yBar - y,Py+R_obj_rel,c_outlierLikelyhood);

        stateStandardDeviations = HT( horzcat(Mx - stateEstimates*M1 - K*My + K*yBar*M1, K*S_obj_rel) );
        stateEstimates = stateEstimates - K*(yBar - y);
        return KF_OK;
    }

// RHM 7/7/08: Change for resetting (return int)
KfUpdateResult KF::fieldObjectmeas(double distance,double bearing,double objX, double objY, double distanceErrorOffset, double distanceErrorRelative, double bearingError)
{
  double R_range = distanceErrorOffset + distanceErrorRelative * pow(distance, 2);
  double R_bearing = bearingError;
  //if(not_goal && INGORE_RANGE) R_range= 22500;	//150^2

  // Calculate update uncertainties - S_obj_rel & R_obj_rel
  Matrix S_obj_rel = Matrix(2,2,false);
  S_obj_rel[0][0] = sqrt(R_range);
  S_obj_rel[1][1] = sqrt(R_bearing);

  Matrix R_obj_rel = S_obj_rel * S_obj_rel.transp(); // R = S^2

  // Unscented KF Stuff.
  Matrix yBar;                                  	//reset
  Matrix Py;
  Matrix Pxy=Matrix(7, 2, false);                    //Pxy=[0;0;0];
  Matrix scriptX = CalculateSigmaPoints();
  const unsigned int numSigmaPoints = scriptX.getn();
        //----------------------------------------------------------------
  Matrix scriptY = Matrix(2, numSigmaPoints, false);
  Matrix temp = Matrix(2, 1, false);

  double dX,dY,Cc,Ss;

  for(int i = 0; i < numSigmaPoints; i++)
  {
          dX = objX-scriptX[0][i];
          dY = objY-scriptX[1][i];
        temp[0][0] = sqrt(dX*dX + dY*dY);
        temp[1][0] = normaliseAngle(atan2(dY,dX) - scriptX[2][i]);
        scriptY.setCol(i, temp.getCol(0));
  }
  Matrix Mx = Matrix(scriptX.getm(), numSigmaPoints, false);
  Matrix My = Matrix(scriptY.getm(), numSigmaPoints, false);
  for(int i = 0; i < numSigmaPoints; i++){
    Mx.setCol(i, sqrtOfTestWeightings[0][i] * scriptX.getCol(i));
    My.setCol(i, sqrtOfTestWeightings[0][i] * scriptY.getCol(i));
  }

  Matrix M1 = sqrtOfTestWeightings;
  yBar = My * M1.transp(); // Predicted Measurement.
  Py = (My - yBar * M1) * (My - yBar * M1).transp();
  Pxy = (Mx - stateEstimates * M1) * (My - yBar * M1).transp();

  Matrix K = Pxy * Invert22(Py + R_obj_rel); // K = Kalman filter gain.

  Matrix y = Matrix(2,1,false); // Measurement. I terms of relative (x,y).
  y[0][0] = distance;
  y[1][0] = bearing;

  //end of standard ukf stuff
        //RHM: 20/06/08 Outlier rejection.
  double innovation2 = convDble((yBar - y).transp() * Invert22(Py + R_obj_rel) * (yBar - y));

  // Update Alpha
  double innovation2measError = convDble((yBar - y).transp() * Invert22(R_obj_rel) * (yBar - y));
  m_alpha *= 1 / (1 + innovation2measError);
  //alpha *= CalculateAlphaWeighting(yBar - y,Py+R_obj_rel,c_outlierLikelyhood);


  // SB: 27/06/2011
  // Changing the outcome of obtaining an outlier during the game playing state//
  // During the game, if an outlier is detected, it will increase the variance of current model
  // If there are multiple outliers, and the variance goes beyond a certain level, the models are reset
  // However the new position of models are uniformly spread around the current location within a square boundary
  if (innovation2 > c_threshold2)
  {
//          #ifdef PLAYING_STATE_RESETTING
//                        // Increase the variance of this model by 10%
//                        //std::cout<<"\nIncreasing uncertainty by 10%";
//                    stateStandardDeviations[0][0] += stateStandardDeviations[0][0]*0.1;
//                    stateStandardDeviations[1][1] += stateStandardDeviations[1][1]*0.1;
//                    stateStandardDeviations[2][2] += stateStandardDeviations[2][2]*0.1;
//                    stateStandardDeviations[3][3] += stateStandardDeviations[3][3]*0.1;
//                    stateStandardDeviations[4][4] += stateStandardDeviations[4][4]*0.1;
//                    stateStandardDeviations[5][5] += stateStandardDeviations[5][5]*0.1;
//                    stateStandardDeviations[6][6] += stateStandardDeviations[6][6]*0.1;
//          #endif
                return KF_OUTLIER;
        }

  stateStandardDeviations = HT( horzcat(Mx - stateEstimates*M1 - K*My + K*yBar*M1, K*S_obj_rel) );
  stateEstimates = stateEstimates - K*(yBar - y);
  return KF_OK;
}

bool  KF::isVarianceOutOfBounds()
{

	if (variance(0) > VARIANCE_REST_THRESHOLD && variance(1) > VARIANCE_REST_THRESHOLD)
	   return true;
	else
		return false;
}


//
//RHM: 26/6/08 New function, needed for shared ball stuff (and maybe others....)
//======================================================================
//
void KF::linear2MeasurementUpdate(double Y1,double Y2, double SR11, double SR12, double SR22, int index1, int index2){
  //
  // KF update (called for example by shared ball) where a pair of measurements, [Y1;Y2];
  // with variance Square Root of Covariance (SR) [SR11 SR12; 0 SR22]
  // are predicted to be Xhat[index1][0] and Xhat[index2][0] respectively
  // This is based on the function fieldObjectmeas, but simplified.
  //
  // Example Call (given data from wireless: ballX, ballY, SRballXX, SRballXY, SRballYY)
  //      linear2MeasurementUpdate( ballX, ballY, SRballXX, SRballXY, SRballYY, 3, 4 )
  Matrix SR = Matrix(2,2,false);
  SR[0][0] = SR11;
  SR[0][1] = SR12;
  SR[1][1] = SR22;

  Matrix R = Matrix(2,2,false);
  R = SR * SR.transp();

  Matrix Py = Matrix(2, 2, false);
  Matrix Pxy = Matrix(7, 2, false);                   
 
  Matrix CS = Matrix(2, 7, false);
  CS.setRow(0, stateStandardDeviations.getRow(index1));
  CS.setRow(1, stateStandardDeviations.getRow(index2));

  Py = CS * CS.transp();
  Pxy = stateStandardDeviations * CS.transp();

  Matrix K = Pxy * Invert22(Py + R);   //Invert22

  Matrix y = Matrix(2, 1, false);
  y[0][0] = Y1;
  y[1][0] = Y2;
    
  Matrix yBar = Matrix(2, 1, false); //Estimated values of the measurements Y1,Y2
  yBar[0][0] = stateEstimates[index1][0];
  yBar[1][0] = stateEstimates[index2][0]; 
	//RHM: (3) Outlier rejection.
	double innovation2 = convDble( (yBar - y).transp() * Invert22(Py + SR * SR.transp()) * (yBar - y) ); 	
	//std::cout<<innovation2<<std::endl;
	if (innovation2 > c_threshold2){
		//std::cout<<"+++++++++++++++++not update+++++++++++++++++"<<std::endl;
		return;
	}
	if(innovation2 < 0){
		//std::cout<<"**********************************"<<std::endl;
		//Py.print();
		//std::cout<<"**********************************"<<std::endl;
	}

	stateStandardDeviations = HT(horzcat(stateStandardDeviations - K * CS, K * SR));
  stateEstimates = stateEstimates - K * (yBar - y);
  return;
}
//======================================================================================================
//
// RHM: 26/6/08 New Pseudo code for ball information to send over wireless
// Information to add to the broadcast to other robots:
// (along with other information)
// * Frame Number {timing indicator)
// * Frames since I last saw the ball
// * Xhat[3][0] (est ball X)
// * Xhat[4][0] (est ball Y)
// * SRxx, SRxy, SRyy (3 elements of 2x2 sqrt matrix formed as: HT(vertical_cat(S.getRow[3], S.getRow[4])))
// * Xhat[0][0]
// * Xhat[1][0] (could be used in behaviour)
// * other stuff probably
//
//

KfUpdateResult KF::updateAngleBetween(double angle, double x1, double y1, double x2, double y2, double sd_angle)
{
        // Method to take the angle between two objects, that is,
        // angle between object 1 (with fixed field coords (x1,y1))
        // and object 2 (with fixed field coords (x2,y2) ) and perform an update.
        // Note: as distinct from almost all other updates, the actual robot orientation
        // doesn't matter for this update, and so 'cropping' etc. of the sigma points is not requied.

        double R_angle;

    // Unscented KF Stuff.
    double yBar;                                  	//reset
    double Py;
    Matrix Pxy = Matrix(7, 1, false);                    //Pxy=[0;0;0];
    Matrix scriptX = CalculateSigmaPoints();
    const unsigned int numSigmaPoints = scriptX.getn();
    //----------------------------------------------------------------
    Matrix scriptY = Matrix(1, numSigmaPoints, false);

    double angleToObj1;
    double angleToObj2;

    for (int i = 0; i < numSigmaPoints; i++)
    {
        angleToObj1 = atan2 ( y1 - scriptX[1][i], x1 - scriptX[0][i] );
        angleToObj2 = atan2 ( y2 - scriptX[1][i], x2 - scriptX[0][i] );
        scriptY[0][i] = normaliseAngle(angleToObj1 - angleToObj2);
    }

    Matrix Mx = Matrix(scriptX.getm(), numSigmaPoints, false);
    Matrix My = Matrix(scriptY.getm(), numSigmaPoints, false);
    for (int i = 0; i < numSigmaPoints; i++)
    {
        Mx.setCol(i, sqrtOfTestWeightings[0][i] * scriptX.getCol(i));
        My.setCol(i, sqrtOfTestWeightings[0][i] * scriptY.getCol(i));
    }

    Matrix M1 = sqrtOfTestWeightings;
    yBar = convDble ( My * M1.transp() ); // Predicted Measurement.
    Py = convDble ((My - yBar * M1) * (My - yBar * M1).transp());
    Pxy = (Mx - stateEstimates * M1) * (My - yBar * M1).transp();

    R_angle  = sd_angle * sd_angle;

    Matrix K = Pxy /( Py + R_angle ); // K = Kalman filter gain.

    double y = angle;    //end of standard ukf stuff
    //Outlier rejection.
    double innovation2 = (yBar - y) * (yBar - y ) / ( Py + R_angle );

    // Update Alpha (used in multiple model)
    double innovation2measError = (yBar - y) * (yBar - y ) / R_angle ;
    m_alpha *= 1 / (1 + innovation2measError);

    if (innovation2 > c_threshold2)
    {
        return KF_OUTLIER;
    }

    stateStandardDeviations = HT( horzcat(Mx - stateEstimates*M1 - K*My + K*yBar*M1, K*sd_angle) );
    stateEstimates = stateEstimates - K*(yBar - y);
    return KF_OK;
}


double KF::variance(int Xi) const
{
	return convDble(stateStandardDeviations.getRow(Xi)*stateStandardDeviations.getRow(Xi).transp());
}



double KF::sd(int Xi) const
{
	return sqrt(variance(Xi));
}



double KF::state(int stateID) const
{
  return stateEstimates[stateID][0];
}



Matrix KF::GetBallSR() const
{
  return HT(vertcat(stateStandardDeviations.getRow(3), stateStandardDeviations.getRow(4)));
}



double KF::getDistanceToPosition(double posX, double posY) const
{
  double selfX = stateEstimates[0][0];
  double selfY = stateEstimates[1][0];
  double diffX = selfX - posX;
  double diffY = selfY - posY;
  double distance = sqrt( diffX * diffX + diffY * diffY );
  return distance;
}



double KF::getBearingToPosition(double posX, double posY) const
{
  double selfX = stateEstimates[0][0];
  double selfY = stateEstimates[1][0];
  double selfHeading = stateEstimates[2][0];
  double diffX = posX - selfX;
  double diffY = posY - selfY;
  if( (diffX == 0) && (diffY == 0)) diffY = 0.0001;
  double positionHeading = atan2(diffY, diffX);
  double bearing = normaliseAngle(positionHeading - selfHeading);
  return bearing;
}



bool KF::clipState(int stateIndex, double minValue, double maxValue){
    bool clipped = false;
	if(stateEstimates[stateIndex][0] > maxValue){
		double mult, Pii;
		Matrix Si;
		Si = stateStandardDeviations.getRow(stateIndex);
		Pii = convDble(Si * Si.transp());
		mult = (stateEstimates[stateIndex][0] - maxValue) / Pii;
		stateEstimates = stateEstimates - mult * stateStandardDeviations * Si.transp();
		stateEstimates[stateIndex][0] = maxValue;
        clipped = true;
	}
	if(stateEstimates[stateIndex][0] < minValue){
		double mult, Pii;
		Matrix Si;
		Si = stateStandardDeviations.getRow(stateIndex);
		Pii = convDble(Si * Si.transp());
		mult = (stateEstimates[stateIndex][0] - minValue) / Pii;
		stateEstimates = stateEstimates - mult * stateStandardDeviations * Si.transp();
		stateEstimates[stateIndex][0] = minValue;
        clipped = true;
	}
        stateEstimates[2][0] = normaliseAngle(stateEstimates[2][0]);
    return clipped;
}

Matrix KF::CalculateSigmaPoints() const
{
    const unsigned int numSigmaPoints = 2 * nStates + 1;
    Matrix sigmaPoints = Matrix(stateEstimates.getm(), numSigmaPoints, false);
    sigmaPoints.setCol(0, stateEstimates);                         //scriptX(:,1)=Xhat;

//----------------Saturate ScriptX angle sigma points to not wrap
    double sigmaAngleMax = 2.5;
    unsigned int neg_index;
    for(int i=1;i<nStates+1;i++){//hack to make test points distributed
        neg_index = nStates + i;
        // Addition Portion.
        sigmaPoints.setCol(i, stateEstimates + sqrt((double)nStates + c_Kappa) * stateStandardDeviations.getCol(i - 1));
        // Crop heading
        sigmaPoints[2][i] = crop(sigmaPoints[2][i], (-sigmaAngleMax + stateEstimates[2][0]), (sigmaAngleMax + stateEstimates[2][0]));
        // Subtraction Portion.
        sigmaPoints.setCol(neg_index,stateEstimates - sqrt((double)nStates + c_Kappa) * stateStandardDeviations.getCol(i - 1));
        // Crop heading
        sigmaPoints[2][neg_index] = crop(sigmaPoints[2][neg_index], (-sigmaAngleMax + stateEstimates[2][0]), (sigmaAngleMax + stateEstimates[2][0]));
    }
    return sigmaPoints;
}

float KF::CalculateAlphaWeighting(const Matrix& innovation, const Matrix& innovationVariance, float outlierLikelyhood) const
{
    const int numMeas = 2;
    float notOutlierLikelyhood = 1.0 - outlierLikelyhood;
    float expRes = exp(-0.5*convDble(innovation.transp()*Invert22(innovationVariance)*innovation));
    float fracRes = 1.0 / ( sqrt( pow(2*PI,numMeas)*determinant(innovationVariance) ) );
    return notOutlierLikelyhood * fracRes * expRes + outlierLikelyhood;
}


std::string KF::summary(bool brief) const
{
    std::stringstream buffer;
    buffer << "Model Id " << id() << " Alpha: " << alpha() << " Position: (" << state(KF::selfX) << ",";
    buffer << state(KF::selfY) << "," << state(KF::selfTheta) << ")" << std::endl << "Ball Position: (" << state(KF::ballX) << ",";
    buffer << state(KF::ballY) << ")" << std::endl;
    if(!brief)
    {
            buffer << "Variance - Position: (" << sd(KF::selfX) << ",";
            buffer << sd(KF::selfY) << "," << sd(KF::selfTheta) << ") Ball Position: (" << sd(KF::ballX) << ",";
            buffer << sd(KF::ballY) << ")" << std::endl;
    }
    return buffer.str();
}

std::ostream& operator<< (std::ostream& output, const KF& p_kf)
{
    output.write(reinterpret_cast<const char*>(&p_kf.m_isActive), sizeof(p_kf.m_isActive));
    if(p_kf.m_isActive)
    {
        output.write(reinterpret_cast<const char*>(&p_kf.m_alpha), sizeof(p_kf.m_alpha));
        WriteMatrix(output,p_kf.stateEstimates);
        WriteMatrix(output,p_kf.stateStandardDeviations);
    }
    return output;
}

std::istream& operator>> (std::istream& input, KF& p_kf)
{
    input.read(reinterpret_cast<char*>(&p_kf.m_isActive), sizeof(p_kf.m_isActive));
    if(p_kf.m_isActive)
    {
        input.read(reinterpret_cast<char*>(&p_kf.m_alpha), sizeof(p_kf.m_alpha));
        p_kf.stateEstimates  = ReadMatrix(input);
        p_kf.stateStandardDeviations = ReadMatrix(input);
    }
    return input;
}
