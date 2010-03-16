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

using namespace mathGeneral;

// Tuning Values (Constants)
const float KF::c_Kappa = 1.0f; // weight used in w matrix. (Constant)
const float KF::c_ballDecayRate=0.975f; // Ball weighting. (Constant)
const float KF::c_threshold2 = 15.0f; // Threshold for outlier rejection. Magic Number. (Constant)

// Ball distance measurement error weightings (Constant)
const float KF::c_R_ball_theta = 0.0001f;
const float KF::c_R_ball_range_offset = 25.0f; // (5cm)^2
const float KF::c_R_ball_range_relative = 0.0025f; // 5% of range added.


KF::KF(){
    /**************************Initialization**************************/
	/*This is where values can be adjusted*/
  //frameRate = g_fps;
  frameRate = 30;

  alpha = 1.0; // Accuracy of model (0.0 -> 1.0)
  isActive = false; // Model currently in use.
  toBeActivated = false; // Model to be in use.

// Update Uncertainty
  updateUncertainties = Matrix(7, 7, true);
  updateUncertainties[5][5] = c_ballDecayRate; // Ball velocity x
  updateUncertainties[6][6] = c_ballDecayRate; // Ball velocity y
  updateUncertainties[3][5] = 1.0f/frameRate; // [ballX][ballXvelocity]
  updateUncertainties[4][6] = 1.0f/frameRate; // [ballY][ballYvelocity]

  init();									//Initialisation of Xhat and S

// Process Noise - Matrix Square Root of Q
  sqrtOfProcessNoise = Matrix(7,7,true);
  sqrtOfProcessNoise[0][0] = 2.0; // Robot X coord.
  sqrtOfProcessNoise[1][1] = 2.0; // Robot Y coord.
  sqrtOfProcessNoise[2][2] = 0.0316228; // Robot Theta.
  sqrtOfProcessNoise[3][3] = 4.0; // Ball X.
  sqrtOfProcessNoise[4][4] = 4.0; // Ball Y.
  sqrtOfProcessNoise[5][5] = 3.16228; // Ball X Velocity.
  sqrtOfProcessNoise[6][6] = 3.16228; // Ball Y Velocity.

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

// Create square root of W matrix
  sqrtOfTestWeightings = Matrix(1,2*nStates+1,false);
  sqrtOfTestWeightings[0][0] = sqrt(c_Kappa/(nStates+c_Kappa));
  double outerWeighting = sqrt(1.0/(2*(nStates+c_Kappa)));
  for(int i=1; i <= 2*nStates; i++){
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


void KF::timeUpdate(double odometeryForward, double odometeryLeft, double odometeryTurn){
   	//-----------------------Update for locomotion
	updateUncertainties[0][2] = -odometeryForward*sin(stateEstimates[2][0]) - odometeryLeft*cos(stateEstimates[2][0]); //??
	updateUncertainties[1][2] = odometeryForward*cos(stateEstimates[2][0]) - odometeryLeft*sin(stateEstimates[2][0]); //??

  // Movement along X-axis from odometry.
	stateEstimates[0][0] = stateEstimates[0][0] + odometeryForward*cos(stateEstimates[2][0]) - odometeryLeft*sin(stateEstimates[2][0]);
  // Movement along Y-axis from odometry.
	stateEstimates[1][0] = stateEstimates[1][0] + odometeryForward*sin(stateEstimates[2][0]) + odometeryLeft*cos(stateEstimates[2][0]);
  // Rotational movement in theta (turn) from odometry.
	stateEstimates[2][0] = stateEstimates[2][0] + odometeryTurn;

   	//-----------------------Update for ball velocity	
	stateEstimates[3][0] = stateEstimates[3][0] + stateEstimates[5][0]/frameRate; // Update ball x position by ball x velocity.
	stateEstimates[4][0] = stateEstimates[4][0] + stateEstimates[6][0]/frameRate; // Update ball y position by ball y velocity.
	stateEstimates[5][0] = c_ballDecayRate*stateEstimates[5][0]; // Reduce ball x velocity assuming deceleration
	stateEstimates[6][0] = c_ballDecayRate*stateEstimates[6][0]; // Reduce ball y velocity assuming deceleration

  // Householder transform. Unscented KF algorithm. Takes a while.
	stateStandardDeviations=HT(horzcat(updateUncertainties*stateStandardDeviations, sqrtOfProcessNoise));
	
        stateEstimates[2][0] = normaliseAngle(stateEstimates[2][0]); // unwrap the robots angle to keep within -pi < theta < pi.
	return;
}



// RHM 7/7/08: Additional function for resetting
void KF::Reset(){
	// Add extra uncertainty
	stateStandardDeviations = HT(horzcat(stateStandardDeviations, sqrtOfProcessNoiseReset));    
}


KfUpdateResult KF::odometeryUpdate(double odom_X, double odom_Y, double odom_Theta, double R_X, double R_Y, double R_Theta){

  // Calculate relative update Standard Deviation and Variance matricies.
  Matrix S_odom_rel = Matrix(3,3,false);
  S_odom_rel[0][0] = sqrt(R_X);
  S_odom_rel[1][1] = sqrt(R_Y);
  S_odom_rel[2][2] = sqrt(R_Theta);

  Matrix R_odom_rel = S_odom_rel * S_odom_rel.transp(); // R = S^2

  // Unscented KF Stuff.
  Matrix yBar;                                  	//reset
  Matrix Py;
  Matrix Pxy=Matrix(7, 2, false);                    //Pxy=[0;0;0];
  Matrix scriptX=Matrix(stateEstimates.getm(), 2 * nStates + 1, false);
  scriptX.setCol(0, stateEstimates);                         //scriptX(:,1)=Xhat;                  
    
  //----------------Saturate ScriptX angle sigma points to not wrap
  double sigmaAngleMax = 2.5;
  for(int i=1;i<nStates+1;i++){//hack to make test points distributed
    // Addition Portion.
    scriptX.setCol(i, stateEstimates + sqrt((double)nStates + c_Kappa) * stateStandardDeviations.getCol(i - 1));
	// Crop heading
    scriptX[2][i] = crop(scriptX[2][i], (-sigmaAngleMax + stateEstimates[2][0]), (sigmaAngleMax + stateEstimates[2][0]));
    // Subtraction Portion.
    scriptX.setCol(nStates + i,stateEstimates - sqrt((double)nStates + c_Kappa) * stateStandardDeviations.getCol(i - 1));
	// Crop heading
    scriptX[2][nStates + i] = crop(scriptX[2][nStates + i], (-sigmaAngleMax + stateEstimates[2][0]), (sigmaAngleMax + stateEstimates[2][0]));
  }
	//----------------------------------------------------------------
 
  Matrix Mx = Matrix(scriptX.getm(), 2 * nStates + 1, false);
  for(int i = 0; i < 2 * nStates + 1; i++){
    Mx.setCol(i, sqrtOfTestWeightings[0][i] * scriptX.getCol(i));
  }

  // RUN MOTION MODEL HERE -> motionModel(MX,lambda)
  Matrix updatedSigmaPoints = Mx;
    
  // Update Mean
  Matrix newStateEstimates(stateEstimates.getm(),stateEstimates.getn(), false);
  Matrix newStandardDeviations(stateStandardDeviations.getm(),stateStandardDeviations.getn(), false);

  for(int i=1; i <= 2*nStates; i++){
    newStateEstimates = sqrtOfTestWeightings[0][i]*sqrtOfTestWeightings[0][i]*updatedSigmaPoints.getCol(i);
  }

  Matrix temp(stateEstimates.getm(),stateEstimates.getn(), false);
  // Update Covariance
  for(int i=1; i <= 2*nStates; i++){
	temp = updatedSigmaPoints.getCol(i) - newStateEstimates;
    newStandardDeviations = sqrtOfTestWeightings[0][i]*sqrtOfTestWeightings[0][i]*temp*temp.transp();
  }
  
  stateEstimates = newStateEstimates;
  stateStandardDeviations = newStandardDeviations;
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

  Matrix scriptX = Matrix(stateEstimates.getm(),2 * nStates + 1,false);
  scriptX.setCol(0,stateEstimates);                         //scriptX(:,1)=Xhat; Current state.
  for(int i = 1; i <= nStates; i++){  // Unscented KF. Creates test points used to compare against vision data.
    // Addition Portion.
    scriptX.setCol(i, stateEstimates + sqrt(nStates + c_Kappa) * stateStandardDeviations.getCol(i - 1));
    // Subtraction Portion.
    scriptX.setCol(nStates + i, stateEstimates - sqrt(nStates + c_Kappa) * stateStandardDeviations.getCol(i - 1));
  }
    
  Matrix scriptY = Matrix(2, 2 * nStates + 1, false);
  Matrix temp = Matrix(2,1,false);
  for(int i = 0; i < 2 * nStates + 1; i++){
    temp[0][0] = (scriptX[3][i] - scriptX[0][i]) * cos(scriptX[2][i]) + (scriptX[4][i] - scriptX[1][i]) * sin(scriptX[2][i]);
    temp[1][0] = -(scriptX[3][i] - scriptX[0][i]) * sin(scriptX[2][i]) + (scriptX[4][i] - scriptX[1][i]) * cos(scriptX[2][i]);
    scriptY.setCol(i,temp.getCol(0));
  }
    
  Matrix Mx = Matrix(scriptX.getm(), 2 * nStates + 1, false);
  Matrix My = Matrix(scriptY.getm(), 2 * nStates + 1, false);  
  for(int i = 0; i < 2 * nStates + 1; i++){
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
  Matrix scriptX=Matrix(stateEstimates.getm(), 2 * nStates + 1, false);
  scriptX.setCol(0, stateEstimates);                         //scriptX(:,1)=Xhat;                  
    
  //----------------Saturate ScriptX angle sigma points to not wrap
  double sigmaAngleMax = 2.5;
  for(int i=1;i<nStates+1;i++){//hack to make test points distributed
    // Addition Portion.
    scriptX.setCol(i, stateEstimates + sqrt((double)nStates + c_Kappa) * stateStandardDeviations.getCol(i - 1));
	// Crop heading.
    scriptX[2][i] = crop(scriptX[2][i], (-sigmaAngleMax + stateEstimates[2][0]), (sigmaAngleMax + stateEstimates[2][0]));
    // Subtraction Portion.
    scriptX.setCol(nStates + i,stateEstimates - sqrt((double)nStates + c_Kappa) * stateStandardDeviations.getCol(i - 1));
	// Crop heading.
    scriptX[2][nStates + i] = crop(scriptX[2][nStates + i], (-sigmaAngleMax + stateEstimates[2][0]), (sigmaAngleMax + stateEstimates[2][0]));
  }
	//----------------------------------------------------------------
  Matrix scriptY = Matrix(2, 2 * nStates + 1, false);
  Matrix temp = Matrix(2, 1, false);
 
  double dX,dY,Cc,Ss;
 
  for(int i = 0; i < 2 * nStates + 1; i++){
 	  dX = objX-scriptX[0][i];
 	  dY = objY-scriptX[1][i];
 	  Cc = cos(scriptX[2][i]);
 	  Ss = sin(scriptX[2][i]);       
    temp[0][0] = dX * Cc + dY * Ss;
    temp[1][0] = -dX * Ss + dY * Cc; 
    scriptY.setCol(i, temp.getCol(0));
  }
  Matrix Mx = Matrix(scriptX.getm(), 2 * nStates + 1, false);
  Matrix My = Matrix(scriptY.getm(), 2 * nStates + 1, false);  
  for(int i = 0; i < 2 * nStates + 1; i++){
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
  alpha *= 1 / (1 + innovation2measError);

  if (innovation2 > c_threshold2){
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



double KF::variance(int Xi){
	return convDble(stateStandardDeviations.getRow(Xi)*stateStandardDeviations.getRow(Xi).transp());
}



double KF::sd(int Xi){
	return sqrt(variance(Xi));
}



double KF::getState(int stateID){
  return stateEstimates[stateID][0];
}



Matrix KF::GetBallSR(){
  return HT(vertcat(stateStandardDeviations.getRow(3), stateStandardDeviations.getRow(4)));
}



double KF::getDistanceToPosition(double posX, double posY){
  double selfX = stateEstimates[0][0];
  double selfY = stateEstimates[1][0];
  double diffX = selfX - posX;
  double diffY = selfY - posY;
  double distance = sqrt( diffX * diffX + diffY * diffY );
  return distance;
}



double KF::getBearingToPosition(double posX, double posY){
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
