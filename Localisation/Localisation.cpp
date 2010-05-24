#include "Localisation.h"
#include "NUPlatform/NUSensors/NUSensorsData.h"

#include "Tools/Math/General.h"
#include <string>
#include <stdlib.h>
#include <iostream>
#include <fstream>

//#define debug_out cout
#if DEBUG_LOCALISATION_VERBOSITY > 0
#define debug_out debug_file
#endif // DEBUG_LOCALISATION_VERBOSITY > 0

using namespace mathGeneral;

typedef std::vector<StationaryObject> StationaryObjects;
typedef StationaryObjects::iterator StationaryObjectsIt;
typedef StationaryObjects::const_iterator StationaryObjectsConstIt;

typedef std::vector<MobileObject> MobileObjects;
typedef MobileObjects::iterator MobileObjectsIt;
typedef MobileObjects::const_iterator MobileObjectsConstIt;

typedef std::vector<AmbiguousObject> AmbiguousObjects;
typedef AmbiguousObjects::iterator AmbiguousObjectsIt;
typedef AmbiguousObjects::const_iterator AmbiguousObjectsConstIt;

// Constant value initialisation
const float Localisation::c_LargeAngleSD = 1.5f;   //For variance check
const float Localisation::c_OBJECT_ERROR_THRESHOLD = 0.3f;
const float Localisation::c_OBJECT_ERROR_DECAY = 0.94f;
const float Localisation::c_RESET_SUM_THRESHOLD = 5.0f; // 3 // then 8.0 (home)
const int Localisation::c_RESET_NUM_THRESHOLD = 2;

// Object distance measurement error weightings (Constant)
const float Localisation::R_obj_theta = 0.001f; // (0.01 rad)^2
const float Localisation::R_obj_range_offset = 10.0f*10.0f; // (10cm)^2
const float Localisation::R_obj_range_relative = 0.02f; // 10% of range added. (0.1)^2

const float Localisation::centreCircleBearingError = (float)(deg2rad(10)*deg2rad(10)); // (10 degrees)^2

Localisation::Localisation()
{
    for(int m = 0; m < c_MAX_MODELS; m++)
    {
	    models[m].isActive = true; // Start with just the first model model
	    models[m].toBeActivated = false;	
    }

	
    wasPreviouslyPenalised = false;

    feedbackPosition[0] = 0;
    feedbackPosition[1] = 0;
    feedbackPosition[2] = 0;
    

    // RHM 7/7/08: Extra array for resetting algorithm
    for(int m = 0; m < c_MAX_MODELS; m++){
        for (int i=0; i< c_numOutlierTrackedObjects; i++) modelObjectErrors[m][i] = 0.0;
    }

    #if DEBUG_LOCALISATION_VERBOSITY > 0
    #ifdef WIN32
    debug_file.open("Localisation.log", ios::out | ios::trunc);
    #else
    debug_file.open("/var/volatile/Localisation.log", ios::out | ios::trunc);
    #endif // WIN32
    #endif // DEBUG_LOCALISATION_VERBOSITY > 0

    return;
}



Localisation::~Localisation()
{
    #if DEBUG_LOCALISATION_VERBOSITY > 0
    debug_file.close();
    #endif // DEBUG_LOCALISATION_VERBOSITY > 0
}


//--------------------------------- MAIN FUNCTIONS  ---------------------------------//


void Localisation::process(NUSensorsData* data, FieldObjects* fobs)
{
    float odo_time;
    vector<float> odo;
    data->getOdometry(odo_time, odo);
	odomForward = odo[0];
	odomLeft = odo[1];
	odomTurn = odo[2];
    
	ProcessObjects(0,fobs,NULL);
//	doTimeUpdate(odomForward,odomLeft,odomTurn);
}





void Localisation::ProcessObjects(int frameNumber, FieldObjects* ourfieldObjects, void* mostRecentPackets)
{
	int numUpdates = 0;
	int updateResult;  
	currentFrameNumber = frameNumber;
	objects = ourfieldObjects;

	CheckGameState();
    //if(balanceFallen) return;
// 	debug_out  << "Dont put anything "<<endl;
	#if DEBUG_LOCALISATION_VERBOSITY > 2
		if(numUpdates == 0 )
		{ 
			debug_out  <<"[" << currentFrameNumber << "]: Update Starting." << endl;
			for(int i = 0; i < c_MAX_MODELS; i++){
				if(models[i].isActive == false) continue;
				debug_out  << "[" << currentFrameNumber << "]: Model[" << i << "]";
				debug_out  << " [alpha = " << models[i].alpha << "]";
				debug_out  << " Robot X: " << models[i].getState(0);
				debug_out  << " Robot Y: " << models[i].getState(1);
				debug_out  << " Robot Theta: " << models[i].getState(2) << endl;
			}
		}
	#endif // DEBUG_LOCALISATION_VERBOSITY > 2
	
	
	// Correct orientation to face a goal if you can see it and are unsure which way you are facing.
 	varianceCheckAll(); 
	
	// perform odometry update and change the variance of the model
	doTimeUpdate(fabs(odomForward), odomLeft, odomTurn);
// 	doTimeUpdate(0,0,0);
	
	#if DEBUG_LOCALISATION_VERBOSITY > 2
    		debug_out  << "[" << currentFrameNumber << "]: Time update - odomForward = " << odomForward 
			<< " odomLeft = " << odomLeft << " odomTurn = " << odomTurn << endl;
	#endif // DEBUG_LOCALISATION_VERBOSITY > 2

    
       
 	if(objects != NULL)
  	{
		// Proccess the Stationary Known Field Objects
		StationaryObjectsIt currStat(objects->stationaryFieldObjects.begin());
		StationaryObjectsConstIt endStat(objects->stationaryFieldObjects.end());
		
		for(; currStat != endStat; ++currStat)
		{
			if(currStat->isObjectVisible() == false) continue; // Skip objects that were not seen.
			//TODO: Remove this if condition once we are sure of distance and bearing of all FOs
			if(currStat->getID() == FieldObjects::FO_BLUE_LEFT_GOALPOST || currStat->getID() == FieldObjects::FO_BLUE_RIGHT_GOALPOST
						|| currStat->getID() == FieldObjects::FO_YELLOW_LEFT_GOALPOST 
						|| currStat->getID() == FieldObjects::FO_YELLOW_RIGHT_GOALPOST )
			{
				updateResult = doKnownLandmarkMeasurementUpdate((*currStat));
				numUpdates++;
			}
		}
// 	
// 	
// 		
// 		// Proccess the Moving Known Field Objects
// 		MobileObjectsIt currMob(objects->mobileFieldObjects.begin());
// 		MobileObjectsConstIt endMob(objects->mobileFieldObjects.end());
// 		
// 		for (; currMob != endMob; ++currMob)
// 		{
// 			if(currMob->isObjectVisible() == false) continue; // Skip objects that were not seen.
// 			updateResult = doBallMeasurementUpdate((*currMob));
// 			numUpdates++;
// 		}
// 		
// 				
		NormaliseAlphas();
// 
// 		#if SHARED_BALL_ON
// 		/*
// 		// Check the game packets.
// 		// We only want to do the shared ball updates if we can't see the ball ourselves.
// 		// there have been probems where the team will keep sharing the previous position of their ball
// 		// and updates in vision do not supercede the shared data.
// 				int myPlayerNumber = GameController::getInstance().getPlayerNumber();
// 				if(ourfieldObjects[FO_BALL].framesSinceLastSeen > 3){ // TODO: Change to a SD value
// 				for(int robotNum = 0; robotNum < NUM_ROBOTS; robotNum++){
// 				if(myPlayerNumber == (robotNum+1)) continue;
// 				if(mostRecentPackets[robotNum].processedWM == false){
// 				if(mostRecentPackets[robotNum].packet.ball.seen == true){
// 		#if DEBUG_LOCALISATION_VERBOSITY > 2
// 				debug_out  << "[" << currentFrameNumber << "]: Doing Shared ball update from robot " << robotNum+1 <<  endl;
// 		#endif
// 				doSharedBallUpdate(mostRecentPackets[robotNum].packet.ball);
// 			}
// 				else {
// 		#if DEBUG_LOCALISATION_VERBOSITY > 2
// 				debug_out  << "[" << currentFrameNumber << "]: Skipping shared ball update from robot " << robotNum+1 << endl;
// 		#endif
// 			}
// 				mostRecentPackets[robotNum].processedWM = true;
// 			}
// 			}
// 			}
// 		*/
// 		#endif // SHARED_BALL_ON
// 		
// 		NormaliseAlphas();
// 		
// 		
		#if MULTIPLE_MODELS_ON
    		// Do Ambiguous objects.
		AmbiguousObjectsIt currAmb(objects->ambiguousFieldObjects.begin());
		AmbiguousObjectsConstIt endAmb(objects->ambiguousFieldObjects.end());
		for(; currAmb != endAmb; ++currAmb){
			if(currAmb->isObjectVisible() == false) continue; // Skip objects that were not seen.
			if(currStat->getID() == FieldObjects::FO_BLUE_GOALPOST_UNKNOWN || currStat->getID() == FieldObjects::FO_YELLOW_GOALPOST_UNKNOWN)
			{
				updateResult = doAmbiguousLandmarkMeasurementUpdate((*currAmb), objects->stationaryFieldObjects);
				NormaliseAlphas();
				numUpdates++;
			}
		}

		MergeModels(c_MAX_MODELS_AFTER_MERGE);
		#endif // MULTIPLE_MODELS_ON

		
		
		
		
		///****************************************
		#if DEBUG_LOCALISATION_VERBOSITY > 0
		for (int currID = 0; currID < c_MAX_MODELS; currID++){
			if(models[currID].isActive )
			{
				
				debug_out<<"Model : "<<currID<<" Pos  : "<<models[currID].stateEstimates[0][0]<<", "
					<<models[currID].stateEstimates[0][1]<<","<<
						models[currID].stateEstimates[0][2]<<endl;
			}
		}
		#endif
		
		///****************************************
		
		
		
		
// 		// Check for model reset. -> with multiple models just remove if not last one??
// 		// Need to re-do reset to be model specific.
// 		//int numReset = CheckForOutlierResets();
// 		CheckForOutlierResets();
// 		
// 		// clip models back on to field.
// 		clipActiveModelsToField();
// 		
// 		// Store WM Data in Field Objects.
// 		int bestModelID = getBestModelID(); 
// 		// Get the best model to use.
// 	
// 		/*
// 		for(int objID = 0; objID < NUM_FIELD_OBJECTS; objID++)
// 		{
// 			if(ourfieldObjects[objID].isFixed())
// 			{
// 				ourfieldObjects[objID].wmDistance = models[bestModelID].getDistanceToPosition(ourfieldObjects[objID].wmX,
// 		 							ourfieldObjects[objID].wmY);
// 				ourfieldObjects[objID].wmBearing = models[bestModelID].getBearingToPosition(ourfieldObjects[objID].wmX,
// 		 							ourfieldObjects[objID].wmY);
// 			}
// 			else if(objID == FO_BALL)
// 			{
// 				ourfieldObjects[objID].wmX = models[bestModelID].getState(3);
// 				ourfieldObjects[objID].sdx = models[bestModelID].sd(3);
// 				ourfieldObjects[objID].wmY = models[bestModelID].getState(4);
// 				ourfieldObjects[objID].sdy = models[bestModelID].sd(4);
// 				ourfieldObjects[objID].vX = models[bestModelID].getState(5);
// 				ourfieldObjects[objID].vY = models[bestModelID].getState(6);
// 				ourfieldObjects[objID].wmDistance = models[bestModelID].getDistanceToPosition(ourfieldObjects[objID].wmX,
// 		 							ourfieldObjects[objID].wmY);
// 				ourfieldObjects[objID].wmBearing = models[bestModelID].getBearingToPosition(ourfieldObjects[objID].wmX, 
// 									ourfieldObjects[objID].wmY);
// 			}
// 			else if(objID == FO_TEAM1)
// 			{
// 				ourfieldObjects[objID].wmX = models[bestModelID].getState(0);
// 				ourfieldObjects[objID].sdx = models[bestModelID].sd(0);
// 				ourfieldObjects[objID].wmY = models[bestModelID].getState(1);
// 				ourfieldObjects[objID].sdy = models[bestModelID].sd(1);
// 				ourfieldObjects[objID].wmOrientation = models[bestModelID].getState(2);
// 				ourfieldObjects[objID].sdtheta = models[bestModelID].sd(2);
// 			}
// 		}
// 		*/
// 		
// 		
// 		#if DEBUG_LOCALISATION_VERBOSITY > 2
// 			if(numUpdates > 0)
// 			{
// 				for (int i = 0; i < c_MAX_MODELS; i++){
// 					if(models[i].isActive == false) continue;
// 					debug_out  << "[" << currentFrameNumber << "]: Model[" << i << "]";
// 					debug_out  << " [alpha = " << models[i].alpha << "]";
// 					debug_out  << " Robot X: " << models[i].getState(0);
// 					debug_out  << " Robot Y: " << models[i].getState(1);
// 					debug_out  << " Robot Theta: " << models[i].getState(2) << endl;
// 				}
// 				debug_out  << "[" << currentFrameNumber << "]: Best Model";
// 				debug_out  << " [alpha = " << models[bestModelID].alpha << "]";
// 				debug_out  << " Robot X: " << models[bestModelID].getState(0);
// 				debug_out  << " Robot Y: " << models[bestModelID].getState(1);
// 				debug_out  << " Robot Theta: " << models[bestModelID].getState(2) << endl;
// 			}
// 		#endif // DEBUG_LOCALISATION_VERBOSITY > 2
 
 		
 		
 	
 	}
	
}











void Localisation::CheckGameState()
{
    /*
    GameController* gc = &GameController::getInstance();
    bool isPenalised = gc->isPenalised();
    int currentState = gc->getGameState();

    // If robot has been penalised and penalty is lifted, reset to the penalty replacement positions.
    if( (isPenalised == false) && (wasPreviouslyPenalised == true) ){
        doPenaltyReset();
    } 
    else if ( (previousGameState != currentState) && (previousGameState == STATE_INITIAL)){
        doPlayerReset();
    }

    if(gc->getTimeSinceLastBallOut() == 0){
        // Increase uncertainty of ball position if it has gone out.. Cause it has probably been moved.
        for (int modelNumber = 0; modelNumber < c_MAX_MODELS; modelNumber++){
            if(models[modelNumber].isActive == false) continue;
            models[modelNumber].stateStandardDeviations[3][3] = 150.0; // 100 cm
            models[modelNumber].stateStandardDeviations[4][4] = 100.0; // 150 cm
            models[modelNumber].stateStandardDeviations[5][5] = 10.0;   // 10 cm/s
            models[modelNumber].stateStandardDeviations[6][6] = 10.0;   // 10 cm/s
        }
    }
    
    if(balanceFallen == true){
        for (int modelNumber = 0; modelNumber < c_MAX_MODELS; modelNumber++){
            // Increase heading uncertainty if fallen
            models[modelNumber].stateStandardDeviations[2][2] = 2.0;   // 2 radians
        }
    }
    wasPreviouslyPenalised = isPenalised;
    previousGameState = currentState;
}

void Localisation::doPenaltyReset()
{
#if DEBUG_LOCALISATION_VERBOSITY > 0
    debug_out  << "[" << currentFrameNumber << "] Performing penalty reset." << endl;
#endif // DEBUG_LOCALISATION_VERBOSITY > 0

    // Reset all of the models
    for(int m = 0; m < c_MAX_MODELS; m++){
        // reset outlier error count
	    for (int i=0; i<NUM_FIELD_OBJECTS; i++) modelObjectErrors[m][i] = 0.0;

        // Disable models    
        models[m].isActive = false;
        models[m].toBeActivated = false;
    }

    // setup model 0 as one 'T'
    models[0].isActive = true;
    models[0].alpha = 0.5;

    // Set the state estimates
    models[0].stateEstimates[0][0] = 0.0;       // Robot x
    models[0].stateEstimates[1][0] = 200.0;     // Robot y
    models[0].stateEstimates[2][0] = -PI/2.0;    // Robot heading
    models[0].stateEstimates[3][0] = 0.0;       // Ball x 
    models[0].stateEstimates[4][0] = 0.0;       // Ball y
    models[0].stateEstimates[5][0] = 0.0;       // Ball vx
    models[0].stateEstimates[6][0] = 0.0;       // Ball vy

    // Set the uncertainties
    resetSdMatrix(0);
    
    // setup model 1 as the other 'T'
    models[1].isActive = true;  
    models[1].alpha = 0.5;

    // Set the state estimates
    models[1].stateEstimates[0][0] = 0.0;       // Robot x
    models[1].stateEstimates[1][0] = -200.0;    // Robot y
    models[1].stateEstimates[2][0] = PI/2.0;   // Robot heading
    models[1].stateEstimates[3][0] = 0.0;       // Ball x
    models[1].stateEstimates[4][0] = 0.0;       // Ball y
    models[1].stateEstimates[5][0] = 0.0;       // Ball vx
    models[1].stateEstimates[6][0] = 0.0;       // ball vy

    // Set the uncertainties
    resetSdMatrix(1);
*/
    return;
}

void Localisation::doPlayerReset()
{
    #if DEBUG_LOCALISATION_VERBOSITY > 0
    debug_out  << "[" << currentFrameNumber << "] Performing player reset." << endl;
    #endif // DEBUG_LOCALISATION_VERBOSITY > 0

    // Reset all of the models
    for(int m = 0; m < c_MAX_MODELS; m++)
    {
        // reset outlier error count
	    if(objects!=NULL)
            	for (unsigned int i=0; i<objects->stationaryFieldObjects.size(); i++) modelObjectErrors[m][i] = 0.0;

        // Disable models    
        models[m].isActive = false;
        models[m].toBeActivated = false;
    }

    // setup model 0 as in yellow goals
    models[0].isActive = true;
    models[0].alpha = 0.25;

    models[0].stateEstimates[0][0] = -300.0;         // Robot x
    models[0].stateEstimates[1][0] = 0.0;           // Robot y
    models[0].stateEstimates[2][0] = 0.0;           // Robot heading
    models[0].stateEstimates[3][0] = 0.0;       // Ball x 
    models[0].stateEstimates[4][0] = 0.0;       // Ball y
    models[0].stateEstimates[5][0] = 0.0;       // Ball vx
    models[0].stateEstimates[6][0] = 0.0;       // Ball vy

    // Set the uncertainties
    resetSdMatrix(0);

    // setup model 1 as in blue goals
    models[1].isActive = true;
    models[1].alpha = 0.25;

    models[1].stateEstimates[0][0] = -300.0;        // Robot x
    models[1].stateEstimates[1][0] = 0.0;           // Robot y
    models[1].stateEstimates[2][0] = 0.0;           // Robot heading
    models[1].stateEstimates[3][0] = 0.0;       // Ball x 
    models[1].stateEstimates[4][0] = 0.0;       // Ball y
    models[1].stateEstimates[5][0] = 0.0;       // Ball vx
    models[1].stateEstimates[6][0] = 0.0;       // Ball vy

    // Set the uncertainties
    resetSdMatrix(1);

    // setup model 2 as one half way 'T'
    models[2].isActive = true;
    models[2].alpha = 0.25;

    models[2].stateEstimates[0][0] = -300.0;        // Robot x
    models[2].stateEstimates[1][0] = 0.0;           // Robot y
    models[2].stateEstimates[2][0] = 0.0;           // Robot heading
    models[2].stateEstimates[3][0] = 0.0;       // Ball x 
    models[2].stateEstimates[4][0] = 0.0;       // Ball y
    models[2].stateEstimates[5][0] = 0.0;       // Ball vx
    models[2].stateEstimates[6][0] = 0.0;       // Ball vy

    // Set the uncertainties
    resetSdMatrix(2);

    // setup model 3 as other half way 'T'
    models[3].isActive = true;
    models[3].alpha = 0.25;

    models[3].stateEstimates[0][0] = -300.0;        // Robot x
    models[3].stateEstimates[1][0] = 0.0;           // Robot y
    models[3].stateEstimates[2][0] = 0.0;           // Robot heading
    models[3].stateEstimates[3][0] = 0.0;       // Ball x 
    models[3].stateEstimates[4][0] = 0.0;       // Ball y
    models[3].stateEstimates[5][0] = 0.0;       // Ball vx
    models[3].stateEstimates[6][0] = 0.0;       // Ball vy

    // Set the uncertainties
    resetSdMatrix(3);
    return;
}


void Localisation::resetSdMatrix(int modelNumber)
{
    // Set the uncertainties
//     models[modelNumber].stateStandardDeviations[0][0] = 150.0; // 100 cm
//     models[modelNumber].stateStandardDeviations[1][1] = 100.0; // 150 cm
//     models[modelNumber].stateStandardDeviations[2][2] = 2.0;   // 2 radians
//     models[modelNumber].stateStandardDeviations[3][3] = 150.0; // 100 cm
//     models[modelNumber].stateStandardDeviations[4][4] = 100.0; // 150 cm
//     models[modelNumber].stateStandardDeviations[5][5] = 10.0;   // 10 cm/s
//     models[modelNumber].stateStandardDeviations[6][6] = 10.0;   // 10 cm/s
//     
    
    models[modelNumber].stateStandardDeviations[0][0] = 10.0; // 100 cm
    models[modelNumber].stateStandardDeviations[1][1] = 10.0; // 150 cm
    models[modelNumber].stateStandardDeviations[2][2] = 0.2;   // 2 radians
    models[modelNumber].stateStandardDeviations[3][3] = 10.0; // 100 cm
    models[modelNumber].stateStandardDeviations[4][4] = 10.0; // 150 cm
    models[modelNumber].stateStandardDeviations[5][5] = 1.0;   // 10 cm/s
    models[modelNumber].stateStandardDeviations[6][6] = 1.0;   // 10 cm/s
    
    
    return;  
}

bool Localisation::clipModelToField(int modelID)
{
//    const double fieldXLength = 440.0;
//    const double fieldYLength = 680.0;
    const double fieldXLength = 680.0;
    const double fieldYLength = 440.0;
    const double fieldXMax = fieldXLength / 2.0;
    const double fieldXMin = - fieldXLength / 2.0; 
    const double fieldYMax = fieldYLength / 2.0; 
    const double fieldYMin = - fieldYLength / 2.0;

    bool wasClipped = false;
    bool clipped;
    double prevX, prevY, prevTheta;
    prevX = models[modelID].getState(0);
    prevY = models[modelID].getState(1);
    prevTheta = models[modelID].getState(2);

    clipped = models[modelID].clipState(0, fieldXMin, fieldXMax);		// Clipping for robot's X
    #if DEBUG_LOCALISATION_VERBOSITY > 1
    if(clipped){
        debug_out  << "[" << currentFrameNumber << "]: Model[" << modelID << "]";
        debug_out  << " [alpha = " << models[modelID].alpha << "]";
        debug_out  << " State(0) clipped.";
        debug_out  << " (" << prevX << "," << prevY << "," << prevTheta << ") -> (" << models[modelID].getState(0);
        debug_out  << "," << models[modelID].getState(1) << "," << models[modelID].getState(2) << ")" << endl;
    }
    #endif // DEBUG_LOCALISATION_VERBOSITY > 1
    wasClipped = wasClipped || clipped;

    prevX = models[modelID].getState(0);
    prevY = models[modelID].getState(1);
    prevTheta = models[modelID].getState(2);

    clipped = models[modelID].clipState(1, fieldYMin, fieldYMax);		// Clipping for robot's Y

    #if DEBUG_LOCALISATION_VERBOSITY > 1
    if(clipped){
        debug_out  << "[" << currentFrameNumber << "]: Model[" << modelID << "]";
        debug_out  << " [alpha = " << models[modelID].alpha << "]";
        debug_out  << " State(1) clipped." << endl;
        debug_out  << " (" << prevX << "," << prevY << "," << prevTheta << ") -> (" << models[modelID].getState(0);
        debug_out  << "," << models[modelID].getState(1) << "," << models[modelID].getState(2) << ")" << endl;
    }
    #endif // DEBUG_LOCALISATION_VERBOSITY > 1

    wasClipped = wasClipped || clipped;
    prevX = models[modelID].getState(0);
    prevY = models[modelID].getState(1);
    prevTheta = models[modelID].getState(2);

    clipped = models[modelID].clipState(3, fieldXMin, fieldXMax);		// Clipping for ball's X

    #if DEBUG_LOCALISATION_VERBOSITY > 1
    if(clipped){
        debug_out  << "[" << currentFrameNumber << "]: Model[" << modelID << "]";
        debug_out  << " [alpha = " << models[modelID].alpha << "]";
        debug_out  << " State(3) clipped." << endl;
        debug_out  << " (" << prevX << "," << prevY << "," << prevTheta << ") -> (" << models[modelID].getState(0);
        debug_out  << "," << models[modelID].getState(1) << "," << models[modelID].getState(2) << ")" << endl;
    }
    #endif // DEBUG_LOCALISATION_VERBOSITY > 1
    
    wasClipped = wasClipped || clipped;

    prevX = models[modelID].getState(0);
    prevY = models[modelID].getState(1);
    prevTheta = models[modelID].getState(2);

    clipped = models[modelID].clipState(4, fieldYMin, fieldYMax);		// Clipping for ball's Y

    #if DEBUG_LOCALISATION_VERBOSITY > 1
    if(clipped){
        debug_out  << "[" << currentFrameNumber << "]: Model[" << modelID << "]";
        debug_out  << " [alpha = " << models[modelID].alpha << "]";
        debug_out  << " State(4) clipped." << endl;
        debug_out  << " (" << prevX << "," << prevY << "," << prevTheta << ") -> (" << models[modelID].getState(0);
        debug_out  << "," << models[modelID].getState(1) << "," << models[modelID].getState(2) << ")" << endl;
    }
    #endif // DEBUG_LOCALISATION_VERBOSITY > 1
    
    wasClipped = wasClipped || clipped;

    return wasClipped;
}



bool Localisation::clipActiveModelsToField()
{
    bool wasClipped = false;
    bool modelClipped = false;
    for(int modelID = 0; modelID < c_MAX_MODELS; modelID++){
        if(models[modelID].isActive == true){
            modelClipped = clipModelToField(modelID);
            wasClipped = wasClipped || modelClipped;
        }
    }
    return wasClipped;
}



bool Localisation::doTimeUpdate(float odomForward, float odomLeft, float odomTurn)
{
    bool result = false;
    for(int modelID = 0; modelID < c_MAX_MODELS; modelID++)
    {
        if(models[modelID].isActive == false) continue; // Skip Inactive models.
        result = true;
	models[modelID].performFiltering(odomForward, odomLeft, odomTurn);
    }
    return result;
}


/*
int Localisation::doSharedBallUpdate(WirelessFieldObj &sharedBall)
{
    int kf_return;
    int numSuccessfulUpdates = 0;
    double sharedBallX = sharedBall.x;
    double sharedBallY = sharedBall.y;
    double SRXX = sharedBall.SRXX;
    double SRXY = sharedBall.SRXY;
    double SRYY = sharedBall.SRYY;

#if DEBUG_LOCALISATION_VERBOSITY > 1
    debug_out  << "[" << currentFrameNumber << "]: Doing Shared Ball Update. X = " << sharedBallX << " Y = " << sharedBallY << " SRXX = " << SRXX << " SRXY = " << SRXY << "SRYY = " << SRYY << endl;
#endif

    for(int modelID = 0; modelID < c_MAX_MODELS; modelID++){
        if(models[modelID].isActive == false) continue; // Skip Inactive models.
        kf_return = KF_OK;
        models[modelID].linear2MeasurementUpdate(sharedBallX, sharedBallY, SRXX, SRXY, SRYY, 3, 4);
        if(kf_return == KF_OK) numSuccessfulUpdates++;
    }
    return numSuccessfulUpdates;
}
*/


int Localisation::doBallMeasurementUpdate(MobileObject &ball)
{
    int kf_return;
    int numSuccessfulUpdates = 0;

    #if DEBUG_LOCALISATION_VERBOSITY > 1
    debug_out  <<"[" << currentFrameNumber << "]: Doing Ball Update. Distance = " << ball.measuredDistance() << " Bearing = " << ball.measuredBearing() << endl;
    #endif // DEBUG_LOCALISATION_VERBOSITY > 1

    double flatBallDistance = ball.measuredDistance() * cos(ball.measuredElevation());
    for(int modelID = 0; modelID < c_MAX_MODELS; modelID++){
        if(models[modelID].isActive == false) continue; // Skip Inactive models.
        kf_return = KF_OK;
        kf_return = models[modelID].ballmeas(flatBallDistance, ball.measuredBearing());
        if(kf_return == KF_OK) numSuccessfulUpdates++;
    }
    return numSuccessfulUpdates;
}

int Localisation::doKnownLandmarkMeasurementUpdate(StationaryObject &landmark)
{
    int kf_return;
    int numSuccessfulUpdates = 0;
    int objID = landmark.getID();
    //double flatObjectDistance = landmark.measuredDistance() * cos(landmark.measuredElevation());
    double flatObjectDistance = landmark.measuredDistance();

	double distanceOffsetError = R_obj_range_offset;
	double distanceRelativeError = R_obj_range_relative;
	double bearingError = R_obj_theta;

	switch(objID)
	{
        	case FieldObjects::FO_CORNER_CENTRE_CIRCLE:
			bearingError = centreCircleBearingError;
			break;
		default:
			break;
	}

    for(int modelID = 0; modelID < c_MAX_MODELS; modelID++)
    {
        if(models[modelID].isActive == false) continue; // Skip Inactive models.

        #if DEBUG_LOCALISATION_VERBOSITY > 1
        debug_out  <<"[" << currentFrameNumber << "]: Model[" << modelID << "] Landmark Update. "; 
        //debug_out  << "Object = " << landmark.name();
        debug_out  << " Distance = " << landmark.measuredDistance();
        debug_out  << " Bearing = " << landmark.measuredBearing();
        debug_out  << " Location = (" << landmark.X() << "," << landmark.Y() << ")...";
        #endif // DEBUG_LOCALISATION_VERBOSITY > 1

        if(landmark.measuredBearing() != landmark.measuredBearing())
	{

            #if DEBUG_LOCALISATION_VERBOSITY > 0
            debug_out  << "ABORTED Object Update Bearing is NaN skipping object." << endl;
            #endif // DEBUG_LOCALISATION_VERBOSITY > 0

            continue;
        }
	kf_return = KF_OK;
        kf_return = models[modelID].fieldObjectmeas(flatObjectDistance, landmark.measuredBearing(),landmark.X(), landmark.Y(),
			distanceOffsetError, distanceRelativeError, bearingError);
        if(kf_return == KF_OUTLIER) modelObjectErrors[modelID][landmark.getID()] += 1.0;

        #if DEBUG_LOCALISATION_VERBOSITY > 1
       		 if(kf_return == KF_OK) 
			 debug_out  << "OK" << endl;
        	else 
			debug_out  << "OUTLIER" << endl;
        #endif // DEBUG_LOCALISATION_VERBOSITY > 1

        if(kf_return == KF_OK) 
		numSuccessfulUpdates++;
    }
    return numSuccessfulUpdates;
}



int Localisation::doAmbiguousLandmarkMeasurementUpdate(AmbiguousObject &ambigousObject, const vector<StationaryObject>& possibleObjects)
{
    int kf_return;

    /*
    #if AMBIGUOUS_CORNERS_ON <= 0
    if((ambigousObject.getID() != FO_BLUE_GOALPOST_UNKNOWN) && (ambigousObject.getID() != FO_YELLOW_GOALPOST_UNKNOWN)){
    #if DEBUG_LOCALISATION_VERBOSITY > 1
        debug_out  <<"[" << currentFrameNumber << "]: ingored unkown object " << ambigousObject.name() << std::endl;
    #endif // DEBUG_LOCALISATION_VERBOSITY > 1
        return KF_OUTLIER;
    }
    #endif // AMBIGUOUS_CORNERS_ON <= 0
    */

    vector<int> possabilities = ambigousObject.getPossibleObjectIDs();
    unsigned int numOptions = possabilities.size();
    int outlierModelID = -1;
    int numFreeModels = getNumFreeModels();
    int numActiveModels = getNumActiveModels();
    int numRequiredModels = numActiveModels * (numOptions); // An extra base model.

    if(numFreeModels < numRequiredModels){
        int maxActiveAfterMerge = c_MAX_MODELS /  (numOptions + 1);

        #if DEBUG_LOCALISATION_VERBOSITY > 2
        debug_out  <<"[" << currentFrameNumber << "]: Only " <<  numFreeModels << " Free. Need " << numRequiredModels << " for Update." << endl;
        debug_out  <<"[" << currentFrameNumber << "]: Merging to " << maxActiveAfterMerge << " Max models." << endl;
        #endif // DEBUG_LOCALISATION_VERBOSITY > 2

        MergeModels(maxActiveAfterMerge);

        #if DEBUG_LOCALISATION_VERBOSITY > 2
        debug_out  <<"[" << currentFrameNumber << "]: " << getNumFreeModels() << " models now available." << endl;
        #endif // DEBUG_LOCALISATION_VERBOSITY > 2

        if(getNumFreeModels() < (getNumActiveModels() * (int)numOptions)){

            #if DEBUG_LOCALISATION_VERBOSITY > 0
            debug_out  <<"[" << currentFrameNumber << "]: " << "Not enough models. Aborting Update." << endl;
            #endif // DEBUG_LOCALISATION_VERBOSITY > 0

            return KF_OUTLIER;
        }
    }

    #if DEBUG_LOCALISATION_VERBOSITY > 1
    //debug_out <<"[" << currentFrameNumber << "]: Doing Ambiguous Object Update. Object = " << ambigousObject.name();
    debug_out << " Distance = " << ambigousObject.measuredDistance();
    debug_out  << " Bearing = " << ambigousObject.measuredBearing() << endl;
    #endif // DEBUG_LOCALISATION_VERBOSITY > 1

    for (int modelID = 0; modelID < c_MAX_MODELS; modelID++){
        if(models[modelID].isActive == false) continue; // Skip inactive models.

        // Copy initial model to the temporary model.
        tempModel = models[modelID];
        tempModel.isActive = false;
        tempModel.toBeActivated = true;
        
        // Save Original model as outlier option.
        models[modelID].alpha*=0.0005;
        outlierModelID = -1;
//        modelObjectErrors[modelID][ambigousObject.getID()] += 1.0;
  
        // Now go through each of the possible options, and apply it to a copy of the model
        for(unsigned int optionNumber = 0; optionNumber < numOptions; optionNumber++){
            int possibleObjectID = possabilities[optionNumber];
            int newModelID = FindNextFreeModel();
    
            // If an invalid modelID has been returned, something has gone horribly wrong, so stop here.
            if(newModelID < 0){ 

                #if DEBUG_LOCALISATION_VERBOSITY > 0
                debug_out  <<"[" << currentFrameNumber << "]: !!! WARNING !!! Bad Model ID returned. Update aborted." << endl;
                #endif // DEBUG_LOCALISATION_VERBOSITY > 0

                for(int m = 0; m < c_MAX_MODELS; m++) models[m].toBeActivated = false;
                return -1;
            }

            models[newModelID] = tempModel; // Get the new model from the temp

            // Copy outlier history from the current model.
            for (int i=0; i<c_numOutlierTrackedObjects; i++){
                modelObjectErrors[newModelID][i] = modelObjectErrors[modelID][i];
            }

            // Do the update.
            kf_return =  models[newModelID].fieldObjectmeas(ambigousObject.measuredDistance(), ambigousObject.measuredBearing(),possibleObjects[possibleObjectID].X(), possibleObjects[possibleObjectID].Y(), R_obj_range_offset, R_obj_range_relative, R_obj_theta);

            #if DEBUG_LOCALISATION_VERBOSITY > 2
            debug_out  <<"[" << currentFrameNumber << "]: Splitting model[" << modelID << "] to model[" << newModelID << "].";
            //debug_out  << " Object = " << fieldObjects[possibleObjectID].name();
            debug_out  << "\tLocation = (" << possibleObjects[possibleObjectID].X() << "," << possibleObjects[possibleObjectID].Y() << ")...";
            #endif // DEBUG_LOCALISATION_VERBOSITY > 2

            // If the update reult was an outlier rejection, the model need not be kept as the
            // information is already contained in the designated outlier model created earlier
            if (kf_return == KF_OUTLIER) {
		        models[newModelID].toBeActivated=false;
		   /*
        if (outlierModelID < 0) {
          outlierModelID = newModelID;
        } else {
          MergeTwoModels(outlierModelID, newModelID); 
          models[newModelID].toBeActivated=false;
        }
		*/
            }

#if DEBUG_LOCALISATION_VERBOSITY > 2
            if(kf_return == KF_OK) debug_out  << "OK" << "  Resulting alpha = " << models[newModelID].alpha << endl;
            else debug_out  << "OUTLIER" << "  Resulting alpha = " << models[newModelID].alpha << endl;
#endif

        }
    }
    // Split alpha between choices and also activate models
    for (int i=0; i< c_MAX_MODELS; i++) {
        if (models[i].toBeActivated) {
            models[i].alpha *= 1.0/((float)numOptions); // Divide each models alpha by the numbmer of splits.
            models[i].isActive=true;
        }
        models[i].toBeActivated=false; // Turn off activation flag
    }
    return 1;
}



bool Localisation::MergeTwoModels(int index1, int index2)
{
    // Merges second model into first model, then disables second model.
    bool success = true;
    if(index1 == index2) success = false; // Don't merge the same model.
//    if((model[index1].active == false) || (model[index2].active == false)) success = false; // Both models must be active.
    if(success == false){

#if DEBUG_LOCALISATION_VERBOSITY > 0
        debug_out  <<"[" << currentFrameNumber << "]: Merge Between model[" << index1 << "] and model[" << index2 << "] FAILED." << endl;
#endif // DEBUG_LOCALISATION_VERBOSITY > 0

        return success;
    }

    // Merge alphas
    double alphaMerged = models[index1].alpha + models[index2].alpha;
    double alpha1 = models[index1].alpha / alphaMerged;
    double alpha2 = models[index2].alpha / alphaMerged;

    Matrix xMerged; // Merge State matrix

    // If one model is much more correct than the other, use the correct states.
    // This prevents drifting from continuouse splitting and merging even when one model is much more likely.
    if(models[index1].alpha > 10*models[index2].alpha){
        xMerged = models[index1].stateEstimates;
    } 
    else if (models[index2].alpha > 10*models[index1].alpha){
        xMerged = models[index2].stateEstimates;
    } 
    else {
        xMerged = (alpha1 * models[index1].stateEstimates + alpha1 * models[index2].stateEstimates);
        // Fix angle.
        double angleDiff = models[index2].stateEstimates[2][0] - models[index1].stateEstimates[2][0];
        angleDiff = normaliseAngle(angleDiff);
        xMerged[2][0] = normaliseAngle(models[index1].stateEstimates[2][0] + alpha2*angleDiff);
    }
 
    // Merge Covariance matrix (S = sqrt(P))
    Matrix xDiff = models[index1].stateEstimates - xMerged;
    Matrix p1 = (models[index1].stateStandardDeviations * models[index1].stateStandardDeviations.transp() + xDiff * xDiff.transp());

    xDiff = models[index2].stateEstimates - xMerged;
    Matrix p2 = (models[index2].stateStandardDeviations * models[index2].stateStandardDeviations.transp() + xDiff * xDiff.transp());
  
    Matrix sMerged = cholesky(alpha1 * p1 + alpha2 * p2); // P merged = alpha1 * p1 + alpha2 * p2.

    // Copy merged value to first model
    models[index1].alpha = alphaMerged;
    models[index1].stateEstimates = xMerged;
    models[index1].stateStandardDeviations = sMerged;

    // Disable second model
    models[index2].isActive = false;
    models[index2].toBeActivated = false;

    for (int i=0; i<c_numOutlierTrackedObjects; i++) modelObjectErrors[index2][i] = 0.0; // Reset outlier values.
    return true;
}



int Localisation::getNumActiveModels()
{
    int numActive = 0;
    for (int modelID = 0; modelID < c_MAX_MODELS; modelID++){
        if(models[modelID].isActive == true) numActive++;
    }
    return numActive;
}



int Localisation::getNumFreeModels()
{
    int numFree = 0;
    for (int modelID = 0; modelID < c_MAX_MODELS; modelID++){
        if((models[modelID].isActive == false) && (models[modelID].toBeActivated == false)) numFree++;    
    }
    return numFree;
}



KF &Localisation::getBestModel()
{
    return models[getBestModelID()];
}



int Localisation::getBestModelID()
{
    // Return model with highest alpha value.
    int bestID = 0;
    for (int currID = 0; currID < c_MAX_MODELS; currID++){
        if(models[currID].isActive == false) continue; // Skip inactive models.
        if(models[currID].alpha > models[bestID].alpha) bestID = currID;
    }
    return bestID;
}

bool Localisation::CheckModelForOutlierReset(int modelID)
{
    // RHM 7/7/08: Suggested incorporation of 'Resetting' for possibly 'kidnapped' robot
    //----------------------------------------------------------
    double sum = 0.0;
    int numObjects = 0;
    bool reset = false;
    for(int objID = 0; objID < c_numOutlierTrackedObjects; objID++){
        sum += modelObjectErrors[modelID][objID];
        if (modelObjectErrors[modelID][objID] > c_OBJECT_ERROR_THRESHOLD) numObjects+=1;
        modelObjectErrors[modelID][objID] *= c_OBJECT_ERROR_DECAY;
    }

    // Check if enough recent 'outliers' that we should reset ?
    if ((sum > c_RESET_SUM_THRESHOLD) && (numObjects >= c_RESET_NUM_THRESHOLD)) {
        reset = true;
        models[modelID].Reset(); //Reset KF varainces. Leave Xhat!

        #if DEBUG_LOCALISATION_VERBOSITY > 1
        debug_out << "[" << currentFrameNumber << "]: Model[" << modelID << "] Reset due to outliers." << endl;
        #endif // DEBUG_LOCALISATION_VERBOSITY > 1

        for (int i=0; i<c_numOutlierTrackedObjects; i++) modelObjectErrors[modelID][i] = 0.0; // Reset the outlier history
    }
    return reset;
}



int  Localisation::CheckForOutlierResets()
{
    bool numResets = 0;
    for (int modelID = 0; modelID < c_MAX_MODELS; modelID++){
        if(CheckModelForOutlierReset(modelID)) numResets++;
    }
    return numResets;
}



int Localisation::varianceCheckAll()
{
    int numModelsChanged = 0;
    bool changed;
    for (int currID = 0; currID < c_MAX_MODELS; currID++){
        if(models[currID].isActive == false)
	{
		continue; // Skip inactive models.
	}
        changed = varianceCheck(currID);
        if(changed) 
	{
		numModelsChanged++;
	}
	
    }
    return numModelsChanged;
}

bool Localisation::varianceCheck(int modelID)
{
    // Which direction on the field you should be facing to see the goals
    const double blueDirection = PI;
    const double yellowDirection = 0;

    bool   changed = false;
    double var = models[modelID].variance(2);	//angle variance
    bool   largeVariance = (var > (c_LargeAngleSD * c_LargeAngleSD));
    
    // If we think we know where we are facing don't change anything
//    if(largeVariance == false) return changed;

     // Otherwise try to adjust to fit a goal we can see.
     // Blue Goal - From center at PI radians bearing.
    if (objects == NULL)
    {
	    return false;    
    }
     if( (objects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST].isObjectVisible() == true) && (objects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST].measuredDistance() > 100) )
     {	
  	  #if DEBUG_LOCALISATION_VERBOSITY > 0
	     debug_out<<"Localisation : Saw left blue goal , and distance is : "
			    <<objects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST].measuredDistance()<<endl;
	  #endif
          models[modelID].stateEstimates[2][0]=(blueDirection - objects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST].measuredBearing());
              changed = true;
     }
 
     else if( (objects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST].isObjectVisible() == true) && (objects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST].measuredDistance() > 100) )
     {
	#if DEBUG_LOCALISATION_VERBOSITY > 0
	     debug_out<<"Localisation : Saw right blue goal , and distance is : "
			    <<objects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST].measuredDistance()<<endl;
	#endif

         models[modelID].stateEstimates[2][0]=(blueDirection - objects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST].measuredBearing());
         changed = true;
     }
         /* NEED TO FIX THIS I DON't KNOW HOW IT WILL WORK YET!
 	else if( (objects[FO_BLUE_GOALPOST_UNKNOWN].seen == true) && (objects[FO_BLUE_GOALPOST_UNKNOWN].visionDistance > 100) ){
 		models[modelID].stateEstimates[2][0]=(blueDirection - objects[FO_BLUE_GOALPOST_UNKNOWN].visionBearing);
         changed = true;
 	}
         */
 
   // Yellow Goal - From center at 0.0 radians bearing.
     if( (objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST].isObjectVisible() == true) &&
	  (objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST].measuredDistance() > 100) )
     {
	#if DEBUG_LOCALISATION_VERBOSITY > 0
	     debug_out<<"Localisation : Saw left yellow goal , and distance is : "
			    <<objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST].measuredDistance()<<endl;
	#endif
			     
         models[modelID].stateEstimates[2][0]=(yellowDirection -
			 objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST].measuredBearing());
         changed = true;
     }
     else if( (objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST].isObjectVisible() == true) &&
	       (objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST].measuredDistance() > 100) )
     {
	     
	#if DEBUG_LOCALISATION_VERBOSITY > 0
	     debug_out<<"Localisation : Saw left yellow goal , and distance is : "
			    <<objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST].measuredDistance()<<endl;
	#endif
			     
         models[modelID].stateEstimates[2][0]=(yellowDirection -
			 objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST].measuredBearing());
         changed = true;
     }
     
     
     /* NEED TO FIX THIS I DON't KNOW HOW IT WILL WORK YET!
     else if( (objects[FO_YELLOW_GOALPOST_UNKNOWN].seen == true) && (objects[FO_YELLOW_GOALPOST_UNKNOWN].visionDistance > 100) ){
         models[modelID].stateEstimates[2][0]=(yellowDirection - objects[FO_YELLOW_GOALPOST_UNKNOWN].visionBearing);
         changed = true;
     }
     */
 
	#if DEBUG_LOCALISATION_VERBOSITY > 1
   	  if(changed)
	  {
		  
         	debug_out << "[" << currentFrameNumber << "]: Model[" << modelID << "]";
         	debug_out << "Bearing adjusted due to Goal. New Value = " << models[modelID].stateEstimates[2][0] << endl;
	  }
 	#endif
   	  return changed;
}



void Localisation::NormaliseAlphas()
{
    // Normalise all of the models alpha values such that all active models sum to 1.0
    double sumAlpha=0.0;
    for (int i = 0; i < c_MAX_MODELS; i++) {
        if (models[i].isActive) {
            sumAlpha+=models[i].alpha;
        }
    }
    if(sumAlpha == 1) return;
    if (sumAlpha == 0) sumAlpha = 1e-12;
    for (int i = 0; i < c_MAX_MODELS; i++) {
        if (models[i].isActive) {
            models[i].alpha=models[i].alpha/sumAlpha;
        }
    }
}



int Localisation::FindNextFreeModel()
{
    for (int i=0; i<c_MAX_MODELS; i++) {
        if ((models[i].isActive == true) || (models[i].toBeActivated == true)) continue;
        else return i;
    }
    return -1; // NO FREE MODELS - This is very, very bad.
}


// Reset all of the models
void Localisation::ResetAll()
{

#if DEBUG_LOCALISATION_VERBOSITY > 1
    debug_out  <<"[" << currentFrameNumber << "]: Resetting All Models." << endl;
#endif

    for(int modelNum = 0; modelNum < c_MAX_MODELS; modelNum++){
        models[modelNum].init();
        for (int i=0; i<c_numOutlierTrackedObjects; i++) modelObjectErrors[modelNum][i] = 0.0; // Reset outlier values.
    }
}



//**************************************************************************
//  This method begins the process of merging close models together

void Localisation::MergeModels(int maxAfterMerge) {
    MergeModelsBelowThreshold(0.001);
    MergeModelsBelowThreshold(0.01);
  
//  double threshold=0.04;
    double threshold=0.05;

    while (getNumActiveModels()>maxAfterMerge) {
        MergeModelsBelowThreshold(threshold);
//      threshold*=5.0;
        threshold+=0.05;
    }
    return;
}



void Localisation::PrintModelStatus(int modelID)
{
#if DEBUG_LOCALISATION_VERBOSITY > 1
  debug_out  <<"[" << currentFrameNumber << "]: Model[" << modelID << "]";
  debug_out  << "[alpha=" << models[modelID].alpha << "]";
  debug_out  << " active = " << models[modelID].isActive;
  debug_out  << " activate = " << models[modelID].toBeActivated << endl;
#endif
  return;
}



void Localisation::MergeModelsBelowThreshold(double MergeMetricThreshold)
{
    double mergeM;
    for (int i = 0; i < c_MAX_MODELS; i++) {
        for (int j = i; j < c_MAX_MODELS; j++) {
            if(i == j) continue;
            if (!models[i].isActive || !models[j].isActive ) continue;
            mergeM = abs( MergeMetric(i,j) );
            if (mergeM < MergeMetricThreshold) { //0.5
#if DEBUG_LOCALISATION_VERBOSITY > 2
                debug_out  <<"[" << currentFrameNumber << "]: Merging Model[" << j << "][alpha=" << models[j].alpha << "]";
                debug_out  << " into Model[" << i << "][alpha=" << models[i].alpha << "] " << " Merge Metric = " << mergeM << endl  ;
#endif
                MergeTwoModels(i,j);
            }
        }
    }
}



//************************************************************************
// model to compute a metric for how 'far' apart two models are in terms of merging.
double Localisation::MergeMetric(int index1, int index2)
{   
    if (index1==index2) return 10000.0;
    if (!models[index1].isActive || !models[index2].isActive ) return 10000.0; //at least one model inactive
    Matrix xdif = models[index1].stateEstimates - models[index2].stateEstimates;
    Matrix p1 = models[index1].stateStandardDeviations * models[index1].stateStandardDeviations.transp();
    Matrix p2 = models[index2].stateStandardDeviations * models[index2].stateStandardDeviations.transp();
  
    xdif[2][0] = normaliseAngle(xdif[2][0]);

    double dij=0;
    for (int i=0; i<p1.getm(); i++) {
        dij+=(xdif[i][0]*xdif[i][0]) / (p1[i][i]+p2[i][i]);
    }
    return dij*( (models[index1].alpha*models[index2].alpha) / (models[index1].alpha+models[index2].alpha) );
}


void Localisation::feedback(double* feedback)
{
	feedbackPosition[0] = feedback[0];
	feedbackPosition[1] = feedback[1];
	feedbackPosition[2] = feedback[2];
}

void Localisation::measureLocalization(double x,double y,double theta)
{
// 	cout<<stateEstimates
// 	cout<<x<<", "<<stateEstimates[0][0]<<", "<<y<<", "stateEstimates[1][0]<<", "<<theta<<", "<<stateEstimates[2][0]<<endl;
	
	models[0].measureLocalization(x,y,theta);

}