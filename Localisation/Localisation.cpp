#include "Localisation.h"

#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/GameInformation/GameInformation.h"
#include "Infrastructure/TeamInformation/TeamInformation.h"

#include "Tools/Math/General.h"
#include <string>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include "nubotdataconfig.h"
#include "nubotconfig.h"

#define MULTIPLE_MODELS_ON 1
#define AMBIGUOUS_CORNERS_ON 0
#define SHARED_BALL_ON 1
#define TWO_OBJECT_UPDATE_ON 1

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
const float Localisation::R_obj_theta = 0.0316f*0.0316f;        // (0.01 rad)^2
const float Localisation::R_obj_range_offset = 10.0f*10.0f;     // (10cm)^2
const float Localisation::R_obj_range_relative = 0.15f*0.15f;   // 20% of range added

const float Localisation::centreCircleBearingError = (float)(deg2rad(20)*deg2rad(20)); // (10 degrees)^2

const float Localisation::sdTwoObjectAngle = (float) 0.02; //Small! error in angle difference is normally very small

Localisation::Localisation(int playerNumber): m_timestamp(0)
{
    
    m_previously_incapacitated = true;
    m_previous_game_state = GameInformation::InitialState;
    m_currentFrameNumber = 0;

    feedbackPosition[0] = 0;
    feedbackPosition[1] = 0;
    feedbackPosition[2] = 0;
	
    amILost = true;
    lostCount = 0;
    timeSinceFieldObjectSeen = 0;

    initSingleModel(67.5f, 0, mathGeneral::PI);

    #if DEBUG_LOCALISATION_VERBOSITY > 0
        std::stringstream debugLogName;
        debugLogName << DATA_DIR;
        if(playerNumber) debugLogName << playerNumber;
        debugLogName << "Localisation.log";
        debug_file.open(debugLogName.str().c_str());
        debug_file.clear();
        debug_file << "Localisation" << std::endl;
    #endif // DEBUG_LOCALISATION_VERBOSITY > 0
    return;
}

Localisation::Localisation(const Localisation& source): TimestampedData()
{
    *this = source;
}

Localisation& Localisation::operator=(const Localisation& source)
{
    if (this != &source) // protect against invalid self-assignment
    {
        m_timestamp = source.m_timestamp;
        for (int i = 0; i < c_MAX_MODELS; i++)
        {
            m_models[i] = source.m_models[i];
        }

        m_currentFrameNumber = source.m_currentFrameNumber;
        for(int i = 0; i < c_MAX_MODELS; i++)
        {
            for ( int j = 0; j < c_numOutlierTrackedObjects; j++)
            {
                m_modelObjectErrors[i][j] = source.m_modelObjectErrors[i][j];
            }
        }

        // Game state memory
        m_previously_incapacitated = source.m_previously_incapacitated;
        m_previous_game_state = source.m_previous_game_state;
    }
    // by convention, always return *this
    return *this;
}


Localisation::~Localisation()
{
    #if DEBUG_LOCALISATION_VERBOSITY > 0
    debug_file.close();
    #endif // DEBUG_LOCALISATION_VERBOSITY > 0
}


//--------------------------------- MAIN FUNCTIONS  ---------------------------------//


void Localisation::process(NUSensorsData* sensor_data, FieldObjects* fobs, const GameInformation* gameInfo, const TeamInformation* teamInfo)
{

    m_frame_log.str("");
    if (sensor_data == NULL or fobs == NULL)
        return;

    float time_increment = sensor_data->CurrentTime - m_timestamp;
    m_timestamp = sensor_data->CurrentTime;
    m_currentFrameNumber++;

#if LOC_SUMMARY > 0
    m_frame_log << "Frame " << m_currentFrameNumber << " Time: " << m_timestamp << std::endl;
#endif

    bool doProcessing = CheckGameState(sensor_data->isIncapacitated(), gameInfo);

    if(doProcessing == false)
    {
        #if LOC_SUMMARY > 0
        m_frame_log << "Processing Cancelled." << std::endl;
        #endif
        return;
    }
    


    #ifndef USE_VISION
        vector<float> gps;
        float compass;
        if (sensor_data->getGps(gps) and sensor_data->getCompass(compass))
        {   
            #if LOC_SUMMARY > 0
            m_frame_log << "Setting position from GPS: (" << gps[0] << "," << gps[1] << "," << compass << ")" << std::endl;
            #endif
            fobs->self.updateLocationOfSelf(gps[0], gps[1], compass, 0.1, 0.1, 0.01, false);
            return;
        }
    #else
        vector<float> odo;
        if (sensor_data->getOdometry(odo))
        {
            float fwd = odo[0];
            float side = odo[1];
            float turn = odo[2];
            // perform odometry update and change the variance of the model
            #if LOC_SUMMARY > 0
            m_frame_log << "Time Update - Odometry: (" << fwd << "," << side << "," << turn << ")";
            m_frame_log << " Time Increment: " << time_increment << std::endl;
            #endif
            doTimeUpdate(fwd, side, turn, time_increment);
            #if LOC_SUMMARY > 0
            m_frame_log << "Result: " << getBestModel().summary();
            #endif
        }

        std::vector<TeamPacket::SharedBall> sharedBalls = teamInfo->getSharedBalls();
        #if LOC_SUMMARY > 0
        m_frame_log << "Observation Update:" << std::endl;
        int objseen = 0;
        bool seen;
        std::vector<StationaryObject>::iterator s_it = fobs->stationaryFieldObjects.begin();
        while(s_it != fobs->stationaryFieldObjects.end())
        {
            seen = s_it->isObjectVisible();
            if(seen)
                ++objseen;
            ++s_it;
        }
        m_frame_log << "Stationary Objects: " << objseen << std::endl;

        objseen = 0;
        for (int i=0; i < fobs->mobileFieldObjects.size(); i++)
        {
            if(fobs->mobileFieldObjects[i].isObjectVisible()) ++objseen;
        }
        m_frame_log << "Mobile Objects: " << objseen << std::endl;
        m_frame_log << "Ambiguous Objects: " << fobs->ambiguousFieldObjects.size() << std::endl;
        m_frame_log << "Shared Information: " << sharedBalls.size() << std::endl;
        #endif

        ProcessObjects(fobs, sharedBalls, time_increment);
    #endif
}

void Localisation::ProcessObjects(FieldObjects* fobs, const vector<TeamPacket::SharedBall>& sharedballs, float time_increment)
{
    int numUpdates = 0;
    int updateResult;
    int usefulObjectCount = 0;

    //if(balanceFallen) return;
// 	debug_out  << "Dont put anything "<<endl;
#if DEBUG_LOCALISATION_VERBOSITY > 2
    if(numUpdates == 0 )
    {
        debug_out  <<"[" << m_timestamp << "]: Update Starting." << endl;
        for(int i = 0; i < c_MAX_MODELS; i++){
            if(m_models[i].active() == false) continue;
            debug_out  << "[" << m_timestamp << "]: Model[" << i << "]";
            debug_out  << " [alpha = " << m_models[i].alpha() << "]";
            debug_out  << " Robot X: " << m_models[i].state(KF::selfX);
            debug_out  << " Robot Y: " << m_models[i].state(KF::selfY);
            debug_out  << " Robot Theta: " << m_models[i].state(KF::selfTheta) << endl;
        }
    }
#endif // DEBUG_LOCALISATION_VERBOSITY > 2
	
	
    // Correct orientation to face a goal if you can see it and are unsure which way you are facing.
    //varianceCheckAll();

    // Proccess the Stationary Known Field Objects
    StationaryObjectsIt currStat(fobs->stationaryFieldObjects.begin());
    StationaryObjectsConstIt endStat(fobs->stationaryFieldObjects.end());

    for(; currStat != endStat; ++currStat)
    {
        if(currStat->isObjectVisible() == false) continue; // Skip objects that were not seen.
        updateResult = doKnownLandmarkMeasurementUpdate((*currStat));
        numUpdates++;
        usefulObjectCount++;
    }

    // Two Object update
#if TWO_OBJECT_UPDATE_ON
    if( fobs->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST].isObjectVisible()
        && fobs->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST].isObjectVisible())
        {
            doTwoObjectUpdate(fobs->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST],
                              fobs->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST]);
        }
    if( fobs->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST].isObjectVisible()
        && fobs->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST].isObjectVisible())
        {
            doTwoObjectUpdate(fobs->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST],
                              fobs->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST]);
        }
#endif


    // Proccess the Moving Known Field Objects
    MobileObjectsIt currMob(fobs->mobileFieldObjects.begin());
    MobileObjectsConstIt endMob(fobs->mobileFieldObjects.end());

    for (; currMob != endMob; ++currMob)
    {
        if(currMob->isObjectVisible() == false) continue; // Skip objects that were not seen.
        updateResult = doBallMeasurementUpdate((*currMob));
        numUpdates++;
        usefulObjectCount++;
    }
    NormaliseAlphas();

#if SHARED_BALL_ON
    // Check the game packets.
    // We only want to do the shared ball updates if we can't see the ball ourselves.
    // there have been probems where the team will keep sharing the previous position of their ball
    // and updates in vision do not supercede the shared data.
    if(fobs->mobileFieldObjects[FieldObjects::FO_BALL].TimeSinceLastSeen() > 250)    // TODO: Change to a SD value
    { 
        for (size_t i=0; i<sharedballs.size(); i++)
        {
            doSharedBallUpdate(sharedballs[i]);
            if (sharedballs[i].TimeSinceLastSeen < 500)    // if another robot can see the ball then it is not lost
                fobs->mobileFieldObjects[FieldObjects::FO_BALL].updateIsLost(false);
        }
    }
#endif // SHARED_BALL_ON

        NormaliseAlphas();

#if MULTIPLE_MODELS_ON
        // Do Ambiguous objects.
        AmbiguousObjectsIt currAmb(fobs->ambiguousFieldObjects.begin());
        AmbiguousObjectsConstIt endAmb(fobs->ambiguousFieldObjects.end());
        for(; currAmb != endAmb; ++currAmb){
            if(currAmb->isObjectVisible() == false) continue; // Skip objects that were not seen.
            updateResult = doAmbiguousLandmarkMeasurementUpdate((*currAmb), fobs->stationaryFieldObjects);
            NormaliseAlphas();
            numUpdates++;
            if(currAmb->getID() == FieldObjects::FO_BLUE_GOALPOST_UNKNOWN or currAmb->getID() == FieldObjects::FO_YELLOW_GOALPOST_UNKNOWN)
                usefulObjectCount++;
        }

        MergeModels(c_MAX_MODELS_AFTER_MERGE);
#endif // MULTIPLE_MODELS_ON

#if DEBUG_LOCALISATION_VERBOSITY > 1
        for (int currID = 0; currID < c_MAX_MODELS; currID++){
            if(m_models[currID].active())
            {

                debug_out   <<"Model : "<<currID<<" Pos  : "<<m_models[currID].stateEstimates[0][0]<<", "
                            <<m_models[currID].stateEstimates[0][1]<<","<< m_models[currID].stateEstimates[0][2]<<endl;
            }
        }
#endif // DEBUG_LOCALISATION_VERBOSITY > 0
    
    
        if (usefulObjectCount > 0)
            timeSinceFieldObjectSeen = 0;
        else
            timeSinceFieldObjectSeen += time_increment;

        // Check for model reset. -> with multiple models just remove if not last one??
        // Need to re-do reset to be model specific.
        //int numReset = CheckForOutlierResets();
        CheckForOutlierResets();

        // clip models back on to field.
        clipActiveModelsToField();

        // Store WM Data in Field Objects.
        //int bestModelID = getBestModelID();
        // Get the best model to use.
        WriteModelToObjects(getBestModel(), fobs);

#if DEBUG_LOCALISATION_VERBOSITY > 2
        const KF* bestModel = &(getBestModel());
        if(numUpdates > 0)
        {
            for (int i = 0; i < c_MAX_MODELS; i++){
                if(m_models[i].active() == false) continue;
                debug_out  << "[" << m_timestamp << "]: Model[" << i << "]";
                debug_out  << " [alpha = " << m_models[i].alpha() << "]";
                debug_out  << " Robot X: " << m_models[i].state(0);
                debug_out  << " Robot Y: " << m_models[i].state(1);
                debug_out  << " Robot Theta: " << m_models[i].state(2) << endl;
            }
            debug_out  << "[" << m_timestamp << "]: Best Model";
            debug_out  << " [alpha = " << bestModel->alpha() << "]";
            debug_out  << " Robot X: " << bestModel->state(0);
            debug_out  << " Robot Y: " << bestModel->state(1);
            debug_out  << " Robot Theta: " << bestModel->state(2) << endl;
        }
#endif // DEBUG_LOCALISATION_VERBOSITY > 2	
}

void Localisation::WriteModelToObjects(const KF &model, FieldObjects* fieldObjects)
{

    // Write stationary objects.
    /*
    StationaryObjectsIt currStat = fieldObjects->stationaryFieldObjects.begin();
    StationaryObjectsConstIt endStat = fieldObjects->stationaryFieldObjects.end();
    float x,y;
    float distance,bearing;
    while(currStat != endStat)
    {
        x = (*currStat).X();
        y = (*currStat).Y();
        distance = model.getDistanceToPosition(x,y);
        bearing = model.getBearingToPosition(x,y);
        (*currStat).updateEstimatedRelativeVariables(distance, bearing, 0.0f);
        ++currStat;
    }
    */

    // Set the balls location.
    float distance,bearing;
    distance = model.getDistanceToPosition(model.state(KF::ballX), model.state(KF::ballY));
    bearing = model.getBearingToPosition(model.state(KF::ballX), model.state(KF::ballY));
    fieldObjects->mobileFieldObjects[fieldObjects->FO_BALL].updateObjectLocation(model.state(KF::ballX),model.state(KF::ballY),model.sd(KF::ballX), model.sd(KF::ballY));
    fieldObjects->mobileFieldObjects[fieldObjects->FO_BALL].updateObjectVelocities(model.state(KF::ballXVelocity),model.state(KF::ballYVelocity),model.sd(KF::ballXVelocity), model.sd(KF::ballYVelocity));
    fieldObjects->mobileFieldObjects[fieldObjects->FO_BALL].updateEstimatedRelativeVariables(distance, bearing, 0.0f);
    fieldObjects->mobileFieldObjects[fieldObjects->FO_BALL].updateSharedCovariance(model.GetBallSR());

	bool lost = false;
	if (lostCount > 20 or timeSinceFieldObjectSeen > 15000)
		lost = true;

    // Set my location.
    fieldObjects->self.updateLocationOfSelf(model.state(KF::selfX), model.state(KF::selfY), model.state(KF::selfTheta), model.sd(KF::selfX), model.sd(KF::selfY), model.sd(KF::selfTheta),
											lost);
}

bool Localisation::CheckGameState(bool currently_incapacitated, const GameInformation* game_info)
{
    GameInformation::TeamColour team_colour = game_info->getTeamColour();
    GameInformation::RobotState current_state = game_info->getCurrentState();
    if (currently_incapacitated)
    {   // if the robot is incapacitated there is no point running localisation
        m_previous_game_state = current_state;
        m_previously_incapacitated = true;
        #if LOC_SUMMARY > 0
        m_frame_log << "Robot is incapscitated." << std::endl;
        #endif
        return false;
    }
    
    if (current_state == GameInformation::InitialState or current_state == GameInformation::FinishedState or current_state == GameInformation::PenalisedState or current_state == GameInformation::SubstituteState)
    {   // if we are in initial, finished, penalised or substitute states do not do localisation
        m_previous_game_state = current_state;
        m_previously_incapacitated = currently_incapacitated;
        #if LOC_SUMMARY > 0
        m_frame_log << "Robot in non-processing state: " << GameInformation::stateName(current_state) << std::endl;
        #endif
        return false;
    }
    
    if (current_state == GameInformation::ReadyState)
    {   // if are in ready. If previously in initial or penalised do a reset. Also reset if fallen over.
        if (m_previous_game_state == GameInformation::InitialState)
            doInitialReset(team_colour);
        else if (m_previous_game_state == GameInformation::PenalisedState)
            doPenaltyReset();
        else if (m_previously_incapacitated and not currently_incapacitated)
            doFallenReset();
    }
    else if (current_state == GameInformation::SetState)
    {   // if we are in set look for manual placement, if detected then do a reset.
        if (m_previously_incapacitated and not currently_incapacitated)
            doSetReset(team_colour, game_info->getPlayerNumber(), game_info->haveKickoff());
    }
    else
    {   // if we are playing. If previously penalised do a reset. Also reset if fallen over
        if (m_previous_game_state == GameInformation::PenalisedState)
            doPenaltyReset();
        else if (m_previously_incapacitated and not currently_incapacitated)
            doFallenReset();
    }
    
    m_previously_incapacitated = currently_incapacitated;
    m_previous_game_state = current_state;
    return true;
}

void Localisation::ClearAllModels()
{
    // Reset all of the models
    for(int m = 0; m < c_MAX_MODELS; m++){
        // reset outlier error count
        for (int i=0; i<c_numOutlierTrackedObjects; i++) m_modelObjectErrors[m][i] = 0.0;

        // Disable model
        m_models[m].setActive(false);
        m_models[m].m_toBeActivated = false;
    }
    return;
}

void Localisation::initSingleModel(float x, float y, float theta)
{
#if DEBUG_LOCALISATION_VERBOSITY > 0
    debug_out  << "Initialising single model." << endl;
#endif // DEBUG_LOCALISATION_VERBOSITY > 0
    ClearAllModels();
    setupModel(0,1,x,y,theta);
}

void Localisation::doInitialReset(GameInformation::TeamColour team_colour)
{
    #if LOC_SUMMARY > 0
    m_frame_log << "Reset leaving initial." << std::endl;
    #endif
#if DEBUG_LOCALISATION_VERBOSITY > 0
    debug_out  << "Performing initial->ready reset." << endl;
#endif // DEBUG_LOCALISATION_VERBOSITY > 0
    
    ClearAllModels();
    
    // The models are always in the team's own half
    // Model 0: On left sideline facing in, 1/3 from half way
    // Model 1: On left sideline facing in, 2/3 from half way
    // Model 2: On right sideline facing in, 1/3 from half way
    // Model 3: On right sideline facing in, 2/3 from half way
    // Model 4: In centre of own half facing opponents goal
    const int num_models = 5;
	 
    float left_y = 200.0;
    float left_heading = -PI/2;
    float right_y = -200.0;
    float right_heading = PI/2;
    
    float front_x = -300.0/4;
    float centre_x = -300.0/2;
    float centre_heading = 0;
    float back_x = -300*(3.0f/4);
    if (team_colour == GameInformation::RedTeam)
    {   // flip the invert the x for the red team and set the heading to PI
        front_x = -front_x;
        centre_x = -centre_x;
        centre_heading = PI;
        back_x = -back_x;
    }
    
    setupModel(0, num_models, front_x, right_y, right_heading);
    setupModelSd(0, 50, 15, 0.2);
    setupModel(1, num_models, back_x, right_y, right_heading);
    setupModelSd(1, 50, 15, 0.2);
    setupModel(2, num_models, front_x, left_y, left_heading);
    setupModelSd(2, 50, 15, 0.2);
    setupModel(3, num_models, back_x, left_y, left_heading);
    setupModelSd(3, 50, 15, 0.2);
    setupModel(4, num_models, centre_x, 0, centre_heading);
    setupModelSd(4, 100, 150, PI/2);
    
    return;
}

void Localisation::doSetReset(GameInformation::TeamColour team_colour, int player_number, bool have_kickoff)
{
    #if LOC_SUMMARY > 0
    m_frame_log << "Reset due to manual positioning." << std::endl;
    #endif
#if DEBUG_LOCALISATION_VERBOSITY > 0
    debug_out  << "Performing manual position reset." << endl;
#endif // DEBUG_LOCALISATION_VERBOSITY > 0
    ClearAllModels();
    float x, y, heading;
    const float position_sd = 15;
    const float heading_sd = 0.1;
    int num_models;
    if (player_number == 1)
    {   // if we are the goal keeper and we get manually positioned we know exactly where we will be put
        num_models = 1;
        x = -300;
        y = 0; 
        heading = 0;
        if (team_colour == GameInformation::RedTeam)
            swapFieldStateTeam(x, y, heading);
        setupModel(0, num_models, x, y, heading);
    }
    else
    {   // if we are a field player and we get manually positioned we could be in a three different places
        if (have_kickoff)
        {   // the attacking positions are on the circle or on either side of the penalty spot
            num_models = 3;
            // on the circle
            x = -60;
            y = 0;
            heading = 0;
            if (team_colour == GameInformation::RedTeam)
                swapFieldStateTeam(x, y, heading);
            setupModel(0, num_models, x, y, heading);
            // on the left of the penalty spot
            x = -120;
            y = 70;
            heading = 0;
            if (team_colour == GameInformation::RedTeam)
                swapFieldStateTeam(x, y, heading);
            setupModel(1, num_models, x, y, heading);
            // on the right of the penalty spot
            x = -120;
            y = -70;
            heading = 0;
            if (team_colour == GameInformation::RedTeam)
                swapFieldStateTeam(x, y, heading);
            setupModel(2, num_models, x, y, heading);
            
        }
        else
        {   // the defensive positions are on either corner of the penalty box
            num_models = 2;
            // top penalty box corner
            x = -240;
            y = 110;
            heading = 0;
            if (team_colour == GameInformation::RedTeam)
                swapFieldStateTeam(x, y, heading);
            setupModel(0, num_models, x, y, heading);
            // bottom penalty box corner
            x = -240;
            y = -110;
            heading = 0;
            if (team_colour == GameInformation::RedTeam)
                swapFieldStateTeam(x, y, heading);
            setupModel(1, num_models, x, y, heading);
        }
    }
    
    // setup the standard deviations
    for (int i=0; i<num_models; i++)
        setupModelSd(i, position_sd, position_sd, heading_sd);
}

void Localisation::doPenaltyReset()
{
    #if LOC_SUMMARY > 0
    m_frame_log << "Reset due to penalty." << std::endl;
    #endif
#if DEBUG_LOCALISATION_VERBOSITY > 0
    debug_out  << "Performing penalty reset." << endl;
#endif // DEBUG_LOCALISATION_VERBOSITY > 0

    ClearAllModels();

    const int num_models = 2;
    // setup model 0 as top 'T'
    setupModel(0, num_models, 0, 200, -PI/2.0);
    setupModelSd(0, 75, 25, 0.35);
    
    // setup model 1 as bottom 'T'
    setupModel(1, num_models, 0, -200, PI/2.0);
    setupModelSd(0, 75, 25, 0.35);
    return;
}

void Localisation::doFallenReset()
{
    #if LOC_SUMMARY > 0
    m_frame_log << "Reset due to fall." << std::endl;
    #endif
#if DEBUG_LOCALISATION_VERBOSITY > 0
    debug_out  << "Performing fallen reset." << endl;
#endif // DEBUG_LOCALISATION_VERBOSITY > 0
    for (int modelNumber = 0; modelNumber < c_MAX_MODELS; modelNumber++)
    {   // Increase heading uncertainty if fallen
        m_models[modelNumber].stateStandardDeviations[0][0] += 15;        // Robot x
        m_models[modelNumber].stateStandardDeviations[1][1] += 15;        // Robot y
        m_models[modelNumber].stateStandardDeviations[2][2] += 0.707;     // Robot heading
    }
}

void Localisation::doReset()
{
    #if DEBUG_LOCALISATION_VERBOSITY > 0
    debug_out  << "Performing player reset." << endl;
    #endif // DEBUG_LOCALISATION_VERBOSITY > 0

    ClearAllModels();

    // setup model 0 as in yellow goals
    m_models[0].setActive(true);
    m_models[0].setAlpha(0.25);

    m_models[0].stateEstimates[0][0] = 300.0;         // Robot x
    m_models[0].stateEstimates[1][0] = 0.0;           // Robot y
    m_models[0].stateEstimates[2][0] = PI;        // Robot heading
    m_models[0].stateEstimates[3][0] = 0.0;           // Ball x
    m_models[0].stateEstimates[4][0] = 0.0;           // Ball y
    m_models[0].stateEstimates[5][0] = 0.0;           // Ball vx
    m_models[0].stateEstimates[6][0] = 0.0;           // Ball vy

    // Set the uncertainties
    resetSdMatrix(0);

    // setup model 1 as in blue goals
    m_models[1].setActive(true);
    m_models[1].setAlpha(0.25);

    m_models[1].stateEstimates[0][0] = -300.0;        // Robot x
    m_models[1].stateEstimates[1][0] = 0.0;           // Robot y
    m_models[1].stateEstimates[2][0] = 0.0;           // Robot heading
    m_models[1].stateEstimates[3][0] = 0.0;       // Ball x
    m_models[1].stateEstimates[4][0] = 0.0;       // Ball y
    m_models[1].stateEstimates[5][0] = 0.0;       // Ball vx
    m_models[1].stateEstimates[6][0] = 0.0;       // Ball vy

    // Set the uncertainties
    resetSdMatrix(1);

    // setup model 2 as top half way 'T'
    m_models[2].setActive(true);
    m_models[2].setAlpha(0.25);

    m_models[2].stateEstimates[0][0] = 0.0;        // Robot x
    m_models[2].stateEstimates[1][0] = 200.0;           // Robot y
    m_models[2].stateEstimates[2][0] = -PI/2.0;           // Robot heading
    m_models[2].stateEstimates[3][0] = 0.0;       // Ball x
    m_models[2].stateEstimates[4][0] = 0.0;       // Ball y
    m_models[2].stateEstimates[5][0] = 0.0;       // Ball vx
    m_models[2].stateEstimates[6][0] = 0.0;       // Ball vy

    // Set the uncertainties
    resetSdMatrix(2);

    // setup model 3 as other half way 'T'
    m_models[3].setActive(true);
    m_models[3].setAlpha(0.25);

    m_models[3].stateEstimates[0][0] = 0.0;        // Robot x
    m_models[3].stateEstimates[1][0] = -200.0;           // Robot y
    m_models[3].stateEstimates[2][0] = PI/2.0;           // Robot heading
    m_models[3].stateEstimates[3][0] = 0.0;       // Ball x
    m_models[3].stateEstimates[4][0] = 0.0;       // Ball y
    m_models[3].stateEstimates[5][0] = 0.0;       // Ball vx
    m_models[3].stateEstimates[6][0] = 0.0;       // Ball vy

    // Set the uncertainties
    resetSdMatrix(3);
    return;
}

void Localisation::doBallOutReset()
{
#if DEBUG_LOCALISATION_VERBOSITY > 0
    debug_out  << "Performing ball out reset." << endl;
#endif // DEBUG_LOCALISATION_VERBOSITY > 0
    // Increase uncertainty of ball position if it has gone out.. Cause it has probably been moved.
    for (int modelNumber = 0; modelNumber < c_MAX_MODELS; modelNumber++){
        if(m_models[modelNumber].active() == false) continue;
        m_models[modelNumber].stateStandardDeviations[3][3] += 100.0; // 100 cm
        m_models[modelNumber].stateStandardDeviations[4][4] += 60.0; // 60 cm
        m_models[modelNumber].stateStandardDeviations[5][5] += 10.0;   // 10 cm/s
        m_models[modelNumber].stateStandardDeviations[6][6] += 10.0;   // 10 cm/s
    }
    return;
}

/*! @brief Setup model modelNumber with the given x, y and heading */
void Localisation::setupModel(int modelNumber, int numModels, float x, float y, float heading)
{
    m_models[modelNumber].setActive(true);
    m_models[modelNumber].setAlpha(1.0f/numModels);
    
    m_models[modelNumber].stateEstimates[0][0] = x;             // Robot x
    m_models[modelNumber].stateEstimates[1][0] = y;             // Robot y
    m_models[modelNumber].stateEstimates[2][0] = heading;       // Robot heading
    m_models[modelNumber].stateEstimates[3][0] = 0.0;           // Ball x
    m_models[modelNumber].stateEstimates[4][0] = 0.0;           // Ball y
    m_models[modelNumber].stateEstimates[5][0] = 0.0;           // Ball vx
    m_models[modelNumber].stateEstimates[6][0] = 0.0;           // Ball vy
}

/*! @brief Setup a model's standard deviations with the given sdx, sdy, sdheading */
void Localisation::setupModelSd(int modelNumber, float sdx, float sdy, float sdheading)
{
    m_models[modelNumber].stateStandardDeviations[0][0] = sdx;        // Robot x
    m_models[modelNumber].stateStandardDeviations[1][1] = sdy;        // Robot y
    m_models[modelNumber].stateStandardDeviations[2][2] = sdheading;  // Robot heading
    m_models[modelNumber].stateStandardDeviations[3][3] = 150.0;      // Ball x
    m_models[modelNumber].stateStandardDeviations[4][4] = 100.0;      // Ball y
    m_models[modelNumber].stateStandardDeviations[5][5] = 10.0;       // Ball velocity x
    m_models[modelNumber].stateStandardDeviations[6][6] = 10.0;       // Ball velocity y
}

void Localisation::resetSdMatrix(int modelNumber)
{
     // Set the uncertainties
     m_models[modelNumber].stateStandardDeviations[0][0] = 150.0; // 150 cm
     m_models[modelNumber].stateStandardDeviations[1][1] = 100.0; // 100 cm
     m_models[modelNumber].stateStandardDeviations[2][2] = PI;   // 2 radians
     m_models[modelNumber].stateStandardDeviations[3][3] = 150.0; // 150 cm
     m_models[modelNumber].stateStandardDeviations[4][4] = 100.0; // 100 cm
     m_models[modelNumber].stateStandardDeviations[5][5] = 10.0;   // 10 cm/s
     m_models[modelNumber].stateStandardDeviations[6][6] = 10.0;   // 10 cm/s

    
//    models[modelNumber].stateStandardDeviations[0][0] = 10.0; // 100 cm
//    models[modelNumber].stateStandardDeviations[1][1] = 10.0; // 150 cm
//    models[modelNumber].stateStandardDeviations[2][2] = 0.2;   // 2 radians
//    models[modelNumber].stateStandardDeviations[3][3] = 10.0; // 100 cm
//    models[modelNumber].stateStandardDeviations[4][4] = 10.0; // 150 cm
//    models[modelNumber].stateStandardDeviations[5][5] = 1.0;   // 10 cm/s
//    models[modelNumber].stateStandardDeviations[6][6] = 1.0;   // 10 cm/s
    
    
    return;  
}

/*! @brief Changes the field state so that it will be the position for the other team */
void Localisation::swapFieldStateTeam(float& x, float& y, float& heading)
{
    x = -x;
    y = -y;
    heading = normaliseAngle(heading + PI);
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
    prevX = m_models[modelID].state(0);
    prevY = m_models[modelID].state(1);
    prevTheta = m_models[modelID].state(2);

    clipped = m_models[modelID].clipState(0, fieldXMin, fieldXMax);		// Clipping for robot's X
    #if DEBUG_LOCALISATION_VERBOSITY > 1
    if(clipped){
        debug_out  << "[" << m_timestamp << "]: Model[" << modelID << "]";
        debug_out  << " [alpha = " << m_models[modelID].alpha() << "]";
        debug_out  << " State(0) clipped.";
        debug_out  << " (" << prevX << "," << prevY << "," << prevTheta << ") -> (" << m_models[modelID].state(0);
        debug_out  << "," << m_models[modelID].state(1) << "," << m_models[modelID].state(2) << ")" << endl;
    }
    #endif // DEBUG_LOCALISATION_VERBOSITY > 1
    wasClipped = wasClipped || clipped;

    prevX = m_models[modelID].state(0);
    prevY = m_models[modelID].state(1);
    prevTheta = m_models[modelID].state(2);

    clipped = m_models[modelID].clipState(1, fieldYMin, fieldYMax);		// Clipping for robot's Y

    #if DEBUG_LOCALISATION_VERBOSITY > 1
    if(clipped){
        debug_out  << "[" << m_timestamp << "]: Model[" << modelID << "]";
        debug_out  << " [alpha = " << m_models[modelID].alpha() << "]";
        debug_out  << " State(1) clipped." << endl;
        debug_out  << " (" << prevX << "," << prevY << "," << prevTheta << ") -> (" << m_models[modelID].state(0);
        debug_out  << "," << m_models[modelID].state(1) << "," << m_models[modelID].state(2) << ")" << endl;
    }
    #endif // DEBUG_LOCALISATION_VERBOSITY > 1

    wasClipped = wasClipped || clipped;
    prevX = m_models[modelID].state(0);
    prevY = m_models[modelID].state(1);
    prevTheta = m_models[modelID].state(2);

    clipped = m_models[modelID].clipState(3, fieldXMin, fieldXMax);		// Clipping for ball's X

    #if DEBUG_LOCALISATION_VERBOSITY > 1
    if(clipped){
        debug_out  << "[" << m_timestamp << "]: Model[" << modelID << "]";
        debug_out  << " [alpha = " << m_models[modelID].alpha() << "]";
        debug_out  << " State(3) clipped." << endl;
        debug_out  << " (" << prevX << "," << prevY << "," << prevTheta << ") -> (" << m_models[modelID].state(0);
        debug_out  << "," << m_models[modelID].state(1) << "," << m_models[modelID].state(2) << ")" << endl;
    }
    #endif // DEBUG_LOCALISATION_VERBOSITY > 1
    
    wasClipped = wasClipped || clipped;

    prevX = m_models[modelID].state(0);
    prevY = m_models[modelID].state(1);
    prevTheta = m_models[modelID].state(2);

    clipped = m_models[modelID].clipState(4, fieldYMin, fieldYMax);		// Clipping for ball's Y

    #if DEBUG_LOCALISATION_VERBOSITY > 1
    if(clipped){
        debug_out  << "[" << m_timestamp << "]: Model[" << modelID << "]";
        debug_out  << " [alpha = " << m_models[modelID].alpha() << "]";
        debug_out  << " State(4) clipped." << endl;
        debug_out  << " (" << prevX << "," << prevY << "," << prevTheta << ") -> (" << m_models[modelID].state(0);
        debug_out  << "," << m_models[modelID].state(1) << "," << m_models[modelID].state(2) << ")" << endl;
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
        if(m_models[modelID].active() == true){
            modelClipped = clipModelToField(modelID);
            wasClipped = wasClipped || modelClipped;
        }
    }
    return wasClipped;
}

bool Localisation::doTimeUpdate(float odomForward, float odomLeft, float odomTurn, double timeIncrement)
{
    bool result = false;
    for(int modelID = 0; modelID < c_MAX_MODELS; modelID++)
    {
        if(m_models[modelID].active() == false) continue; // Skip Inactive models.
        result = true;
        m_models[modelID].timeUpdate(timeIncrement);
        m_models[modelID].performFiltering(odomForward, odomLeft, odomTurn);
    }
    
	//------------------------- Trial code for entropy ---- Made to work only on webots as of now
	int bestIndex = getBestModelID();
	double rmsDistance = 0;
	double entropy = 0;
	double bestModelEntropy = 0;
	
	for(int modelID = 0; modelID < c_MAX_MODELS; modelID++)
    {
        if(m_models[modelID].active() == false) continue; // Skip Inactive models.
        
		rmsDistance = pow (
                                                   pow((m_models[bestIndex].stateEstimates[0][0] - m_models[modelID].stateEstimates[0][0]),2) +
                                               pow((m_models[bestIndex].stateEstimates[1][0] - m_models[modelID].stateEstimates[1][0]),2) +
                                                   pow((m_models[bestIndex].stateEstimates[2][0] - m_models[modelID].stateEstimates[2][0]),2) , 0.5 );
                entropy += (rmsDistance * m_models[modelID].alpha());
		 
    }
	
	Matrix bestModelCovariance(3,3,false);
	
	
	for(int i =0 ; i < 3 ; i++)
	{
		for(int j = 0 ; j < 3 ; j++ )
		{
                        bestModelCovariance[i][j] = m_models[bestIndex].stateStandardDeviations[i][j]  ;
		}
	}
	
    bestModelCovariance = bestModelCovariance * bestModelCovariance.transp();							   
    bestModelEntropy =  0.5 * ( 3 + 3*log(2 * PI ) + log(  determinant(bestModelCovariance) ) ) ;
	
    if(entropy >55 && m_models[bestIndex].alpha()<50 )
        amILost = true;
	else if (entropy <=55 && bestModelEntropy > 6.5)
        amILost = true;
	else 
		amILost = false;
    
    if(	amILost )
		lostCount++;
	else 
		lostCount = 0;		
	// End ------------------------- Trial code for entropy ---- Made to work only on webots as of now

	return result;
}

int Localisation::doSharedBallUpdate(const TeamPacket::SharedBall& sharedBall)
{
    int kf_return;
    int numSuccessfulUpdates = 0;
    float timeSinceSeen = sharedBall.TimeSinceLastSeen;
    double sharedBallX = sharedBall.X;
    double sharedBallY = sharedBall.Y;
    double SRXX = sharedBall.SRXX;
    double SRXY = sharedBall.SRXY;
    double SRYY = sharedBall.SRYY;
    
    if (timeSinceSeen > 0)      // don't process sharedBalls unless they are seen
        return 0;
    
    #if DEBUG_LOCALISATION_VERBOSITY > 2
        debug_out  << "[" << m_timestamp << "]: Doing Shared Ball Update. X = " << sharedBallX << " Y = " << sharedBallY << " SRXX = " << SRXX << " SRXY = " << SRXY << "SRYY = " << SRYY << endl;
    #endif

    for(int modelID = 0; modelID < c_MAX_MODELS; modelID++){
        if(m_models[modelID].active() == false) continue; // Skip Inactive models.
        kf_return = KF_OK;
        m_models[modelID].linear2MeasurementUpdate(sharedBallX, sharedBallY, SRXX, SRXY, SRYY, 3, 4);
        if(kf_return == KF_OK) numSuccessfulUpdates++;
    }
    return numSuccessfulUpdates;
}

int Localisation::doBallMeasurementUpdate(MobileObject &ball)
{
    int kf_return;
    int numSuccessfulUpdates = 0;

    if(IsValidObject(ball) == false)
    {
    #if DEBUG_LOCALISATION_VERBOSITY > 0
        debug_out  <<"[" << m_timestamp << "]: Skipping Invalid Ball Update. Distance = " << ball.measuredDistance() << " Bearing = " << ball.measuredBearing() << endl;
    #endif // DEBUG_LOCALISATION_VERBOSITY > 1
        return KF_OUTLIER;
    }

    #if DEBUG_LOCALISATION_VERBOSITY > 2
    debug_out  <<"[" << m_timestamp << "]: Doing Ball Update. Distance = " << ball.measuredDistance() << " Bearing = " << ball.measuredBearing() << endl;
    #endif // DEBUG_LOCALISATION_VERBOSITY > 1

    double flatBallDistance = ball.measuredDistance() * cos(ball.measuredElevation());
    for(int modelID = 0; modelID < c_MAX_MODELS; modelID++){
        if(m_models[modelID].active() == false) continue; // Skip Inactive models.
        kf_return = KF_OK;
        kf_return = m_models[modelID].ballmeas(flatBallDistance, ball.measuredBearing());
        if(kf_return == KF_OK) numSuccessfulUpdates++;
    }
    return numSuccessfulUpdates;
}

int Localisation::doKnownLandmarkMeasurementUpdate(StationaryObject &landmark)
{
    int kf_return;
    int numSuccessfulUpdates = 0;

    if(IsValidObject(landmark) == false)
    {
#if DEBUG_LOCALISATION_VERBOSITY > 0
        debug_out  <<"[" << m_timestamp << "] Skipping Bad Landmark Update: ";
        debug_out  << landmark.getName();
        debug_out  << " Distance = " << landmark.measuredDistance();
        debug_out  << " Bearing = " << landmark.measuredBearing();
        debug_out  << endl;
#endif // DEBUG_LOCALISATION_VERBOSITY > 1
        return KF_OUTLIER;
    }

    int objID = landmark.getID();
    double flatObjectDistance = landmark.measuredDistance() * cos(landmark.measuredElevation());
    //double flatObjectDistance = landmark.measuredDistance();

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
        if(m_models[modelID].active() == false) continue; // Skip Inactive models.

#if DEBUG_LOCALISATION_VERBOSITY > 2
        debug_out  <<"[" << m_timestamp << "]: Model[" << modelID << "] Landmark Update. ";
        debug_out  << "Object = " << landmark.getName();
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
        kf_return = m_models[modelID].fieldObjectmeas(flatObjectDistance, landmark.measuredBearing(),landmark.X(), landmark.Y(),
			distanceOffsetError, distanceRelativeError, bearingError);
        if(kf_return == KF_OUTLIER) m_modelObjectErrors[modelID][landmark.getID()] += 1.0;

#if DEBUG_LOCALISATION_VERBOSITY > 0
        if(kf_return != KF_OK)
        {
            debug_out << "OUTLIER!" << endl;
            debug_out << "Model[" << modelID << "]: Outlier Detected - " << landmark.getName() << endl;
            debug_out << "Measured - Distance = " << landmark.measuredDistance() << " Bearing = " << landmark.measuredBearing() << endl;
            debug_out << "Expected - Distance = " << m_models[modelID].getDistanceToPosition(landmark.X(),landmark.Y()) << " Bearing = " << m_models[modelID].getBearingToPosition(landmark.X(),landmark.Y()) << endl;
        }
        else
        {
            debug_out << "OK!" << endl;
        }
#endif // DEBUG_LOCALISATION_VERBOSITY > 1

        if(kf_return == KF_OK) numSuccessfulUpdates++;
    }
    return numSuccessfulUpdates;
}

int Localisation::doTwoObjectUpdate(StationaryObject &landmark1, StationaryObject &landmark2)
{
    float totalAngle = landmark1.measuredBearing() - landmark2.measuredBearing();

    #if DEBUG_LOCALISATION_VERBOSITY > 0
    debug_out << "Two Object Update:" << endl;
    debug_out << landmark1.getName() << " - Bearing = " << landmark1.measuredBearing() << endl;
    debug_out << landmark2.getName() << " - Bearing = " << landmark2.measuredBearing() << endl;
    #endif
    for (int currID = 0; currID < c_MAX_MODELS; currID++){
        if(m_models[currID].active())
        {
            m_models[currID].updateAngleBetween(totalAngle,landmark1.X(),landmark1.Y(),landmark2.X(),landmark2.Y(),sdTwoObjectAngle);
        }
    }
    return 1;
}

int Localisation::doAmbiguousLandmarkMeasurementUpdate(AmbiguousObject &ambigousObject, const vector<StationaryObject>& possibleObjects)
{
    int kf_return;

    if(IsValidObject(ambigousObject) == false)
    {
    #if DEBUG_LOCALISATION_VERBOSITY > 1
        debug_out  <<"[" << m_timestamp << "] Skipping Ambiguous Object: ";
        debug_out  << ambigousObject.getName();
        debug_out  << " Distance = " << ambigousObject.measuredDistance();
        debug_out  << " Bearing = " << ambigousObject.measuredBearing();
    #endif // DEBUG_LOCALISATION_VERBOSITY > 1
        return KF_OUTLIER;
    }

    #if AMBIGUOUS_CORNERS_ON <= 0
    if((ambigousObject.getID() != FieldObjects::FO_BLUE_GOALPOST_UNKNOWN) && (ambigousObject.getID() != FieldObjects::FO_YELLOW_GOALPOST_UNKNOWN)){
    #if DEBUG_LOCALISATION_VERBOSITY > 1
        debug_out  <<"[" << m_timestamp << "]: ingored unkown object " << ambigousObject.getName() << std::endl;
    #endif // DEBUG_LOCALISATION_VERBOSITY > 1
        return KF_OUTLIER;
    }
    #endif // AMBIGUOUS_CORNERS_ON <= 0

    vector<int> possabilities = ambigousObject.getPossibleObjectIDs();
    unsigned int numOptions = possabilities.size();
    int outlierModelID = -1;
    int numFreeModels = getNumFreeModels();
    int numActiveModels = getNumActiveModels();
    int numRequiredModels = numActiveModels * (numOptions); // An extra base model.

    if(numFreeModels < numRequiredModels){
        int maxActiveAfterMerge = c_MAX_MODELS /  (numOptions + 1);

        #if DEBUG_LOCALISATION_VERBOSITY > 2
        debug_out  <<"[" << m_timestamp << "]: Only " <<  numFreeModels << " Free. Need " << numRequiredModels << " for Update." << endl;
        debug_out  <<"[" << m_timestamp << "]: Merging to " << maxActiveAfterMerge << " Max models." << endl;
        #endif // DEBUG_LOCALISATION_VERBOSITY > 2

        MergeModels(maxActiveAfterMerge);

        #if DEBUG_LOCALISATION_VERBOSITY > 2
        debug_out  <<"[" << m_timestamp << "]: " << getNumFreeModels() << " models now available." << endl;
        #endif // DEBUG_LOCALISATION_VERBOSITY > 2

        if(getNumFreeModels() < (getNumActiveModels() * (int)numOptions)){

            #if DEBUG_LOCALISATION_VERBOSITY > 0
            debug_out  <<"[" << m_timestamp << "]: " << "Not enough models. Aborting Update." << endl;
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
        if(m_models[modelID].active() == false) continue; // Skip inactive models.

        // Copy initial model to the temporary model.
        m_tempModel = m_models[modelID];
        m_tempModel.setActive(false);
        m_tempModel.m_toBeActivated = true;
        
        // Save Original model as outlier option.
        m_models[modelID].setAlpha(m_models[modelID].alpha()*0.0005);
        outlierModelID = -1;
//        modelObjectErrors[modelID][ambigousObject.getID()] += 1.0;
  
        // Now go through each of the possible options, and apply it to a copy of the model
        for(unsigned int optionNumber = 0; optionNumber < numOptions; optionNumber++){
            int possibleObjectID = possabilities[optionNumber];
            int newModelID = FindNextFreeModel();
    
            // If an invalid modelID has been returned, something has gone horribly wrong, so stop here.
            if(newModelID < 0){ 

                #if DEBUG_LOCALISATION_VERBOSITY > 0
                debug_out  <<"[" << m_timestamp << "]: !!! WARNING !!! Bad Model ID returned. Update aborted." << endl;
                #endif // DEBUG_LOCALISATION_VERBOSITY > 0

                for(int m = 0; m < c_MAX_MODELS; m++) m_models[m].m_toBeActivated = false;
                return -1;
            }

            m_models[newModelID] = m_tempModel; // Get the new model from the temp

            // Copy outlier history from the current model.
            for (int i=0; i<c_numOutlierTrackedObjects; i++){
                m_modelObjectErrors[newModelID][i] = m_modelObjectErrors[modelID][i];
            }

            // Do the update.
            kf_return =  m_models[newModelID].fieldObjectmeas(ambigousObject.measuredDistance(), ambigousObject.measuredBearing(),possibleObjects[possibleObjectID].X(), possibleObjects[possibleObjectID].Y(), R_obj_range_offset, R_obj_range_relative, R_obj_theta);

            #if DEBUG_LOCALISATION_VERBOSITY > 2
            debug_out  <<"[" << m_timestamp << "]: Splitting model[" << modelID << "] to model[" << newModelID << "].";
            //debug_out  << " Object = " << fieldObjects[possibleObjectID].name();
            debug_out  << "\tLocation = (" << possibleObjects[possibleObjectID].X() << "," << possibleObjects[possibleObjectID].Y() << ")...";
            #endif // DEBUG_LOCALISATION_VERBOSITY > 2

            // If the update reult was an outlier rejection, the model need not be kept as the
            // information is already contained in the designated outlier model created earlier
            if (kf_return == KF_OUTLIER) {
                m_models[newModelID].m_toBeActivated=false;
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
            if(kf_return == KF_OK) debug_out  << "OK" << "  Resulting alpha = " << m_models[newModelID].alpha() << endl;
            else debug_out  << "OUTLIER" << "  Resulting alpha = " << m_models[newModelID].alpha() << endl;
#endif

        }
    }
    // Split alpha between choices and also activate models
    for (int i=0; i< c_MAX_MODELS; i++) {
        if (m_models[i].m_toBeActivated) {
            m_models[i].setAlpha(m_models[i].alpha()*1.0/((float)numOptions)); // Divide each models alpha by the numbmer of splits.
            m_models[i].setActive(true);
        }
        m_models[i].m_toBeActivated=false; // Turn off activation flag
    }
    return 1;
}



bool Localisation::MergeTwoModels(int index1, int index2)
{
    // Merges second model into first model, then disables second model.
    bool success = true;
    if(index1 == index2) success = false; // Don't merge the same model.
    if((m_models[index1].active() == false) || (m_models[index2].active() == false)) success = false; // Both models must be active.
    if(success == false){
#if DEBUG_LOCALISATION_VERBOSITY > 2
        debug_out  <<"[" << m_timestamp << "]: Merge Between model[" << index1 << "] and model[" << index2 << "] FAILED." << endl;
#endif // DEBUG_LOCALISATION_VERBOSITY > 0
        return success;
    }

    // Merge alphas
    double alphaMerged = m_models[index1].alpha() + m_models[index2].alpha();
    double alpha1 = m_models[index1].alpha() / alphaMerged;
    double alpha2 = m_models[index2].alpha() / alphaMerged;

    Matrix xMerged; // Merge State matrix

    // If one model is much more correct than the other, use the correct states.
    // This prevents drifting from continuouse splitting and merging even when one model is much more likely.
    if(m_models[index1].alpha() > 10*m_models[index2].alpha()){
        xMerged = m_models[index1].stateEstimates;
    } 
    else if (m_models[index2].alpha() > 10*m_models[index1].alpha()){
        xMerged = m_models[index2].stateEstimates;
    } 
    else {
        xMerged = (alpha1 * m_models[index1].stateEstimates + alpha1 * m_models[index2].stateEstimates);
        // Fix angle.
        double angleDiff = m_models[index2].stateEstimates[2][0] - m_models[index1].stateEstimates[2][0];
        angleDiff = normaliseAngle(angleDiff);
        xMerged[2][0] = normaliseAngle(m_models[index1].stateEstimates[2][0] + alpha2*angleDiff);
    }
 
    // Merge Covariance matrix (S = sqrt(P))
    Matrix xDiff = m_models[index1].stateEstimates - xMerged;
    Matrix p1 = (m_models[index1].stateStandardDeviations * m_models[index1].stateStandardDeviations.transp() + xDiff * xDiff.transp());

    xDiff = m_models[index2].stateEstimates - xMerged;
    Matrix p2 = (m_models[index2].stateStandardDeviations * m_models[index2].stateStandardDeviations.transp() + xDiff * xDiff.transp());
  
    Matrix sMerged = cholesky(alpha1 * p1 + alpha2 * p2); // P merged = alpha1 * p1 + alpha2 * p2.

    // Copy merged value to first model
    m_models[index1].setAlpha(alphaMerged);
    m_models[index1].stateEstimates = xMerged;
    m_models[index1].stateStandardDeviations = sMerged;

    // Disable second model
    m_models[index2].setActive(false);
    m_models[index2].m_toBeActivated = false;

    for (int i=0; i<c_numOutlierTrackedObjects; i++) m_modelObjectErrors[index2][i] = 0.0; // Reset outlier values.
    return true;
}



int Localisation::getNumActiveModels()
{
    int numActive = 0;
    for (int modelID = 0; modelID < c_MAX_MODELS; modelID++){
        if(m_models[modelID].active() == true) numActive++;
    }
    return numActive;
}



int Localisation::getNumFreeModels()
{
    int numFree = 0;
    for (int modelID = 0; modelID < c_MAX_MODELS; modelID++){
        if((m_models[modelID].active() == false) && (m_models[modelID].m_toBeActivated == false)) numFree++;
    }
    return numFree;
}


const KF& Localisation::getModel(int modelNumber) const
{
    return m_models[modelNumber];
}

const KF& Localisation::getBestModel() const
{
    return m_models[getBestModelID()];
}

int Localisation::getBestModelID() const
{
    // Return model with highest alpha value.
    int bestID = 0;
    for (int currID = 0; currID < c_MAX_MODELS; currID++){
        if(m_models[currID].active() == false) continue; // Skip inactive models.
        if(m_models[currID].alpha() > m_models[bestID].alpha()) bestID = currID;
    }
    return bestID;
}

bool Localisation::IsValidObject(const Object& theObject)
{
    bool isValid = true;
    if(theObject.measuredDistance() == 0.0) isValid = false;
    if(theObject.measuredDistance() != theObject.measuredDistance()) isValid = false;
    if(theObject.measuredBearing() != theObject.measuredBearing()) isValid = false;
    if(theObject.measuredElevation() != theObject.measuredElevation()) isValid = false;
    return isValid;
}

bool Localisation::CheckModelForOutlierReset(int modelID)
{
    // RHM 7/7/08: Suggested incorporation of 'Resetting' for possibly 'kidnapped' robot
    //----------------------------------------------------------
    double sum = 0.0;
    int numObjects = 0;
    bool reset = false;
    for(int objID = 0; objID < c_numOutlierTrackedObjects; objID++){
        switch(objID)
        {
        case FieldObjects::FO_BLUE_LEFT_GOALPOST:
        case FieldObjects::FO_BLUE_RIGHT_GOALPOST:
        case FieldObjects::FO_YELLOW_LEFT_GOALPOST:
        case FieldObjects::FO_YELLOW_RIGHT_GOALPOST:
            sum += m_modelObjectErrors[modelID][objID];
            if (m_modelObjectErrors[modelID][objID] > c_OBJECT_ERROR_THRESHOLD) numObjects+=1;
            m_modelObjectErrors[modelID][objID] *= c_OBJECT_ERROR_DECAY;
            break;
        default:
            break;
        }
    }

    // Check if enough recent 'outliers' that we should reset ?
    if ((sum > c_RESET_SUM_THRESHOLD) && (numObjects >= c_RESET_NUM_THRESHOLD)) {
        reset = true;
        //models[modelID].Reset(); //Reset KF varainces. Leave Xhat!

        #if DEBUG_LOCALISATION_VERBOSITY > 0
        debug_out << "[" << m_timestamp << "]: Model[" << modelID << "] Reset due to outliers." << endl;
        #endif // DEBUG_LOCALISATION_VERBOSITY > 1

        for (int i=0; i<c_numOutlierTrackedObjects; i++) m_modelObjectErrors[modelID][i] = 0.0; // Reset the outlier history
    }
    return reset;
}


int  Localisation::CheckForOutlierResets()
{
    bool numResets = 0;
    for (int modelID = 0; modelID < c_MAX_MODELS; modelID++){
        if(CheckModelForOutlierReset(modelID))
        {
            m_models[modelID].setActive(false);
            numResets++;
        }
    }
    if(getNumActiveModels() < 1)
    {
        this->doReset();
    }
    return numResets;
}


int Localisation::varianceCheckAll(FieldObjects* fobs)
{
    int numModelsChanged = 0;
    bool changed;
    for (int currID = 0; currID < c_MAX_MODELS; currID++){
        if(m_models[currID].active() == false)
	{
		continue; // Skip inactive models.
	}
        changed = varianceCheck(currID, fobs);
        if(changed) 
	{
		numModelsChanged++;
	}
	
    }
    return numModelsChanged;
}

bool Localisation::varianceCheck(int modelID, FieldObjects* fobs)
{
    // Which direction on the field you should be facing to see the goals
    const double blueDirection = PI;
    const double yellowDirection = 0;

    bool   changed = false;
    //double var = models[modelID].variance(2);	//angle variance
    //bool   largeVariance = (var > (c_LargeAngleSD * c_LargeAngleSD));
    
    // If we think we know where we are facing don't change anything
//    if(largeVariance == false) return changed;

     // Otherwise try to adjust to fit a goal we can see.
     // Blue Goal - From center at PI radians bearing.
     if( (fobs->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST].isObjectVisible() == true) && (fobs->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST].measuredDistance() > 100) )
     {	
  	  #if DEBUG_LOCALISATION_VERBOSITY > 0
	     debug_out<<"Localisation : Saw left blue goal , and distance is : "
                            <<fobs->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST].measuredDistance()<<endl;
	  #endif
          m_models[modelID].stateEstimates[2][0]=(blueDirection - fobs->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST].measuredBearing());
              changed = true;
     }
 
     else if( (fobs->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST].isObjectVisible() == true) && (fobs->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST].measuredDistance() > 100) )
     {
	#if DEBUG_LOCALISATION_VERBOSITY > 0
	     debug_out<<"Localisation : Saw right blue goal , and distance is : "
                            <<fobs->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST].measuredDistance()<<endl;
	#endif

         m_models[modelID].stateEstimates[2][0]=(blueDirection - fobs->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST].measuredBearing());
         changed = true;
     }
         /* NEED TO FIX THIS I DON't KNOW HOW IT WILL WORK YET!
 	else if( (objects[FO_BLUE_GOALPOST_UNKNOWN].seen == true) && (objects[FO_BLUE_GOALPOST_UNKNOWN].visionDistance > 100) ){
 		models[modelID].stateEstimates[2][0]=(blueDirection - objects[FO_BLUE_GOALPOST_UNKNOWN].visionBearing);
         changed = true;
 	}
         */
 
   // Yellow Goal - From center at 0.0 radians bearing.
     if( (fobs->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST].isObjectVisible() == true) &&
          (fobs->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST].measuredDistance() > 100) )
     {
	#if DEBUG_LOCALISATION_VERBOSITY > 0
	     debug_out<<"Localisation : Saw left yellow goal , and distance is : "
                            <<fobs->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST].measuredDistance()<<endl;
	#endif
			     
         m_models[modelID].stateEstimates[2][0]=(yellowDirection -
                         fobs->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST].measuredBearing());
         changed = true;
     }
     else if( (fobs->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST].isObjectVisible() == true) &&
               (fobs->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST].measuredDistance() > 100) )
     {
	     
	#if DEBUG_LOCALISATION_VERBOSITY > 0
	     debug_out<<"Localisation : Saw left yellow goal , and distance is : "
                            <<fobs->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST].measuredDistance()<<endl;
	#endif
			     
         m_models[modelID].stateEstimates[2][0]=(yellowDirection -
                         fobs->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST].measuredBearing());
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
		  
                debug_out << "[" << m_timestamp << "]: Model[" << modelID << "]";
                debug_out << "Bearing adjusted due to Goal. New Value = " << m_models[modelID].stateEstimates[2][0] << endl;
	  }
 	#endif
   	  return changed;
}

void Localisation::NormaliseAlphas()
{
    // Normalise all of the models alpha values such that all active models sum to 1.0
    double sumAlpha=0.0;
    for (int i = 0; i < c_MAX_MODELS; i++)
    {
        if (m_models[i].active())
        {
            sumAlpha+=m_models[i].alpha();
        }
    }
    if(sumAlpha == 1) return;
    if (sumAlpha == 0) sumAlpha = 1e-12;
    for (int i = 0; i < c_MAX_MODELS; i++)
    {
        if (m_models[i].active())
        {
            m_models[i].setAlpha(m_models[i].alpha()/sumAlpha);
        }
    }
}



int Localisation::FindNextFreeModel()
{
    for (int i=0; i<c_MAX_MODELS; i++) {
        if ((m_models[i].active() == true) || (m_models[i].active() == true)) continue;
        else return i;
    }
    return -1; // NO FREE MODELS - This is very, very bad.
}


// Reset all of the models
void Localisation::ResetAll()
{

#if DEBUG_LOCALISATION_VERBOSITY > 0
    debug_out  <<"[" << m_timestamp << "]: Resetting All Models." << endl;
#endif

    for(int modelNum = 0; modelNum < c_MAX_MODELS; modelNum++){
        m_models[modelNum].init();
        for (int i=0; i<c_numOutlierTrackedObjects; i++) m_modelObjectErrors[modelNum][i] = 0.0; // Reset outlier values.
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
#if DEBUG_LOCALISATION_VERBOSITY > 2
  debug_out  <<"[" << m_currentFrameNumber << "]: Model[" << modelID << "]";
  debug_out  << "[alpha=" << m_models[modelID].alpha() << "]";
  debug_out  << " active = " << m_models[modelID].active();
  debug_out  << " activate = " << m_models[modelID].m_toBeActivated << endl;
#endif
  return;
}



void Localisation::MergeModelsBelowThreshold(double MergeMetricThreshold)
{
    double mergeM;
    for (int i = 0; i < c_MAX_MODELS; i++) {
        for (int j = i; j < c_MAX_MODELS; j++) {
            if(i == j) continue;
            if (!m_models[i].active() || !m_models[j].active()) continue;
            mergeM = abs( MergeMetric(i,j) );
            if (mergeM < MergeMetricThreshold) { //0.5
#if DEBUG_LOCALISATION_VERBOSITY > 2
                debug_out  <<"[" << m_currentFrameNumber << "]: Merging Model[" << j << "][alpha=" << m_models[j].alpha() << "]";
                debug_out  << " into Model[" << i << "][alpha=" << m_models[i].alpha() << "] " << " Merge Metric = " << mergeM << endl  ;
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
    if (!m_models[index1].active() || !m_models[index2].active()) return 10000.0; //at least one model inactive
    Matrix xdif = m_models[index1].stateEstimates - m_models[index2].stateEstimates;
    Matrix p1 = m_models[index1].stateStandardDeviations * m_models[index1].stateStandardDeviations.transp();
    Matrix p2 = m_models[index2].stateStandardDeviations * m_models[index2].stateStandardDeviations.transp();
  
    xdif[2][0] = normaliseAngle(xdif[2][0]);

    double dij=0;
    for (int i=0; i<p1.getm(); i++) {
        dij+=(xdif[i][0]*xdif[i][0]) / (p1[i][i]+p2[i][i]);
    }
    return dij*( (m_models[index1].alpha()*m_models[index2].alpha()) / (m_models[index1].alpha()+m_models[index2].alpha()) );
}


void Localisation::feedback(double* feedback)
{
	feedbackPosition[0] = feedback[0];
	feedbackPosition[1] = feedback[1];
	feedbackPosition[2] = feedback[2];
}

std::ostream& operator<< (std::ostream& output, const Localisation& p_loc)
{
    int numodels = p_loc.c_MAX_MODELS;
    output.write(reinterpret_cast<const char*>(&p_loc.m_timestamp), sizeof(p_loc.m_timestamp));
    output.write(reinterpret_cast<const char*>(&numodels), sizeof(numodels));
    for (int i = 0; i < numodels; ++i)
    {
        output << p_loc.m_models[i];
        for (int j = 0; j < p_loc.c_numOutlierTrackedObjects; ++j)
            output.write(reinterpret_cast<const char*>(&p_loc.m_modelObjectErrors[i][j]), sizeof(p_loc.m_modelObjectErrors[i][j]));
    }
    output.write(reinterpret_cast<const char*>(&p_loc.m_previously_incapacitated), sizeof(p_loc.m_previously_incapacitated));
    output.write(reinterpret_cast<const char*>(&p_loc.m_previous_game_state), sizeof(p_loc.m_previous_game_state));
    return output;
}

std::istream& operator>> (std::istream& input, Localisation& p_loc)
{
    int numModels;
    input.read(reinterpret_cast<char*>(&p_loc.m_timestamp), sizeof(p_loc.m_timestamp));
    input.read(reinterpret_cast<char*>(&numModels), sizeof(numModels));
    for (int i = 0; i < numModels; ++i)
    {
        input >> p_loc.m_models[i];
        for (int j = 0; j < p_loc.c_numOutlierTrackedObjects; ++j)
            input.read(reinterpret_cast<char*>(&p_loc.m_modelObjectErrors[i][j]), sizeof(p_loc.m_modelObjectErrors[i][j]));
    }
    input.read(reinterpret_cast<char*>(&p_loc.m_previously_incapacitated), sizeof(p_loc.m_previously_incapacitated));
    input.read(reinterpret_cast<char*>(&p_loc.m_previous_game_state), sizeof(p_loc.m_previous_game_state));
    return input;
}
