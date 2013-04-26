#include "SelfLocalisation.h"

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

#include "Tools/Profiling/Profiler.h"
#include "Localisation/Filters/IKalmanFilter.h"
#include "Localisation/Filters/KFBuilder.h"
#include "Localisation/Filters/IKFModel.h"
#include "Localisation/Filters/MobileObjectModel.h"
#include "Localisation/Filters/RobotModel.h"
#include <algorithm>

#include <assert.h>


#define MULTIPLE_MODELS_ON 1
#define AMBIGUOUS_CORNERS_ON 1
#define SHARED_BALL_ON 0
#define TWO_OBJECT_UPDATE_ON 0

#define CENTER_CIRCLE_ON 1 

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
const float SelfLocalisation::c_LargeAngleSD = PI/2;   //For variance check

// Object distance measurement error weightings (Constant)
const float SelfLocalisation::c_obj_theta_variance = 0.1f*0.1f;        // (0.1 rad)^2

const float SelfLocalisation::c_obj_range_offset_variance = 20.0f*20.0f;     // (25cm)^2
const float SelfLocalisation::c_obj_range_relative_variance = 0.20f*0.20f;   // 20% of range added
const float SelfLocalisation::c_centre_circle_heading_variance = (float)(deg2rad(20)*deg2rad(20)); // (10 degrees)^2
const float SelfLocalisation::c_twoObjectAngleVariance = 0.05f*0.05f; //Small! error in angle difference is normally very small

/*! @brief Constructor
    @param playerNumber The player number of the current robot/system. This assists in choosing reset positions.
 */
SelfLocalisation::SelfLocalisation(int playerNumber, const LocalisationSettings& settings): m_timestamp(0), m_settings(settings)
{
    init();
    return;
}

SelfLocalisation::SelfLocalisation(int playerNumber): m_timestamp(0)
{
    // Set default settings
    m_settings.setBranchMethod(LocalisationSettings::branch_exhaustive);
    m_settings.setPruneMethod(LocalisationSettings::prune_viterbi);
    m_settings.setBallLocFilter(KFBuilder::kseq_ukf_filter);
    m_settings.setBallLocModel(KFBuilder::kmobile_object_model);
    m_settings.setSelfLocFilter(KFBuilder::kseq_ukf_filter);
    m_settings.setSelfLocModel(KFBuilder::krobot_model);

    init();
    return;
}

/*! @brief Copy Constructor
    @param source The source localisation system from which to copy
 */
SelfLocalisation::SelfLocalisation(const SelfLocalisation& source): TimestampedData(), m_settings(source.m_settings)
{
    m_ball_filter = NULL;
    *this = source;
}

/*! @brief Copy operator
    @param source The source localisation system from which to copy

    @return Returns the current localisation following the copy.
 */
SelfLocalisation& SelfLocalisation::operator=(const SelfLocalisation& source)
{
    if (this != &source) // protect against invalid self-assignment
    {
        m_timestamp = source.m_timestamp;
        m_currentFrameNumber = source.m_currentFrameNumber;
        m_previously_incapacitated = source.m_previously_incapacitated;
        m_previous_game_state = source.m_previous_game_state;
        m_frame_log.str(source.m_frame_log.str());
        m_gps = source.m_gps;
        m_compass = source.m_compass;
        m_hasGps = source.m_hasGps;
        m_settings = source.m_settings;

        clearModels();

        // New robot models
        m_robot_filters.clear();
        IKalmanFilter* filter;
        for (std::list<IKalmanFilter*>::const_iterator filter_it = source.m_robot_filters.begin(); filter_it != source.m_robot_filters.end(); ++filter_it)
        {
            filter = (*filter_it)->Clone();
            m_robot_filters.push_back(filter);
        }

        if (m_ball_filter!=NULL)
        {
            delete m_ball_filter;
        }
        m_ball_filter = newBallModel();
        m_ball_filter->initialiseEstimate(source.m_ball_filter->estimate());
    }
    // by convention, always return *this
    return *this;
}

void SelfLocalisation::init()
{
    m_hasGps = false;
    m_previously_incapacitated = true;
    m_previous_game_state = GameInformation::InitialState;
    m_currentFrameNumber = 0;
    total_bad_known_objects = 0;

    m_amILost = true;
    m_lostCount = 100;
    m_timeSinceFieldObjectSeen = 0;
    m_gps.resize(2,0.0f);

    m_pastAmbiguous.resize(FieldObjects::NUM_AMBIGUOUS_FIELD_OBJECTS);
    m_prevSharedBalls.clear();

    m_ball_filter = newBallModel();

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
}

/*! @brief Destructor
 */
SelfLocalisation::~SelfLocalisation()
{
    clearModels();
    if (m_ball_filter!=NULL)
    {
        delete m_ball_filter;
    }
    #if DEBUG_LOCALISATION_VERBOSITY > 0
    debug_file.close();
    #endif // DEBUG_LOCALISATION_VERBOSITY > 0
}


IKalmanFilter* SelfLocalisation::newBallModel()
{
    IKalmanFilter* filter = KFBuilder::getNewFilter(m_settings.ballLocFilter(), m_settings.ballLocModel());
    filter->enableOutlierFiltering(false);  // disable
    filter->enableWeighting(false);         // disable
    filter->setActive();
    return filter;
}

IKalmanFilter* SelfLocalisation::robotFilter()
{
    return KFBuilder::getNewFilter(m_settings.selfLocFilter(), m_settings.selfLocModel());
}

IKalmanFilter* SelfLocalisation::newRobotModel()
{
    IKalmanFilter* filter = robotFilter();

    // set initial settings.
    filter->enableOutlierFiltering();
    filter->setOutlierThreshold(15.f);
    filter->enableWeighting();
    filter->setActive();
    filter->m_creation_time = GetTimestamp();
    return filter;
}

IKalmanFilter* SelfLocalisation::newRobotModel(IKalmanFilter* filter, const StationaryObject& measured_object, const MeasurementError &error,
                                               int ambiguous_id, double timestamp)
{
    Matrix meas_noise = error.errorCovariance();

    IKalmanFilter* new_filter = filter->Clone();
    new_filter->AssignNewId();  // update with a new ID.

    Matrix meas(2,1,false);
    meas[0][0] = measured_object.measuredDistance() * cos(measured_object.measuredElevation());
    meas[1][0] = measured_object.measuredBearing();

    Matrix args(2,1,false);
    args[0][0] = measured_object.X();
    args[1][0] = measured_object.Y();

    bool success = new_filter->measurementUpdate(meas, meas_noise, args, RobotModel::klandmark_measurement);
    new_filter->setActive(success);

    if(new_filter->active())
    {
        new_filter->m_creation_time = timestamp;
        new_filter->m_parent_history_buffer = filter->m_parent_history_buffer;
        new_filter->m_parent_history_buffer.push_back(filter->id());
        new_filter->m_parent_id = filter->id();
        new_filter->m_split_option = measured_object.getID();
        new_filter->m_previous_decisions = filter->m_previous_decisions;
        new_filter->m_previous_decisions[ambiguous_id] = measured_object.getID();
    }

    return new_filter;
}

//--------------------------------- MAIN FUNCTIONS  ---------------------------------//

/*! @brief Process function
    Processes the data from the current frame to determine the new estimation of the robots position.

    @param sensor_data Data from the robots sensors. These are used to determine the odometry of the robot.
    @param fobs The object information output by the vision module. This contains objects identified and their relative positions.
    @param gameInfo Contains information about the game. This is used to determine resetting triggers when set conditions are met.
    @param teamInfo The team information. This contains the information received from other robots.
 */
void SelfLocalisation::process(NUSensorsData* sensor_data, FieldObjects* fobs, const GameInformation* gameInfo, const TeamInformation* teamInfo)
{
    Profiler prof("Localisation");
    prof.start();

    m_frame_log.str(""); // Clear buffer.
    if (sensor_data == NULL or fobs == NULL)
        return;

    // Calculate time passed since previous frame.
    float time_increment = sensor_data->CurrentTime - m_timestamp;
    m_timestamp = sensor_data->CurrentTime;
    m_currentFrameNumber++;

#if LOC_SUMMARY_LEVEL > 0
    m_frame_log << "Frame " << m_currentFrameNumber << " Time: " << m_timestamp << std::endl;
#endif

    // Check if processing is required.
    bool processing_required = CheckGameState(sensor_data->isIncapacitated(), gameInfo);

    // retrieve gps data
    if (sensor_data->getGps(m_gps) and sensor_data->getCompass(m_compass))
    {
        m_hasGps = true;
    }

    // Retrieve odometry data
    vector<float> odo;
    bool odom_ok = sensor_data->getOdometry(odo);

    if(processing_required == false)
    {
        #if LOC_SUMMARY_LEVEL > 0
        m_frame_log << "Processing Cancelled." << std::endl;
        #endif
        return;
    }

#ifndef USE_VISION
    // If vision is disabled, gps coordinates are used in its place to trach location.
    vector<float> gps;
    float compass;
    if (sensor_data->getGps(gps) and sensor_data->getCompass(compass))
    {
        #if LOC_SUMMARY_LEVEL > 0
        m_frame_log << "Setting position from GPS: (" << gps[0] << "," << gps[1] << "," << compass << ")" << std::endl;
        #endif
        fobs->self.updateLocationOfSelf(gps[0], gps[1], compass, 0.1, 0.1, 0.01, false);
        return;
    }
#else
    prof.split("Initial Processing");
    if (odom_ok)
    {
        float fwd = odo[0];
        float side = odo[1];
        float turn = odo[2];
        // perform odometry update and change the variance of the model

        #if LOC_SUMMARY_LEVEL > 0
        m_frame_log << "Time Update - Odometry: (" << fwd << "," << side << "," << turn << ")";
        m_frame_log << " Time Increment: " << time_increment << std::endl;
        #endif
        // Hack... Sometimes we get really big odometry turn values that are not real.
        // TODO: See if this can be removed.
        if(fabs(turn) > 1.0f) turn = 0;

        doTimeUpdate(fwd, side, turn, time_increment);

        #if LOC_SUMMARY_LEVEL > 0
            m_frame_log << std::endl << "Result: " << getBestModel()->summary(false);
        #endif
    }
    prof.split("Time Update");

    #if LOC_SUMMARY_LEVEL > 0
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
    for (unsigned int i=0; i < fobs->mobileFieldObjects.size(); i++)
    {
        if(fobs->mobileFieldObjects[i].isObjectVisible()) ++objseen;
    }
    m_frame_log << "Mobile Objects: " << objseen << std::endl;
    m_frame_log << "Ambiguous Objects: " << fobs->ambiguousFieldObjects.size() << std::endl;
    #endif
    ProcessObjects(fobs, time_increment);
    prof.split("Object Update");

    ProcessObjects(fobs, time_increment);

// Shared ball stuff
    MobileObject& ball = fobs->mobileFieldObjects[FieldObjects::FO_BALL];
    if(ball.lost() and ball.TimeLastSeen() > 3000)
    {
        std::vector<TeamPacket::SharedBall> shared_balls = FindNewSharedBalls(teamInfo->getSharedBalls());
        sharedBallUpdate(shared_balls);
    }

    // clip models back on to field.
    clipActiveModelsToField();
    prof.split("Clipping");

    // Store WM Data in Field Objects.
    //int bestModelID = getBestModelID();
    // Get the best model to use.
    WriteModelToObjects(getBestModel(), fobs);
    prof.split("Writing Models");

#if LOC_SUMMARY_LEVEL > 0
    m_frame_log << std::endl <<  "Final Result: " << ModelStatusSummary();
    #endif

#endif
    prof.stop();
    return;
}

void SelfLocalisation::IndividualStationaryObjectUpdate(FieldObjects* fobs, float time_increment)
{
    int numUpdates = 0;
    int usefulObjectCount = 0;

    // Proccess the Stationary Known Field Objects
    StationaryObjectsIt currStat(fobs->stationaryFieldObjects.begin());
    StationaryObjectsConstIt endStat(fobs->stationaryFieldObjects.end());

    // all objects at once.
    unsigned int objectsAdded = 0;
    unsigned int totalSuccessfulUpdates = 0;
    for(; currStat != endStat; ++currStat)
    {
        if(currStat->isObjectVisible() == false) continue; // Skip objects that were not seen.
#if CENTER_CIRCLE_ON
        totalSuccessfulUpdates += landmarkUpdate(*currStat);
        objectsAdded++;
#else
        if(!((*currStat).getName() == fobs->stationaryFieldObjects[FieldObjects::FO_CORNER_CENTRE_CIRCLE].getName()))
        {
            totalSuccessfulUpdates += landmarkUpdate(*currStat);
            objectsAdded++;
        }
#endif
    }


    if(objectsAdded > 0 and totalSuccessfulUpdates < 1)
    {
        total_bad_known_objects += objectsAdded;
    }
    else
    {
        total_bad_known_objects = 0;
    }

    if(total_bad_known_objects > 3)
    {
        // reset
        if(m_settings.pruneMethod() != LocalisationSettings::prune_none and m_settings.pruneMethod() != LocalisationSettings::prune_unknown)
        {
            doReset();
        }
        else
        {
            doSingleReset();
        }
        // reapply the updates.
        currStat = fobs->stationaryFieldObjects.begin();
        endStat = fobs->stationaryFieldObjects.end();
        for(; currStat != endStat; ++currStat)
        {
            if(currStat->isObjectVisible() == false) continue; // Skip objects that were not seen.
    #if CENTER_CIRCLE_ON
            totalSuccessfulUpdates += landmarkUpdate(*currStat);
            objectsAdded++;
    #else
            if(!((*currStat).getName() == fobs->stationaryFieldObjects[FieldObjects::FO_CORNER_CENTRE_CIRCLE].getName()))
            {
                totalSuccessfulUpdates += landmarkUpdate(*currStat);
                objectsAdded++;
            }
    #endif
        }
    }

    numUpdates+=objectsAdded;
    usefulObjectCount+=objectsAdded;

    NormaliseAlphas();
    return;
}

/*! @brief Process objects
    Processes the field objects and perfroms the correction updates required from the observations.

    @param fobs The object information output by the vision module. This contains objects identified and their relative positions.
    @param time_increment The time that has elapsed since the previous localisation frame.

 */
void SelfLocalisation::ProcessObjects(FieldObjects* fobs, float time_increment)
{
    Profiler prof("Process Objects");
    prof.start();
    int numUpdates = 0;
    int updateResult;
    int usefulObjectCount = 0;

#if DEBUG_LOCALISATION_VERBOSITY > 2
    if(numUpdates == 0 )
    {
        debug_out  <<"[" << m_timestamp << "]: Update Starting." << endl;
        for (std::list<IKalmanFilter*>::const_iterator model_it = m_robot_filters.begin(); model_it != m_robot_filters.end(); ++model_it)
        {
            if((*model_it)->active() == false) continue;
            debug_out  << "[" << m_timestamp << "]: Model[" << (*model_it)->id() << "]";
            debug_out  << " [alpha = " << (*model_it)->getFilterWeight() << "]";
            debug_out  << " Robot X: " << (*model_it)->mean(RobotModel::kstates_x);
            debug_out  << " Robot Y: " << (*model_it)->mean(RobotModel::kstates_y);
            debug_out  << " Robot Theta: " << (*model_it)->mean(RobotModel::kstates_heading) << endl;
        }
    }
#endif // DEBUG_LOCALISATION_VERBOSITY > 2

    IndividualStationaryObjectUpdate(fobs, time_increment);

    prof.split("Known Object Update");

    if(m_settings.pruneMethod() != LocalisationSettings::prune_none and m_settings.pruneMethod() != LocalisationSettings::prune_unknown)
    {
        // Two Object update
    //#if TWO_OBJECT_UPDATE_ON
        StationaryObject& leftBlue = fobs->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST];
        StationaryObject& rightBlue = fobs->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST];
        StationaryObject& leftYellow = fobs->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST];
        StationaryObject& rightYellow = fobs->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST];

        if( leftBlue.isObjectVisible() and rightBlue.isObjectVisible())
        {
            doTwoObjectUpdate(leftBlue, rightBlue);
        }
        if( leftYellow.isObjectVisible() and rightYellow.isObjectVisible())
        {
            doTwoObjectUpdate(leftYellow, rightYellow);
        }
    }
    prof.split("Two Object Update");

#if MULTIPLE_MODELS_ON
        bool blueGoalSeen = fobs->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST].isObjectVisible() || fobs->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST].isObjectVisible();
        bool yellowGoalSeen = fobs->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST].isObjectVisible() || fobs->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST].isObjectVisible();
        removeAmbiguousGoalPairs(fobs->ambiguousFieldObjects, yellowGoalSeen, blueGoalSeen);
        // Do Ambiguous objects.
        AmbiguousObjectsIt currAmb(fobs->ambiguousFieldObjects.begin());
        AmbiguousObjectsConstIt endAmb(fobs->ambiguousFieldObjects.end());
        for(; currAmb != endAmb; ++currAmb){
            if(currAmb->isObjectVisible() == false) continue; // Skip objects that were not seen.
            //std::cout << "Ambiguous object update: " << currAmb->getName() << " (" << m_robot_filters.size() << ")" << std::endl << std::flush;
            std::vector<int> possible_ids = currAmb->getPossibleObjectIDs();
            std::vector<StationaryObject*> poss_obj;
            poss_obj.reserve(possible_ids.size());
            for(std::vector<int>::iterator pos_it = possible_ids.begin(); pos_it != possible_ids.end(); ++pos_it)
            {
                poss_obj.push_back(&(fobs->stationaryFieldObjects[(*pos_it)]));
            }

            updateResult = ambiguousLandmarkUpdate((*currAmb), poss_obj);
            NormaliseAlphas();
            PruneModels();
            numUpdates++;
            if(currAmb->getID() == FieldObjects::FO_BLUE_GOALPOST_UNKNOWN or currAmb->getID() == FieldObjects::FO_YELLOW_GOALPOST_UNKNOWN)
                usefulObjectCount++;
        }
#endif // MULTIPLE_MODELS_ON
        prof.split("Ambiguous Objects.");
    //#endif

        NormaliseAlphas();
        PruneModels();
        prof.split("Pruning");

        ballUpdate(fobs->mobileFieldObjects[FieldObjects::FO_BALL]);
        prof.split("Ball Update");

#if DEBUG_LOCALISATION_VERBOSITY > 1
        for (std::list<IKalmanFilter*>::const_iterator model_it = m_robot_filters.begin(); model_it != m_robot_filters.end(); ++model_it)
        {
            if( (*model_it)->active() )
            {
                debug_out   << "Model : " << (*model_it)->id() << " Pos  : " << (*model_it)->mean(RobotModel::kstates_x) << ", "
                        << (*model_it)->mean(RobotModel::kstates_y) << "," << (*model_it)->mean(RobotModel::kstates_heading) << std::endl;
            }
        }
#endif // DEBUG_LOCALISATION_VERBOSITY > 0
    
        if (usefulObjectCount > 0)
            m_timeSinceFieldObjectSeen = 0;
        else
            m_timeSinceFieldObjectSeen += time_increment;

#if DEBUG_LOCALISATION_VERBOSITY > 2
        const IKalmanFilter* bestModel = getBestModel();
        if(numUpdates > 0)
        {
            for (std::list<IKalmanFilter*>::const_iterator model_it = m_robot_filters.begin(); model_it != m_robot_filters.end(); ++model_it)
            {
                if( (*model_it)->active() == false) continue;
                debug_out  << "[" << m_timestamp << "]: Model[" << (*model_it)->id() << "]";
                debug_out  << " [alpha = " << (*model_it)->getFilterWeight() << "]";
                debug_out  << " Robot X: " << (*model_it)->mean(RobotModel::kstates_x);
                debug_out  << " Robot Y: " << (*model_it)->mean(RobotModel::kstates_y);
                debug_out  << " Robot Theta: " << (*model_it)->mean(RobotModel::kstates_heading) << endl;
            }
            debug_out  << "[" << m_timestamp << "]: Best Model";
            debug_out  << " [alpha = " << bestModel->getFilterWeight() << "]";
            debug_out  << " Robot X: " << bestModel->mean(RobotModel::kstates_x);
            debug_out  << " Robot Y: " << bestModel->mean(RobotModel::kstates_y);
            debug_out  << " Robot Theta: " << bestModel->mean(RobotModel::kstates_heading) << endl;
        }
#endif // DEBUG_LOCALISATION_VERBOSITY > 2	
        prof.stop();
}

/*! @brief Remove ambiguous pairs

    Scans the ambiguous objects and determines if too many of a similar object has been seen. If there are too many of the same object seen the
    objects are removed and not processed.

    @param ambiguousobjects A vector contianing all o fthe ambiguous objects.
    @param yellow_seen Boolean value. True if a fixed yellow goal has been seen. False if it has not.
    @param blue_seen Boolean value. True if a fixed blue goal has been seen. False if it has not.

 */
void SelfLocalisation::removeAmbiguousGoalPairs(std::vector<AmbiguousObject>& ambiguousobjects, bool yellow_seen, bool blue_seen)
{
    // Do Ambiguous objects.
    AmbiguousObjectsIt currAmb(ambiguousobjects.begin());
    AmbiguousObjectsConstIt endAmb(ambiguousobjects.end());
    AmbiguousObjectsIt blueGoal = ambiguousobjects.end();
    AmbiguousObjectsIt yellowGoal = ambiguousobjects.end();
    bool toomanyblue = blue_seen;
    bool toomanyyellow = yellow_seen;

    for(; currAmb != endAmb; ++currAmb)
    {
        if(currAmb->isObjectVisible() == false) continue; // Skip objects that were not seen.
        if(currAmb->getID() == FieldObjects::FO_YELLOW_GOALPOST_UNKNOWN)
        {
            if(yellowGoal == endAmb)
            {
                yellowGoal = currAmb;
            }
            else
            {
                toomanyyellow = true;
                currAmb->setIsVisible(false);
            }
        }
        if(toomanyyellow) yellowGoal->setIsVisible(false);

        if(currAmb->getID() == FieldObjects::FO_BLUE_GOALPOST_UNKNOWN)
        {
            if(blueGoal == endAmb)
            {
                blueGoal = currAmb;
            }
            else
            {
                toomanyblue=true;
                currAmb->setIsVisible(false);
            }
        }
        if(toomanyblue) blueGoal->setIsVisible(false);
    }
    return;
}

/*! @brief Writes current model to the objects

    Writes the current model to the field objects.

    @param model The model to be written.
    @param fieldObjects The objects to which the state will be written.

 */
void SelfLocalisation::WriteModelToObjects(const IKalmanFilter* model, FieldObjects* fieldObjects)
{
    // Check if lost.
    bool lost = false;
    if (m_lostCount > 20)
        lost = true;

    MultivariateGaussian est = model->estimate();

    // Update the robots location.
    fieldObjects->self.updateLocationOfSelf(est.mean(0), est.mean(1), est.mean(2), est.sd(0), est.sd(1), est.sd(2), false);

    Self& self = fieldObjects->self;

    // Now update the ball
    MobileObject& ball = fieldObjects->mobileFieldObjects[FieldObjects::FO_BALL];

    // pre-calculate the trig.
    float hcos = cos(self.Heading());
    float hsin = sin(self.Heading());

    MultivariateGaussian ball_estimate = m_ball_filter->estimate();

    // Retrieve the ball model values.
    const float relBallX = ball_estimate.mean(MobileObjectModel::kstates_x_pos);
    const float relBallY = ball_estimate.mean(MobileObjectModel::kstates_y_pos);
    const float relBallSdX = ball_estimate.sd(MobileObjectModel::kstates_x_pos);
    const float relBallSdY = ball_estimate.sd(MobileObjectModel::kstates_y_pos);
    const float relBallXVel = ball_estimate.mean(MobileObjectModel::kstates_x_vel);
    const float relBallYVel = ball_estimate.mean(MobileObjectModel::kstates_y_vel);
    const float relBallSdXVel = ball_estimate.sd(MobileObjectModel::kstates_x_vel);
    const float relBallSdYVel = ball_estimate.sd(MobileObjectModel::kstates_y_vel);

    // Rotate the relative ball postion to alight with the forward looking robot on the field.
    const float rotatedX = relBallX * hcos - relBallY * hsin;
    const float rotatedY = relBallX * hsin + relBallY * hcos;

    // Calculate the Ball location in field coordinates.
    const float ballFieldLocationX = self.wmX() + rotatedX;
    const float ballFieldLocationY = self.wmY() + rotatedY;

    // Calculate the ball location SD in field coordinates. - Not yet implemented
    const float ballFieldSdX = relBallSdX * hcos - relBallSdY * hsin;
    const float ballFieldSdY = relBallSdX * hsin + relBallSdY * hcos;

    // Calculate the Ball velocity in field coordinates.
    const float ballFieldVelocityX = relBallXVel * hcos - relBallYVel * hsin;
    const float ballFieldVelocityY = relBallXVel * hsin + relBallYVel * hcos;

    // Calculate the ball velocity SD in field coordinates. - Not yet implemented
    const float ballFieldVelocitySdX = relBallSdXVel * hcos - relBallSdYVel * hsin;
    const float ballFieldVelocitySdY = relBallSdXVel * hsin + relBallSdYVel * hcos;

    // Calculate the relative distance and heading.
    const float ballDistance = sqrt(relBallX*relBallX + relBallY*relBallY);
    const float ballHeading = atan2(relBallY, relBallX);

    // Calculate the rotation matrix for the ball covariance.
    Matrix rotMatrix(2,2,false);
    rotMatrix[0][0] = hcos;
    rotMatrix[0][1] = hsin;
    rotMatrix[1][0] = -hsin;
    rotMatrix[1][1] = hcos;

    // Retrieve the robots positional variance.
    Matrix selfPositionVariance(2,2,false);

    selfPositionVariance[0][0] = est.covariance(0,0);
    selfPositionVariance[0][1] = est.covariance(0,1);
    selfPositionVariance[1][0] = est.covariance(1,0);
    selfPositionVariance[1][1] = est.covariance(1,1);
	
	
	Matrix covMatrix(2,2,false);
    covMatrix[0][0] = ball_estimate.covariance(0,0);
    covMatrix[0][1] = ball_estimate.covariance(0,1);
    covMatrix[1][0] = ball_estimate.covariance(1,0);
    covMatrix[1][1] = ball_estimate.covariance(1,1);

    // calculate the field variance of the ball R^-1 * Sigma * R + SelfVariance
    // We are assuming that the variances are independant.
    //Matrix fieldBallVariance = InverseMatrix(rotMatrix) * m_ball_model->covariance() * rotMatrix + selfPositionVariance;
	Matrix fieldBallVariance = InverseMatrix(rotMatrix) * covMatrix * rotMatrix;
	
	//std::cout << covMatrix << std::endl;

    // Write the results to the ball object.
    ball.updateObjectLocation(ballFieldLocationX, ballFieldLocationY, ballFieldSdX, ballFieldSdY);
    ball.updateObjectVelocities(ballFieldVelocityX,ballFieldVelocityY,ballFieldVelocitySdX, ballFieldVelocitySdY);
    ball.updateEstimatedRelativeVariables(ballDistance, ballHeading, 0.0f);

    if(fieldBallVariance.isValid())
    {
        ball.updateSharedCovariance(fieldBallVariance);
    }

    const float lost_ball_sd = 150.0f;
    const float max_sd = 2 * std::max(relBallSdX, relBallSdY);
    const bool ballIsLost = max_sd > lost_ball_sd;
    ball.updateIsLost(ballIsLost, m_timestamp);
    return;
}


/*! @brief Checks the current state of the game and determines actions to be taken.

    Checks the current state of the game and determines if processing will be done. Sencondly also triggers a number of resets
    based on the game condiitions.

    @param currently_incapacitated Indicator for the robots condition. True if it has fallen or has otherwise been disrupted. False if it has not.
    @param game_info Information about the current state of the game.

    @return Returns True if processing should be performed. False if it should not.

 */
bool SelfLocalisation::CheckGameState(bool currently_incapacitated, const GameInformation* game_info)
{
    //GameInformation::TeamColour team_colour = game_info->getTeamColour();
    GameInformation::TeamColour team_colour = GameInformation::RedTeam;
    GameInformation::RobotState current_state = game_info->getCurrentState();
    /*
    if (currently_incapacitated)
    {   // if the robot is incapacitated there is no point running localisation
        m_previous_game_state = current_state;
        m_previously_incapacitated = true;
        #if LOC_SUMMARY_LEVEL > 0
        m_frame_log << "Robot is incapscitated." << std::endl;
        #endif
        return false;
    }
    */

    if (current_state == GameInformation::InitialState or current_state == GameInformation::FinishedState or current_state == GameInformation::PenalisedState)
    {   // if we are in initial, finished, penalised or substitute states do not do localisation
        m_previous_game_state = current_state;
        m_previously_incapacitated = currently_incapacitated;
        #if LOC_SUMMARY_LEVEL > 0
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

void SelfLocalisation::initSingleModel(float x, float y, float heading)
{
#if DEBUG_LOCALISATION_VERBOSITY > 0
    debug_out  << "Initialising single model." << endl;
#endif // DEBUG_LOCALISATION_VERBOSITY > 0
    clearModels();
    std::vector<MultivariateGaussian> positions;
    positions.reserve(1);
    MultivariateGaussian temp(3);
    temp.setMean(mean_matrix(x,y,heading));
    temp.setCovariance(covariance_matrix(150.0f*150.0f, 100.0f*100.0f, 2*2*PI*2*PI));
    positions.push_back(temp);
    InitialiseModels(positions);
    initBallModel(m_ball_filter);
}

void SelfLocalisation::initBallModel(IKalmanFilter* ball_model)
{
    const unsigned int total_states = ball_model->model()->totalStates();
    if(ball_model == NULL) return;
    MobileObjectModel::State state;
    Matrix mean(total_states, 1, false);
    Matrix covariance(total_states, total_states, false);

    // Assign initial covariance
    const double initial_pos_cov = 100*100;
    const double initial_vel_cov = 10*10;
    state = MobileObjectModel::kstates_x_pos;
    covariance[state][state] = initial_pos_cov;
    state = MobileObjectModel::kstates_y_pos;
    covariance[state][state] = initial_pos_cov;
    state = MobileObjectModel::kstates_x_vel;
    covariance[state][state] = initial_vel_cov;
    state = MobileObjectModel::kstates_y_vel;
    covariance[state][state] = initial_vel_cov;

    MultivariateGaussian estimate(mean, covariance);
    ball_model->initialiseEstimate(estimate);
}

void SelfLocalisation::doSingleInitialReset(GameInformation::TeamColour team_colour)
{
    float initial_heading = 0 + (team_colour==GameInformation::RedTeam?mathGeneral::PI:0);
    initSingleModel(0,0,initial_heading);
    initBallModel(m_ball_filter);
}

void SelfLocalisation::doSingleReset()
{
    clearModels();
    initSingleModel(0,0,0);
    initBallModel(m_ball_filter);
}

void SelfLocalisation::doInitialReset(GameInformation::TeamColour team_colour)
{
    #if LOC_SUMMARY_LEVEL > 0
    m_frame_log << "Reset leaving initial." << std::endl;
    #endif
#if DEBUG_LOCALISATION_VERBOSITY > 0
    debug_out  << "Performing initial->ready reset." << endl;
#endif // DEBUG_LOCALISATION_VERBOSITY > 0

    // For the probabalistic data association technique we only want a single model present.
    if(m_settings.pruneMethod() == LocalisationSettings::prune_unknown || m_settings.pruneMethod() == LocalisationSettings::prune_none || m_settings.pruneMethod() == LocalisationSettings::prune_max_likelyhood)
    {
        return doSingleInitialReset(team_colour);
    }

    clearModels();

    // The models are always in the team's own half
    // Model 0: On left sideline facing in, 1/3 from half way
    // Model 1: On left sideline facing in, 2/3 from half way
    // Model 2: On right sideline facing in, 1/3 from half way
    // Model 3: On right sideline facing in, 2/3 from half way
    // Model 4: In centre of own half facing opponents goal
    // Model 5: In goal keeper position facing opponents goal

    float left_y = 200.0;
    float left_heading = -PI/2;
    float right_y = -200.0;
    float right_heading = PI/2;

    float front_x = -300.0/4.0f;
    float centre_x = -300.0/2.0f;
    float centre_heading = 0;
    float back_x = -300*(3.0f/4.0f);
    float goal_line_x = -300;
    if (team_colour == GameInformation::RedTeam)
    {   // flip the invert the x for the red team and set the heading to PI
        front_x = -front_x;
        centre_x = -centre_x;
        centre_heading = PI;
        back_x = -back_x;
        goal_line_x = -goal_line_x;
    }

    float cov_x = pow(100.f,2);
    float cov_y = pow(75.f,2);
    //float cov_head = pow(6.f,2);
    float cov_head = pow(1.f,2);
    Matrix cov_matrix = covariance_matrix(cov_x, cov_y, cov_head);
    MultivariateGaussian temp(3);
    temp.setCovariance(cov_matrix);
    std::vector<MultivariateGaussian> positions;
    positions.reserve(5);

    // Position 1
    temp.setMean(mean_matrix(front_x, right_y, right_heading));
    positions.push_back(temp);

    // Position 2
    temp.setMean(mean_matrix(back_x, right_y, right_heading));
    positions.push_back(temp);

    // Position 3
    temp.setMean(mean_matrix(front_x, left_y, left_heading));
    positions.push_back(temp);

    // Position 4
    temp.setMean(mean_matrix(back_x, left_y, left_heading));
    positions.push_back(temp);

    // Postition 5
    temp.setMean(mean_matrix(2*centre_x, 95.0f, centre_heading));
    positions.push_back(temp);

    // Postition 6
    temp.setMean(mean_matrix(2*centre_x, -95.0f, centre_heading));
    positions.push_back(temp);

    // Postition 7
    temp.setCovariance(covariance_matrix(pow(200.0f,2), pow(150.0f,2), pow(1.f,2)));
    temp.setMean(mean_matrix(centre_x, 0.0f, centre_heading));
    positions.push_back(temp);

    // Postition 8
    temp.setMean(mean_matrix(2*centre_x, 0.0f, centre_heading));
    positions.push_back(temp);

    InitialiseModels(positions);
    initBallModel(m_ball_filter);
    return;
}

void SelfLocalisation::doSetReset(GameInformation::TeamColour team_colour, int player_number, bool have_kickoff)
{
    #if LOC_SUMMARY_LEVEL > 0
    m_frame_log << "Reset due to manual positioning." << std::endl;
    #endif
#if DEBUG_LOCALISATION_VERBOSITY > 0
    debug_out  << "Performing manual position reset." << endl;
#endif // DEBUG_LOCALISATION_VERBOSITY > 0
    clearModels();
    float x, y, heading;
    const float position_sd = 15;
    const float heading_sd = 0.1;
    int num_filters;
    std::vector<MultivariateGaussian> positions;
    MultivariateGaussian temp(3);
    temp.setCovariance(covariance_matrix(position_sd*position_sd, position_sd*position_sd, heading_sd*heading_sd));
    if (player_number == 1)
    {   // if we are the goal keeper and we get manually positioned we know exactly where we will be put
        num_filters = 1;
        x = -300;
        y = 0; 
        heading = 0;
        if (team_colour == GameInformation::RedTeam)
            swapFieldStateTeam(x, y, heading);
        temp.setMean(mean_matrix(x,y,heading));
        positions.push_back(temp);
    }
    else
    {   // if we are a field player and we get manually positioned we could be in a three different places
        if (have_kickoff)
        {   // the attacking positions are on the circle or on either side of the penalty spot
            num_filters = 3;
            // on the circle
            x = -60;
            y = 0;
            heading = 0;
            if (team_colour == GameInformation::RedTeam)
                swapFieldStateTeam(x, y, heading);
            temp.setMean(mean_matrix(x,y,heading));
            positions.push_back(temp);
            // on the left of the penalty spot
            x = -120;
            y = 70;
            heading = 0;
            if (team_colour == GameInformation::RedTeam)
                swapFieldStateTeam(x, y, heading);
            temp.setMean(mean_matrix(x,y,heading));
            positions.push_back(temp);
            // on the right of the penalty spot
            x = -120;
            y = -70;
            heading = 0;
            if (team_colour == GameInformation::RedTeam)
                swapFieldStateTeam(x, y, heading);
            temp.setMean(mean_matrix(x,y,heading));
            positions.push_back(temp);
            
        }
        else
        {   // the defensive positions are on either corner of the penalty box
            num_filters = 2;
            // top penalty box corner
            x = -240;
            y = 110;
            heading = 0;
            if (team_colour == GameInformation::RedTeam)
                swapFieldStateTeam(x, y, heading);
            temp.setMean(mean_matrix(x,y,heading));
            positions.push_back(temp);
            // bottom penalty box corner
            x = -240;
            y = -110;
            heading = 0;
            if (team_colour == GameInformation::RedTeam)
                swapFieldStateTeam(x, y, heading);
            temp.setMean(mean_matrix(x,y,heading));
            positions.push_back(temp);
        }
    }
    
    // Add the models.
    InitialiseModels(positions);
    initBallModel(m_ball_filter);
    return;
}

void SelfLocalisation::doPenaltyReset()
{
    #if LOC_SUMMARY_LEVEL > 0
    m_frame_log << "Reset due to penalty." << std::endl;
    #endif
#if DEBUG_LOCALISATION_VERBOSITY > 0
    debug_out  << "Performing penalty reset." << endl;
#endif // DEBUG_LOCALISATION_VERBOSITY > 0

    clearModels();

    std::vector<MultivariateGaussian> positions;
    positions.reserve(2);
    MultivariateGaussian temp(3);
    temp.setCovariance(covariance_matrix(75.0f*75.0f, 25.0f*25.0f, 0.35f*0.35f));

    // setup model 0 as top 'T'
    temp.setMean(mean_matrix(0.0f, 200.0, -PI/2.0f));
    positions.push_back(temp);
    
    // setup model 1 as bottom 'T'
    temp.setMean(mean_matrix(0.0f, -200.0f, PI/2.0f));
    positions.push_back(temp);

    InitialiseModels(positions);
    initBallModel(m_ball_filter);
    return;
}

void SelfLocalisation::doFallenReset()
{
    Matrix temp;
    #if LOC_SUMMARY_LEVEL > 0
    m_frame_log << "Reset due to fall." << std::endl;
    #endif
#if DEBUG_LOCALISATION_VERBOSITY > 0
    debug_out  << "Performing fallen reset." << endl;
#endif // DEBUG_LOCALISATION_VERBOSITY > 0

    // New models
    for(std::list<IKalmanFilter*>::iterator filter_it = m_robot_filters.begin(); filter_it != m_robot_filters.end(); ++ filter_it)
    {
        MultivariateGaussian est = (*filter_it)->estimate();
        temp = est.covariance();
        temp[2][2] += 0.707*0.707;     // Robot heading
        est.setCovariance(temp);
        (*filter_it)->initialiseEstimate(est);
    }

    addToBallVariance(50*50, 50*50, 0.f, 0.f);
    return;
}

void SelfLocalisation::doReset()
{
    #if DEBUG_LOCALISATION_VERBOSITY > 0
    debug_out  << "Performing player reset." << endl;
    #endif // DEBUG_LOCALISATION_VERBOSITY > 0

    clearModels();

    std::vector<MultivariateGaussian> newPositions;
    newPositions.reserve(4);

    MultivariateGaussian temp(3);

    temp.setCovariance(covariance_matrix(150.0f*150.0f, 100.0f*100.0f, 2*PI*2*PI));

    // First position ->
    temp.setMean(mean_matrix(300.0f, 0.0f, PI));
    newPositions.push_back(temp);

    // Second Position ->
    temp.setMean(mean_matrix(-300.0f, 0.0f, 0.0f));
    newPositions.push_back(temp);

    // Third Position ->
    temp.setMean(mean_matrix(0.0f, 200.0f, -PI/2.0f));
    newPositions.push_back(temp);

    // Fourth Positions ->
    temp.setMean(mean_matrix(0.0f, -200.0f, PI/2.0f));
    newPositions.push_back(temp);

    InitialiseModels(newPositions);
    initBallModel(m_ball_filter);
    return;
}

void SelfLocalisation::doBallOutReset()
{
#if DEBUG_LOCALISATION_VERBOSITY > 0
    debug_out  << "Performing ball out reset." << endl;
#endif // DEBUG_LOCALISATION_VERBOSITY > 0
    addToBallVariance(100*100, 60*60, 0.0f, 0.0f);
    return;
}

/*! @brief Changes the field state so that it will be the position for the other team */
void SelfLocalisation::swapFieldStateTeam(float& x, float& y, float& heading)
{
    x = -x;
    y = -y;
    heading = normaliseAngle(heading + PI);
}

bool SelfLocalisation::clipRobotState(MultivariateGaussian* estimate, int stateIndex, double minValue, double maxValue)
{
    bool clipped = false;
    Matrix mean = estimate->mean();
    Matrix covariance = estimate->covariance();
    if(mean[stateIndex][0] > maxValue){
        double mult, Pii;
        Matrix Si;
        Si = covariance.getRow(stateIndex);
        Pii = convDble(Si * Si.transp());
        mult = (mean[stateIndex][0] - maxValue) / Pii;
        mean = mean - mult * covariance * Si.transp();
        mean[stateIndex][0] = maxValue;
        clipped = true;
    }
    if(mean[stateIndex][0] < minValue){
        double mult, Pii;
        Matrix Si;
        Si = covariance.getRow(stateIndex);
        Pii = convDble(Si * Si.transp());
        mult = (mean[stateIndex][0] - minValue) / Pii;
        mean = mean - mult * covariance * Si.transp();
        mean[stateIndex][0] = minValue;
        clipped = true;
    }
    mean[2][0] = mathGeneral::normaliseAngle(mean[2][0]);
    estimate->setMean(mean);
    estimate->setCovariance(covariance);
    return clipped;
}

/*! @brief Clips the model to a position on the field.
    @param theModel THe model to be clipped

    @return True if clipping was required. False if not.
*/
bool SelfLocalisation::clipEstimateToField(MultivariateGaussian* estimate)
{
    const double fieldXLength = 680.0;
    const double fieldYLength = 440.0;
    const double fieldXMax = fieldXLength / 2.0;
    const double fieldXMin = - fieldXLength / 2.0;
    const double fieldYMax = fieldYLength / 2.0;
    const double fieldYMin = - fieldYLength / 2.0;

    bool wasClipped = false;
    bool clipped;
    double prevX, prevY, prevTheta;
    prevX = estimate->mean(0);
    prevY = estimate->mean(1);
    prevTheta = estimate->mean(2);

    clipped = clipRobotState(estimate, 0, fieldXMin, fieldXMax);		// Clipping for robot's X
    #if DEBUG_LOCALISATION_VERBOSITY > 1
    if(clipped){
        debug_out  << "[" << m_timestamp << "]: Model[" << theModel->id() << "]";
        debug_out  << " [alpha = " << theModel->getFilterWeight() << "]";
        debug_out  << " State(0) clipped.";
        debug_out  << " (" << prevX << "," << prevY << "," << prevTheta << ") -> (" << theModel->mean(RobotModel::kstates_x);
        debug_out  << "," << theModel->mean(RobotModel::kstates_y) << "," << theModel->mean(RobotModel::kstates_heading) << ")" << endl;
    }
    #endif // DEBUG_LOCALISATION_VERBOSITY > 1
    wasClipped = wasClipped || clipped;

    prevX = estimate->mean(0);
    prevY = estimate->mean(1);
    prevTheta = estimate->mean(2);

    clipped = clipRobotState(estimate, 1, fieldYMin, fieldYMax);		// Clipping for robot's Y

    #if DEBUG_LOCALISATION_VERBOSITY > 1
    if(clipped){
        debug_out  << "[" << m_timestamp << "]: Model[" << theModel->id() << "]";
        debug_out  << " [alpha = " << theModel->getFilterWeight() << "]";
        debug_out  << " State(1) clipped." << endl;
        debug_out  << " (" << prevX << "," << prevY << "," << prevTheta << ") -> (" << theModel->mean(RobotModel::kstates_x);
        debug_out  << "," << theModel->mean(RobotModel::kstates_y) << "," << theModel->mean(RobotModel::kstates_heading) << ")" << endl;
    }
    #endif // DEBUG_LOCALISATION_VERBOSITY > 1

    wasClipped = wasClipped || clipped;

    return wasClipped;
}

/*! @brief Clips all of the models to the field.

    @return True if clipping was required. False if not required.
*/
bool SelfLocalisation::clipActiveModelsToField()
{
    bool wasClipped = false;
    bool modelClipped = false;

    // New models
    MultivariateGaussian est;
    for (std::list<IKalmanFilter*>::const_iterator filter_it = m_robot_filters.begin(); filter_it != m_robot_filters.end(); ++filter_it)
    {
        if((*filter_it)->active() == false) continue;
        est = (*filter_it)->estimate();
        modelClipped = clipEstimateToField(&est);
        if(modelClipped) (*filter_it)->initialiseEstimate(est);
        wasClipped = wasClipped || modelClipped;
    }
    return wasClipped;
}

bool SelfLocalisation::doTimeUpdate(float odomForward, float odomLeft, float odomTurn, double timeIncrement)
{
    // put values into odometry measurement matrix
    Matrix odometry(3,1,false);
    odometry[0][0] = odomForward;
    odometry[1][0] = odomLeft;
    odometry[2][0] = odomTurn;

    // calculate the measurement noise.
    Matrix measurementNoise = Matrix(3,3,true);
    measurementNoise[0][0] = 0.2*0.2; // Robot X coord.
    measurementNoise[1][1] = 0.2*0.2; // Robot Y coord.
    measurementNoise[2][2] = 0.005*0.005; // Robot Theta. 0.00001

    // calculate the linear process noise.
    Matrix processNoise = Matrix(4,4,true);
    processNoise[0][0] = 2*2;
    processNoise[1][1] = 2*2;
    processNoise[2][2] = 4*4;
    processNoise[3][3] = 4*4;

    double deltaTimeSeconds = timeIncrement * 1e-3; // Convert from milliseconds to seconds.

    processNoise = deltaTimeSeconds * processNoise;

    // perform time update on the ball model.
    m_ball_filter->timeUpdate(deltaTimeSeconds, odometry, processNoise, measurementNoise);

    bool result = false;
    processNoise = Matrix(3,3,false);
    processNoise[0][0] = pow(0.5f,2);
    processNoise[1][1] = pow(0.5f,2);
    processNoise[2][2] = pow(0.015f, 2);

    for (std::list<IKalmanFilter*>::iterator model_it = m_robot_filters.begin(); model_it != m_robot_filters.end(); ++model_it)
    {
        if(not (*model_it)->active()) continue; // Skip Inactive models.
        result = true;
        // proportianl odometry nosie
        Matrix prop_odom_noise(3,3,false);
        float theta = (*model_it)->estimate().mean(2);
        float ctheta  = cos(theta);
        float stheta  = sin(theta);

        float xnoise = 0.1 * (odometry[0][0] * ctheta - odometry[1][0] * stheta);
        float ynoise = 0.1 * (odometry[0][0] * stheta + odometry[1][0] * ctheta);
        float tnoise = 0.1 * odometry[2][0] + 0.0001 * (fabs(odometry[0][0]) + fabs(odometry[1][0]));
        prop_odom_noise[0][0] = pow(xnoise, 2);
        prop_odom_noise[1][1] = pow(ynoise, 2);
        prop_odom_noise[2][2] = pow(tnoise, 2);
        (*model_it)->timeUpdate(deltaTimeSeconds, odometry, processNoise+prop_odom_noise, measurementNoise);

    }
    
    if(m_amILost)
        m_lostCount++;
    else
        m_lostCount = 0;
    return result;
}

int SelfLocalisation::landmarkUpdate(StationaryObject &landmark)
{
#if LOC_SUMMARY_LEVEL > 0
    m_frame_log << std::endl << "Known landmark update: " << landmark.getName() << std::endl;
#endif
    int kf_return;
    int numSuccessfulUpdates = 0;

    if(landmark.validMeasurement() == false)
    {
    #if LOC_SUMMARY_LEVEL > 0
        m_frame_log << "Skipping invalid." << std::endl;
    #endif
#if DEBUG_LOCALISATION_VERBOSITY > 0
        debug_out  <<"[" << m_timestamp << "] Skipping Bad Landmark Update: ";
        debug_out  << landmark.getName();
        debug_out  << " Distance = " << landmark.measuredDistance();
        debug_out  << " Bearing = " << landmark.measuredBearing();
        debug_out  << endl;
#endif // DEBUG_LOCALISATION_VERBOSITY > 1
        return 0;
    }

    double flatObjectDistance = landmark.measuredDistance() * cos(landmark.measuredElevation());

    MeasurementError temp_error;
    temp_error.setDistance(c_obj_range_offset_variance + c_obj_range_relative_variance * pow(flatObjectDistance,2));
    temp_error.setHeading(c_obj_theta_variance);

    Matrix measurement(2,1);
    measurement[0][0] = flatObjectDistance;
    measurement[1][0] = landmark.measuredBearing();

    Matrix args(2,1,false);
    args[0][0] = landmark.X();
    args[1][0] = landmark.Y();

    for (std::list<IKalmanFilter*>::const_iterator model_it = m_robot_filters.begin(); model_it != m_robot_filters.end(); ++model_it)
    {
        if((*model_it)->active() == false) continue; // Skip Inactive models.

#if DEBUG_LOCALISATION_VERBOSITY > 2
        debug_out  <<"[" << m_timestamp << "]: Model[" << (*model_it)->id() << "] Landmark Update. ";
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
        kf_return = 1;
        kf_return = (*model_it)->measurementUpdate(measurement, temp_error.errorCovariance(), args, RobotModel::klandmark_measurement);

#if DEBUG_LOCALISATION_VERBOSITY > 0
        if(kf_return != 1)
        {
            Matrix estimated_measurement = (*model_it)->CalculateMeasurementPrediction(landmark.X(),landmark.Y());
            debug_out << "OUTLIER!" << endl;
            debug_out << "Model[" << (*model_it)->id() << "]: Outlier Detected - " << landmark.getName() << endl;
            debug_out << "Measured - Distance = " << landmark.measuredDistance() << " Bearing = " << landmark.measuredBearing() << endl;
            debug_out << "Expected - Distance = " << estimated_measurement[0][0] << " Bearing = " << estimated_measurement[1][0] << endl;
        }
        else
        {
            debug_out << "OK!" << endl;
        }
#endif // DEBUG_LOCALISATION_VERBOSITY > 1

    MultivariateGaussian est = (*model_it)->estimate();
    #if LOC_SUMMARY_LEVEL > 0
        m_frame_log << "Model " << (*model_it)->id() << " updated using " << landmark.getName() << " measurment." << std::endl;
        m_frame_log << "Measurement: Distance = " << flatObjectDistance << ", Heading = " << landmark.measuredBearing() <<std::endl;
        m_frame_log << "Position: X = " << landmark.X() << ", Y = " << landmark.Y() <<std::endl;
        m_frame_log << "Current State: " << est.mean(0) << ", " << est.mean(1) << ", " << est.mean(2) << std::endl;

        if(m_hasGps)
        {
            float dX = landmark.X()-m_gps[0];
            float dY = landmark.Y()-m_gps[1];
            float Cc = cos(m_compass);
            float Ss = sin(m_compass);
            float x_gps = dX * Cc + dY * Ss;
            float y_gps =  -dX * Ss + dY * Cc;
            m_frame_log << "GPS Expected: (" << x_gps << "," << y_gps << ")" << std::endl;
        }

        float model_x = est.mean(0);
        float model_y = est.mean(1);
        float model_heading = est.mean(2);

        float exp_dist = sqrt(pow(landmark.X() - model_x,2) + pow(landmark.Y() - model_y,2));
        float positionHeading = atan2(landmark.Y() - model_y, landmark.X() - model_x);
        float exp_heading = normaliseAngle(positionHeading - model_heading);
        m_frame_log << "Expected: " << exp_dist << ", " << exp_heading <<std::endl;
        m_frame_log << "Measured: " << flatObjectDistance << "," << landmark.measuredBearing() << std::endl;
        m_frame_log << "Result = ";
        m_frame_log << ((kf_return==0)?"Outlier":"Success") << std::endl;
    #endif
        if(kf_return == 1) numSuccessfulUpdates++;
        if(kf_return == 0)
        {
            Matrix cov = est.covariance();
            Matrix added_noise = covariance_matrix(1,1,0.0001);
            cov = cov + added_noise;
            est.setCovariance(cov);
            (*model_it)->initialiseEstimate(est);
        }
    }
    return numSuccessfulUpdates;
}


/*! @brief Do all of the fancy stuff we only get to do when we have two good reliable objects.
    @return 1 if good, 0 if bad.
*/
int SelfLocalisation::doTwoObjectUpdate(StationaryObject &landmark1, StationaryObject &landmark2)
{

#if LOC_SUMMARY_LEVEL > 0
    m_frame_log << "Performing 2 object update on objects:\n";
    m_frame_log << "Object 1 -\n";
    m_frame_log << landmark1.toString() << std::endl;
    m_frame_log << "Object 2 -\n";
    m_frame_log << landmark2.toString() << std::endl;
#endif
    // do the special update
    float angle_beween_objects = mathGeneral::normaliseAngle(landmark1.measuredBearing() - landmark2.measuredBearing());

    Matrix measurement(1,1,false);
    measurement[0][0] = angle_beween_objects;

    Matrix noise(1,1,false);
    noise[0][0] = c_twoObjectAngleVariance;

    // args are [object1_loc; object2_loc]
    Matrix args(2,2,false);
    args[0][0] = landmark1.X();
    args[0][1] = landmark1.Y();
    args[1][0] = landmark2.X();
    args[1][1] = landmark2.Y();


    for (std::list<IKalmanFilter*>::const_iterator model_it = m_robot_filters.begin(); model_it != m_robot_filters.end(); ++model_it)
    {
        (*model_it)->measurementUpdate(measurement, noise, args, RobotModel::kangle_between_landmark_measurement);

    #if LOC_SUMMARY_LEVEL > 0
        MultivariateGaussian est = (*model_it)->estimate();
        m_frame_log << "Model " << (*model_it)->id() << " updated using " << landmark1.getName() << " + " << landmark2.getName() << " combined measurment." << std::endl;
        m_frame_log << "Measurement: Angle = " << angle_beween_objects <<std::endl;
        m_frame_log << "Position1: X = " << landmark1.X() << ", Y = " << landmark1.Y() <<std::endl;
        m_frame_log << "Position2: X = " << landmark2.X() << ", Y = " << landmark2.Y() <<std::endl;
        m_frame_log << "Current State: " << est.mean(0) << ", " << est.mean(1) << ", " << est.mean(2) << std::endl;
    #endif
    }

    Vector2<float> position = TriangulateTwoObject(landmark1, landmark2);

    float avg_x = (landmark1.X() + landmark2.X()) / 2.f;
    float avg_y = (landmark1.Y() + landmark2.Y()) / 2.f;

    float avg_heading = (landmark1.measuredBearing() + landmark2.measuredBearing()) / 2.f;
    float best_heading = atan2(avg_y - position.y, avg_x - position.x) - avg_heading;

    if(position.abs() == 0.0f)
    {
        return 0;
    }

    Matrix mean = mean_matrix(position.x, position.y, best_heading);
    Matrix cov = covariance_matrix(pow(20.f,2), pow(50.f,2), pow(0.5,2));
    float alpha = getBestModel()->getFilterWeight() * 0.0001f;

    IKalmanFilter* temp = newRobotModel();
    temp->initialiseEstimate(MultivariateGaussian(mean, cov));
    temp->setFilterWeight(alpha);
    temp->setActive(true);
    m_robot_filters.push_back(temp);

#if LOC_SUMMARY_LEVEL > 0
    m_frame_log << "Reset Model Added: " << std::endl << temp->summary(true) << std::endl;
#endif
    return 1;
}

/*! @brief Perfroms an ambiguous measurement update. This is a interface function to access a variety of methods.
    @return The number of models that were removed during this process.
*/
int SelfLocalisation::ambiguousLandmarkUpdate(AmbiguousObject &ambiguousObject, const vector<StationaryObject*>& possibleObjects)
{

    if(possibleObjects.size() < 1)
    {
#if LOC_SUMMARY_LEVEL > 0
        m_frame_log << "Ignoring Ambiguous Object - " << ambiguousObject.getName() << std::endl;
#endif
        return 0;
    }

#if LOC_SUMMARY_LEVEL > 0
    float measured_distance = ambiguousObject.measuredDistance();
    float measured_heading = ambiguousObject.measuredBearing();
    m_frame_log << "Ambiguous object: " << ambiguousObject.getName() << " at (" << measured_distance << "," << measured_heading << ")" << std::endl;
#endif

    if(m_settings.branchMethod() == LocalisationSettings::branch_exhaustive)
    {
        return ambiguousLandmarkUpdateExhaustive(ambiguousObject, possibleObjects);
    }
    else if (m_settings.branchMethod() == LocalisationSettings::branch_selective)
    {
        return ambiguousLandmarkUpdateSelective(ambiguousObject, possibleObjects);
    }
    else if (m_settings.branchMethod() == LocalisationSettings::branch_constraint)
    {
        return ambiguousLandmarkUpdateConstraint(ambiguousObject, possibleObjects);
    }
    else if (m_settings.branchMethod() == LocalisationSettings::branch_probDataAssoc)
    {
        return ambiguousLandmarkUpdateProbDataAssoc(ambiguousObject, possibleObjects);
    }
    return 0;
}

bool SelfLocalisation::ballUpdate(const MobileObject& ball)
{
    const float time_for_new_loc = 1000;
    if(ball.isObjectVisible())
    {
        const float distance = ball.measuredDistance() * cos(ball.measuredElevation());
        const float heading = ball.measuredBearing();

        Matrix measurement(2,1,false);
        measurement[0][0] = distance;
        measurement[1][0] = heading;

        Matrix measurementNoise(2,2,false);
        measurementNoise[0][0] = 5.0*5.0 + c_obj_range_relative_variance * pow(distance,2);
        measurementNoise[1][1] = 0.01*0.01;

        m_ball_filter->measurementUpdate(measurement, measurementNoise, Matrix(), 0);
        if((m_timestamp - m_prev_ball_update_time) > time_for_new_loc)
        {
            MultivariateGaussian est = m_ball_filter->estimate();
            Matrix filter_mean = est.mean();
            filter_mean[2][0] = 0.0;
            filter_mean[3][0] = 0.0;
            est.setMean(filter_mean);
            m_ball_filter->initialiseEstimate(est);
        }
        m_prev_ball_update_time = m_timestamp;
    }
}

/*! @brief Prunes the models using a selectable method. This is a interface function to access a variety of methods.
    @return The number of models that were removed during this process.
*/
int SelfLocalisation::PruneModels()
{
    removeInactiveModels();    // Clear out all deactivated models.

    if(m_settings.pruneMethod() == LocalisationSettings::prune_merge)
    {
        MergeModels(c_MAX_MODELS_AFTER_MERGE);
    }
    else if (m_settings.pruneMethod() == LocalisationSettings::prune_max_likelyhood)
    {
        removeSimilarModels();
        PruneMaxLikelyhood();
    }
    else if (m_settings.pruneMethod() == LocalisationSettings::prune_viterbi)
    {
        removeSimilarModels();
        PruneViterbi(c_MAX_MODELS_AFTER_MERGE);
    }
    else if (m_settings.pruneMethod() == LocalisationSettings::prune_nscan)
    {
        PruneNScan(3);
    }
    NormaliseAlphas();
    return 0;
}

/*! @brief Prunes the models using the maximum likelyhood method. This removes all but the mot probable model.
    @return The number of models that were removed during this process.
*/
int SelfLocalisation::PruneMaxLikelyhood()
{
    return PruneViterbi(1);     // Max likelyhood is the equivalent of keeping the best model.
}

struct model_ptr_cmp
{
    bool operator()( IKalmanFilter* lhs, IKalmanFilter* rhs)
    {
        return lhs->getFilterWeight() < rhs->getFilterWeight();
    }
};

/*! @brief Prunes the models using the Viterbi method. This removes lower probability models to a maximum total models.
    @param order The number of models to be kept at the end for the process.
    @return The number of models that were removed during this process.
*/
int SelfLocalisation::PruneViterbi(unsigned int order)
{
    removeInactiveModels();
    if(m_robot_filters.size() <= order) return 0;                      // No pruning required if not above maximum.
    m_robot_filters.sort(model_ptr_cmp());                             // Sort, results in order smallest to largest.
    unsigned int num_to_remove = m_robot_filters.size() - order;       // Number of models that need to be removed.

    std::list<IKalmanFilter*>::iterator begin_remove = m_robot_filters.begin();   // Beginning of removal range
    std::list<IKalmanFilter*>::iterator end_remove = m_robot_filters.begin();
    std::advance(end_remove,num_to_remove);                     // End of removal range (not removed)

    std::for_each (begin_remove, end_remove, std::bind2nd(std::mem_fun(&IKalmanFilter::setActive), false));

    int num_removed = removeInactiveModels();    // Clear out all deactivated models.
    assert(m_robot_filters.size() == order);   // Result should have been achieved or something is broken.
    return num_removed;
}

bool AlphaSumPredicate( const ParentSum& a, const ParentSum& b )
{
        return a.second <  b.second;
}

/*! @brief Prunes the models using an N-Scan Pruning method.
    @param N The number of braches backward to prune the decision tree
    @return The number of models that were removed during this process.
*/
int SelfLocalisation::PruneNScan(unsigned int N)
{
    std::vector<ParentSum> results;
    // Sum the alphas of sibling branches from a common parent at branch K-N
    for (std::list<IKalmanFilter*>::iterator model_it = m_robot_filters.begin(); model_it != m_robot_filters.end(); ++model_it)
    {
        if((*model_it)->active() == false) continue;
        unsigned int parent_id = (*model_it)->m_parent_history_buffer.at(N);
        float alpha = (*model_it)->getFilterWeight();
        bool added = false;
        if(parent_id == 0) continue;
        // Sort all of the models
        for(std::vector<ParentSum>::iterator resIt = results.begin(); resIt != results.end(); ++resIt)
        {
            // Assign to existing sum.
            if(resIt->first == parent_id)
            {
                resIt->second += alpha;
                added = true;
                break;
            }
        }
        // If not able to assign to an existing sum create a new one.
        if(!added)
        {
            ParentSum temp = make_pair(parent_id, alpha);
            results.push_back(temp);
        }
    }

    if(results.size() > 0)
    {
        std::sort(results.begin(), results.end(), AlphaSumPredicate); // Sort by alpha sum, smallest to largest
        unsigned int bestParentId = results.back().first;       // Get the parent Id of the best branch.

        // Remove all siblings not created from the best branch.
        for (std::list<IKalmanFilter*>::iterator model_it = m_robot_filters.begin(); model_it != m_robot_filters.end(); ++model_it)
        {
            unsigned int parent_id = (*model_it)->m_parent_history_buffer.at(N);
            if(parent_id == 0) continue;  // was not involved with this branch.
            if(parent_id != bestParentId)
            {
                (*model_it)->setActive(false);
            }
        }
    }
    removeInactiveModels();
    return 0;
}

/*! @brief Performs an ambiguous measurement update using the exhaustive process. This creates a new model for each possible location for the measurement.
    @return Returns RESULT_OK if measurment updates were sucessfully performed, RESULT_OUTLIER if they were not.
*/
int SelfLocalisation::ambiguousLandmarkUpdateExhaustive(AmbiguousObject &ambiguousObject, const vector<StationaryObject*>& possibleObjects)
{
    const float outlier_factor = 0.001;
    std::list<IKalmanFilter*> new_models;
    IKalmanFilter* temp_mod;
    StationaryObject temp_object;

    MeasurementError error = calculateError(ambiguousObject);

    for (std::list<IKalmanFilter*>::iterator model_it = m_robot_filters.begin(); model_it != m_robot_filters.end(); ++model_it)
    {
        if((*model_it)->active() == false) continue;
        unsigned int models_added = 0;
        for(std::vector<StationaryObject*>::const_iterator obj_it = possibleObjects.begin(); obj_it != possibleObjects.end(); ++obj_it)
        {
            temp_object = *(*obj_it);
            temp_object.CopyObject(ambiguousObject);
            temp_mod = newRobotModel(*model_it, temp_object, error, ambiguousObject.getID(), GetTimestamp());
            new_models.push_back(temp_mod);

            if(temp_mod->active())
            {
                models_added++;
            }

            MultivariateGaussian est = (*model_it)->estimate();

#if LOC_SUMMARY_LEVEL > 0

            const double dX = temp_object.X() - est.mean(0);
            const double dY = temp_object.Y() - est.mean(1);

            float expected_distance = sqrt(dX*dX + dY*dY);;
            float expected_heading = mathGeneral::normaliseAngle(atan2(dY,dX) - est.mean(2));
            m_frame_log << "Model [" << (*model_it)->id() << " - > " << temp_mod->id() << "] Ambiguous object update: " << std::string((*obj_it)->getName());
            m_frame_log << " exp (" << expected_distance << ", " << expected_heading << ")  Result: ";
            if(temp_mod->active())
            {
                m_frame_log << "Valid update (Alpha = " << temp_mod->getFilterWeight() << ")";
            }
            else
            {
                m_frame_log << "Outlier";
            }
            m_frame_log << std::endl;
#endif
        }
        removeInactiveModels(new_models);
        if(models_added)
        {
            (*model_it)->setActive(false);
        }
        else
        {
            (*model_it)->setFilterWeight(outlier_factor * (*model_it)->getFilterWeight());
        }
    }
    if(new_models.size() > 0)
    {
        m_robot_filters.insert(m_robot_filters.end(), new_models.begin(), new_models.end());
        new_models.clear();
    }
    removeInactiveModels();
    return 0;
}

/*! @brief Performs an ambiguous measurement update using the constraint method.
    This method uses only options that should be visible.
    @return Returns RESULT_OK if measurment updates were sucessfully performed, RESULT_OUTLIER if they were not.
*/
int SelfLocalisation::ambiguousLandmarkUpdateConstraint(AmbiguousObject &ambiguousObject, const vector<StationaryObject*>& possibleObjects)
{
    const float outlier_factor = 0.001;
    std::list<IKalmanFilter*> new_models;
    IKalmanFilter* temp_mod, *curr_model;

    MeasurementError error = calculateError(ambiguousObject);
    StationaryObject temp_object;

    for (std::list<IKalmanFilter*>::iterator model_it = m_robot_filters.begin(); model_it != m_robot_filters.end(); ++model_it)
    {
        curr_model = (*model_it);
        if(curr_model->active() == false) continue;
        unsigned int models_added = 0;
        vector<StationaryObject*> poss_objects;

        //Self position = curr_model->GenerateSelfState();
        Self position;
        MultivariateGaussian est = curr_model->estimate();
        position.updateLocationOfSelf(est.mean(0), est.mean(1), est.mean(2), est.sd(0), est.sd(1), est.sd(2), false);

        float headYaw;
        Blackboard->Sensors->getPosition(NUSensorsData::HeadYaw, headYaw);
        //! TODO: The FOV of the camera should NOT be hard-coded!
        poss_objects = filterToVisible(position, possibleObjects, headYaw, 0.81f);

        for(std::vector<StationaryObject*>::const_iterator obj_it = poss_objects.begin(); obj_it != poss_objects.end(); ++obj_it)
        {
            temp_object = *(*obj_it);
            temp_object.CopyObject(ambiguousObject);
            temp_mod = newRobotModel(curr_model, temp_object, error, ambiguousObject.getID(), GetTimestamp());
            new_models.push_back(temp_mod);
            if(temp_mod->active())
                models_added++;

#if LOC_SUMMARY_LEVEL > 0
            m_frame_log << "Model [" << curr_model->id() << " - > " << temp_mod->id() << "] Ambiguous object update: " << std::string((*obj_it)->getName());
            m_frame_log << "  Result: " << (temp_mod->active() ? "Valid update" : "Outlier");
            if(temp_mod->active())
            {
                m_frame_log << "Valid update (Alpha = " << temp_mod->getFilterWeight() << ")";
            }
            else
            {
                m_frame_log << "Outlier";
            }
            m_frame_log << std::endl;
#endif
        }
        removeInactiveModels(new_models);
        if(models_added)
        {
            curr_model->setActive(false);
        }
        else // outlier
        {
            curr_model->setFilterWeight(outlier_factor * (*model_it)->getFilterWeight());
        }
    }
    if(new_models.size() > 0)
    {
        m_robot_filters.insert(m_robot_filters.end(), new_models.begin(), new_models.end());
        new_models.clear();
    }
    removeInactiveModels();
    return 0;
}

int SelfLocalisation::ambiguousLandmarkUpdateSelective(AmbiguousObject &ambiguousObject, const vector<StationaryObject*>& possibleObjects)
{
    const float c_Max_Elapsed_Time = 100.0f;
    const float outlier_factor = 0.001;

    // First step: Check if the same ambiguous type has beem seen recently.
    float time_elapsed = ambiguousObject.TimeLastSeen() - m_pastAmbiguous[ambiguousObject.getID()].TimeLastSeen();
    bool seen_recently = (time_elapsed < c_Max_Elapsed_Time);
    bool similar_meas_found = false;
    if(seen_recently)
    {
        // Second step: Check if the previous measurment was consistent with the current measurment

        // Calculate the distances between the two measurments
        float flatDistancePrev = m_pastAmbiguous[ambiguousObject.getID()].measuredDistance() * cos(m_pastAmbiguous[ambiguousObject.getID()].measuredElevation());
        float flatDistanceCurr = ambiguousObject.measuredDistance() * cos(ambiguousObject.measuredElevation());
        float dist_delta = fabs(flatDistanceCurr - flatDistancePrev);
        float heading_delta = fabs(ambiguousObject.measuredBearing() - m_pastAmbiguous[ambiguousObject.getID()].measuredBearing());

        // Calculate the allowable deviation
        MeasurementError error = calculateError(ambiguousObject);
        float max_distance_deviation = error.distance();
        float max_heading_deviation = error.heading();

        similar_meas_found = (dist_delta < max_distance_deviation) && (heading_delta < max_heading_deviation);
        if(similar_meas_found)
        {
            // Third Step (A): Apply mesurment using previous decision if measurement is consistant.
            std::list<IKalmanFilter*> new_models;
            IKalmanFilter* temp_model = NULL;

            for (std::list<IKalmanFilter*>::iterator model_it = m_robot_filters.begin(); model_it != m_robot_filters.end(); ++model_it)
            {
                // Get the id of the object to use for the update.
                unsigned int object_id = (*model_it)->previousSplitOption(ambiguousObject);
                // Find the option that matches the previous decision
                for(std::vector<StationaryObject*>::const_iterator obj_it = possibleObjects.begin(); obj_it != possibleObjects.end(); ++obj_it)
                {
                    if((*obj_it)->getID() == object_id)
                    {
                        // Perform the update
                        StationaryObject update_object(*(*obj_it));
                        update_object.CopyMeasurement(ambiguousObject);
                        temp_model = newRobotModel((*model_it), update_object, error, ambiguousObject.getID(), GetTimestamp());
                        new_models.push_back(temp_model);  // add the model to the new list.
                        if(temp_model->active())
                        {
                            (*model_it)->setActive(false); // disable the old model.
                        }
                        else
                        {
                            (*model_it)->setFilterWeight(outlier_factor * (*model_it)->getFilterWeight());
                        }
                    }
                }
                // Check if model has been updated yet
                // If not need to do a exhaustive update for this model
                if((*model_it)->active())
                {
                    bool update_performed = false;
                    for(vector<StationaryObject*>::const_iterator option_it = possibleObjects.begin(); option_it != possibleObjects.end(); ++option_it)
                    {
                        StationaryObject update_object(*(*option_it));
                        update_object.CopyMeasurement(ambiguousObject);
                        temp_model = newRobotModel((*model_it), update_object, error, ambiguousObject.getID(), GetTimestamp());
                        new_models.push_back(temp_model);
                        if(temp_model->active())
                        {
                            update_performed = true;
                        }
                    }
                    if(update_performed)
                    {
                        (*model_it)->setActive(false);
                    }
                }
            }
            removeInactiveModels(new_models);
            m_robot_filters.insert(m_robot_filters.begin(),new_models.begin(), new_models.end());
        }
    }

    int return_value = 0;
    if(!similar_meas_found or !similar_meas_found)
    {
        // Third Step (B): Perfrom splitting if measurement was not previously seen or not consistant.
        // Use the regular splitting method.
        return_value = ambiguousLandmarkUpdateExhaustive(ambiguousObject, possibleObjects);
    }
    // Save new info from object
    m_pastAmbiguous[ambiguousObject.getID()] = ambiguousObject;
    removeInactiveModels();
    return return_value;
}

int SelfLocalisation::ambiguousLandmarkUpdateProbDataAssoc(AmbiguousObject &ambiguousObject, const vector<StationaryObject*>& possibleObjects)
{
    MeasurementError error = calculateError(ambiguousObject);
    //result = m_robot_filters.front()->MeasurementUpdate(ambiguousObject, possibleObjects, error);
    removeInactiveModels();
    return 0;
}

bool SelfLocalisation::MergeTwoModels(IKalmanFilter* model_a, IKalmanFilter* model_b)
{
    // Merges second model into first model, then disables second model.
    bool success = true;
    if(model_a == model_b) success = false; // Don't merge the same model.
    if((not model_a->active()) or (not model_b->active())) success = false; // Both models must be active.

    if(success == false)
    {
        return success;
    }

    // Merge alphas
    double newest_creation_time = model_a->creationTime() > model_b->creationTime() ? model_a->creationTime() : model_b->creationTime();
    double alphaMerged = model_a->getFilterWeight() + model_b->getFilterWeight();
    double alphaA = model_a->getFilterWeight() / alphaMerged;
    double alphaB = model_b->getFilterWeight() / alphaMerged;

    MultivariateGaussian estimate_a = model_a->estimate();
    MultivariateGaussian estimate_b = model_b->estimate();

    Matrix xMerged; // Merge State matrix

    // If one model is much more correct than the other, use the correct states.
    // This prevents drifting from continuouse splitting and merging even when one model is much more likely.
    if(alphaA > 10*alphaB)
    {
        xMerged = estimate_a.mean();
    }
    else if (alphaB > 10*alphaA)
    {
        xMerged = estimate_b.mean();
    }
    else
    {
        xMerged = (alphaA * estimate_a.mean() + alphaB * estimate_b.mean());
        // Fix angle.
        double angleDiff = estimate_b.mean(RobotModel::kstates_heading) - estimate_a.mean(RobotModel::kstates_heading);
        angleDiff = normaliseAngle(angleDiff);
        xMerged[RobotModel::kstates_heading][0] = normaliseAngle(estimate_a.mean(RobotModel::kstates_heading) + alphaB*angleDiff);
    }

    // Merge Covariance matrix (S = sqrt(P))
    Matrix xDiff = estimate_a.mean() - xMerged;
    Matrix pA = (estimate_a.covariance() + xDiff * xDiff.transp());

    xDiff = estimate_b.mean() - xMerged;
    Matrix pB = (estimate_b.covariance() + xDiff * xDiff.transp());

    Matrix pMerged = alphaA * pA + alphaB * pB;

    // Copy merged value to first model
    model_a->setFilterWeight(alphaMerged);
    model_a->m_creation_time = newest_creation_time;

    estimate_a.setMean(xMerged);
    estimate_b.setCovariance(pMerged);

    model_a->initialiseEstimate(estimate_a);

    // Disable second model
    model_a->setActive(true);
    model_b->setActive(false);
    return true;
}

bool SelfLocalisation::MergeTwoModelsPreserveBestMean(IKalmanFilter* model_a, IKalmanFilter* model_b)
{
    // Merges second model into first model, then disables second model.
    bool success = true;
    if(model_a == model_b) success = false; // Don't merge the same model.
    if((not model_a->active()) or (not model_b->active())) success = false; // Both models must be active.

    if(success == false)
    {
#if LOC_SUMMARY_LEVEL > 0
        m_frame_log  <<"[" << m_timestamp << "]: Merge Between model[" << model_a->id() << "] and model[" << model_b->id() << "] FAILED." << endl;
        if(model_a == model_b)
        {
            m_frame_log << "Reason - index: " << model_a->id() << " = " << "index: " << model_b->id() << std::endl;
        } else if (model_a->active() == false)
        {
            m_frame_log << "Reason - model " << model_a->id() << " is inactive."<< std::endl;
        } else if (model_b->active() == false)
        {
            m_frame_log << "Reason - model " << model_b->id() << " is inactive."<< std::endl;
        }
#endif // LOC_SUMMARY_LEVEL > 0
#if DEBUG_LOCALISATION_VERBOSITY > 2
        cout << "Merge failed." <<std::endl;
        debug_out  <<"[" << m_timestamp << "]: Merge Between model[" << model_a->id() << "] and model[" << model_b->id() << "] FAILED." << endl;
#endif // DEBUG_LOCALISATION_VERBOSITY > 0
        return success;
    }

    // Merge alphas
    double newest_creation_time = model_a->creationTime() > model_b->creationTime() ? model_a->creationTime() : model_b->creationTime();
    double alphaMerged = model_a->getFilterWeight() + model_b->getFilterWeight();
    double alphaA = model_a->getFilterWeight() / alphaMerged;
    double alphaB = model_b->getFilterWeight() / alphaMerged;

    MultivariateGaussian estimate_a = model_a->estimate();
    MultivariateGaussian estimate_b = model_b->estimate();

    Matrix xMerged; // Merge State matrix

    // If one model is much more correct than the other, use the correct states.
    // This prevents drifting from continuouse splitting and merging even when one model is much more likely.
    if(alphaA > alphaB)
    {
        xMerged = estimate_a.mean();
    }
    else
    {
        xMerged = estimate_b.mean();
    }

    // Merge Covariance matrix (S = sqrt(P))
    Matrix xDiff = estimate_a.mean() - xMerged;
    Matrix pA = (estimate_a.covariance() + xDiff * xDiff.transp());

    xDiff = estimate_b.mean() - xMerged;
    Matrix pB = (estimate_b.covariance() + xDiff * xDiff.transp());

    Matrix pMerged = alphaA * pA + alphaB * pB;

    // Copy merged value to first model
    model_a->setFilterWeight(alphaMerged);
    model_a->m_creation_time = newest_creation_time;

    estimate_a.setMean(xMerged);
    estimate_b.setCovariance(pMerged);

    model_a->initialiseEstimate(estimate_a);

    // Disable second model
    model_a->setActive(true);
    model_b->setActive(false);
    return true;
}

double SelfLocalisation::MergeMetric(const IKalmanFilter* model_a, const IKalmanFilter* model_b) const
{
    if (model_a==model_b) return 10000.0;   // The same model
    if ((not model_a->active()) or (not model_b->active())) return 10000.0; //at least one model inactive
//    return QuinlanMetric(model_a, model_b);
//    return WilliamsMetric(model_a, model_b)*1e3;
//    return SalmondMetric(model_a, model_b);
    return RunnallMetric(model_a, model_b);

}


double MVE(const Matrix& x1, const Matrix& x2, const Matrix P1plusP2)
{
    Matrix xdiff = x1 - x2;
    xdiff[RobotModel::kstates_heading][0] = normaliseAngle(xdiff[RobotModel::kstates_heading][0]);

    double multiplier = pow(determinant(2*mathGeneral::PI*P1plusP2),-0.5);
    double exponent = -0.5 * convDble(xdiff.transp() * InverseMatrix(P1plusP2) * xdiff);

    return multiplier * exp(exponent);

}

bool SelfLocalisation::MetricTest()
{
    IKalmanFilter* a = newRobotModel();
    IKalmanFilter* b = newRobotModel();
    a->setFilterWeight(0.5);
    b->setFilterWeight(0.5);

    Matrix amean(1,1);
    Matrix bmean(1,1);

    Matrix acov(1,1);
    Matrix bcov(1,1);

    amean[0][0] = 0;
    bmean[0][0] = 0;

    acov[0][0] = 1;
    bcov[0][0] = 6;

    a->initialiseEstimate(MultivariateGaussian(amean, acov));
    b->initialiseEstimate(MultivariateGaussian(bmean, bcov));

    std::cout << "Test 1" << std::endl;
    std::cout << "-Model A-" << std::endl << "Mean:" << std::endl << amean << std::endl << "Covariance:" << std::endl << acov << std::endl;
    std::cout << "-Model B-" << std::endl << "Mean:" << std::endl << bmean << std::endl << "Covariance:" << std::endl << bcov << std::endl;

    std::cout << "Salmond = " << SalmondMetric(a,b) << std::endl;
    std::cout << "Williams = " << WilliamsMetric(a,b) << std::endl;
    std::cout << "Runnall = " << RunnallMetric(a,b) << std::endl;

    amean[0][0] = -0.1;
    bmean[0][0] = 0.1;

    acov[0][0] = 1;
    bcov[0][0] = 1;

    a->initialiseEstimate(MultivariateGaussian(amean, acov));
    b->initialiseEstimate(MultivariateGaussian(bmean, bcov));

    std::cout << "Test 2" << std::endl;
    std::cout << "-Model A-" << std::endl << "Mean:" << std::endl << amean << std::endl << "Covariance:" << std::endl << acov << std::endl;
    std::cout << "-Model B-" << std::endl << "Mean:" << std::endl << bmean << std::endl << "Covariance:" << std::endl << bcov << std::endl;

    std::cout << "Salmond = " << SalmondMetric(a,b) << std::endl;
    std::cout << "Williams = " << WilliamsMetric(a,b) << std::endl;
    std::cout << "Runnall = " << RunnallMetric(a,b) << std::endl;
    return true;
}

double SelfLocalisation::WilliamsMetric(const IKalmanFilter* model_a, const IKalmanFilter* model_b) const
{
    double wa = model_a->getFilterWeight();
    double wb = model_b->getFilterWeight();
    double wab = wa + wb;

    Matrix xa = model_a->estimate().mean();
    Matrix xb = model_b->estimate().mean();

    Matrix Pa = model_a->estimate().covariance();
    Matrix Pb = model_b->estimate().covariance();

    Matrix xdiff = xa - xb;
    Matrix Pab = wa / wab * Pa + wb / wab * Pb + wa * wb / (wab * wab) * xdiff * xdiff.transp();
    Matrix xab = (wa*xa + wb*xb) / wab;


    double Jhh = wa*wa*MVE(xa, xa, Pa + Pa) + wa*wb*MVE(xa, xb, Pa + Pb) + wb*wa*MVE(xb, xa, Pb + Pa) + wb*wb*MVE(xb, xb, Pb + Pb);
    double Jhr = wa*wab*MVE(xa, xab, Pa + Pab) + wb*wab*MVE(xb, xab, Pb + Pab);
    double Jrr = wab*wab*MVE(xab, xab, Pab + Pab);
    return Jhh - 2 * Jhr + Jrr;
}

double SelfLocalisation::SalmondMetric(const IKalmanFilter* model_a, const IKalmanFilter* model_b) const
{
    double wa = model_a->getFilterWeight();
    double wb = model_b->getFilterWeight();
    double wab = wa + wb;

    Matrix xdiff = model_a->estimate().mean() - model_b->estimate().mean();
    Matrix Pab = wa / wab * model_a->estimate().covariance() + wb / wab * model_b->estimate().covariance() + wa * wb / (wab * wab) * xdiff * xdiff.transp();

    return wa * wb / wab * convDble(xdiff.transp() * InverseMatrix(Pab) * xdiff);
}

double SelfLocalisation::RunnallMetric(const IKalmanFilter* model_a, const IKalmanFilter* model_b) const
{
    double wa = model_a->getFilterWeight();
    double wb = model_b->getFilterWeight();
    double wab = wa + wb;

    Matrix xdiff = model_a->estimate().mean() - model_b->estimate().mean();
    Matrix Pab = wa / wab * model_a->estimate().covariance() + wb / wab * model_b->estimate().covariance() + wa * wb / (wab * wab) * xdiff * xdiff.transp();

    return 0.5 * (wab*log(determinant(Pab)) - wa*log(determinant(model_a->estimate().covariance())) - wb*log(determinant(model_b->estimate().covariance())));
}

double SelfLocalisation::QuinlanMetric(const IKalmanFilter* model_a, const IKalmanFilter* model_b) const
{
    MultivariateGaussian estimate_a = model_a->estimate();
    MultivariateGaussian estimate_b = model_b->estimate();
    Matrix xdif = estimate_a.mean() - estimate_b.mean();
    Matrix p1 = estimate_a.covariance();
    Matrix p2 = estimate_b.covariance();

    xdif[RobotModel::kstates_heading][0] = normaliseAngle(xdif[RobotModel::kstates_heading][0]);

    double dij=0;
    for (int i=0; i<p1.getm(); i++) {
        dij+=(xdif[i][0]*xdif[i][0]) / (p1[i][i]+p2[i][i]);
    }
    return dij*( (model_a->getFilterWeight()*model_b->getFilterWeight()) / (model_a->getFilterWeight()+model_b->getFilterWeight()) );
}

/*! @brief Determines the number of active models.
    @returnThe number of models active.
*/
unsigned int SelfLocalisation::getNumActiveModels()
{
    unsigned int numActive = 0;
    for (std::list<IKalmanFilter*>::const_iterator model_it = m_robot_filters.begin(); model_it != m_robot_filters.end(); ++model_it)
    {
        if((*model_it)->active() == true) numActive++;
    }
    return numActive;
}

/*! @brief Return the number of free models.
    @return The number of models available.
*/
unsigned int SelfLocalisation::getNumFreeModels()
{
    return c_MAX_MODELS - getNumActiveModels();
}

/*! @brief Comparison function for two models.
    @param modelA The first model.
    @param modelB The second model.
    @return True if modelA is smaller than modelB. False if Model A is greater than or equal to ModelB.
*/
bool model_alpha_lesser(IKalmanFilter* modelA, IKalmanFilter* modelB)
{
    return modelA->getFilterWeight() < modelB->getFilterWeight();
}

/*! @brief Retrieve the best available model.
    @return A pointer to the best available model.
*/
const IKalmanFilter* SelfLocalisation::getBestModel() const
{
    assert(m_robot_filters.size() > 0);
    IKalmanFilter* result;
    std::list<IKalmanFilter*>::const_iterator best_mod_it;

    best_mod_it = max_element(m_robot_filters.begin(), m_robot_filters.end(), model_alpha_lesser);
    result = (*best_mod_it);
    return result;
}

const IKalmanFilter* SelfLocalisation::getBallModel() const
{
    return m_ball_filter;
}

/*! @brief Normalises the alphas of all exisiting models.
    The alphas of all existing models are normalised so that the total probablility of the set sums to 1.0.
*/
void SelfLocalisation::NormaliseAlphas()
{
    // Normalise all of the models alpha values such that all active models sum to 1.0
    double sumAlpha=0.0;
    double weight;
    for (std::list<IKalmanFilter*>::iterator model_it = m_robot_filters.begin(); model_it != m_robot_filters.end(); ++model_it)
    {
        if ((*model_it)->active())
        {
            sumAlpha += (*model_it)->getFilterWeight();
        }
    }
    if(sumAlpha == 1) return;
    if (sumAlpha == 0) sumAlpha = 1e-12;
    for (std::list<IKalmanFilter*>::iterator model_it = m_robot_filters.begin(); model_it != m_robot_filters.end(); ++model_it)
    {
        if ((*model_it)->active())
        {
            weight = (*model_it)->getFilterWeight()/sumAlpha;
            (*model_it)->setFilterWeight(weight);
        }
    }


}

//**************************************************************************
//  This method begins the process of merging close models together
/*! @brief Merges models until the number active is below the maximum given.
    @param maxAfterMerge The maximum number of models that will remain after the mreging process.
*/
void SelfLocalisation::MergeModels(int maxAfterMerge)
{
    MergeModelsBelowThreshold(0.001);
    double threshold=0.01;
    while (getNumActiveModels()>maxAfterMerge)
    {
        MergeModelsBelowThreshold(threshold);
        threshold*=5.0;
    }
    removeInactiveModels();
    return;
}


/*! @brief Creates a summary of the localisation in human readable string form.

    @return A string containing the summary.
*/
std::string SelfLocalisation::ModelStatusSummary()
{
    std::stringstream temp;
    temp << "Active model summary." << std::endl;
    for (std::list<IKalmanFilter*>::iterator model_it = m_robot_filters.begin(); model_it != m_robot_filters.end(); ++model_it)
    {
            temp << (*model_it)->summary(false);
    }
    return temp.str();
}

/*! @brief Prints a models status.

    @param model The model
*/
void SelfLocalisation::PrintModelStatus(const IKalmanFilter *model)
{
#if DEBUG_LOCALISATION_VERBOSITY > 2
  debug_out  <<"[" << m_currentFrameNumber << "]: Model[" << model->id() << "]";
  debug_out  << "[alpha=" << model->getFilterWeight() << "]";
  debug_out  << " active = " << model->active() << endl;
#endif
  return;
}

/*! @brief Merges all model pairs with a merge metric below the given threshold.

    @param MergeMetricThreshold The threshold.
    @return The merge metric of the pair.
*/
void SelfLocalisation::MergeModelsBelowThreshold(double MergeMetricThreshold)
{
    double mergeM;
    // New models
    for (std::list<IKalmanFilter*>::iterator i = m_robot_filters.begin(); i != m_robot_filters.end(); ++i)
    {
        for (std::list<IKalmanFilter*>::iterator j = i; j != m_robot_filters.end(); ++j)
        {
            if((*i) == (*j))
            {
                continue;
            }
            IKalmanFilter* modelA = (*i);
            IKalmanFilter* modelB = (*j);
            if ((modelA->active()==false) or (modelB->active()==false))
            {
                continue;
            }
            mergeM = abs( MergeMetric(modelA,modelB) );
            if (mergeM <= MergeMetricThreshold)
            { //0.5
#if LOC_SUMMARY_LEVEL > 0
            m_frame_log << "Model " << (*j)->id() << " merged into " << (*i)->id() << std::endl;
#endif
#if DEBUG_LOCALISATION_VERBOSITY > 2
                debug_out  <<"[" << m_currentFrameNumber << "]: Merging Model[" << modelB->id() << "][alpha=" << modelB->getFilterWeight() << "]";
                debug_out  << " into Model[" << modelA->id() << "][alpha=" << modelA->getFilterWeight() << "] " << " Merge Metric = " << mergeM << endl  ;
#endif
                MergeTwoModels(modelA,modelB);
            }
        }
    }
    return;
}

bool SelfLocalisation::operator ==(const SelfLocalisation& b) const
{
    if(m_timestamp != b.m_timestamp) return false;
    if(m_currentFrameNumber != b.m_currentFrameNumber) return false;
    if(m_previously_incapacitated != b.m_previously_incapacitated) return false;
    if(m_previous_game_state != b.m_previous_game_state) return false;
    if(m_hasGps != b.m_hasGps) return false;
    if(m_hasGps)
    {
        if(m_compass != b.m_compass) return false;
        if(m_gps[0] != b.m_gps[0]) return false;
        if(m_gps[1] != b.m_gps[1]) return false;
    }
    if(*m_ball_filter != *b.m_ball_filter) return false;

    unsigned int num_robot_filters = m_robot_filters.size();
    if(num_robot_filters != b.m_robot_filters.size()) return false;
    std::list<IKalmanFilter*>::const_iterator local_model = m_robot_filters.begin();
    std::list<IKalmanFilter*>::const_iterator b_model = b.m_robot_filters.begin();
    for (unsigned int i=0; i < num_robot_filters; ++i)
    {
        if(*(*local_model) != *(*b_model)) return false;
        ++local_model;
        ++b_model;
    }
    return true;
}

/*!
@brief Outputs a binary representation of the Self Localisation system.
@param output The output stream.
@return The output stream.
*/
std::ostream& SelfLocalisation::writeStreamBinary (std::ostream& output) const
{
    char head = header();
    output.write(&head, sizeof(head));
    output.write(reinterpret_cast<const char*>(&m_timestamp), sizeof(m_timestamp));
    output.write(reinterpret_cast<const char*>(&m_currentFrameNumber), sizeof(m_currentFrameNumber));
    output.write(reinterpret_cast<const char*>(&m_previously_incapacitated), sizeof(m_previously_incapacitated));
    output.write(reinterpret_cast<const char*>(&m_previous_game_state), sizeof(m_previous_game_state));
    output.write(reinterpret_cast<const char*>(&m_hasGps), sizeof(m_hasGps));
    if(m_hasGps)
    {
        output.write(reinterpret_cast<const char*>(&m_compass), sizeof(m_compass));
        output.write(reinterpret_cast<const char*>(&m_gps[0]), sizeof(m_gps[0]));
        output.write(reinterpret_cast<const char*>(&m_gps[1]), sizeof(m_gps[1]));
    }

    output.write(reinterpret_cast<const char*>(&m_settings), sizeof(m_settings));

    // Write the ball model
    m_ball_filter->writeStreamBinary(output);

    // Write the slef localisation models.
    unsigned int num_robot_filters = m_robot_filters.size();
    output.write(reinterpret_cast<const char*>(&num_robot_filters), sizeof(num_robot_filters));
    for (std::list<IKalmanFilter*>::const_iterator model_it = m_robot_filters.begin(); model_it != m_robot_filters.end(); ++model_it)
    {
        IKalmanFilter* currModel = (*model_it);
        currModel->writeStreamBinary(output);
    }
    return output;
}

/*!
@brief Reads in a Self localisation system from the input stream.
@param input The input stream.
@return The input stream.
*/
std::istream& SelfLocalisation::readStreamBinary (std::istream& input)
{
    char header;
    input.read(&header, sizeof(header));
    input.read(reinterpret_cast<char*>(&m_timestamp), sizeof(m_timestamp));
    input.read(reinterpret_cast<char*>(&m_currentFrameNumber), sizeof(m_currentFrameNumber));
    input.read(reinterpret_cast<char*>(&m_previously_incapacitated), sizeof(m_previously_incapacitated));
    input.read(reinterpret_cast<char*>(&m_previous_game_state), sizeof(m_previous_game_state));
    input.read(reinterpret_cast<char*>(&m_hasGps), sizeof(m_hasGps));
    if(m_hasGps)
    {
        input.read(reinterpret_cast<char*>(&m_compass), sizeof(m_compass));
        input.read(reinterpret_cast<char*>(&m_gps[0]), sizeof(m_gps[0]));
        input.read(reinterpret_cast<char*>(&m_gps[1]), sizeof(m_gps[1]));
    }

    input.read(reinterpret_cast<char*>(&m_settings), sizeof(m_settings));

    // Read the ball model
    m_ball_filter->readStreamBinary(input);

    // Write the slef localisation models.
    unsigned int num_filters;
    input.read(reinterpret_cast<char*>(&num_filters), sizeof(num_filters));
    clearModels();
    for (unsigned int i = 0; i < num_filters; ++i)
    {
        IKalmanFilter* currModel = newRobotModel();
        currModel->readStreamBinary(input);
        m_robot_filters.push_back(currModel);
    }
    return input;
}

/*! @brief output stream
    Writes the localisation object to an output stream
    @param output The output stream to which the data will be written
    @param p_loc The localisation to be writen.
    @return The ouput stream
*/
std::ostream& operator<< (std::ostream& output, const SelfLocalisation& p_loc)
{
    p_loc.writeStreamBinary(output);
    return output;
}

/*! @brief input stream
    Reads in the localisation from an input stream.
    @param input The input stream fro which data will be read.
    @param p_loc The localisation to which the data will be written.
    @return The input stream
*/
std::istream& operator>> (std::istream& input, SelfLocalisation& p_loc)
{
    p_loc.readStreamBinary(input);
    return input;
}

/*! @brief Clears all of the current models.

 */
void SelfLocalisation::clearModels()
{
    for(std::list<IKalmanFilter*>::iterator loc_it = m_robot_filters.begin(); loc_it != m_robot_filters.end(); ++loc_it)
    {
        delete (*loc_it);
    }
    m_robot_filters.clear();
}


float translation_distance(const MultivariateGaussian& a, const MultivariateGaussian& b)
{
    float diff_x = a.mean(RobotModel::kstates_x) - b.mean(RobotModel::kstates_x);
    float diff_y = a.mean(RobotModel::kstates_y) - b.mean(RobotModel::kstates_y);
    return sqrt(diff_x * diff_x + diff_y * diff_y);
}

float heading_distance(const MultivariateGaussian& a, const MultivariateGaussian& b)
{
    float diff_head = a.mean(RobotModel::kstates_heading) - b.mean(RobotModel::kstates_heading);
    return diff_head;
}

void SelfLocalisation::removeSimilarModels()
{
    const float min_trans_dist = 5;
    const float min_head_dist = 0.01;

    for(std::list<IKalmanFilter*>::iterator iter1 = m_robot_filters.begin(); iter1 != m_robot_filters.end(); ++iter1)
    {
        if(!(*iter1)->active()) continue;
        for(std::list<IKalmanFilter*>::iterator iter2 = m_robot_filters.begin(); iter2 != m_robot_filters.end(); ++iter2)
        {
            if(!(*iter2)->active()) continue;
            if(iter1 == iter2) continue;    // don't compare the same models.
            float trans_dist = translation_distance((*iter1)->estimate(), (*iter2)->estimate());
            float head_dist = heading_distance((*iter1)->estimate(), (*iter2)->estimate());
            if( (trans_dist < min_trans_dist) and (head_dist < min_head_dist))
            {
                float total_alpha = (*iter1)->getFilterWeight() + (*iter2)->getFilterWeight();
                if((*iter1)->getFilterWeight() < (*iter2)->getFilterWeight())
                {
                    (*iter1)->setActive(false);
                    (*iter2)->setFilterWeight(total_alpha);
                }
                else
                {
                    (*iter1)->setFilterWeight(total_alpha);
                    (*iter2)->setActive(false);
                }
            }
        }
    }
    removeInactiveModels();
    NormaliseAlphas();
    return;
}


/*! @brief Initialises all of the models desribed in the vector and weights them evenly.

    @param positions A vector containing any number of moments (measurements) of position.
*/
void SelfLocalisation::InitialiseModels(const std::vector<MultivariateGaussian>& positions)
{
    if(positions.size() <= 0) return;      // Don't do anything if no positions are given.
    const float split_alpha = 1.0f / positions.size();            // Uniform split alpha for all models.
    IKalmanFilter* temp;
    IKalmanFilter* filter;
    clearModels();

#if LOC_SUMMARY_LEVEL > 0
    m_frame_log << "Intitialising models." << std::endl;
    m_frame_log << "Positions:" << std::endl;
    for (std::vector<MultivariateGaussian>::const_iterator pos_it = positions.begin(); pos_it != positions.end(); ++pos_it)
    {
        m_frame_log << "(" << (*pos_it).mean(RobotModel::kstates_x) << "," << (*pos_it).mean(RobotModel::kstates_y) << "," << (*pos_it).mean(RobotModel::kstates_heading);
        m_frame_log << ") - (" << (*pos_it).sd(RobotModel::kstates_x) << "," << (*pos_it).sd(RobotModel::kstates_y) << "," << (*pos_it).sd(RobotModel::kstates_heading) << std::endl;
    }
#endif

    for (std::vector<MultivariateGaussian>::const_iterator pos = positions.begin(); pos != positions.end(); pos++)
    {
        filter = newRobotModel();
        filter->setFilterWeight(split_alpha);
        filter->setActive(true);
        filter->initialiseEstimate(*pos);
        m_robot_filters.push_back(filter);
    }
    return;
}


void SelfLocalisation::setModels(std::list<IKalmanFilter*>& newModels)
{
    clearModels();
    m_robot_filters = newModels;
    return;
}

void SelfLocalisation::addToBallVariance(float x_pos_var, float y_pos_var, float x_vel_var, float y_vel_var)
{
    Matrix cov = m_ball_filter->estimate().covariance();
    const unsigned int total_states = m_ball_filter->model()->totalStates();

    // Create a matrix for the addative noise.
    Matrix additiveNoise(total_states, total_states, false);
    additiveNoise[MobileObjectModel::kstates_x_pos][MobileObjectModel::kstates_x_pos] = x_pos_var;
    additiveNoise[MobileObjectModel::kstates_y_pos][MobileObjectModel::kstates_y_pos] = y_pos_var;
    additiveNoise[MobileObjectModel::kstates_x_vel][MobileObjectModel::kstates_x_vel] = x_vel_var;
    additiveNoise[MobileObjectModel::kstates_y_vel][MobileObjectModel::kstates_y_vel] = y_vel_var;

    // Add the extra variance
    cov = cov + additiveNoise;

    MultivariateGaussian est = m_ball_filter->estimate();
    cov = est.covariance();
    cov = cov + additiveNoise;
    est.setCovariance(cov);
    m_ball_filter->initialiseEstimate(est);
    return;
}

void SelfLocalisation::setBallVariance(float x_pos_var, float y_pos_var, float x_vel_var, float y_vel_var)
{
    const unsigned int total_states = m_ball_filter->model()->totalStates();
    // Create a matrix for the addative noise.
    Matrix cov(total_states, total_states, false);
    cov[MobileObjectModel::kstates_x_pos][MobileObjectModel::kstates_x_pos] = x_pos_var;
    cov[MobileObjectModel::kstates_y_pos][MobileObjectModel::kstates_y_pos] = y_pos_var;
    cov[MobileObjectModel::kstates_x_vel][MobileObjectModel::kstates_x_vel] = x_vel_var;
    cov[MobileObjectModel::kstates_y_vel][MobileObjectModel::kstates_y_vel] = y_vel_var;

    MultivariateGaussian est = m_ball_filter->estimate();
    est.setCovariance(cov);
    m_ball_filter->initialiseEstimate(est);
    return;
}

/*! @brief Create a 3x1 mean matrix with value as defiend by the parameters.

Creates a 3x1 matrix with the mean of each attribute set as specified.

@param x_var Mean in the x state.
@param y_var Mean in the y state.
@param heading_var Mean in the heading state.
@retun A matrix implementation of the described mean vector.
*/
Matrix SelfLocalisation::mean_matrix(float x, float y, float heading)
{
    Matrix temp_mean(RobotModel::kstates_total, 1, false);
    temp_mean[RobotModel::kstates_x][0] = x;
    temp_mean[RobotModel::kstates_y][0] = y;
    temp_mean[RobotModel::kstates_heading][0] = heading;
    return temp_mean;
}

/*! @brief Create a 3x3 covariance matrix with diagonals as defiend by the parameters.

Creates a 3x3 covariance matrix with the variance of each attribute set as specified.

@param x_var Variance in the x state.
@param y_var Variance in the y state.
@param heading_var Variance in the heading state.
@retun A matrix implementation of the described covariance.
*/
Matrix SelfLocalisation::covariance_matrix(float x_var, float y_var, float heading_var)
{
    Matrix temp_cov(RobotModel::kstates_total, RobotModel::kstates_total, false);
    temp_cov[RobotModel::kstates_x][RobotModel::kstates_x] = x_var;
    temp_cov[RobotModel::kstates_y][RobotModel::kstates_y] = y_var;
    temp_cov[RobotModel::kstates_heading][RobotModel::kstates_heading] = heading_var;
    return temp_cov;
}


/*! @brief Remove Inactive model from the default container

@retun The number of models removed.
*/
unsigned int SelfLocalisation::removeInactiveModels()
{
    unsigned int result = removeInactiveModels(m_robot_filters);  // Remove inactive models from the default container.
    unsigned int numActive = getNumActiveModels();
    assert(m_robot_filters.size() == numActive);
    return result;
}

bool is_null(const IKalmanFilter* pointer)
{
    return (pointer == NULL);
}

/*! @brief Remove Inactive model from a specified container.

Iterates through the container and removes any inactive models found.

@param container The container to remove inactive models from.
@retun The number of models removed.
*/
unsigned int SelfLocalisation::removeInactiveModels(std::list<IKalmanFilter*>& container)
{
    const unsigned int num_before = container.size();   // Save original size

    for (std::list<IKalmanFilter*>::iterator model_it = container.begin(); model_it != container.end(); ++model_it)
    {
        if((*model_it)->active()==false)
        {
            delete (*model_it);
            (*model_it) = NULL;
        }
    }
    container.erase(remove_if(container.begin(), container.end(), is_null), container.end());
    return num_before - container.size();               // Return number removed: original size - new size
}

/*!
@brief Calculate the variance in the given measurement based on the objects distance.
@param theObject The object the error variance is to be calculated for.
@return The error of the measurement given as the variance.
*/
MeasurementError SelfLocalisation::calculateError(const Object& theObject)
{
    MeasurementError error;
    error.setDistance(c_obj_range_offset_variance + c_obj_range_relative_variance * pow(theObject.measuredDistance() * cos(theObject.measuredElevation()),2));
    error.setHeading(c_obj_theta_variance);
    return error;
}

Vector2<float> SelfLocalisation::TriangulateTwoObject(const StationaryObject& object1, const StationaryObject& object2)
{
    // Algorithm from http://local.wasp.uwa.edu.au/~pbourke/geometry/2circle/
    Vector2<float> p0, p1, p2, p3a, p3b, p3;
    p0.x = object1.X();
    p0.y = object1.Y();

    p1.x = object2.X();
    p1.y = object2.Y();

    float r0 = object1.measuredDistance();
    float r1 = object2.measuredDistance();

    float d = sqrt(pow(p0.x - p1.x,2) + pow(p0.y - p1.y, 2));

    if(d > r0 + r1 )
    {
        return p3;
    }
    else if(d < abs(r0 - r1))
    {
        return p3;
    }


    float a = (r0*r0 - r1*r1 + d*d) / (2 * d);

    float h = sqrt(r0*r0 - a*a);

    p2.x = p0.x + a * (p1.x - p0.x) / d;
    p2.y = p0.y + a * (p1.y - p0.y) / d;

    p3a.x = p2.x + h * (p1.y - p0.y) / d;
    p3a.y = p2.y + h * (p1.x - p0.x) / d;

    p3b.x = p2.x - h * (p1.y - p0.y) / d;
    p3b.y = p2.y - h * (p1.x - p0.x) / d;

    if(p3a.abs() < p3b.abs())
        p3 = p3a;
    else
        p3 = p3b;

    return p3;
}

bool SelfLocalisation::sharedBallUpdate(const std::vector<TeamPacket::SharedBall>& sharedBalls)
{

    // NOT IMPLEMENTED
    return false;
    std::vector<TeamPacket::SharedBall>::const_iterator their_ball = sharedBalls.begin();

    const MultivariateGaussian& best_estimate = (*getBestModel()).estimate();
    float robotx = best_estimate.mean(RobotModel::kstates_x);
    float roboty = best_estimate.mean(RobotModel::kstates_y);
    float robotheading = best_estimate.mean(RobotModel::kstates_heading);

    float sinheading = sin(robotheading);
    float cosheading = cos(robotheading);

    Matrix relativePosition(2,1,false);

    while(their_ball != sharedBalls.end())
    {
        const TeamPacket::SharedBall& sharedball = *their_ball;

        // skip ones that are too old
        if(sharedball.TimeSinceLastSeen > 300.f) continue;

        float fieldx = sharedball.X;
        float fieldy = sharedball.Y;

        relativePosition[0][0] = (fieldx - robotx) * cosheading + (fieldy - roboty) * sinheading;
        relativePosition[1][0] = -(fieldx - robotx) * sinheading + (fieldy - roboty) * cosheading;

        Matrix covariance(2,2,false);
        covariance[0][0] = sharedball.SRXX;
        covariance[0][1] = sharedball.SRXY;
        covariance[1][0] = sharedball.SRXY;
        covariance[1][1] = sharedball.SRYY;

        // TODO: Do somehting that does the update.
        //m_ball_model->directUpdate(relativePosition, covariance);

        ++their_ball;
    }
}

std::vector<TeamPacket::SharedBall> SelfLocalisation::FindNewSharedBalls(const std::vector<TeamPacket::SharedBall>& allSharedBalls)
{
    std::vector<TeamPacket::SharedBall> updateBalls;
    updateBalls.reserve(allSharedBalls.size());

    for(unsigned int b = 0; b < allSharedBalls.size(); b++)
    {
        std::vector<TeamPacket::SharedBall>::iterator b_it = m_prevSharedBalls.begin();
        std::vector<TeamPacket::SharedBall>::const_iterator end_it = m_prevSharedBalls.end();
        bool previouslyUsed = false;
        while(b_it != end_it)
        {
            if((allSharedBalls[b].TimeSinceLastSeen == b_it->TimeSinceLastSeen)
               and (allSharedBalls[b].X == b_it->X)
               and (allSharedBalls[b].Y == b_it->Y)
               and (allSharedBalls[b].SRXX == b_it->SRXX)
               and (allSharedBalls[b].SRXY == b_it->SRXY)
               and (allSharedBalls[b].SRYY == b_it->SRYY))
            {
                previouslyUsed = true;
                break;
            }
            ++b_it;
        }
        if(!previouslyUsed)
            updateBalls.push_back(allSharedBalls[b]);
    }
    m_prevSharedBalls = allSharedBalls;
    return updateBalls;
}

vector<StationaryObject*> SelfLocalisation::filterToVisible(const Self& location, const vector<StationaryObject*>& possibleObjects, float headPan, float fovX)
{
    const float c_view_direction = location.Heading() + headPan;
    const float c_view_range = fovX + 2 * location.sdHeading();

    vector<StationaryObject*> result;
    result.reserve(possibleObjects.size());

    for(vector<StationaryObject*>::const_iterator obj_it = possibleObjects.begin(); obj_it != possibleObjects.end(); ++obj_it)
    {
        float obj_heading = location.CalculateBearingToStationaryObject(*(*obj_it));
        // Calculate the distance from the viewing direction to the object.
        float delta_angle = mathGeneral::normaliseAngle(obj_heading - c_view_direction);

        // If the distance to the object heading is within the viewing range the object may be seen,
        if(fabs(delta_angle) < c_view_range)
        {
            result.push_back(*obj_it);
        }
    }
    return result;
}
