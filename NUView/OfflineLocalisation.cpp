#include <sstream>
#include <fstream>
#include "OfflineLocalisation.h"
#include "FileAccess/LogFileReader.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Localisation/SelfLocalisation.h"
#include <QElapsedTimer>
#include <QDir>
#include "Tools/Math/General.h"
#include "Tools/Math/Vector3.h"
#include <sys/time.h>
#include "Localisation/Filters/KFBuilder.h"
#include "Localisation/Filters/RobotModel.h"
#include "Localisation/Filters/IWeightedKalmanFilter.h"

/*! @brief Default Constructor
 */
OfflineLocalisation::OfflineLocalisation(LogFileReader* reader, QObject *parent): QThread(parent), m_log_reader(reader)
{
    m_workingSelfLoc = 0;
    m_stop_called = false;
    m_sim_data_available = false;
    m_num_models_created = 0;
    m_experiment_run_time = 0.0f;
    m_settings.setBranchMethod(LocalisationSettings::branch_exhaustive);
    m_settings.setPruneMethod(LocalisationSettings::prune_merge);
    m_running = false;
}

/*! @brief Class destructor
 */
OfflineLocalisation::~OfflineLocalisation()
{
    if(m_workingSelfLoc) delete m_workingSelfLoc;
    ClearBuffer();
}


/*! @brief  Removes any existing data from the buffer, and sets its size to zero.
 */
void OfflineLocalisation::ClearBuffer()
{
    for(std::vector<SelfLocalisation*>::iterator loc_it = m_self_loc_frame_buffer.begin(); loc_it != m_self_loc_frame_buffer.end(); ++loc_it)
    {
        delete (*loc_it);
    }
    m_self_loc_frame_buffer.clear();
    m_self_frame_info.clear();
    m_performance.clear();
    m_sim_data_available = false;
    emit SimDataChanged(m_sim_data_available);
}

/*! @brief  Clears any previous information, and intialises the system with the
            initial state.
    @param initialState The initial state of the system.
 */
void OfflineLocalisation::Initialise()
{
    delete m_workingSelfLoc;
    m_workingSelfLoc = new SelfLocalisation(0, m_settings);
    ClearBuffer();
    m_stop_called = false;
    m_sim_data_available = false;
    return;
}

/*! @brief Open the logs at the given paths.
    @param sensorLogPath The path to the file containing the sensor logs.
    @param objectsLogPath The path to the file containing the Field Object logs.
    @return Returns True if the files were opened successfuly. False if they were not.
 */
bool OfflineLocalisation::OpenLogs(const std::string& initialLogPath)
{
    // Open Files
    m_log_reader->openFile(QString::fromStdString(initialLogPath));

    // Check if it worked ok.
    bool success = (m_log_reader->numFrames() > 0);
    if(success and m_log_reader) m_log_reader->firstFrame();
    return success;
}

/*! @brief  Determine if the system has been properly initialised. If this condition is not met,
            the system beaviour may be undefined if run.
    @return True if the system been Inititalised sucessfully. False if it has not.
 */
bool OfflineLocalisation::IsInitialised()
{
    bool filesOpenOk = (m_log_reader->numFrames() > 0);
    bool internalDataOk = (m_workingSelfLoc != NULL) and (m_self_loc_frame_buffer.size() >= 0);
    QStringList availableData = m_log_reader->AvailableData();
    bool all_data_available = HasRequiredData(availableData);

    return (filesOpenOk && internalDataOk && all_data_available);
}

bool OfflineLocalisation::HasRequiredData(QStringList& available_data)
{
    QStringList required_data, possible_sensor_data;
    required_data << "object" << "teaminfo" << "gameinfo";
    possible_sensor_data << "sensor" << "locsensor";
    bool sensor_avialable = false;
    bool required_data_available = true;
    QStringList::const_iterator constIterator;
    for (constIterator = required_data.constBegin(); constIterator != required_data.constEnd();++constIterator)
    {
        if(available_data.contains(*constIterator,Qt::CaseInsensitive) == false)
        {
            required_data_available = false;
        }
    }

    for (constIterator = possible_sensor_data.constBegin(); constIterator != possible_sensor_data.constEnd();++constIterator)
    {
        if(available_data.contains(*constIterator,Qt::CaseInsensitive) == true)
            sensor_avialable = true;
    }
    return required_data_available && sensor_avialable;
}

/*! @brief Run the simulation using the available data.
    @return True if simulation sucessful. False if unsuccessful.
 */
#include <sstream>
void OfflineLocalisation::run()
{
    qDebug("Starting experiment.");
    m_running = true;
    Initialise();
    // Don't do anything if not initialised properly.
    if(IsInitialised() == false)
    {
        qDebug("Not initialised.");
        m_running = false;
        return;
    }

    m_stop_called = false;
    m_sim_data_available = false;
    const NUSensorsData* tempSensor;
    FieldObjects* tempObjects;
    const TeamInformation* tempTeamInfo;
    const GameInformation* tempGameInfo;

    int framesProcessed = 0;
    int totalFrames = m_log_reader->numFrames();
    m_log_reader->firstFrame();

    m_performance.reserve(totalFrames);
    m_self_loc_frame_buffer.reserve(totalFrames);
    m_self_frame_info.reserve(totalFrames);

    emit updateProgress(framesProcessed+1,totalFrames);

    bool save_signal = m_log_reader->blockSignals(true); // Block signals
    int save_frame = m_log_reader->currentFrame();

    QElapsedTimer experimentTimer;
    experimentTimer.start();
    unsigned int initial_model_id = m_workingSelfLoc->getBestModel()->id();

    while (true)
    {
        tempSensor = m_log_reader->GetSensorData();
        tempObjects = m_log_reader->GetObjectData();
        tempTeamInfo = m_log_reader->GetTeamInfo();
        tempGameInfo = m_log_reader->GetGameInfo();

        // Break if things look bad.
        if(tempSensor == NULL or tempObjects == NULL or tempTeamInfo == NULL or tempGameInfo == NULL)
        {
            break;
        }
/*
        AmbiguousObject t_corner(FieldObjects::FO_CORNER_UNKNOWN_T, "Unknown T");
        AmbiguousObject l_corner(FieldObjects::FO_CORNER_UNKNOWN_INSIDE_L, "Unknown Inside L");

        AmbiguousObject b_post(FieldObjects::FO_BLUE_GOALPOST_UNKNOWN, "Unknown Blue Post");
        AmbiguousObject y_post(FieldObjects::FO_YELLOW_GOALPOST_UNKNOWN, "Unknown Yellow Post");


        b_post.setIsVisible(true);
        y_post.setIsVisible(true);
        l_corner.setIsVisible(true);
        t_corner.setIsVisible(true);

        b_post.addPossibleObjectID(FieldObjects::FO_BLUE_LEFT_GOALPOST);
        b_post.addPossibleObjectID(FieldObjects::FO_BLUE_RIGHT_GOALPOST);

        y_post.addPossibleObjectID(FieldObjects::FO_YELLOW_LEFT_GOALPOST);
        y_post.addPossibleObjectID(FieldObjects::FO_YELLOW_RIGHT_GOALPOST);

        t_corner.addPossibleObjectID(FieldObjects::FO_CORNER_YELLOW_T_LEFT);
        t_corner.addPossibleObjectID(FieldObjects::FO_CORNER_YELLOW_T_RIGHT);
        t_corner.addPossibleObjectID(FieldObjects::FO_CORNER_CENTRE_T_LEFT);
        t_corner.addPossibleObjectID(FieldObjects::FO_CORNER_CENTRE_T_RIGHT);
        t_corner.addPossibleObjectID(FieldObjects::FO_CORNER_BLUE_T_LEFT);
        t_corner.addPossibleObjectID(FieldObjects::FO_CORNER_BLUE_T_RIGHT);
        t_corner.addPossibleObjectID(FieldObjects::FO_CORNER_PROJECTED_T_YELLOW_LEFT);
        t_corner.addPossibleObjectID(FieldObjects::FO_CORNER_PROJECTED_T_YELLOW_RIGHT);
        t_corner.addPossibleObjectID(FieldObjects::FO_CORNER_PROJECTED_T_BLUE_LEFT);
        t_corner.addPossibleObjectID(FieldObjects::FO_CORNER_PROJECTED_T_BLUE_RIGHT);


        l_corner.addPossibleObjectID(FieldObjects::FO_CORNER_YELLOW_FIELD_LEFT);
        l_corner.addPossibleObjectID(FieldObjects::FO_CORNER_YELLOW_FIELD_RIGHT);
        l_corner.addPossibleObjectID(FieldObjects::FO_CORNER_YELLOW_PEN_LEFT);
        l_corner.addPossibleObjectID(FieldObjects::FO_CORNER_YELLOW_PEN_RIGHT);
        l_corner.addPossibleObjectID(FieldObjects::FO_CORNER_BLUE_PEN_LEFT);
        l_corner.addPossibleObjectID(FieldObjects::FO_CORNER_BLUE_PEN_RIGHT);
        l_corner.addPossibleObjectID(FieldObjects::FO_CORNER_BLUE_FIELD_LEFT);
        l_corner.addPossibleObjectID(FieldObjects::FO_CORNER_BLUE_FIELD_RIGHT);

        for(std::vector<StationaryObject>::iterator object = tempObjects->stationaryFieldObjects.begin(); object != tempObjects->stationaryFieldObjects.end(); ++object)
        {
            if(object->isObjectVisible() == false) continue;
            switch(object->getID())
            {
                case FieldObjects::FO_BLUE_LEFT_GOALPOST:
                case FieldObjects::FO_BLUE_RIGHT_GOALPOST:
                    b_post.CopyMeasurement(*object);
                    object->setIsVisible(false);
                    tempObjects->ambiguousFieldObjects.push_back(b_post);
                    break;

                case FieldObjects::FO_YELLOW_LEFT_GOALPOST:
                case FieldObjects::FO_YELLOW_RIGHT_GOALPOST:
                    y_post.CopyMeasurement(*object);
                    object->setIsVisible(false);
                    tempObjects->ambiguousFieldObjects.push_back(y_post);
                    break;

                case FieldObjects::FO_CORNER_YELLOW_FIELD_LEFT:
                case FieldObjects::FO_CORNER_YELLOW_FIELD_RIGHT:
                case FieldObjects::FO_CORNER_YELLOW_PEN_LEFT:
                case FieldObjects::FO_CORNER_YELLOW_PEN_RIGHT:
                case FieldObjects::FO_CORNER_BLUE_PEN_LEFT:
                case FieldObjects::FO_CORNER_BLUE_PEN_RIGHT:
                case FieldObjects::FO_CORNER_BLUE_FIELD_LEFT:
                case FieldObjects::FO_CORNER_BLUE_FIELD_RIGHT:
                    l_corner.CopyMeasurement(*object);
                    object->setIsVisible(false);
                    tempObjects->ambiguousFieldObjects.push_back(l_corner);
                    break;
                case FieldObjects::FO_CORNER_YELLOW_T_LEFT:
                case FieldObjects::FO_CORNER_YELLOW_T_RIGHT:
                case FieldObjects::FO_CORNER_CENTRE_T_LEFT:
                case FieldObjects::FO_CORNER_CENTRE_T_RIGHT:
                case FieldObjects::FO_CORNER_BLUE_T_LEFT:
                case FieldObjects::FO_CORNER_BLUE_T_RIGHT:
                case FieldObjects::FO_CORNER_PROJECTED_T_YELLOW_LEFT:
                case FieldObjects::FO_CORNER_PROJECTED_T_YELLOW_RIGHT:
                case FieldObjects::FO_CORNER_PROJECTED_T_BLUE_LEFT:
                case FieldObjects::FO_CORNER_PROJECTED_T_BLUE_RIGHT:
                    t_corner.CopyMeasurement(*object);
                    object->setIsVisible(false);
                    tempObjects->ambiguousFieldObjects.push_back(t_corner);
                    break;

                default:
                    break;
            }
        }
        */

        for(std::vector<AmbiguousObject>::iterator object = tempObjects->ambiguousFieldObjects.begin(); object != tempObjects->ambiguousFieldObjects.end(); ++object)
        {
            // Add if seen
            if(object->isObjectVisible())
            {
                if(object->getID() == FieldObjects::FO_CORNER_UNKNOWN_T or object->getID() == FieldObjects::FO_CORNER_UNKNOWN_INSIDE_L
                        or object->getID() == FieldObjects::FO_CORNER_UNKNOWN_OUTSIDE_L)
                {
                    object->addPossibleObjectID(FieldObjects::FO_CORNER_CENTRE_CIRCLE_INTERSECT_LEFT);
                    object->addPossibleObjectID(FieldObjects::FO_CORNER_CENTRE_CIRCLE_INTERSECT_RIGHT);
                }
            }
        }


        AddFrame(tempSensor, tempObjects, tempTeamInfo, tempGameInfo);

        if(m_log_reader->nextFrameAvailable())
        {
            m_log_reader->nextFrame();
        }
        else break;
        ++framesProcessed;
        if(m_stop_called) break;
        emit updateProgress(framesProcessed+1,totalFrames);
    }
    if(!m_stop_called)
    {
        m_sim_data_available = true;
    }
    float exp_time = experimentTimer.elapsed() / 1000.0f;
    float total_time = 0.0f;
    float max_time = 0;
    unsigned int max_frame = 0;
    unsigned int curr_frame = 0;
    float temp_time;

    for(std::vector<LocalisationPerformanceMeasure>::iterator it = m_performance.begin(); it != m_performance.end(); ++it)
    {
        temp_time = it->processingTime();
        total_time += temp_time;
        max_time = max(max_time, temp_time);
        if(max_time == temp_time)
            max_frame = curr_frame;
        ++curr_frame;
    }

    IWeightedKalmanFilter* test = KFBuilder::getNewFilter(KFBuilder::kseq_ukf_filter, KFBuilder::krobot_model);
    m_num_models_created = test->id() - initial_model_id;
    delete test;
    m_experiment_run_time = exp_time * 1000;
    std::cout << "Number of models created: " << m_num_models_created << std::endl;
    std::cout << "Total Processing time: " << total_time * 1000 << " ms" <<std::endl;
    std::cout << "Experiment time: " << m_experiment_run_time << " ms" << std::endl;
    std::cout << "Max frame time: " << max_time << " ms - frame: " << max_frame << std::endl;


    m_log_reader->setFrame(save_frame);
    m_log_reader->blockSignals(save_signal);
    m_running = false;
    emit SimDataChanged(m_sim_data_available);
    emit ProcessingComplete(m_log_reader->path());
    return;
}

/*! @brief Run the localisation algorithm on the provided data and add a new localisation frame.
    @param sensorData The sensor data for the new frame.
    @param objectData The observed objects for the current frame.
 */
void OfflineLocalisation::AddFrame(const NUSensorsData* sensorData, FieldObjects* objectData, const TeamInformation* teamInfo, const GameInformation* gameInfo)
{
    timeval t1, t2;
    double elapsedTime;

    // Need to make copies, since source is const

    NUSensorsData tempSensors2 = (*sensorData);
    QElapsedTimer timer;
    long int ms_elapsed;

    // start timer
    gettimeofday(&t1, NULL);

    timer.start();
    m_workingSelfLoc->process(&tempSensors2,objectData,gameInfo,teamInfo);
    ms_elapsed = timer.elapsed();

    // stop timer
    gettimeofday(&t2, NULL);
    elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;      // sec to ms
    elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;   // us to ms

    SelfLocalisation* self_temp = new SelfLocalisation(*m_workingSelfLoc);

    LocalisationPerformanceMeasure performance_measure;
//    performance_measure.setProcessingTime(ms_elapsed / 1000.0f);
    performance_measure.setProcessingTime(elapsedTime);
    vector<float> gps;
    float compass;
    if(tempSensors2.getGps(gps) and tempSensors2.getCompass(compass))
    {
        float err_x, err_y, err_head;
        const MultivariateGaussian best_estimate = m_workingSelfLoc->getBestModel()->estimate();
        err_x = best_estimate.mean(RobotModel::kstates_x) - gps[0];
        err_y = best_estimate.mean(RobotModel::kstates_y) - gps[1];
        err_head = mathGeneral::normaliseAngle(best_estimate.mean(RobotModel::kstates_heading) - compass);
        performance_measure.setError(err_x, err_y, err_head);
    }
    m_performance.push_back(performance_measure);
    m_self_loc_frame_buffer.push_back(self_temp);
    QString self_info(m_workingSelfLoc->frameLog().c_str());
    m_self_frame_info.push_back(self_info);
    return;
}

int OfflineLocalisation::NumberOfLogFrames()
{
    return m_log_reader->numFrames();
}

int OfflineLocalisation::NumberOfFrames()
{
    return m_self_loc_frame_buffer.size();
}

const SelfLocalisation* OfflineLocalisation::GetSelfFrame(int frameNumber)
{
    int index = frameNumber-1;
    if( (index < 0) || (index >= NumberOfFrames()))
        return NULL;
    else
    {
        return m_self_loc_frame_buffer[index];
    }
}

QString OfflineLocalisation::GetSelfFrameInfo(int frameNumber)
{
    int index = frameNumber-1;
    if( (index < 0) || (index >= NumberOfFrames()))
        return NULL;
    else
        return m_self_frame_info[index];
}

/*! @brief Writes a text based summary of the current experiment to file.
    @param logPath Path to which the log will be written
 */
bool OfflineLocalisation::WriteReport(const std::string& reportPath)
{
    return WriteXML(reportPath);
}


std::string BeginTag(std::string tag)
{
    std::stringstream temp;
    temp << "<" << tag << ">";
    return temp.str();
}

std::string EndTag(std::string tag)
{
    std::stringstream temp;
    temp << "</" << tag << ">";
    return temp.str();
}

std::string Tabbing(unsigned int num_tabs)
{
    std::stringstream result;
    for(unsigned int tab = 0; tab < num_tabs; ++tab)
    {
        result << "  ";
    }
    return result.str();
}

/*! @brief Writes a text based summary of the current experiment to file.
    @param logPath Path to which the log will be written
 */
bool OfflineLocalisation::WriteXML(const std::string& xmlPath)
{
    std::string temp = "", tag="";
    unsigned int tab_depth = 0;
    bool file_saved = false;

    if(hasSimData())
    {
        // Collect the data to be added to the report.
        // This is perforiming the loop done while porcessing, but reading the data back here makes the
        // whole process a lot neater.
        unsigned int total_frames  = m_self_loc_frame_buffer.size();
        temp = m_log_reader->path().toStdString();
        std::string path = temp.erase(temp.rfind(QDir::separator().toAscii())+1);

        unsigned int num_frames = NumberOfFrames();

        std::vector< Vector3<float> > measured_positions(num_frames);
        std::vector< Vector3<float> > estimated_positions(num_frames);
        std::vector<unsigned int> stationary_object_count;
        std::vector<unsigned int> ambiguous_object_count;
        stationary_object_count.resize(FieldObjects::NUM_STAT_FIELD_OBJECTS, 0);
        ambiguous_object_count.resize(FieldObjects::NUM_AMBIGUOUS_FIELD_OBJECTS, 0);


        std::vector<unsigned int> obs_frames;
        std::vector<unsigned int> obs_ids;
        std::vector<float> obs_distance;
        std::vector<float> obs_heading;
        std::vector<float> exp_distance;
        std::vector<float> exp_heading;

        std::vector<unsigned int> amb_obs_frames;
        std::vector<unsigned int> amb_obs_ids;
        std::vector<int> amb_exp_id;
        std::vector<float> amb_obs_distance;
        std::vector<float> amb_obs_heading;
        std::vector<float> amb_exp_distance;
        std::vector<float> amb_exp_heading;
        std::vector<unsigned int> amb_decision_id;

        unsigned int total_amb_obj = 0;

        std::vector<std::vector<float> > odom;

        FieldObjects* tempObjects;
        const TeamInformation* tempTeamInfo;
        const GameInformation* tempGameInfo;
        const SelfLocalisation* temp_loc;

        m_log_reader->blockSignals(true);
        for (unsigned int frame = 0; frame < num_frames; ++frame)
        {
            m_log_reader->setFrame(frame);
            tempObjects = m_log_reader->GetObjectData();
            const NUSensorsData* sensors = m_log_reader->GetSensorData();
            NUSensorsData tempSensor = (*sensors);
            tempTeamInfo = m_log_reader->GetTeamInfo();
            tempGameInfo = m_log_reader->GetGameInfo();

            vector<float> gps;
            float compass;
            Vector3<float> measure(0,0,0);
            Self gps_location;
            if(tempSensor.getGps(gps) and tempSensor.getCompass(compass))
            {
                measure.x = gps[0];
                measure.y = gps[1];
                measure.z = compass;
                gps_location.updateLocationOfSelf(measure.x, measure.y,measure.z,0.1,0.1,0.01,false);
            }
            measured_positions[frame] = measure;

            std::vector<float> odometry;
            if(tempSensor.getOdometry(odometry))
            {
                odometry.push_back(frame);
                odom.push_back(odometry);
            }
            else
            {
                odometry.resize(3,0);
                odometry.push_back(frame);
                odom.push_back(odometry);
            }

            // Localisation result
            temp_loc = GetSelfFrame(frame+1);
            assert(temp_loc);
            Vector3<float> estimate(0,0,0);
            const IWeightedKalmanFilter* best_model = temp_loc->getBestModel();
            const MultivariateGaussian best_estimate = best_model->estimate();
            estimate.x = best_estimate.mean(RobotModel::kstates_x);
            estimate.y = best_estimate.mean(RobotModel::kstates_y);
            estimate.z = best_estimate.mean(RobotModel::kstates_heading);
            estimated_positions[frame] = estimate;


            for(std::vector<StationaryObject>::const_iterator object = tempObjects->stationaryFieldObjects.begin(); object != tempObjects->stationaryFieldObjects.end(); ++object)
            {
                // Add if seen
                if(object->isObjectVisible())
                {
                    stationary_object_count[object->getID()]++;
                    obs_frames.push_back(frame);
                    obs_ids.push_back(object->getID());
                    obs_distance.push_back(object->measuredDistance()*cos(object->estimatedElevation()));
                    obs_heading.push_back(object->measuredBearing());
                    exp_distance.push_back(gps_location.CalculateDistanceToStationaryObject(*object));
                    exp_heading.push_back(gps_location.CalculateBearingToStationaryObject(*object));
                }
            }
            for(std::vector<AmbiguousObject>::iterator object = tempObjects->ambiguousFieldObjects.begin(); object != tempObjects->ambiguousFieldObjects.end(); ++object)
            {
                // Add if seen
                if(object->isObjectVisible())
                {
                    // Ignore the mobile objects for now.
                    if(object->getID() == FieldObjects::FO_PINK_ROBOT_UNKNOWN) continue;
                    if(object->getID() == FieldObjects::FO_BLUE_ROBOT_UNKNOWN) continue;
                    if(object->getID() == FieldObjects::FO_ROBOT_UNKNOWN) continue;
                    if(object->getID() == FieldObjects::FO_OBSTACLE) continue;

                    if(object->getID() == FieldObjects::FO_CORNER_UNKNOWN_T or object->getID() == FieldObjects::FO_CORNER_UNKNOWN_INSIDE_L
                            or object->getID() == FieldObjects::FO_CORNER_UNKNOWN_OUTSIDE_L)
                    {
                        object->addPossibleObjectID(FieldObjects::FO_CORNER_CENTRE_CIRCLE_INTERSECT_LEFT);
                        object->addPossibleObjectID(FieldObjects::FO_CORNER_CENTRE_CIRCLE_INTERSECT_RIGHT);
                    }

                    total_amb_obj++;
                    ambiguous_object_count[object->getID()]++;
                    amb_obs_frames.push_back(frame);
                    amb_obs_ids.push_back(object->getID());
                    amb_obs_distance.push_back(object->measuredDistance()*cos(object->estimatedElevation()));
                    amb_obs_heading.push_back(object->measuredBearing());

                    // Determine most likely option and use it to get expected measurements
                    int likely_id = tempObjects->getClosestStationaryOption(gps_location, *object);
                    assert(likely_id >= 0);
                    amb_exp_id.push_back(likely_id);
                    amb_exp_distance.push_back(gps_location.CalculateDistanceToStationaryObject(tempObjects->stationaryFieldObjects[likely_id]));
                    amb_exp_heading.push_back(gps_location.CalculateBearingToStationaryObject(tempObjects->stationaryFieldObjects[likely_id]));

                    double heading = gps_location.CalculateBearingToStationaryObject(tempObjects->stationaryFieldObjects[likely_id]);
                    double error = fabs(mathGeneral::normaliseAngle(heading - object->measuredBearing()));
//                    if(error > 0.5)
//                    {
//                        std::cout << frame << " - Error: " << error << " - expected object: " << tempObjects->stationaryFieldObjects[likely_id].getName() << std::endl;
//                    }

                    // Now we want to check which option was chosen as the best by the localistion system at the time of update.
                    // Must be a new model to be the result of a split.
                    if(temp_loc->GetTimestamp() == temp_loc->getBestModel()->creationTime())
                    {
                        unsigned int decision_id = temp_loc->getBestModel()->previousSplitOption(*object);
                        amb_decision_id.push_back(decision_id);
                    }
                    // If there was no new update used, set the id to the num of objects (invalid)
                    else
                    {
                        amb_decision_id.push_back(FieldObjects::NUM_STAT_FIELD_OBJECTS);
                    }
                }
            }

            // All of these vectors should be the same size.
            assert(obs_frames.size() == obs_ids.size());
            assert(obs_ids.size() == obs_distance.size());
            assert(obs_distance.size() == obs_heading.size());
            assert(amb_decision_id.size() == total_amb_obj);
        }
        // Re-enable signals from the log reader.
        m_log_reader->blockSignals(false);

        // Write info to file
        std::ofstream output_file(xmlPath.c_str());
        if(output_file.is_open() && output_file.good())
        {
            // ******************
            // Experiment details
            // ******************
            output_file << Tabbing(tab_depth++) << BeginTag("report") << std::endl;
            output_file << Tabbing(tab_depth++) << BeginTag("experiment") << std::endl;

            output_file << Tabbing(tab_depth) << BeginTag("filename") << path << EndTag("filename") << std::endl;
            output_file << Tabbing(tab_depth) << BeginTag("frames") << total_frames << EndTag("frames") << std::endl;

            FieldObjects* tempObjects = m_log_reader->GetObjectData();

            // Add the stationary objects.
//            output_file << Tabbing(tab_depth++) << BeginTag("stationary_objects") << std::endl;
            for(std::vector<StationaryObject>::const_iterator object = tempObjects->stationaryFieldObjects.begin(); object != tempObjects->stationaryFieldObjects.end(); ++object)
            {
                output_file << Tabbing(tab_depth++) << BeginTag("stationary_object") << std::endl;

                output_file << Tabbing(tab_depth) << BeginTag("id") << object->getID() << EndTag("id") << std::endl;
                output_file << Tabbing(tab_depth) << BeginTag("name") << object->getName() << EndTag("name") << std::endl;
                output_file << Tabbing(tab_depth) << BeginTag("count") << stationary_object_count[object->getID()] << EndTag("count") << std::endl;

                output_file << Tabbing(--tab_depth) << EndTag("stationary_object") << std::endl;
            }
//            output_file << Tabbing(--tab_depth) << EndTag("stationary_objects") << std::endl;

            // Add the ambiguous objects.
//            output_file << Tabbing(tab_depth++) << BeginTag("ambiguous_objects") << std::endl;
            for(unsigned int id = 0; id < FieldObjects::NUM_AMBIGUOUS_FIELD_OBJECTS; ++id)
            {
                output_file << Tabbing(tab_depth++) << BeginTag("ambiguous_object") << std::endl;

                output_file << Tabbing(tab_depth) << BeginTag("id") << id << EndTag("id") << std::endl;
                output_file << Tabbing(tab_depth) << BeginTag("name") << FieldObjects::ambiguousName(id) << EndTag("name") << std::endl;
                output_file << Tabbing(tab_depth) << BeginTag("count") << ambiguous_object_count[id] << EndTag("count") << std::endl;

                output_file << Tabbing(--tab_depth) << EndTag("ambiguous_object") << std::endl;
            }
//            output_file << Tabbing(--tab_depth) << EndTag("ambiguous_objects") << std::endl;

            // Add observations
//            output_file << Tabbing(tab_depth++) << BeginTag("observations") << std::endl;
            for (unsigned int i = 0; i < obs_frames.size(); ++i)
            {
                output_file << Tabbing(tab_depth++) << BeginTag("observation") << std::endl;
//                output_file << Tabbing(tab_depth++) << BeginTag("item") << std::endl;
                output_file << Tabbing(tab_depth) << BeginTag("frame") << obs_frames[i] << EndTag("frame") <<std::endl;
                output_file << Tabbing(tab_depth) << BeginTag("id") << obs_ids[i] << EndTag("id") <<std::endl;

                output_file << Tabbing(tab_depth++) << BeginTag("estimate") << std::endl;
                output_file << Tabbing(tab_depth) << BeginTag("distance") << obs_distance[i] << EndTag("distance") <<std::endl;
                output_file << Tabbing(tab_depth) << BeginTag("heading") << obs_heading[i] << EndTag("heading") <<std::endl;
                output_file << Tabbing(--tab_depth) << EndTag("estimate") << std::endl;

                output_file << Tabbing(tab_depth++) << BeginTag("expected") << std::endl;
                output_file << Tabbing(tab_depth) << BeginTag("distance") << exp_distance[i] << EndTag("distance") <<std::endl;
                output_file << Tabbing(tab_depth) << BeginTag("heading") << exp_heading[i] << EndTag("heading") <<std::endl;
                output_file << Tabbing(--tab_depth) << EndTag("expected") << std::endl;

//                output_file << Tabbing(--tab_depth) << EndTag("item") << std::endl;
                  output_file << Tabbing(--tab_depth) << EndTag("observation") << std::endl;
            }
//            output_file << Tabbing(--tab_depth) << EndTag("observations") << std::endl;

            // Add ambiguous observations
//            output_file << Tabbing(tab_depth++) << BeginTag("ambiguous_observations") << std::endl;
            for (unsigned int i = 0; i < amb_obs_frames.size(); ++i)
            {
//                output_file << Tabbing(tab_depth++) << BeginTag("item") << std::endl;
                output_file << Tabbing(tab_depth++) << BeginTag("ambiguous_observation") << std::endl;
                output_file << Tabbing(tab_depth) << BeginTag("frame") << amb_obs_frames[i] << EndTag("frame") <<std::endl;
                output_file << Tabbing(tab_depth) << BeginTag("id") << amb_obs_ids[i] << EndTag("id") <<std::endl;

                output_file << Tabbing(tab_depth++) << BeginTag("estimate") << std::endl;
                output_file << Tabbing(tab_depth) << BeginTag("distance") << amb_obs_distance[i] << EndTag("distance") <<std::endl;
                output_file << Tabbing(tab_depth) << BeginTag("heading") << amb_obs_heading[i] << EndTag("heading") <<std::endl;
                output_file << Tabbing(--tab_depth) << EndTag("estimate") << std::endl;

                output_file << Tabbing(tab_depth++) << BeginTag("expected") << std::endl;
                output_file << Tabbing(tab_depth) << BeginTag("id") << amb_exp_id[i] << EndTag("id") << std::endl;
                output_file << Tabbing(tab_depth) << BeginTag("distance") << amb_exp_distance[i] << EndTag("distance") << std::endl;
                output_file << Tabbing(tab_depth) << BeginTag("heading") << amb_exp_heading[i] << EndTag("heading") << std::endl;
                output_file << Tabbing(--tab_depth) << EndTag("expected") << std::endl;

//                output_file << Tabbing(--tab_depth) << EndTag("item") << std::endl;
                output_file << Tabbing(--tab_depth) << EndTag("ambiguous_observation") << std::endl;
            }
//            output_file << Tabbing(--tab_depth) << EndTag("ambiguous_observations") << std::endl;

            // Add odometry information
//            output_file << Tabbing(tab_depth++) << BeginTag("odometry") << std::endl;
            for(std::vector<std::vector<float> >::iterator odom_it = odom.begin(); odom_it != odom.end(); ++odom_it)
            {
//                output_file << Tabbing(tab_depth++) << BeginTag("item") << std::endl;
                output_file << Tabbing(tab_depth++) << BeginTag("odometry") << std::endl;
                output_file << Tabbing(tab_depth) << BeginTag("frame") << (*odom_it)[3] << EndTag("frame") <<std::endl;
                output_file << Tabbing(tab_depth) << BeginTag("x") << (*odom_it)[0] << EndTag("x") <<std::endl;
                output_file << Tabbing(tab_depth) << BeginTag("y") << (*odom_it)[1] << EndTag("y") <<std::endl;
                output_file << Tabbing(tab_depth) << BeginTag("turn") << (*odom_it)[2] << EndTag("turn") <<std::endl;
//                output_file << Tabbing(--tab_depth) << EndTag("item") << std::endl;
                output_file << Tabbing(--tab_depth) << EndTag("odometry") << std::endl;
            }
//            output_file << Tabbing(--tab_depth) << EndTag("odometry") << std::endl;

            // Add measured position data
//            output_file << Tabbing(tab_depth++) << BeginTag("measured_position") << std::endl;
            for (unsigned int frame_id = 0; frame_id < total_frames; ++frame_id)
            {
//                output_file << Tabbing(tab_depth++) << BeginTag("item") << std::endl;
                output_file << Tabbing(tab_depth++) << BeginTag("measured_position") << std::endl;
                output_file << Tabbing(tab_depth) << BeginTag("frame") << frame_id << EndTag("frame") <<std::endl;
                output_file << Tabbing(tab_depth) << BeginTag("x") << measured_positions[frame_id].x << EndTag("x") <<std::endl;
                output_file << Tabbing(tab_depth) << BeginTag("y") << measured_positions[frame_id].y << EndTag("y") <<std::endl;
                output_file << Tabbing(tab_depth) << BeginTag("heading") << measured_positions[frame_id].z << EndTag("heading") <<std::endl;
//                output_file << Tabbing(--tab_depth) << EndTag("item") << std::endl;
                output_file << Tabbing(--tab_depth) << EndTag("measured_position") << std::endl;
            }
//            output_file << Tabbing(--tab_depth) << EndTag("measured_position") << std::endl;

            output_file << Tabbing(--tab_depth) << EndTag("experiment") << std::endl;

            // *******************
            // Experiment results.
            // *******************
            output_file << Tabbing(tab_depth++) <<  BeginTag("results") << std::endl;

            output_file << Tabbing(tab_depth) << BeginTag("branch_method") << m_settings.branchMethodString() << EndTag("branch_method") << std::endl;
            output_file << Tabbing(tab_depth) << BeginTag("prune_method") << m_settings.pruneMethodString() << EndTag("prune_method") << std::endl;
            output_file << Tabbing(tab_depth) << BeginTag("total_models") << m_num_models_created << EndTag("total_models") << std::endl;
            output_file << Tabbing(tab_depth) << BeginTag("runtime") << m_experiment_run_time << EndTag("runtime") << std::endl;

//            output_file << Tabbing(tab_depth++) << BeginTag("estimated_position") << std::endl;
            for (unsigned int frame_id = 0; frame_id < total_frames; ++frame_id)
            {
//                output_file << Tabbing(tab_depth++) << BeginTag("item") << std::endl;
                output_file << Tabbing(tab_depth++) << BeginTag("estimated_position") << std::endl;
                output_file << Tabbing(tab_depth) << BeginTag("frame") << frame_id << EndTag("frame") <<std::endl;
                output_file << Tabbing(tab_depth) << BeginTag("processing_time") << m_performance[frame_id].processingTime() << EndTag("processing_time") <<std::endl;
                output_file << Tabbing(tab_depth) << BeginTag("x") << estimated_positions[frame_id].x << EndTag("x") <<std::endl;
                output_file << Tabbing(tab_depth) << BeginTag("y") << estimated_positions[frame_id].y << EndTag("y") <<std::endl;
                output_file << Tabbing(tab_depth) << BeginTag("heading") << estimated_positions[frame_id].z << EndTag("heading") <<std::endl;
                output_file << Tabbing(--tab_depth) << EndTag("estimated_position") << std::endl;
//                output_file << Tabbing(--tab_depth) << EndTag("item") << std::endl;
            }
//            output_file << Tabbing(--tab_depth) << EndTag("estimated_position") << std::endl;

//            output_file << Tabbing(tab_depth++) << BeginTag("ambiguous_decision") << std::endl;
            for (std::vector<unsigned int>::iterator dec_it = amb_decision_id.begin(); dec_it != amb_decision_id.end(); ++dec_it)
            {
//                output_file << Tabbing(tab_depth++) << BeginTag("item") << std::endl;
                output_file << Tabbing(tab_depth++) << BeginTag("ambiguous_decision") << std::endl;
                output_file << Tabbing(tab_depth) << BeginTag("id") << (*dec_it) << EndTag("id") <<std::endl;
                output_file << Tabbing(--tab_depth) << EndTag("ambiguous_decision") << std::endl;
//                output_file << Tabbing(--tab_depth) << EndTag("item") << std::endl;
            }
//            output_file << Tabbing(--tab_depth) << EndTag("ambiguous_decision") << std::endl;

            output_file << Tabbing(--tab_depth) << EndTag("results") << std::endl;
            output_file << Tabbing(--tab_depth) << EndTag("report") << std::endl;

        }
        output_file.close();
    }
    return file_saved;
}
