#include <sstream>
#include <fstream>
#include "OfflineLocalisation.h"
#include "FileAccess/LogFileReader.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Localisation/SelfLocalisation.h"
#include "Localisation/Localisation.h"
#include "Localisation/SelfLocalisationTests.h"
#include <QElapsedTimer>

/*! @brief Default Constructor
 */
OfflineLocalisation::OfflineLocalisation(LogFileReader* reader, QObject *parent): QThread(parent), m_log_reader(reader)
{
    m_workingLoc = 0;
    m_workingSelfLoc = 0;
    //Initialise(Localisation());
    m_stop_called = false;
    m_sim_data_available = false;
    m_settings.setBranchMethod(LocalisationSettings::branch_exhaustive);
    m_settings.setPruneMethod(LocalisationSettings::prune_merge);
    RunTests();
}

/*! @brief Class destructor
 */
OfflineLocalisation::~OfflineLocalisation()
{
    if(m_workingLoc) delete m_workingLoc;
    if(m_workingSelfLoc) delete m_workingSelfLoc;
    ClearBuffer();
}


/*! @brief  Removes any existing data from the buffer, and sets its size to zero.
 */
void OfflineLocalisation::ClearBuffer()
{
    for(std::vector<Localisation*>::iterator loc_it = m_localisation_frame_buffer.begin(); loc_it != m_localisation_frame_buffer.end(); ++loc_it)
    {
        delete (*loc_it);
    }
    for(std::vector<SelfLocalisation*>::iterator loc_it = m_self_loc_frame_buffer.begin(); loc_it != m_self_loc_frame_buffer.end(); ++loc_it)
    {
        delete (*loc_it);
    }
    m_self_loc_frame_buffer.clear();
    m_localisation_frame_buffer.clear();
    m_frame_info.clear();
    m_self_frame_info.clear();
    m_performance .clear();
    m_sim_data_available = false;
}

/*! @brief  Clears any previous information, and intialises the system with the
            initial state.
    @param initialState The initial state of the system.
 */
void OfflineLocalisation::Initialise(const Localisation& intialState)
{
    delete m_workingLoc;
    delete m_workingSelfLoc;
    m_workingLoc = new Localisation(intialState);
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
    return success;
}

/*! @brief  Determine if the system has been properly initialised. If this condition is not met,
            the system beaviour may be undefined if run.
    @return True if the system been Inititalised sucessfully. False if it has not.
 */
bool OfflineLocalisation::IsInitialised()
{
    bool filesOpenOk = (m_log_reader->numFrames() > 0);
    bool internalDataOk = (m_workingLoc != NULL) && (m_localisation_frame_buffer.size() >= 0);
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
void OfflineLocalisation::run()
{
    Initialise(Localisation());
    // Don't do anything if not initialised properly.
    if(IsInitialised() == false) return;
    m_stop_called = false;
    m_sim_data_available = false;
    const NUSensorsData* tempSensor;
    FieldObjects* tempObjects;
    const TeamInformation* tempTeamInfo;
    const GameInformation* tempGameInfo;

    int framesProcessed = 0;
    int totalFrames = m_log_reader->numFrames();
    m_log_reader->firstFrame();
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
        if(tempSensor == NULL or tempObjects == NULL or tempTeamInfo == NULL or tempGameInfo == NULL) break;
        AddFrame(tempSensor, tempObjects, tempTeamInfo, tempGameInfo);
        if(m_log_reader->nextFrameAvailable()) m_log_reader->nextFrame();
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
    for(std::vector<LocalisationPerformanceMeasure>::iterator it = m_performance.begin(); it != m_performance.end(); ++it)
    {
        total_time += it->processingTime();
    }

    Model test;
    std::cout << "Number of models created: " << test.id() - initial_model_id << std::endl;
    std::cout << "Total Processing time: " << total_time * 1000 << " ms" <<std::endl;
    std::cout << "Experiment time: " << exp_time * 1000 << " ms" << std::endl;


    m_log_reader->setFrame(save_frame);
    m_log_reader->blockSignals(save_signal);

    emit SimDataChanged(m_sim_data_available);
    return;
}
#include <QDebug>
/*! @brief Run the localisation algorithm on the provided data and add a new localisation frame.
    @param sensorData The sensor data for the new frame.
    @param objectData The observed objects for the current frame.
 */
void OfflineLocalisation::AddFrame(const NUSensorsData* sensorData, FieldObjects* objectData, const TeamInformation* teamInfo, const GameInformation* gameInfo)
{
    // Need to make copies, since source is const
    NUSensorsData tempSensors = (*sensorData);
    NUSensorsData tempSensors2 = (*sensorData);
    //FieldObjects tempObj = (*objectData);
    QElapsedTimer timer;
    long int ms_elapsed;

    m_workingLoc->process(&tempSensors,objectData,gameInfo,teamInfo);

    timer.start();
    m_workingSelfLoc->process(&tempSensors2,objectData,gameInfo,teamInfo);
    ms_elapsed = timer.elapsed();
    Localisation* temp = new Localisation((*m_workingLoc));
    SelfLocalisation* self_temp = new SelfLocalisation(*m_workingSelfLoc);

    LocalisationPerformanceMeasure performance_measure;
    performance_measure.setProcessingTime(ms_elapsed / 1000.0f);
    m_performance.push_back(performance_measure);
    m_localisation_frame_buffer.push_back(temp);
    m_self_loc_frame_buffer.push_back(self_temp);
    QString info(m_workingLoc->frameLog().c_str());
    m_frame_info.push_back(info);
    QString self_info(m_workingSelfLoc->frameLog().c_str());
    m_self_frame_info.push_back(self_info);
}

int OfflineLocalisation::NumberOfLogFrames()
{
    return m_log_reader->numFrames();
}

int OfflineLocalisation::NumberOfFrames()
{
    return m_localisation_frame_buffer.size();
}

const Localisation* OfflineLocalisation::GetFrame(int frameNumber)
{
    int index = frameNumber-1;
    if( (index < 0) || (index >= NumberOfFrames()))
        return NULL;
    else
        return m_localisation_frame_buffer[index];
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


QString OfflineLocalisation::GetFrameInfo(int frameNumber)
{
    int index = frameNumber-1;
    if( (index < 0) || (index >= NumberOfFrames()))
        return NULL;
    else
        return m_frame_info[index];
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
bool OfflineLocalisation::WriteLog(const std::string& logPath)
{
    bool file_saved = false;
    if(hasSimData())
    {
        std::ofstream output_file(logPath.c_str());
        if(output_file.is_open() && output_file.good())
        {
            std::vector<Localisation*>::iterator it = m_localisation_frame_buffer.begin();
            while(it != m_localisation_frame_buffer.end())
            {
                output_file << (*(*it));
                ++it;
            }
        }
        output_file.close();
    }
    return file_saved;
}

