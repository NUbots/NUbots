#include "OfflineLocalisation.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include <sstream>

/*! @brief Default Constructor
 */
OfflineLocalisation::OfflineLocalisation(QObject *parent): QThread(parent)
{
    m_intialLoc = new Localisation();
    Initialise(m_intialLoc);
    m_stop_called = false;
    m_sim_data_available = false;
}

/*! @brief Constructor with initialisation.
 */
OfflineLocalisation::OfflineLocalisation(const Localisation& initialState, const std::string& initialLogPath)
{
    m_intialLoc = new Localisation();
    Initialise(&initialState);
    OpenLogs(initialLogPath);
    m_stop_called = false;
    m_sim_data_available = false;
    return;
}

/*! @brief Class destructor
 */
OfflineLocalisation::~OfflineLocalisation()
{
    if(m_workingLoc) delete m_workingLoc;
    for(int i = 0; i < m_localisation_frame_buffer.size(); i++)
    {
        delete m_localisation_frame_buffer[i];
    }
    m_localisation_frame_buffer.clear();
}


/*! @brief  Removes any existing data from the buffer, and sets its size to zero.
 */
void OfflineLocalisation::ClearBuffer()
{
    for(int i = 0; i < m_localisation_frame_buffer.size(); i++)
    {
        delete m_localisation_frame_buffer[i];
    }
    m_localisation_frame_buffer.clear();
    m_sim_data_available = false;
}

/*! @brief  Clears any previous information, and intialises the system with the
            initial state.
    @param initialState The initial state of the system.
 */
void OfflineLocalisation::Initialise(const Localisation* intialState)
{
    delete m_workingLoc;
    m_workingLoc = new Localisation(*intialState);
    ClearBuffer();
    m_localisation_frame_buffer.push_back(new Localisation(*intialState));
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
    m_log_reader.openFile(QString::fromStdString(initialLogPath));

    // Check if it worked ok.
    bool success = (m_log_reader.numFrames() > 0);
    return success;
}

/*! @brief  Determine if the system has been properly initialised. If this condition is not met,
            the system beaviour may be undefined if run.
    @return True if the system been Inititalised sucessfully. False if it has not.
 */
bool OfflineLocalisation::IsInitialised()
{
    bool filesOpenOk = (m_log_reader.numFrames() > 0);
    bool internalDataOk = (m_workingLoc != NULL) && (m_localisation_frame_buffer.size() > 0);

    return (filesOpenOk && internalDataOk);
}

/*! @brief Run the simulation using the available data.
    @return True if simulation sucessful. False if unsuccessful.
 */
bool OfflineLocalisation::Run()
{
    // Don't do anything if not initialised properly.
    Initialise(m_intialLoc);
    if(IsInitialised() == false) return false;
    const NUSensorsData* tempSensor;
    const FieldObjects* tempObjects;
    const TeamInformation* tempTeamInfo;
    const GameInformation* tempGameInfo;

    int framesProcessed = 0;
    int totalFrames = m_log_reader.numFrames();
    m_log_reader.firstFrame();
    emit updateProgress(framesProcessed+1,totalFrames);
    while (m_log_reader.currentFrame() < totalFrames)
    {
        tempSensor = m_log_reader.GetSensorData();
        tempObjects = m_log_reader.GetObjectData();
        tempTeamInfo = m_log_reader.GetTeamInfo();
        tempGameInfo = m_log_reader.GetGameInfo();
        AddFrame(tempSensor, tempObjects, tempTeamInfo, tempGameInfo);
        if(m_log_reader.nextFrameAvailable()) m_log_reader.nextFrame();
        else break;
        ++framesProcessed;
        emit updateProgress(framesProcessed+1,totalFrames);
    }
    return framesProcessed > 0;
}

void OfflineLocalisation::run()
{
    Initialise(m_intialLoc);
    // Don't do anything if not initialised properly.
    if(IsInitialised() == false) return;
    m_stop_called = false;
    m_sim_data_available = false;
    const NUSensorsData* tempSensor;
    const FieldObjects* tempObjects;
    const TeamInformation* tempTeamInfo;
    const GameInformation* tempGameInfo;

    int framesProcessed = 0;
    int totalFrames = m_log_reader.numFrames();
    m_log_reader.firstFrame();
    emit updateProgress(framesProcessed+1,totalFrames);
    while (m_log_reader.currentFrame() < totalFrames)
    {
        tempSensor = m_log_reader.GetSensorData();
        tempObjects = m_log_reader.GetObjectData();
        tempTeamInfo = m_log_reader.GetTeamInfo();
        tempGameInfo = m_log_reader.GetGameInfo();
        AddFrame(tempSensor, tempObjects, tempTeamInfo, tempGameInfo);
        if(m_log_reader.nextFrameAvailable()) m_log_reader.nextFrame();
        else break;
        ++framesProcessed;
        emit updateProgress(framesProcessed+1,totalFrames);
        if(m_stop_called) break;
    }
    if(!m_stop_called) m_sim_data_available = true;
    return;
}

/*! @brief Run the localisation algorithm on the provided data and add a new localisation frame.
    @param sensorData The sensor data for the new frame.
    @param objectData The observed objects for the current frame.
 */
void OfflineLocalisation::AddFrame(const NUSensorsData* sensorData, const FieldObjects* objectData, const TeamInformation* teamInfo, const GameInformation* gameInfo)
{
    // Need to make copies, since source is const
    NUSensorsData tempSensors = (*sensorData);

    FieldObjects tempObj = (*objectData);

    m_workingLoc->process(&tempSensors,&tempObj,gameInfo,teamInfo);

    Localisation* temp = new Localisation((*m_workingLoc));
    m_localisation_frame_buffer.push_back(temp);
}

int OfflineLocalisation::NumberOfLogFrames()
{
    return m_log_reader.numFrames();
}

int OfflineLocalisation::NumberOfFrames()
{
    return m_localisation_frame_buffer.size();
}

const Localisation* OfflineLocalisation::GetFrame(int frameNumber)
{
    if( (frameNumber < 0) || frameNumber > NumberOfFrames())
        return NULL;
    else
        return m_localisation_frame_buffer[frameNumber];
}

/*! @brief Writes a text based summary of the current experiment to file.
    @param logPath Path to which the log will be written
 */
bool OfflineLocalisation::WriteLog(const std::string& logPath)
{
    return false;
}

