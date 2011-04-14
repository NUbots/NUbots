#include "OfflineLocalisation.h"

/*! @brief Default Constructor
 */
OfflineLocalisation::OfflineLocalisation()
{
    m_workingLoc = new Localisation();
}

/*! @brief Constructor with initialisation.
 */
OfflineLocalisation::OfflineLocalisation(const Localisation& initialState, const std::string& sensorLogPath, const std::string& objectsLogPath)
{
    m_workingLoc = new Localisation();
    Initialise(initialState);
    OpenLogs(sensorLogPath, objectsLogPath);
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
}

/*! @brief  Clears any previous information, and intialises the system with the
            initial state.
    @param initialState The initial state of the system.
 */
void OfflineLocalisation::Initialise(const Localisation& intialState)
{
    ClearBuffer();
    delete m_workingLoc;
    m_workingLoc = new Localisation(intialState);
    Localisation* temp = new Localisation((*m_workingLoc));
    m_localisation_frame_buffer.push_back(temp);
    return;
}

/*! @brief Open the logs at the given paths.
    @param sensorLogPath The path to the file containing the sensor logs.
    @param objectsLogPath The path to the file containing the Field Object logs.
    @return Returns True if the files were opened successfuly. False if they were not.
 */
bool OfflineLocalisation::OpenLogs(const std::string& sensorLogPath, const std::string& objectsLogPath)
{
    // Open Files
    sensorLog.OpenFile(sensorLogPath);
    objectLog.OpenFile(objectsLogPath);

    // Check if it worked ok.
    bool success = sensorLog.IsValid() && objectLog.IsValid();
    return success;
}

/*! @brief  Determine if the system has been properly initialised. If this condition is not met,
            the system beaviour may be undefined if run.
    @return True if the system been Inititalised sucessfully. False if it has not.
 */
bool OfflineLocalisation::IsInitialised()
{
    bool filesOpenOk = sensorLog.IsValid() && objectLog.IsValid();
    bool internalDataOk = (m_workingLoc != NULL) && (m_localisation_frame_buffer.size() > 0);

    return (filesOpenOk && internalDataOk);
}

/*! @brief Run the simulation using the available data.
    @return True if simulation sucessful. False if unsuccessful.
 */
bool OfflineLocalisation::Run()
{
    // Don't do anything if not initialised properly.
    if(IsInitialised() == false) return false;
    NUSensorsData* tempSensor;
    FieldObjects* tempObjects;
    int framesProcessed = 0;
    while ( (sensorLog.CurrentFrameSequenceNumber() < sensorLog.TotalFrames())
            && (objectLog.CurrentFrameSequenceNumber() < objectLog.TotalFrames())){

        tempSensor = sensorLog.ReadNextFrame();
        tempObjects = objectLog.ReadNextFrame();
        AddFrame(tempSensor, tempObjects);
        ++framesProcessed;
    }
    return framesProcessed > 0;
}

/*! @brief Run the localisation algorithm on the provided data and add a new localisation frame.
    @param sensorData The sensor data for the new frame.
    @param objectData The observed objects for the current frame.
 */
void OfflineLocalisation::AddFrame(NUSensorsData* sensorData, FieldObjects* objectData)
{
    ///! @todo Add default GameInfo and TeamInfo objects.
    int playerNumber = 1;
    int teamNumber = 1;
    GameInformation gameInfo(playerNumber,teamNumber);
    TeamInformation teamInfo(playerNumber,teamNumber);

    m_workingLoc->process(sensorData,objectData,&gameInfo,&teamInfo);

    Localisation* temp = new Localisation((*m_workingLoc));
    m_localisation_frame_buffer.push_back(temp);

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

