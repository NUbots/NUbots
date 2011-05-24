#ifndef OFFLINELOCALISATION_H
#define OFFLINELOCALISATION_H

#include "Localisation/Localisation.h"
#include "FileAccess/LogFileReader.h"
#include <string>
#include <vector>

class FieldObjects;
class NUSensorsData;
class TeamInformation;
class GameInformation;

class OfflineLocalisation : public QObject
{
Q_OBJECT
public:
    explicit OfflineLocalisation();
    ~OfflineLocalisation();
    OfflineLocalisation(const Localisation& intialState, const std::string& initialLogPath);
    void Initialise(const Localisation& intialState);
    bool OpenLogs(const std::string& intialLogPath);
    bool Run();
    bool WriteLog(const std::string& logPath);
    int NumberOfFrames();
    const Localisation* GetFrame(int frameNumber);
    bool IsInitialised();
private:
    void AddFrame(const NUSensorsData* sensorData, const FieldObjects* objectData, const TeamInformation* teamInfo=NULL, const GameInformation* gameInfo=NULL);
    void ClearBuffer();
    std::vector<Localisation*> m_localisation_frame_buffer;
    Localisation* m_workingLoc;
    LogFileReader m_log_reader;
};

#endif // OFFLINELOCALISATION_H
