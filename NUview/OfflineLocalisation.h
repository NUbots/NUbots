#ifndef OFFLINELOCALISATION_H
#define OFFLINELOCALISATION_H

#include "Localisation/Localisation.h"
#include "FileAccess/StreamFileReader.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include <string>
#include <vector>

class OfflineLocalisation
{
public:
    OfflineLocalisation();
    ~OfflineLocalisation();
    OfflineLocalisation(const Localisation& intialState, const std::string& sensorLogPath, const std::string& objectsLogPath);
    void Initialise(const Localisation& intialState);
    bool OpenLogs(const std::string& sensorLogPath, const std::string& objectsLogPath);
    bool Run();
    bool WriteLog(const std::string& logPath);
    int NumberOfFrames();
    const Localisation* GetFrame(int frameNumber);
    bool IsInitialised();
private:
    void AddFrame(NUSensorsData* sensorData, FieldObjects* objectData);
    void ClearBuffer();
    std::vector<Localisation*> m_localisation_frame_buffer;
    Localisation* m_workingLoc;
    StreamFileReader<NUSensorsData> sensorLog;
    StreamFileReader<FieldObjects> objectLog;
};

#endif // OFFLINELOCALISATION_H
