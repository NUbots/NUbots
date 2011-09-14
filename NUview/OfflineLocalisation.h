#ifndef OFFLINELOCALISATION_H
#define OFFLINELOCALISATION_H

#include <string>
#include <vector>
#include <QThread>
#include <QString>

// Class forward declerations.
class LogFileReader;
class FieldObjects;
class NUSensorsData;
class TeamInformation;
class GameInformation;
class Localisation;
class SelfLocalisation;

class OfflineLocalisation : public QThread
{
Q_OBJECT
public:
    explicit OfflineLocalisation(LogFileReader* reader, QObject *parent = 0);
    ~OfflineLocalisation();
//    OfflineLocalisation(const Localisation& intialState, const std::string& initialLogPath);
    void Initialise(const Localisation& intialState);
    bool OpenLogs(const std::string& intialLogPath);
    bool Run();
    bool WriteLog(const std::string& logPath);
    int NumberOfLogFrames();
    int NumberOfFrames();
    const Localisation* GetFrame(int frameNumber);
    const SelfLocalisation* GetSelfFrame(int frameNumber);
    QString GetFrameInfo(int frameNumber);
    QString GetSelfFrameInfo(int frameNumber);
    bool IsInitialised();
    void run();
    void stop(){m_stop_called = true;}
    bool wasStopped(){return m_stop_called;}
    bool hasSimData(){return m_sim_data_available;}
    bool HasRequiredData(QStringList& availableData);
private:
    void AddFrame(const NUSensorsData* sensorData, FieldObjects* objectData, const TeamInformation* teamInfo=NULL, const GameInformation* gameInfo=NULL);
    void ClearBuffer();
    std::vector<Localisation*> m_localisation_frame_buffer;
    std::vector<SelfLocalisation*> m_self_loc_frame_buffer;
    std::vector<QString> m_frame_info;
    std::vector<QString> m_self_frame_info;
    Localisation* m_workingLoc;
    SelfLocalisation* m_workingSelfLoc;
    LogFileReader* m_log_reader;
    bool m_stop_called;
    bool m_sim_data_available;
signals:
    void SimDataChanged(bool);
    void updateProgress(int,int);
};

#endif // OFFLINELOCALISATION_H
