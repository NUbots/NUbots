#ifndef LOGFILEREADER_H
#define LOGFILEREADER_H

#include <QObject>
#include <QString>
#include <QStringList>
#include <vector>
#include <QFileInfo>
#include "LogFileFormatReader.h"
class NUSensorsData;
class TeamInformation;
class GameInformation;

class LogFileReader : public QObject
{
Q_OBJECT
public:
    explicit LogFileReader(QObject *parent = 0);
    ~LogFileReader();
    int openFile(QString fileName);
    bool closeFile();
    int numFrames(){if(currentFileReader){return currentFileReader->numFrames();}return 0;}
    int currentFrame(){if(currentFileReader){return currentFileReader->currentFrame();}return 0;}

    const Localisation* GetLocalisationData(){return currentFileReader->GetLocalisationData();}
    const SelfLocalisation* GetSelfLocalisationData(){return currentFileReader->GetSelfLocalisationData();}
    const NUImage* GetRawImage(){return currentFileReader->GetImageData();}
    const NUSensorsData* GetSensorData(){return currentFileReader->GetSensorData();}
    FieldObjects* GetObjectData(){return currentFileReader->GetObjectData();}
    const GameInformation* GetGameInfo(){return currentFileReader->GetGameInfo();}
    const TeamInformation* GetTeamInfo(){return currentFileReader->GetTeamInfo();}

    bool nextFrameAvailable()
    {
        return currentFrame() < numFrames();
    }

    bool previousFrameAvailable()
    {
        return currentFrame() > 0;
    }

    std::vector<QFileInfo> AvailableLogFiles()
    {
        if(currentFileReader)
            return currentFileReader->AvailableLogFiles();
        else
            return std::vector<QFileInfo>();
    }
    QStringList AvailableData()
    {
        if(currentFileReader)
            return currentFileReader->AvailableData();
        else
            return QStringList("None");
    }
    QString path()
    {
        if(currentFileReader)
            return currentFileReader->filePath();
        else
            return QString("None");
    }


signals:
    // Signals to trigger available controls
    void nextFrameAvailable(bool);
    void previousFrameAvailable(bool);
    void firstFrameAvailable(bool);
    void lastFrameAvailable(bool);
    void setFrameAvailable(bool);
    void LocalisationDataChanged(const Localisation*);
    void SelfLocalisationDataChanged(const SelfLocalisation*);
    void rawImageChanged(const NUImage*);
    void sensorDataChanged(const float*, const float*, const float*);
    void sensorDataChanged(NUSensorsData*);
    void ObjectDataChanged(const FieldObjects*);
    void TeamInfoChanged(const TeamInformation*);
    void GameInfoChanged(const GameInformation*);
    void frameChanged(int,int);
    void cameraChanged(int);
    void AvailableDataChanged(QStringList);
    void OpenLogFilesChanged(std::vector<QFileInfo>);


    void fileOpened(QString);
    void fileClosed();

public slots:
    int nextFrame();
    int previousFrame();
    int firstFrame();
    int lastFrame();
    int setFrame(int frameNumber);

protected:
    LogFileFormatReader* currentFileReader;

private:
    void emitControlAvailability();
};

#endif // LOGFILEREADER_H
