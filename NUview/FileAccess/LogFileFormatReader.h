#ifndef LOGFILEFORMATREADER_H
#define LOGFILEFORMATREADER_H

#include <QObject>
#include <QString>
#include <QFileInfo>

class Localisation;
class NUSensorsData;
class NUImage;
class LocWmFrame;
class FieldObjects;
class GameInformation;
class TeamInformation;

class LogFileFormatReader : public QObject
{
Q_OBJECT
public:
    explicit LogFileFormatReader(QObject *parent = 0);
    ~LogFileFormatReader();
    virtual int openFile(const QString& filename) = 0;  // Must be implemented by derived classes.
    virtual bool closeFile() = 0;                       // Must be implemented by derived classes.

    // Fetching information
    QString fileName() const {return m_fileInformation.fileName();}
    QString filePath() const {return m_fileInformation.absoluteFilePath();}
    QString fileType() const {return m_fileInformation.suffix();}
    int fileLengthInBytes() const {return m_fileInformation.size();}
    int numFrames() const {return m_totalFrames;}
    int currentFrame() const {return m_currentFrameIndex;}
    virtual bool fileGood() = 0;

    virtual bool isNextFrameAvailable(){return false;}
    virtual bool isPreviousFrameAvailable(){return false;}
    virtual bool isFirstFrameAvailable(){return false;}
    virtual bool isLastFrameAvailable(){return false;}
    virtual bool isSetFrameAvailable(){return false;}

    virtual const Localisation* GetLocalisationData(){return NULL;}
    virtual const NUImage* GetImageData(){return NULL;}
    virtual const NUSensorsData* GetSensorData(){return NULL;}
    virtual const FieldObjects* GetObjectData(){return NULL;}
    virtual const GameInformation* GetGameInfo(){return NULL;}
    virtual const TeamInformation* GetTeamInfo(){return NULL;}

signals:
    void LocalisationDataChanged(const Localisation*);
    void LocalisationFrameChanged(const LocWmFrame*);
    void ObjectDataChanged(const FieldObjects*);
    void rawImageChanged(const NUImage*);
    void sensorDataChanged(const float*,const float*,const float*);
    void sensorDataChanged(NUSensorsData*);
    void GameInfoChanged(const GameInformation*);
    void TeamInfoChanged(const TeamInformation*);
    void frameChanged(int,int);
    void cameraChanged(int);

public slots:
    virtual int nextFrame();
    virtual int previousFrame();
    virtual int firstFrame();
    virtual int lastFrame();
    virtual int setFrame(int frameNumber);

protected:
    // File Information
    QFileInfo m_fileInformation;
    int m_totalFrames;
    int m_currentFrameIndex;

private:
    static void displayControlError(const QString& controlName);
};

#endif // LOGFILEFORMATREADER_H
