#ifndef LOGFILEFORMATREADER_H
#define LOGFILEFORMATREADER_H

#include <QObject>
#include <QString>
#include <QFileInfo>
#include "Tools/Image/NUimage.h"

class LogFileFormatReader : public QObject
{
Q_OBJECT
public:
    explicit LogFileFormatReader(QObject *parent = 0);
    ~LogFileFormatReader();
    virtual int openFile(const QString& filename) = 0;  // Must be implemented by derived classes.
    virtual bool closeFile() = 0;                       // Must be implemented by derived classes.

    // Fetching information
    QString fileName() const {return fileInformation.fileName();};
    QString filePath() const {return fileInformation.absoluteFilePath();};
    QString fileType() const {return fileInformation.suffix();};
    int fileLengthInBytes() const {return fileInformation.size();};
    int numFrames() const {return totalFrames;};
    int currentFrame() const {return currentFrameIndex;};
    virtual bool fileGood() = 0;
    const NUimage& getImage(){return rawImageBuffer;};

    virtual bool isNextFrameAvailable(){return false;};
    virtual bool isPreviousFrameAvailable(){return false;};
    virtual bool isFirstFrameAvailable(){return false;};
    virtual bool isLastFrameAvailable(){return false;};
    virtual bool isSetFrameAvailable(){return false;};

signals:
    void newRawImageAvailable(const NUimage*);
    void newSensorDataAvailable(const float*,const float*,const float*);
    void frameLoadingCompleted(int,int);

public slots:
    virtual int nextFrame();
    virtual int previousFrame();
    virtual int firstFrame();
    virtual int lastFrame();
    virtual int setFrame(int frameNumber);

protected:
    // File Information
    QFileInfo fileInformation;
    int totalFrames;
    int currentFrameIndex;

    // Data Buffers
    NUimage rawImageBuffer;
    //!TODO: sensor data must be updated
    float jointSensorsBuffer[100];
    float balanceSensorsBuffer[100];
    float touchSensorsBuffer[100];

private:
    static void displayControlError(const QString& controlName);
};

#endif // LOGFILEFORMATREADER_H
