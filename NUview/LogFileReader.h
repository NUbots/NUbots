#ifndef LOGFILEREADER_H
#define LOGFILEREADER_H

#include <QObject>
#include <QString>
#include <fstream>
#include "Tools/FileFormats/NUbotImage.h"
#include "Tools/Image/NUimage.h"

class LogFileReader : public QObject
{
Q_OBJECT
public:
    explicit LogFileReader(QObject *parent = 0);
    ~LogFileReader();
    int openFile(QString fileName);
    bool closeFile();

    // Fetching information
    const QString& fileName() const {return currentFileName;};
    const QString& fileType() const {return currentFileExtension;};
    int numFrames() const {return totalFrames;};
    int currentFrame() const {return currentFrameIndex;};

signals:
    // Signals to trigger available controls
    void nextFrameAvailable(bool);
    void previousFrameAvailable(bool);
    void firstFrameAvailable(bool);
    void lastFrameAvailable(bool);
    void setFrameAvailable(bool);

    void newRawImageAvailable(NUimage*);
    void newSensorDataAvailable(float*,float*,float*);
    void frameLoadingCompleted(int,int);

    void fileOpened(QString);
    void fileClosed();

public slots:
    int nextFrame();
    int previousFrame();
    int firstFrame();
    int lastFrame();
    int setFrame(int frameNumber);

protected:
    // File Information
    QString currentFileName;
    QString currentFileExtension;
    bool fileOpen;
    int fileLengthInBytes;
    int totalFrames;
    int currentFrameIndex;

    // File access
    ifstream currentFileStream;
    NUbotImage currentNifFile;

    // Data Buffers
    NUimage rawImageBuffer;

    //!TODO: sensor data must be updated
    float jointSensorsBuffer[100];
    float balanceSensorsBuffer[100];
    float touchSensorsBuffer[100];
private:
    static QString getFileTypeFromName(const QString& fileName);
    static int calculateFramesInStream(ifstream& stream);
    static QString getErrorFlagsFromStream(ifstream& stream);
    void emitControlAvailability();
};

#endif // LOGFILEREADER_H
