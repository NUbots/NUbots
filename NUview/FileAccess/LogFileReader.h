#ifndef LOGFILEREADER_H
#define LOGFILEREADER_H

#include <QObject>
#include <QString>
#include "LogFileFormatReader.h"
class NUSensorsData;

class LogFileReader : public QObject
{
Q_OBJECT
public:
    explicit LogFileReader(QObject *parent = 0);
    ~LogFileReader();
    int openFile(QString fileName);
    bool closeFile();
    int numFrames(){if(currentFileReader){return currentFileReader->numFrames();}return 0;};
    int currentFrame(){if(currentFileReader){return currentFileReader->currentFrame();}return 0;};

signals:
    // Signals to trigger available controls
    void nextFrameAvailable(bool);
    void previousFrameAvailable(bool);
    void firstFrameAvailable(bool);
    void lastFrameAvailable(bool);
    void setFrameAvailable(bool);

    void rawImageChanged(const NUimage*);
    void sensorDataChanged(const float*, const float*, const float*);
    void sensorDataChanged(NUSensorsData*);
    void frameChanged(int,int);
    void cameraChanged(int);

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
