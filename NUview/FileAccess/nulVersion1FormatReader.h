#ifndef NULVERSION1FORMATREADER_H
#define NULVERSION1FORMATREADER_H

#include "LogFileFormatReader.h"
#include <fstream>
#include "Infrastructure/NUImage/NUImage.h"

class nulVersion1FormatReader : public LogFileFormatReader
{
Q_OBJECT
public:
    explicit nulVersion1FormatReader(QObject *parent = 0);
    explicit nulVersion1FormatReader(const QString& filename, QObject *parent = 0);
    ~nulVersion1FormatReader();

    int openFile(const QString& filename);
    bool closeFile();
    bool fileGood(){return (fileStream.good() && fileStream.is_open());};
    const NUImage* getImage(){return &rawImageBuffer;};


    bool isNextFrameAvailable();
    bool isFirstFrameAvailable();
    bool isPreviousFrameAvailable();
signals:

public slots:
    int nextFrame();
    int firstFrame();
    int previousFrame();
protected:
    std::ifstream fileStream;
    int m_frameLength;

    // Data Buffers
    NUImage rawImageBuffer;
    //!TODO: sensor data must be updated
    float jointSensorsBuffer[100];
    float balanceSensorsBuffer[100];
    float touchSensorsBuffer[100];
};

#endif // NULVERSION1FORMATREADER_H
