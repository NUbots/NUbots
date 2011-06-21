#ifndef NIFVERSION1FORMATREADER_H
#define NIFVERSION1FORMATREADER_H

#include "LogFileFormatReader.h"
#include "Tools/FileFormats/NUbotImage.h"
#include "Infrastructure/NUImage/NUImage.h"


class nifVersion1FormatReader : public LogFileFormatReader
{
Q_OBJECT
public:
    explicit nifVersion1FormatReader(QObject *parent = 0);
    explicit nifVersion1FormatReader(const QString& filename, QObject *parent = 0);
    ~nifVersion1FormatReader();

    int openFile(const QString& filename);
    bool closeFile();
    bool fileGood(){return (nifFileReader.currentFile.good() && nifFileReader.currentFile.is_open());};

    const NUImage* getImage(){return &rawImageBuffer;};

    bool isNextFrameAvailable();
    bool isPreviousFrameAvailable();
    bool isFirstFrameAvailable();
    bool isLastFrameAvailable();
    bool isSetFrameAvailable();

    std::vector<QFileInfo> AvailableLogFiles()const;
    QStringList AvailableData() const;

signals:

public slots:
    int nextFrame();
    int previousFrame();
    int firstFrame();
    int lastFrame();
    int setFrame(int frameNumber);

protected:
    NUbotImage nifFileReader;

    // Data Buffers
    NUImage rawImageBuffer;
    //!TODO: sensor data must be updated
    float jointSensorsBuffer[100];
    float balanceSensorsBuffer[100];
    float touchSensorsBuffer[100];
};

#endif // NIFVERSION1FORMATREADER_H
